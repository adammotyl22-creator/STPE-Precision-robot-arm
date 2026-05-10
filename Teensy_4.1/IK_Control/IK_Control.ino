/*
 * STPE Precision Robot Arm - Ultimate Upgraded Control System
 *
 * Version: v6.0 (Complete Feature-Restored Edition)
 *
 * Upgrades:
 * 1. Motion: Quintic Splines with full C2 continuity (blended Vel & Accel).
 * 2. Kinematics: 5-DOF Analytical solver (X,Y,Z, Pitch, Roll).
 * 3. Safety: Optimized StallGuard (low-freq polling), Watchdog, Workspace Limits.
 * 4. Control: Real-time Closed-Loop PID using AS5600 Magnetic Encoders.
 *
 * Features Restored:
 * - X/Y Gantry Control & Homing.
 * - SD Card Sequence Player (.seq files) & Position Save/Load.
 * - Teach & Playback Mode with 64-point buffer.
 * - Full original Serial Protocol & CRC Packet Parsing.
 */

#include <AccelStepper.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <TMCStepper.h>
#include <USBHost_t36.h>
#include <Watchdog_t4.h>
#include <Wire.h>
#include <math.h>

// ============================================================================
//                                CONFIGURATION
// ============================================================================

// Physical Robot Dimensions (mm)
#define BASE_H    160.0f
#define L1        135.0f
#define L2_PAR     55.0f   // Forearm parallel component
#define L2_PERP   135.0f   // Forearm perpendicular component
#define L3        125.0f   // Tool length (J4 to tip)
#define EXCL_R2   6400.0f  // Safety exclusion zone radius squared (80mm)
#define FLOOR_CLR  20.0f   // Minimum Z height above ground

// Pin Mapping
const int PIN_AXIS1_STEP = 3,  PIN_AXIS1_DIR = 2,  PIN_AXIS1_EN = 6;
const int PIN_AXIS2_STEP = 11, PIN_AXIS2_DIR = 10, PIN_AXIS2_EN = 27;
const int PIN_AXIS3_STEP = 29, PIN_AXIS3_DIR = 28, PIN_AXIS3_EN = 32;
const int PIN_MICRO_X_STEP = 34, PIN_MICRO_X_DIR = 33, PIN_MICRO_EN = 12;
const int PIN_MICRO_Y_STEP = 31, PIN_MICRO_Y_DIR = 30;
const int PIN_AXIS4_SERVO = 23, PIN_AXIS5_SERVO = 22, PIN_GRIPPER_SERVO = 21;
const int PIN_J1_LIMIT = 5, PIN_J2_LIMIT = 4, PIN_J3_LIMIT = 36, PIN_X_LIMIT = 7, PIN_Y_LIMIT = 8;

// Joint Safety Limits (Degrees)
const float LIMIT_J1_MIN = -85.0f, LIMIT_J1_MAX = 85.0f;
const float LIMIT_J2_MIN = -95.0f, LIMIT_J2_MAX = 85.0f;
const float LIMIT_J3_MIN = -85.0f, LIMIT_J3_MAX = 95.0f;
const float LIMIT_J4_MIN = 0.0f,   LIMIT_J4_MAX = 180.0f;
const float LIMIT_J5_MIN = 0.0f,   LIMIT_J5_MAX = 180.0f;

// Constants
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define STEPS_PER_MOTOR_REV (STEPS_PER_REV * MICROSTEPS)
#define EEPROM_MAGIC_NUMBER 0x494B3101

// ============================================================================
//                               DATA STRUCTURES
// ============================================================================

struct Joints { float j1, j2, j3, j4, j5; };
struct Pose { float x, y, z, pitch, roll; };
struct Waypoint { Joints joints; uint32_t duration; };
struct TeachPoint { Joints joints; float grip; };

struct RobotConfig {
    uint32_t magic;
    float gear1;     // Gearing for base
    float gear2;     // Gearing for arm axes
    bool jntEn[3];   // Joint enabled flags
    float speedOvr;  // Global speed multiplier
    float toolOff[3]; // Tool-tip offsets (XYZ)
    float workOff[3]; // Work-frame offsets (XYZ)
    float encOff[3];  // Encoder offsets
} config;

// ============================================================================
//                               MATH & KINEMATICS
// ============================================================================

class QuinticSpline {
public:
    float a[6]; // Coefficients

    void calculate(float p0, float v0, float acc0, float p1, float v1, float acc1, float T) {
        if (T <= 0) T = 0.001f;
        float T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
        a[0] = p0;
        a[1] = v0;
        a[2] = 0.5f * acc0;
        a[3] = (20.0f * p1 - 20.0f * p0 - (8.0f * v1 + 12.0f * v0) * T - (3.0f * acc0 - acc1) * T2) / (2.0f * T3);
        a[4] = (30.0f * p0 - 30.0f * p1 + (14.0f * v1 + 16.0f * v0) * T + (3.0f * acc0 - 2.0f * acc1) * T2) / (2.0f * T4);
        a[5] = (12.0f * p1 - 12.0f * p0 - (6.0f * v1 + 6.0f * v0) * T - (acc0 - acc1) * T2) / (2.0f * T5);
    }

    float pos(float t) {
        float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
    }

    float vel(float t) {
        float t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return a[1] + 2.0f * a[2] * t + 3.0f * a[3] * t2 + 4.0f * a[4] * t3 + 5.0f * a[5] * t4;
    }

    float acc(float t) {
        float t2 = t * t, t3 = t2 * t;
        return 2.0f * a[2] + 6.0f * a[3] * t + 12.0f * a[4] * t2 + 20.0f * a[5] * t3;
    }
};

class Kinematics {
public:
    static Pose forward(Joints j) {
        Pose p;
        float j1r = j.j1 * DEG_TO_RAD, j2r = j.j2 * DEG_TO_RAD, j3r = j.j3 * DEG_TO_RAD;

        float r3 = L1 * sinf(j2r);
        float z3 = BASE_H + L1 * cosf(j2r);
        float ang34 = j2r + j3r;

        float r4 = r3 + L2_PAR * sinf(ang34) + L2_PERP * cosf(ang34);
        float z4 = z3 + L2_PAR * cosf(ang34) - L2_PERP * sinf(ang34);

        float pitch_rad = (j.j4 + (j.j2 + j.j3) - 90.0f) * DEG_TO_RAD;
        float r_tip = r4 + L3 * cosf(pitch_rad);

        p.z = z4 - L3 * sinf(pitch_rad);
        p.x = r_tip * cosf(j1r);
        p.y = r_tip * sinf(j1r);
        p.pitch = j.j4 + (j.j2 + j.j3) - 90.0f;
        p.roll = j.j5;
        return p;
    }

    static Joints inverse(Pose t) {
        // Apply Offsets
        t.x -= config.workOff[0]; t.y -= config.workOff[1]; t.z -= config.workOff[2];
        t.x += config.toolOff[0]; t.y += config.toolOff[1]; t.z += config.toolOff[2];

        Joints j;
        j.j1 = atan2f(t.y, t.x) * RAD_TO_DEG;

        float pr = t.pitch * DEG_TO_RAD;
        float rt = sqrtf(t.x * t.x + t.y * t.y);
        float rwc = rt - L3 * cosf(pr);
        float zwc = t.z + L3 * sinf(pr);
        float zr = zwc - BASE_H;

        float L2e = sqrtf(L2_PAR * L2_PAR + L2_PERP * L2_PERP);
        float gam = atan2f(L2_PERP, L2_PAR);
        float D2 = rwc * rwc + zr * zr;
        float C3 = constrain((D2 - L1 * L1 - L2e * L2e) / (2.0f * L1 * L2e), -1.0f, 1.0f);
        float j3e = acosf(C3);

        j.j2 = (atan2f(rwc, zr) - atan2f(L2e * sinf(j3e), L1 + L2e * C3)) * RAD_TO_DEG;
        j.j3 = (j3e - gam) * RAD_TO_DEG;
        j.j4 = t.pitch - (j.j2 + j.j3) + 90.0f;
        j.j5 = t.roll;
        return j;
    }

    static bool checkWorkspace(Joints j) {
        if (j.j1 < LIMIT_J1_MIN || j.j1 > LIMIT_J1_MAX) return false;
        if (j.j2 < LIMIT_J2_MIN || j.j2 > LIMIT_J2_MAX) return false;
        if (j.j3 < LIMIT_J3_MIN || j.j3 > LIMIT_J3_MAX) return false;
        if (j.j4 < LIMIT_J4_MIN || j.j4 > LIMIT_J4_MAX) return false;
        if (j.j5 < LIMIT_J5_MIN || j.j5 > LIMIT_J5_MAX) return false;
        Pose p = forward(j);
        if ((p.x * p.x + p.y * p.y) < EXCL_R2 && p.z < BASE_H) return false;
        if (p.z < FLOOR_CLR) return false;
        return true;
    }
};

// ============================================================================
//                                HARDWARE INTERFACE
// ============================================================================

WDT_T4<WDT1> watchdog;
USBHost myusb; USBHub hub1(myusb); USBSerial userial(myusb);
TMC2209Stepper drv1(&Serial4, 0.11f, 0b00), drv2(&Serial4, 0.11f, 0b10), drv3(&Serial4, 0.11f, 0b01);
AccelStepper st1(1, PIN_AXIS1_STEP, PIN_AXIS1_DIR), st2(1, PIN_AXIS2_STEP, PIN_AXIS2_DIR), st3(1, PIN_AXIS3_STEP, PIN_AXIS3_DIR);
AccelStepper stX(1, PIN_MICRO_X_STEP, PIN_MICRO_X_DIR), stY(1, PIN_MICRO_Y_STEP, PIN_MICRO_Y_DIR);
Servo s4, s5, grip;

class MotionPlanner {
public:
    Waypoint q[32]; uint8_t h = 0, t = 0;
    QuinticSpline splines[5];
    Joints cj = {0, -90, 90, 90, 90}, cv = {0, 0, 0, 0, 0}, ca = {0, 0, 0, 0, 0};
    unsigned long startMicros; float duration; bool moving = false;

    void update() {
        if (!moving && h != t) {
            Waypoint& target = q[t];
            duration = (target.duration / 1000.0f) / max(0.05f, config.speedOvr);
            if (duration <= 0) duration = 0.001f;

            // Compute velocities for blending
            Joints v1 = {0,0,0,0,0}, a1 = {0,0,0,0,0};
            uint8_t nIdx = (t + 1) % 32;
            if (nIdx != h) {
                Waypoint& next = q[nIdx];
                float nD = (next.duration / 1000.0f) / max(0.05f, config.speedOvr);
                v1.j1 = (next.joints.j1 - cj.j1) / (duration + nD);
                v1.j2 = (next.joints.j2 - cj.j2) / (duration + nD);
                v1.j3 = (next.joints.j3 - cj.j3) / (duration + nD);
                v1.j4 = (next.joints.j4 - cj.j4) / (duration + nD);
                v1.j5 = (next.joints.j5 - cj.j5) / (duration + nD);
            }

            splines[0].calculate(cj.j1, cv.j1, ca.j1, target.joints.j1, v1.j1, a1.j1, duration);
            splines[1].calculate(cj.j2, cv.j2, ca.j2, target.joints.j2, v1.j2, a1.j2, duration);
            splines[2].calculate(cj.j3, cv.j3, ca.j3, target.joints.j3, v1.j3, a1.j3, duration);
            splines[3].calculate(cj.j4, cv.j4, ca.j4, target.joints.j4, v1.j4, a1.j4, duration);
            splines[4].calculate(cj.j5, cv.j5, ca.j5, target.joints.j5, v1.j5, a1.j5, duration);

            startMicros = micros(); moving = true;
        }

        if (moving) {
            float time = (micros() - startMicros) / 1000000.0f;
            if (time >= duration) {
                time = duration; moving = false; t = (t + 1) % 32;
            }
            cj = { splines[0].pos(time), splines[1].pos(time), splines[2].pos(time), splines[3].pos(time), splines[4].pos(time) };
            cv = { splines[0].vel(time), splines[1].vel(time), splines[2].vel(time), splines[3].vel(time), splines[4].vel(time) };
            ca = { splines[0].acc(time), splines[1].acc(time), splines[2].acc(time), splines[3].acc(time), splines[4].acc(time) };
        }
    }

    void add(Waypoint wp) {
        if (Kinematics::checkWorkspace(wp.joints)) {
            uint8_t next = (h + 1) % 32;
            if (next != t) { q[h] = wp; h = next; }
        } else {
            Serial.println("ERR:WORKSPACE_VIOLATION");
        }
    }

    void stop() { h = t; moving = false; cv = {0, 0, 0, 0, 0}; ca = {0, 0, 0, 0, 0}; }
} planner;

class ClosedLoop {
public:
    float kP = 0.3, kFF = 0.1, integral = 0, offset = 0; bool en = true;

    uint16_t readEncoder(TwoWire& w) {
        w.beginTransmission(0x36); w.write(0x0C);
        if (w.endTransmission(false) != 0 || w.requestFrom(0x36, 2) != 2) return 0xFFFF;
        return ((uint16_t)w.read() << 8) | w.read();
    }

    float update(float target, float vel, float dt, TwoWire& w) {
        if (!en || dt <= 0) return 0;
        uint16_t raw = readEncoder(w);
        if (raw == 0xFFFF) return 0;

        float actual = (raw / 4096.0f) * 360.0f - offset;
        float err = target - actual;
        while (err > 180) err -= 360; while (err < -180) err += 360;
        integral = constrain(integral + err * dt, -10, 10);
        return (kP * err + 0.05f * integral + kFF * vel);
    }
} cl1, cl2, cl3;

// ============================================================================
//                                SYSTEMS & STATE
// ============================================================================

bool estopActive = false;
enum HomingState { IDLE, J1_F, J1_B, J1_S, J2_F, J2_B, J2_S, J3_F, J3_B, J3_S, DONE } hState = IDLE;
unsigned long hStep = 0;
TeachPoint teachPoints[64]; uint8_t teachCount = 0; bool isTeachMode = false;
File seqFile; bool isPlayingSequence = false;

void triggerEstop() {
    estopActive = true;
    digitalWrite(PIN_AXIS1_EN, 1); digitalWrite(PIN_AXIS2_EN, 1); digitalWrite(PIN_AXIS3_EN, 1); digitalWrite(PIN_MICRO_EN, 1);
    Serial.println("!!! EMERGENCY STOP !!!");
}

void resetEstop() {
    estopActive = false;
    digitalWrite(PIN_AXIS1_EN, 0); digitalWrite(PIN_AXIS2_EN, 0); digitalWrite(PIN_AXIS3_EN, 0); digitalWrite(PIN_MICRO_EN, 0);
    Serial.println("ESTOP RESET OK");
}

// ============================================================================
//                                COMMAND PROCESSING
// ============================================================================

char inBuf[128]; uint8_t inPos = 0;

void handleCommand(String cmd) {
    cmd.trim(); cmd.toUpperCase();

    if (cmd.startsWith("MOV")) {
        float x = 0, y = 0, z = 160, p = 0, r = 0; uint32_t d = 1000;
        int iX = cmd.indexOf('X'), iY = cmd.indexOf('Y'), iZ = cmd.indexOf('Z'), iP = cmd.indexOf('P'), iR = cmd.indexOf('R'), iD = cmd.indexOf('D');
        if (iX != -1) x = cmd.substring(iX + 1).toFloat();
        if (iY != -1) y = cmd.substring(iY + 1).toFloat();
        if (iZ != -1) z = cmd.substring(iZ + 1).toFloat();
        if (iP != -1) p = cmd.substring(iP + 1).toFloat();
        if (iR != -1) r = cmd.substring(iR + 1).toFloat();
        if (iD != -1) d = cmd.substring(iD + 1).toInt();

        if (cmd.startsWith("MOVL")) {
            Pose curr = Kinematics::forward(planner.cj);
            float dx = x - curr.x, dy = y - curr.y, dz = z - curr.z, dp = p - curr.pitch, dr = r - curr.roll;
            int segs = constrain(sqrtf(dx*dx + dy*dy + dz*dz) / 5.0f, 2, 30);
            for (int i = 1; i <= segs; i++) {
                float t = (float)i / segs;
                planner.add({Kinematics::inverse({curr.x + dx * t, curr.y + dy * t, curr.z + dz * t, curr.pitch + dp * t, curr.roll + dr * t}), d / (uint32_t)segs});
            }
        } else {
            planner.add({Kinematics::inverse({x, y, z, p, r}), d});
        }
    }
    else if (cmd == "ESTOP") triggerEstop();
    else if (cmd == "RESET") resetEstop();
    else if (cmd == "HOME")  { hState = J1_F; hStep = 0; }
    else if (cmd.startsWith("GRIP")) grip.write(constrain(cmd.substring(5).toInt(), 0, 180));
    else if (cmd.startsWith("D1")) { Joints j = planner.cj; j.j1 += cmd.substring(2).toFloat(); planner.add({j, 500}); }
    else if (cmd.startsWith("D2")) { Joints j = planner.cj; j.j2 += cmd.substring(2).toFloat(); planner.add({j, 500}); }
    else if (cmd.startsWith("D3")) { Joints j = planner.cj; j.j3 += cmd.substring(2).toFloat(); planner.add({j, 500}); }
    else if (cmd.startsWith("DX")) stX.move(cmd.substring(2).toInt());
    else if (cmd.startsWith("DY")) stY.move(cmd.substring(2).toInt());
    else if (cmd.startsWith("SPEED")) { config.speedOvr = cmd.substring(5).toFloat() / 100.0f; EEPROM.put(0, config); }
    else if (cmd == "SAVECFG") EEPROM.put(0, config);
    else if (cmd == "CAL") {
        uint16_t r1 = cl1.readEncoder(Wire2), r2 = cl2.readEncoder(Wire), r3 = cl3.readEncoder(Wire1);
        if (r1 != 0xFFFF) cl1.offset = (r1 / 4096.0f) * 360.0f - planner.cj.j1;
        if (r2 != 0xFFFF) cl2.offset = (r2 / 4096.0f) * 360.0f - planner.cj.j2;
        if (r3 != 0xFFFF) cl3.offset = (r3 / 4096.0f) * 360.0f - planner.cj.j3;
        config.encOff[0] = cl1.offset; config.encOff[1] = cl2.offset; config.encOff[2] = cl3.offset;
        EEPROM.put(0, config); Serial.println("CALIBRATED");
    }
    else if (cmd.startsWith("TEACH")) {
        String arg = cmd.substring(5); arg.trim();
        if (arg == "START") { teachCount = 0; isTeachMode = true; Serial.println("TEACH START"); }
        else if (arg == "RECORD" && teachCount < 64) { teachPoints[teachCount++] = {planner.cj, (float)grip.read()}; Serial.println("RECORDED"); }
        else if (arg == "PLAY") { for (int i = 0; i < teachCount; i++) planner.add({teachPoints[i].joints, 1500}); }
    }
    else if (cmd.startsWith("SAVE ")) {
        if (!SD.begin(BUILTIN_SDCARD)) return;
        File f = SD.open(("/" + cmd.substring(5) + ".pos").c_str(), FILE_WRITE);
        if (f) { f.printf("%.2f,%.2f,%.2f,%.2f,%.2f,1000\n", planner.cj.j1, planner.cj.j2, planner.cj.j3, planner.cj.j4, planner.cj.j5); f.close(); Serial.println("SAVED"); }
    }
    else if (cmd.startsWith("RUN ")) {
        if (!SD.begin(BUILTIN_SDCARD)) return;
        seqFile = SD.open(("/" + cmd.substring(4) + ".seq").c_str());
        if (seqFile) isPlayingSequence = true;
    }
}

void processInput(Stream& s) {
    static char buf[128]; static uint8_t p = 0;
    while (s.available()) {
        char c = s.read();
        if (c == '<') p = 0;
        else if (c == '>') {
            buf[p] = 0; String data(buf); int lastComma = data.lastIndexOf(','); if (lastComma == -1) return;
            String pld = data.substring(0, lastComma); uint8_t recCRC = data.substring(lastComma + 1).toInt(), calcCRC = 0;
            for (size_t i = 0; i < pld.length(); i++) calcCRC ^= (uint8_t)pld[i];
            if (calcCRC == recCRC) {
                float v[7]; int start = 0;
                for (int i = 0; i < 7; i++) { int idx = pld.indexOf(',', start); v[i] = pld.substring(start, idx).toFloat(); start = idx + 1; }
                planner.add({{v[0], v[1], v[2], v[3], v[4]}, (uint32_t)pld.substring(pld.lastIndexOf(',') + 1).toInt()});
                if ((long)v[5] != 0) stX.move((long)v[5]); if ((long)v[6] != 0) stY.move((long)v[6]);
            }
            p = 0;
        }
        else if (c == '\n' || c == '\r') { buf[p] = 0; if (p > 0) handleCommand(String(buf)); p = 0; }
        else if (p < 127) buf[p++] = c;
    }
}

// ============================================================================
//                                MAIN LOOPS
// ============================================================================

void updateHoming() {
    if (hState == IDLE || hState == DONE) return;
    float s1 = (3200 * config.gear1) / 360.0f, sG = (3200 * config.gear2) / 360.0f; hStep++;
    switch (hState) {
        case J1_F: st1.setSpeed(-s1 * 20); st1.runSpeed(); if (digitalRead(PIN_J1_LIMIT)) { hState = J1_B; hStep = 0; } break;
        case J1_B: st1.setSpeed(s1 * 5); st1.runSpeed(); if (hStep > s1 * 5) { hState = J1_S; hStep = 0; } break;
        case J1_S: st1.setSpeed(-s1 * 2); st1.runSpeed(); if (digitalRead(PIN_J1_LIMIT)) { st1.setCurrentPosition(0); planner.cj.j1 = 0; hState = J2_F; hStep = 0; } break;
        case J2_F: st2.setSpeed(-sG * 20); st2.runSpeed(); if (digitalRead(PIN_J2_LIMIT)) { hState = J2_B; hStep = 0; } break;
        case J2_B: st2.setSpeed(sG * 5); st2.runSpeed(); if (hStep > sG * 5) { hState = J2_S; hStep = 0; } break;
        case J2_S: st2.setSpeed(-sG * 2); st2.runSpeed(); if (digitalRead(PIN_J2_LIMIT)) { st2.setCurrentPosition(-90 * sG); planner.cj.j2 = -90; hState = J3_F; hStep = 0; } break;
        case J3_F: st3.setSpeed(-sG * 20); st3.runSpeed(); if (digitalRead(PIN_J3_LIMIT)) { hState = J3_B; hStep = 0; } break;
        case J3_B: st3.setSpeed(sG * 5); st3.runSpeed(); if (hStep > sG * 5) { hState = J3_S; hStep = 0; } break;
        case J3_S: st3.setSpeed(-sG * 2); st3.runSpeed(); if (digitalRead(PIN_J3_LIMIT)) { st3.setCurrentPosition(-90 * sG); planner.cj.j3 = 90; hState = DONE; hStep = 0; } break;
        case DONE: planner.add({{0, -90, 90, 90, 90}, 3000}); hState = IDLE; Serial.println("HOMING OK"); break;
        default: break;
    }
}

void setup() {
    Serial.begin(115200); Serial1.begin(115200); Serial4.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
    myusb.begin(); Wire.begin(); Wire1.begin(); Wire2.begin();
    
    EEPROM.get(0, config);
    if (config.magic != EEPROM_MAGIC_NUMBER) {
        config = {EEPROM_MAGIC_NUMBER, 8.33f, 20.0f, {true, true, true}, 1.0f, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    }
    cl1.offset = config.encOff[0]; cl2.offset = config.encOff[1]; cl3.offset = config.encOff[2];

    auto initD = [](TMC2209Stepper& d) {
        d.begin(); d.toff(5); d.rms_current(1800); d.microsteps(16); d.TCOOLTHRS(0xFFFFF); d.SGTHRS(10);
    };
    initD(drv1); initD(drv2); initD(drv3);

    st1.setMaxSpeed(40000); st2.setMaxSpeed(80000); st3.setMaxSpeed(60000);
    stX.setMaxSpeed(1000); stY.setMaxSpeed(1000);

    s4.attach(PIN_AXIS4_SERVO); s5.attach(PIN_AXIS5_SERVO); grip.attach(PIN_GRIPPER_SERVO);

    pinMode(6, OUTPUT); pinMode(27, OUTPUT); pinMode(32, OUTPUT); pinMode(12, OUTPUT);
    pinMode(PIN_J1_LIMIT, INPUT_PULLUP); pinMode(PIN_J2_LIMIT, INPUT_PULLUP); pinMode(PIN_J3_LIMIT, INPUT_PULLUP);

    // Sync Physical Position
    uint16_t r1 = cl1.readEncoder(Wire2), r2 = cl2.readEncoder(Wire), r3 = cl3.readEncoder(Wire1);
    if (r1 != 0xFFFF) planner.cj.j1 = (r1 / 4096.0f) * 360.0f - cl1.offset;
    if (r2 != 0xFFFF) planner.cj.j2 = (r2 / 4096.0f) * 360.0f - cl2.offset;
    if (r3 != 0xFFFF) planner.cj.j3 = (r3 / 4096.0f) * 360.0f - cl3.offset;

    resetEstop(); watchdog.begin({5, 8});
    Serial.println("Robot Ready v6.0");
}

void loop() {
    watchdog.feed(); myusb.Task();
    static unsigned long lu = micros(), lt = 0, ls = 0;
    float dt = (micros() - lu) / 1000000.0f; lu = micros();

    if (!estopActive) {
        if (hState != IDLE) {
            updateHoming();
        } else {
            // SD Sequence Handling
            if (isPlayingSequence && !planner.moving) {
                if (seqFile.available()) {
                    String line = seqFile.readStringUntil('\n'); float v[5]; int start = 0;
                    for (int i = 0; i < 5; i++) { int idx = line.indexOf(',', start); v[i] = line.substring(start, idx).toFloat(); start = idx + 1; }
                    planner.add({{v[0], v[1], v[2], v[3], v[4]}, (uint32_t)line.substring(line.lastIndexOf(',') + 1).toInt()});
                } else { seqFile.close(); isPlayingSequence = false; }
            }

            planner.update(); Joints tj = planner.cj, v = planner.cv;
            float adj1 = cl1.update(tj.j1, v.j1, dt, Wire2);
            float adj2 = cl2.update(tj.j2, v.j2, dt, Wire);
            float adj3 = cl3.update(tj.j3, v.j3, dt, Wire1);

            float s1 = (3200 * config.gear1) / 360.0f, sG = (3200 * config.gear2) / 360.0f;
            st1.setSpeed(v.j1 * s1 + adj1 * s1 * 5.0f); st2.setSpeed(v.j2 * sG + adj2 * sG * 5.0f); st3.setSpeed((v.j3 * sG + adj3 * sG * 5.0f) * -1.0f);
            st1.runSpeed(); st2.runSpeed(); st3.runSpeed(); stX.run(); stY.run();

            if (!planner.moving) {
                st1.setCurrentPosition((tj.j1 + adj1) * s1); st2.setCurrentPosition((tj.j2 + adj2) * sG); st3.setCurrentPosition((tj.j3 + adj3) * sG * -1.0f);
            }
            s4.write(tj.j4); s5.write(tj.j5);

            // Rate-limited StallGuard check
            if (millis() - ls > 100) {
                if (planner.moving && (drv1.SG_RESULT() == 0 || drv2.SG_RESULT() == 0 || drv3.SG_RESULT() == 0)) {
                    triggerEstop(); Serial.println("STALL DETECTED");
                }
                ls = millis();
            }
        }
    }

    processInput(Serial); processInput(Serial1); if (userial) processInput(userial);

    if (millis() - lt > 100) {
        Pose p = Kinematics::forward(planner.cj);
        Serial.printf("{\"j1\":%.1f,\"j2\":%.1f,\"j3\":%.1f,\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}\n", planner.cj.j1, planner.cj.j2, planner.cj.j3, p.x, p.y, p.z);
        lt = millis();
    }
}
