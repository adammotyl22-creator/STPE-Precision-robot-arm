/*
 * STPE Precision Robot Arm - Complete Upgraded Control System
 *
 * Version: v5.2 (Definitive High-Readability Single File)
 *
 * Objectives:
 * - Optimization: Quintic Spline interpolation for smooth C2-continuous motion.
 * - Upgrade: 5-DOF Analytical Inverse Kinematics (supports X,Y,Z, Pitch, Roll).
 * - Safety: TMC2209 StallGuard monitoring, Hardware Watchdog, Joint Limits.
 * - Readability: Refactored into Object-Oriented structure with clear spacing.
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

// ================= HARDWARE CONFIGURATION =================
#define BASE_H    160.0f
#define L1        135.0f
#define L2_PAR     55.0f
#define L2_PERP   135.0f
#define L3        125.0f   // Distance from J4 to gripper tip
#define EXCL_R2   6400.0f  // 80mm radius exclusion zone
#define FLOOR_CLR  20.0f

const int PIN_AXIS1_STEP = 3, PIN_AXIS1_DIR = 2, PIN_AXIS1_EN = 6;
const int PIN_AXIS2_STEP = 11, PIN_AXIS2_DIR = 10, PIN_AXIS2_EN = 27;
const int PIN_AXIS3_STEP = 29, PIN_AXIS3_DIR = 28, PIN_AXIS3_EN = 32;
const int PIN_MICRO_X_STEP = 34, PIN_MICRO_X_DIR = 33, PIN_MICRO_Y_STEP = 31, PIN_MICRO_Y_DIR = 30, PIN_MICRO_EN = 12;
const int PIN_AXIS4_SERVO = 23, PIN_AXIS5_SERVO = 22, PIN_GRIPPER_SERVO = 21;
const int PIN_J1_LIMIT = 5, PIN_J2_LIMIT = 4, PIN_J3_LIMIT = 36, PIN_X_LIMIT = 7, PIN_Y_LIMIT = 8;

const float LIMIT_J1_MIN = -85.0f, LIMIT_J1_MAX = 85.0f;
const float LIMIT_J2_MIN = -95.0f, LIMIT_J2_MAX = 85.0f;
const float LIMIT_J3_MIN = -85.0f, LIMIT_J3_MAX = 95.0f;
const float LIMIT_J4_MIN = 0.0f,   LIMIT_J4_MAX = 180.0f;
const float LIMIT_J5_MIN = 0.0f,   LIMIT_J5_MAX = 180.0f;

#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define STEPS_PER_MOTOR_REV (STEPS_PER_REV * MICROSTEPS)
#define EEPROM_MAGIC_NUMBER 0x494B3101

struct RobotConfig {
    uint32_t magic;
    float gear1;
    float gear2;
    bool jntEn[3];
    float speedOvr;
    float toolOff[3];
    float workOff[3];
} config;

struct Joints { float j1, j2, j3, j4, j5; };
struct Pose { float x, y, z, pitch, roll; };
struct Waypoint { Joints joints; uint32_t duration; };

// ================= UTILITIES & MATH =================
class QuinticSpline {
public:
    float a[6];

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

    static Joints inverse(Pose target) {
        Joints j;
        j.j1 = atan2f(target.y, target.x) * RAD_TO_DEG;

        float pr = target.pitch * DEG_TO_RAD;
        float rt = sqrtf(target.x * target.x + target.y * target.y);
        float rwc = rt - L3 * cosf(pr);
        float zwc = target.z + L3 * sinf(pr);
        float zr = zwc - BASE_H;

        float L2e = sqrtf(L2_PAR * L2_PAR + L2_PERP * L2_PERP);
        float gamma = atan2f(L2_PERP, L2_PAR);
        float D2 = rwc * rwc + zr * zr;
        float C3 = constrain((D2 - L1 * L1 - L2e * L2e) / (2.0f * L1 * L2e), -1.0f, 1.0f);
        float j3e = acosf(C3);

        j.j2 = (atan2f(rwc, zr) - atan2f(L2e * sinf(j3e), L1 + L2e * C3)) * RAD_TO_DEG;
        j.j3 = (j3e - gamma) * RAD_TO_DEG;
        j.j4 = target.pitch - (j.j2 + j.j3) + 90.0f;
        j.j5 = target.roll;
        return j;
    }
};

// ================= HARDWARE INTERFACE =================
WDT_T4<WDT1> watchdog;
USBHost myusb;
USBHub hub1(myusb);
USBSerial userial(myusb);

TMC2209Stepper drv1(&Serial4, 0.11f, 0b00), drv2(&Serial4, 0.11f, 0b10), drv3(&Serial4, 0.11f, 0b01);
AccelStepper st1(1, PIN_AXIS1_STEP, PIN_AXIS1_DIR);
AccelStepper st2(1, PIN_AXIS2_STEP, PIN_AXIS2_DIR);
AccelStepper st3(1, PIN_AXIS3_STEP, PIN_AXIS3_DIR);
AccelStepper stX(1, PIN_MICRO_X_STEP, PIN_MICRO_X_DIR);
AccelStepper stY(1, PIN_MICRO_Y_STEP, PIN_MICRO_Y_DIR);
Servo s4, s5, grip;

class MotionPlanner {
public:
    Waypoint q[32];
    uint8_t h = 0, t = 0;
    QuinticSpline splines[5];
    Joints cj = {0, -90, 90, 90, 90};
    Joints cv = {0, 0, 0, 0, 0};
    unsigned long startMicros;
    float duration;
    bool moving = false;

    void update() {
        if (!moving && h != t) {
            Waypoint& target = q[t];
            duration = target.duration / 1000.0f;
            if (duration <= 0) duration = 0.001f;

            for (int i = 0; i < 5; i++) {
                float p0 = (i == 0 ? cj.j1 : (i == 1 ? cj.j2 : (i == 2 ? cj.j3 : (i == 3 ? cj.j4 : cj.j5))));
                float p1 = (i == 0 ? target.joints.j1 : (i == 1 ? target.joints.j2 : (i == 2 ? target.joints.j3 : (i == 3 ? target.joints.j4 : target.joints.j5))));
                float v0 = (i == 0 ? cv.j1 : (i == 1 ? cv.j2 : (i == 2 ? cv.j3 : (i == 3 ? cv.j4 : cv.j5))));
                splines[i].calculate(p0, v0, 0, p1, 0, 0, duration);
            }
            startMicros = micros();
            moving = true;
        }

        if (moving) {
            float time = (micros() - startMicros) / 1000000.0f;
            if (time >= duration) {
                time = duration;
                moving = false;
                t = (t + 1) % 32;
            }
            cj = { splines[0].pos(time), splines[1].pos(time), splines[2].pos(time), splines[3].pos(time), splines[4].pos(time) };
            cv = { splines[0].vel(time), splines[1].vel(time), splines[2].vel(time), splines[3].vel(time), splines[4].vel(time) };
        }
    }

    void add(Waypoint wp) {
        if (Kinematics::checkWorkspace(wp.joints)) {
            uint8_t next = (h + 1) % 32;
            if (next != t) {
                q[h] = wp;
                h = next;
            }
        } else {
            Serial.println("ERR: WORKSPACE_VIOLATION");
        }
    }

    void stop() {
        h = t;
        moving = false;
        cv = {0, 0, 0, 0, 0};
    }
} planner;

class ClosedLoop {
public:
    float kP = 0.3, kFF = 0.1, integral = 0, lastErr = 0, offset = 0;
    bool en = true;

    uint16_t readEncoder(TwoWire& w) {
        w.beginTransmission(0x36);
        w.write(0x0C);
        if (w.endTransmission(false) != 0) return 0xFFFF;
        if (w.requestFrom(0x36, 2) != 2) return 0xFFFF;
        return ((uint16_t)w.read() << 8) | w.read();
    }

    float update(float target, float vel, float dt, TwoWire& w) {
        if (!en || dt <= 0) return 0;
        uint16_t raw = readEncoder(w);
        if (raw == 0xFFFF) return 0;

        float actual = (raw / 4096.0f) * 360.0f - offset;
        float err = target - actual;
        while (err > 180) err -= 360;
        while (err < -180) err += 360;

        integral = constrain(integral + err * dt, -10, 10);
        float der = (err - lastErr) / dt;
        lastErr = err;

        return (kP * err + 0.05f * integral + kFF * vel);
    }
} cl1, cl2, cl3;

// ================= SYSTEMS & STATE =================
bool estopActive = false;
enum HomingState { IDLE, J1_F, J1_B, J1_S, J2_F, J2_B, J2_S, J3_F, J3_B, J3_S, DONE } homingState = IDLE;
unsigned long hStep = 0;

struct TeachPoint { Joints joints; float grip; };
TeachPoint teachPoints[64];
uint8_t teachCount = 0;
bool isTeachMode = false;

void triggerEstop() {
    estopActive = true;
    digitalWrite(PIN_AXIS1_EN, 1);
    digitalWrite(PIN_AXIS2_EN, 1);
    digitalWrite(PIN_AXIS3_EN, 1);
    digitalWrite(PIN_MICRO_EN, 1);
    Serial.println("!!! EMERGENCY STOP !!!");
}

void resetEstop() {
    estopActive = false;
    digitalWrite(PIN_AXIS1_EN, 0);
    digitalWrite(PIN_AXIS2_EN, 0);
    digitalWrite(PIN_AXIS3_EN, 0);
    digitalWrite(PIN_MICRO_EN, 0);
    Serial.println("ESTOP RESET OK");
}

// ================= COMMAND PROCESSING =================
char inBuf[128];
uint8_t inPos = 0;

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.startsWith("MOV")) {
        float x = 0, y = 0, z = 160, p = 0, r = 0;
        uint32_t d = 1000;
        int iX = cmd.indexOf('X'), iY = cmd.indexOf('Y'), iZ = cmd.indexOf('Z'), iP = cmd.indexOf('P'), iR = cmd.indexOf('R'), iD = cmd.indexOf('D');
        if (iX != -1) x = cmd.substring(iX + 1).toFloat();
        if (iY != -1) y = cmd.substring(iY + 1).toFloat();
        if (iZ != -1) z = cmd.substring(iZ + 1).toFloat();
        if (iP != -1) p = cmd.substring(iP + 1).toFloat();
        if (iR != -1) r = cmd.substring(iR + 1).toFloat();
        if (iD != -1) d = cmd.substring(iD + 1).toInt();
        planner.add({Kinematics::inverse({x, y, z, p, r}), d});
    } else if (cmd == "ESTOP") {
        triggerEstop();
    } else if (cmd == "RESET") {
        resetEstop();
    } else if (cmd == "HOME") {
        homingState = J1_F;
        hStep = 0;
    } else if (cmd.startsWith("GRIP")) {
        grip.write(cmd.substring(5).toInt());
    } else if (cmd.startsWith("D1")) {
        Joints j = planner.cj; j.j1 += cmd.substring(2).toFloat(); planner.add({j, 500});
    } else if (cmd.startsWith("D2")) {
        Joints j = planner.cj; j.j2 += cmd.substring(2).toFloat(); planner.add({j, 500});
    } else if (cmd.startsWith("D3")) {
        Joints j = planner.cj; j.j3 += cmd.substring(2).toFloat(); planner.add({j, 500});
    } else if (cmd.startsWith("DX")) {
        stX.move(cmd.substring(2).toInt());
    } else if (cmd.startsWith("DY")) {
        stY.move(cmd.substring(2).toInt());
    } else if (cmd == "SAVECFG") {
        EEPROM.put(0, config);
        Serial.println("Config Saved");
    } else if (cmd == "CAL") {
        uint16_t r1 = cl1.readEncoder(Wire2), r2 = cl2.readEncoder(Wire), r3 = cl3.readEncoder(Wire1);
        if (r1 != 0xFFFF) cl1.offset = (r1 / 4096.0f) * 360.0f - planner.cj.j1;
        if (r2 != 0xFFFF) cl2.offset = (r2 / 4096.0f) * 360.0f - planner.cj.j2;
        if (r3 != 0xFFFF) cl3.offset = (r3 / 4096.0f) * 360.0f - planner.cj.j3;
        Serial.println("CALIBRATED");
    } else if (cmd.startsWith("TEACH")) {
        String arg = cmd.substring(5); arg.trim();
        if (arg == "START") { isTeachMode = true; teachCount = 0; Serial.println("TEACH START"); }
        else if (arg == "RECORD") { if(teachCount < 64) teachPoints[teachCount++] = {planner.cj, (float)grip.read()}; Serial.println("RECORDED"); }
        else if (arg == "STOP") { isTeachMode = false; Serial.println("TEACH STOP"); }
    }
}

void processInput(Stream& s) {
    while (s.available()) {
        char c = s.read();
        if (c == '<') {
            inPos = 0;
            inBuf[0] = 0;
        } else if (c == '>') {
            inBuf[inPos] = 0;
            String data(inBuf);
            int lastComma = data.lastIndexOf(',');
            if (lastComma == -1) return;

            String payload = data.substring(0, lastComma);
            uint8_t receivedCRC = (uint8_t)data.substring(lastComma + 1).toInt();
            uint8_t calculatedCRC = 0;
            for (size_t i = 0; i < payload.length(); i++) calculatedCRC ^= (uint8_t)payload[i];

            if (calculatedCRC == receivedCRC) {
                float v[5];
                int start = 0;
                for (int i = 0; i < 5; i++) {
                    int idx = payload.indexOf(',', start);
                    if (idx == -1) break;
                    v[i] = payload.substring(start, idx).toFloat();
                    start = idx + 1;
                }
                uint32_t dur = payload.substring(payload.lastIndexOf(',') + 1).toInt();
                planner.add({{v[0], v[1], v[2], v[3], v[4]}, dur});
            } else {
                Serial.println("ERR: CRC_MISMATCH");
            }
            inPos = 0;
        } else if (c == '\n' || c == '\r') {
            inBuf[inPos] = 0;
            if (inPos > 0) handleCommand(String(inBuf));
            inPos = 0;
        } else if (inPos < 127) {
            inBuf[inPos++] = c;
        }
    }
}

// ================= MAIN LOOPS =================
void updateHoming() {
    if (homingState == IDLE || homingState == DONE) return;
    float s1 = (3200 * config.gear1) / 360.0f;
    float sG = (3200 * config.gear2) / 360.0f;
    hStep++;

    switch (homingState) {
        case J1_F: st1.setSpeed(-s1 * 20); st1.runSpeed(); if (digitalRead(PIN_J1_LIMIT)) { homingState = J1_B; hStep = 0; } break;
        case J1_B: st1.setSpeed(s1 * 5); st1.runSpeed(); if (hStep > s1 * 5) { homingState = J1_S; hStep = 0; } break;
        case J1_S: st1.setSpeed(-s1 * 2); st1.runSpeed(); if (digitalRead(PIN_J1_LIMIT)) { st1.setCurrentPosition(0); planner.cj.j1 = 0; homingState = J2_F; hStep = 0; } break;
        case J2_F: st2.setSpeed(-sG * 20); st2.runSpeed(); if (digitalRead(PIN_J2_LIMIT)) { homingState = J2_B; hStep = 0; } break;
        case J2_B: st2.setSpeed(sG * 5); st2.runSpeed(); if (hStep > sG * 5) { homingState = J2_S; hStep = 0; } break;
        case J2_S: st2.setSpeed(-sG * 2); st2.runSpeed(); if (digitalRead(PIN_J2_LIMIT)) { st2.setCurrentPosition(-90 * sG); planner.cj.j2 = -90; homingState = J3_F; hStep = 0; } break;
        case J3_F: st3.setSpeed(-sG * 20); st3.runSpeed(); if (digitalRead(PIN_J3_LIMIT)) { homingState = J3_B; hStep = 0; } break;
        case J3_B: st3.setSpeed(sG * 5); st3.runSpeed(); if (hStep > sG * 5) { homingState = J3_S; hStep = 0; } break;
        case J3_S: st3.setSpeed(-sG * 2); st3.runSpeed(); if (digitalRead(PIN_J3_LIMIT)) { st3.setCurrentPosition(-90 * sG); planner.cj.j3 = 90; homingState = DONE; hStep = 0; } break;
        case DONE: planner.add({{0, -90, 90, 90, 90}, 3000}); homingState = IDLE; Serial.println("HOMING COMPLETE"); break;
        default: break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial4.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
    
    myusb.begin();
    Wire.begin(); Wire1.begin(); Wire2.begin();
    
    EEPROM.get(0, config);
    if (config.magic != EEPROM_MAGIC_NUMBER) {
        config = {EEPROM_MAGIC_NUMBER, 8.33f, 20.0f, {true, true, true}, 1.0f, {0, 0, 0}, {0, 0, 0}};
    }
    
    auto initD = [](TMC2209Stepper& d) {
        d.begin(); d.toff(5); d.rms_current(1800); d.microsteps(16);
        d.TCOOLTHRS(0xFFFFF); d.SGTHRS(10);
    };
    initD(drv1); initD(drv2); initD(drv3);

    st1.setMaxSpeed(40000); st2.setMaxSpeed(80000); st3.setMaxSpeed(60000);
    stX.setMaxSpeed(1000); stY.setMaxSpeed(1000);
    
    s4.attach(PIN_AXIS4_SERVO); s5.attach(PIN_AXIS5_SERVO); grip.attach(PIN_GRIPPER_SERVO);
    
    pinMode(PIN_AXIS1_EN, OUTPUT); pinMode(PIN_AXIS2_EN, OUTPUT); pinMode(PIN_AXIS3_EN, OUTPUT); pinMode(PIN_MICRO_EN, OUTPUT);
    pinMode(PIN_J1_LIMIT, INPUT_PULLUP); pinMode(PIN_J2_LIMIT, INPUT_PULLUP); pinMode(PIN_J3_LIMIT, INPUT_PULLUP);
    
    resetEstop();
    watchdog.begin({5, 8});
    Serial.println("Precision Robot Control v5.2 Ready");
}

void loop() {
    watchdog.feed();
    myusb.Task();

    static unsigned long lu = micros();
    float dt = (micros() - lu) / 1000000.0f;
    lu = micros();

    if (!estopActive) {
        if (homingState != IDLE) {
            updateHoming();
        } else {
            planner.update();
            Joints tj = planner.cj;
            Joints v = planner.cv;

            float adj1 = cl1.update(tj.j1, v.j1, dt, Wire2);
            float adj2 = cl2.update(tj.j2, v.j2, dt, Wire);
            float adj3 = cl3.update(tj.j3, v.j3, dt, Wire1);

            float s1 = (3200 * config.gear1) / 360.0f;
            float sG = (3200 * config.gear2) / 360.0f;

            // Velocity mode drive with real-time adjustment
            st1.setSpeed(v.j1 * s1 + adj1 * s1 * 5.0f);
            st2.setSpeed(v.j2 * sG + adj2 * sG * 5.0f);
            st3.setSpeed((v.j3 * sG + adj3 * sG * 5.0f) * -1.0f);

            st1.runSpeed(); st2.runSpeed(); st3.runSpeed();
            stX.run(); stY.run();

            if (!planner.moving) {
                st1.setCurrentPosition((tj.j1 + adj1) * s1);
                st2.setCurrentPosition((tj.j2 + adj2) * sG);
                st3.setCurrentPosition((tj.j3 + adj3) * sG * -1.0f);
            }

            s4.write(tj.j4);
            s5.write(tj.j5);

            // StallGuard: check only while moving at significant velocity
            if (planner.moving && (drv1.SG_RESULT() == 0 || drv2.SG_RESULT() == 0 || drv3.SG_RESULT() == 0)) {
                triggerEstop();
                Serial.println("STALL DETECTED");
            }
        }
    }

    processInput(Serial);
    processInput(Serial1);
    if (userial) processInput(userial);

    static uint32_t lt = 0;
    if (millis() - lt > 100) {
        Pose p = Kinematics::forward(planner.cj);
        Serial.printf("{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"j4\":%.1f,\"j5\":%.1f}\n", p.x, p.y, p.z, planner.cj.j4, planner.cj.j5);
        lt = millis();
    }
}
