/*
 * IK Control - Inverse Kinematics + Full Hardware + USB Host
 * - 3x Main Arm (TMC2209)
 * - 2x X/Y Gantry (A4988)
 * - 3x Servos (Axis 4, 5 + Gripper)
 * - 3x AS5600 Encoders
 * - USB Host (Master Gesture Controller)
 * - Non-Blocking Homing State Machine
 * - Analytical IK Solver
 * - Cartesian Linear Path (MOVL)
 * - Teach & Playback Mode
 * - JSON Telemetry Stream
 * - Tool / Work Frame Offsets
 * - Watchdog Timer
 *
 * ROBOT DIMENSIONS:
 * - Link 1 (Shoulder): 135 mm
 * - Link 2 (Forearm):  135 mm
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

struct SplinePoint { float j1, j2, j3, j4, j5; };
SplinePoint calcTangent(SplinePoint prev, SplinePoint next);

const char *FIRMWARE_VERSION = "v3.0.0";
const char *BUILD_DATE = __DATE__;
const char *BUILD_TIME = __TIME__;

// ─── Watchdog ────────────────────────────────────────────────────────────────
WDT_T4<WDT1> watchdog;

// ================= USB HOST SETUP =================
USBHost myusb;
USBHub hub1(myusb);
USBSerial userial(myusb); // Serial from Pro Micro

// ================= PIN DEFINITIONS =================
// Axis 1 (Main Base)
const int PIN_AXIS1_STEP = 3;
const int PIN_AXIS1_DIR = 2;
const int PIN_AXIS1_EN = 6;

// Axis 2 (Shoulder)
const int PIN_AXIS2_STEP = 11;
const int PIN_AXIS2_DIR = 10;
const int PIN_AXIS2_EN = 27;

// Axis 3 (Elbow)
const int PIN_AXIS3_STEP = 29;
const int PIN_AXIS3_DIR = 28;
const int PIN_AXIS3_EN = 32;

// X/Y Gantry (DVD Steppers)
const int PIN_MICRO_X_STEP = 34;
const int PIN_MICRO_X_DIR = 33;
const int PIN_MICRO_Y_STEP = 31;
const int PIN_MICRO_Y_DIR = 30;
const int PIN_MICRO_EN = 12;
const int PIN_MICRO_X_LIMIT = 7;
const int PIN_MICRO_Y_LIMIT = 8;

// Servos (Axis 4 & 5 - Wrist)
const int PIN_AXIS4_SERVO = 23;
const int PIN_AXIS5_SERVO = 22;

// Gripper servo (Axis 6)
const int PIN_GRIPPER_SERVO = 21;

// ── Limit switch pins (wire your physical switches here) ──────────────────
const int PIN_J1_LIMIT = 5;
const int PIN_J2_LIMIT = 4;
const int PIN_J3_LIMIT = 36;

// ── EEPROM memory map ─────────────────────────────────────────────────────
#define EEPROM_MAGIC_ADDR      0    // 4 bytes
#define EEPROM_GEAR1_ADDR      4    // 4 bytes float
#define EEPROM_GEAR2_ADDR      8    // 4 bytes float
#define EEPROM_JNT_EN_ADDR    12    // 3 bytes  (one per joint)
#define EEPROM_SPEED_OVR_ADDR 15    // 4 bytes float
#define EEPROM_TOOL_OFF_ADDR  19    // 12 bytes (3x float)
#define EEPROM_WORK_OFF_ADDR  31    // 12 bytes (3x float)
#define EEPROM_MAGIC_NUMBER   0x494B3101  // "IK" + version 01

// ── Speed override (global multiplier 0.0–1.0) ───────────────────────────
float speedOverride = 1.0;   // 1.0 = 100%, 0.5 = 50%

// ── SD card flag ──────────────────────────────────────────────────────────
static bool sd_ready = false;
File currentSequenceFile;
bool isPlayingSequence = false;

// AS5600 Encoders — Hardware I2C
// ENC1 (Base)     → Wire2: SDA=25, SCL=24
// ENC2 (Shoulder) → Wire:  SDA=18, SCL=19
// ENC3 (Elbow)    → Wire1: SDA=44, SCL=45

float enc1Offset = 0;
float enc2Offset = 0;
float enc3Offset = 0;

// Cached encoder values to prevent redundant I2C reads
uint16_t lastEnc2 = 0;
uint16_t lastEnc3 = 0;

bool useEncoders = true; // Default: Enabled
bool estopActive = false; // Track emergency stop state

// UART / TMC Configuration
#define SERIAL_PORT Serial4
#define R_SENSE 0.11f

// ================= ROBOT GEOMETRY =================
#define BASE_H    160.0f
#define L1        135.0f
#define L2_PAR     55.0f   // along J3 arm direction
#define L2_PERP   135.0f   // perpendicular (forearm extension)
#define L3         25.0f   // J4→J5 wrist link (mm)
#define EXCL_R2   6400.0f  // r < 80mm exclusion zone (80² = 6400)
#define FLOOR_CLR  20.0f   // min clearance above floor

// Wrist presets for calculateIK
#define WRIST_LEVEL     0   // tool poziomo (pick & place z boku)
#define WRIST_DOWN_0    1   // tool pionowo w dol, J5=90deg  (pick z gory, prosto)
#define WRIST_DOWN_90   2   // tool pionowo w dol, J5=180deg (pick z gory, obrocony)
#define WRIST_PITCH_0   3   // pitch=0deg,   J5 z parametru
#define WRIST_PITCH_90  4   // pitch=90deg,  J5 z parametru
#define WRIST_PITCH_180 5   // pitch=180deg, J5 z parametru
#define WRIST_MANUAL    6   // J4 i J5 podane recznie

// ================= SAFETY LIMITS =================
// Limity miękkie — jeden zestaw, używany wszędzie
const float LIMIT_J1_MIN = -85.0f, LIMIT_J1_MAX =  85.0f;
const float LIMIT_J2_MIN = -95.0f, LIMIT_J2_MAX =  85.0f;
const float LIMIT_J3_MIN = -85.0f, LIMIT_J3_MAX =  95.0f;
const float LIMIT_J4_MIN =   0.0f, LIMIT_J4_MAX = 180.0f;
const float LIMIT_J5_MIN =   0.0f, LIMIT_J5_MAX = 180.0f;

// Commanded Positions (Closed Loop Targets)
float commandedAngle1 = 0;
float commandedAngle2 = 0;
float commandedAngle3 = 0;

// ================= MOTION SETTINGS =================
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; // 3200

// Main Arm Gearing
float gearboxRatioAxis1 = 8.33;
float stepsPerDegAxis1 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0; // ~74.05

float gearboxRatio = 20.0;
float stepsPerDegGeared = (STEPS_PER_MOTOR_REV * gearboxRatio) / 360.0; // ~177.78

// J3 (elbow) is mechanically inverted versus IK positive angle convention.
const float J3_STEP_SIGN = -1.0f;
inline long j3AngleToSteps(float deg) { return (long)(deg * stepsPerDegGeared * J3_STEP_SIGN); }

// ================= OBJECTS =================
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, 0b00); // Address 0
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, 0b10); // Address 2
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, 0b01); // Address 1

AccelStepper stepper1(AccelStepper::DRIVER, PIN_AXIS1_STEP, PIN_AXIS1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, PIN_AXIS2_STEP, PIN_AXIS2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, PIN_AXIS3_STEP, PIN_AXIS3_DIR);
AccelStepper stepperX(AccelStepper::DRIVER, PIN_MICRO_X_STEP, PIN_MICRO_X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, PIN_MICRO_Y_STEP, PIN_MICRO_Y_DIR);

Servo servo4;
Servo servo5;
Servo gripper;
float gripperAngle = 90.0f;

// Buffers (Fixed 128 bytes)
char inputBuffer[128];
uint8_t inputBufferPos = 0;
char usbBuffer[128];
uint8_t usbBufferPos = 0;
char espBuffer[128];
uint8_t espBufferPos = 0;

// PID Controller Structure
struct PIDController {
  float kP, kI, kD, kFF, kG;
  float integral = 0;
  float lastError = 0;
  float maxIntegral = 100.0;
  float deadband = 0.5;

  // Initialize with specific gains
  void init(float p, float i, float d, float ff, float g = 0.0) {
    kP = p; kI = i; kD = d; kFF = ff; kG = g;
    integral = 0;
    lastError = 0;
  }

  float calculate(float error, float dt, float targetVelocity) {
    // Deadband
    if (abs(error) < deadband) {
      integral = 0; // Reset integral in deadband to avoid windup
      return kFF * targetVelocity;
    }

    integral += error * dt;

    // Anti-windup
    if (abs(error) > 20.0)
      integral = 0;
    integral = constrain(integral, -maxIntegral, maxIntegral);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = (kP * error) + (kI * integral) + (kD * derivative) +
                   (kFF * targetVelocity);
    return output;
  }
};

PIDController pid1, pid2, pid3;

// Closed Loop Adjustments
float pidAdj1 = 0.0f;
float pidAdj2 = 0.0f;
float pidAdj3 = 0.0f;

// Timing Constants
const unsigned long MAX_INPUT_TIME_US = 15000;
const unsigned long MAX_LOOP_TIME_US = 10000;
const uint8_t MAX_ENCODER_FAILURES = 10;

// State Flags
bool manualCL = false; // Set by CL_ON: suppress auto-disable on encoder errors
bool encEnabled[3] = {false, true, true};
bool jointEnabled[3] = {true, true, true};
bool useTelemetry = false;    // 10Hz position stream (AR4 format)
bool useJsonTelemetry = false; // 10Hz JSON stream for dashboard

// ── Collision detection ──────────────────────────────────────────────────
#define COLLISION_THRESH_DEG  0.5f
int  J1collisionTrue = 0;
int  J2collisionTrue = 0;
int  J3collisionTrue = 0;

// ── Hold current state ───────────────────────────────────────────────────
bool holdCurrentActive = false;

// ── Waypoint queue ───────────────────────────────────────────────────────
struct Waypoint {
  float j1, j2, j3, j4, j5;
  unsigned long duration_ms;
};

const uint8_t WAYPOINT_QUEUE_SIZE = 32;  // increased from 16
Waypoint motionQueue[WAYPOINT_QUEUE_SIZE];
uint8_t qHead = 0;
uint8_t qTail = 0;
bool isMovingQueue = false;

// ── Spline Interpolation State ───────────────────────────────────────────
float splineT = 0.0f;
unsigned long lastSplineTime = 0; // stored in microseconds!
float splineDuration = 0.0f;      // stored in microseconds!
SplinePoint sp1, sp2, sv1, sv2;
SplinePoint spPrev;
bool wasContinuous = false;

// ── Slider / Joystick Streaming Target ────────────────────────────────
// When slider commands arrive rapidly, we skip the spline queue and
// directly set a live target for AccelStepper to chase.
bool   streamMode       = false;
float  streamTargetJ1   = 0.0f;
float  streamTargetJ2   = 0.0f;
float  streamTargetJ3   = 0.0f;
float  streamVelJ1      = 0.0f;
float  streamVelJ2      = 0.0f;
float  streamVelJ3      = 0.0f;
unsigned long streamLastUpdateMs = 0;
unsigned long lastStreamMicros = 0;
const unsigned long STREAM_TIMEOUT_MS = 300; // ms without update → back to hold

SplinePoint calcTangent(SplinePoint prev, SplinePoint next) {
   SplinePoint v;
   v.j1 = 0.5f * (next.j1 - prev.j1);
   v.j2 = 0.5f * (next.j2 - prev.j2);
   v.j3 = 0.5f * (next.j3 - prev.j3);
   v.j4 = 0.5f * (next.j4 - prev.j4);
   v.j5 = 0.5f * (next.j5 - prev.j5);
   return v;
}

// ── Tool & Work Frame Offsets ────────────────────────────────────────────
struct OffsetFrame { float x = 0, y = 0, z = 0; };
OffsetFrame toolOffset;  // tool-tip offset from J5 centre (mm)
OffsetFrame workOffset;  // work-object origin shift (mm)

// ── Teach Mode ───────────────────────────────────────────────────────────
struct TeachPoint { float j1, j2, j3, j4, j5, grip; };
const uint8_t MAX_TEACH_POINTS = 64;
TeachPoint teachPoints[MAX_TEACH_POINTS];
uint8_t teachCount   = 0;
uint8_t teachPlayIdx = 0;
bool isTeachMode     = false;  // arm in teach-recording mode
bool isTeachPlaying  = false;

// ── Non-Blocking Homing State Machine ───────────────────────────────────
enum HomingState {
  HOMING_IDLE,
  HOMING_J1_FAST, HOMING_J1_BACKOFF, HOMING_J1_SLOW,
  HOMING_J2_FAST, HOMING_J2_BACKOFF, HOMING_J2_SLOW,
  HOMING_J3_FAST, HOMING_J3_BACKOFF, HOMING_J3_SLOW,
  HOMING_COMPLETE, HOMING_FAULT
};
HomingState homingState  = HOMING_IDLE;
bool homingDoJ1 = false, homingDoJ2 = false, homingDoJ3 = false;
uint8_t homingRetries    = 0;
unsigned long homingStepCount = 0;
const uint8_t MAX_HOME_RETRIES_NB = 3;

// Function Prototypes
void calculateIK(float x, float y, float z, unsigned long duration = 1000,
                 int wristMode = WRIST_LEVEL,
                 float j4_manual_deg = 90.0f, float j5_deg = 90.0f, bool isMovL = false);
void moveToAngles(float theta1, float theta2, float theta3,
                  unsigned long duration_ms, bool skipWorkspaceCheck = false,
                  float t4 = -1.0f, float t5 = -1.0f, bool isMovL = false);
void parseCommand(String command, float value);
void parsePacket(char *data);
void processInput(Stream &stream, char *buffer, uint8_t &pos);
void printDriver(const char *name, TMC2209Stepper &driver);
uint16_t readAS5600(TwoWire &wire);
void updateClosedLoop();
float getEncoderAngle(uint16_t raw, float offset);

// ── EEPROM helpers ────────────────────────────────────────────────────────
bool eeprom_valid();
void eeprom_save_all();
void eeprom_load_all();

// ── Forward Kinematics ───────────────────────────────────────────────────
void forwardKinematics(float j1_deg, float j2_deg, float j3_deg,
                       float &x, float &y, float &z, bool raw = false);

// ── Workspace check ──────────────────────────────────────────────────────
bool checkWorkspace(float j1, float j2, float j3, float j4, float j5);

// ── Homing ───────────────────────────────────────────────────────────────
void startHoming(bool doJ1, bool doJ2, bool doJ3);
void updateHoming();           // called every loop() — non-blocking
void homeAxes(bool doJ1, bool doJ2, bool doJ3); // legacy blocking (kept for compat)

// ── AR4-format responses ─────────────────────────────────────────────────
void sendRobotPos();
void sendACK();
void sendError(const char *code);
void sendJSONTelemetry();      // JSON telemetry stream

// ── Encoder collision detection ──────────────────────────────────────────
void checkEncoders();

// ── iHold current management ─────────────────────────────────────────────
void applyHoldCurrent();
void applyRunCurrent();
void applySpeedOverride();

// ── SD card teach/playback ───────────────────────────────────────────────
bool initSD_local();
void savePosition(String name);
void loadPosition(String name);
void runSequence(String name);
void updateSequenceQueue();

// ── Linear Cartesian Move ────────────────────────────────────────────────
void movL(float x2, float y2, float z2, unsigned long duration,
          int wristMode = WRIST_LEVEL, float j4 = 90.0f, float j5 = 90.0f);

// ── Gripper ──────────────────────────────────────────────────────────────
void setGripper(float angle);

// ── Teach Mode ───────────────────────────────────────────────────────────
void updateTeachPlayback();

// ================= GRIPPER =================
void setGripper(float angle) {
  gripperAngle = constrain(angle, 0.0f, 180.0f);
  gripper.write((int)gripperAngle);
}

// ================= JSON TELEMETRY =================
void sendJSONTelemetry() {
  float cx, cy, cz;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, cx, cy, cz);

  Serial.print("{\"j1\":");   Serial.print(commandedAngle1, 2);
  Serial.print(",\"j2\":");   Serial.print(commandedAngle2, 2);
  Serial.print(",\"j3\":");   Serial.print(commandedAngle3, 2);
  Serial.print(",\"j4\":");   Serial.print(servo4.read());
  Serial.print(",\"j5\":");   Serial.print(servo5.read());
  Serial.print(",\"grip\":"); Serial.print((int)gripperAngle);
  Serial.print(",\"x\":");    Serial.print(cx, 1);
  Serial.print(",\"y\":");    Serial.print(cy, 1);
  Serial.print(",\"z\":");    Serial.print(cz, 1);
  Serial.print(",\"spd\":");  Serial.print((int)(speedOverride * 100));
  Serial.print(",\"estop\":"); Serial.print(estopActive ? "true" : "false");
  Serial.print(",\"homing\":"); Serial.print(homingState != HOMING_IDLE ? "true" : "false");
  Serial.print(",\"teach\":"); Serial.print(isTeachMode ? "true" : "false");
  Serial.print(",\"tpts\":");  Serial.print(teachCount);
  Serial.println("}");
}

// ================= LINEAR CARTESIAN MOVE (MOVL) =================
void movL(float x2, float y2, float z2, unsigned long duration,
          int wristMode, float j4, float j5) {
  // Compute current TCP via FK
  float x1, y1, z1;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, x1, y1, z1);

  float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
  float dist = sqrtf(dx*dx + dy*dy + dz*dz);
  if (dist < 1.0f) { // already there (< 1 mm), just do IK directly
    calculateIK(x2, y2, z2, duration, wristMode, j4, j5);
    return;
  }

  // Determine number of segments (aim for ~5 mm per step, capped at 30)
  int segs = (int)(dist / 5.0f);
  segs = constrain(segs, 2, 30);
  unsigned long segDur = duration / (unsigned long)segs;
  if (segDur < 50) segDur = 50;  // minimum 50ms per segment

  Serial.print("MOVL: dist="); Serial.print(dist, 1);
  Serial.print("mm segs="); Serial.println(segs);

  float sj4 = (float)servo4.read();
  float sj5 = (float)servo5.read();

  for (int i = 1; i <= segs; i++) {
    float t = (float)i / segs;
    float xi = x1 + dx * t;
    float yi = y1 + dy * t;
    float zi = z1 + dz * t;
    float ji4 = sj4 + (j4 - sj4) * t;
    float ji5 = sj5 + (j5 - sj5) * t;
    // calculateIK will enqueue into the waypoint queue
    calculateIK(xi, yi, zi, segDur, wristMode, ji4, ji5, true);
  }
}

// ================= TEACH MODE PLAYBACK =================
void updateTeachPlayback() {
  if (!isTeachPlaying) return;
  if (isMovingQueue || qHead != qTail) return; // wait for current move to finish

  if (teachPlayIdx >= teachCount) {
    isTeachPlaying = false;
    teachPlayIdx = 0;
    Serial.println("TEACH: Playback complete");
    sendACK();
    return;
  }

  TeachPoint &tp = teachPoints[teachPlayIdx++];
  // Use the new smoothed moveToAngles for Teach playback too!
  moveToAngles(tp.j1, tp.j2, tp.j3, 1500, false, tp.j4, tp.j5);
  setGripper(tp.grip);
}

// ================= NON-BLOCKING HOMING =================
void startHoming(bool doJ1, bool doJ2, bool doJ3) {
  if (estopActive) {
    Serial.println("ERR: Cannot home while ESTOP active");
    return;
  }
  homingDoJ1 = doJ1; homingDoJ2 = doJ2; homingDoJ3 = doJ3;
  homingRetries = 0;
  homingStepCount = 0;
  qHead = 0; qTail = 0; isMovingQueue = false;

  // Apply homing speeds
  stepper1.setMaxSpeed(20000.0f);
  stepper2.setMaxSpeed(35000.0f);
  stepper3.setMaxSpeed(25000.0f);

  Serial.println("HOMING: Starting (non-blocking)...");

  if (doJ1) {
    homingState = HOMING_J1_FAST;
    stepper1.setSpeed(-stepsPerDegAxis1 * 20.0f);  // 20 deg/s fast
  } else if (doJ2) {
    homingState = HOMING_J2_FAST;
    stepper2.setSpeed(-stepsPerDegGeared * 20.0f);
  } else if (doJ3) {
    homingState = HOMING_J3_FAST;
    stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
  }
}

void updateHoming() {
  if (homingState == HOMING_IDLE || homingState == HOMING_COMPLETE || homingState == HOMING_FAULT)
    return;

  const long MAX_FAST_STEPS  = (long)(200.0f * stepsPerDegGeared);
  const long BACKOFF_STEPS   = (long)(5.0f   * stepsPerDegGeared);
  const float SLOW_SPD_DEG   = 2.0f;

  homingStepCount++;

  switch (homingState) {

    // ── J1 FAST ─────────────────────────────────────────────────────────
    case HOMING_J1_FAST:
      stepper1.runSpeed();
      if (digitalRead(PIN_J1_LIMIT) == HIGH) {
        stepper1.setSpeed(stepsPerDegAxis1 * SLOW_SPD_DEG); // reverse
        homingStepCount = 0;
        homingState = HOMING_J1_BACKOFF;
        Serial.print("HOMING J1 backoff... ");
      } else if (homingStepCount > (unsigned long)(180.0f * stepsPerDegAxis1)) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J1");
        } else {
          stepper1.setSpeed(stepsPerDegAxis1 * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J1_BACKOFF;
        }
      }
      break;

    case HOMING_J1_BACKOFF:
      stepper1.runSpeed();
      if (homingStepCount >= (unsigned long)(5.0f * stepsPerDegAxis1)) {
        stepper1.setSpeed(-stepsPerDegAxis1 * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J1_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J1_SLOW:
      stepper1.runSpeed();
      if (digitalRead(PIN_J1_LIMIT) == HIGH) {
        stepper1.setCurrentPosition(0);
        commandedAngle1 = 0.0f;
        if (encEnabled[0]) {
          uint16_t raw = readAS5600(Wire2);
          if (raw != 0xFFFF) enc1Offset = (raw / 4096.0f) * 360.0f - commandedAngle1;
        }
        Serial.println("OK");
        homingRetries = 0; homingStepCount = 0;
        if (homingDoJ2) {
          stepper2.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J2_FAST;
          Serial.print("HOMING J2 fast... ");
        } else if (homingDoJ3) {
          stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J3_FAST;
          Serial.print("HOMING J3 fast... ");
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegAxis1)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J1_SLOW");
      }
      break;

    // ── J2 FAST ─────────────────────────────────────────────────────────
    case HOMING_J2_FAST:
      stepper2.runSpeed();
      if (digitalRead(PIN_J2_LIMIT) == HIGH) {
        stepper2.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J2_BACKOFF;
        Serial.print("HOMING J2 backoff... ");
      } else if (homingStepCount > (unsigned long)MAX_FAST_STEPS) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J2");
        } else {
          stepper2.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J2_BACKOFF;
        }
      }
      break;

    case HOMING_J2_BACKOFF:
      stepper2.runSpeed();
      if (homingStepCount >= (unsigned long)BACKOFF_STEPS) {
        stepper2.setSpeed(-stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J2_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J2_SLOW:
      stepper2.runSpeed();
      if (digitalRead(PIN_J2_LIMIT) == HIGH) {
        stepper2.setCurrentPosition((long)(-90.0f * stepsPerDegGeared));
        commandedAngle2 = -90.0f;
        uint16_t raw = readAS5600(Wire);
        if (raw != 0xFFFF) enc2Offset = (raw / 4096.0f) * 360.0f - commandedAngle2;
        Serial.println("OK");
        homingRetries = 0; homingStepCount = 0;
        if (homingDoJ3) {
          stepper3.setSpeed(-stepsPerDegGeared * 20.0f);
          homingState = HOMING_J3_FAST;
          Serial.print("HOMING J3 fast... ");
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegGeared)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J2_SLOW");
      }
      break;

    // ── J3 FAST ─────────────────────────────────────────────────────────
    case HOMING_J3_FAST:
      stepper3.runSpeed();
      if (digitalRead(PIN_J3_LIMIT) == HIGH) {
        stepper3.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J3_BACKOFF;
        Serial.print("HOMING J3 backoff... ");
      } else if (homingStepCount > (unsigned long)MAX_FAST_STEPS) {
        if (++homingRetries >= MAX_HOME_RETRIES_NB) {
          homingState = HOMING_FAULT;
          Serial.println("ERR:HOME_FAULT_J3");
        } else {
          stepper3.setSpeed(stepsPerDegGeared * SLOW_SPD_DEG);
          homingStepCount = 0;
          homingState = HOMING_J3_BACKOFF;
        }
      }
      break;

    case HOMING_J3_BACKOFF:
      stepper3.runSpeed();
      if (homingStepCount >= (unsigned long)BACKOFF_STEPS) {
        stepper3.setSpeed(-stepsPerDegGeared * SLOW_SPD_DEG);
        homingStepCount = 0;
        homingState = HOMING_J3_SLOW;
        Serial.print("slow... ");
      }
      break;

    case HOMING_J3_SLOW:
      stepper3.runSpeed();
      if (digitalRead(PIN_J3_LIMIT) == HIGH) {
        stepper3.setCurrentPosition(j3AngleToSteps(-90.0f));
        commandedAngle3 = -90.0f;
        uint16_t raw = readAS5600(Wire1);
        if (raw != 0xFFFF) enc3Offset = (raw / 4096.0f) * 360.0f - commandedAngle3;
        Serial.println("OK");
        homingState = HOMING_COMPLETE;
      } else if (homingStepCount > (unsigned long)(10.0f * stepsPerDegGeared)) {
        homingState = HOMING_FAULT;
        Serial.println("ERR:HOME_FAULT_J3_SLOW");
      }
      break;

    case HOMING_COMPLETE:
      applySpeedOverride();
      servo4.write(90); servo5.write(90);
      moveToAngles(0.0f, -90.0f, 90.0f, 3000, true);
      sendACK();
      Serial.println("HOMING: Complete");
      homingState = HOMING_IDLE;
      break;

    case HOMING_FAULT:
      Serial.println("HOMING: FAULT - some axes failed");
      homingState = HOMING_IDLE;
      break;

    default:
      break;
  }
}

// ================= EEPROM HELPERS =================
bool eeprom_valid() {
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return (magic == EEPROM_MAGIC_NUMBER);
}

void eeprom_save_all() {
  uint32_t magic = EEPROM_MAGIC_NUMBER;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
  EEPROM.put(EEPROM_GEAR1_ADDR, gearboxRatioAxis1);
  EEPROM.put(EEPROM_GEAR2_ADDR, gearboxRatio);
  EEPROM.put(EEPROM_JNT_EN_ADDR,     (uint8_t)jointEnabled[0]);
  EEPROM.put(EEPROM_JNT_EN_ADDR + 1, (uint8_t)jointEnabled[1]);
  EEPROM.put(EEPROM_JNT_EN_ADDR + 2, (uint8_t)jointEnabled[2]);
  EEPROM.put(EEPROM_SPEED_OVR_ADDR, speedOverride);
  EEPROM.put(EEPROM_TOOL_OFF_ADDR,  toolOffset);
  EEPROM.put(EEPROM_WORK_OFF_ADDR,  workOffset);
}

void eeprom_load_all() {
  if (!eeprom_valid()) return;
  EEPROM.get(EEPROM_GEAR1_ADDR, gearboxRatioAxis1);
  EEPROM.get(EEPROM_GEAR2_ADDR, gearboxRatio);
  stepsPerDegAxis1   = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0;
  stepsPerDegGeared  = (STEPS_PER_MOTOR_REV * gearboxRatio)      / 360.0;
  uint8_t j0, j1, j2;
  EEPROM.get(EEPROM_JNT_EN_ADDR,     j0);
  EEPROM.get(EEPROM_JNT_EN_ADDR + 1, j1);
  EEPROM.get(EEPROM_JNT_EN_ADDR + 2, j2);
  jointEnabled[0] = (bool)j0;
  jointEnabled[1] = (bool)j1;
  jointEnabled[2] = (bool)j2;
  EEPROM.get(EEPROM_SPEED_OVR_ADDR, speedOverride);
  speedOverride = constrain(speedOverride, 0.05f, 1.0f);
  EEPROM.get(EEPROM_TOOL_OFF_ADDR,  toolOffset);
  EEPROM.get(EEPROM_WORK_OFF_ADDR,  workOffset);

  // Validate offsets to prevent NaN issues
  if (isnan(toolOffset.x) || isnan(toolOffset.y) || isnan(toolOffset.z))
    toolOffset = {0, 0, 0};
  if (isnan(workOffset.x) || isnan(workOffset.y) || isnan(workOffset.z))
    workOffset = {0, 0, 0};
}

// ================= FORWARD KINEMATICS =================
void forwardKinematics(float j1_deg, float j2_deg, float j3_deg,
                       float &x, float &y, float &z, bool raw) {
    float j1r = j1_deg * DEG_TO_RAD;
    float j2r = j2_deg * DEG_TO_RAD;
    float j3r = j3_deg * DEG_TO_RAD;

    // J3 position
    // j2=0° = pionowo w górę: reach=sin(j2), height=cos(j2)
    float j3_reach = L1 * sinf(j2r);
    float j3_z     = BASE_H + L1 * cosf(j2r);

    // Absolutny kąt segmentu J3→J4
    float ang34 = j2r + j3r;

    // L-shape: L2_PAR wzdłuż ang34, L2_PERP prostopadle
    // HOME verify: j2=-90,j3=+90 → ang34=0
    //   reach = -135 + 55*sin(0) + 135*cos(0) = 0 ✓
    //   z     =  160 + 55*cos(0) - 135*sin(0) = 215 ✓
    float j4_reach = j3_reach
                   + L2_PAR  * sinf(ang34)
                   + L2_PERP * cosf(ang34);
    float j4_z     = j3_z
                   + L2_PAR  * cosf(ang34)
                   - L2_PERP * sinf(ang34);

    x = j4_reach * cosf(j1r);
    y = j4_reach * sinf(j1r);
    z = j4_z;

    if (!raw) {
        // Apply inverse offsets to report position in Work Frame
        x += workOffset.x;  y += workOffset.y;  z += workOffset.z;
        x -= toolOffset.x;  y -= toolOffset.y;  z -= toolOffset.z;
    }
}

// ================= WORKSPACE CHECK =================
bool checkWorkspace(float j1, float j2, float j3, float j4, float j5) {

    // 1. Limity miękkie — ten sam zestaw co LIMIT_*
    if (j1 < LIMIT_J1_MIN || j1 > LIMIT_J1_MAX) return false;
    if (j2 < LIMIT_J2_MIN || j2 > LIMIT_J2_MAX) return false;
    if (j3 < LIMIT_J3_MIN || j3 > LIMIT_J3_MAX) return false;
    if (j4 < LIMIT_J4_MIN || j4 > LIMIT_J4_MAX) return false;
    if (j5 < LIMIT_J5_MIN || j5 > LIMIT_J5_MAX) return false;

    const float j1r = j1 * DEG_TO_RAD;
    const float j2r = j2 * DEG_TO_RAD;
    const float j3r = j3 * DEG_TO_RAD;

    float j3_reach = L1 * sinf(j2r);
    float j3_z     = BASE_H + L1 * cosf(j2r);
    float ang34    = j2r + j3r;
    float j4_reach = j3_reach
                   + L2_PAR  * sinf(ang34)
                   + L2_PERP * cosf(ang34);
    float j4_z     = j3_z
                   + L2_PAR  * cosf(ang34)
                   - L2_PERP * sinf(ang34);

    float j4_x = j4_reach * cosf(j1r);
    float j4_y = j4_reach * sinf(j1r);

    // 2. Strefa zakazu (zabezpieczenie bazy) -> ignoruj gdy TCP powiewa bezpiecznie ponad bazą (BASE_H)
    if ((j4_x * j4_x + j4_y * j4_y) < EXCL_R2 && j4_z < BASE_H) return false;

    // 3. Prześwit J3 i J4 nad podłożem
    if (j3_z < FLOOR_CLR) return false;
    if (j4_z < FLOOR_CLR) return false;

    // 4. L-shape nie uderza w kolumnę bazy
    if (j2 > 60.0f && j3 < -50.0f) return false;

    return true;
}

// ================= AR4-FORMAT RESPONSES =================
void sendRobotPos() {
  // Compute current FK position
  float cx, cy, cz;
  forwardKinematics(commandedAngle1, commandedAngle2, commandedAngle3, cx, cy, cz);

  Serial.print("A"); Serial.print(commandedAngle1, 3);
  Serial.print("B"); Serial.print(commandedAngle2, 3);
  Serial.print("C"); Serial.print(commandedAngle3, 3);
  Serial.print("D"); Serial.print(servo4.read());
  Serial.print("E"); Serial.print(servo5.read());
  Serial.print("F0");           // J6 placeholder
  Serial.print("G"); Serial.print(cx, 2);
  Serial.print("H"); Serial.print(cy, 2);
  Serial.print("I"); Serial.println(cz, 2);
}

void sendACK() {
  Serial.println("OK");
}

void sendError(const char *code) {
  Serial.println(code);
}

// ================= ENCODER COLLISION DETECTION =================
void checkEncoders() {
  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;

  // J1 — Base (Wire2)
  if (encEnabled[0]) {
    uint16_t raw = readAS5600(Wire2);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc1Offset);
      float error  = fabsf(commandedAngle1 - actual);
      if (error > COLLISION_THRESH_DEG) {
        J1collisionTrue = 1;
        Serial.print("COLLISION J1: err=");
        Serial.print(error, 1);
        Serial.println("deg");
      }
    }
  }

  // J2 — we have a real encoder
  if (encEnabled[1]) {
    uint16_t raw = readAS5600(Wire);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc2Offset);
      float error  = fabsf(commandedAngle2 - actual);
      if (error > COLLISION_THRESH_DEG) {
        J2collisionTrue = 1;
        Serial.print("COLLISION J2: err=");
        Serial.print(error, 1);
        Serial.println("deg");
      }
    }
  }

  // J3 — Elbow (Wire1)
  if (encEnabled[2]) {
    uint16_t raw = readAS5600(Wire1);
    if (raw != 0xFFFF) {
      float actual = getEncoderAngle(raw, enc3Offset);
      float error  = fabsf(commandedAngle3 - actual);
      if (error > COLLISION_THRESH_DEG) {
        J3collisionTrue = 1;
        Serial.print("COLLISION J3: err=");
        Serial.print(error, 1);
        Serial.println("deg");
      }
    }
  }

  // Build collision flag string (AR4 format: "EC000")
  int total = J1collisionTrue + J2collisionTrue + J3collisionTrue;
  if (total > 0) {
    String flag = "EC";
    flag += String(J1collisionTrue);
    flag += String(J2collisionTrue);
    flag += String(J3collisionTrue);
    Serial.println(flag);
    if (!estopActive) {
      Serial.println("ERR: COLLISION DETECTED - Triggering ESTOP");
      parseCommand("ESTOP", 0);
    }
  }
}

// ================= IHOLD CURRENT REDUCTION =================
void applyHoldCurrent() {
  if (holdCurrentActive) return;
  // irun stays at full for responsiveness; ihold drops to ~25%
  driver1.ihold(24);   driver1.irun(31);
  driver2.ihold(24);   driver2.irun(31);
  driver3.ihold(24);   driver3.irun(31);
  holdCurrentActive = true;
}

void applyRunCurrent() {
  if (!holdCurrentActive) return;
  driver1.ihold(16);  driver1.irun(31);
  driver2.ihold(16);  driver2.irun(31);
  driver3.ihold(16);  driver3.irun(31);
  holdCurrentActive = false;
}

void applySpeedOverride() {
  // Main axes (J1, J2, J3) are now controlled by the Cubic Spline Interpolator.
  // Their AccelStepper limits remain unconstrained so they can track the spline instantly.
  // We only apply the override to the gantry steppers here.
  stepperX.setMaxSpeed(500.0f * speedOverride);
  stepperY.setMaxSpeed(500.0f * speedOverride);
}

// ================= SD CARD INITIALIZATION =================
bool initSD_local() {
  if (sd_ready) return true;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("ERR: SD init failed");
    return false;
  }
  sd_ready = true;
  return true;
}

// ================= SD CARD — poprawiony format z J4/J5 =================
void savePosition(String name) {
    if (!initSD_local()) return;
    String filename = "/" + name + ".pos";
    if (SD.exists(filename.c_str())) {
        SD.remove(filename.c_str());
    }
    File f = SD.open(filename.c_str(), FILE_WRITE);
    if (!f) { sendError("ERR: SD open fail"); return; }
    f.print(commandedAngle1, 4); f.print(",");
    f.print(commandedAngle2, 4); f.print(",");
    f.print(commandedAngle3, 4); f.print(",");
    f.print((float)servo4.read(), 1); f.print(",");
    f.print((float)servo5.read(), 1); f.print(",");
    f.println("1000");
    f.close();
    Serial.print("SAVED: "); Serial.println(filename);
    sendACK();
}

void loadPosition(String name) {
    if (!initSD_local()) return;
    String filename = "/" + name + ".pos";
    File f = SD.open(filename.c_str(), FILE_READ);
    if (!f) { sendError("ERR: File not found"); return; }
    String line = f.readStringUntil('\n');
    f.close();
    line.trim();

    // Format: j1,j2,j3,j4,j5,duration  (6 pól)
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1+1);
    int i3 = line.indexOf(',', i2+1);
    int i4 = line.indexOf(',', i3+1);
    int i5 = line.indexOf(',', i4+1);

    if (i1 == -1 || i2 == -1 || i3 == -1) {
        sendError("ERR: File format invalid");
        return;
    }

    float j1 = line.substring(0, i1).toFloat();
    float j2 = line.substring(i1+1, i2).toFloat();
    float j3 = line.substring(i2+1, i3).toFloat();

    if (i4 != -1 && i5 != -1) {
        // nowy format z J4/J5
        float j4 = line.substring(i3+1, i4).toFloat();
        float j5 = line.substring(i4+1, i5).toFloat();
        unsigned long dur = line.substring(i5+1).toInt();
        float j4_c = constrain(j4, LIMIT_J4_MIN, LIMIT_J4_MAX);
        float j5_c = constrain(j5, LIMIT_J5_MIN, LIMIT_J5_MAX);
        moveToAngles(j1, j2, j3, dur, false, j4_c, j5_c);
    } else {
        // stary format bez J4/J5 (backward compat)
        unsigned long dur = line.substring(i3+1).toInt();
        moveToAngles(j1, j2, j3, dur);
    }

    Serial.print("LOADED: "); Serial.println(filename);
    sendACK();
}

void runSequence(String name) {
  if (!initSD_local()) return;
  if (isPlayingSequence) {
    currentSequenceFile.close();
  }
  String filename = "/" + name + ".seq";
  currentSequenceFile = SD.open(filename.c_str(), FILE_READ);
  if (!currentSequenceFile) { sendError("ERR: Sequence not found"); return; }
  isPlayingSequence = true;
  Serial.print("RUN: "); Serial.println(filename);
  sendACK();
}

void updateSequenceQueue() {
  if (!isPlayingSequence) return;
  
  while (currentSequenceFile.available()) {
    if (((qHead + 1) % WAYPOINT_QUEUE_SIZE) == qTail) {
      return; // Queue is full, yield to loop()
    }
    
    String line = currentSequenceFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.charAt(0) == '#') continue;
    
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1+1);
    int i3 = line.indexOf(',', i2+1);
    int i4 = line.indexOf(',', i3+1);
    int i5 = line.indexOf(',', i4+1);
    // Simple validation gracefully ignores bad lines
    if(i1 == -1 || i2 == -1 || i3 == -1 || i4 == -1 || i5 == -1) continue;

    float j1 = line.substring(0, i1).toFloat();
    float j2 = line.substring(i1+1, i2).toFloat();
    float j3 = line.substring(i2+1, i3).toFloat();
    float j4 = line.substring(i3+1, i4).toFloat();
    float j5 = line.substring(i4+1, i5).toFloat();
    unsigned long dur = line.substring(i5+1).toInt();
    
    // Set servos
    servo4.write((int)constrain(j4, LIMIT_J4_MIN, LIMIT_J4_MAX));
    servo5.write((int)constrain(j5, LIMIT_J5_MIN, LIMIT_J5_MAX));
    
    // moveToAngles now enqueues the coordinate safely
    moveToAngles(j1, j2, j3, dur);
  }
  
  currentSequenceFile.close();
  isPlayingSequence = false;
  Serial.println("Sequence Finished");
}

// ================= HOMING WITH LIMIT SWITCHES =================
void homeAxes(bool doJ1, bool doJ2, bool doJ3) {
  if (estopActive) {
    Serial.println("ERR: Cannot home while ESTOP is active");
    return;
  }

  Serial.println("HOMING: Starting...");

  const float HOME_FAST_DEG_S = 20.0;   // 20 degrees/sec
  const float HOME_SLOW_DEG_S =  2.0;   //  2 degrees/sec
  const float BACKOFF_DEG = 5.0;        // always means 5° regardless of gearing
  const int   MAX_STEPS_J1 = (int)(180.0f * stepsPerDegAxis1);   // full J1 range
  const int   MAX_STEPS_J23 = (int)(200.0f * stepsPerDegGeared);  // a bit over J2 range
  const uint8_t MAX_HOME_RETRIES = 3;   // maximum homing attempts per axis

  // Convert to steps/sec properly per axis
  float fastStepsPerSec1 = HOME_FAST_DEG_S * stepsPerDegAxis1;
  float slowStepsPerSec1 = HOME_SLOW_DEG_S * stepsPerDegAxis1;
  float fastStepsPerSecGeared = HOME_FAST_DEG_S * stepsPerDegGeared;
  float slowStepsPerSecGeared = HOME_SLOW_DEG_S * stepsPerDegGeared;

  // Convert backoff to steps per axis
  long backoff1 = (long)(BACKOFF_DEG * stepsPerDegAxis1);
  long backoffGeared = (long)(BACKOFF_DEG * stepsPerDegGeared);

  // Track homing status for safety dependencies
  bool j1Homed = false;
  bool j2Homed = false;
  bool j3Homed = false;

  // Helper lambda — drive one stepper until its limit pin goes HIGH
  auto driveToLimit = [](AccelStepper &s, int limitPin, int dir, float stepsPerSec, int maxSt) -> bool {
    s.setSpeed(dir * stepsPerSec);
    for (int i = 0; i < maxSt; i++) {
      if (digitalRead(limitPin) == HIGH) return true;  // switch triggered (goes HIGH when pressed)
      s.runSpeed();
      myusb.Task();
      delayMicroseconds(50);  // small delay to prevent overwhelming the stepper
    }
    return false;  // timed out
  };

  // ── J1 ─────────────────────────────────────────────────────────────────
  if (doJ1) {
    uint8_t j1Retries = 0;
    
    while (!j1Homed && j1Retries < MAX_HOME_RETRIES) {
      Serial.print("HOMING J1 fast... ");
      stepper1.setMaxSpeed(20000.0);
      bool hit = driveToLimit(stepper1, PIN_J1_LIMIT, -1, fastStepsPerSec1, MAX_STEPS_J1);
      if (!hit) {
        j1Retries++;
        // Back off before retrying so we get a clean run-up
        for (int i = 0; i < backoff1; i++) {
          stepper1.setSpeed(slowStepsPerSec1);
          stepper1.runSpeed();
          delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSec1));
        }
        Serial.print("missed (retry ");
        Serial.print(j1Retries);
        Serial.println(")");
        if (j1Retries >= MAX_HOME_RETRIES) {
          Serial.println("ERR:HOME_FAULT_J1");
          break;
        }
      } else {
        // back off
        for (int i = 0; i < backoff1; i++) {
          stepper1.setSpeed(slowStepsPerSec1);
          stepper1.runSpeed();
          delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSec1));
        }
        Serial.print("slow... ");
        driveToLimit(stepper1, PIN_J1_LIMIT, -1, slowStepsPerSec1, backoff1 * 2);
        stepper1.setCurrentPosition(0);
        commandedAngle1 = 0;

        // Re-calibrate encoder offset to match
        if (encEnabled[0]) {
          uint16_t raw = readAS5600(Wire2);
          if (raw != 0xFFFF) {
            enc1Offset = (raw / 4096.0f) * 360.0f - commandedAngle1;
          }
        }

        j1Homed = true;
        Serial.println("OK");
      }
    }
  }

  // ── J2 ─────────────────────────────────────────────────────────────────
  if (doJ2) {
    // Check if prerequisite axis (J1) is homed
    if (doJ1 && !j1Homed) {
      Serial.println("ERR: J2 home skipped, J1 not homed");
    } else {
      uint8_t j2Retries = 0;
      
      while (!j2Homed && j2Retries < MAX_HOME_RETRIES) {
        Serial.print("HOMING J2 fast... ");
        stepper2.setMaxSpeed(35000.0);
        bool hit = driveToLimit(stepper2, PIN_J2_LIMIT, -1, fastStepsPerSecGeared, MAX_STEPS_J23);
        if (!hit) {
          j2Retries++;
          // Back off before retrying so we get a clean run-up
          for (int i = 0; i < backoffGeared; i++) {
            stepper2.setSpeed(slowStepsPerSecGeared);
            stepper2.runSpeed();
            delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSecGeared));
          }
          Serial.print("missed (retry ");
          Serial.print(j2Retries);
          Serial.println(")");
          if (j2Retries >= MAX_HOME_RETRIES) {
            Serial.println("ERR:HOME_FAULT_J2");
            break;
          }
        } else {
          for (int i = 0; i < backoffGeared; i++) {
            stepper2.setSpeed(slowStepsPerSecGeared);
            stepper2.runSpeed();
            delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSecGeared));
          }
          Serial.print("slow... ");
          driveToLimit(stepper2, PIN_J2_LIMIT, -1, slowStepsPerSecGeared, backoffGeared * 2);
          stepper2.setCurrentPosition((long)(-90.0f * stepsPerDegGeared));
          commandedAngle2 = -90.0f;

          // Re-calibrate encoder offset to match
          uint16_t raw = readAS5600(Wire);
          if (raw != 0xFFFF) {
            enc2Offset = (raw / 4096.0f) * 360.0f - commandedAngle2;
          }

          j2Homed = true;
          Serial.println("OK");
        }
      }
    }
  }

  // ── J3 ─────────────────────────────────────────────────────────────────
  if (doJ3) {
    // Check if prerequisite axis (J2) is homed
    if (doJ2 && !j2Homed) {
      Serial.println("ERR: J3 home skipped, J2 not homed");
    } else {
      uint8_t j3Retries = 0;
      
      while (!j3Homed && j3Retries < MAX_HOME_RETRIES) {
        Serial.print("HOMING J3 fast... ");
        stepper3.setMaxSpeed(25000.0);
        bool hit = driveToLimit(stepper3, PIN_J3_LIMIT, -1, fastStepsPerSecGeared, MAX_STEPS_J23);
        if (!hit) {
          j3Retries++;
          // Back off before retrying so we get a clean run-up
          for (int i = 0; i < backoffGeared; i++) {
            stepper3.setSpeed(slowStepsPerSecGeared);
            stepper3.runSpeed();
            delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSecGeared));
          }
          Serial.print("missed (retry ");
          Serial.print(j3Retries);
          Serial.println(")");
          if (j3Retries >= MAX_HOME_RETRIES) {
            Serial.println("ERR:HOME_FAULT_J3");
            break;
          }
        } else {
          for (int i = 0; i < backoffGeared; i++) {
            stepper3.setSpeed(slowStepsPerSecGeared);
            stepper3.runSpeed();
            delayMicroseconds((unsigned long)(1000000.0f / slowStepsPerSecGeared));
          }
          Serial.print("slow... ");
          driveToLimit(stepper3, PIN_J3_LIMIT, -1, slowStepsPerSecGeared, backoffGeared * 2);
          stepper3.setCurrentPosition(j3AngleToSteps(-90.0f));
          commandedAngle3 = -90.0f;  // home J3 = -90°

          // Re-calibrate encoder offset to match
          uint16_t raw = readAS5600(Wire1);
          if (raw != 0xFFFF) {
            enc3Offset = (raw / 4096.0f) * 360.0f - commandedAngle3;
          }

          j3Homed = true;
          Serial.println("OK");
        }
      }
    }
  }

  // Only proceed with normal operation if all requested axes homed successfully
  bool allHomed = true;
  if (doJ1 && !j1Homed) allHomed = false;
  if (doJ2 && !j2Homed) allHomed = false;
  if (doJ3 && !j3Homed) allHomed = false;

  if (allHomed) {
    qHead = 0; qTail = 0; isMovingQueue = false;
    applySpeedOverride();  // restore override-scaled speeds
    // przejście do pozycji startowej (home position)
    servo4.write(90);
    servo5.write(90);
    moveToAngles(0.0f, -90.0f, 90.0f, 3000, true);  // pozycja gotowości
    sendACK();
  } else {
    Serial.println("HOMING: Incomplete - some axes failed to home");
  }
}

// ================= HOMING GANTRY =================
void homeGantry(bool doX, bool doY) {
  if (estopActive) {
    Serial.println("ERR: Cannot home while ESTOP is active");
    return;
  }

  Serial.println("HOMING GANTRY: Starting...");
  digitalWrite(PIN_MICRO_EN, LOW); // Enable drivers
  myusb.Task(); // Keep alive

  auto driveGantryLim = [](AccelStepper &s, int limitPin, float stepsPerSec, int maxSt) -> bool {
    s.setSpeed(stepsPerSec);
    for (int i = 0; i < maxSt; i++) {
      if (digitalRead(limitPin) == HIGH) return true;  // switch triggered (goes HIGH when pressed)
      s.runSpeed();
      delayMicroseconds(100);
    }
    return false;  // timed out
  };

  const float FAST_SPD = -500.0f; // negative towards switch
  const float SLOW_SPD = -100.0f;
  const long  BOUNCE   = 200;     // relative steps away
  const int   MAX_X_STEPS = 10000;
  const int   MAX_Y_STEPS = 10000;

  if (doX) {
    Serial.print("X... ");
    stepperX.setMaxSpeed(1000);
    // Move to limit
    if (driveGantryLim(stepperX, PIN_MICRO_X_LIMIT, FAST_SPD, MAX_X_STEPS)) {
       // Back off
       stepperX.setSpeed(-FAST_SPD);
       for(int i=0; i<BOUNCE; i++) { stepperX.runSpeed(); delayMicroseconds(100); }
       // Slow hit
       driveGantryLim(stepperX, PIN_MICRO_X_LIMIT, SLOW_SPD, BOUNCE*2);
       stepperX.setCurrentPosition(0);
       Serial.println("OK");
    } else {
       Serial.println("ERR: TIMEOUT");
    }
  }

  if (doY) {
    Serial.print("Y... ");
    stepperY.setMaxSpeed(1000);
    // Move to limit
    if (driveGantryLim(stepperY, PIN_MICRO_Y_LIMIT, FAST_SPD, MAX_Y_STEPS)) {
       // Back off
       stepperY.setSpeed(-FAST_SPD);
       for(int i=0; i<BOUNCE; i++) { stepperY.runSpeed(); delayMicroseconds(100); }
       // Slow hit
       driveGantryLim(stepperY, PIN_MICRO_Y_LIMIT, SLOW_SPD, BOUNCE*2);
       stepperY.setCurrentPosition(0);
       Serial.println("OK");
    } else {
       Serial.println("ERR: TIMEOUT");
    }
  }

  // Steppers will auto-disable via the timeout in loop() !
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // ESP32 Serial
  SERIAL_PORT.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
  myusb.begin(); // USB Host Init
  delay(100);
  while (!Serial && millis() < 3000)
    ;

  // ========== BANNER ==========
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║      5-DOF ROBOT ARM CONTROL SYSTEM               ║");
  Serial.println("║      Inverse Kinematics + Closed Loop             ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Build Date: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);
  Serial.println();

  // ========== PIN SETUP ==========
  Serial.println("━━━━ HARDWARE INITIALIZATION ━━━━");
  Serial.print("[1/5] Configuring GPIO pins... ");
  pinMode(PIN_AXIS1_STEP, OUTPUT);
  pinMode(PIN_AXIS1_DIR, OUTPUT);
  pinMode(PIN_AXIS1_EN, OUTPUT);
  pinMode(PIN_AXIS2_STEP, OUTPUT);
  pinMode(PIN_AXIS2_DIR, OUTPUT);
  pinMode(PIN_AXIS2_EN, OUTPUT);
  pinMode(PIN_AXIS3_STEP, OUTPUT);
  pinMode(PIN_AXIS3_DIR, OUTPUT);
  pinMode(PIN_AXIS3_EN, OUTPUT);
  pinMode(PIN_MICRO_X_STEP, OUTPUT);
  pinMode(PIN_MICRO_X_DIR, OUTPUT);
  pinMode(PIN_MICRO_Y_STEP, OUTPUT);
  pinMode(PIN_MICRO_Y_DIR, OUTPUT);
  pinMode(PIN_MICRO_EN, OUTPUT);
  digitalWrite(PIN_MICRO_EN, HIGH); // Disabled by default
  Serial.println("OK");

  // Limit switch pins
  pinMode(PIN_J1_LIMIT, INPUT_PULLUP);
  pinMode(PIN_J2_LIMIT, INPUT_PULLUP);
  pinMode(PIN_J3_LIMIT, INPUT_PULLUP);
  pinMode(PIN_MICRO_X_LIMIT, INPUT_PULLUP);
  pinMode(PIN_MICRO_Y_LIMIT, INPUT_PULLUP);

  // Enable Drivers
  digitalWrite(PIN_AXIS1_EN, LOW);
  digitalWrite(PIN_AXIS2_EN, LOW);
  digitalWrite(PIN_AXIS3_EN, LOW);

  // ========== TMC2209 DRIVERS ==========
  Serial.print("[2/5] Initializing TMC2209 Drivers... ");
  auto initDriver = [](TMC2209Stepper &d) {
    d.begin();
    delay(50);
    d.pdn_disable(true);
    delay(10);
    d.mstep_reg_select(true);
    delay(10);
    d.toff(5);
    delay(10);
    d.rms_current(1800);
    delay(10);
    d.microsteps(MICROSTEPS);
    delay(10);
    d.en_spreadCycle(false);
    d.pwm_autoscale(true);
    d.TPWMTHRS(300);
  };
  initDriver(driver1);
  initDriver(driver2);
  initDriver(driver3);
  Serial.println("OK");

  // ========== STEPPERS & SERVOS ==========
  Serial.print("[3/5] Configuring Steppers & Servos... ");
  // Main axes: instant-response position trackers for the Spline Interpolator.
  // Acceleration must be very high so AccelStepper has no ramp of its own
  // that could conflict with the spline's continuous position stream.
  stepper1.setMaxSpeed(40000.0);    stepper1.setAcceleration(4000000.0);
  stepper2.setMaxSpeed(80000.0);    stepper2.setAcceleration(8000000.0);
  stepper3.setMaxSpeed(60000.0);    stepper3.setAcceleration(6000000.0);

  stepperX.setMaxSpeed(500.0);
  stepperX.setAcceleration(500.0);
  stepperY.setMaxSpeed(500.0);
  stepperY.setAcceleration(500.0);

  servo4.attach(PIN_AXIS4_SERVO);
  servo4.write(10);  // Start at minimum safe angle
  servo5.attach(PIN_AXIS5_SERVO);
  servo5.write(10);  // Start at minimum safe angle
  Serial.println("OK");

  // ========== AS5600 ENCODERS ==========
  Serial.print("[4/5] Initializing AS5600 Encoders... ");

  // Wire2 → ENC1 (Base) (SDA=25, SCL=24) — Initialized but disabled by default
  Wire2.setSDA(25);
  Wire2.setSCL(24);
  Wire2.begin();
  Wire2.setClock(1000000); // 1MHz Fast Mode Plus
  // encEnabled[0] = false; // already set in global

  // Wire → ENC2 (Shoulder) (SDA=18, SCL=19)
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(1000000); // 1MHz Fast Mode Plus

  // Wire1 → ENC3 (Shoulder) (SDA=44, SCL=45) — Initialized but disabled by default
  Wire1.setSDA(44);
  Wire1.setSCL(45);
  Wire1.begin();
  Wire1.setClock(1000000); // 1MHz Fast Mode Plus
  // encEnabled[2] = false; // already set in global

  delay(50);
  Serial.println("OK");

  // ========== PID CONTROLLERS ==========
  // Axis 1: Base (High precision, no gravity)
  pid1.init(0.2, 0.0, 0.0, 0.1, 0.0);
  // Axis 2: Shoulder (Higher torque, gravity compensation)
  pid2.init(0.4, 0.0, 0.0, 0.1, 0.15);
  // Axis 3: Elbow (Increased from 0.3 to 0.4 to match J2)
  pid3.init(0.4, 0.0, 0.0, 0.1, 0.15);


  uint16_t enc1Raw = encEnabled[0] ? readAS5600(Wire2) : 0xFFFF;
  uint16_t enc2Raw = readAS5600(Wire);
  uint16_t enc3Raw = encEnabled[2] ? readAS5600(Wire1) : 0xFFFF;

  bool setupAnyError = false;
  if (encEnabled[0] && enc1Raw == 0xFFFF) setupAnyError = true;
  if (encEnabled[1] && enc2Raw == 0xFFFF) setupAnyError = true;
  if (encEnabled[2] && enc3Raw == 0xFFFF) setupAnyError = true;

  if (setupAnyError) {
    Serial.println("WARN");
    Serial.println("      \u26a0 One or more ENABLED encoders not responding!");
    Serial.println("      Check magnet alignment and I2C connections");
  } else {
    Serial.println("OK");
  }

  enc1Offset = (encEnabled[0] && enc1Raw != 0xFFFF) ? (enc1Raw / 4096.0) * 360.0 : 0;
  enc2Offset = (enc2Raw != 0xFFFF) ? (enc2Raw / 4096.0) * 360.0 : 0;
  enc3Offset = (encEnabled[2] && enc3Raw != 0xFFFF) ? (enc3Raw / 4096.0) * 360.0 : 0;

  Serial.print("      Encoder Offsets: ");
  Serial.print(encEnabled[0] ? String(enc1Offset, 1) + "\u00b0" : "N/A");
  Serial.print(" | ");
  Serial.print(String(enc2Offset, 1) + "\u00b0");
  Serial.print(" | ");
  Serial.print(encEnabled[2] ? String(enc3Offset, 1) + "\u00b0" : "N/A");
  Serial.println();

  // ========== COMMUNICATION CHECK ==========
  Serial.print("[5/5] Checking Communication Links... ");
  Serial.println();

  // USB Host
  Serial.print("      USB Host (Pro Micro): ");
  if (userial) {
    Serial.println("CONNECTED");
  } else {
    Serial.println("NOT DETECTED");
  }

  // ESP32 Serial
  Serial.print("      ESP32 (Serial1):      ");
  if (Serial1) {
    Serial.println("READY");
  } else {
    Serial.println("NOT READY");
  }

  Serial.println();

  // ========== SYSTEM HEALTH CHECK ==========
  Serial.println("━━━━ SYSTEM HEALTH CHECK ━━━━");
  printDriver("Axis 1", driver1);
  printDriver("Axis 2", driver2);
  printDriver("Axis 3", driver3);
  Serial.println();

  // ========== CONFIGURATION SUMMARY ==========
  Serial.println("━━━━ CONFIGURATION ━━━━");
  Serial.print("Robot Geometry:    L1=");
  Serial.print(L1, 0);
  Serial.print("mm, L2_PAR=");
  Serial.print(L2_PAR, 0);
  Serial.print("mm, L2_PERP=");
  Serial.print(L2_PERP, 0);
  Serial.println("mm");

  Serial.print("Axis 1 Gearing:    ");
  Serial.print(gearboxRatioAxis1, 2);
  Serial.print(":1 (");
  Serial.print(stepsPerDegAxis1, 2);
  Serial.println(" steps/deg)");

  Serial.print("Axis 2/3 Gearing:  ");
  Serial.print(gearboxRatio, 1);
  Serial.print(":1 (");
  Serial.print(stepsPerDegGeared, 2);
  Serial.println(" steps/deg)");

  Serial.print("Microsteps:        ");
  Serial.println(MICROSTEPS);

  Serial.print("TMC Current:       1800mA");
  Serial.println();

  Serial.println("Safety Limits:");
  Serial.print("  J1: ");
  Serial.print(LIMIT_J1_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J1_MAX, 0);
  Serial.println("°");

  Serial.print("  J2: ");
  Serial.print(LIMIT_J2_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J2_MAX, 0);
  Serial.println("°");

  Serial.print("  J3: ");
  Serial.print(LIMIT_J3_MIN, 0);
  Serial.print("° to ");
  Serial.print(LIMIT_J3_MAX, 0);
  Serial.println("°");
  Serial.println();

  Serial.print("Closed Loop:       ");
  Serial.println(useEncoders ? "ENABLED" : "DISABLED");
  Serial.println();

  // ========== COMMAND REFERENCE ==========
  Serial.println("━━━━ AVAILABLE COMMANDS ━━━━");
  Serial.println();
  Serial.println("MOTION CONTROL:");
  Serial.println("  MOV X# Y# Z# [W<0-6>] [J4#] [J5#] [D#ms]");
  Serial.println("  MOVL X# Y# Z# [W<0-6>] [J4#] [J5#] [D#ms] - Linear Move");
  Serial.println("    W0=poziomo  W1=dol_0  W2=dol_90");
  Serial.println("    W3=pitch0   W4=pitch90  W5=pitch180  W6=manual");
  Serial.println("  D1 <degrees>              - Move Axis 1 (Base) relative");
  Serial.println(
      "  D2 <degrees>              - Move Axis 2 (Shoulder) relative");
  Serial.println("  D3 <degrees>              - Move Axis 3 (Elbow) relative");
  Serial.println("  DX <steps>                - Move Gantry X relative");
  Serial.println("  DY <steps>                - Move Gantry Y relative");
  Serial.println("  A4 <angle>                - Set Servo 4 position (0-180)");
  Serial.println("  A5 <angle>                - Set Servo 5 position (0-180)");
  Serial.println();
  Serial.println("CALIBRATION & SETUP:");
  Serial.println("  HOME [SWITCH|ENCODER|MANUAL] - Zero all stepper positions (default SWITCH)");
  Serial.println("  HOMEJ1/HOMEJ2/HOMEJ3     - Home individual axes (switches)");
  Serial.println(
      "  CAL                       - Calibrate encoders to current position");
  Serial.println("  CL_ON                     - Enable closed-loop control");
  Serial.println("  CL_OFF                    - Disable closed-loop control");
  Serial.println("  REENABLE                  - Recover drivers from ESTOP");
  Serial.println();
  Serial.println("TEACH & PLAYBACK:");
  Serial.println("  SAVE <name>               - Save position to SD card");
  Serial.println("  LOAD <name>               - Load and move to position");
  Serial.println("  RUN <name>                - Run sequence from SD card");
  Serial.println();
  Serial.println("DIAGNOSTICS:");
  Serial.println("  ENC                       - Read encoder positions");
  Serial.println("  DIAG                      - TMC driver diagnostics");
  Serial.println("  ENCCHECK                  - Manual collision detection");
  Serial.println();
  Serial.println("CONFIGURATION:");
  Serial.println("  MS <speed>                - Set Gantry max speed");
  Serial.println("  SPEED <percent>           - Set speed override (5-100%)");
  Serial.println("  SAVECFG                   - Save config to EEPROM");
  Serial.println();
  Serial.println("PACKET FORMAT (from controller):");
  Serial.println("  <pos1,pos2,pos3,pos4,dx,dy>");
  Serial.println();

  // ========== READY ==========
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║              SYSTEM READY                         ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Waiting for commands...");
  Serial.println();

  // Load persisted config from EEPROM
  eeprom_load_all();
  applySpeedOverride();

  // ihold default
  applyHoldCurrent();

  // SD card
  initSD_local();

  // Gripper servo
  gripper.attach(PIN_GRIPPER_SERVO);
  gripper.write(90);
  gripperAngle = 90.0f;

  // Watchdog — 8 second timeout; loop() must call watchdog.feed() every cycle
  WDT_timings_t wdtCfg;
  wdtCfg.trigger = 5;   // warning interrupt at 5 s
  wdtCfg.timeout = 8;   // hard reset at 8 s
  watchdog.begin(wdtCfg);

  Serial.println("Watchdog armed (8s timeout).");
}

// ================= LOOP =================
void loop() {
  unsigned long loopStart = micros();
  watchdog.feed();   // kick watchdog every loop
  myusb.Task();      // USB Host task

  // run/hold current switching — also active during velocity-mode spline
  bool anyRunning = isMovingQueue ||
                    stepper1.isRunning() ||
                    stepper2.isRunning() ||
                    stepper3.isRunning();
  if (anyRunning) applyRunCurrent();
  else            applyHoldCurrent();

  // NOTE: stepper1/2/3 are driven further down in the loop in VELOCITY MODE
  // (runSpeed) or HOLD MODE (run), after the spline computes velocity.
  // Only gantry runs here.
  stepperX.run();
  stepperY.run();

  // --- AUTO-DISABLE GANTRY ---
  static unsigned long lastGantryMove = 0;
  if (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
      digitalWrite(PIN_MICRO_EN, LOW); // ENABLE (Active low)
      lastGantryMove = millis();
  } else {
      if (millis() - lastGantryMove > 500) {
          digitalWrite(PIN_MICRO_EN, HIGH); // DISABLE
      }
  }

  // Process Inputs
  unsigned long inputStart = micros();
  processInput(Serial, inputBuffer, inputBufferPos);
  if (userial)
    processInput(userial, usbBuffer, usbBufferPos);
  processInput(Serial1, espBuffer, espBufferPos);

  if (micros() - inputStart > MAX_INPUT_TIME_US) {
    Serial.println("WARN: Input processing slow");
  }

  // --- TELEMETRY STREAM (10Hz) ---
  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= 100) {
    lastTelem = millis();
    if (useTelemetry)     sendRobotPos();
    if (useJsonTelemetry) sendJSONTelemetry();
  }

  // Non-blocking homing state machine
  updateHoming();

  // Teach mode playback
  updateTeachPlayback();

  // --- WAYPOINT PROCESSING ---
  updateSequenceQueue();

  if (qHead != qTail && !isMovingQueue) {
    // ── Start Spline Segment ──
    isMovingQueue = true;
    Waypoint &wpTarget = motionQueue[qTail];
    
    // Base duration (requested by user/MOVL)
    float baseDuration = (float)wpTarget.duration_ms;
    if (baseDuration < 1.0f) baseDuration = 1.0f; // Prevent div zero

    // Safety Scaling: Calculate minimum physical time needed based on max speeds (in microseconds)
    float delta1 = fabsf(wpTarget.j1 - commandedAngle1) * stepsPerDegAxis1;
    float delta2 = fabsf(wpTarget.j2 - commandedAngle2) * stepsPerDegGeared;
    float delta3 = fabsf(wpTarget.j3 - commandedAngle3) * stepsPerDegGeared;
    
    // Use 40000, 80000, 60000 as the hardware absolute max limits (steps/s)
    float minTime1 = (delta1 / 40000.0f) * 1.5f * 1000000.0f;
    float minTime2 = (delta2 / 80000.0f) * 1.5f * 1000000.0f;
    float minTime3 = (delta3 / 60000.0f) * 1.5f * 1000000.0f;
    float minRequiredTime = max(minTime1, max(minTime2, minTime3));

    // Force safe duration (in microseconds)
    splineDuration = max(baseDuration * 1000.0f, minRequiredTime);

    // P1 (Start)
    sp1.j1 = commandedAngle1;
    sp1.j2 = commandedAngle2;
    sp1.j3 = commandedAngle3;
    sp1.j4 = servo4.read();
    sp1.j5 = servo5.read();

    // P2 (Target)
    sp2.j1 = wpTarget.j1;
    sp2.j2 = wpTarget.j2;
    sp2.j3 = wpTarget.j3;
    sp2.j4 = wpTarget.j4;
    sp2.j5 = wpTarget.j5;

    // V1 (Start Velocity)
    if (wasContinuous) {
      sv1 = calcTangent(spPrev, sp2);
    } else {
      sv1 = {0,0,0,0,0}; // Starting from rest
    }

    // V2 (End Velocity)
    uint8_t nextIdx = (qTail + 1) % WAYPOINT_QUEUE_SIZE;
    if (nextIdx != qHead) {
      // There is a next waypoint, motion is continuous
      Waypoint &wpNext = motionQueue[nextIdx];
      SplinePoint sp3 = {wpNext.j1, wpNext.j2, wpNext.j3, wpNext.j4, wpNext.j5};
      sv2 = calcTangent(sp1, sp3);
      wasContinuous = true;
    } else {
      // Last waypoint, stop here
      sv2 = {0,0,0,0,0};
      wasContinuous = false;
    }

    spPrev = sp1; // Save for next segment's V1
    splineT = 0.0f;
    lastSplineTime = micros(); // Use microsecond timer for sub-millisecond precision!
    applyRunCurrent();
  }

  if (isMovingQueue) {
    unsigned long now = micros();
    float dt_us = (float)(now - lastSplineTime); // microseconds
    lastSplineTime = now;

    // Advance splineT using microsecond dt for smooth sub-millisecond steps
    float currentOverride = max(0.05f, speedOverride);
    float realDuration_us = splineDuration / currentOverride;
    
    splineT += dt_us / realDuration_us;
    if (splineT >= 1.0f) splineT = 1.0f;

    float t = splineT;
    float t2 = t * t;
    float t3 = t2 * t;

    // Cubic Hermite basis functions
    float h1 =  2*t3 - 3*t2 + 1;
    float h2 = -2*t3 + 3*t2;
    float h3 =    t3 - 2*t2 + t;
    float h4 =    t3 -   t2;

    commandedAngle1 = h1*sp1.j1 + h2*sp2.j1 + h3*sv1.j1 + h4*sv2.j1;
    commandedAngle2 = h1*sp1.j2 + h2*sp2.j2 + h3*sv1.j2 + h4*sv2.j2;
    commandedAngle3 = h1*sp1.j3 + h2*sp2.j3 + h3*sv1.j3 + h4*sv2.j3;
    
    servo4.write(h1*sp1.j4 + h2*sp2.j4 + h3*sv1.j4 + h4*sv2.j4);
    servo5.write(h1*sp1.j5 + h2*sp2.j5 + h3*sv1.j5 + h4*sv2.j5);

    // End of segment
    if (splineT >= 1.0f) {
      qTail = (qTail + 1) % WAYPOINT_QUEUE_SIZE;
      isMovingQueue = false;
      if (qHead == qTail) {
        applySpeedOverride(); // restore default overrides
      }
    }
  }
  // ---------------------------

  // --- UNIFIED STEPPER UPDATE ---
  // Apply Spline position + Closed-Loop offset (pX)
  float p1 = useEncoders && jointEnabled[0] && encEnabled[0] ? pidAdj1 : 0.0f;
  float p2 = useEncoders && jointEnabled[1] && encEnabled[1] ? pidAdj2 : 0.0f;
  float p3 = useEncoders && jointEnabled[2] && encEnabled[2] ? pidAdj3 : 0.0f;

  if (isMovingQueue) {
    // ── VELOCITY MODE: feed the spline derivative directly to the motors──
    // Derivative of Cubic Hermite: dP/dt = dh1*P1 + dh2*P2 + dh3*V1 + dh4*V2
    // Divided by duration to convert from normalised t-space to degrees/second.
    float t  = splineT;
    float t2 = t * t;
    float currentOverride = max(0.05f, speedOverride);
    float realDuration_s  = (splineDuration / currentOverride) / 1000000.0f;
    if (realDuration_s < 0.001f) realDuration_s = 0.001f;

    float dh1 =  6.0f*t2 - 6.0f*t;
    float dh2 = -6.0f*t2 + 6.0f*t;
    float dh3 =  3.0f*t2 - 4.0f*t + 1.0f;
    float dh4 =  3.0f*t2 - 2.0f*t;

    // Instantaneous velocity in degrees/second for each joint
    float vel1 = (dh1*sp1.j1 + dh2*sp2.j1 + dh3*sv1.j1 + dh4*sv2.j1) / realDuration_s;
    float vel2 = (dh1*sp1.j2 + dh2*sp2.j2 + dh3*sv1.j2 + dh4*sv2.j2) / realDuration_s;
    float vel3 = (dh1*sp1.j3 + dh2*sp2.j3 + dh3*sv1.j3 + dh4*sv2.j3) / realDuration_s;

    // Convert to steps/s and clamp to hardware limits
    float spd1 = constrain(vel1 * stepsPerDegAxis1,  -40000.0f, 40000.0f);
    float spd2 = constrain(vel2 * stepsPerDegGeared, -80000.0f, 80000.0f);
    float spd3 = constrain(vel3 * stepsPerDegGeared * J3_STEP_SIGN, -60000.0f, 60000.0f);

    stepper1.setSpeed(spd1);
    stepper2.setSpeed(spd2);
    stepper3.setSpeed(spd3);

    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();

    // Keep commandedAngle up to date so hold target is correct when spline ends
    stepper1.setCurrentPosition((long)roundf((commandedAngle1 + p1) * stepsPerDegAxis1));
    stepper2.setCurrentPosition((long)roundf((commandedAngle2 + p2) * stepsPerDegGeared));
    stepper3.setCurrentPosition(j3AngleToSteps(commandedAngle3 + p3));

  } else {
    // ── HOLD / STREAM MODE ──

    // Stream mode: live slider dragging using a custom Critically Damped Spring.
    // This totally bypasses AccelStepper's internal moveTo() profiler which is
    // known to stutter when constantly interrupted with new moving targets.
    if (streamMode) {
      if (lastStreamMicros == 0) lastStreamMicros = micros();
      unsigned long nowUs = micros();
      float dt = (nowUs - lastStreamMicros) / 1000000.0f;
      lastStreamMicros = nowUs;
      if (dt > 0.05f) dt = 0.05f; // cap dt to 50ms if loop hangs

      // Spring physics parameters - tune these for feel!
      float stiffness = 6.0f;  // Higher = tracks slider tighter/faster. Lower = heavier, lazier feel.
      float maxVel = 20.0f;    // max speed in degrees/sec (very slow and safe)
      float maxAcc = 40.0f;    // max acceleration in degrees/sec^2

      auto updateSpring = [&](float target, float &pos, float &vel) {
          // Critically damped spring math: a = w^2 * err - 2*w*v
          float acc = (stiffness * stiffness * (target - pos)) - (2.0f * stiffness * vel);
          acc = constrain(acc, -maxAcc, maxAcc);
          vel += acc * dt;
          vel = constrain(vel, -maxVel, maxVel);
          pos += vel * dt;
      };

      updateSpring(streamTargetJ1, commandedAngle1, streamVelJ1);
      updateSpring(streamTargetJ2, commandedAngle2, streamVelJ2);
      updateSpring(streamTargetJ3, commandedAngle3, streamVelJ3);

      stepper1.setSpeed(streamVelJ1 * stepsPerDegAxis1);
      stepper2.setSpeed(streamVelJ2 * stepsPerDegGeared);
      stepper3.setSpeed(streamVelJ3 * stepsPerDegGeared * J3_STEP_SIGN);

      stepper1.runSpeed();
      stepper2.runSpeed();
      stepper3.runSpeed();

      stepper1.setCurrentPosition((long)roundf((commandedAngle1 + p1) * stepsPerDegAxis1));
      stepper2.setCurrentPosition((long)roundf((commandedAngle2 + p2) * stepsPerDegGeared));
      stepper3.setCurrentPosition(j3AngleToSteps(commandedAngle3 + p3));

      // Timeout check — only exit stream mode if slider stopped AND arm has settled
      bool isSettled = (fabsf(streamTargetJ1 - commandedAngle1) < 0.1f) &&
                       (fabsf(streamTargetJ2 - commandedAngle2) < 0.1f) &&
                       (fabsf(streamTargetJ3 - commandedAngle3) < 0.1f) &&
                       (fabsf(streamVelJ1) < 0.1f) &&
                       (fabsf(streamVelJ2) < 0.1f) &&
                       (fabsf(streamVelJ3) < 0.1f);

      if ((millis() - streamLastUpdateMs > STREAM_TIMEOUT_MS) && isSettled) {
        streamMode = false;
        lastStreamMicros = 0;
        commandedAngle1 = streamTargetJ1;
        commandedAngle2 = streamTargetJ2;
        commandedAngle3 = streamTargetJ3;
      }
    } else {
      // ── HOLD MODE: AccelStepper position control to stay at commanded angle ──
      long tgt1 = (long)roundf((commandedAngle1 + p1) * stepsPerDegAxis1);
      long tgt2 = (long)roundf((commandedAngle2 + p2) * stepsPerDegGeared);
      long tgt3 = j3AngleToSteps(commandedAngle3 + p3);

      if (tgt1 != stepper1.targetPosition()) stepper1.moveTo(tgt1);
      if (tgt2 != stepper2.targetPosition()) stepper2.moveTo(tgt2);
      if (tgt3 != stepper3.targetPosition()) stepper3.moveTo(tgt3);

      stepper1.run();
      stepper2.run();
      stepper3.run();
    }

  }
  // Closed Loop Correction
  updateClosedLoop();

  // ── NEW: post-move encoder collision check ───────────────────────
  static bool wasRunning = false;
  if (wasRunning && !anyRunning) {   // just stopped
    checkEncoders();
    sendRobotPos();                  // AR4-style position reply
  }
  wasRunning = anyRunning;

  // Loop Timing Check
  if (micros() - loopStart > MAX_LOOP_TIME_US) {
    Serial.print("WARN: Loop overrun: ");
    Serial.println(micros() - loopStart);
  }
}

// ================= IK — Analytical 2-Link Planar =================
void calculateIK(float x, float y, float z, unsigned long duration,
                 int wristMode, float j4_manual_deg, float j5_deg, bool isMovL) {
    // Apply work-object and tool offsets
    x -= workOffset.x;  y -= workOffset.y;  z -= workOffset.z;
    x += toolOffset.x;  y += toolOffset.y;  z += toolOffset.z;

    Serial.print("IK Target: X="); Serial.print(x);
    Serial.print(" Y="); Serial.print(y);
    Serial.print(" Z="); Serial.println(z);

    // 1. J1 — rotation in XY plane
    float theta1 = atan2(y, x);
    float deg1 = theta1 * RAD_TO_DEG;

    // 2. Horizontal reach from J1 axis
    float r = sqrtf(x*x + y*y);

    bool was_clamped = false;
    // Floor collision clamp
    if (z < FLOOR_CLR) {
        z = FLOOR_CLR;
        was_clamped = true;
    }
    // Base exclusion zone clamp
    if (r*r < EXCL_R2 && z < BASE_H) {
        r = sqrtf(EXCL_R2);
        was_clamped = true;
    }

    float z_rel = z - BASE_H;

    // 3. Analytical 2-link planar IK for J2, J3
    float L2_eff = sqrtf(L2_PAR*L2_PAR + L2_PERP*L2_PERP);
    float gamma = atan2f(L2_PERP, L2_PAR); // Phase shift for L-shape
    
    float D2 = r*r + z_rel*z_rel;
    float C3 = (D2 - L1*L1 - L2_eff*L2_eff) / (2.0f * L1 * L2_eff);
    
    // Out of reach clamp
    if (C3 > 1.0f) { C3 = 1.0f; was_clamped = true; }
    if (C3 < -1.0f) { C3 = -1.0f; was_clamped = true; }
    
    float j3_eff_pos = acosf(C3);
    float j3_eff_neg = -j3_eff_pos;
    
    float alpha_pos = atan2f(L2_eff * sinf(j3_eff_pos), L1 + L2_eff * C3);
    float alpha_neg = atan2f(L2_eff * sinf(j3_eff_neg), L1 + L2_eff * C3);
    
    float beta = atan2f(r, z_rel);
    
    float j2_1_deg = (beta - alpha_pos) * RAD_TO_DEG;
    float j3_1_deg = (j3_eff_pos - gamma) * RAD_TO_DEG;
    float j2_2_deg = (beta - alpha_neg) * RAD_TO_DEG;
    float j3_2_deg = (j3_eff_neg - gamma) * RAD_TO_DEG;

    // Joint limit clamping
    if (deg1 < LIMIT_J1_MIN) { deg1 = LIMIT_J1_MIN; was_clamped = true; }
    if (deg1 > LIMIT_J1_MAX) { deg1 = LIMIT_J1_MAX; was_clamped = true; }

    auto clampJ = [](float j, float minJ, float maxJ, bool &clamped) {
        if (j < minJ) { clamped = true; return minJ; }
        if (j > maxJ) { clamped = true; return maxJ; }
        return j;
    };

    bool sol1_clamped = was_clamped;
    float c_j2_1 = clampJ(j2_1_deg, LIMIT_J2_MIN, LIMIT_J2_MAX, sol1_clamped);
    float c_j3_1 = clampJ(j3_1_deg, LIMIT_J3_MIN, LIMIT_J3_MAX, sol1_clamped);

    bool sol2_clamped = was_clamped;
    float c_j2_2 = clampJ(j2_2_deg, LIMIT_J2_MIN, LIMIT_J2_MAX, sol2_clamped);
    float c_j3_2 = clampJ(j3_2_deg, LIMIT_J3_MIN, LIMIT_J3_MAX, sol2_clamped);
    
    bool sol1_valid = checkWorkspace(deg1, c_j2_1, c_j3_1, 90.0f, 90.0f);
    bool sol2_valid = checkWorkspace(deg1, c_j2_2, c_j3_2, 90.0f, 90.0f);
    
    float best_j2 = 0, best_j3 = 0;
    bool found_solution = false;
    
    // Choose best valid solution. Prefer the one minimizing total joint displacement
    if (sol1_valid && sol2_valid) {
        float d1 = abs(c_j2_1 - commandedAngle2) + abs(c_j3_1 - commandedAngle3);
        float d2 = abs(c_j2_2 - commandedAngle2) + abs(c_j3_2 - commandedAngle3);
        if (d1 < d2) {
            best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        } else {
            best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        }
        found_solution = true;
    } else if (sol1_valid) {
        best_j2 = c_j2_1; best_j3 = c_j3_1; was_clamped = sol1_clamped;
        found_solution = true;
    } else if (sol2_valid) {
        best_j2 = c_j2_2; best_j3 = c_j3_2; was_clamped = sol2_clamped;
        found_solution = true;
    }
    
    if (!found_solution) {
        Serial.println("ERR:WORKSPACE_VIOLATION");
        return;
    }

    if (was_clamped) {
        Serial.println("WARN: Out of workspace - clamped to closest valid");
    }

    Serial.print("IK Solution: J1="); Serial.print(deg1,2);
    Serial.print(" J2="); Serial.print(best_j2,2);
    Serial.print(" J3="); Serial.println(best_j3,2);

    // -- Wrist ------------------------------------------------------------
    // ang34_deg = absolutny kat koncowki ramienia (0deg=gora, 90deg=poziomo w przod)
    // servo4_angle = pitch_target - ang34_deg + 90deg
    // bo serwo w srodku zakresu (90deg) = tool w kierunku ramienia J3
    float ang34_deg = best_j2 + best_j3;
    float servo4_angle, servo5_angle;

    switch (wristMode) {
        case WRIST_LEVEL:     // tool poziomo, J5 srodek
            servo4_angle = 0.0f   - ang34_deg + 90.0f;
            servo5_angle = 90.0f;
            break;
        case WRIST_DOWN_0:    // tool w dol, J5 srodek
            servo4_angle = 90.0f  - ang34_deg + 90.0f;
            servo5_angle = 90.0f;
            break;
        case WRIST_DOWN_90:   // tool w dol, J5 obrocony
            servo4_angle = 90.0f  - ang34_deg + 90.0f;
            servo5_angle = 180.0f;
            break;
        case WRIST_PITCH_0:   // staly pitch 0deg, J5 z parametru
            servo4_angle = 0.0f   - ang34_deg + 90.0f;
            servo5_angle = j5_deg;
            break;
        case WRIST_PITCH_90:  // staly pitch 90deg, J5 z parametru
            servo4_angle = 90.0f  - ang34_deg + 90.0f;
            servo5_angle = j5_deg;
            break;
        case WRIST_PITCH_180: // staly pitch 180deg, J5 z parametru
            servo4_angle = 180.0f - ang34_deg + 90.0f;
            servo5_angle = j5_deg;
            break;
        case WRIST_MANUAL:
        default:
            servo4_angle = j4_manual_deg;
            servo5_angle = j5_deg;
            break;
    }

    servo4_angle = constrain(servo4_angle, LIMIT_J4_MIN, LIMIT_J4_MAX);
    servo5_angle = constrain(servo5_angle, LIMIT_J5_MIN, LIMIT_J5_MAX);

    Serial.print("  Wrist W"); Serial.print(wristMode);
    Serial.print(" J4="); Serial.print(servo4_angle,1);
    Serial.print("deg J5="); Serial.print(servo5_angle,1); Serial.println("deg");

    moveToAngles(deg1, best_j2, best_j3, duration, false, servo4_angle, servo5_angle, isMovL);
}

void moveToAngles(float t1, float t2, float t3, unsigned long duration_ms,
                  bool skipWorkspaceCheck, float t4, float t5, bool isMovL) {
    t1 = constrain(t1, LIMIT_J1_MIN, LIMIT_J1_MAX);
    t2 = constrain(t2, LIMIT_J2_MIN, LIMIT_J2_MAX);
    t3 = constrain(t3, LIMIT_J3_MIN, LIMIT_J3_MAX);

    if (!skipWorkspaceCheck) {
        if (!checkWorkspace(t1, t2, t3, 90.0f, 90.0f)) {
            Serial.println("ERR:WORKSPACE_VIOLATION");
            return;
        }
    }

    if (duration_ms == 0) {
        // D0 indicates live slider streaming. Bypass the queue entirely.
        qHead = 0; qTail = 0; isMovingQueue = false; wasContinuous = false;
        streamTargetJ1 = t1;
        streamTargetJ2 = t2;
        streamTargetJ3 = t3;
        streamMode = true;
        streamLastUpdateMs = millis();
        
        if (t4 >= 0) servo4.write((int)t4);
        if (t5 >= 0) servo5.write((int)t5);
        applySpeedOverride();
        return;
    }

    // Any standard queued waypoint MUST cancel live stream mode, otherwise
    // the arm will return to the last slider position after the waypoint finishes!
    streamMode = false;

    uint8_t nextHead = (qHead + 1) % WAYPOINT_QUEUE_SIZE;
    if (nextHead == qTail) {
        Serial.println("WARN: Waypoint queue full");
        return;
    }

    // Queue depth guard for slider/streaming use-case:
    // If there are already 2+ pending waypoints, replace the last queued
    // item instead of adding a new one. This prevents the arm from
    // accumulating a backlog of stale slider positions.
    uint8_t queueDepth = (qHead - qTail + WAYPOINT_QUEUE_SIZE) % WAYPOINT_QUEUE_SIZE;
    if (!isMovL && queueDepth >= 2) {
        // Overwrite the most recently queued waypoint (one before qHead)
        uint8_t lastQueued = (qHead - 1 + WAYPOINT_QUEUE_SIZE) % WAYPOINT_QUEUE_SIZE;
        if (t4 < 0) t4 = motionQueue[lastQueued].j4;
        if (t5 < 0) t5 = motionQueue[lastQueued].j5;
        motionQueue[lastQueued] = { t1, t2, t3, t4, t5, duration_ms };
        return;  // qHead stays the same — no new entry added
    }

    if (t4 < 0) t4 = (float)servo4.read();
    if (t5 < 0) t5 = (float)servo5.read();

    motionQueue[qHead] = { t1, t2, t3, t4, t5, duration_ms };
    qHead = nextHead;
}

// ================= SIMPLE CONTROL COMMANDS =================
void parseCommand(String command, float value) {
  if (command == "MOV") {
    // Validation handled in upper layer or assumed here for IK
    // Note: MOV command usually handled by direct parsing in processInput for
    // validation But here it seems calculateIK is called directly. To strictly
    // validate MOV X Y Z, I'd need to parse inside processInput. For now,
    // retaining existing logic but adding guard in case.
    return;
  }

  else if (command == "D1") {
    if (!jointEnabled[0]) { Serial.println("WARN: J1 disabled"); return; }
    // Streaming mode: compute absolute target from current position
    float req = commandedAngle1 + value;
    req = constrain(req, LIMIT_J1_MIN, LIMIT_J1_MAX);
    streamTargetJ1 = req;
    streamTargetJ2 = commandedAngle2;
    streamTargetJ3 = commandedAngle3;
    streamMode = true;
    streamLastUpdateMs = millis();
    // Also stop any queued spline so stream takes immediate effect
    qHead = qTail; isMovingQueue = false; wasContinuous = false;
  } else if (command == "D2") {
    if (!jointEnabled[1]) { Serial.println("WARN: J2 disabled"); return; }
    float req = commandedAngle2 + value;
    req = constrain(req, LIMIT_J2_MIN, LIMIT_J2_MAX);
    streamTargetJ1 = commandedAngle1;
    streamTargetJ2 = req;
    streamTargetJ3 = commandedAngle3;
    streamMode = true;
    streamLastUpdateMs = millis();
    qHead = qTail; isMovingQueue = false; wasContinuous = false;
  } else if (command == "D3") {
    if (!jointEnabled[2]) { Serial.println("WARN: J3 disabled"); return; }
    float req = commandedAngle3 + value;
    req = constrain(req, LIMIT_J3_MIN, LIMIT_J3_MAX);
    streamTargetJ1 = commandedAngle1;
    streamTargetJ2 = commandedAngle2;
    streamTargetJ3 = req;
    streamMode = true;
    streamLastUpdateMs = millis();
    qHead = qTail; isMovingQueue = false; wasContinuous = false;
  } else if (command == "DX") {
    stepperX.move((long)value);
  } else if (command == "DY") {
    stepperY.move((long)value);
  } else if (command == "A4") {
    servo4.write((int)constrain(value, LIMIT_J4_MIN, LIMIT_J4_MAX));
  } else if (command == "A5") {
    servo5.write((int)constrain(value, LIMIT_J5_MIN, LIMIT_J5_MAX));
  } else if (command == "MS") {
    stepperX.setMaxSpeed(value);
    stepperY.setMaxSpeed(value);
  } else if (command == "HOMEJ1") {
    startHoming(true, false, false);
  } else if (command == "HOMEJ2") {
    startHoming(false, true, false);
  } else if (command == "HOMEJ3") {
    startHoming(false, false, true);
  } else if (command.startsWith("HOME")) {
    String method = "SWITCH";
    int sp = command.indexOf(' ');
    if (sp != -1) method = command.substring(sp + 1).trim();
    if (method == "SWITCH") {
      startHoming(true, true, true);  // non-blocking
    } else if (method == "MICRO" || method == "GANTRY") {
      homeGantry(true, true);
    } else if (method == "ENCODER" || method == "MANUAL") {
      // For encoder/manual, assume current position is the HOME position (0, -90, 90)
      stepper1.setCurrentPosition(0); stepper1.moveTo(0);
      stepper2.setCurrentPosition((long)(-90.0f * stepsPerDegGeared)); stepper2.moveTo((long)(-90.0f * stepsPerDegGeared));
      stepper3.setCurrentPosition(j3AngleToSteps(90.0f)); stepper3.moveTo(j3AngleToSteps(90.0f));
      
      servo4.write(90);
      servo5.write(90);
      
      commandedAngle1 = 0.0f;
      commandedAngle2 = -90.0f;
      commandedAngle3 = 90.0f;
      if (method == "ENCODER") {
        // Update encoder offsets assuming current is home
        uint16_t r1 = readAS5600(Wire2);
        if (r1 != 0xFFFF) enc1Offset = (r1 / 4096.0) * 360.0 - commandedAngle1;
        uint16_t r2 = readAS5600(Wire);
        if (r2 != 0xFFFF) enc2Offset = (r2 / 4096.0) * 360.0 - commandedAngle2;
        uint16_t r3 = readAS5600(Wire1);
        if (r3 != 0xFFFF) enc3Offset = (r3 / 4096.0) * 360.0 - commandedAngle3;
      }
      qHead = 0; qTail = 0; isMovingQueue = false;
      applySpeedOverride();
      sendACK();
      Serial.println("HOMING: Manual/Encoder - positions set to home");
    } else {
      Serial.println("ERR: Unknown HOME method (use SWITCH, ENCODER, or MANUAL)");
    }
  } else if (command == "CAL") {
    uint16_t r1 = readAS5600(Wire2);
    if (r1 != 0xFFFF) enc1Offset = (r1 / 4096.0) * 360.0 - commandedAngle1;
    uint16_t r2 = readAS5600(Wire);
    if (r2 != 0xFFFF) enc2Offset = (r2 / 4096.0) * 360.0 - commandedAngle2;
    uint16_t r3 = readAS5600(Wire1);
    if (r3 != 0xFFFF) enc3Offset = (r3 / 4096.0) * 360.0 - commandedAngle3;
    
    Serial.println("CALIBRATED: Encoder Offsets synced to current commanded angles");
  } else if (command == "DIAG") {
    Serial.println("\n=== DIAGNOSTICS ===");
    printDriver("Axis 1", driver1);
    printDriver("Axis 2", driver2);
    printDriver("Axis 3", driver3);
  } else if (command == "ENC") {
    Serial.println("\n=== ENCODER POSITIONS ===");
    Serial.print("Encoder 1 (Base):     ");
    if (encEnabled[0]) Serial.println(readAS5600(Wire2)); else Serial.println("DISABLED");
    Serial.print("Encoder 2 (Shoulder): ");
    if (encEnabled[1]) Serial.println(readAS5600(Wire)); else Serial.println("DISABLED");
    Serial.print("Encoder 3 (Elbow):    ");
    if (encEnabled[2]) Serial.println(readAS5600(Wire1)); else Serial.println("DISABLED");
  } else if (command == "CL_ON") {
    useEncoders = true;
    manualCL = true;
    Serial.println("Closed Loop: ENABLED (manual)");
  } else if (command == "CL_OFF") {
    useEncoders = false;
    manualCL = false;
    Serial.println("Closed Loop: DISABLED");
  } else if (command == "ENC1") {
    encEnabled[0] = (value != 0);
    Serial.print("Encoder 1: ");
    Serial.println(encEnabled[0] ? "ENABLED" : "DISABLED");
  } else if (command == "ENC2") {
    encEnabled[1] = (value != 0);
    Serial.print("Encoder 2: ");
    Serial.println(encEnabled[1] ? "ENABLED" : "DISABLED");
  } else if (command == "ENC3") {
    encEnabled[2] = (value != 0);
    Serial.print("Encoder 3: ");
    Serial.println(encEnabled[2] ? "ENABLED" : "DISABLED");
  } else if (command == "JNT1") {
    jointEnabled[0] = (value != 0);
    Serial.print("Joint 1: ");
    Serial.println(jointEnabled[0] ? "ENABLED" : "DISABLED");
  } else if (command == "JNT2") {
    jointEnabled[1] = (value != 0);
    Serial.print("Joint 2: ");
    Serial.println(jointEnabled[1] ? "ENABLED" : "DISABLED");
  } else if (command == "JNT3") {
    jointEnabled[2] = (value != 0);
    Serial.print("Joint 3: ");
    Serial.println(jointEnabled[2] ? "ENABLED" : "DISABLED");
  } else if (command == "GEAR1") {
    gearboxRatioAxis1 = value;
    stepsPerDegAxis1 = (STEPS_PER_MOTOR_REV * gearboxRatioAxis1) / 360.0;
    Serial.print("Gearbox Ratio Axis 1: ");
    Serial.println(gearboxRatioAxis1, 2);
  } else if (command == "GEAR2") {
    gearboxRatio = value;
    stepsPerDegGeared = (STEPS_PER_MOTOR_REV * gearboxRatio) / 360.0;
    Serial.print("Gearbox Ratio Axis 2/3: ");
    Serial.println(gearboxRatio, 2);
  } else if (command == "TELEM") {
    useTelemetry = (value != 0);
    Serial.print("Telemetry: ");
    Serial.println(useTelemetry ? "ENABLED" : "DISABLED");
  } else if (command == "REENABLE") {
    digitalWrite(PIN_AXIS1_EN, LOW);
    digitalWrite(PIN_AXIS2_EN, LOW);
    digitalWrite(PIN_AXIS3_EN, LOW);
    applyHoldCurrent();
    estopActive = false;
    Serial.println("Drivers re-enabled");
  } else if (command == "ESTOP") {
    estopActive = true;
    // EMERGENCY STOP: Halt all motion immediately
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepperX.stop();
    stepperY.stop();

    // Physically disable drivers (release holding torque)
    digitalWrite(PIN_AXIS1_EN, HIGH);
    digitalWrite(PIN_AXIS2_EN, HIGH);
    digitalWrite(PIN_AXIS3_EN, HIGH);
    digitalWrite(PIN_MICRO_EN, HIGH);

    // Clear Queue
    qHead = 0;
    qTail = 0;
    isMovingQueue = false;
    Serial.println("!!! EMERGENCY STOP: MOTION HALTED, DRIVERS DISABLED !!!");
  }
  // ── SPEED OVERRIDE ──────────────────────────────────────────────────────
  else if (command == "SPEED") {
    speedOverride = constrain(value / 100.0f, 0.05f, 1.0f);
    applySpeedOverride();
    eeprom_save_all();
    Serial.print("Speed override: ");
    Serial.print(speedOverride * 100.0f, 0);
    Serial.println("%");
  }
  // ── POSITION QUERY (AR4 compatible) ─────────────────────────────────────
  else if (command == "RP") {
    sendRobotPos();
  }
  // ── ENCODER CHECK MANUAL TRIGGER ────────────────────────────────────────
  else if (command == "ENCCHECK") {
    checkEncoders();
  }
  // ── SAVE CONFIG TO EEPROM ───────────────────────────────────────────────
  else if (command == "SAVECFG") {
    eeprom_save_all();
    Serial.println("Config saved to EEPROM");
  }
  // ── JSON TELEMETRY ───────────────────────────────────────────────────────
  else if (command == "JTELEM") {
    useJsonTelemetry = (value != 0);
    Serial.print("JSON Telemetry: ");
    Serial.println(useJsonTelemetry ? "ENABLED" : "DISABLED");
  }
  // ── SOFT STOP (drain queue gracefully) ──────────────────────────────────
  else if (command == "STOP") {
    qHead = qTail;
    isMovingQueue = false;
    isTeachPlaying = false;
    isPlayingSequence = false;
    Serial.println("Motion stopped (queue cleared)");
    sendACK();
  }
  // ── STATUS DUMP ─────────────────────────────────────────────────────────
  else if (command == "STATUS") {
    sendJSONTelemetry();
  }
  // ── GRIPPER ─────────────────────────────────────────────────────────────
  else if (command == "GRIP") {
    // GRIP <angle>  |  GRIP OPEN  |  GRIP CLOSE
    // When called from parseCommand, 'value' is numeric (e.g. GRIP 45)
    // String forms (GRIP OPEN/CLOSE) handled in processInput
    setGripper(value);
    Serial.print("Gripper: "); Serial.println((int)gripperAngle);
  }
  // ── TOOL FRAME OFFSET ───────────────────────────────────────────────────
  else if (command == "TOOLX") { toolOffset.x = value; eeprom_save_all(); Serial.print("Tool X: "); Serial.println(value); }
  else if (command == "TOOLY") { toolOffset.y = value; eeprom_save_all(); Serial.print("Tool Y: "); Serial.println(value); }
  else if (command == "TOOLZ") { toolOffset.z = value; eeprom_save_all(); Serial.print("Tool Z: "); Serial.println(value); }
  // ── WORK FRAME OFFSET ───────────────────────────────────────────────────
  else if (command == "WORKX") { workOffset.x = value; eeprom_save_all(); Serial.print("Work X: "); Serial.println(value); }
  else if (command == "WORKY") { workOffset.y = value; eeprom_save_all(); Serial.print("Work Y: "); Serial.println(value); }
  else if (command == "WORKZ") { workOffset.z = value; eeprom_save_all(); Serial.print("Work Z: "); Serial.println(value); }
  // ── TEACH MODE ──────────────────────────────────────────────────────────
  else if (command == "TEACH") {
    // TEACH START | TEACH RECORD | TEACH STOP | TEACH PLAY | TEACH CLEAR
    // Numeric: TEACH 0=START, 1=RECORD, 2=STOP, 3=PLAY, 4=CLEAR
    int mode = (int)value;
    if (mode == 0) {  // START
      isTeachMode = true;
      teachCount = 0;
      teachPlayIdx = 0;
      Serial.println("TEACH: Recording started. Send TEACH 1 to record each point.");
    } else if (mode == 1) {  // RECORD
      if (!isTeachMode) { Serial.println("WARN: Not in teach mode (send TEACH 0 first)"); return; }
      if (teachCount >= MAX_TEACH_POINTS) { Serial.println("WARN: Teach buffer full"); return; }
      teachPoints[teachCount++] = { commandedAngle1, commandedAngle2, commandedAngle3,
                                    (float)servo4.read(), (float)servo5.read(), gripperAngle };
      Serial.print("TEACH: Point "); Serial.print(teachCount); Serial.println(" recorded");
    } else if (mode == 2) {  // STOP
      isTeachMode = false;
      isTeachPlaying = false;
      Serial.print("TEACH: "); Serial.print(teachCount); Serial.println(" points stored");
    } else if (mode == 3) {  // PLAY
      if (teachCount == 0) { Serial.println("WARN: No teach points"); return; }
      isTeachPlaying = true;
      teachPlayIdx = 0;
      Serial.println("TEACH: Playback started");
    } else if (mode == 4) {  // CLEAR
      teachCount = 0;
      teachPlayIdx = 0;
      isTeachMode = false;
      isTeachPlaying = false;
      Serial.println("TEACH: Buffer cleared");
    }
  }
}

// ================= DRIVER DIAGNOSTICS =================
void printDriver(const char *name, TMC2209Stepper &driver) {
  Serial.print("--- ");
  Serial.print(name);
  Serial.print(" --- ");
  uint8_t ifcnt = driver.IFCNT();

  if (ifcnt > 0) {
    uint32_t status = driver.DRV_STATUS();
    Serial.println("Connection: OK");
    if (status & 0x01)       Serial.println("  [WARN] Over-Temperature Pre-Warning!");
    if (status & 0x02)       Serial.println("  [ERR]  Over-Temperature SHUTDOWN!");
    if (status & 0x04)       Serial.println("  [ERR]  Short to Ground (Phase A)!");
    if (status & 0x08)       Serial.println("  [ERR]  Short to Ground (Phase B)!");
    if (status & 0x10)       Serial.println("  [ERR]  Short to Supply (Phase A)!");
    if (status & 0x20)       Serial.println("  [ERR]  Short to Supply (Phase B)!");
    if (status & 0x40)       Serial.println("  [WARN] Open Load (Phase A) - Disconnected?");
    if (status & 0x80)       Serial.println("  [WARN] Open Load (Phase B) - Disconnected?");
    if (status & 0x80000000) Serial.println("  [INFO] Standstill Indicator");
  } else {
    Serial.println("FAIL (No UART Response)");
  }
}

// ================= HARDWARE I2C FOR AS5600 =================
uint16_t readAS5600(TwoWire &wire) {
  const uint8_t AS5600_ADDR = 0x36;
  const uint8_t AS5600_ANGLE_H = 0x0C;

  wire.beginTransmission(AS5600_ADDR);
  wire.write(AS5600_ANGLE_H);
  if (wire.endTransmission(false) != 0)
    return 0xFFFF; // no ACK

  if (wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2)
    return 0xFFFF;

  uint8_t hi = wire.read();
  uint8_t lo = wire.read();
  return ((uint16_t)hi << 8) | lo;
}

// ================= INPUT PROCESSING (PC + USB) =================
void processInput(Stream &stream, char *buffer, uint8_t &pos) {
  while (stream.available() > 0) {
    char c = stream.read();

    // Packet Start
    if (c == '<') {
      pos = 0;
      buffer[pos] = 0; // Clear
      continue;
    }
    // Packet End
    else if (c == '>') {
      buffer[pos] = 0; // Null terminate
      parsePacket(buffer);
      pos = 0;
      continue;
    }
    // Newline (Legacy/IK Commands)
    else if (c == '\n') {
      buffer[pos] = 0; // Null terminate
      String bufStr = String(buffer);
      bufStr.trim();

      if (bufStr.length() > 0 && bufStr.indexOf(',') == -1) {
        bufStr.toUpperCase();

        // ── String command parsing for SAVE/LOAD/RUN ────────────────────
        if (bufStr.startsWith("SAVE ") || bufStr.startsWith("LOAD ") ||
            bufStr.startsWith("RUN ")  || bufStr.startsWith("HOME ")) {
          int sp = bufStr.indexOf(' ');
          String cmd  = bufStr.substring(0, sp);
          String name = bufStr.substring(sp + 1);
          name.trim();
          if      (cmd == "SAVE") savePosition(name);
          else if (cmd == "LOAD") loadPosition(name);
          else if (cmd == "RUN")  runSequence(name);
          else if (cmd == "HOME") parseCommand(bufStr, 0); // pass full "HOME METHOD" string
        }
        // ── GRIP OPEN / GRIP CLOSE / GRIP <angle> ──────────────────────────
        else if (bufStr.startsWith("GRIP")) {
          if      (bufStr == "GRIP OPEN")  setGripper(0.0f);
          else if (bufStr == "GRIP CLOSE") setGripper(180.0f);
          else if (bufStr.length() > 5)    setGripper(bufStr.substring(5).toFloat());
          Serial.print("Gripper: "); Serial.println((int)gripperAngle);
        }
        // ── TEACH START / RECORD / STOP / PLAY / CLEAR ─────────────────────
        else if (bufStr.startsWith("TEACH")) {
          String arg = bufStr.substring(5); arg.trim();
          if      (arg == "START")  parseCommand("TEACH", 0);
          else if (arg == "RECORD") parseCommand("TEACH", 1);
          else if (arg == "STOP")   parseCommand("TEACH", 2);
          else if (arg == "PLAY")   parseCommand("TEACH", 3);
          else if (arg == "CLEAR")  parseCommand("TEACH", 4);
          else if (arg.length() > 0 && isdigit(arg[0])) {
            parseCommand("TEACH", arg.toFloat());
          } else {
            Serial.println("ERR: TEACH [START|RECORD|STOP|PLAY|CLEAR] or TEACH <0-4>");
          }
        }
        // ── MOVL — Linear Cartesian Move ───────────────────────────────────
        else if (bufStr.startsWith("MOVL")) {
          int idxX  = bufStr.indexOf('X');
          int idxY  = bufStr.indexOf('Y');
          int idxZ  = bufStr.indexOf('Z');
          int idxW  = bufStr.indexOf('W');
          int idxJ4 = bufStr.indexOf(" J4");
          int idxJ5 = bufStr.indexOf(" J5");
          int idxD  = bufStr.indexOf('D');

          if (idxX != -1 && idxY != -1 && idxZ != -1 && idxY > idxX && idxZ > idxY) {
            int endZ = bufStr.length();
            if (idxW  != -1 && idxW  > idxZ) endZ = min(endZ, idxW);
            if (idxJ4 != -1 && idxJ4 > idxZ) endZ = min(endZ, idxJ4);
            if (idxD  != -1 && idxD  > idxZ) endZ = min(endZ, idxD);

            float x = bufStr.substring(idxX+1, idxY).toFloat();
            float y = bufStr.substring(idxY+1, idxZ).toFloat();
            float z = bufStr.substring(idxZ+1, endZ).toFloat();
            int   wMode = WRIST_LEVEL;
            float j4m = 90.0f, j5m = 90.0f;
            unsigned long dur = 2000;

            if (idxW  != -1) wMode = (int)bufStr.substring(idxW+1).toFloat();
            if (idxJ4 != -1) j4m   = bufStr.substring(idxJ4+3).toFloat();
            if (idxJ5 != -1) j5m   = bufStr.substring(idxJ5+3).toFloat();
            if (idxD  != -1) dur   = (unsigned long)bufStr.substring(idxD+1).toInt();

            movL(x, y, z, dur, wMode, j4m, j5m);
          } else {
            Serial.println("ERR: Invalid MOVL format (Expect MOVL X# Y# Z#)");
          }
        }
        // ── MOV — Joint-interpolated IK move ──────────────────────────────
        else if (bufStr.startsWith("MOV")) {
          int idxX  = bufStr.indexOf('X');
          int idxY  = bufStr.indexOf('Y');
          int idxZ  = bufStr.indexOf('Z');
          int idxW  = bufStr.indexOf('W');
          int idxJ4 = bufStr.indexOf(" J4");
          int idxJ5 = bufStr.indexOf(" J5");
          int idxD  = bufStr.indexOf('D');

          if (idxX != -1 && idxY != -1 && idxZ != -1 && idxY > idxX && idxZ > idxY) {
            // Wyznacz koniec wartosci Z (pierwszy marker po Z)
            int endZ = bufStr.length();
            if (idxW  != -1 && idxW  > idxZ) endZ = min(endZ, idxW);
            if (idxJ4 != -1 && idxJ4 > idxZ) endZ = min(endZ, idxJ4);
            if (idxD  != -1 && idxD  > idxZ) endZ = min(endZ, idxD);

            float x = bufStr.substring(idxX+1, idxY).toFloat();
            float y = bufStr.substring(idxY+1, idxZ).toFloat();
            float z = bufStr.substring(idxZ+1, endZ).toFloat();

            int   wMode = WRIST_LEVEL;
            float j4m   = 90.0f;
            float j5m   = 90.0f;
            unsigned long dur = 1000;

            if (idxW  != -1) wMode = (int)bufStr.substring(idxW+1).toFloat();
            if (idxJ4 != -1) j4m   = bufStr.substring(idxJ4+3).toFloat();
            if (idxJ5 != -1) j5m   = bufStr.substring(idxJ5+3).toFloat();
            if (idxD  != -1) dur   = (unsigned long)bufStr.substring(idxD+1).toInt();

            calculateIK(x, y, z, dur, wMode, j4m, j5m);
          } else {
            Serial.println("ERR: Invalid MOV format (Expect MOV X# Y# Z#)");
          }
        }
        // Simple Control commands
        else {
          int spaceIndex = bufStr.indexOf(' ');
          String cmd =
              (spaceIndex == -1) ? bufStr : bufStr.substring(0, spaceIndex);
          float val = (spaceIndex == -1)
                          ? 0.0
                          : bufStr.substring(spaceIndex + 1).toFloat();
          parseCommand(cmd, val);
        }
      }
      pos = 0; // Reset
    }
    // Character
    else if (c != '\r') {
      if (pos < 127) {
        buffer[pos++] = c;
      } else {
        // Overflow - Reset or Warn
        // Serial.println("WARN: Buffer Overflow");
        pos = 0;
      }
    }
  }
}

// Parse Packet: "j1,j2,j3,j4,j5,dx,dy,duration"
void parsePacket(char* data) {
  if (estopActive) return;
  
  char* lastComma = strrchr(data, ',');
  if (!lastComma) {
    Serial.println("ERR: Malformed Packet (No CRC)");
    return;
  }

  *lastComma = '\0'; // Split payload and CRC
  uint8_t receivedCRC = (uint8_t)atoi(lastComma + 1);

  uint8_t calculatedCRC = 0;
  for (char* p = data; *p != '\0'; p++) {
    calculatedCRC ^= (uint8_t)(*p);
  }

  if (calculatedCRC != receivedCRC) {
    Serial.print("ERR: CRC Mismatch (Calc:");
    Serial.print(calculatedCRC);
    Serial.print(" Recv:");
    Serial.print(receivedCRC);
    Serial.println(")");
    return;
  }

  Serial.print("PKT (CRC OK): ");
  Serial.println(data);

  float values[8]; // j1, j2, j3, j4, j5, dx, dy, duration
  int idx = 0;
  
  char* token = strtok(data, ",");
  while (token != NULL && idx < 8) {
    values[idx++] = atof(token);
    token = strtok(NULL, ",");
  }

  // --- Retro-compatibility Fallback ---
  if (idx == 7) {
    values[7] = 1000; // Default duration for non-duration packets
    idx = 8;
  } else if (idx == 6) {
    values[6] = 0; // dx
    values[7] = 1000; // duration
    idx = 8;
  }

  // Packet Validation
  if (idx != 8) {
    Serial.println("ERR: Bad Packet format (Expect 8 values)");
    return;
  }

  // === POSITION CONTROL (Absolute) with Safety Limits ===
  float angle1 = constrain(values[0], LIMIT_J1_MIN, LIMIT_J1_MAX);
  float angle2 = constrain(values[1], LIMIT_J2_MIN, LIMIT_J2_MAX);
  float angle3 = constrain(values[2], LIMIT_J3_MIN, LIMIT_J3_MAX);
  unsigned long duration = (unsigned long)values[7];

  // Use smooth waypoint movement for primary joints and servos
  float j4 = constrain(values[3], LIMIT_J4_MIN, LIMIT_J4_MAX);
  float j5 = constrain(values[4], LIMIT_J5_MIN, LIMIT_J5_MAX);
  moveToAngles(angle1, angle2, angle3, duration, false, j4, j5);

  // Micro Manipulator (Relative/Incremental)
  if ((long)values[5] != 0) {
    stepperX.move((long)values[5]);
  }
  if ((long)values[6] != 0) {
    stepperY.move((long)values[6]);
  }
}

// ================= CLOSED LOOP CONTROL =================
// Helper to get normalized angle from encoder
float getEncoderAngle(uint16_t raw, float offset) {
  float rawAngle = (raw / 4096.0) * 360.0;
  float delta = rawAngle - offset;

  // Normalize to -180 to +180
  while (delta > 180.0)
    delta -= 360.0;
  while (delta < -180.0)
    delta += 360.0;

  return delta;
}

void updateClosedLoop() {
  if (!useEncoders)
    return;

  // Rate Limit: Run at 20Hz (50ms)
  static unsigned long lastLoop = 0;
  static uint8_t errorCount = 0;

  unsigned long now = millis();
  if (now - lastLoop < 50)
    return;
  float dt = (now - lastLoop) / 1000.0; // Time in seconds
  if (dt < 0.001f)
    return; // Skip if less than 1ms has passed (prevent div by zero)
  lastLoop = now;

  // Per-axis encoder read (skip disabled axes, use 0 as placeholder)
  // Base uses Wire2
  uint16_t enc1 = encEnabled[0] ? readAS5600(Wire2) : 0;
  // Shoulder uses Wire
  uint16_t enc2 = encEnabled[1] ? readAS5600(Wire) : 0;
  // Elbow (not connected yet, uses Wire1)
  uint16_t enc3 = encEnabled[2] ? readAS5600(Wire1) : 0;

  // Cache for telemetry
  lastEnc2 = enc2;
  lastEnc3 = enc3;

  // Error Handling (only check enabled encoders)
  bool anyError =
      (encEnabled[0] && enc1 == 0xFFFF) ||
      (encEnabled[1] && enc2 == 0xFFFF) ||
      (encEnabled[2] && enc3 == 0xFFFF);
  if (anyError) {
    if (manualCL) {
      Serial.print("WARN: Encoder error (ignored in manual CL) enc1=");
      Serial.print(enc1, HEX);
      Serial.print(" enc2=");
      Serial.print(enc2, HEX);
      Serial.print(" enc3=");
      Serial.println(enc3, HEX);
    } else {
      errorCount++;
      if (errorCount > MAX_ENCODER_FAILURES) {
        useEncoders = false;
        Serial.println("ERR: Encoders failed, disabling closed-loop");
      }
    }
    return;
  }
  errorCount = 0;

  // Lambda for correction with PID
  auto correctAxis = [&](uint16_t enc, float offset, PIDController &pid,
                         float commandedAngle, float prevCommandedAngle,
                         bool jEnabled, bool eEnabled) -> float {
    if (!jEnabled || !eEnabled)
      return 0.0f; // Skip disabled axes
    float actualAngle = getEncoderAngle(enc, offset);
    float error = commandedAngle - actualAngle; // Setpoint - ProcessVariable

    // Normalize Error
    while (error > 180.0)
      error -= 360.0;
    while (error < -180.0)
      error += 360.0;

    // Velocity Feedforward calculation
    float targetVelocity = (commandedAngle - prevCommandedAngle) / dt;

    // PID Calculation
    float adjustment = pid.calculate(error, dt, targetVelocity);

    // Gravity Compensation: pid.kG * cos(angle)
    float gravityComp = pid.kG * cos(commandedAngle * DEG_TO_RAD);
    return adjustment + gravityComp;
  };

  static float prevCommanded1 = 0, prevCommanded2 = 0, prevCommanded3 = 0;

  pidAdj1 = correctAxis(enc1, enc1Offset, pid1,
              commandedAngle1, prevCommanded1, jointEnabled[0], encEnabled[0]);
  pidAdj2 = correctAxis(enc2, enc2Offset, pid2,
              commandedAngle2, prevCommanded2, jointEnabled[1], encEnabled[1]);
  pidAdj3 = correctAxis(enc3, enc3Offset, pid3,
              commandedAngle3, prevCommanded3, jointEnabled[2], encEnabled[2]);

  prevCommanded1 = commandedAngle1;
  prevCommanded2 = commandedAngle2;
  prevCommanded3 = commandedAngle3;
}