#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ─── Robot Geometry ──────────────────────────────────────────────────────────
#define BASE_H    160.0f
#define L1        135.0f
#define L2_PAR     55.0f   // along J3 arm direction
#define L2_PERP   135.0f   // perpendicular (forearm extension)
#define L3         25.0f   // J4→J5 wrist link (mm)
#define EXCL_R2   6400.0f  // r < 80mm exclusion zone (80² = 6400)
#define FLOOR_CLR  20.0f   // min clearance above floor

// ─── Pin Definitions ─────────────────────────────────────────────────────────
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

// Servos
const int PIN_AXIS4_SERVO = 23;
const int PIN_AXIS5_SERVO = 22;
const int PIN_GRIPPER_SERVO = 21;

// Limit switches
const int PIN_J1_LIMIT = 5;
const int PIN_J2_LIMIT = 4;
const int PIN_J3_LIMIT = 36;

// ─── Stepper Settings ───────────────────────────────────────────────────────
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const float STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPS; // 3200

// ─── Safety Limits ──────────────────────────────────────────────────────────
const float LIMIT_J1_MIN = -85.0f, LIMIT_J1_MAX =  85.0f;
const float LIMIT_J2_MIN = -95.0f, LIMIT_J2_MAX =  85.0f;
const float LIMIT_J3_MIN = -85.0f, LIMIT_J3_MAX =  95.0f;
const float LIMIT_J4_MIN =   0.0f, LIMIT_J4_MAX = 180.0f;
const float LIMIT_J5_MIN =   0.0f, LIMIT_J5_MAX = 180.0f;

// ─── EEPROM memory map ─────────────────────────────────────────────────────
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_GEAR1_ADDR      4
#define EEPROM_GEAR2_ADDR      8
#define EEPROM_JNT_EN_ADDR    12
#define EEPROM_SPEED_OVR_ADDR 15
#define EEPROM_TOOL_OFF_ADDR  19
#define EEPROM_WORK_OFF_ADDR  31
#define EEPROM_MAGIC_NUMBER   0x494B3101

#endif
