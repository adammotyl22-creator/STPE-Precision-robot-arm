#include <Arduino.h>
#include <Wire.h>
#include <USBHost_t36.h>
#include <Watchdog_t4.h>
#include "../include/Config.h"
#include "../src/Hardware/MotorController.h"
#include "../src/Hardware/ServoController.h"
#include "../src/Hardware/Encoder.h"
#include "../src/Motion/MotionPlanner.h"
#include "../src/Motion/ClosedLoopController.h"
#include "../src/Motion/SequencePlayer.h"
#include "../src/Motion/TeachManager.h"
#include "../src/SafetyManager.h"
#include "../src/HomingManager.h"
#include "../src/Communication/CommandProcessor.h"
#include "../src/ConfigurationManager.h"

// Configuration
RobotConfig config;

// Watchdog
WDT_T4<WDT1> watchdog;

// USB Host
USBHost myusb;
USBHub hub1(myusb);
USBSerial userial(myusb);

// Hardware
MotorController axis1(PIN_AXIS1_STEP, PIN_AXIS1_DIR, PIN_AXIS1_EN, &Serial4, 0.11f, 0b00);
MotorController axis2(PIN_AXIS2_STEP, PIN_AXIS2_DIR, PIN_AXIS2_EN, &Serial4, 0.11f, 0b10);
MotorController axis3(PIN_AXIS3_STEP, PIN_AXIS3_DIR, PIN_AXIS3_EN, &Serial4, 0.11f, 0b01);

MotorController axisX(PIN_MICRO_X_STEP, PIN_MICRO_X_DIR, PIN_MICRO_EN, nullptr, 0, 0);
MotorController axisY(PIN_MICRO_Y_STEP, PIN_MICRO_Y_DIR, PIN_MICRO_EN, nullptr, 0, 0);

ServoController servo4(PIN_AXIS4_SERVO, LIMIT_J4_MIN, LIMIT_J4_MAX);
ServoController servo5(PIN_AXIS5_SERVO, LIMIT_J5_MIN, LIMIT_J5_MAX);
ServoController gripper(PIN_GRIPPER_SERVO, 0, 180);

Encoder enc1(Wire2);
Encoder enc2(Wire);
Encoder enc3(Wire1);

// Systems
MotionPlanner planner;
SafetyManager safety(axis1, axis2, axis3);
SequencePlayer seqPlayer(planner);
TeachManager teachMgr(planner);
HomingManager homingMgr(axis1, axis2, axis3, enc1, enc2, enc3, planner, config);
CommandProcessor cmdProcessor(planner, safety, seqPlayer, teachMgr, homingMgr);

// Closed Loop Controllers
PIDGains gains1 = {0.2, 0.0, 0.0, 0.1, 0.0};
PIDGains gains2 = {0.4, 0.0, 0.0, 0.1, 0.15};
PIDGains gains3 = {0.4, 0.0, 0.0, 0.1, 0.15};

ClosedLoopController cl1(enc1, axis1, gains1);
ClosedLoopController cl2(enc2, axis2, gains2);
ClosedLoopController cl3(enc3, axis3, gains3);

unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial4.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
    
    myusb.begin();
    ConfigurationManager::load(config);
    
    // I2C Setup
    Wire.setSDA(18); Wire.setSCL(19); Wire.begin(); Wire.setClock(1000000);
    Wire1.setSDA(44); Wire1.setSCL(45); Wire1.begin(); Wire1.setClock(1000000);
    Wire2.setSDA(25); Wire2.setSCL(24); Wire2.begin(); Wire2.setClock(1000000);

    // Motor Initialization
    axis1.begin(1800, MICROSTEPS, 300);
    axis2.begin(1800, MICROSTEPS, 300);
    axis3.begin(1800, MICROSTEPS, 300);
    
    axis1.setMaxSpeed(40000); axis1.setAcceleration(4000000);
    axis2.setMaxSpeed(80000); axis2.setAcceleration(8000000);
    axis3.setMaxSpeed(60000); axis3.setAcceleration(6000000);

    axisX.setMaxSpeed(500); axisX.setAcceleration(500);
    axisY.setMaxSpeed(500); axisY.setAcceleration(500);
    
    servo4.begin();
    servo5.begin();
    gripper.begin();
    
    // Safety
    axis1.enableStallGuard(10);
    axis2.enableStallGuard(10);
    axis3.enableStallGuard(10);
    
    safety.resetEstop();
    
    // Closed Loop
    cl1.setEnabled(true);
    cl2.setEnabled(true);
    cl3.setEnabled(true);

    // Watchdog Setup
    WDT_timings_t wdtCfg;
    wdtCfg.trigger = 5;
    wdtCfg.timeout = 8;
    watchdog.begin(wdtCfg);
    
    Serial.println("Robot Arm Modular Control v4.5 Ready");
    lastUpdate = micros();
}

void loop() {
    watchdog.feed();
    myusb.Task();
    
    unsigned long now = micros();
    float dt = (float)(now - lastUpdate) / 1000000.0f;
    lastUpdate = now;

    safety.update();
    
    if (safety.isSafe()) {
        if (homingMgr.isHoming()) {
            homingMgr.update();
        } else {
            planner.update();
            seqPlayer.update();
            teachMgr.update();

            Joints target = planner.getCurrentJoints();
            Joints vel = planner.getCurrentVelocities();

            // Closed Loop Correction
            float adj1 = cl1.update(target.j1, vel.j1, dt);
            float adj2 = cl2.update(target.j2, vel.j2, dt);
            float adj3 = cl3.update(target.j3, vel.j3, dt);

            // Convert angles to steps
            float stepsPerDeg1 = (STEPS_PER_MOTOR_REV * config.gear1) / 360.0f;
            float stepsPerDegGeared = (STEPS_PER_MOTOR_REV * config.gear2) / 360.0f;

            axis1.setSpeed(vel.j1 * stepsPerDeg1);
            axis2.setSpeed(vel.j2 * stepsPerDegGeared);
            axis3.setSpeed(vel.j3 * stepsPerDegGeared * -1.0f);

            axis1.runSpeed();
            axis2.runSpeed();
            axis3.runSpeed();

            axisX.run();
            axisY.run();

            // Sync internal position for AccelStepper AFTER move
            if (!planner.isMoving()) {
                axis1.setCurrentPosition((target.j1 + adj1) * stepsPerDeg1);
                axis2.setCurrentPosition((target.j2 + adj2) * stepsPerDegGeared);
                axis3.setCurrentPosition((target.j3 + adj3) * stepsPerDegGeared * -1.0f);
            }

            servo4.write(target.j4);
            servo5.write(target.j5);
        }
    }
    
    cmdProcessor.processSerial(Serial);
    cmdProcessor.processSerial(Serial1);
    if (userial) cmdProcessor.processSerial(userial);
    
    static uint32_t lastTelem = 0;
    if (millis() - lastTelem > 100) {
        cmdProcessor.sendTelemetry(Serial);
        lastTelem = millis();
    }
}
