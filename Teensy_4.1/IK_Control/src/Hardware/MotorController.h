#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <AccelStepper.h>
#include <TMCStepper.h>
#include "../Config.h"

class MotorController {
public:
    MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, HardwareSerial* serial, float rsense, uint8_t addr);
    void begin(uint16_t rms, uint16_t microsteps, uint32_t tpwmthrs);
    void setSpeed(float stepsPerSec);
    void runSpeed();
    void run();
    void stop();
    void setCurrentPosition(long position);
    long targetPosition();
    void moveTo(long position);
    bool isRunning();
    void setMaxSpeed(float speed);
    void setAcceleration(float accel);

    // TMC specific
    void setHoldCurrent(uint8_t ihold);
    void setRunCurrent(uint8_t irun);
    void enableStallGuard(int16_t threshold);
    uint32_t getDrvStatus();
    uint16_t getStallResult();
    void enable(bool en);

private:
    AccelStepper _stepper;
    TMC2209Stepper _driver;
    uint8_t _enPin;
};

#endif
