#include "MotorController.h"

MotorController::MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enPin, HardwareSerial* serial, float rsense, uint8_t addr)
    : _stepper(AccelStepper::DRIVER, stepPin, dirPin), _driver(serial, rsense, addr), _enPin(enPin) {
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH); // Start disabled
}

void MotorController::begin(uint16_t rms, uint16_t microsteps, uint32_t tpwmthrs) {
    _driver.begin();
    delay(10);
    _driver.pdn_disable(true);
    _driver.mstep_reg_select(true);
    _driver.toff(5);
    _driver.rms_current(rms);
    _driver.microsteps(microsteps);
    _driver.en_spreadCycle(false);
    _driver.pwm_autoscale(true);
    _driver.TPWMTHRS(tpwmthrs);
    _driver.TCOOLTHRS(0xFFFFF); // Enable CoolStep/StallGuard at all speeds
}

void MotorController::setSpeed(float stepsPerSec) {
    _stepper.setSpeed(stepsPerSec);
}

void MotorController::runSpeed() {
    _stepper.runSpeed();
}

void MotorController::run() {
    _stepper.run();
}

void MotorController::stop() {
    _stepper.stop();
}

void MotorController::setCurrentPosition(long position) {
    _stepper.setCurrentPosition(position);
}

long MotorController::targetPosition() {
    return _stepper.targetPosition();
}

void MotorController::moveTo(long position) {
    _stepper.moveTo(position);
}

bool MotorController::isRunning() {
    return _stepper.isRunning();
}

void MotorController::setMaxSpeed(float speed) {
    _stepper.setMaxSpeed(speed);
}

void MotorController::setAcceleration(float accel) {
    _stepper.setAcceleration(accel);
}

void MotorController::setHoldCurrent(uint8_t ihold) {
    _driver.ihold(ihold);
}

void MotorController::setRunCurrent(uint8_t irun) {
    _driver.irun(irun);
}

void MotorController::enableStallGuard(int16_t threshold) {
    _driver.SGTHRS(threshold);
}

uint32_t MotorController::getDrvStatus() {
    return _driver.DRV_STATUS();
}

uint16_t MotorController::getStallResult() {
    return _driver.SG_RESULT();
}

void MotorController::enable(bool en) {
    digitalWrite(_enPin, en ? LOW : HIGH);
}
