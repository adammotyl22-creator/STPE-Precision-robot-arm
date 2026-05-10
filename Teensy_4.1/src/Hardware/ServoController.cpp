#include "ServoController.h"

ServoController::ServoController(uint8_t pin, float minAngle, float maxAngle)
    : _pin(pin), _minAngle(minAngle), _maxAngle(maxAngle) {}

void ServoController::begin() {
    _servo.attach(_pin);
}

void ServoController::write(float angle) {
    float constrainedAngle = constrain(angle, _minAngle, _maxAngle);
    _servo.write((int)constrainedAngle);
}

float ServoController::read() {
    return (float)_servo.read();
}
