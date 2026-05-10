#include "ClosedLoopController.h"

ClosedLoopController::ClosedLoopController(Encoder& enc, MotorController& motor, PIDGains gains)
    : _enc(enc), _motor(motor), _gains(gains), _integral(0), _lastError(0), _offset(0), _enabled(false) {}

void ClosedLoopController::setEnabled(bool enabled) { _enabled = enabled; if (!enabled) _integral = 0; }
void ClosedLoopController::setOffset(float offset) { _offset = offset; }

float ClosedLoopController::update(float targetAngle, float targetVelocity, float dt) {
    if (!_enabled || dt <= 0) return 0;

    float actualAngle = _enc.getAngle(_offset);
    if (isnan(actualAngle)) return 0;

    float error = targetAngle - actualAngle;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    _integral += error * dt;
    if (abs(error) > 20.0f) _integral = 0; // Anti-windup
    _integral = constrain(_integral, -100.0f, 100.0f);

    float derivative = (error - _lastError) / dt;
    _lastError = error;

    float adjustment = (_gains.kP * error) + (_gains.kI * _integral) + (_gains.kD * derivative) + (_gains.kFF * targetVelocity);
    float gravityComp = _gains.kG * cosf(targetAngle * DEG_TO_RAD);

    return adjustment + gravityComp;
}
