#include "SafetyManager.h"

SafetyManager::SafetyManager(MotorController& m1, MotorController& m2, MotorController& m3)
    : _m1(m1), _m2(m2), _m3(m3), _estopActive(false) {}

void SafetyManager::update() {
    if (_estopActive) return;
    _checkStallGuard();
}

void SafetyManager::_checkStallGuard() {
    // StallGuard monitoring. SG_RESULT is 0 at stop/low speed, so we check movement.
    bool moving = _m1.isRunning() || _m2.isRunning() || _m3.isRunning();
    if (moving) {
        if ((_m1.isRunning() && _m1.getStallResult() == 0) ||
            (_m2.isRunning() && _m2.getStallResult() == 0) ||
            (_m3.isRunning() && _m3.getStallResult() == 0)) {
            triggerEstop();
            Serial.println("!!! STALL DETECTED - ESTOP TRIGGERED !!!");
        }
    }
}

bool SafetyManager::isSafe() {
    return !_estopActive;
}

void SafetyManager::triggerEstop() {
    _estopActive = true;
    _m1.enable(false);
    _m2.enable(false);
    _m3.enable(false);
}

void SafetyManager::resetEstop() {
    _estopActive = false;
    _m1.enable(true);
    _m2.enable(true);
    _m3.enable(true);
}

bool SafetyManager::estopActive() {
    return _estopActive;
}
