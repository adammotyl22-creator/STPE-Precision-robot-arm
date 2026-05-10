#include "SafetyManager.h"

SafetyManager::SafetyManager(MotorController& m1, MotorController& m2, MotorController& m3)
    : _m1(m1), _m2(m2), _m3(m3), _estopActive(false) {}

void SafetyManager::update() {
    if (_estopActive) return;
    _checkStallGuard();
}

void SafetyManager::_checkStallGuard() {
    // StallGuard threshold tuning is hardware dependent.
    // Typically, SG_RESULT decreases as load increases.
    // If SG_RESULT reaches 0, a stall is likely.
    if (_m1.getStallResult() == 0 || _m2.getStallResult() == 0 || _m3.getStallResult() == 0) {
        // Trigger ESTOP if any motor stalls
        triggerEstop();
        Serial.println("!!! STALL DETECTED - ESTOP TRIGGERED !!!");
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
