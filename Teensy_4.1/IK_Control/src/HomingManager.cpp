#include "HomingManager.h"

HomingManager::HomingManager(MotorController& m1, MotorController& m2, MotorController& m3, Encoder& e1, Encoder& e2, Encoder& e3, MotionPlanner& planner, RobotConfig& config)
    : _m1(m1), _m2(m2), _m3(m3), _e1(e1), _e2(e2), _e3(e3), _planner(planner), _state(HOMING_IDLE) {
    _stepsPerDeg1 = (STEPS_PER_MOTOR_REV * config.gear1) / 360.0f;
    _stepsPerDegGeared = (STEPS_PER_MOTOR_REV * config.gear2) / 360.0f;
}

void HomingManager::start(bool doJ1, bool doJ2, bool doJ3) {
    _doJ1 = doJ1; _doJ2 = doJ2; _doJ3 = doJ3;
    _state = _doJ1 ? HOMING_J1_FAST : (_doJ2 ? HOMING_J2_FAST : (_doJ3 ? HOMING_J3_FAST : HOMING_IDLE));
    _stepCount = 0;
    _retries = 0;

    if (_state == HOMING_J1_FAST) _m1.setSpeed(-_stepsPerDeg1 * 20.0f);
    else if (_state == HOMING_J2_FAST) _m2.setSpeed(-_stepsPerDegGeared * 20.0f);
    else if (_state == HOMING_J3_FAST) _m3.setSpeed(-_stepsPerDegGeared * 20.0f);
}

void HomingManager::update() {
    if (_state == HOMING_IDLE) return;

    const float SLOW_SPD_DEG = 2.0f;
    _stepCount++;

    switch (_state) {
        case HOMING_J1_FAST:
            _m1.runSpeed();
            if (digitalRead(PIN_J1_LIMIT) == HIGH) {
                _m1.setSpeed(_stepsPerDeg1 * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J1_BACKOFF;
            }
            break;
        case HOMING_J1_BACKOFF:
            _m1.runSpeed();
            if (_stepCount > (uint32_t)(5.0f * _stepsPerDeg1)) {
                _m1.setSpeed(-_stepsPerDeg1 * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J1_SLOW;
            }
            break;
        case HOMING_J1_SLOW:
            _m1.runSpeed();
            if (digitalRead(PIN_J1_LIMIT) == HIGH) {
                _m1.setCurrentPosition(0);
                _state = _doJ2 ? HOMING_J2_FAST : (_doJ3 ? HOMING_J3_FAST : HOMING_COMPLETE);
                if (_state == HOMING_J2_FAST) _m2.setSpeed(-_stepsPerDegGeared * 20.0f);
                else if (_state == HOMING_J3_FAST) _m3.setSpeed(-_stepsPerDegGeared * 20.0f);
                _stepCount = 0;
            }
            break;

        case HOMING_J2_FAST:
            _m2.runSpeed();
            if (digitalRead(PIN_J2_LIMIT) == HIGH) {
                _m2.setSpeed(_stepsPerDegGeared * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J2_BACKOFF;
            }
            break;
        case HOMING_J2_BACKOFF:
            _m2.runSpeed();
            if (_stepCount > (uint32_t)(5.0f * _stepsPerDegGeared)) {
                _m2.setSpeed(-_stepsPerDegGeared * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J2_SLOW;
            }
            break;
        case HOMING_J2_SLOW:
            _m2.runSpeed();
            if (digitalRead(PIN_J2_LIMIT) == HIGH) {
                _m2.setCurrentPosition((long)(-90.0f * _stepsPerDegGeared));
                _state = _doJ3 ? HOMING_J3_FAST : HOMING_COMPLETE;
                if (_state == HOMING_J3_FAST) _m3.setSpeed(-_stepsPerDegGeared * 20.0f);
                _stepCount = 0;
            }
            break;

        case HOMING_J3_FAST:
            _m3.runSpeed();
            if (digitalRead(PIN_J3_LIMIT) == HIGH) {
                _m3.setSpeed(_stepsPerDegGeared * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J3_BACKOFF;
            }
            break;
        case HOMING_J3_BACKOFF:
            _m3.runSpeed();
            if (_stepCount > (uint32_t)(5.0f * _stepsPerDegGeared)) {
                _m3.setSpeed(-_stepsPerDegGeared * SLOW_SPD_DEG);
                _stepCount = 0;
                _state = HOMING_J3_SLOW;
            }
            break;
        case HOMING_J3_SLOW:
            _m3.runSpeed();
            if (digitalRead(PIN_J3_LIMIT) == HIGH) {
                _m3.setCurrentPosition((long)(90.0f * _stepsPerDegGeared * -1.0f));
                _state = HOMING_COMPLETE;
                _stepCount = 0;
            }
            break;

        case HOMING_COMPLETE:
            _planner.addWaypoint({{0, -90, 90, 90, 90}, 3000});
            _state = HOMING_IDLE;
            Serial.println("HOMING COMPLETE");
            break;
        default: break;
    }
}

bool HomingManager::isHoming() { return _state != HOMING_IDLE; }
