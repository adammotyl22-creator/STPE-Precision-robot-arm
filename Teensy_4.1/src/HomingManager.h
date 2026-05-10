#ifndef HOMING_MANAGER_H
#define HOMING_MANAGER_H

#include <Arduino.h>
#include "Hardware/MotorController.h"
#include "Hardware/Encoder.h"
#include "Motion/MotionPlanner.h"

enum HomingState {
  HOMING_IDLE,
  HOMING_J1_FAST, HOMING_J1_BACKOFF, HOMING_J1_SLOW,
  HOMING_J2_FAST, HOMING_J2_BACKOFF, HOMING_J2_SLOW,
  HOMING_J3_FAST, HOMING_J3_BACKOFF, HOMING_J3_SLOW,
  HOMING_COMPLETE, HOMING_FAULT
};

#include "ConfigurationManager.h"

class HomingManager {
public:
    HomingManager(MotorController& m1, MotorController& m2, MotorController& m3, Encoder& e1, Encoder& e2, Encoder& e3, MotionPlanner& planner, RobotConfig& config);
    void start(bool doJ1, bool doJ2, bool doJ3);
    void update();
    bool isHoming();

private:
    MotorController &_m1, &_m2, &_m3;
    Encoder &_e1, &_e2, &_e3;
    MotionPlanner &_planner;
    HomingState _state;
    bool _doJ1, _doJ2, _doJ3;
    uint32_t _stepCount;
    uint8_t _retries;

    float _stepsPerDeg1;
    float _stepsPerDegGeared;
};

#endif
