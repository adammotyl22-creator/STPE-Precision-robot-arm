#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "Config.h"
#include "Hardware/MotorController.h"

class SafetyManager {
public:
    SafetyManager(MotorController& m1, MotorController& m2, MotorController& m3);
    void update();
    bool isSafe();
    void triggerEstop();
    void resetEstop();
    bool estopActive();

private:
    MotorController &_m1, &_m2, &_m3;
    bool _estopActive;

    void _checkStallGuard();
};

#endif
