#ifndef CLOSED_LOOP_CONTROLLER_H
#define CLOSED_LOOP_CONTROLLER_H

#include "Hardware/Encoder.h"
#include "Hardware/MotorController.h"

struct PIDGains {
    float kP, kI, kD, kFF, kG;
};

class ClosedLoopController {
public:
    ClosedLoopController(Encoder& enc, MotorController& motor, PIDGains gains);
    float update(float targetAngle, float targetVelocity, float dt);
    void setEnabled(bool enabled);
    void setOffset(float offset);

private:
    Encoder& _enc;
    MotorController& _motor;
    PIDGains _gains;
    float _integral;
    float _lastError;
    float _offset;
    bool _enabled;
};

#endif
