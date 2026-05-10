#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include "../Config.h"

class ServoController {
public:
    ServoController(uint8_t pin, float minAngle, float maxAngle);
    void begin();
    void write(float angle);
    float read();

private:
    Servo _servo;
    uint8_t _pin;
    float _minAngle, _maxAngle;
};

#endif
