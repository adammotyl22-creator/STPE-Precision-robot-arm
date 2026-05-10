#ifndef ENCODER_H
#define ENCODER_H

#include <Wire.h>
#include "Config.h"

class Encoder {
public:
    Encoder(TwoWire& wire);
    void begin();
    uint16_t readRaw();
    float getAngle(float offset);
    bool isConnected();

private:
    TwoWire& _wire;
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t AS5600_ANGLE_H = 0x0C;
};

#endif
