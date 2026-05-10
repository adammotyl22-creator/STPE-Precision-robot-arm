#include "Encoder.h"

Encoder::Encoder(TwoWire& wire) : _wire(wire) {}

void Encoder::begin() {
    // _wire.begin() should be called outside if shared
}

uint16_t Encoder::readRaw() {
    _wire.beginTransmission(AS5600_ADDR);
    _wire.write(AS5600_ANGLE_H);
    if (_wire.endTransmission(false) != 0) return 0xFFFF;

    if (_wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;

    uint8_t hi = _wire.read();
    uint8_t lo = _wire.read();
    return ((uint16_t)hi << 8) | lo;
}

float Encoder::getAngle(float offset) {
    uint16_t raw = readRaw();
    if (raw == 0xFFFF) return NAN;

    float rawAngle = (raw / 4096.0f) * 360.0f;
    float delta = rawAngle - offset;

    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;

    return delta;
}

bool Encoder::isConnected() {
    _wire.beginTransmission(AS5600_ADDR);
    return (_wire.endTransmission() == 0);
}
