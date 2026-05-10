#include "QuinticSpline.h"

void QuinticSpline::calculate(float p0, float v0, float a0_in, float p1, float v1, float a1_in, float T) {
    if (T <= 0) T = 0.001f;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;

    _c.a0 = p0;
    _c.a1 = v0;
    _c.a2 = 0.5f * a0_in;
    _c.a3 = (20.0f * p1 - 20.0f * p0 - (8.0f * v1 + 12.0f * v0) * T - (3.0f * a0_in - a1_in) * T2) / (2.0f * T3);
    _c.a4 = (30.0f * p0 - 30.0f * p1 + (14.0f * v1 + 16.0f * v0) * T + (3.0f * a0_in - 2.0f * a1_in) * T2) / (2.0f * T4);
    _c.a5 = (12.0f * p1 - 12.0f * p0 - (6.0f * v1 + 6.0f * v0) * T - (a0_in - a1_in) * T2) / (2.0f * T5);
}

float QuinticSpline::getPos(float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    return _c.a0 + _c.a1 * t + _c.a2 * t2 + _c.a3 * t3 + _c.a4 * t4 + _c.a5 * t5;
}

float QuinticSpline::getVel(float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    return _c.a1 + 2.0f * _c.a2 * t + 3.0f * _c.a3 * t2 + 4.0f * _c.a4 * t3 + 5.0f * _c.a5 * t4;
}

float QuinticSpline::getAcc(float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return 2.0f * _c.a2 + 6.0f * _c.a3 * t + 12.0f * _c.a4 * t2 + 20.0f * _c.a5 * t3;
}
