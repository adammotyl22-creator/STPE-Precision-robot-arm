#ifndef QUINTIC_SPLINE_H
#define QUINTIC_SPLINE_H

#include <Arduino.h>

struct SplineCoeffs {
    float a0, a1, a2, a3, a4, a5;
};

class QuinticSpline {
public:
    void calculate(float p0, float v0, float a0_in, float p1, float v1, float a1_in, float T);
    float getPos(float t);
    float getVel(float t);
    float getAcc(float t);

private:
    SplineCoeffs _c;
};

#endif
