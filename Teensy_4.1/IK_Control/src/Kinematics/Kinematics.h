#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "../Config.h"

struct Joints {
    float j1, j2, j3, j4, j5;
};

struct Pose {
    float x, y, z, pitch, roll;
};

class Kinematics {
public:
    static Joints inverse(Pose target);
    static Pose forward(Joints j);
    static bool checkWorkspace(Joints j);
    static void movL(Pose target, unsigned long duration, class MotionPlanner& planner);
};

#endif
