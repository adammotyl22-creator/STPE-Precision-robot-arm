#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <Arduino.h>
#include "QuinticSpline.h"
#include "../Kinematics/Kinematics.h"

struct Waypoint {
    Joints joints;
    unsigned long duration_ms;
};

class MotionPlanner {
public:
    MotionPlanner();
    bool addWaypoint(Waypoint wp);
    void update();
    Joints getCurrentJoints();
    Joints getCurrentVelocities();
    bool isMoving();
    void stop();

private:
    static const uint8_t QUEUE_SIZE = 32;
    Waypoint _queue[QUEUE_SIZE];
    uint8_t _head, _tail;

    QuinticSpline _splines[5];
    Joints _currentJoints;
    Joints _currentVel;
    Joints _currentAcc;

    unsigned long _startTime;
    float _duration;
    bool _isMoving;

    void _startNextSegment();
};

#endif
