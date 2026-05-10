#ifndef TEACH_MANAGER_H
#define TEACH_MANAGER_H

#include "Motion/MotionPlanner.h"

struct TeachPoint {
    Joints joints;
};

class TeachManager {
public:
    TeachManager(MotionPlanner& planner);
    void start();
    void record(Joints j);
    void stop();
    void play();
    void clear();
    void update();
    bool isRecording();
    bool isPlaying();

private:
    MotionPlanner& _planner;
    static const uint8_t MAX_POINTS = 64;
    TeachPoint _points[MAX_POINTS];
    uint8_t _count;
    uint8_t _playIdx;
    bool _isRecording;
    bool _isPlaying;
};

#endif
