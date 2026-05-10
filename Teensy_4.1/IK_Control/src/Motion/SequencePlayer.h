#ifndef SEQUENCE_PLAYER_H
#define SEQUENCE_PLAYER_H

#include <SD.h>
#include "MotionPlanner.h"

class SequencePlayer {
public:
    SequencePlayer(MotionPlanner& planner);
    bool begin();
    void savePosition(String name, Joints j, unsigned long duration);
    void loadPosition(String name);
    void runSequence(String name);
    void update();
    bool isPlaying();

private:
    MotionPlanner& _planner;
    File _currentFile;
    bool _isPlaying;
    bool _sdReady;
};

#endif
