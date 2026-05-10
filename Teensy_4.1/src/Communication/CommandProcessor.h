#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "Motion/MotionPlanner.h"
#include "Motion/SequencePlayer.h"
#include "Motion/TeachManager.h"
#include "SafetyManager.h"
#include "HomingManager.h"

class CommandProcessor {
public:
    CommandProcessor(MotionPlanner& planner, SafetyManager& safety, SequencePlayer& seq, TeachManager& teach, HomingManager& home);
    void processSerial(Stream& stream);
    void sendTelemetry(Stream& stream);

private:
    MotionPlanner& _planner;
    SafetyManager& _safety;
    SequencePlayer& _seq;
    TeachManager& _teach;
    HomingManager& _home;
    char _buffer[128];
    uint8_t _pos;

    void _handleCommand(String cmd);
    void _handlePacket(String data);
};

#endif
