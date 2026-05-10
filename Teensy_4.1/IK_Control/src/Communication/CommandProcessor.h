#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "Motion/MotionPlanner.h"
#include "Motion/SequencePlayer.h"
#include "Motion/TeachManager.h"
#include "SafetyManager.h"
#include "HomingManager.h"
#include "Hardware/ServoController.h"
#include "ConfigurationManager.h"

class CommandProcessor {
public:
    CommandProcessor(MotionPlanner& planner, SafetyManager& safety, SequencePlayer& seq, TeachManager& teach, HomingManager& home, ServoController& s4, ServoController& s5, ServoController& grip, RobotConfig& config);
    void processSerial(Stream& stream);
    void sendTelemetry(Stream& stream);

private:
    MotionPlanner& _planner;
    SafetyManager& _safety;
    SequencePlayer& _seq;
    TeachManager& _teach;
    HomingManager& _home;
    ServoController &_s4, &_s5, &_grip;
    RobotConfig& _config;
    char _buffer[128];
    uint8_t _pos;

    void _handleCommand(String cmd);
    void _handlePacket(String data);
};

#endif
