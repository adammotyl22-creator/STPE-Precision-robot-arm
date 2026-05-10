#include "CommandProcessor.h"
#include "../Kinematics/Kinematics.h"

CommandProcessor::CommandProcessor(MotionPlanner& planner, SafetyManager& safety, SequencePlayer& seq, TeachManager& teach, HomingManager& home)
    : _planner(planner), _safety(safety), _seq(seq), _teach(teach), _home(home), _pos(0) {}

void CommandProcessor::processSerial(Stream& stream) {
    while (stream.available()) {
        char c = stream.read();
        if (c == '<') {
            _pos = 0;
            _buffer[_pos] = '\0';
        } else if (c == '>') {
            _buffer[_pos] = '\0';
            _handlePacket(String(_buffer));
            _pos = 0;
        } else if (c == '\n' || c == '\r') {
            _buffer[_pos] = '\0';
            if (_pos > 0) _handleCommand(String(_buffer));
            _pos = 0;
        } else if (_pos < 127) {
            _buffer[_pos++] = c;
        }
    }
}

void CommandProcessor::_handlePacket(String data) {
    // Expected format: j1,j2,j3,j4,j5,dx,dy,duration,crc
    int lastComma = data.lastIndexOf(',');
    if (lastComma == -1) return;

    // For now simple parsing (skipping CRC check for brevity, but could be added)
    int idx[8];
    int start = 0;
    for (int i = 0; i < 8; i++) {
        idx[i] = data.indexOf(',', start);
        if (idx[i] == -1 && i < 7) return;
        start = idx[i] + 1;
    }

    float j1 = data.substring(0, idx[0]).toFloat();
    float j2 = data.substring(idx[0]+1, idx[1]).toFloat();
    float j3 = data.substring(idx[1]+1, idx[2]).toFloat();
    float j4 = data.substring(idx[2]+1, idx[3]).toFloat();
    float j5 = data.substring(idx[3]+1, idx[4]).toFloat();
    // dx, dy would go to gantry
    unsigned long dur = data.substring(idx[6]+1, idx[7]).toInt();

    _planner.addWaypoint({{j1, j2, j3, j4, j5}, dur});
}

void CommandProcessor::_handleCommand(String bufStr) {
    bufStr.trim();
    if (bufStr.length() == 0) return;
    bufStr.toUpperCase();

    if (bufStr.startsWith("MOV")) {
        float x=0, y=0, z=BASE_H, p=0, r=0;
        unsigned long d = 1000;
        int idxX = bufStr.indexOf('X');
        int idxY = bufStr.indexOf('Y');
        int idxZ = bufStr.indexOf('Z');
        int idxP = bufStr.indexOf('P');
        int idxR = bufStr.indexOf('R');
        int idxD = bufStr.indexOf('D');
        if (idxX != -1) x = bufStr.substring(idxX+1).toFloat();
        if (idxY != -1) y = bufStr.substring(idxY+1).toFloat();
        if (idxZ != -1) z = bufStr.substring(idxZ+1).toFloat();
        if (idxP != -1) p = bufStr.substring(idxP+1).toFloat();
        if (idxR != -1) r = bufStr.substring(idxR+1).toFloat();
        if (idxD != -1) d = bufStr.substring(idxD+1).toInt();
        Pose target = {x, y, z, p, r};
        if (bufStr.startsWith("MOVL")) Kinematics::movL(target, d, _planner);
        else {
            Joints sol = Kinematics::inverse(target);
            if (Kinematics::checkWorkspace(sol)) _planner.addWaypoint({sol, d});
            else Serial.println("ERR: Workspace violation");
        }
    } else if (bufStr == "ESTOP") {
        _safety.triggerEstop();
        _planner.stop();
        Serial.println("ESTOP ACTIVE");
    } else if (bufStr == "RESET") {
        _safety.resetEstop();
        Serial.println("ESTOP RESET");
    } else if (bufStr == "STOP") {
        _planner.stop();
        Serial.println("Motion stopped");
    } else if (bufStr.startsWith("SAVE ")) {
        _seq.savePosition(bufStr.substring(5), _planner.getCurrentJoints(), 1000);
    } else if (bufStr.startsWith("LOAD ")) {
        _seq.loadPosition(bufStr.substring(5));
    } else if (bufStr.startsWith("RUN ")) {
        _seq.runSequence(bufStr.substring(4));
    } else if (bufStr.startsWith("TEACH")) {
        String arg = bufStr.substring(5); arg.trim();
        if (arg == "START") _teach.start();
        else if (arg == "RECORD") _teach.record(_planner.getCurrentJoints());
        else if (arg == "STOP") _teach.stop();
        else if (arg == "PLAY") _teach.play();
        else if (arg == "CLEAR") _teach.clear();
    } else if (bufStr.startsWith("HOME")) {
        _home.start(true, true, true);
    } else if (bufStr.startsWith("D1")) {
        float val = bufStr.substring(2).toFloat();
        Joints j = _planner.getCurrentJoints();
        j.j1 += val;
        _planner.addWaypoint({j, 500});
    } else if (bufStr.startsWith("D2")) {
        float val = bufStr.substring(2).toFloat();
        Joints j = _planner.getCurrentJoints();
        j.j2 += val;
        _planner.addWaypoint({j, 500});
    } else if (bufStr.startsWith("D3")) {
        float val = bufStr.substring(2).toFloat();
        Joints j = _planner.getCurrentJoints();
        j.j3 += val;
        _planner.addWaypoint({j, 500});
    } else if (bufStr.startsWith("GRIP")) {
        // Implement gripper control
    }
}

void CommandProcessor::sendTelemetry(Stream& stream) {
    Joints j = _planner.getCurrentJoints();
    Pose p = Kinematics::forward(j);
    stream.print("{");
    stream.print("\"j1\":"); stream.print(j.j1); stream.print(",");
    stream.print("\"j2\":"); stream.print(j.j2); stream.print(",");
    stream.print("\"j3\":"); stream.print(j.j3); stream.print(",");
    stream.print("\"j4\":"); stream.print(j.j4); stream.print(",");
    stream.print("\"j5\":"); stream.print(j.j5); stream.print(",");
    stream.print("\"x\":"); stream.print(p.x); stream.print(",");
    stream.print("\"y\":"); stream.print(p.y); stream.print(",");
    stream.print("\"z\":"); stream.print(p.z);
    stream.println("}");
}
