#include "SequencePlayer.h"

SequencePlayer::SequencePlayer(MotionPlanner& planner) : _planner(planner), _isPlaying(false), _sdReady(false) {}

bool SequencePlayer::begin() {
    if (_sdReady) return true;
    if (!SD.begin(BUILTIN_SDCARD)) return false;
    _sdReady = true;
    return true;
}

void SequencePlayer::savePosition(String name, Joints j, unsigned long duration) {
    if (!begin()) return;
    String filename = "/" + name + ".pos";
    if (SD.exists(filename.c_str())) SD.remove(filename.c_str());
    File f = SD.open(filename.c_str(), FILE_WRITE);
    if (!f) return;
    f.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%lu\n", j.j1, j.j2, j.j3, j.j4, j.j5, duration);
    f.close();
}

void SequencePlayer::loadPosition(String name) {
    if (!begin()) return;
    String filename = "/" + name + ".pos";
    File f = SD.open(filename.c_str(), FILE_READ);
    if (!f) return;
    String line = f.readStringUntil('\n');
    f.close();

    float val[5];
    unsigned long dur;
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1+1);
    int i3 = line.indexOf(',', i2+1);
    int i4 = line.indexOf(',', i3+1);
    int i5 = line.indexOf(',', i4+1);

    if (i1 == -1) return;
    val[0] = line.substring(0, i1).toFloat();
    val[1] = line.substring(i1+1, i2).toFloat();
    val[2] = line.substring(i2+1, i3).toFloat();
    val[3] = line.substring(i3+1, i4).toFloat();
    val[4] = line.substring(i4+1, i5).toFloat();
    dur = line.substring(i5+1).toInt();

    _planner.addWaypoint({{val[0], val[1], val[2], val[3], val[4]}, dur});
}

void SequencePlayer::runSequence(String name) {
    if (!begin()) return;
    if (_isPlaying) _currentFile.close();
    String filename = "/" + name + ".seq";
    _currentFile = SD.open(filename.c_str(), FILE_READ);
    if (!_currentFile) return;
    _isPlaying = true;
}

void SequencePlayer::update() {
    if (!_isPlaying) return;
    if (_planner.isMoving()) return; // Wait for current move to finish

    while (_currentFile.available()) {
        String line = _currentFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) continue;

        float val[5];
        unsigned long dur;
        int i1 = line.indexOf(',');
        int i2 = line.indexOf(',', i1+1);
        int i3 = line.indexOf(',', i2+1);
        int i4 = line.indexOf(',', i3+1);
        int i5 = line.indexOf(',', i4+1);

        if (i1 == -1) continue;
        val[0] = line.substring(0, i1).toFloat();
        val[1] = line.substring(i1+1, i2).toFloat();
        val[2] = line.substring(i2+1, i3).toFloat();
        val[3] = line.substring(i3+1, i4).toFloat();
        val[4] = line.substring(i4+1, i5).toFloat();
        dur = line.substring(i5+1).toInt();

        _planner.addWaypoint({{val[0], val[1], val[2], val[3], val[4]}, dur});
        return; // Process one waypoint at a time to allow planner to start
    }
    _currentFile.close();
    _isPlaying = false;
}

bool SequencePlayer::isPlaying() { return _isPlaying; }
