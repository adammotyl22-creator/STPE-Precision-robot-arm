#include "MotionPlanner.h"

MotionPlanner::MotionPlanner() : _head(0), _tail(0), _isMoving(false) {
    _currentJoints = {0, -90, 90, 90, 90};
    _currentVel = {0, 0, 0, 0, 0};
    _currentAcc = {0, 0, 0, 0, 0};
}

bool MotionPlanner::addWaypoint(Waypoint wp) {
    uint8_t nextHead = (_head + 1) % QUEUE_SIZE;
    if (nextHead == _tail) return false;

    _queue[_head] = wp;
    _head = nextHead;
    return true;
}

void MotionPlanner::update() {
    if (!_isMoving) {
        if (_head != _tail) {
            _startNextSegment();
        } else {
            return;
        }
    }

    float t = (float)(micros() - _startTime) / 1000000.0f;
    if (t >= _duration) {
        t = _duration;
        _isMoving = false;

        _currentJoints.j1 = _splines[0].getPos(t);
        _currentJoints.j2 = _splines[1].getPos(t);
        _currentJoints.j3 = _splines[2].getPos(t);
        _currentJoints.j4 = _splines[3].getPos(t);
        _currentJoints.j5 = _splines[4].getPos(t);

        _currentVel.j1 = _splines[0].getVel(t);
        _currentVel.j2 = _splines[1].getVel(t);
        _currentVel.j3 = _splines[2].getVel(t);
        _currentVel.j4 = _splines[3].getVel(t);
        _currentVel.j5 = _splines[4].getVel(t);

        _currentAcc.j1 = _splines[0].getAcc(t);
        _currentAcc.j2 = _splines[1].getAcc(t);
        _currentAcc.j3 = _splines[2].getAcc(t);
        _currentAcc.j4 = _splines[3].getAcc(t);
        _currentAcc.j5 = _splines[4].getAcc(t);

        _tail = (_tail + 1) % QUEUE_SIZE;
    } else {
        _currentJoints.j1 = _splines[0].getPos(t);
        _currentJoints.j2 = _splines[1].getPos(t);
        _currentJoints.j3 = _splines[2].getPos(t);
        _currentJoints.j4 = _splines[3].getPos(t);
        _currentJoints.j5 = _splines[4].getPos(t);

        _currentVel.j1 = _splines[0].getVel(t);
        _currentVel.j2 = _splines[1].getVel(t);
        _currentVel.j3 = _splines[2].getVel(t);
        _currentVel.j4 = _splines[3].getVel(t);
        _currentVel.j5 = _splines[4].getVel(t);

        _currentAcc.j1 = _splines[0].getAcc(t);
        _currentAcc.j2 = _splines[1].getAcc(t);
        _currentAcc.j3 = _splines[2].getAcc(t);
        _currentAcc.j4 = _splines[3].getAcc(t);
        _currentAcc.j5 = _splines[4].getAcc(t);
    }
}

void MotionPlanner::_startNextSegment() {
    Waypoint& target = _queue[_tail];
    _duration = (float)target.duration_ms / 1000.0f;
    if (_duration <= 0) _duration = 0.001f;

    Joints v1 = {0,0,0,0,0};
    Joints a1 = {0,0,0,0,0};

    // Continuous motion check
    uint8_t nextIdx = (_tail + 1) % QUEUE_SIZE;
    if (nextIdx != _head) {
        Waypoint& after = _queue[nextIdx];
        // simple heuristic for velocity at waypoint: average of segments
        // v = (p2 - p0) / (T1 + T2)
        float nextDur = (float)after.duration_ms / 1000.0f;
        float totalT = _duration + nextDur;
        v1.j1 = (after.joints.j1 - _currentJoints.j1) / totalT;
        v1.j2 = (after.joints.j2 - _currentJoints.j2) / totalT;
        v1.j3 = (after.joints.j3 - _currentJoints.j3) / totalT;
        v1.j4 = (after.joints.j4 - _currentJoints.j4) / totalT;
        v1.j5 = (after.joints.j5 - _currentJoints.j5) / totalT;
    }

    _splines[0].calculate(_currentJoints.j1, _currentVel.j1, _currentAcc.j1, target.joints.j1, v1.j1, a1.j1, _duration);
    _splines[1].calculate(_currentJoints.j2, _currentVel.j2, _currentAcc.j2, target.joints.j2, v1.j2, a1.j2, _duration);
    _splines[2].calculate(_currentJoints.j3, _currentVel.j3, _currentAcc.j3, target.joints.j3, v1.j3, a1.j3, _duration);
    _splines[3].calculate(_currentJoints.j4, _currentVel.j4, _currentAcc.j4, target.joints.j4, v1.j4, a1.j4, _duration);
    _splines[4].calculate(_currentJoints.j5, _currentVel.j5, _currentAcc.j5, target.joints.j5, v1.j5, a1.j5, _duration);

    _startTime = micros();
    _isMoving = true;
}

Joints MotionPlanner::getCurrentJoints() { return _currentJoints; }
Joints MotionPlanner::getCurrentVelocities() { return _currentVel; }
bool MotionPlanner::isMoving() { return _isMoving || _head != _tail; }
void MotionPlanner::stop() { _head = _tail; _isMoving = false; _currentVel = {0,0,0,0,0}; _currentAcc = {0,0,0,0,0}; }
