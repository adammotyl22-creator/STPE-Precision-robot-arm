#include "TeachManager.h"

TeachManager::TeachManager(MotionPlanner& planner)
    : _planner(planner), _count(0), _playIdx(0), _isRecording(false), _isPlaying(false) {}

void TeachManager::start() { _isRecording = true; _count = 0; }
void TeachManager::record(Joints j) { if (_isRecording && _count < MAX_POINTS) _points[_count++].joints = j; }
void TeachManager::stop() { _isRecording = false; _isPlaying = false; }
void TeachManager::play() { if (_count > 0) { _isPlaying = true; _playIdx = 0; } }
void TeachManager::clear() { _count = 0; _isRecording = false; _isPlaying = false; }

void TeachManager::update() {
    if (!_isPlaying) return;
    if (_planner.isMoving()) return;

    if (_playIdx < _count) {
        _planner.addWaypoint({_points[_playIdx++].joints, 1500});
    } else {
        _isPlaying = false;
    }
}

bool TeachManager::isRecording() { return _isRecording; }
bool TeachManager::isPlaying() { return _isPlaying; }
