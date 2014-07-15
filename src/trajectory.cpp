#include "trajectory.h"

TrajectoryPoint::TrajectoryPoint(double i_x, double i_y) {
    x = i_x;
    y = i_y;
}

TrajectoryPoint::~TrajectoryPoint() {}

Trajectory::Trajectory(int i_maxLength) : maxLength(i_maxLength) {
    if(i_maxLength < 0) maxLength = 0;
    trajectoryPoints.clear();
}

Trajectory::~Trajectory(){
    trajectoryPoints.clear();
}

std::deque<SpTrajectoryPoint>::iterator Trajectory::begin() {
    return trajectoryPoints.begin();
}

std::deque<SpTrajectoryPoint>::iterator Trajectory::end() {
    return trajectoryPoints.end();
}

std::deque<SpTrajectoryPoint>::reverse_iterator Trajectory::rbegin() {
    return trajectoryPoints.rbegin();
}

std::deque<SpTrajectoryPoint>::reverse_iterator Trajectory::rend() {
    return trajectoryPoints.rend();
}


SpTrajectoryPoint Trajectory::operator[](int index){
    return trajectoryPoints[index];
}

SpTrajectoryPoint Trajectory::back() {
    return trajectoryPoints.back();
}

SpTrajectoryPoint Trajectory::front() {
    return trajectoryPoints.front();
}

unsigned int Trajectory::size() {
    return trajectoryPoints.size();
}

void Trajectory::insert(double x, double y) {
    SpTrajectoryPoint newPoint(new TrajectoryPoint(x, y));

    if(maxLength > 0 && size() == maxLength)
        trajectoryPoints.pop_front();

    trajectoryPoints.push_back(newPoint);
}

void Trajectory::clear() {
    trajectoryPoints.clear();
}

void Trajectory::copyPoints(Trajectory *buffer) {
    trajectoryPoints = buffer->trajectoryPoints;
}
