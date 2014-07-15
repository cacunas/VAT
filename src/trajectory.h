#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include<QSharedPointer>
#include<deque>

class TrajectoryPoint{
    public:
        double x, y;
        TrajectoryPoint(double i_x, double i_y);
        ~TrajectoryPoint();
};

typedef QSharedPointer<TrajectoryPoint> SpTrajectoryPoint;

class Trajectory {
    private:
        //maxLength = 0 means no end
        unsigned int maxLength;
    public:
        std::deque<SpTrajectoryPoint> trajectoryPoints;
        Trajectory(int);
        ~Trajectory();
        std::deque<SpTrajectoryPoint>::iterator begin();
        std::deque<SpTrajectoryPoint>::iterator end();
        std::deque<SpTrajectoryPoint>::reverse_iterator rbegin();
        std::deque<SpTrajectoryPoint>::reverse_iterator rend();
        SpTrajectoryPoint operator[](int);
        unsigned int size();
        SpTrajectoryPoint back();
        SpTrajectoryPoint front();
        void clear();
        void insert(double, double);
        void copyPoints(Trajectory *);
};

#endif // TRAJECTORY_H
