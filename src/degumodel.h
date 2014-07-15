#ifndef DEGUMODEL_H
#define DEGUMODEL_H

#include "hullmodel.h"
#include "RMMT/RMMMobileObject.h"
#include <QSharedPointer>

class DeguInstance {
public:
    DeguInstance();
    ~DeguInstance();

    SpHullModel hull;
    point2D<int> centerPosition; //axis center position
    point2D<int> headPosition;   //estimated head position

    bool headByHull;   //head could be determined by hull
    bool headAtPoint1; //head is near point 1 (false means point 2), valid if headByHull=true

    bool headByVelocity;   //head could be determined by velocity

    double length;
    double Vx;
    double Vy;
    double Rhull;
    double RV;
    double R;
    int frameNumber;
};

typedef QSharedPointer<DeguInstance> SpDeguInstance;

class Datapool;

class DeguModel {
public:
    DeguModel(int i_size);
    ~DeguModel();

    void insert(Datapool *i_data, SpHullModel h, SpRMMMobileObject o);
    void move(Datapool *i_data, SpRMMMobileObject o);
    void keep(int frame);

    static double DEGU_SIZE_NO_TAIL;
    static double DEGU_SIZE_WITH_TAIL;

    std::deque<SpDeguInstance> instances;
    point2D<int> headPosition;
    std::deque< point2D<int> > trajectory;
    std::deque< point2D<int> > head_trajectory;
    std::deque<int>            frame_numbers;
    int size;
    int numFramesSeen;
    int insertionType;
};

#endif // DEGUMODEL_H
