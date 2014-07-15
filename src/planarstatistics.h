#ifndef PLANARSTATISTICS_H
#define PLANARSTATISTICS_H

#include "Datapool.h"
#include <QFile>

class PlanarStatistics
{
public:
    PlanarStatistics(Datapool *i_data);
    ~PlanarStatistics();
    bool setParameters(QDomNode& config);
    void setCurrentTime();
    void init();
    void run();

    bool first;

    Datapool *m_data;
    QString m_ofile;
    int m_osample;
    QFile m_file;
    double m_timeForStill;
    double m_speedForStill;
    double m_RateOfFramesInPerimeter;

    unsigned long frame_count;
    double accumulated_time;

    TimeStamp lastTimeStamp;
    int lastMilliSecondsDifference;
    double secs_diff;

    std::deque<unsigned long> ids;

    //Metric 1: Time moving
    double total_time_moving;
    //Metric 2: Time still
    double total_time_still;

    double current_time_still;
    point2D<double> last_position;

    //Metric 4: Total distance
    double total_distance;

    //Metric 5: Number of times stopping
    int times_stopping;

    //Metric 6: Distance interval = Total_Distance / #Stops
    double distance_interval;

    //Metrics 3, 7, 8 with Event Learning

    //Metric 9: % distance in perimeter
    double distanceInPerimeter;
    double distanceInPerimeterRate;

    //Metric 10: % time in perimeter
    double timeInPerimeter;
    double timeInPerimeterRate;

    QString m_OutZone;
    bool outZonePresent;
    QSharedPointer<world::ZoneH> ozone;
};

#endif // PLANARSTATISTICS_H
