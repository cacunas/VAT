#include "planarstatistics.h"
#include "VideoAnalysis.h"
#include <QTextStream>

PlanarStatistics::PlanarStatistics(Datapool *i_data):m_data(i_data) {
    frame_count = 0;
    accumulated_time = 0.0;
    first = true;
}

PlanarStatistics::~PlanarStatistics() {}

bool PlanarStatistics::setParameters(QDomNode& config){
    QDomNode n;
    if(config.isNull()) { //Parameter set for module not defined
        m_ofile = "planar-statistics-results.txt";
        m_osample = 1;
        m_timeForStill = 0.5;
        m_speedForStill = 5; // [cm/sec]
    } else {
        if( ( n = XmlCommon::getParameterNode("OutputFileName", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'OutputFileName' for PlanarStatisticsModule. Taking default value (planar-statistics-results.txt).");
            m_ofile = "planar-statistics-results.txt";
        } else
            m_ofile = XmlCommon::getParameterValue(n);

        if( ( n = XmlCommon::getParameterNode("OutputSampleRate", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'OutputSampleRate' for PlanarStatisticsModule. Taking default value (1).");
            m_osample = 1;
        } else
            m_osample = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("TimeForStill", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'TimeForStill' for PlanarStatisticsModule. Taking default value (0.5).");
            m_timeForStill = 0.5;
        } else
            m_timeForStill = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("SpeedForStill", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'SpeedForStill' for PlanarStatisticsModule. Taking default value (5.0).");
            m_speedForStill = 5.0;
        } else
            m_speedForStill = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("RateOfFramesInPerimeter", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'RateOfFramesInPerimeter' for PlanarStatisticsModule. Taking default value (0.5).");
            m_RateOfFramesInPerimeter = 0.5;
        } else
            m_RateOfFramesInPerimeter = XmlCommon::getParameterValue(n).toDouble();

        outZonePresent = true;
        if( ( n = XmlCommon::getParameterNode("OutZoneForPerimeter", config) ).isNull() ) {
            AppendToLog("PlanarStatisticsModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'OutZoneForPerimeter' for PlanarStatisticsModule. Statistics for perimeter deactivated.");
            outZonePresent = false;
            m_OutZone = "";
        } else
            m_OutZone = XmlCommon::getParameterValue(n);

    }

    return true;
}

void PlanarStatistics::init() {
    m_file.setFileName(m_ofile);
    lastTimeStamp.millisecond = -1;
    lastMilliSecondsDifference = 0;


    //Verify perimeter exclusion zone
    if(outZonePresent == true) {
        bool found = false;
        std::vector< QSharedPointer<world::ZoneH> >::iterator it,
                it_end = m_data->sceneModel->ZonesH.end();
        QSharedPointer<world::ZoneH> z;

        for(it=m_data->sceneModel->ZonesH.begin(); it != it_end; it++) {
            z = *it;
            if(z->name == m_OutZone) {
                found = true;
                ozone = z;
            }
        }
        if(found == false)
            outZonePresent = false;
    }

    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        AppendToLog("PlanarStatistics Error: Unable to open file '"+ m_ofile +"' for writing.");
        return;
    }
    QTextStream statsStream(&m_file);
    if(outZonePresent) {
        statsStream << "\t\t#1\t#2\t#4\t#5\t#6\t#9\t#10" << endl;
        statsStream << "Num\tFrame\tT Mov\tT Still\tDist\tStops\tD Inter\t%D Per\t%T Per " << endl;
    } else {
        statsStream << "\t\t#1\t#2\t#4\t#5\t#6" << endl;
        statsStream << "Num\tFrame\tT Mov\tT Still\tDist\tStops\tD Inter" << endl;
    }
    m_file.close();


}

void PlanarStatistics::setCurrentTime() {
    TimeStamp *ts = &m_data->currentHeader->ts;

    //For first frame
    if(lastTimeStamp.millisecond < 0)
      accumulated_time = 0;
    else {
      int sec_diff = ts->second - lastTimeStamp.second;
      //Supposing that maximal frame rate will produce a change between frames in second level, we will just analyze at this level
      if(sec_diff < 0) //A minute change
        sec_diff = 60 + ts->second - lastTimeStamp.second;

      lastMilliSecondsDifference = sec_diff*1000 +  ts->millisecond - lastTimeStamp.millisecond;
      secs_diff = lastMilliSecondsDifference/1000.0;
      accumulated_time += secs_diff;
    }

    memcpy(&lastTimeStamp, ts, sizeof(TimeStamp));

}



void PlanarStatistics::run() {
    unsigned long id;

    //Update counters:
    frame_count++;
    setCurrentTime();

    //Tracking Stats
    if(m_data->degu != NULL && !m_data->degu->trajectory.empty()) {
        if(first) { //init stats
            first = false;
            total_time_moving = 0.0;
            total_time_still = 0.0;

            current_time_still = 0.0;
            last_position.x = last_position.y = 0;
            SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix,
                                              m_data->degu->trajectory.back().x,  m_data->degu->trajectory.back().y,
                                                  &(last_position.x), &(last_position.y) );
            total_distance = 0.0;
            times_stopping = 0;
            distance_interval = 0.0;
            if(outZonePresent) {
                distanceInPerimeter = 0.0;
                distanceInPerimeterRate = 0.0;

                timeInPerimeter = 0.0;
                timeInPerimeterRate = 0.0;
            }
        } else {
            double x_1 = last_position.x , y_1 = last_position.y;
            SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix,
                                              m_data->degu->trajectory.back().x,  m_data->degu->trajectory.back().y,
                                              &(last_position.x), &(last_position.y) );
            double dx = last_position.x - x_1,
                   dy = last_position.y - y_1,
                   D = sqrt(dx*dx + dy*dy);
            double V = D/secs_diff;

            total_distance += D;

            if(V > m_speedForStill) { //moving
                if(current_time_still == 0.0) { //It was moving
                    total_time_moving += secs_diff;
                } else { //It was still
                    //See if time still is relevant
                    total_time_moving += secs_diff/2;
                    if(current_time_still < m_timeForStill) {
                        current_time_still += secs_diff/2.0; //Assuming half time still between frames
                        if(current_time_still >= m_timeForStill) {
                            times_stopping++;
                            total_time_still += current_time_still;
                        } else {
                            total_time_moving += current_time_still;
                        }
                    }
                    current_time_still = 0.0;
                }
            } else { //It is initially still
                if(current_time_still == 0.0) { //It was moving
                    current_time_still += secs_diff/2.0; //Assuming half time still between frames
                    total_time_moving += secs_diff/2.0;
                } else { //It was still
                    if(current_time_still >= m_timeForStill) { //It is already officially stopped
                        current_time_still += secs_diff;
                        total_time_still += secs_diff;
                    } else {
                        current_time_still += secs_diff;
                        //See if time still is relevant
                        if(current_time_still >= m_timeForStill) {
                            times_stopping++;
                            total_time_still += current_time_still;
                        }
                    }
                }
            }

            if(times_stopping > 0)
                distance_interval = total_distance / times_stopping;

            if(outZonePresent) {
                int i, count = 0;
                std::deque<SpDeguInstance> &instances = m_data->degu->instances;
                double x, y, size = instances.size();
                SpDeguInstance instance;

                for(i=0; i<size; i++) {
                    SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix,
                                                      instances[i]->centerPosition.x,
                                                      instances[i]->centerPosition.y,
                                                      &x, &y );
                    if(!ozone->pointInZone(x, y))
                        count++;
                }

                if(count/size > m_RateOfFramesInPerimeter) {
                    distanceInPerimeter += D;
                    distanceInPerimeterRate = D / total_distance;

                    timeInPerimeter += secs_diff;
                    timeInPerimeterRate = secs_diff / accumulated_time;

                }

            }
        }


        //Show in file
        if(frame_count % m_osample == 0) {
            if (!m_file.open(QIODevice::Append | QIODevice::Text)) {
                AppendToLog("PlanarStatistics Error: Unable to open file '"+ m_ofile +"' for append.");
                return;
            }
            QTextStream statsStream(&m_file);
            statsStream.setRealNumberPrecision(4);
            if(outZonePresent) {
                statsStream << frame_count         << "\t"
                            << m_data->frameNumber << "\t"
                            << total_time_moving   << "\t"
                            << total_time_still    << "\t"
                            << total_distance      << "\t"
                            << times_stopping      << "\t"
                            << distance_interval   << "\t"
                            << distanceInPerimeterRate << "\t"
                            << timeInPerimeterRate << endl;
            } else {
                statsStream << frame_count         << "\t"
                            << m_data->frameNumber << "\t"
                            << total_time_moving   << "\t"
                            << total_time_still    << "\t"
                            << total_distance      << "\t"
                            << times_stopping      << "\t"
                            << distance_interval   << endl;
            }
            m_file.close();
        }

    }
}
