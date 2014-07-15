#ifndef SEGMENTATIONMODULE_H
#define SEGMENTATIONMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

#include "opencv2/opencv.hpp"


class segmentationModule: public ModuleInterface {
public:
    segmentationModule(Datapool *i_data);
    ~segmentationModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    //Global updating settings functions
    bool setGlobalZones();
    bool setGlobalMask();
    bool insideInZone(int i, int j);
    bool outsideOutZone(int i, int j);

    void updateBGGlobally(QImage *current);

    bool readImageOnDisk(QString dir,QImage **o_rpOutputImage);


    private:
        int currentFrame;
        int line;        
        int cont;
        bool BGup;
        double alpha;    //tasa de aprendizaje
        uchar T;
        bool m_global;
        int m_N;
        int m_K;
        QString m_InZone;
        QString m_OutZone;
        bool m_inDefined;
        bool m_outDefined;
        bool m_aperture;
        bool globalMaskReady;
        QImage *globalMask;
        int globalValidPoints;
        bool global_validated;
        bool validate_once;
        bool different_bg_current;
        std::map<int, std::deque<RLESegment> > RLEglobal;

        zoneType zInType, zOutType;
        QSharedPointer<world::Zone> zin3D, zout3D;
        QSharedPointer<world::Zone2D> zin2D, zout2D;
        QSharedPointer<world::ZoneH> zinH, zoutH;

        QString bgImName;
        bool first;
        bool globalNotSet;
        bool m_mask;
        QString m_maskImName;
        void applyAperture(QImage *, QImage *);
        void getDiff(QImage *, QImage *);
        void updateBG(QImage *);
        std::map<int, std::deque<int> > globalPoints;
        cv::Mat element;

        bool chromaticSegmentationIsActivated;
        int minGrade;
        int maxGrade;
        int minSaturation;
        int maxLight;
};

#endif // SEGMENTATIONMODULE_H
