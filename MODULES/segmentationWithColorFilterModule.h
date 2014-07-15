#ifndef SEGMENTATIONWITHCOLORFILTER_H
#define SEGMENTATIONWITHCOLORFILTER_H


#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>


class segmentationWithColorFilterModule: public ModuleInterface {
public:
    segmentationWithColorFilterModule(Datapool *i_data);
    ~segmentationWithColorFilterModule();

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
        void getChromaticSegmentation(QImage *, QImage *,int,int);
        void updateBG(QImage *);
        std::map<int, std::deque<int> > globalPoints;
        cv::Mat element;

        int minGrade, maxGrade;
};


#endif // SEGMENTATIONWITHCOLORFILTER_H
