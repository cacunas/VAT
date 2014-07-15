#ifndef __DATAPOOL_H__
#define __DATAPOOL_H__

#include <QImage>
#ifdef __COMPILE_QT5__
    #include <QtWidgets/QTextEdit>
#else
    #include<QTextEdit>
#endif
#include <deque>
#include <map>
#include "src/calibration.h"
#include "src/blob.h"
#include "src/hullmodel.h"
#include "src/object.h"
#include "src/ModelInterface.h"
#include "src/ImageHeader.h"
#include "src/MobileObject.h"
#include "src/MobileObject2D.h"
#include "src/RMMT/RMMMobileObject.h"
#include "src/rlesegment.h"
#include "src/IncrementalEventLearning.h"
#include "src/degumodel.h"
#include "MODELS/DeguTrackingModel.h"

class Datapool: public QObject {
    Q_OBJECT
    public:
        Datapool();
        ~Datapool();
        static QVector<QRgb> *setGrayScaleTable();
        void appendToLog(const QString&);
        static QString getCurrentDateTimeString();
        void pauseApp();
        void clear();
        void init();
        //Data:
        //Frame number from acquisition module
        int frameNumber;

        //Program modules configuration
        std::vector<QString> moduleSequenceNames;
        std::vector<double> moduleSequenceMSTime;
        std::vector<double> moduleSequenceMeanTime;
        std::vector<double> moduleSequenceVarTime;
        std::vector<int> moduleSequenceMinTime;
        std::vector<int> moduleSequenceMaxTime;
        double totalTime;
        long processedFrames;

        //Scene information
        SceneModel *sceneModel;
        std::map<ObjectType, SpModelInterface> objectModels;

        //Current Image Frame
        QImage *currentImage;
        QImage *previousImage;

        //Current Gray Image Frame
        QImage *grayImage;

        //Image header with date/time information
        ImageHeader *currentHeader;

        //Background Image
        QImage *bgImage;

        //Mask Image
        QImage *maskImage;

        //Segmentation results
        QImage *fgImage; //Indexed8
        int *labelMatrix; //Array representing the image data of
                          //labels for connected components (same size of images).

        //Regional segmentation results
        QImage *currentDiffImage; //difference w/r to previous image in (t) and (t-1)
        QImage *previousDiffImage;//difference w/r to previous image in (t-1) and (t-2)
        QImage *rFgImage; //RGB888

        //Personality
        QImage *personalityPCAImage; //RGB888

        //RLE segments of FG image, ordered by row
        std::map<int, std::deque<RLESegment> > RLEFGImage;
        //RLE regions of FG image, ordered by label
        std::map<int, RLERegion> RLERegions;

        //Incremental Learning Information
        miles::SpIncrementalEventLearning eventLearning;
        miles::SpLearningAttributes availableAttributes;
        std::map<unsigned long, std::map<int, miles::SpLevelData> > levelData;
        std::map<unsigned long, std::map<long int, double> > Pr;

        //MODELS

        //Blob list
        std::vector<Blob> blobs;
        //std::vector<HullModel> hulls; //Convex hull, with OpenCV classes
        //Object list
        std::deque<SpMobileObject> objects;
        std::deque<SpMobileObject2D> objects2D;
        std::deque<SpRMMMobileObject> RMMobjects;

        std::deque<SpHullModel> hulls; //Convex hull model, with OpenCV classes

        std::map<int, Object> fake_objects;

        //TEMP
        std::map<QString, DeguTrackingModel *> modelsToFollow;

  //Event list
  //std::deque<Event> events;

  //OTHER GLOBAL INFORMATION:
  //Current configuration file name
  QString m_config_file;

  //Color table for grayscale image
  QVector<QRgb> *grayScaleTable;


  //For Degu detection
  DeguModel *degu;

  //For Personality Module
  QImage *persoImage;

  /*! This flag can be used to stop module sequence excecution at any point
   * Useful for modules that require an initialization process and don't
   * provide valid output until certain conditions are met.
  */
  bool runModules;

  signals:
    void logString(QString);
    void pause();
};


#endif
