#ifndef __BACKGROUND_INITIALIZATION_MODULE_H__
#define __BACKGROUND_INITIALIZATION_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

//First Blocks class
#define UNDEFINED 0
#define STATIC 1
#define MOVING 255

//Second Blocks class
#define BACKGROUND 2
#define STILL_OBJ 49
#define ILLUM_CHANGE 250
#define MOVING_OBJ 253

//patter position after template matching
#define INF 0.0
#define MID 0.5
#define SUP 1.0


class BackgroundInitializationModule: public ModuleInterface
{
  public:
    BackgroundInitializationModule(Datapool *i_data);
    ~BackgroundInitializationModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

  private:
    int sizeBlock;
    int numBlockUndef;
    int thresholdStill;      //number of frame necesary to agregate still block to background
    bool initializationStage;
    bool firstTime;
    double aggregationRate;
    double thresholdMov;
    double thresholdCorrelation;
    cv::Mat currRepresentation, prevRepresentation, currentFrameC1, previousFrameC1, currentFrameC3, background;

    bool isMovingBlock(float SADcenter, float SADblock);
    bool isStillObj(uchar value);
    bool isMovingObj(uchar value);
    double corrCoeff(uchar *ptr_img, uchar *ptr_bg);
    double sideMatch(cv::Mat ROI, cv::Mat block, bool isBlockImg);
    void blockMatching(int imgWidth, int imgHeight);
    void blockClassification(int imgWidth, int imgHeight);
    void blockUpdate(int imgWidth, int imgHeight);
};
#endif
