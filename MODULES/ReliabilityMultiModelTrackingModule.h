#ifndef __RELIABILITYMMTRACKING_MODULE_H__
#define __RELIABILITYMMTRACKING_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "src/calibration.h"
#include "ModuleInterface.h"
#include "src/RMMT/RMMTracker.h"

class ReliabilityMultiModelTrackingModule: public ModuleInterface {
 public:
    ReliabilityMultiModelTrackingModule(Datapool *i_data);
    ~ReliabilityMultiModelTrackingModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);
  
    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

 private:
    //Internal data:
    RMMTracker *m_RMMTracker;
};


#endif
