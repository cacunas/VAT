#ifndef __RELIABILITYTRACKING_MODULE_H__
#define __RELIABILITYTRACKING_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "src/calibration.h"
#include "ModuleInterface.h"
#include "src/ReliabilityTracker.h"

class ReliabilityTrackingModule: public ModuleInterface {
 public:
    ReliabilityTrackingModule(Datapool *i_data);
    ~ReliabilityTrackingModule();

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
    ReliabilityTracker *m_ReliabilityTracker;
};


#endif
