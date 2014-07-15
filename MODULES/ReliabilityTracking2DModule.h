#ifndef __RELIABILITYTRACKING2D_MODULE_H__
#define __RELIABILITYTRACKING2D_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "src/calibration.h"
#include "ModuleInterface.h"
#include "src/ReliabilityTracker2D.h"

class ReliabilityTracking2DModule: public ModuleInterface {
 public:
    ReliabilityTracking2DModule(Datapool *i_data);
    ~ReliabilityTracking2DModule();

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
    ReliabilityTracker2D *m_ReliabilityTracker;
};


#endif
