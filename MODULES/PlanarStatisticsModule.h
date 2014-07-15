#ifndef PLANARSTATISTICSMODULE_H
#define PLANARSTATISTICSMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include "src/planarstatistics.h"

class PlanarStatisticsModule: public ModuleInterface {
public:
    PlanarStatisticsModule(Datapool *i_data);
    ~PlanarStatisticsModule();

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
        PlanarStatistics *p;
};

#endif // PlanarStatisticsModule_H
