#ifndef CONNECTEDCOMPONENTSRLE_H
#define CONNECTEDCOMPONENTSRLE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include "src/rlesegment.h"
#include <QDir>

class ConnectedComponentsRLEModule: public ModuleInterface {
public:
    ConnectedComponentsRLEModule(Datapool *i_data);
    ~ConnectedComponentsRLEModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    private:
        int step;
        int tolerance;
        void setLabel();
};

#endif // CONNECTEDCOMPONENTSRLE_H
