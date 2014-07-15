#ifndef CONNECTEDCOMPONENTS2STEPS_H
#define CONNECTEDCOMPONENTS2STEPS_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

class ConnectedComponentsTwoStepsModule: public ModuleInterface {
public:
    ConnectedComponentsTwoStepsModule(Datapool *i_data);
    ~ConnectedComponentsTwoStepsModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    void updateConnections(int new_v, int old_v, std::map<int, int> &con);

    private:
        void setLabel();

};

#endif // CONNECTEDCOMPONENTS2STEPS_H
