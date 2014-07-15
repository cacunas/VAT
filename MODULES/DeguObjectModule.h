#ifndef DEGUOBJECTMODULE_H
#define DEGUOBJECTMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include "src/hullmodel.h"

#include <QDir>

class DeguObjectModule: public ModuleInterface {
public:
    DeguObjectModule(Datapool *i_data);
    ~DeguObjectModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    int line;
    int cont;

    std::deque<int> m_ids;
    Rectangle<int> lastReliableRect;
    bool foundBefore;
    int DTolerance;
};

#endif // DEGUOBJECTMODULE_H
