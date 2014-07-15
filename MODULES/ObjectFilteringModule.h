#ifndef OBJECTFILTERINGMODULE_H
#define OBJECTFILTERINGMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

class ObjectFilteringModule: public ModuleInterface {
public:
    ObjectFilteringModule(Datapool *i_data);
    ~ObjectFilteringModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    bool objectOK(SpRMMMobileObject object);

    private:
        //Internal data:
        int m_minArea;
        int m_minHeight;
        int m_minWidth;
        int m_imgWidth;
        int m_imgHeight;
        int m_numPixWidth;
        int m_numPixHeight;
        int m_minNumFrames;
        int m_minNumFramesReliability;
        double m_minReliability;
        int m_minObjects;
        int m_maxObjects;
        std::set<int> m_ids;
};

#endif // ObjectFilteringModule_H
