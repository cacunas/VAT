#ifndef TRACKINGMODULE_H
#define TRACKINGMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "src/calibration.h"
#include <QDir>
#include <math.h>
#include "src/trackingatribute.h"
#include "ModuleInterface.h"

class trackingModule: public ModuleInterface {
public:
    trackingModule(Datapool *i_data);
    ~trackingModule();

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
    //QString m_fileName;   //para xml
    position m_pos;
    //QDomDocument *xmlGT;  //para xml
    //QDomElement root;     //para xml
    //QDomNode currentNode; //para xml
    int currentFrame;
    int line;

    // Premerge
    void preMerge(int Hmax);
    bool blobSortCriterionPos (Blob b1, Blob b2);
    bool blobSortCriterionBase (Blob b1, Blob b2);
};

#endif // TRACKINGMODULE_H
