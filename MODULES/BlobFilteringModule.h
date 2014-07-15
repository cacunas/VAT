#ifndef BLOBFILTERINGMODULE_H
#define BLOBFILTERINGMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

class BlobFilteringModule: public ModuleInterface {
public:
    BlobFilteringModule(Datapool *i_data);
    ~BlobFilteringModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    bool blobOK(Blob *blob);

    void deleteBlobFromForeground(Blob *blob);


    private:
        //Internal data:
        int m_minArea;
        int m_minHeight;
        int m_minWidth;
        double m_ignoreBorderRate;
        int m_imgWidth;
        int m_imgHeight;
        int m_numPixWidth;
        int m_numPixHeight;
        bool m_delInForeground;
        bool borderConstraintsSet;

};

#endif // BLOBFILTERINGMODULE_H
