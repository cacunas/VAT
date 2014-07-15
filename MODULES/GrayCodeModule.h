#ifndef GRAYCODEMODULE_H
#define GRAYCODEMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include <QDir>

class GrayCodeModule: public ModuleInterface {
public:
    enum bit_code {
        BIT_1 = 0x01, //00000001
        BIT_2 = 0x02, //00000010
        BIT_3 = 0x04, //00000100
        BIT_4 = 0x08, //00001000
        BIT_5 = 0x10, //00010000
        BIT_6 = 0x20, //00100000
        BIT_7 = 0x40, //01000000
        BIT_8 = 0x80  //10000000
    };


    GrayCodeModule(Datapool *i_data);
    ~GrayCodeModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    bool first;
    void getGray(QImage *);
    static uchar getGrayCode(uchar &input);
};

#endif // GrayCodeModule_H
