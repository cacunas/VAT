#ifndef LIGHTSOURCE_H
#define LIGHTSOURCE_H

#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilitysinglemodelinterface.h"
#include "src/blob.h"
#include "Blob2DFromBGSubstractionModel.h"

class LightSourceInstance: public Blob2DFromBGSubstractionModelInstance {
public:
    LightSourceInstance(Datapool *i_data);

    ~LightSourceInstance();

    //Initializes the visible model attributes for the interface.
    //The function is called by the interface constructor
    void initAttributes();

    //sets activation criteria: reliability on input (distance, bad data),
    //   needs (occlusion, priority),
    //sets priority of models (hierarchy).
    void setParameters(QDomNode &i_parameters);

    //sets activation flags for each model, according to the current context
    void activate(Blob *);

    //updates the model
    void update(Blob *);

    //sets blob enclosing the model.
    void setBlobInterface();

    //Visualization

    void printAttributes();

    void printGeneral();

    bool draw(QPainter &painter);

    bool draw(QPainter &painter, int r, int g, int b);

};


#endif // LIGHTSOURCE_H
