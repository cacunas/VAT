#ifndef BLOB2DFROMBGSUBSTRACTIONMODEL_H
#define BLOB2DFROMBGSUBSTRACTIONMODEL_H

#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilitysinglemodelinterface.h"
#include "src/blob.h"

class Datapool;

class Blob2DFromBGSubstractionModel: public ReliabilitySingleModelInterface {
public:
    Blob2DFromBGSubstractionModel();

    virtual ~Blob2DFromBGSubstractionModel();

    //IMPLEMENT!! Initializes the model attributes and dynamics for the interface.
    //The function is called by the interface constructor
    virtual void initInstanceAttributes();

    //IMPLEMENT!! Initializes the visible model attributes for the interface.
    //The function is called by the interface constructor
    virtual void initDynamicsAttributes();

    //IMPLEMENT!! Copy function for specific data from the instance (general_copy calls it after copying base info)
    virtual void copy(SpReliabilitySingleModelInterface);

    //IMPLEMENT!! Copy general structures and parameters of specific model
    virtual void copy_structure(SpReliabilitySingleModelInterface);

    //IMPLEMENT!! sets parameters from xml file
    virtual void setParameters(QDomNode &i_parameters);

    //IMPLEMENT!! sets activation criteria for each model: reliability on input (distance, bad data),
    //   needs (occlusion, priority),
    //sets priority of models (hierarchy).
    //Checked before update phase
    virtual void activate(Blob *);

    //IMPLEMENT!! initializes every activated model, according to their own input (region-blob-segments schema)
    //Executed only once at the begginning of single model creation (after 'copy_structure' and 'copy' methods
    //are used to copy the information).
    virtual void init();

    //IMPLEMENT!! updates activated model instance attributes for current frame
    virtual void updateInstance(Blob *);

    //IMPLEMENT!! updates every activated model dynamics
    virtual void updateDynamics();

    //IMPLEMENT!! feedback process for depurating information based on dependant models.
    //It should be programmed checking the presence of different models that can
    //improve the estimation of model attributes.
    virtual void feedback();

    //IMPLEMENT!! sets global probability for the model (how good is the model)
    virtual void setProbabilityAndReliability();

    //IMPLEMENT!! sets blob enclosing the model in forward process (update)
    virtual void setForwardBlobInterface();

    //IMPLEMENT!! sets blob enclosing the model in backward process (feedback)
    virtual void setBackwardBlobInterface();

    //OPTIONAL
    virtual bool draw(QPainter &painter);

    virtual bool draw(QPainter &painter, int r, int g, int b);

    //Initialize dynamics attribute with instance value
    void initAttribute(ReliabilityDynamicsAttribute &d, AttributeDupletValue &i);

    //Attribute update, with minimal attribute value constraint
    void updateMinConstrainedAttribute(const QString &name, ReliabilityDynamicsAttribute &d, double min);

    //Standard attribute update, with no attribute value constraints
    void updateAttribute(const QString &name, ReliabilityDynamicsAttribute &d);

    void updateAttributeDynamics(double data, double RData, double acuity,
                                 double estimate, double currentCooling, AttributeTripletValue &att);

    void updateAttributeDynamicsNoData(double estimate, double acuity, double currentCooling,
                                       AttributeTripletValue &att);

    double projectAttributeValue(ReliabilityDynamicsAttribute &, double &R);

    virtual Rectangle<int> getEstimator(double &R);

    double m_minimalAttributeVelocityReliability;
    double m_minimalAttributeAccelerationReliability;
    double m_minimalWidth;
    double m_minimalHeight;

    int m_pixelAcuity;
};

typedef QSharedPointer<Blob2DFromBGSubstractionModel> SpBlob2DFromBGSubstractionModel;


#endif // BLOB2DFROMBGSUBSTRACTIONMODEL_H
