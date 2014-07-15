#ifndef RELIABILITYSINGLEMODELINTERFACE_H
#define RELIABILITYSINGLEMODELINTERFACE_H

#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilityattribute.h"
#include "src/blob.h"

class Datapool;

class RMMMobileObject;

class ModelInstanceDataPack {
public:
    std::map <QString, AttributeDupletValue> attributes;
    std::map <QString, AttributeDupletImage> attributes_image;
    std::map <QString, AttributeDupletDataset> attributes_dataset;
    DetectionProblemType dp_type;
};

class ModelDynamicsDataPack {
public:
    std::map <QString, ReliabilityDynamicsAttribute> dynamics;
    std::map <QString, ReliabilityDynamicsAttributeDataset> dynamics_dataset;
};

class ModelBuffer {
public:
    ModelBuffer();
    ~ModelBuffer();
    std::deque<ModelInstanceDataPack>::iterator begin();
    std::deque<ModelInstanceDataPack>::iterator end();
    std::deque<ModelInstanceDataPack>::reverse_iterator rbegin();
    std::deque<ModelInstanceDataPack>::reverse_iterator rend();
    ModelInstanceDataPack &operator[](int);
    unsigned int size();
    ModelInstanceDataPack &back();
    ModelInstanceDataPack &front();
    void clear();
    void insert(ModelInstanceDataPack *);
    void insert(ModelInstanceDataPack &pack);
    void resetMaxLength(int);
    void copyModels(ModelBuffer *);

    friend std::ostream& operator<<(std::ostream&,const ModelBuffer&);

    std::deque<ModelInstanceDataPack> instances;
    unsigned int maxLength;
};

class ReliabilitySingleModelInterface;

typedef QSharedPointer<ReliabilitySingleModelInterface> SpReliabilitySingleModelInterface;

class ReliabilitySingleModelInterface
{
public:
    ReliabilitySingleModelInterface();

    //IMPLEMENT!!
    virtual ~ReliabilitySingleModelInterface();

    void initFromXML(Datapool *i_data, int bsize, QString name);

    //IMPLEMENT!! Initializes the model attributes and dynamics for the interface.
    //The function is called by the interface constructor
    virtual void initInstanceAttributes() = 0;

    //IMPLEMENT!! Initializes the visible model attributes for the interface.
    //The function is called by the interface constructor
    virtual void initDynamicsAttributes() = 0;

    //Copy general structures and call specific copy function
    void general_copy(SpReliabilitySingleModelInterface);

    //IMPLEMENT!! Copy function for specific data from the instance (general_copy calls it after copying base info)
    virtual void copy(SpReliabilitySingleModelInterface);

    //Copy general structures
    void copy_general_structure(SpReliabilitySingleModelInterface);

    //IMPLEMENT!! Copy general structures and parameters of specific model
    virtual void copy_structure(SpReliabilitySingleModelInterface);

    //IMPLEMENT!! sets parameters from xml file
    virtual void setParameters(QDomNode &i_parameters) = 0;

    //IMPLEMENT!! sets activation criteria for each model: reliability on input (distance, bad data),
    //   needs (occlusion, priority),
    //sets priority of models (hierarchy).
    //Checked before update phase
    virtual void activate(Blob *) = 0;

    //IMPLEMENT!! initializes every activated model, according to their own input (region-blob-segments schema)
    //Executed only once at the begginning of single model creation (after 'copy_structure' and 'copy' methods
    //are used to copy the information).
    virtual void init() = 0;

    //IMPLEMENT!! updates activated model instance attributes for current frame
    virtual void updateInstance(Blob *) = 0;

    //IMPLEMENT!! updates every activated model dynamics
    virtual void updateDynamics() = 0;

    //IMPLEMENT!! feedback process for depurating information based on dependant models.
    //It should be programmed checking the presence of different models that can
    //improve the estimation of model attributes.
    virtual void feedback() = 0;

    //Visualization
    virtual void printInstances();

    virtual void printDynamics();

    virtual void printGeneral();

    virtual void printGeneral(std::ostream& out);

    virtual bool draw(QImage *image);

    virtual bool draw(QPainter &painter, int r, int g, int b);

    //sets global probability P for the model (how good is the model)
    //sets global reliability R for the model (how good is the data used in the model)
    virtual void setProbabilityAndReliability() = 0;

    //sets blob enclosing the model in forward process (update)
    virtual void setForwardBlobInterface() = 0;

    //sets blob enclosing the model in backward process (feedback)
    virtual void setBackwardBlobInterface() = 0;

    //Get 2D central bounding box point from the model
    virtual void setTrajectory2D(double &, double &, double &);

    //Estimates the bounding box size and position for next
    //frame according to its own dynamics model
    //It also sets Reliability R of the model estimation.
    //The function defaults to the 'binterface' bounding box.
    //Nevertheless, a better implementation really depending on the
    //specific dynamics model is expected.
    virtual Rectangle<int> getEstimator(double &R);

    static double normalisedSigmaCoherenceReliability(double sigma, double acuity);

    //Tag name
    QString name;

    //Model class name
    QString model_name;

    //Link to datapool
    Datapool *m_data;

    //Flag for activation according to different criteria
    bool active;

    //Global probability and reliability for model
    double P, R_forward, R_backward;

    //Reference to the associated mobile object
    RMMMobileObject *m_mobile;

    //Attributes

    //1. Instance buffer
    ModelBuffer instances;

    ModelInstanceDataPack instanceTemplate;

    //2. Dynamics model
    ModelDynamicsDataPack dynamics;

    //Output bounding box and information after update (forward) process
    Blob binterface_forward;

    //Output bounding box and information after feedback (backward) process
    Blob binterface_backward;

    //Access to dependences, or dependants
    std::deque<SpReliabilitySingleModelInterface> dependences;
    std::deque<SpReliabilitySingleModelInterface> dependants;

};


#endif // RELIABILITYSINGLEMODELINTERFACE_H
