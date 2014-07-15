#ifndef RELIABILITYMULTIMODEL_H
#define RELIABILITYMULTIMODEL_H

//#include "Datapool.h"
#include "reliabilitysinglemodelinterface.h"

class Blob;

class RMMMobileObject;

class ReliabilityMultiModel;

class Datapool;

typedef QSharedPointer<ReliabilityMultiModel> SpReliabilityMultiModel;

class ReliabilityMultiModel {

public:
    ReliabilityMultiModel();
    ReliabilityMultiModel(Datapool *i_data);
    ReliabilityMultiModel(ReliabilityMultiModel *model);
    ~ReliabilityMultiModel();
    //sets activation flags for each model, according to the current context
    void activateModels();
    void activateModels(std::deque<SpReliabilitySingleModelInterface> &);

    //Initializes owner mobile object pointer for every model.
    void init(RMMMobileObject *);
    void init(RMMMobileObject *, std::deque<SpReliabilitySingleModelInterface> &);

    //Gets last Model instance according to its name, else return spointer, with spointer.isNull() == true.
    SpReliabilitySingleModelInterface getSingleModelByName(QString name);
    SpReliabilitySingleModelInterface getSingleModelByName(QString name,
                                                           std::deque<SpReliabilitySingleModelInterface> &models);

    //Returns true if it finds a Model instance according to its name, else returns false.
    bool findSingleModelByName(QString name);
    bool findSingleModelByName(QString name, std::deque<SpReliabilitySingleModelInterface> &models);

    //forward process of models, starting with non-dependant models.
    void update(Blob *blob);

    //backward process of models, starting with last models, with no associated
    //dependant models.
    void feedback();


    //Estimator of position of visual evidence
    Rectangle<int> getVisualEvidenceEstimator(double &R);

    //Starts processing of parameters (only for multimodel template)
    bool setParameters(QDomDocument &);

    //Process names for model and model class. Checks availability of classes and unicity of names.
    // (only for multimodel template)
    bool setModelList(QDomElement &parent);

    //Pre-loaded lists for backward and forward independent models
    //in DAG.
    void setIndependentLists();

    //Process dependences for each model and checks DAG constraint, using topological ordering.
    // (only for multimodel template)
    bool checkDependences(std::map<QString, std::set<QString> > &dependences, std::deque<QString> &name_list);
    void setDependences(std::map<QString, std::set<QString> > &dependences, std::deque<QString> &ordered_list);

    void getGlobalProbability(double &R, double &P);

    void set2DTrajectoryPoint(double &x, double &y, double &R);

    Datapool *m_data;

    std::deque<SpReliabilitySingleModelInterface> multiModelDAG;
    std::set<QString> forwardIndependentModels;
    std::set<QString> backwardIndependentModels;
//    bool active;
    Blob binterface;
//    double P;
//    double R;

    double currentTimeDiffSeconds;
    //Reference to the associated mobile object
    RMMMobileObject *mobile;
};

typedef QSharedPointer<ReliabilityMultiModel> SpReliabilityMultiModel;


#endif // RELIABILITYMULTIMODEL_H
