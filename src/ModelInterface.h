#ifndef _MODEL_INTERFACE_H_
#define _MODEL_INTERFACE_H_

#include "blob.h"
#include "GaussianFunction.h"
#include <QDomNode>

class Datapool;
class ModelInterface;
class SceneModel;

typedef QSharedPointer<ModelInterface> SpModelInterface;

class Criterion {
 public:
  Criterion();
  ~Criterion();
  SpGaussianFunction spFunction;
  double score;
  double reliability;
};

class ModelInterface {
  
 public :
        bool velocity_defined;
        bool IsRigid;
        std::map<QString, Criterion> m_mapCriteria;
        std::map<ObjectSubtype, SpModelInterface > m_mapPostures;

        ModelInterface();
        ModelInterface(Datapool *i_data);
        ~ModelInterface();
  
        //bool hasCriterion(const QString& i_criterionName);
        //double getMinForCriterion(const QString& i_criterionName);
        //double getMaxForCriterion(const QString& i_criterionName);
        //double getMeanForCriterion(const QString& i_criterionName);

        double computeScore(Blob *i_blob, SceneModel *smodel);

        //double computeScoreForCriterion(Blob *i_blob, SceneModel *smodel, const std::string& i_criterionName);


        void computeCriteriaScores(Blob *blob, SceneModel *smodel);

        void computeReliabilities(Blob *blob, SceneModel *smodel);

        void setParameters(QDomNode &i_parameters);

        void addSubModel(ObjectSubtype i_subType, SpModelInterface i_spModel);

 protected:
        Datapool *m_data;
};


#endif


