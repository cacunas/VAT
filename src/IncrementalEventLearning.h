#ifndef INCREMENTALEVENTLEARNING_H
#define INCREMENTALEVENTLEARNING_H

#include <QDomNode>
#include <map>
#include <deque>
#include <string>
#include <QFile>

#include "LearningAttributesSpecification.h"

#define MAX_LEN_BUFFER 512

namespace miles {

class IncrementalEventLearning {
  public:
    static double minConditionalP;
    Datapool *m_data;
    QString m_learningContextsFile;
    QString m_hierarchyStatsFile;
    QFile m_statsFile;
    bool m_activateStats;
    int nValidContexts;
    SpLearningAttributes availableAttributes;
    std::map<int, SpLearningContext> validContexts;
    std::map<unsigned long, SpLearningContextualisedObject> followedObjects;
    std::map<unsigned long, bool> presentInThisFrame;
    SpXMLTreeGenerator treeGenerator;
    bool first;
    IncrementalEventLearning(Datapool *i_data);
    ~IncrementalEventLearning();
    void setContextualisedObjects(Datapool &datapool);
    void run(double);
    void init();
    bool setParameters(QDomNode &config);
    void setLearningContexts();

    bool invalidNumericNameForContext(std::string name, SpLearningContext context);
    bool invalidSymbolicNameForContext(std::string name, SpLearningContext context);
    bool invalidValueNameForContext(attribute_id att_id, std::string name);

    SpLearningStateConcept createNewNode(QDomNode& config, SpLearningContext context, long int &free_state_id);
    SpHierarchyTree generateTree(QString filename, SpLearningContext context);

    QDomDocument *xmlConfig;
    QDomElement rootConfig;

  };

  typedef QSharedPointer<IncrementalEventLearning> SpIncrementalEventLearning;

} // end namespace miles

#endif	
