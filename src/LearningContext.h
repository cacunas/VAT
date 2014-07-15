#ifndef LEARNING_CONTEXT_H
#define LEARNING_CONTEXT_H

#include "LearningAttributesSpecification.h"
#include "common.h"
#include <map>
#include <deque>
#include <string>
#define DEFAULT_ACUITY 1.0

namespace miles {
  
  class LearningNumeric {
  public:
    attribute_id att_id;
    double acuity;
    LearningNumeric(attribute_id i_att_id, double i_acuity);
    ~LearningNumeric();
  };

  typedef QSharedPointer<LearningNumeric> SpLearningNumeric;

  class LearningSymbolic {
  public:
    attribute_id att_id;

    LearningSymbolic(attribute_id i_att_id);
    ~LearningSymbolic();
  };


  typedef QSharedPointer<LearningSymbolic> SpLearningSymbolic;

  class LearningContext {
  public:
    int id;
    SpHierarchyTree eventsHierarchy;
    
    QString name;
    QString initialHierarchyFile;
    std::map<unsigned long, SpLearningContextualisedObject> contextualisedObjects;

    std::set<ObjectType> involvedObjects;
    std::map<attribute_id, SpLearningNumeric> numericalAttributes;
    std::map<attribute_id, SpLearningSymbolic> symbolicAttributes;

    void print(SpLearningAttributes availableAttributes);
    void cleanInvolvedList();
    void generateGnuPlot(std::string filename, int max_distance_to_leaf);

    LearningContext();
    ~LearningContext();

  };
} // end namespace

#endif	
