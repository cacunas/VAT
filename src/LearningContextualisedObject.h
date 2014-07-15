#ifndef LEARNING_CONTEXTUALISED_OBJECT_H
#define LEARNING_CONTEXTUALISED_OBJECT_H

#include "LearningAttributesSpecification.h"
#include <map>
#include <deque>
#include <string>

namespace miles {

  class ContextualisedNumericalData {
  public:
    double value;
    double reliability;
    ContextualisedNumericalData(double i_value, double i_reliability);
    ~ContextualisedNumericalData();
  };

  class ContextualisedSymbolicData {
  public:
    std::string value;
    double reliability;
    ContextualisedSymbolicData(std::string i_value, double i_reliability);
    ~ContextualisedSymbolicData();
  };

  class LevelData {
  public:
    long int currentState;
    double timeStayingAtCurrent;
    int analysedFrames;
    std::deque<long int> lastEvents;
    //EVENT
    LevelData();
    ~LevelData();
  };
  
  class LearningContextualisedObject {
  public:
    std::map<int, std::map<int, SpLevelData> > levelData;
    std::map<int, std::map<int, SpLevelData> > lastLevelData;
    int lastUpdatedFrame;

    std::map<int, std::map<attribute_id, SpContextualisedNumericalData > > numericalData;
    std::map<int, std::map<attribute_id, SpContextualisedSymbolicData > > symbolicData;
    std::deque<int> activeContexts;
    int numLostFrames;
    int numFrames;
    unsigned long trackedObjectId;

    void processLearningContext(SpMObject object, int lc_id, SpLearningContext lcontext, SpLearningAttributes availableAttributes, Datapool &datapool);
    void updateConcernedLearningContexts(SpMObject object, SpLearningAttributes availableAttributes, Datapool &datapool);

    LearningContextualisedObject(unsigned long object_id);
    ~LearningContextualisedObject();

  };
  
} // end namespace

#endif	
