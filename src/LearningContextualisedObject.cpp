#include <map>
#include <deque>
#include <string>
#include "LearningContextualisedObject.h"
#include "LearningContext.h"

namespace miles {

  LevelData::LevelData():currentState(-1), timeStayingAtCurrent(0.0) {}

  LevelData::~LevelData() { }

  ContextualisedNumericalData::ContextualisedNumericalData(double i_value, double i_reliability): value(i_value), reliability(i_reliability) {}

  ContextualisedNumericalData::~ContextualisedNumericalData() {}


  ContextualisedSymbolicData::ContextualisedSymbolicData(std::string i_value, double i_reliability): value(i_value), reliability(i_reliability) {}

  ContextualisedSymbolicData::~ContextualisedSymbolicData() {}


  //    void print(SpCLearningAttributes availableAttributes);
  LearningContextualisedObject::LearningContextualisedObject(unsigned long object_id): numLostFrames(0), numFrames(0), trackedObjectId(object_id) {}

  LearningContextualisedObject::~LearningContextualisedObject() {}
  
  void LearningContextualisedObject::processLearningContext(SpMObject object, int lc_id, SpLearningContext lcontext, SpLearningAttributes availableAttributes, Datapool &datapool) {

    int i, n = lcontext->numericalAttributes.size();
    attribute_id id;
    std::map<attribute_id, SpLearningNumeric>::iterator LCnumIt = lcontext->numericalAttributes.begin();

    for(i = 0; i < n; i++, LCnumIt++) {
      id = (*LCnumIt).first;
      SpContextualisedNumericalData numeric(new ContextualisedNumericalData(availableAttributes->getValueNumeric[id](object, datapool), availableAttributes->getReliability[id](object, datapool)));
      numericalData[lc_id][id] = numeric;
    }

    n = lcontext->symbolicAttributes.size();
    std::map<attribute_id, SpLearningSymbolic>::iterator LCsymIt = lcontext->symbolicAttributes.begin();

    for(i = 0; i < n; i++, LCsymIt++) {
      id = (*LCsymIt).first;
      SpContextualisedSymbolicData symbolic(new ContextualisedSymbolicData(availableAttributes->getValueSymbolic[id](object, datapool), availableAttributes->getReliability[id](object, datapool)));
      symbolicData[lc_id][id] = symbolic;
    }

  }


  void LearningContextualisedObject::updateConcernedLearningContexts(SpMObject object, SpLearningAttributes availableAttributes, Datapool &datapool) {
    int i, n = numericalData.size(), j, m;
    int id;
    attribute_id att_id;
    std::map<int, std::map<attribute_id, SpContextualisedNumericalData > >::iterator LCnumIt = numericalData.begin();
    std::map<int, std::map<attribute_id, SpContextualisedSymbolicData > >::iterator LCsymIt  = symbolicData.begin();
    std::map<attribute_id, SpContextualisedNumericalData >::iterator numIt;
    std::map<attribute_id, SpContextualisedSymbolicData >::iterator symIt;
    SpContextualisedNumericalData numeric;
    SpContextualisedSymbolicData symbolic;

    for(i = 0; i < n; i++, LCnumIt++) {
      id = (*LCnumIt).first;
      m = (*LCnumIt).second.size();
      numIt = (*LCnumIt).second.begin();
      for(j = 0; j < m; j++, numIt++) {
	att_id = (*numIt).first;
	(*numIt).second->value = availableAttributes->getValueNumeric[att_id](object, datapool);
	(*numIt).second->reliability = availableAttributes->getReliability[att_id](object, datapool);
      }
    }

    n = symbolicData.size();

    for(i = 0; i < n; i++, LCsymIt++) {
      id = (*LCsymIt).first;
      m = (*LCsymIt).second.size();
      symIt = (*LCsymIt).second.begin();
      for(j = 0; j < m; j++, symIt++) {
	att_id = (*symIt).first;
	(*symIt).second->value = availableAttributes->getValueSymbolic[att_id](object, datapool);
	(*symIt).second->reliability = availableAttributes->getReliability[att_id](object, datapool);
      }
    }

  }

  
} // end namespace
