#ifndef INCREMENTAL_EVENT_LEARNING_MODULE_H
#define INCREMENTAL_EVENT_LEARNING_MODULE_H

#include "src/IncrementalEventLearning.h"
#include "ModuleInterface.h"

using namespace miles;

//#define ELM_OUTPUT

class IncrementalEventLearningModule : public ModuleInterface  {
public:
    /*
    IncrementalEventLearningModule IncrementalEventLearningModule.h
    This is a module for Incremental Event Learning using the Component CIncrementalEventLearning.
    */
        SpIncrementalEventLearning m_EventLearning;

    IncrementalEventLearningModule(Datapool *d);
    ~IncrementalEventLearningModule();
	
    static double getProbabilityRecognition(SpLearningContext context, SpLearningContextualisedObject currentObject, long int stateId);
    static void printState( std::ostream& out, SpLearningStateConcept state, int parent, SpLearningAttributes availableAttributes, int totalN);

    void displayStateReducedInfo( std::ostream& out, SpLearningStateConcept state, int totalN);
    void recogniseEvents();

    bool run();

    bool init();

    bool setParameters(QDomNode &config);

    bool updateParameters();
};

#endif
