#include "IncrementalEventLearningModule.h"
#include "src/LearningContext.h"
#include "src/HierarchyTree.h"
#include "src/LearningContextualisedObject.h"
#include "src/LearningConcept.h"

    IncrementalEventLearningModule::IncrementalEventLearningModule(Datapool *d):ModuleInterface(d) {
        SpIncrementalEventLearning newLearning(new IncrementalEventLearning(d));
        m_EventLearning = newLearning;
    }

    IncrementalEventLearningModule::~IncrementalEventLearningModule() {}

    bool IncrementalEventLearningModule::run() {
        m_EventLearning->setContextualisedObjects(*this->m_data);
        m_EventLearning->run(this->m_data->currentHeader->ts_diff);

        recogniseEvents();

        return true;
    }

    bool IncrementalEventLearningModule::init() {
      m_data->eventLearning = m_EventLearning;
      m_data->availableAttributes = m_EventLearning->availableAttributes;
      return true;
    }

    bool IncrementalEventLearningModule::setParameters(QDomNode &config) {
        m_EventLearning->init();
        if(m_EventLearning->setParameters(config) == false)
            return false;

        m_EventLearning->setLearningContexts();

        return true;
    }

    bool IncrementalEventLearningModule::updateParameters(){
        return true;
    }

    double IncrementalEventLearningModule::getProbabilityRecognition(SpLearningContext context, SpLearningContextualisedObject currentObject, long int stateId) {
        SpLearningStateConcept state = context->eventsHierarchy->registeredStates[stateId];
        double Pr = 0.0, Rsum = 0.0, value, reliability;
        int i, I, m;

        if( (I = state->numericalAttributeList.size()) > 0) {
            std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = state->numericalAttributeList.begin();
            for(i = 0; i < I; i++, numericalIt++) {
                value = currentObject->numericalData[0][(*numericalIt).first]->value;
                reliability = currentObject->numericalData[0][(*numericalIt).first]->reliability;
                Pr += (*numericalIt).second->distribution->getNormalisedValue(value) * reliability;
                Rsum += reliability;
            }
        }

        if( (I = state->symbolicAttributeList.size()) > 0) {
            std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = state->symbolicAttributeList.begin();
            std::map<std::string, SpSymbolicValue>::iterator values;
            SpSymbolicAttributeValuePair currentSym;

            for(i = 0; i < I; i++, symbolicIt++) {
                currentSym = (*symbolicIt).second;
                values = currentSym->values.begin();
                m = currentSym->values.size();
                value = currentSym->values[currentObject->symbolicData[0][(*symbolicIt).first]->value]->P;
                reliability = currentObject->symbolicData[0][(*symbolicIt).first]->reliability;

                Pr +=  value * reliability;
                Rsum += reliability;
            }
        }
        return Pr / Rsum;
    }

    void IncrementalEventLearningModule::printState( std::ostream& out, SpLearningStateConcept state, int parent, SpLearningAttributes availableAttributes, int totalN){

        char value[100];
        std::string toString;

        out << "\t\t\t<STATE";
        sprintf(value,"%d",parent);
        toString=value;
        out << " PARENT=\"" << toString << "\"";

        sprintf(value,"%d",(int)state->getID());
        toString=value;
        out << " ID=\"" << toString << "\"";

        sprintf(value,"%.4f", (double)state->getN() / (double)totalN);
        toString=value;
        out << " P=\"" << toString << "\"";

        sprintf(value,"%d",state->getN());
        toString=value;
        out << " N=\"" << toString << "\"";

        sprintf(value,"%d",state->NE);
        toString=value;
        out << " NE=\"" << toString << "\"";

        out << ">\n";

        int i, I;

        if( (I = state->numericalAttributeList.size()) > 0) {
            std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = state->numericalAttributeList.begin();
            for(i = 0; i < I; i++, numericalIt++) {
                out << "\t\t\t\t<ATTRIBUTE";
                sprintf(value,"%s", availableAttributes->nameFromId[(*numericalIt).first].c_str());
                toString=value;
                out << " NAME=\"" << toString << "\"";
                out << " TYPE=\"NUMERIC\"";
                sprintf(value,"%.4f", (*numericalIt).second->getMean());
                toString=value;
                out << " MEAN=\"" << toString << "\"";
                sprintf(value,"%.4f", (*numericalIt).second->getSigma());
                toString=value;
                out << " SIGMA=\"" << toString << "\"";
                sprintf(value,"%.4f", (*numericalIt).second->RSum);
                toString=value;
                out << " RSUM=\"" << toString << "\"";
                out << "/>\n";
            }
        }

        if( (I = state->symbolicAttributeList.size()) > 0) {
            std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = state->symbolicAttributeList.begin();
            int j, m;
            std::map<std::string, SpSymbolicValue>::iterator values;
            SpSymbolicAttributeValuePair currentSym;

            for(i = 0; i < I; i++, symbolicIt++) {
                currentSym = (*symbolicIt).second;

                out << "\t\t\t\t<ATTRIBUTE";

                sprintf(value,"%s", availableAttributes->nameFromId[(*symbolicIt).first].c_str());
                toString=value;
                out << " NAME=\"" << toString << "\"";
                out << " TYPE=\"SYMBOLIC\"";
                sprintf(value,"%.4f", currentSym->RSum);
                toString=value;
                out << " RSUM=\"" << toString << "\"";
                out << ">\n";

                values = currentSym->values.begin();
                m = currentSym->values.size();

                for(j = 0; j < m; j++, values++) {
                    out << "\t\t\t\t\t<VALUE";

                    sprintf(value,"%s", (*values).first.c_str());
                    toString=value;
                    out << " NAME=\"" << toString << "\"";
                    sprintf(value,"%d", (*values).second->N);
                    toString=value;
                    out << " N=\"" << toString << "\"";
                    sprintf(value,"%.4f", (*values).second->RSum);
                    toString=value;
                    out << " RSUM=\"" << toString << "\"";
                    out << "/>\n";
                }
                out << "\t\t\t\t</ATTRIBUTE>\n";
            }
        }
        out << "\t\t\t</STATE>\n";
    }

    void IncrementalEventLearningModule::displayStateReducedInfo( std::ostream& out, SpLearningStateConcept state, int totalN) {
        SpLearningAttributes availableAttributes = m_EventLearning->availableAttributes;

        out << "\t\tP = " << (double)state->N / (double)totalN << std::endl;
        printState(out, state,0, availableAttributes, totalN);
    }

    void IncrementalEventLearningModule::recogniseEvents() {
        double tdiff = m_data->currentHeader->ts_diff;
        std::map<unsigned long, SpLearningContextualisedObject>::iterator followedIt = m_EventLearning->followedObjects.begin();
        int i, j, n, m, l = m_EventLearning->followedObjects.size();
        unsigned long id;
        SpLearningContextualisedObject currentObject;

        for(i=0; i<l;i++, followedIt++) {
            id = (*followedIt).first;

            //Set object ids of interest
            currentObject = (*followedIt).second;
            if(currentObject->lastUpdatedFrame != m_data->frameNumber)
                continue;
            if(m_data->levelData.count(id) == 0) { //If the id appears for the first time
                std::map<int, SpLevelData>*levels = &currentObject->levelData[0]; //Just consider first context
                std::map<int, SpLevelData> newLevelData = *levels;
                std::map<long int, double> newPrData;
                std::map<int, SpLevelData>::iterator levelIt = levels->begin();
                m = levels->size();
                for(j=0; j<m; j++, levelIt++) {
                    SpLevelData newData(new LevelData());
                    newData->currentState = (*levelIt).second->currentState;
                    newData->timeStayingAtCurrent = tdiff / 2.0; //As it is starting, we do not know the exact starting moment.
                    newData->analysedFrames = 1;
                    newPrData[newData->currentState] = getProbabilityRecognition(m_EventLearning->validContexts[0], currentObject, newData->currentState);
                    newLevelData[(*levelIt).first] = newData;
                }
                m_data->levelData[id] = newLevelData;
                m_data->Pr[id] = newPrData;
            } else { //Exists already, check changes and store them in a file.
                std::map<int, SpLevelData>*levels = &(m_data->levelData[id]), *currentLevels = &currentObject->levelData[0]; //Just consider first context
                std::map<int, SpLevelData>::iterator levelIt = levels->begin(), currentLevelIt;
                n = levels->size();
                m = currentLevels->size();
                long int previousStateId, currentStateId;
                bool isThere;
                double finalStayingTime, finalPr;
                int k, finalNumFrames, highest = (n > m)? n+1 : m+1;

                //set_the_highest level change
                if(m <= n) {
                    currentLevelIt = currentLevels->begin();
                    for(k=0; k<m; k++, levelIt++, currentLevelIt++) {
                        previousStateId = (*levelIt).second->currentState;
                        currentStateId = (*currentLevelIt).second->currentState;
                        if(currentStateId != previousStateId) { //If state is in the list, there is no event yet for this level
                            highest = k;
                            break;
                        }
                    }
                } else if (m > n) {
                    currentLevelIt = currentLevels->begin();
                    for(k=0; k < n; k++, levelIt++, currentLevelIt++) {
                        previousStateId = (*levelIt).second->currentState;
                        currentStateId = (*currentLevelIt).second->currentState;
                        if(currentStateId != previousStateId) { //If state is in the list, there is no event yet for this level
                            highest = k;
                            break;
                        }
                    }
                }

                levelIt = levels->begin();

                for(k=0; k<n; k++, levelIt++) {
                    previousStateId = (*levelIt).second->currentState;
                    currentLevelIt = currentLevels->begin();
                    isThere = false;
                    for(j=0; j<m; j++, currentLevelIt++) {
                        currentStateId = (*currentLevelIt).second->currentState;
                        if(currentStateId == previousStateId) { //If state is in the list, there is no event yet for this level
                            isThere = true;
                            break;
                        }
                    }

                    //If previous is not there, display the events associated to this starting state, depassing the minimal threshold of permanence
                    if(!isThere) {
                        currentLevelIt = currentLevels->begin();
                        finalStayingTime = (*levelIt).second->timeStayingAtCurrent + tdiff/2.0;

                        if(finalStayingTime > 1.0) { //If the final staying time is of more than ONE second, consider this state interesting

                            finalNumFrames = (*levelIt).second->analysedFrames;
                            finalPr = m_data->Pr[id][previousStateId] / (double) finalNumFrames;
//                            SpLearningStateConcept previousState = m_EventLearning->validContexts[0]->eventsHierarchy->registeredStates[previousStateId];
//                            CU = m_EventLearning->validContexts[0]->eventsHierarchy->categoryUtilityState(previousState);
                            SpLearningStateConcept currentState;
                            SpLearningEventConcept currentEvent;

                          //Display previous state Info
                          //std::cout  << std::endl << "Transition Found for Object " << id << " At Frame " << numFrames << " for Hierarchy Level " << k << " From: " << std::endl;
                          //displayStateInfo(std::cout, previousState, totalN, numFrames - finalNumFrames, finalNumFrames, finalStayingTime, finalPr, CU);

                            for(j=0; j<m; j++, currentLevelIt++) {
                                currentStateId = (*currentLevelIt).second->currentState;
                                currentState = m_EventLearning->validContexts[0]->eventsHierarchy->registeredStates[currentStateId];

                                if(currentState->incomingEvents.count(previousStateId)) { //If it is found, it must have been a transition from previous, so show
                                    currentEvent = currentState->incomingEvents[previousStateId];
/*                                    std::cout << "\tWith Event " << currentEvent->id << ":" << std::endl;
                                    std::cout << "\t\tN = " << currentEvent->N << std::endl;
                                    std::cout << "\t\tP = " << currentEvent->P << std::endl;
                                    std::cout << "\t\tmuT = " << currentEvent->getMean() << std::endl;
                                    std::cout << "\t\tsigmaT = " << currentEvent->getSigma() << std::endl;
                                    std::cout << "\t\tminT = " << currentEvent->minT << std::endl;
                                    std::cout << "\t\tmaxT = " << currentEvent->maxT << std::endl;
                                    std::cout << "\tAnd Arriving State " << currentState->id << ":" << std::endl; */
                                    //displayStateReducedInfo(std::cout, currentState, totalN);
                                }
                            }
                        }
                        m_data->Pr[id].erase(previousStateId);
                    } else if(m_EventLearning->validContexts[0]->eventsHierarchy->registeredStates.count(previousStateId) == 0)
                        m_data->Pr[id].erase(previousStateId);
                    }

                    //Update List of state per level
                    std::map<int, SpLevelData> newLevelData = *currentLevels;
                    currentLevelIt = currentLevels->begin();
                    for(j=0; j<m; j++, currentLevelIt++) {
                        SpLevelData newData(new LevelData());
                        newData->currentState = (*currentLevelIt).second->currentState;
                        newData->timeStayingAtCurrent = (*currentLevelIt).second->timeStayingAtCurrent; //As it is starting, we do not know the exact starting moment.
                        newData->analysedFrames = (*currentLevelIt).second->analysedFrames;
                        if(m_data->Pr[id].count(newData->currentState) == 0)
                            m_data->Pr[id][newData->currentState] = getProbabilityRecognition(m_EventLearning->validContexts[0], currentObject, newData->currentState);
                        else
                            m_data->Pr[id][newData->currentState] += getProbabilityRecognition(m_EventLearning->validContexts[0], currentObject, newData->currentState);
                        newLevelData[(*currentLevelIt).first] = newData;
                        //newLevelData[(*levelIt).first] = newData;
                    }
                    m_data->levelData[id] = newLevelData;
                }
            }
    }


