#include "HierarchyTree.h"
#include "LearningConcept.h"
#include "LearningContext.h"
#include "LearningContextualisedObject.h"
#include <cmath>
#include <cstdlib>
#include <limits.h>

#include <iostream>


namespace miles {

Instance::Instance(): numericalData(NULL), symbolicData(NULL) {}

Instance::~Instance() {}

void Instance::setInstance(std::map<attribute_id, SpContextualisedNumericalData > *i_numericalData,
                           std::map<attribute_id, SpContextualisedSymbolicData > *i_symbolicData) {
    numericalData = i_numericalData;
    symbolicData = i_symbolicData;
}

SpLearningAttributes nullOne;
SpLearningAttributes HierarchyTree::availableAttributes = nullOne;
double HierarchyTree::minConditionalP = 0.1;

HierarchyTree::HierarchyTree():  deepest_level(-1), free_state_id(0), free_event_id(0) {
    SpInstance newInstance(new Instance());
    conceptsHierarchy.clear();
    currentInstance = newInstance;
}

HierarchyTree::~HierarchyTree() {}

tree <SpLearningStateConcept> *HierarchyTree::getHierarchy() {
    return &conceptsHierarchy;
}

void HierarchyTree::recurseToMostLikelySiblingStates(double P, tree<SpLearningStateConcept>::sibling_iterator node,
                                                     std::map<long int, SpLearningStateConcept> &states) {
    if(conceptsHierarchy.number_of_children(node) == 0) {
        //std::cout << "Terminal " << (*node)->id << ": P = " << (*node)->getP() << std::endl;
        states[(*node)->id] = *node;
        return;
    }

    tree<SpLearningStateConcept>::sibling_iterator child, child_end = conceptsHierarchy.end(node);
    bool higherP = false;

    for(child = conceptsHierarchy.begin(node); child != child_end; child++) {
        if((*child)->getP() >= P) {
            higherP = true;
            break;
        }
    }

    // a) If no child is > P, draw it
    if(higherP == false) {
        states[(*node)->id] = *node;
        return;
    } else {
    // b) If at least one child > P,
        //std::cout << "Parent " << (*node)->id << ": P = " << (*node)->getP() << std::endl;
        for(child = conceptsHierarchy.begin(node); child != child_end; child++) {
            //std::cout << "Child " << (*child)->id << ": P = " << (*child)->getP() << std::endl;
            if((*child)->getP() >= P) // Recurse to those >= P
                recurseToMostLikelySiblingStates(P, child, states);
            else                      // Draw those < P
                states[(*child)->id] = *child;
        }
    }
}

bool HierarchyTree::recurseToDeepestSiblingStates(double P, tree<SpLearningStateConcept>::sibling_iterator node,
                                                  std::map<long int, SpLearningStateConcept> &states,
                                                  attribute_id _x, attribute_id _y) {
    int n;
    if( (n = conceptsHierarchy.number_of_children(node)) == 0) {
        //std::cout << "Terminal " << (*node)->id << ": P = " << (*node)->getP() << std::endl;
        SpLearningStateConcept state = *node;
        if(state->getPositionP(_x,_y) > P) {
            states[state->id] = state;
            return true;
        }
        return false;
    }

    tree<SpLearningStateConcept>::sibling_iterator child, child_end = conceptsHierarchy.end(node);
    bool higherP = false, Ps[n];
    int i;
    memset(Ps, false, n*sizeof(bool));

    for(i = 0, child = conceptsHierarchy.begin(node); child != child_end; i++, child++) {
        if(recurseToDeepestSiblingStates(P, child, states,_x,_y)) {
            Ps[i] = true;
            higherP = true;
        }
    }

    // a) If no child is > P, draw it if is likely
    if(higherP == false) {
        SpLearningStateConcept state = *node;
        if(state->getPositionP(_x,_y) > P) {
            states[state->id] = state;
            return true;
        }
        return false;
    }
    // b) If at least one child > P, add those who were not added
    //std::cout << "Parent " << (*node)->id << ": P = " << (*node)->getP() << std::endl;
    for(i=0, child = conceptsHierarchy.begin(node); child != child_end; i++, child++) {
        //std::cout << "Child " << (*child)->id << ": P = " << (*child)->getP() << std::endl;
        if(Ps[i] == false) { // Recurse to those >= P
            states[(*child)->id] = *child;
        }
    }
    return true;
}

//Function considers relevant a group of siblings, if at least one of them depass a threshold
//1. If current is < P draw it
//2. Else
//   2.a) If no child is > P, draw it
//   2.b) If at least one child > P,
//      - Draw those < P
//      - Recurse to those >= P
bool HierarchyTree::getMostLikelySiblingStates(double P, std::map<long int, SpLearningStateConcept> &states) {
    tree<SpLearningStateConcept>::iterator Node = conceptsHierarchy.begin();

    if(isEmptyTree(Node)) {
        return false;
    }

    if(conceptsHierarchy.number_of_children(Node) == 0) {
        states[(*Node)->id] = *Node;
        return true;
    }

    recurseToMostLikelySiblingStates(P, Node, states);

    return true;
}

bool HierarchyTree::getDeepestSiblingStates(double P, std::map<long int, SpLearningStateConcept> &states,
                                            attribute_id _x, attribute_id _y) {
    tree<SpLearningStateConcept>::iterator Node = conceptsHierarchy.begin();

    if(isEmptyTree(Node)) {
        return false;
    }

    if(conceptsHierarchy.number_of_children(Node) == 0) {
        states[(*Node)->id] = *Node;
        return true;
    }

    recurseToDeepestSiblingStates(P, Node, states, _x, _y);
    return true;
}




  int HierarchyTree::getDepth() {
    return deepest_level;
  }

  void HierarchyTree::setDepth(int level) {
    deepest_level = level;
  }

  void HierarchyTree::setDepth() {
   int level=0;
    int kids;
    tree<SpLearningStateConcept>::iterator Node=conceptsHierarchy.begin();
    tree<SpLearningStateConcept>::fixed_depth_iterator level_iter;

    if(isEmptyTree(Node))
      return;

    deepest_level=level;
    (*Node)->setParent(-1);
    (*Node)->level = 0;
    kids=conceptsHierarchy.number_of_children(Node);

    for(level=1;kids>0;level++) {
      kids=0;

      for(level_iter=conceptsHierarchy.begin_fixed(Node, level);
	  conceptsHierarchy.is_valid(level_iter);level_iter++) {
        kids+=conceptsHierarchy.number_of_children(level_iter);
        (*level_iter)->setParent((*(conceptsHierarchy.parent(level_iter)))->getID());
	(*level_iter)->level = level;
      }
      if(kids==0)
	deepest_level=level;
    }

    setDistancesToLeaf(Node);
  }

  int HierarchyTree::setDistancesToLeaf(tree<SpLearningStateConcept>::sibling_iterator Node) {

    if(isTerminal(Node)) {
      (*Node)->distance_to_leaf = 0;
      return 0;
    }

    int min_distance = INT_MAX, curr_min;
    tree<SpLearningStateConcept>::sibling_iterator sib;

    for(sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);sib++)
      if( (curr_min = setDistancesToLeaf(sib)) < min_distance )
	min_distance = curr_min;

    (*Node)->distance_to_leaf = min_distance + 1;
    
    return min_distance + 1;
  }

  void HierarchyTree::setDistancesToLeaf(tree<SpLearningStateConcept>::iterator Node) {
    int min_distance = INT_MAX, curr_min;
    tree<SpLearningStateConcept>::sibling_iterator sib;

    if(isTerminal(Node)) {
      (*Node)->distance_to_leaf = 0;
      return;
    }

    for(sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);sib++)
      if( (curr_min = setDistancesToLeaf(sib)) < min_distance )
	min_distance = curr_min;

    (*Node)->distance_to_leaf = 1 + min_distance;
  }

  int HierarchyTree::execute(SpLearningContext learningContext, SpLearningContextualisedObject contextualisedObject, double currentTimeDiff){
    
    lc_id = learningContext->id;
    currentContext = learningContext;
    currentObject = contextualisedObject;
    tdiff = currentTimeDiff;

    std::map<attribute_id, SpContextualisedNumericalData > *numericalData;
    std::map<attribute_id, SpContextualisedSymbolicData > *symbolicData;
    tree<SpLearningStateConcept>::iterator iter;
    iter=conceptsHierarchy.begin();
    if(contextualisedObject->numericalData.count(lc_id))
      numericalData = &contextualisedObject->numericalData[lc_id];
    else
      numericalData = NULL;

    if(contextualisedObject->symbolicData.count(lc_id))
      symbolicData = &contextualisedObject->symbolicData[lc_id];
    else
      symbolicData = NULL;

    currentInstance->setInstance(numericalData, symbolicData);

    currentStatesForInstance.clear();
    deepest_level = -1;
    deepest_for_instance = 0;
    //DEBUG CODE
    /*    if ( lc_id == 7 && registeredStates.count(10) > 0 ) {
      SpLearningStateConcept toAnalyse = registeredStates[10];
      int i, n = toAnalyse->incomingEvents.size(), m = toAnalyse->outgoingEvents.size(); 
      std::map <long int, SpLearningEventConcept>::iterator incomingIt = toAnalyse->incomingEvents.begin();
      std::map <long int, SpLearningEventConcept>::iterator outgoingIt = toAnalyse->outgoingEvents.begin();
      long int event_id, other_id;
      SpLearningEventConcept event;
      SpLearningStateConcept otherState;
      std::cout << "Context: " << currentContext->name << "for state 10.\n";
      std::cout << "Before MILES:\n";
      //Incoming Events
      std::cout << "\tIncoming Events:\n";
      for(i=0; i<n; i++, incomingIt++) {
	other_id = (*incomingIt).first;
	event = (*incomingIt).second;
	event_id = event->id;
	std::cout << "\t\tIndex" << i << ":\t\tother_id: " << other_id << "\tevent_id: " << event_id << "\n";
      }
      //Outgoing Events
      std::cout << "\tOutgoing Events:\n";
      for(i=0; i<m; i++, outgoingIt++) {
	other_id = (*outgoingIt).first;
	event = (*outgoingIt).second;
	event_id = event->id;
	std::cout << "\t\tIndex" << i << ":\t\tother_id: " << other_id << "\tevent_id: " << event_id << "\n";
      }
    } else if ( lc_id == 7 && registeredStates.count(10) == 0 ) {
      std::cout << "Context: " << currentContext->name << "\n";
      std::cout << "\tNode 10 not present yet.\n";
      }*/
    //END
    insertionDone = terminalDone = splitDone = mergeDone = false;
    MILES(iter, currentInstance, true);
    /*if(insertionDone)
      std::cout << "\tInsertion Done.\n";
    if(terminalDone) 
      std::cout << "\tTerminal Done.\n";
    if(splitDone) 
      std::cout << "\tSplit Done.\n";
    if(mergeDone)
      std::cout << "\tMerge Done.\n";*/
    //DEBUG CODE
    /*if ( lc_id == 7 && registeredStates.count(10) > 0 ) {
      SpLearningStateConcept toAnalyse = registeredStates[10];
      int i, n = toAnalyse->incomingEvents.size(), m = toAnalyse->outgoingEvents.size(); 
      std::map <long int, SpLearningEventConcept>::iterator incomingIt = toAnalyse->incomingEvents.begin();
      std::map <long int, SpLearningEventConcept>::iterator outgoingIt = toAnalyse->outgoingEvents.begin();
      long int event_id, other_id;
      SpLearningEventConcept event;
      SpLearningStateConcept otherState;
      std::cout << "After MILES:\n";
      //Incoming Events
      std::cout << "\tIncoming Events:\n";
      for(i=0; i<n; i++, incomingIt++) {
	other_id = (*incomingIt).first;
	event = (*incomingIt).second;
	event_id = event->id;
	std::cout << "\t\tIndex" << i << ":\t\tother_id: " << other_id << "\tevent_id: " << event_id << "\n";
      }
      std::cout << "\tOutgoing Events:\n";
      //Outgoing Events
      for(i=0; i<m; i++, outgoingIt++) {
	other_id = (*outgoingIt).first;
	event = (*outgoingIt).second;
	event_id = event->id;
	std::cout << "\t\tIndex" << i << ":\t\tother_id: " << other_id << "\tevent_id: " << event_id << "\n";
      }
    } else if ( lc_id == 7 && registeredStates.count(10) == 0 )
    std::cout << "\tNode 10 not present yet.\n";    */
    //END
/*    std::map<int, SpLearningStateConcept>::iterator lit, lite = currentStatesForInstance.end();
    std::cout << "For Tracked Object: " << contextualisedObject->trackedObjectId << std::endl;
    for(lit = currentStatesForInstance.begin(); lit !=lite; lit++) {
        std::cout << "\tLevel: " << (*lit).first <<" -> State: " << (*lit).second->id << std::endl;
    }*/
    setDepth();

    //Update contextualised objects and events
    if(currentObject->numFrames == 1) {
      std::map<int, SpLearningStateConcept>::iterator levelIt = currentStatesForInstance.begin();
      int i, n = currentStatesForInstance.size();
      for(i=0; i<n; i++,levelIt++) {
        SpLevelData newData(new LevelData());
	newData->currentState = (*levelIt).second->id;
	newData->timeStayingAtCurrent = tdiff / 2.0; //As it is starting, we do not know the exact starting moment.
	newData->analysedFrames = 1;
	currentObject->levelData[lc_id][(*levelIt).first] = newData;
      }

    } else {
      int diff_index;
      bool diff_found = false;
      double transition_time;

      std::map<int, SpLevelData>*levels = &currentObject->levelData[lc_id];
      int i, j, n = currentStatesForInstance.size() - 1, m = levels->size() - 1;
      //DEBUG
      /*      std::cout << "\tPrevious States:\n";
      for(i=0; i<=m; i++)
	std::cout << "\t\t" << (*levels)[i]->currentState << "\n";
      std::cout << "\tCurrent States:\n";
      for(i=0; i<=n; i++)
      std::cout << "\t\t" << currentStatesForInstance[i]->id << "\n";*/
      //END

      for(i=0; i<=n && i<=m; i++)
	if((*levels)[i]->currentState != currentStatesForInstance[i]->id) {
	  diff_index = i;
	  diff_found = true;
	  break;
	} else {//Update staying time
	  (*levels)[i]->timeStayingAtCurrent += tdiff;
	  (*levels)[i]->analysedFrames++;
	}
      //Update Events
      if(diff_found) {
        SpLearningStateConcept c1, c2;
        SpLearningEventConcept e;
	double s, diff;
	std::map<int, std::deque<long int> > newEvents;
	long int curr_state;
	for(i=diff_index; i<=m; i++) {
	  (*levels)[i]->lastEvents.clear();
	  curr_state = (*levels)[i]->currentState;
	  c1 = registeredStates[curr_state];
	  c1->NE++;
	  for(j=diff_index; j<=n; j++) {
	    c2 = currentStatesForInstance[j];
	    //	    if(c1->id == c2->id) {
	    //  std::cout<< "Alerta!!! Aca el error!!!\n\n";
	    //  exit(1);
	    // }
	    if(c1->outgoingEvents.count(c2->id) > 0) { //Update

	      transition_time = (*levels)[i]->timeStayingAtCurrent += tdiff / 2.0;
	      
	      e = c1->outgoingEvents[c2->id];
	      s = e->getSigma();
	      diff = transition_time - e->getMean();
	      e->setSigma(sqrt((s*s + diff*diff/(e->N + 1.0))*(e->N)/(e->N + 1.0)));
	      e->setMean((transition_time + e->getMean()*e->N)/(e->N + 1.0));
	      e->N++;
	      if(transition_time < e->minT)
		e->minT = transition_time;
	      if(transition_time > e->maxT)
		e->maxT = transition_time;
	      e->P = (double)e->N / (double)c1->NE;      
	      newEvents[j].push_back(e->id);
	    } else { //Create
	      (*levels)[i]->analysedFrames++;
	      transition_time = (*levels)[i]->timeStayingAtCurrent += tdiff / 2.0;
              SpLearningEventConcept newe(new LearningEventConcept(free_event_id, transition_time, 0.0));
	      free_event_id++;
	      c1->outgoingEvents[c2->id] = newe;
	      c2->incomingEvents[c1->id] = newe;
	      registeredEvents[newe->id] = newe;
	      newe->startingState = c1->id;
	      newe->arrivingState = c2->id;
	      newe->N = 1;
	      newe->minT = transition_time;
	      newe->maxT = transition_time;
	      newe->P = 1.0 / (double)c1->NE;   
	      newEvents[j].push_back(newe->id);
	    }
	  }

	  //Update Outgoing Events Probabilities for state c1
          std::map <long int, SpLearningEventConcept>::iterator evIt = c1->outgoingEvents.begin();
	  int k, numEvents = c1->outgoingEvents.size();
	  for(k=0; k<numEvents; k++, evIt++)
	    (*evIt).second->P = (double)(*evIt).second->N / (double)c1->NE;

	}
	


	for(j=diff_index; j<=n; j++) {	
	  if(j <= m) { //The level exists
	    (*levels)[j]->currentState = currentStatesForInstance[j]->id;
	    (*levels)[j]->analysedFrames = 1;
	    (*levels)[j]->timeStayingAtCurrent = tdiff/2.0;
	    (*levels)[j]->lastEvents = newEvents[j];
	  } else { //Create the level
            SpLevelData newData(new LevelData());
	    newData->currentState = currentStatesForInstance[j]->id;
	    newData->analysedFrames = 1;
	    newData->timeStayingAtCurrent = tdiff/2.0;
	    newData->lastEvents = newEvents[j];
	    (*levels)[j] = newData;
	  }
	}

	//Erase non used levels
	if(m > n) {
	  for(j=n+1; j<=m; j++)
	    currentObject->levelData[lc_id].erase(j);	    
	} 
      }
    }


    return 1;
  }

  void HierarchyTree::print() {
    int level;
    tree<SpLearningStateConcept>::iterator Node=conceptsHierarchy.begin();
    tree<SpLearningStateConcept>::fixed_depth_iterator level_iter;
    
    if(isEmptyTree(Node))
      return;
    
    std::cout << "LEVEL 0:\nNode 0:\n" ;
    (*Node)->print(availableAttributes);
    std::cout.flush();
    
    for(level=1;level<=deepest_level;level++) {
      std::cout << "LEVEL " << level <<":\n";
      std::cout.flush();
      for(level_iter=conceptsHierarchy.begin_fixed(Node, level);
	  level_iter!=0;level_iter++) {
	std::cout << "Node " << (*level_iter)->getID() << ":\n" ;
	std::cout << "\tParent ID: " << (*conceptsHierarchy.parent(level_iter))->getID() << "\n" ;
	(*level_iter)->print(availableAttributes);
	std::cout.flush();
      }
    }
  }

  void HierarchyTree::relabelTree() {
    int level, i=1;
    int kids;
    tree<SpLearningStateConcept>::iterator Node=conceptsHierarchy.begin();
    tree<SpLearningStateConcept>::fixed_depth_iterator level_iter;

    if(isEmptyTree(Node))
      return;

    deepest_level=0;
    
    (*Node)->setID(0);
    (*Node)->setParent(-1);
    kids=conceptsHierarchy.number_of_children(Node);

    for(level=1;kids>0;level++) {
      kids=0;
      for(level_iter=conceptsHierarchy.begin_fixed(Node, level);
	  level_iter!=0;level_iter++) {
	kids+=conceptsHierarchy.number_of_children(level_iter);
	(*level_iter)->setID(i++);
	(*level_iter)->setParent((*(conceptsHierarchy.parent(level_iter)))->getID());

	if(kids==0)
	  deepest_level=level;
      }
    }
  }

  bool HierarchyTree::cutoff(tree<SpLearningStateConcept>::iterator Node, SpInstance Instance) {
    
    int i, n;
    attribute_id id;
    std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt;
    
    //symbolicAttributeList;

    //Test for numerical attributes
    if( (n = (*Node)->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = (*Node)->numericalAttributeList.begin();
      double instanceValue, acuity;
      for(i = 0; i < n; i++, numericalIt++) {
	id = (*numericalIt).first;
	instanceValue = (*Instance->numericalData)[id]->value;
	acuity = currentContext->numericalAttributes[id]->acuity;
	if( fabs(instanceValue - (*numericalIt).second->getMean()) > acuity)
	  return false;
      }
    }

    //Test for symbolic attributes
    if( (n = (*Node)->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = (*Node)->symbolicAttributeList.begin();
      std::string instanceValue;
      SpSymbolicAttributeValuePair currentSym;
      for(i = 0; i < n; i++, symbolicIt++) {
	id = (*symbolicIt).first;
	currentSym = (*symbolicIt).second;
	instanceValue = (*Instance->symbolicData)[id]->value;
	if( currentSym->values[instanceValue]->RSum / currentSym->RSum < ALMOST_ONE)
	  return false;
      }
    }

    return true;
  }

  bool orderedByDistanceToLeafOperator::operator()(SpLearningStateConcept concept1, SpLearningStateConcept concept2) {
    return concept1->distance_to_leaf < concept2->distance_to_leaf;
  }

  bool orderedByStateProbabilityOperator::operator()(SpLearningStateConcept concept1, SpLearningStateConcept concept2) {
    return concept1->getP() < concept2->getP();
  }

  
  void HierarchyTree::MILES(tree<SpLearningStateConcept>::iterator Node, SpInstance Instance, bool doIncorporate) {

    SpLearningStateConcept newChild, mergingNode, currentNode;
    tree<SpLearningStateConcept>::iterator mergingIterator;
    int decision;
    tree<SpLearningStateConcept>::sibling_iterator sib;
    double splitScore=MINIMAL_DOUBLE, mergeScore=MINIMAL_DOUBLE;

/*    if(isEmptyTree(Node))
        std::cout << "Arbol Vacio" << std::endl;
    else {
        currentNode = *Node;
        std::cout << "Nodo en Estado: " << currentNode->id << " -> Parent: " << currentNode->parent << std::endl;
    }*/

    if(doIncorporate)
      deepest_level++;

    if(isEmptyTree(Node)) {

      insertRoot(Node, Instance);

      currentStatesForInstance[deepest_for_instance] = (*conceptsHierarchy.begin());

    } else if(isTerminal(Node)) {

      currentStatesForInstance[deepest_for_instance] = *Node;

      if(!cutoff(Node, Instance))
	createNewTerminal(Node, Instance);
      Incorporate((*Node), conceptsHierarchy.parent(Node), Instance);

    } else {

      tree<SpLearningStateConcept>::iterator bestNode=conceptsHierarchy.begin();
      tree<SpLearningStateConcept>::iterator secondNode=conceptsHierarchy.begin();
      diffNew = diffMerge = diffBest = diffSplit = 0.0;

      if(doIncorporate) {
	Incorporate((*Node), conceptsHierarchy.parent(Node), Instance);
	currentStatesForInstance[deepest_for_instance] = *Node;
	deepest_for_instance++;
      }
      computeKParent();
      
      k_components=computeKComponents(Node);

      computeChildrenScores(Node, Instance);

      newChild=computeNewChild(Node, Instance);

      getBestChildren(&bestNode,&secondNode,Node);
      
      if( conceptsHierarchy.number_of_children(Node) > 2 ) {
	mergingNode = computeMergingNodes(bestNode, secondNode, Node, Instance);
	mergeScore = mergingNode->getScore();
      }
      if( conceptsHierarchy.number_of_children(bestNode) > 0)
	splitScore = computeSplittingNode(bestNode, Node, Instance);

      free(k_components);

      updateChildrenProbabilities(Node);

      decision=determineBest((*bestNode)->getScore(), newChild->getScore(), mergeScore, splitScore);
      /*
      std::cout << "For level " << deepest_level << std::endl;
      std::cout << "\tBest Node Score:  " << (*bestNode)->getScore() << std::endl;
      std::cout << "\tNew Child Score:  " << newChild->getScore() << std::endl;
      std::cout << "\tMerge Score:  " << mergeScore << std::endl;
      std::cout << "\tSplit Score:  " << splitScore << std::endl;
      */
      switch(decision) {
      case NEW_NODE:
	insertNewChild(Node, Instance);
	break;
      case MERGING_NODE:
	mergingIterator=doMerge(Node, bestNode, secondNode, mergingNode);
	MILES(mergingIterator, Instance, true);
	break;
      case BEST_NODE:
	MILES(bestNode, Instance, true);
	break;
      case SPLIT_NODE:
	doSplit(Node, bestNode);
	MILES(Node, Instance, false);
      default:
	//std::cout << "ERROR!!!!\n";
	//std::cout.flush();
	return;
      }
    }
  }

  bool HierarchyTree::kinshipRelation(SpLearningStateConcept state1, SpLearningStateConcept state2) {
    long int id1, id2 = state2->id;
    long int parent;
    do {
      id1 = state1->id;
      parent = state1->parent;
      if(id1 == id2)
	return true;
      if(parent != -1)
	state1 = registeredStates[parent];
    } while(parent != -1);

    return false;

  }


  tree<SpLearningStateConcept>::iterator HierarchyTree::doMerge(tree<SpLearningStateConcept>::iterator Node,
                                                              tree<SpLearningStateConcept>::iterator bestNode,
                                                              tree<SpLearningStateConcept>::iterator secondNode,
                                                              SpLearningStateConcept MergingNode) {
    MergingNode->id = free_state_id++;
    MergingNode->parent = (*Node)->id;
    tree<SpLearningStateConcept>::iterator mergingIterator = conceptsHierarchy.append_child(Node, MergingNode);
    SpLearningStateConcept best = (*bestNode), second = (*secondNode);

    mergeDone = true;

    conceptsHierarchy.reparent(mergingIterator, bestNode, conceptsHierarchy.next_sibling(bestNode));
    conceptsHierarchy.reparent(mergingIterator, secondNode, conceptsHierarchy.next_sibling(secondNode));
    best->parent = second->parent = MergingNode->id;
    //conceptsHierarchy.erase(bestNode);
    //conceptsHierarchy.erase(secondNode);
    updateChildrenProbabilities(Node);
    registeredStates[MergingNode->id] = MergingNode;

    //Update events for the Merging node
    int i, n = best->incomingEvents.size(), m = best->outgoingEvents.size(); 
    long int event_id, event_id1, event_id2, current_id1 = best->id, current_id2 = second->id, other_id;
    std::map <long int, SpLearningEventConcept>::iterator incomingIt = best->incomingEvents.begin();
    std::map <long int, SpLearningEventConcept>::iterator outgoingIt = best->outgoingEvents.begin();
    SpLearningEventConcept event1, event2;
    SpLearningStateConcept otherState;
    double mean, sigma;
    int N;
    double minT, maxT;

    //Set the overall number of generated events for the state as the sum between the two states, minus the interactions
    //between them
    MergingNode->NE = 
      ( best->outgoingEvents.count(current_id2) == 0 ? best->NE : best->NE - best->outgoingEvents[current_id2]->N )
      + ( second->outgoingEvents.count(current_id1) == 0 ? second->NE : second->NE - second->outgoingEvents[current_id1]->N );

    //Check incoming Events
    MergingNode->incomingEvents.clear();

    for(i=0; i<n; i++, incomingIt++) {
      other_id = (*incomingIt).first;
      otherState = registeredStates[other_id];
      if(kinshipRelation(otherState, second)) //Internal interactions are not interesting for a higher level
	continue;
      
      event1 = (*incomingIt).second;
      event_id1 = event1->id;

      if(second->incomingEvents.count(other_id)>0) { //Treat double incoming case
	event2 = second->incomingEvents[other_id];
	event_id2 = event2->id;
	double sumP = event1->P + event2->P;
	double s1 = event1->getSigma(), s2 = event2->getSigma();
	mean = (event1->P*event1->getMean() + event2->P*event2->getMean()) / sumP ;
	sigma = sqrt((event1->P*s1*s1 + event2->P*s2*s2) / sumP);
	N = event1->N + event2->N;
	minT = event1->minT < event2->minT ? event1->minT : event2->minT;
	maxT = event1->maxT < event2->maxT ? event1->maxT : event2->maxT;
      } else {//Treat incoming with best case
	mean = event1->getMean();
	sigma = event1->getSigma();
	N = event1->N;
	minT = event1->minT;
	maxT = event1->maxT;
      } 
      event_id = free_event_id;
      free_event_id++;

      SpLearningEventConcept newEvent(new LearningEventConcept(event_id, mean, sigma));
      newEvent->N = N;
      newEvent->minT = minT; 
      newEvent->maxT = maxT;
      newEvent->P = (double)N/(double)otherState->NE;
      newEvent->startingState = other_id;
      newEvent->arrivingState = MergingNode->id;

      //Update copy state
      MergingNode->incomingEvents[other_id] = newEvent;

      //Update starting state
      otherState->outgoingEvents[MergingNode->id] = newEvent;

      //Update HierarchyTree
      registeredEvents[event_id] = newEvent;
    }

    //Set remaining incoming events, considering the events only arriving to second node
    incomingIt = second->incomingEvents.begin();
    n = second->incomingEvents.size();

    for(i=0; i<n; i++, incomingIt++) {
      other_id = (*incomingIt).first;
      otherState = registeredStates[other_id];
      if(kinshipRelation(otherState, best)) //Internal interactions are not interesting for a higher level
	continue;

      if(best->incomingEvents.count(other_id)==0) { //Treat remaining second only case

	event2 = (*incomingIt).second;
	event_id2 = event2->id;
	mean = event2->getMean();
	sigma = event2->getSigma();
	N = event2->N;
	minT = event2->minT;
	maxT = event2->maxT;
       
	event_id = free_event_id;
	free_event_id++;
        SpLearningEventConcept newEvent(new LearningEventConcept(event_id, mean, sigma));
	newEvent->N = N;
	newEvent->minT = minT; 
	newEvent->maxT = maxT;
	newEvent->P = (double)N/(double)MergingNode->NE;
	newEvent->startingState = other_id;
	newEvent->arrivingState = MergingNode->id;

	//Update copy state
	MergingNode->incomingEvents[other_id] = newEvent;
	
	//Update starting state
	otherState->outgoingEvents[MergingNode->id] = newEvent;

	//Update HierarchyTree
	registeredEvents[event_id] = newEvent;
      }
    }


    //Check outgoing events
    double DeltaA, LambdaA, GammaA, PA, DeltaB, LambdaB, GammaB, PB, Pcond, PAB, PBA, auxMax;
    double meanAB, meanBest, meanBA, meanSecond, sigmaAB, sigmaBest, sigmaBA, sigmaSecond, meanBOther, sigmaBOther;
    int j;
    SpLearningEventConcept eventAB, eventBA;

    MergingNode->outgoingEvents.clear();

    for(j=0; j<m; j++, outgoingIt++) {
      other_id = (*outgoingIt).first;
      otherState = registeredStates[other_id];
      if(kinshipRelation(otherState, second)) //Internal interactions are not interesting for a higher level
	continue;
 
      event1 = (*outgoingIt).second;
      event_id1 = event1->id;
      meanBOther = event1->getMean();
      sigmaBOther = event1->getSigma();

      N = event1->N;

      LambdaA = DeltaA = GammaA = 0.0;
      LambdaB = DeltaB = GammaB = 0.0;
      PB = best->P;
      PA = second->P;
      
      if(best->incomingEvents.count(current_id2) > 0) { 
	eventAB = best->incomingEvents[current_id2];
	PAB = eventAB->P;
	meanAB = eventAB->getMean();
	sigmaAB = eventAB->getSigma();

	if(best->outgoingEvents.count(current_id2) > 0) { //The hardest case, a loop between best and second
	  eventBA = best->outgoingEvents[current_id2];
	  PBA = eventBA->P;
	  meanBA = eventBA->getMean();
	  sigmaBA = eventBA->getSigma();

	  //LambdaA, DeltaA, and GammaA first
	  Pcond = PAB; 
	  i = 0;

          while(Pcond > HierarchyTree::minConditionalP) {
	    LambdaA += Pcond*(i*meanBA + (i+1)*meanAB + meanBOther);
	    DeltaA += Pcond; 
	    GammaA += Pcond*(i*i*sigmaBA*sigmaBA + (i+1)*(i+1)*sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/( 2.0*( i*(i + 1.0) + 1.0) );
	    i++;
	    Pcond *= PAB*PBA; 
	  } 
	  LambdaA *= PA;
	  DeltaA *= PA; 
	  GammaA *= PA;
	  i--;
	  Pcond /= PAB*PBA;
	  maxT = Pcond*(i*eventBA->maxT + (i+1)*eventAB->maxT + event1->maxT);
	  auxMax = Pcond;
	  //Now LambdaB, DeltaB, and GammaB
	  Pcond = 1.0; 
	  i = 0;

          while(Pcond > HierarchyTree::minConditionalP) {
	    LambdaB += Pcond*(i*(meanBA + meanAB) + meanBOther);
	    DeltaB += Pcond; 
	    GammaB += Pcond*(i*i*(sigmaBA*sigmaBA + sigmaAB*sigmaAB) + sigmaBOther*sigmaBOther)/( 2.0*i*i + 1.0 );
	    i++;
	    Pcond *= PAB*PBA; 
	  } 
	  LambdaB *= PB;
	  DeltaB *= PB; 
	  GammaB *= PB;
	  i--;
	  Pcond /= PAB*PBA;
	  maxT += Pcond*(i*(eventBA->maxT + eventAB->maxT) + event1->maxT);
	  auxMax += Pcond;

	  //Update information
	  minT = event1->minT; //The shortest path
	  maxT /= auxMax; //The normalised longuest paths
	  meanBest = (LambdaA + LambdaB)/(DeltaA + DeltaB);
	  sigmaBest = sqrt((GammaA + GammaB)/(DeltaA + DeltaB));
	} else {//This case considers the direct transition from best to other, and the transition second->best->other
	  minT = event1->minT; //The shortest path
	  maxT = event1->maxT + eventAB->maxT; //The normalised longuest paths
	  meanBest = (PB*meanBOther + PA*PAB*(meanAB + meanBOther)) / (PB + PA*PAB);
	  sigmaBest = sqrt((PB*sigmaBOther*sigmaBOther + PA*PAB*(sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/2.0) / (PB + PA*PAB));
	}

      } else { //This case is simple, as only the direct transition from best to other is interesting
	//Update information
	minT = event1->minT;
	maxT = event1->maxT;
	meanBest = meanBOther;
	sigmaBest = sigmaBOther;
      }

      //Check for an event going to the same event
      if(second->outgoingEvents.count(other_id)>0) { //Treat double outgoing case
	double auxMax2;
	event2 = second->outgoingEvents[other_id];
	event_id2 = event2->id;
	meanBOther = event2->getMean();
	sigmaBOther = event2->getSigma();
	N += event2->N;
	LambdaA = DeltaA = GammaA = 0.0;
	LambdaB = DeltaB = GammaB = 0.0;
	PB = second->P;
	PA = best->P;

	if(second->incomingEvents.count(current_id1) > 0) { 
	  eventAB = second->incomingEvents[current_id1];
	  PAB = eventAB->P;
	  meanAB = eventAB->getMean();
	  sigmaAB = eventAB->getSigma();

	  if(second->outgoingEvents.count(current_id1) > 0) { //The hardest case, a loop between best and second
	    eventBA = second->outgoingEvents[current_id1];
	    PBA = eventBA->P;
	    meanBA = eventBA->getMean();
	    sigmaBA = eventBA->getSigma();
	    
	    //LambdaA, DeltaA, and GammaA first
	    Pcond = PAB; 
	    i = 0;

            while(Pcond > HierarchyTree::minConditionalP) {
	      LambdaA += Pcond*(i*meanBA + (i+1)*meanAB + meanBOther);
	      DeltaA += Pcond; 
	      GammaA += Pcond*(i*i*sigmaBA*sigmaBA + (i+1)*(i+1)*sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/( 2.0*( i*(i + 1.0) + 1.0) );
	      i++;
	      Pcond *= PAB*PBA; 
	    } 
	    LambdaA *= PA;
	    DeltaA *= PA; 
	    GammaA *= PA;
	    i--;
	    Pcond /= PAB*PBA;
	    auxMax2 = Pcond*(i*eventBA->maxT + (i+1)*eventAB->maxT + event2->maxT);
	    auxMax = Pcond;
	    //Now LambdaB, DeltaB, and GammaB
	    Pcond = 1.0; 
	    i = 0;

            while(Pcond > HierarchyTree::minConditionalP) {
	      LambdaB += Pcond*(i*(meanBA + meanAB) + meanBOther);
	      DeltaB += Pcond; 
	      GammaB += Pcond*(i*i*(sigmaBA*sigmaBA + sigmaAB*sigmaAB) + sigmaBOther*sigmaBOther)/( 2.0*i*i + 1.0 );
	      i++;
	      Pcond *= PAB*PBA; 
	    } 
	    LambdaB *= PB;
	    DeltaB *= PB; 
	    GammaB *= PB;
	    i--;
	    Pcond /= PAB*PBA;
	    auxMax2 += Pcond*(i*(eventBA->maxT + eventAB->maxT) + event1->maxT);
	    auxMax += Pcond;

	    //Update information
	    if(minT > event2->minT)
	      minT = event2->minT; //The shortest path
	    auxMax2 /= auxMax; //The normalised longuest paths
	    if(auxMax2 > maxT)
	      maxT = auxMax2;
	    meanSecond = (LambdaA + LambdaB)/(DeltaA + DeltaB);
	    sigmaSecond = sqrt((GammaA + GammaB)/(DeltaA + DeltaB));
	  } else {//This case considers the direct transition from best to other, and the transition second->best->other
	    if(minT > event2->minT)
	      minT = event2->minT; //The shortest path
	    auxMax2 = event2->maxT + eventAB->maxT; //The normalised longuest paths
	    if(auxMax2 > maxT)
	      maxT = auxMax2;

	    meanSecond = (PB*meanBOther + PA*PAB*(meanAB + meanBOther)) / (PB + PA*PAB);
	    sigmaSecond = sqrt((PB*sigmaBOther*sigmaBOther + PA*PAB*(sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/2.0) / (PB + PA*PAB));
	  }
	} else { //This case is simple, as only the direct transition from best to other is interesting
	  //Update information
	  if(minT > event2->minT)
	    minT = event2->minT;
	  if(maxT < event2->maxT)
	    maxT = event2->maxT;
	  meanSecond = meanBOther;
	  sigmaSecond = sigmaBOther;
	}

	mean = (event1->P*meanBest + event2->P*meanSecond) / (event1->P + event2->P);
	sigma = sqrt((event1->P*sigmaBest*sigmaBest + event2->P*sigmaSecond*sigmaSecond) / (event1->P + event2->P));
     
      } else {//Treat incoming with best case
	mean = meanBest;
	sigma = sigmaBest;
      } 
      event_id = free_event_id;
      free_event_id++;
      SpLearningEventConcept newEvent(new LearningEventConcept(event_id, mean, sigma));
      newEvent->N = N;
      newEvent->minT = minT; 
      newEvent->maxT = maxT;
      newEvent->P = (double)N/(double)MergingNode->NE;
      newEvent->startingState = MergingNode->id;
      newEvent->arrivingState = other_id;

      //Update copy state
      MergingNode->outgoingEvents[other_id] = newEvent;

      //Update starting state
      otherState->incomingEvents[MergingNode->id] = newEvent;

      //Update HierarchyTree
      registeredEvents[event_id] = newEvent;
    }

    //Set remaining outgoing events, considering the events only starting from second node
    outgoingIt = second->outgoingEvents.begin();
    m = second->outgoingEvents.size();

    for(j=0; j<m; j++, outgoingIt++) {
      other_id = (*outgoingIt).first;
      otherState = registeredStates[other_id];
      if(kinshipRelation(otherState, best)) //Internal interactions are not interesting for a higher level
	continue;
      
      if(best->outgoingEvents.count(other_id)==0) { //Treat remaining second only case

	otherState = registeredStates[other_id];
 
	event2 = (*outgoingIt).second;
	event_id2 = event2->id;
	meanBOther = event2->getMean();
	sigmaBOther = event2->getSigma();

	N = event2->N;

	LambdaA = DeltaA = GammaA = 0.0;
	LambdaB = DeltaB = GammaB = 0.0;
	PA = best->P;
	PB = second->P;

	if(second->incomingEvents.count(current_id1) > 0) { 
	  eventAB = second->incomingEvents[current_id1];
	  PAB = eventAB->P;
	  meanAB = eventAB->getMean();
	  sigmaAB = eventAB->getSigma();

	  if(second->outgoingEvents.count(current_id1) > 0) { //The hardest case, a loop between best and second
	    eventBA = second->outgoingEvents[current_id1];
	    PBA = eventBA->P;
	    meanBA = eventBA->getMean();
	    sigmaBA = eventBA->getSigma();

	    //LambdaA, DeltaA, and GammaA first
	    Pcond = PAB; 
	    i = 0;

            while(Pcond > HierarchyTree::minConditionalP) {
	      LambdaA += Pcond*(i*meanBA + (i+1)*meanAB + meanBOther);
	      DeltaA += Pcond; 
	      GammaA += Pcond*(i*i*sigmaBA*sigmaBA + (i+1)*(i+1)*sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/( 2.0*( i*(i + 1.0) + 1.0) );
	      i++;
	      Pcond *= PAB*PBA; 
	    } 
	    LambdaA *= PA;
	    DeltaA *= PA; 
	    GammaA *= PA;
	    i--;
	    Pcond /= PAB*PBA;
	    maxT = Pcond*(i*eventBA->maxT + (i+1)*eventAB->maxT + event2->maxT);
	    auxMax = Pcond;
	    //Now LambdaB, DeltaB, and GammaB
	    Pcond = 1.0; 
	    i = 0;

            while(Pcond > HierarchyTree::minConditionalP) {
	      LambdaB += Pcond*(i*(meanBA + meanAB) + meanBOther);
	      DeltaB += Pcond; 
	      GammaB += Pcond*(i*i*(sigmaBA*sigmaBA + sigmaAB*sigmaAB) + sigmaBOther*sigmaBOther)/( 2.0*i*i + 1.0 );
	      i++;
	      Pcond *= PAB*PBA; 
	    } 
	    LambdaB *= PB;
	    DeltaB *= PB; 
	    GammaB *= PB;
	    i--;
	    Pcond /= PAB*PBA;
	    maxT += Pcond*(i*(eventBA->maxT + eventAB->maxT) + event2->maxT);
	    auxMax += Pcond;

	    //Update information
	    minT = event2->minT; //The shortest path
	    maxT /= auxMax; //The normalised longuest paths
	    meanSecond = (LambdaA + LambdaB)/(DeltaA + DeltaB);
	    sigmaSecond = sqrt((GammaA + GammaB)/(DeltaA + DeltaB));
	  } else {//This case considers the direct transition from best to other, and the transition second->best->other
	    minT = event2->minT; //The shortest path
	    maxT = event2->maxT + eventAB->maxT; //The normalised longuest paths
	    meanSecond = (PB*meanBOther + PA*PAB*(meanAB + meanBOther)) / (PB + PA*PAB);
	    sigmaSecond = sqrt((PB*sigmaBOther*sigmaBOther + PA*PAB*(sigmaAB*sigmaAB + sigmaBOther*sigmaBOther)/2.0) / (PB + PA*PAB));
	  }

	} else { //This case is simple, as only the direct transition from best to other is interesting
	  //Update information
	  minT = event2->minT;
	  maxT = event2->maxT;
	  meanSecond = meanBOther;
	  sigmaSecond = sigmaBOther;
	}

	mean = meanSecond;
	sigma = sigmaSecond;
      
	event_id = free_event_id;
	free_event_id++;
        SpLearningEventConcept newEvent(new LearningEventConcept(event_id, mean, sigma));
	newEvent->N = N;
	newEvent->minT = minT; 
	newEvent->maxT = maxT;
	newEvent->P = (double)N/(double)MergingNode->NE;
	newEvent->startingState = MergingNode->id;
	newEvent->arrivingState = other_id;

	//Update copy state
	MergingNode->outgoingEvents[other_id] = newEvent;

	//Update starting state
	otherState->incomingEvents[MergingNode->id] = newEvent;

	//Update HierarchyTree
	registeredEvents[event_id] = newEvent;
      }
    }

    //Set the contextulised object list with this new node if the best or second are in the list
    //Check if the contextualised object was already present in best and second sibling nodes. 
    //If this is the case, add the new merge node as the previous node at this level
    std::map<unsigned long, SpLearningContextualisedObject>::iterator objIt = currentContext->contextualisedObjects.begin();
    SpLearningContextualisedObject analysedObject;
    std::map<int, SpLevelData> *leveldata;
    SpLevelData oldData;
    long int currentState;
    int datasize;
    n = currentContext->contextualisedObjects.size();

    for(i=0; i<n; i++, objIt++) {
      analysedObject = (*objIt).second;
      leveldata = &analysedObject->levelData[lc_id];
      datasize = leveldata->size();
      if(datasize > deepest_level+1) {
	currentState = (*leveldata)[deepest_level+1]->currentState;
	if(currentState == best->id || currentState == second->id) {
          SpLevelData currentData(new LevelData());
	  oldData = (*leveldata)[deepest_level+1];
	  currentData->currentState = MergingNode->id;
	  currentData->timeStayingAtCurrent = oldData->timeStayingAtCurrent;
	  currentData->analysedFrames = oldData->analysedFrames;
	  currentData->lastEvents = oldData->lastEvents;
	  for(j=datasize-1; j>deepest_level; j--)
	    analysedObject->levelData[lc_id][j+1] = analysedObject->levelData[lc_id][j];
	  analysedObject->levelData[lc_id][deepest_level+1] = currentData;
	}
      }
    }


    return mergingIterator;

  }

  void HierarchyTree::doSplit(tree<SpLearningStateConcept>::iterator Node,
                               tree<SpLearningStateConcept>::iterator bestNode) {

    int i, n = (*bestNode)->incomingEvents.size(), m = (*bestNode)->outgoingEvents.size(); 
    int kids = conceptsHierarchy.number_of_children(bestNode);
    splitDone = true;

    tree<SpLearningStateConcept>::sibling_iterator sib=conceptsHierarchy.begin(bestNode);

    for(i=0; i<kids; i++, sib++)
      (*sib)->parent = (*Node)->id;
    //Added concrete end iterator, even if it is 0
    conceptsHierarchy.reparent(Node, conceptsHierarchy.begin(bestNode), conceptsHierarchy.end(bestNode));
    
    long int event_id, current_id = (*bestNode)->id, other_id;

    SpLearningEventConcept event;
    SpLearningStateConcept otherState;

    if(n > 0) {
        std::map <long int, SpLearningEventConcept>::iterator incomingIt = (*bestNode)->incomingEvents.begin();
        std::deque <long int> toErase;
        //Incoming Events
        for(i=0; i<n; i++, incomingIt++) {
            other_id = (*incomingIt).first;
            event = (*incomingIt).second;
            event_id = event->id;
            otherState = registeredStates[other_id];
//            std::cout << "ERASE 1" << std::endl;
            otherState->outgoingEvents.erase(current_id);
            toErase.push_back(other_id);
//            if(registeredEvents.count(event_id) > 0)
//                std::cout << "Hay evento " << event_id << std::endl;
            registeredEvents.erase(event_id);
        }
        if(toErase.size() > 0) {
            std::deque <long int>::iterator it, it_end = toErase.end();
            for(it=toErase.begin(); it!=it_end; it++)
                (*bestNode)->incomingEvents.erase(*it);
        }
    }


    //Outgoing Events
    if(m > 0) {
        std::map <long int, SpLearningEventConcept>::iterator outgoingIt = (*bestNode)->outgoingEvents.begin();
        std::deque <long int> toErase;

        for(i=0; i<m; i++, outgoingIt++) {
            other_id = (*outgoingIt).first;
            event = (*outgoingIt).second;
            event_id = event->id;
            otherState = registeredStates[other_id];
//            std::cout << "ERASE 2" << std::endl;
            otherState->incomingEvents.erase(current_id);
            toErase.push_back(other_id);
            registeredEvents.erase(event_id);
        }
        if(toErase.size() > 0) {
            std::deque <long int>::iterator it, it_end = toErase.end();
            for(it=toErase.begin(); it!=it_end; it++)
                (*bestNode)->outgoingEvents.erase(*it);
        }

    }

    //Eliminate relations of objects with the state
    std::map<unsigned long, SpLearningContextualisedObject>::iterator objIt = currentContext->contextualisedObjects.begin();
    int j, maxlevel;
    n = currentContext->contextualisedObjects.size();
    std::map<int, SpLevelData> *pData;
    SpLevelData data;

    for(i=0; i<n; i++, objIt++) {
      pData = &(*objIt).second->levelData[lc_id]; 
      maxlevel = pData->size() - 1;
      if(maxlevel >= deepest_level+1) {
	data = (*pData)[deepest_level+1];
	if(data->currentState == current_id) {
	  for(j=deepest_level+1; j<maxlevel; j++)
	    (*pData)[j] = (*pData)[j+1];
	  (*pData).erase(maxlevel);
	}
      }
    }

    //Eliminate the state
//    std::cout << "ERASE 3" << std::endl;

//    if(registeredStates.count(current_id) > 0)
//        std::cout << "ENCONTRADO" << std::endl;

    registeredStates.erase(current_id);

    conceptsHierarchy.erase(bestNode);

  }

    
  int HierarchyTree::determineBest(double bestScore, double newScore, double mergingScore, double splitScore) {
      
    double scores[CAT_NUMBER] = {newScore, mergingScore, bestScore, splitScore}; 
    double diff[CAT_NUMBER]   = {diffNew,  diffMerge,    diffBest,  diffSplit};
    double best = scores[0], best_diff = -1;;
    //diff[0] ignored because it will always give maximal attribute probability
    int i, category = 0;
    //    printf("score:%.4f\tcat:%d\n",scores[0], 0);      
    for(i=1;i<CAT_NUMBER;i++) {
      //    printf("score:%.4f\tcat:%d\n",scores[i], i);
      if(scores[i] - best > EPSILON || (fabs(scores[i] - bestScore) <= EPSILON && diff[i] > best_diff)) {
	best=scores[i];
	best_diff = diff[i];
	category=i;
      }
    }
    //    printf("best:%.4f\tdiff:%.4f\tcat:%d\n",best, best_diff, category);
    return category;
  }

  void HierarchyTree::getBestChildren(tree<SpLearningStateConcept>::iterator *best,
                                      tree<SpLearningStateConcept>::iterator *second,
                                      tree<SpLearningStateConcept>::iterator Node ) {
      
    tree<SpLearningStateConcept>::sibling_iterator sib;
    double bestScore=MINIMAL_DOUBLE;
    double secondScore=MINIMAL_DOUBLE;
    double bestDiff=MAXIMAL_DOUBLE;
    double secondDiff=MAXIMAL_DOUBLE;

    for(sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);sib++) {
      if(    ( (*sib)->getScore() - bestScore > EPSILON ) 
	  || ( ( fabs((*sib)->getScore() - bestScore) <= EPSILON ) && ((*sib)->getDiff() > bestDiff) ) ) {
	secondScore=bestScore;
	second->node=best->node;
	bestScore=(*sib)->getScore();
	best->node=sib.node;
	secondDiff=bestDiff;
	bestDiff=(*sib)->getDiff();
      } else if(    ( (*sib)->getScore() - secondScore > EPSILON ) 
		 || ( ( fabs((*sib)->getScore() - secondScore) <= EPSILON ) && ((*sib)->getDiff() > secondDiff) ) ) {
	secondScore=(*sib)->getScore();
	secondDiff=(*sib)->getDiff();
	second->node=sib.node;
      }
      //      printf("S:%.4f\tD:%.4f\n",(*sib)->getScore(),(*sib)->getDiff());
    }
    diffBest = (**best)->getDiff();
  }

  void HierarchyTree::computeKParent() {

    int i, I;   
    tree<SpLearningStateConcept>::iterator root;
    double kp_num = 0.0, kp_sym = 0.0;

    root=conceptsHierarchy.begin();

    if( (I = (*root)->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = (*root)->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++)
	kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
      kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
    }

    if( (I = (*root)->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = (*root)->symbolicAttributeList.begin();
      int j, m;
      double kp_att, p;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;
	values = currentSym->values.begin();
	m = currentSym->values.size();
	kp_att = 0.0;
	for(j = 0; j < m; j++, values++) {
	  if(currentSym->RSum == 0)
	    p = 0.0;
	  else
	    p = (*values).second->RSum / currentSym->RSum;
	  kp_att += p*p;
	}
	kp_sym += kp_att;     /*k-component for the parent node.*/
      }

      kp_sym = kp_sym / (double)I;
    }

    k_parent = kp_num + kp_sym;
  }
    
  double *HierarchyTree::computeKComponents(tree<SpLearningStateConcept>::iterator Node) {

    tree<SpLearningStateConcept>::sibling_iterator sib;
    int K=conceptsHierarchy.number_of_children(Node);
    int N = (*Node)->getN();
    double *k_components=(double *)malloc(K*sizeof(double));
    double kp_num, kp_sym, P;
    int i, I, k;
      
    /*Set k-components.*/
    for(i=0,sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);i++,sib++) {
      P = (double)(*sib)->getN() / (double) N;
      kp_num = kp_sym = 0.0;

      if( (I = (*sib)->numericalAttributeList.size()) > 0) {
	std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = (*sib)->numericalAttributeList.begin();
	for(k = 0; k < I; k++, numericalIt++)
	  kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
	kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
      }

      if( (I = (*sib)->symbolicAttributeList.size()) > 0) {
	std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = (*sib)->symbolicAttributeList.begin();
	int j, m;
	double kp_att, p;
	std::map<std::string, SpSymbolicValue>::iterator values;
	SpSymbolicAttributeValuePair currentSym;	
	
	for(k = 0; k < I; k++, symbolicIt++) {
	  currentSym = (*symbolicIt).second;
	  values = currentSym->values.begin();
	  m = currentSym->values.size();
	  kp_att = 0.0;
	  for(j = 0; j < m; j++, values++) {
	    if(currentSym->RSum == 0)
	      p = 0.0;
	    else
	      p = (*values).second->RSum / currentSym->RSum;
	    kp_att += p*p;
	  }
	  kp_sym += kp_att;     /*k-component for the parent node.*/
	}
	
	kp_sym = kp_sym / (double)I;
      }

      k_components[i]=P * (kp_num + kp_sym);
      //      printf("k_component[%d]=%.4f\n", i, k_components[i]);
    }
    
    return k_components;
  }
  
  void HierarchyTree::updateChildrenProbabilities(tree<SpLearningStateConcept>::iterator Node) {
      
    int N=(*Node)->getN();
    tree<SpLearningStateConcept>::sibling_iterator sib;
    for(sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);sib++)
      (*sib)->setP((double)(*sib)->getN()/(double)N);

  }
  
  bool HierarchyTree::isTerminal(tree<SpLearningStateConcept>::iterator Node) {
      
    if(isEmptyTree(Node))
      return false;
      
    if(conceptsHierarchy.number_of_children(Node) == 0)
        return true;

    return false;
  }
  
  bool HierarchyTree::isEmptyTree(tree<SpLearningStateConcept>::iterator Node) {
    if((Node==conceptsHierarchy.end())&&(Node==conceptsHierarchy.begin()))
      return true;
    return false;
  }

  double HierarchyTree::computeSplittingNode(tree<SpLearningStateConcept>::iterator best,
                                             tree<SpLearningStateConcept>::iterator Node,
                                             SpInstance Instance) {
    tree<SpLearningStateConcept>::sibling_iterator sib;
    tree<SpLearningStateConcept>::iterator best_child;
    SpLearningStateConcept auxNode;
    int K = conceptsHierarchy.number_of_children(best) - 1 + conceptsHierarchy.number_of_children(Node);
    int cnum=conceptsHierarchy.number_of_children(best);
    int I;
    int N=(*Node)->getN(), nattr;
    int i, j, k;
    double k_split;
    double best_score=MINIMAL_DOUBLE, best_diff=0.0, score, diff, split_score;
    double P, k_children[cnum], kp_num, kp_sym;
  

    //Sum immediatelly the fixed components of the CU
    score=0.0;
    for(i=0,sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);i++,sib++)
      if(sib!=best)
	score+=k_components[i];

    /*Generate k-components for children without incorporation:*/
    for(i=0,sib=conceptsHierarchy.begin(best);sib!=conceptsHierarchy.end(best);i++,sib++) {
      P = (double)(*sib)->getN() / (double) N;
      kp_num = kp_sym = 0.0;

      if( (I = (*sib)->numericalAttributeList.size()) > 0) {
	std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = (*sib)->numericalAttributeList.begin();
	for(k = 0; k < I; k++, numericalIt++)
	  kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
	kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
      }

      if( (I = (*sib)->symbolicAttributeList.size()) > 0) {
	std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = (*sib)->symbolicAttributeList.begin();
	int j, m;
	double kp_att, p;
	std::map<std::string, SpSymbolicValue>::iterator values;
	SpSymbolicAttributeValuePair currentSym;	
	
	for(k = 0; k < I; k++, symbolicIt++) {
	  currentSym = (*symbolicIt).second;
	  values = currentSym->values.begin();
	  m = currentSym->values.size();
	  kp_att = 0.0;
	  for(j = 0; j < m; j++, values++) {
	    if(currentSym->RSum == 0)
	      p = 0.0;
	    else
	      p = (*values).second->RSum / currentSym->RSum;
	    kp_att += p*p;
	  }
	  kp_sym += kp_att;     /*k-component for the parent node.*/
	}
	
	kp_sym = kp_sym / (double)I;
      }
      
      k_children[i] = P * (kp_num + kp_sym);

    }
    
    /*k-component for splitting.*/
    //Search for the best children for CU
    for(i=0,sib=conceptsHierarchy.begin(best);sib!=conceptsHierarchy.end(best);i++,sib++) {

      split_score=0.0;
      diff=0.0;
      nattr=0;

      //Sum the children scores not incorporating
      for(j=0; j<cnum; j++)
	if(j != i)
	  split_score += k_children[j];
      
      //Calculate score for the children incorporating
      auxNode = copyNode(*sib);
      Incorporate(auxNode, Node, Instance);      

      /*Set score for split node.*/
      kp_num = kp_sym = 0.0;

      if( (I = auxNode->numericalAttributeList.size()) > 0) {
	std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = auxNode->numericalAttributeList.begin();
	for(k = 0; k < I; k++, numericalIt++) {
	  kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
	  diff += (*numericalIt).second->distribution->getValue((*Instance->numericalData)[(*numericalIt).first]->value);
	}
	kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
	nattr += I;
      }

      if( (I = auxNode->symbolicAttributeList.size()) > 0) {
	std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = auxNode->symbolicAttributeList.begin();
	int j, m;
	double kp_att, p;
	std::map<std::string, SpSymbolicValue>::iterator values;
	SpSymbolicAttributeValuePair currentSym;	
	
	for(k = 0; k < I; k++, symbolicIt++) {
	  currentSym = (*symbolicIt).second;
	  values = currentSym->values.begin();
	  m = currentSym->values.size();
	  diff += currentSym->values[ (*Instance->symbolicData)[(*symbolicIt).first]->value ]->P;
	  kp_att = 0.0;
	  for(j = 0; j < m; j++, values++) {
	    if(currentSym->RSum == 0)
	      p = 0.0;
	    else
	      p = (*values).second->RSum / currentSym->RSum;
	    kp_att += p*p;
	  }
	  kp_sym += kp_att;     /*k-component for the parent node.*/
	}
	kp_sym = kp_sym / (double)I;
	nattr += I;
      }
      P = (double) auxNode->getN() / (double) N;
      k_split = P*(kp_num + kp_sym);
													   
      split_score = (split_score + score + k_split - k_parent) / (double) K;

      //Testing new theory
      //split_score = (split_score + score + k_split - k_parent);

      diff /= (double) nattr;

      if(split_score - best_score > EPSILON || (fabs(split_score - best_score) <= EPSILON && diff > best_diff)) {
	best_score = split_score;
	best_diff = diff;
      } 
    }

    diffSplit = best_diff;

    return best_score;

  }

  SpLearningStateConcept HierarchyTree::computeMergingNodes(tree<SpLearningStateConcept>::iterator best,
                                                          tree<SpLearningStateConcept>::iterator second,
                                                          tree<SpLearningStateConcept>::iterator Node,
                                                          SpInstance Instance) {
    
    tree<SpLearningStateConcept>::sibling_iterator sib;
    SpLearningStateConcept mergeNode, auxNode;
    int K = conceptsHierarchy.number_of_children(Node) - 1;
    int I, N=(*Node)->getN(), nattr = 0;
    int i;
    double k_merge;
    double score, diff = 0.0;
    
    /*k-component for merging.*/
    mergeNode = mergeNodes(best, second);
    auxNode = copyNode(mergeNode);
    Incorporate(auxNode, Node, Instance);
    
    /*Set score for merge node.*/
    double kp_num = 0.0, kp_sym = 0.0, P;

    if( (I = auxNode->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = auxNode->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++) {
	kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
	diff += (*numericalIt).second->distribution->getValue((*Instance->numericalData)[(*numericalIt).first]->value);
      }
      kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
      nattr += I;
    }

    if( (I = auxNode->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = auxNode->symbolicAttributeList.begin();
      int j, m;
      double kp_att, p;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;
	values = currentSym->values.begin();
	m = currentSym->values.size();
	diff += currentSym->values[ (*Instance->symbolicData)[(*symbolicIt).first]->value]->P;
	kp_att = 0.0;
	for(j = 0; j < m; j++, values++) {
	  if(currentSym->RSum == 0)
	    p = 0.0;
	  else
	    p = (*values).second->RSum / currentSym->RSum;
	  kp_att += p*p;
	}
	kp_sym += kp_att;     /*k-component for the parent node.*/
      }

      kp_sym = kp_sym / (double)I;
      nattr += I;
    }
    P = (double)(auxNode->getN())/(double)N;
    k_merge = P * (kp_num + kp_sym);

    score = k_merge;

    for(i=0,sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);i++,sib++)
      if((sib!=best) && (sib!=second))
	score+=k_components[i];

    score = (score - k_parent) / (double) K;
    //Testing new theory
    //score = (score - k_parent);
    
    mergeNode->setScore(score);

    diffMerge = diff / (double) nattr;
    mergeNode->setDiff(diffMerge);

    return mergeNode;
  }
    

  SpLearningStateConcept HierarchyTree::mergeNodes(tree<SpLearningStateConcept>::iterator P,
                                                 tree<SpLearningStateConcept>::iterator R ) {
    int i, I;
    SpLearningStateConcept mergeNode = copyNode(*P);

    mergeNode->setN((*P)->getN() + (*R)->getN());
    
    //Numerical attributes
    if( (I = (*P)->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numPIt = (*P)->numericalAttributeList.begin();
      SpNumericalAttributeValuePair numP, numR, numMerge; 
      double sP, sR, mean, sigma, aP, aR, meanP, meanR;
      attribute_id att_id;
      for(i = 0; i < I; i++, numPIt++) {
	att_id = (*numPIt).first;
	numP = (*numPIt).second;
	numR = (*R)->numericalAttributeList[att_id];
	numMerge = mergeNode->numericalAttributeList[att_id];

	numMerge->RSum = numP->RSum + numR->RSum;
	meanP = numP->getMean(); 
	meanR = numR->getMean(); 
	mean = (meanP*numP->RSum + meanR*numR->RSum)/numMerge->RSum;
	numMerge->setMean(mean);
	sP = numP->getSigma();
	sR = numR->getSigma();
	aP = numP->RSum/numMerge->RSum;
	aR = numR->RSum/numMerge->RSum;
	sigma = sqrt(((mean - meanP)*(mean - meanP) + sP*sP)*aP + ((mean - meanR)*(mean - meanR) + sR*sR)*aR);
	numMerge->setSigma(sigma);
      }
    }

    //Symbolic attributes
    if( (I = (*P)->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symPIt = (*P)->symbolicAttributeList.begin();
      SpSymbolicAttributeValuePair symP, symR, symMerge; 
      attribute_id att_id;
      int j, m;
      std::map<std::string, SpSymbolicValue>::iterator valPIt;
      std::string val_id;
      SpSymbolicValue valP, valR, valMerge;

      for(i = 0; i < I; i++, symPIt++) {
	att_id = (*symPIt).first;
	symP = (*symPIt).second;
	symR = (*R)->symbolicAttributeList[att_id];
	symMerge = mergeNode->symbolicAttributeList[att_id];

	symMerge->RSum = symP->RSum + symR->RSum;
	valPIt = symP->values.begin();
	m = symP->values.size();
	for(j = 0; j < m; j++, valPIt++) {
	  val_id = (*valPIt).first;
	  valP = (*valPIt).second;
	  valR = symR->values[val_id];
	  valMerge = symMerge->values[val_id];

	  valMerge->RSum = valP->RSum + valR->RSum;
	  valMerge->N = valP->N + valR->N;
	  valMerge->P = valMerge->RSum/ symMerge->RSum;
	}
      }
    }
    
    return mergeNode;
  }
  
  void HierarchyTree::computeChildrenScores(tree<SpLearningStateConcept>::iterator Node,
                                             SpInstance Instance) {

    tree<SpLearningStateConcept>::sibling_iterator sib;

    int K = conceptsHierarchy.number_of_children(Node), I;
    int N = (*Node)->getN(), nattr;
    int i, j, k;
    double k_current, score_aux, diff_aux;
    double kp_num, kp_sym, P;
    
    SpLearningStateConcept currentNode;

    /*Set scores for actual nodes.*/
    for(i=0,sib=conceptsHierarchy.begin(Node);sib!=conceptsHierarchy.end(Node);i++,sib++) {
      score_aux=0.0;
      for(j=0;j<K;j++)
	if(i==j) {
	  diff_aux=0.0;
	  nattr = 0;

	  currentNode=copyNode((*sib));
	  Incorporate(currentNode, Node, Instance);	    
	  P = (double)currentNode->getN() / (double) N;

	  kp_num = kp_sym = 0.0;
	  if( (I = currentNode->numericalAttributeList.size()) > 0) {
	    std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = currentNode->numericalAttributeList.begin();
	    for(k = 0; k < I; k++, numericalIt++) {
	      kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     
	      diff_aux += (*numericalIt).second->distribution->getValue((*Instance->numericalData)[(*numericalIt).first]->value);
	    }
	    kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
	    nattr += I;
	  }

	  if( (I = currentNode->symbolicAttributeList.size()) > 0) {
	    std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = currentNode->symbolicAttributeList.begin();
	    int l, m;
	    double kp_att, p;
	    std::map<std::string, SpSymbolicValue>::iterator values;
	    SpSymbolicAttributeValuePair currentSym;	

	    for(k = 0; k < I; k++, symbolicIt++) {
	      currentSym = (*symbolicIt).second;
	      values = currentSym->values.begin();
	      m = currentSym->values.size();
	      diff_aux += currentSym->values[ (*Instance->symbolicData)[(*symbolicIt).first]->value]->P;
	      kp_att = 0.0;
	      for(l = 0; l < m; l++, values++) {
		p = (*values).second->RSum / currentSym->RSum;
		kp_att += p*p;
	      }
	      kp_sym += kp_att;     /*k-component for the parent node.*/
	    }

	    kp_sym = kp_sym / (double)I;
	    nattr += I;
	  }

	  k_current = P*(kp_num + kp_sym);
	    
	  score_aux += k_current;
	  (*sib)->setDiff(diff_aux / (double) nattr);
	} else
	  score_aux += k_components[j];
      
      score_aux = (score_aux - k_parent) / (double) K;
      //Testing New Theory
      //score_aux = (score_aux - k_parent);

      (*sib)->setScore(score_aux);
      
    }
  }

  SpLearningStateConcept HierarchyTree::computeNewChild(tree<SpLearningStateConcept>::iterator Node,
                                                      SpInstance Instance) {

    SpLearningStateConcept newNode;
    int N = (*Node)->getN(), nattr = 0;
    int K = conceptsHierarchy.number_of_children(Node), I;
    double initP = (double)1.0 / (double)N;
    double k_new, kp_num, kp_sym, score, diff = 0.0;
    int i, j;

    newNode = copyInstance(Instance);
    newNode->setP(initP);
    
    /*Compute new child score*/
    kp_num = kp_sym = 0.0;

    if( (I = newNode->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = newNode->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++) {
	kp_num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     
	diff += (*numericalIt).second->distribution->getValue((*Instance->numericalData)[(*numericalIt).first]->value);
      }
      kp_num = kp_num / (double)(I * DOUBLEOFSQRTPI);
      nattr += I;
    }

    if( (I = newNode->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = newNode->symbolicAttributeList.begin();
      int l, m;
      double kp_att, p;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;
	values = currentSym->values.begin();
	m = currentSym->values.size();
	diff += currentSym->values[ (*Instance->symbolicData)[(*symbolicIt).first]->value]->P;
	kp_att = 0.0;
	for(l = 0; l < m; l++, values++) {
	  if(currentSym->RSum == 0)
	    p = 0.0;
	  else
	    p = (*values).second->RSum / currentSym->RSum;
	  kp_att += p*p;
	}
	kp_sym += kp_att;     /*k-component for the parent node.*/
      }

      kp_sym = kp_sym / (double)I;
      nattr += I;
    }

    score= k_new = initP*(kp_num + kp_sym);
    
    for(j=0;j<K;j++)
      score+=k_components[j];
    
    score = (score - k_parent) / (double)(K + 1);
    //Testing New Theory
    //score = (score - k_parent);

    newNode->setScore(score);

    diffNew = diff / (double) nattr;
    newNode->setDiff(diffNew);

    return newNode;

  }
  
  void HierarchyTree::createNewTerminal(tree<SpLearningStateConcept>::iterator Node,
                                         SpInstance Instance) {

    SpLearningStateConcept newNodeNode, newNodeInstance;
    double initINP = (double) 1.0 / (double) ((*Node)->getN() + 1.0);
    double initNNP = (double) ((*Node)->getN()) / (double) ((*Node)->getN() + 1.0);
    terminalDone = true;

    newNodeNode = copyNode((*Node));
    newNodeInstance = copyInstance(Instance);
    newNodeInstance->parent = newNodeNode->parent = (*Node)->id;

    newNodeNode->setP(initNNP);
    newNodeInstance->setP(initINP);

    newNodeNode->id = free_state_id;
    conceptsHierarchy.append_child(Node, newNodeNode);
    registeredStates[free_state_id++] = newNodeNode;

    newNodeInstance->id = free_state_id;
    conceptsHierarchy.append_child(Node, newNodeInstance);
    registeredStates[free_state_id++] = newNodeInstance;

    deepest_for_instance++;
    currentStatesForInstance[deepest_for_instance] = newNodeInstance;

    //Events
    copyEvents(*Node, newNodeNode);

    //Check if the contextualised object was already present in parent node. 
    //If this is the case, add the new node node as the previous node at this new level
    std::map<unsigned long, SpLearningContextualisedObject>::iterator objIt = currentContext->contextualisedObjects.begin();
    int i, n = currentContext->contextualisedObjects.size(), datasize;
    SpLearningContextualisedObject analysedObject;
    std::map<int, SpLevelData> *leveldata;
    SpLevelData oldData;
    for(i=0; i<n; i++, objIt++) {
      analysedObject = (*objIt).second;
      leveldata = &analysedObject->levelData[lc_id];
      datasize = leveldata->size();
      if(datasize > deepest_for_instance-1) {
	if((*leveldata)[deepest_for_instance-1]->currentState == (*Node)->id) {
          SpLevelData currentData(new LevelData());
	  oldData = analysedObject->levelData[lc_id][deepest_for_instance-1];
	  currentData->currentState = newNodeNode->id;
	  currentData->timeStayingAtCurrent = oldData->timeStayingAtCurrent;
	  currentData->analysedFrames = oldData->analysedFrames;
	  currentData->lastEvents = oldData->lastEvents;
	  analysedObject->levelData[lc_id][deepest_for_instance] = currentData;
	}
      }
    }
  }
  
  void HierarchyTree::copyEvents(SpLearningStateConcept original, SpLearningStateConcept toCopy) {
    int i, n = original->incomingEvents.size(), m = original->outgoingEvents.size(); 
    std::map <long int, SpLearningEventConcept>::iterator incomingIt = original->incomingEvents.begin();
    //Indexed by the id of the arriving state
    std::map <long int, SpLearningEventConcept>::iterator outgoingIt = original->outgoingEvents.begin();
    long int event_id, current_id = toCopy->id, other_id;

    SpLearningEventConcept oldEvent;
    SpLearningStateConcept otherState;

    //Copy incoming Events
    //Make link with the leaving state
    toCopy->NE = original->NE;
    toCopy->incomingEvents.clear();

    for(i=0; i<n; i++, incomingIt++) {
      other_id = (*incomingIt).first;
      oldEvent = (*incomingIt).second;
      otherState = registeredStates[other_id];

      //Create and update event
      event_id = free_event_id;
      free_event_id++;
      SpLearningEventConcept newEvent(new LearningEventConcept(event_id, oldEvent->getMean(), oldEvent->getSigma()));
      newEvent->N = oldEvent->N;
      newEvent->P = oldEvent->P;
      newEvent->minT = oldEvent->minT; 
      newEvent->maxT = oldEvent->maxT;
      newEvent->startingState = other_id;
      newEvent->arrivingState = current_id;

      //Update copy state
      toCopy->incomingEvents[other_id] = newEvent;

      //Update starting state
      otherState->outgoingEvents[current_id] = newEvent;

      //Update HierarchyTree
      registeredEvents[event_id] = newEvent;
    }

    //Copy outgoing Events
    //Make link with the starting state
    toCopy->outgoingEvents.clear();
    for(i=0; i<m; i++, outgoingIt++) {
      other_id = (*outgoingIt).first;
      oldEvent = (*outgoingIt).second;
      otherState = registeredStates[other_id];

      //Create and update event
      event_id = free_event_id;
      free_event_id++;
      SpLearningEventConcept newEvent(new LearningEventConcept(event_id, oldEvent->getMean(), oldEvent->getSigma()));
      newEvent->N = oldEvent->N;
      newEvent->P = oldEvent->P;
      newEvent->minT = oldEvent->minT; 
      newEvent->maxT = oldEvent->maxT;
      newEvent->startingState = current_id;
      newEvent->arrivingState = other_id;

      //Update copy state
      toCopy->outgoingEvents[other_id] = newEvent;

      //Update starting state
      otherState->incomingEvents[current_id] = newEvent;

      //Update HierarchyTree
      registeredEvents[event_id] = newEvent;
    }

  }

  double HierarchyTree::categoryUtilityState(SpLearningStateConcept state, int totalN, double &totalP) {
    double P = (double) state->N / (double) totalN, num = 0.0, sym = 0.0;
    int i, I;
    
    //Store the contribution to normalisation
    totalP += P;

    if( (I = state->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = state->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++)
	num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
      num = num / (double)(I * DOUBLEOFSQRTPI);
    }

    if( (I = state->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = state->symbolicAttributeList.begin();
      int j, m;
      double kp_att, p;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;
	values = currentSym->values.begin();
	m = currentSym->values.size();
	kp_att = 0.0;
	for(j = 0; j < m; j++, values++) {
	  if(currentSym->RSum == 0)
	    p = 0.0;
	  else
	    p = (*values).second->RSum / currentSym->RSum;
	  kp_att += p*p;
	}
	sym += kp_att;     /*k-component for the parent node.*/
      }

      sym = sym / (double)I;
    }

    return P*(num + sym);

  }

  double HierarchyTree::categoryUtilityState(SpLearningStateConcept state) {
    double num = 0.0, sym = 0.0;
    int i, I, numAttr = 0;
    
    if( (I = state->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = state->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numericalIt++)
	num += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
      numAttr = I;
    }

    if( (I = state->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = state->symbolicAttributeList.begin();
      int j, m;
      double kp_att, p;
      std::map<std::string, SpSymbolicValue>::iterator values;
      SpSymbolicAttributeValuePair currentSym;	

      for(i = 0; i < I; i++, symbolicIt++) {
	currentSym = (*symbolicIt).second;
	values = currentSym->values.begin();
	m = currentSym->values.size();
	kp_att = 0.0;
	for(j = 0; j < m; j++, values++) {
	  if(currentSym->RSum == 0)
	    p = 0.0;
	  else
	    p = (*values).second->RSum / currentSym->RSum;
	  kp_att += p*p;
	}
	sym += kp_att;     /*k-component for the parent node.*/
      }

      numAttr += I;
    }

    return (num + sym) / (double)numAttr;

  }


  
  double HierarchyTree::categoryUtility(bool with_parent) {
    tree<SpLearningStateConcept>::iterator Node=conceptsHierarchy.begin();

    if(isEmptyTree(Node))
      return 0.0;

    int i, I, kids, level = 0, K = conceptsHierarchy.size();
    double num_parent = 0.0, sym_parent = 0.0, totalCU = 0.0;
    tree<SpLearningStateConcept>::fixed_depth_iterator level_iter;
    
    if(K == 1) //If only the root state is present
      return 0.0;

    if(with_parent) {

      if( (I = (*Node)->numericalAttributeList.size()) > 0) {
	std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numericalIt = (*Node)->numericalAttributeList.begin();
	for(i = 0; i < I; i++, numericalIt++)
	  num_parent += currentContext->numericalAttributes[(*numericalIt).first]->acuity / (*numericalIt).second->getSigma();     /*k-component for the parent node.*/
	num_parent = num_parent / (double)(I * DOUBLEOFSQRTPI);
      }
      
      if( (I = (*Node)->symbolicAttributeList.size()) > 0) {
	std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symbolicIt = (*Node)->symbolicAttributeList.begin();
	int j, m;
	double kp_att, p;
	std::map<std::string, SpSymbolicValue>::iterator values;
	SpSymbolicAttributeValuePair currentSym;	
	
	for(i = 0; i < I; i++, symbolicIt++) {
	  currentSym = (*symbolicIt).second;
	  values = currentSym->values.begin();
	  m = currentSym->values.size();
	  kp_att = 0.0;
	  for(j = 0; j < m; j++, values++) {
	    if(currentSym->RSum == 0)
	      p = 0.0;
	    else
	      p = (*values).second->RSum / currentSym->RSum;
	    kp_att += p*p;
	  }
	  sym_parent += kp_att;     /*k-component for the parent node.*/
	}
	
	sym_parent = sym_parent / (double)I;
      }
    }

    int totalN = (*Node)->N;
    double totalP = 0.0; //Normalisation factor

    kids=conceptsHierarchy.number_of_children(Node);

    for(level=1;kids>0;level++) {
      kids=0;
      for(level_iter=conceptsHierarchy.begin_fixed(Node, level);
	  conceptsHierarchy.is_valid(level_iter);level_iter++) {
        kids+=conceptsHierarchy.number_of_children(level_iter);
	totalCU += categoryUtilityState(*level_iter, totalN, totalP);
      }
    }

    if(with_parent)
      return (totalCU / totalP) - num_parent - sym_parent;

    return (totalCU / totalP);

  }
  
  void HierarchyTree::insertNewChild(tree<SpLearningStateConcept>::iterator Node, SpInstance Instance) {
      
    SpLearningStateConcept newNode;
    double initP = (double) 1.0 / (double) (double)((*Node)->getN());
    insertionDone = true;
    
    newNode = copyInstance(Instance);
    newNode->setN(1);
    newNode->setP(initP);
    newNode->id = free_state_id++;
    newNode->parent = (*Node)->id;
    conceptsHierarchy.append_child(Node, newNode);

    currentStatesForInstance[deepest_for_instance] = newNode;
    registeredStates[newNode->id] = newNode;

  }
  
  SpLearningStateConcept HierarchyTree::copyNode(SpLearningStateConcept Node) {
    
    int i, I;
    SpLearningStateConcept newNode(new LearningStateConcept());
    newNode->N = Node->N;;

    if( (I = Node->numericalAttributeList.size()) > 0) {
      std::map <attribute_id, SpNumericalAttributeValuePair>::iterator numIt = Node->numericalAttributeList.begin();
      for(i = 0; i < I; i++, numIt++)
	newNode->insertNumericalPair( (*numIt).first, (*numIt).second->getMean(), (*numIt).second->getSigma(), (*numIt).second->getRSum());	
    }

    if( (I = Node->symbolicAttributeList.size()) > 0) {
      std::map <attribute_id, SpSymbolicAttributeValuePair>::iterator symIt = Node->symbolicAttributeList.begin();
      for(i = 0; i < I; i++, symIt++)
	newNode->insertSymbolicPair( (*symIt).first, (*symIt).second->getRSum(), &(*symIt).second->values);	
    }

    return newNode;
  }
    
  SpLearningStateConcept HierarchyTree::copyInstance(SpInstance Instance) {

    SpLearningStateConcept newNode(new LearningStateConcept());
    int i, I;
    newNode->setN(1);

    
    if( Instance->numericalData != NULL && (I = Instance->numericalData->size()) > 0) {
      std::map<attribute_id, SpContextualisedNumericalData>::iterator numIt = Instance->numericalData->begin();
      for(i = 0; i < I; i++, numIt++)
	newNode->insertNumericalPair( (*numIt).first, (*numIt).second->value, currentContext->numericalAttributes[(*numIt).first]->acuity, (*numIt).second->reliability);	

    }

    if( Instance->symbolicData != NULL && (I = Instance->symbolicData->size()) > 0) {
      std::map <attribute_id, SpContextualisedSymbolicData>::iterator symIt = Instance->symbolicData->begin();
      for(i = 0; i < I; i++, symIt++)
	newNode->insertSymbolicPair( (*symIt).first, (*symIt).second->reliability, (*symIt).second->value, availableAttributes->symbolicValuesList[(*symIt).first]);	
    }

    return newNode;
  }
  
  void HierarchyTree::insertRoot(tree<SpLearningStateConcept>::iterator Node, SpInstance Instance) {
    
    SpLearningStateConcept newNode = copyInstance(Instance);
    newNode->setP(1.0);
    newNode->id = free_state_id++;
    registeredStates[newNode->id] = newNode;
    conceptsHierarchy.insert(Node, newNode); 
  }
  
  
  void HierarchyTree::Incorporate(SpLearningStateConcept Node, tree<SpLearningStateConcept>::iterator Parent,
                                   SpInstance Instance) {
      
    int i, I, N = Node->getN();
    
    if( Instance->numericalData != NULL && (I = Instance->numericalData->size()) > 0) {
      std::map<attribute_id, SpContextualisedNumericalData>::iterator numIt = Instance->numericalData->begin();
      for(i = 0; i < I; i++, numIt++)
	incorporateNumericalValue(Node->numericalAttributeList[(*numIt).first], (*numIt).second, currentContext->numericalAttributes[(*numIt).first]->acuity);	
    }

    if( Instance->symbolicData != NULL && (I = Instance->symbolicData->size()) > 0) {
      std::map <attribute_id, SpContextualisedSymbolicData>::iterator symIt = Instance->symbolicData->begin();
      for(i = 0; i < I; i++, symIt++)
	incorporateSymbolicValue( Node->symbolicAttributeList[(*symIt).first], (*symIt).second);
    }

    Node->setN( N + 1 );
    
    if(Parent != NULL)
      Node->setP( (double)(Node->getN()) / (double) ((*Parent)->getN()) );
    
  }

  void HierarchyTree::incorporateNumericalValue(SpNumericalAttributeValuePair nodePair, SpContextualisedNumericalData instancePair, double acuity) {

    double 
      value = instancePair->value,
      mean = nodePair->getMean(),
      reliability = instancePair->reliability,
      RSum = nodePair->RSum;
    //Sigma formula utilises previous mean value
    nodePair->setSigma( incrementalSigma(mean, nodePair->getSigma(), value, reliability, RSum, acuity) );
    nodePair->setMean( incrementalMean(mean, value, RSum, reliability) );
    nodePair->RSum += reliability; 
  }

  double HierarchyTree::incrementalMean(double mean, double value, double RSum, double reliability) {
    return (value*reliability + mean*RSum)/(reliability + RSum);
  }

  double HierarchyTree::incrementalSigma(double mean, double sigma, double value, double reliability, double RSum, double acuity) {
    double diff=value - mean;
    double RSumPlusR = reliability + RSum;
    double result=sqrt( (sigma*sigma + reliability*diff*diff/RSumPlusR)*RSum/RSumPlusR );
    if(result < acuity)
      return acuity;

    return result;
  }


  void HierarchyTree::incorporateSymbolicValue(SpSymbolicAttributeValuePair nodePair, SpContextualisedSymbolicData instancePair) {

    std::string value = instancePair->value;
    double reliability = instancePair->reliability;

    nodePair->RSum += reliability;

    double RSum = nodePair->RSum;
    std::map<std::string, SpSymbolicValue>::iterator valuesIt = nodePair->values.begin();
    int i, n = nodePair->values.size();

    for(i=0; i<n; i++, valuesIt++) {
      if((*valuesIt).first == value) {
	(*valuesIt).second->RSum += reliability;
	(*valuesIt).second->N++;
      }
      (*valuesIt).second->P = (*valuesIt).second->RSum / RSum;
    }
  }

}
