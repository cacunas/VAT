#ifndef HIERARCHYTREE_H
#define HIERARCHYTREE_H

#include "IncrementalEventLearning.h"
#include "tree.hh"

#define MINIMAL_DOUBLE -10e10
#define MAXIMAL_DOUBLE 10e10
#define EPSILON 0.0000001 
#define ALMOST_ONE 0.99 
#define DOUBLEOFSQRTPI 3.544907701811

namespace miles {

  enum categories { NEW_NODE,
		    MERGING_NODE,
		    BEST_NODE,
		    SPLIT_NODE,
		    CAT_NUMBER
  };

  class Instance {
  public:
    std::map<attribute_id, SpContextualisedNumericalData > *numericalData;
    std::map<attribute_id, SpContextualisedSymbolicData > *symbolicData;
    Instance();
    ~Instance();
    void setInstance(std::map<attribute_id, SpContextualisedNumericalData > *i_numericalData,
                     std::map<attribute_id, SpContextualisedSymbolicData > *i_symbolicData);
      
  };
  
  typedef QSharedPointer<Instance> SpInstance;

  class HierarchyTree {
  public:    
    static SpLearningAttributes availableAttributes;
    static double minConditionalP;

    tree <SpLearningStateConcept> conceptsHierarchy;
    std::map<long int, SpLearningStateConcept> registeredStates;
    std::map<long int, SpLearningEventConcept> registeredEvents;

    int lc_id;
    SpInstance currentInstance;
    SpLearningContextualisedObject currentObject;
    SpLearningContext currentContext;
    std::map<int, SpLearningStateConcept> currentStatesForInstance;

    double *k_components, k_parent; 
    int deepest_level;
    int deepest_for_instance;
    long int free_state_id;
    long int free_event_id;
    double tdiff;
    bool insertionDone, terminalDone, splitDone, mergeDone;
    double diffSplit, diffNew, diffBest, diffMerge;

    HierarchyTree();
    ~HierarchyTree();

    bool getDeepestSiblingStates(double P, std::map<long int, SpLearningStateConcept> &states,
                                 attribute_id _x, attribute_id _y);
    bool recurseToDeepestSiblingStates(double P, tree<SpLearningStateConcept>::sibling_iterator node,
                                       std::map<long int, SpLearningStateConcept> &states,
                                       attribute_id _x, attribute_id _y);


    bool getMostLikelySiblingStates(double P, std::map<long int, SpLearningStateConcept> &states);
    void recurseToMostLikelySiblingStates(double P, tree<SpLearningStateConcept>::sibling_iterator node,
                                          std::map<long int, SpLearningStateConcept> &states);
    double categoryUtility(bool with_parent);
    double categoryUtilityState(SpLearningStateConcept state, int totalN, double &totalP);
    double categoryUtilityState(SpLearningStateConcept state);
    bool kinshipRelation(SpLearningStateConcept state1, SpLearningStateConcept state2);
    tree <SpLearningStateConcept> *getHierarchy();
    void setDepth(int level);
    void setDepth();
    int getDepth();
    int execute(SpLearningContext, SpLearningContextualisedObject, double);
    void print();
    void relabelTree();
    bool cutoff(tree<SpLearningStateConcept>::iterator, SpInstance Instance);
    void MILES(tree<SpLearningStateConcept>::iterator, SpInstance Instance, bool);
  
    int setDistancesToLeaf(tree<SpLearningStateConcept>::sibling_iterator Node);
    void setDistancesToLeaf(tree<SpLearningStateConcept>::iterator Node);

    void copyEvents(SpLearningStateConcept original, SpLearningStateConcept toCopy);


    tree<SpLearningStateConcept>::iterator doMerge(tree<SpLearningStateConcept>::iterator,
                                                 tree<SpLearningStateConcept>::iterator,
                                                 tree<SpLearningStateConcept>::iterator,
                                                 SpLearningStateConcept);
      
    void doSplit(tree<SpLearningStateConcept>::iterator, tree<SpLearningStateConcept>::iterator);
      
    int determineBest(double, double, double, double);

    void getBestChildren(tree<SpLearningStateConcept>::iterator *,
                         tree<SpLearningStateConcept>::iterator *,
                         tree<SpLearningStateConcept>::iterator );

    void computeKParent();
    
    double *computeKComponents(tree<SpLearningStateConcept>::iterator);
      
    void updateChildrenProbabilities(tree<SpLearningStateConcept>::iterator);

    bool isTerminal(tree<SpLearningStateConcept>::iterator);
  
    bool isEmptyTree(tree<SpLearningStateConcept>::iterator);

    double computeSplittingNode(tree<SpLearningStateConcept>::iterator,
                                tree<SpLearningStateConcept>::iterator,
                                SpInstance);
  
    SpLearningStateConcept computeMergingNodes(tree<SpLearningStateConcept>::iterator,
                                             tree<SpLearningStateConcept>::iterator,
                                             tree<SpLearningStateConcept>::iterator,
                                             SpInstance);

    SpLearningStateConcept mergeNodes(tree<SpLearningStateConcept>::iterator,
                                    tree<SpLearningStateConcept>::iterator );
      
    void computeChildrenScores(tree<SpLearningStateConcept>::iterator, SpInstance);

    SpLearningStateConcept computeNewChild(tree<SpLearningStateConcept>::iterator, SpInstance);
  
    void createNewTerminal(tree<SpLearningStateConcept>::iterator, SpInstance);

    void insertNewChild(tree<SpLearningStateConcept>::iterator, SpInstance);
  
    SpLearningStateConcept copyNode(SpLearningStateConcept);
    
    SpLearningStateConcept copyInstance(SpInstance);
  
    void insertRoot(tree<SpLearningStateConcept>::iterator, SpInstance);
  
    void Incorporate(SpLearningStateConcept, tree<SpLearningStateConcept>::iterator, SpInstance);

    void incorporateNumericalValue(SpNumericalAttributeValuePair nodePair, SpContextualisedNumericalData instancePair, double acuity);

    void incorporateSymbolicValue(SpSymbolicAttributeValuePair nodePair, SpContextualisedSymbolicData instancePair);

    double incrementalMean(double, double, double, double);

    double incrementalSigma(double, double, double, double, double, double);

  };

  struct orderedByDistanceToLeafOperator {
    bool operator()(SpLearningStateConcept concept1, SpLearningStateConcept concept2);
  };

  struct orderedByStateProbabilityOperator {
    bool operator()(SpLearningStateConcept concept1, SpLearningStateConcept concept2);
  };

}

#endif
