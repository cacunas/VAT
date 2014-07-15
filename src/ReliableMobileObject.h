#ifndef RELIABLEMOBILE_OBJECT_H
#define RELIABLEMOBILE_OBJECT_H

#include "blob.h"
#include "RMAlternativeSolution.h"
#include <iostream>
#include <map>
#include <deque>

class ReliableMobileObject;

typedef QSharedPointer<ReliableMobileObject> SpReliableMobileObject;

class ReliableMobileObject {
   
  private:
    int bestSolutionIndex;
    double bestMeanProbability;


  public:
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> alternativeSolutions;
    static int m_currentTrackingBlobsNumber;
    int incompleteAlternatives;
    int numUsed;
    bool toErase;
    ReliableMobileObject();
    ~ReliableMobileObject();

    //Blobs used in "follow mobiles" phase.
    bool *usedBlobs;
    bool *involvedBlobs;

    //Init Functions
    void initUsedBlobsList();
    void initInvolvedBlobs();

    //Inserting Functions
    void insertNewSolution(SpRMAlternativeSolution solution);

    //Setting Functions
    void setBestSolution();

    //Getting Functions
    double getBestProbability();
    int getBestSolutionIndex();
    SpRMAlternativeSolution getBestSolution();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>* getAlternativeSolutions();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator begin();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator end();
    void erase(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator last);
    void erase(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator element);

    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator operator[](int);
    unsigned int size();
    bool empty();
    void clear();
    void insert(SpRMAlternativeSolution);
    void insert(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator first,
                std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator last);
    friend std::ostream& operator<<(std::ostream&, SpReliableMobileObject);
 };


typedef ReliableMobileObject* rmobile_object_t;

#endif	
