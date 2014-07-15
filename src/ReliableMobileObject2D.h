#ifndef RELIABLEMOBILE_OBJECT2D_H
#define RELIABLEMOBILE_OBJECT2D_H

#include "blob.h"
#include "RMAlternativeSolution2D.h"
#include <iostream>
#include <map>
#include <deque>

class ReliableMobileObject2D;

typedef QSharedPointer<ReliableMobileObject2D> SpReliableMobileObject2D;

class ReliableMobileObject2D {
   
  private:
    int bestSolutionIndex;
    double bestMeanProbability;


  public:
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator> alternativeSolutions;
    static int m_currentTrackingBlobsNumber;
    int incompleteAlternatives;
    int numUsed;
    bool toErase;
    ReliableMobileObject2D();
    ~ReliableMobileObject2D();

    //Blobs used in "follow mobiles" phase.
    bool *usedBlobs;
    bool *involvedBlobs;

    //Init Functions
    void initUsedBlobsList();
    void initInvolvedBlobs();

    //Inserting Functions
    void insertNewSolution(SpRMAlternativeSolution2D solution);

    //Setting Functions
    void setBestSolution();

    //Getting Functions
    double getBestProbability();
    int getBestSolutionIndex();
    SpRMAlternativeSolution2D getBestSolution();
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>* getAlternativeSolutions();
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator begin();
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator end();
    void erase(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator last);
    void erase(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator element);

    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator operator[](int);
    unsigned int size();
    bool empty();
    void clear();
    void insert(SpRMAlternativeSolution2D);
    void insert(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator first,
                std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator last);
    friend std::ostream& operator<<(std::ostream&, SpReliableMobileObject2D);
 };


typedef ReliableMobileObject2D* rmobile_object2D_t;

#endif	
