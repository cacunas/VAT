#ifndef RMMHYPOTHESISSET_H
#define RMMHYPOTHESISSET_H

#include "src/blob.h"
#include "RMMHypothesis.h"
#include <iostream>
#include <map>
#include <deque>

class RMMHypothesisSet;

typedef QSharedPointer<RMMHypothesisSet> SpRMMHypothesisSet;

class RMMHypothesisSet{
   
  private:
    int bestHypothesisIndex;
    double bestMeanProbability;


  public:
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> hypotheses;
    static int m_currentTrackingBlobsNumber;
    int incompleteHypotheses;
    int numUsed;
    bool toErase;
    RMMHypothesisSet();
    ~RMMHypothesisSet();

    //Blobs used in "follow mobiles" phase.
    bool *usedBlobs;
    bool *involvedBlobs;

    //Init Functions
    void initUsedBlobsList();
    void initInvolvedBlobs();

    //Inserting Functions
    void insertNewHypothesis(SpRMMHypothesis h);

    //Setting Functions
    void setBestHypothesis();

    //Getting Functions
    double getBestProbability();
    int getBestHypothesisIndex();
    SpRMMHypothesis getBestHypothesis();
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>* getHypotheses();
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator begin();
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator end();
    void erase(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator last);
    void erase(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator element);

    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator operator[](int);
    unsigned int size();
    bool empty();
    void clear();
    void insert(SpRMMHypothesis);
    void insert(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator first,
                std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator last);
    friend std::ostream& operator<<(std::ostream&, SpRMMHypothesisSet);
 };


typedef RMMHypothesisSet* rmm_hypothesis_set_t;

#endif	
