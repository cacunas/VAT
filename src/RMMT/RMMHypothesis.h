#ifndef RMMHYPOTHESIS_H
#define RMMHYPOTHESIS_H

#include "src/blob.h"
#include "RMMMobileObject.h"
#include <iostream>
#include <map>
#include <deque>

class RMMHypothesis;

typedef QSharedPointer<RMMHypothesis> SpRMMHypothesis;
  
struct hypothesesOrderedByBestProbabilityOperator {
    bool operator()(SpRMMHypothesis a1, SpRMMHypothesis a2);
};

class RMMHypothesis {
    public:
        std::deque<SpRMMMobileObject> mobiles;
        double hypothesisProbability;
        static int m_currentTrackingBlobsNumber;

        //Blobs used in "follow mobiles" phase.
        bool *usedBlobs;
        bool *involvedBlobs;
        Blob **blobsVector;
        int blobsNumber;
        int numUsed;
        int numCurrentlyLost;
        bool incompleteHypothesis;
        bool toEliminate;
        bool comparedHypothesis;
        bool justFirstFrames;

        int hypothesisNumber;

        std::map<long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> > newObjectsList;
        std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> newHypotheses;

        bool survivedToComparison;
        int numSurvived;

        RMMHypothesis();
        RMMHypothesis(SpRMMHypothesis hypothesisToReplicate);
        ~RMMHypothesis();

        std::deque<SpRMMMobileObject>::iterator begin();
        std::deque<SpRMMMobileObject>::iterator end();
        bool empty();
        unsigned int size();
        void clear();
        std::deque<SpRMMMobileObject>::iterator erase(std::deque<SpRMMMobileObject>::iterator element);
        std::deque<SpRMMMobileObject>::iterator erase(std::deque<SpRMMMobileObject>::iterator first, std::deque<SpRMMMobileObject>::iterator last);

        SpRMMMobileObject operator[](int index);

        //Init Functions
        void initUsedBlobsList();
        void initInvolvedBlobs();

        //Inserting Functions
        void insertNewMobileObject(SpRMMMobileObject newMobileObject);
        void insert(std::deque<SpRMMMobileObject>::iterator pos, std::deque<SpRMMMobileObject>::iterator f, std::deque<SpRMMMobileObject>::iterator l);

        //Getting Functions
        double getProbability();
        std::deque<SpRMMMobileObject> *getMobiles();
        //Setting Functions
        void setProbability(double p);
        void setHypothesisProbability();

        friend std::ostream& operator<<(std::ostream&, SpRMMHypothesis);
};

typedef RMMHypothesis* rmm_hypothesis_t;

#endif	
