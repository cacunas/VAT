#ifndef RMALTERNATIVE_SOLUTION_H
#define RMALTERNATIVE_SOLUTION_H

#include "blob.h"
#include "MobileObject.h"
#include <iostream>
#include <map>
#include <deque>


class mobilePossibility {
    public:
        int *blobsCombo;
        double score;

        mobilePossibility(int blobsNumber);
        ~mobilePossibility();
};

class RMAlternativeSolution;

typedef QSharedPointer<RMAlternativeSolution> SpRMAlternativeSolution;
  
struct alternativesOrderedByBestProbabilityOperator {
    bool operator()(SpRMAlternativeSolution a1, SpRMAlternativeSolution a2);
};


class RMAlternativeSolution {
    
    private:
        std::deque<SpMobileObject> solution;
        double alternativeProbability;


    public:
        static int m_currentTrackingBlobsNumber;

        //Blobs used in "follow mobiles" phase.
        bool *usedBlobs;
        bool *involvedBlobs;
        Blob **blobsVector;
        int blobsNumber;
        int numUsed;
        int numCurrentlyLost;
        bool incompleteAlternative;
        bool toEliminate;
        bool comparedAlternative;
        bool justFirstFrames;

        int alternativeNumber;

        std::map<long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> > newObjectsList;
        std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> newAlternatives;

        bool survivedToComparison;
        int numSurvived;

        RMAlternativeSolution();
        RMAlternativeSolution(SpRMAlternativeSolution altSolutionToReplicate);
        ~RMAlternativeSolution();

        std::deque<SpMobileObject>::iterator begin();
        std::deque<SpMobileObject>::iterator end();
        bool empty();
        unsigned int size();
        void clear();
        std::deque<SpMobileObject>::iterator erase(std::deque<SpMobileObject>::iterator element);
        std::deque<SpMobileObject>::iterator erase(std::deque<SpMobileObject>::iterator first, std::deque<SpMobileObject>::iterator last);

        SpMobileObject operator[](int index);

        //Init Functions
        void initUsedBlobsList();
        void initInvolvedBlobs();

        //Inserting Functions
        void insertNewMobileObject(SpMobileObject newMobileObject);
        void insert(std::deque<SpMobileObject>::iterator pos, std::deque<SpMobileObject>::iterator f, std::deque<SpMobileObject>::iterator l);

        //Getting Functions
        double getProbability();
        std::deque<SpMobileObject> *getSolution();
        //Setting Functions
        void setProbability(double p);
        void setAlternativeProbability();

        friend std::ostream& operator<<(std::ostream&, SpRMAlternativeSolution);
};

typedef RMAlternativeSolution* rmalternative_solution_t;

#endif	
