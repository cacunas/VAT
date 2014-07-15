#ifndef RMALTERNATIVE_SOLUTION2D_H
#define RMALTERNATIVE_SOLUTION2D_H

#include "blob.h"
#include "MobileObject2D.h"
#include <iostream>
#include <map>
#include <deque>



class RMAlternativeSolution2D;

typedef QSharedPointer<RMAlternativeSolution2D> SpRMAlternativeSolution2D;
  
struct alternatives2DOrderedByBestProbabilityOperator {
    bool operator()(SpRMAlternativeSolution2D a1, SpRMAlternativeSolution2D a2);
};


class RMAlternativeSolution2D {

    class mobilePossibility {
        public:
            int *blobsCombo;
            double score;

            mobilePossibility(int blobsNumber);
            ~mobilePossibility();
    };

    private:
        std::deque<SpMobileObject2D> solution;
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

        std::map<long int, std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D> > newObjectsList;
        std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator> newAlternatives;

        bool survivedToComparison;
        int numSurvived;

        RMAlternativeSolution2D();
        RMAlternativeSolution2D(SpRMAlternativeSolution2D altSolutionToReplicate);
        ~RMAlternativeSolution2D();

        std::deque<SpMobileObject2D>::iterator begin();
        std::deque<SpMobileObject2D>::iterator end();
        bool empty();
        unsigned int size();
        void clear();
        std::deque<SpMobileObject2D>::iterator erase(std::deque<SpMobileObject2D>::iterator element);
        std::deque<SpMobileObject2D>::iterator erase(std::deque<SpMobileObject2D>::iterator first, std::deque<SpMobileObject2D>::iterator last);

        SpMobileObject2D operator[](int index);

        //Init Functions
        void initUsedBlobsList();
        void initInvolvedBlobs();

        //Inserting Functions
        void insertNewMobileObject(SpMobileObject2D newMobileObject);
        void insert(std::deque<SpMobileObject2D>::iterator pos, std::deque<SpMobileObject2D>::iterator f, std::deque<SpMobileObject2D>::iterator l);

        //Getting Functions
        double getProbability();
        std::deque<SpMobileObject2D> *getSolution();
        //Setting Functions
        void setProbability(double p);
        void setAlternativeProbability();

        friend std::ostream& operator<<(std::ostream&, SpRMAlternativeSolution2D);
};

typedef RMAlternativeSolution2D* rmalternative_solution2D_t;

#endif	
