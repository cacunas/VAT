#include "RMAlternativeSolution2D.h"
#include "ReliabilityTracker.h"


int RMAlternativeSolution2D::m_currentTrackingBlobsNumber = 0;


RMAlternativeSolution2D::RMAlternativeSolution2D():usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), justFirstFrames(true) {
    alternativeNumber = ReliabilityTracker::m_alternativeNumber;
}

RMAlternativeSolution2D::RMAlternativeSolution2D(SpRMAlternativeSolution2D altSolutionToReplicate):usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), survivedToComparison(false), numSurvived(0) {

    std::deque<SpMobileObject2D>::iterator mobiles_it;
    std::deque<SpMobileObject2D> *solution = altSolutionToReplicate->getSolution();
    for(mobiles_it = solution->begin(); mobiles_it != solution->end(); mobiles_it++)
        insertNewMobileObject( (*mobiles_it) );
    alternativeNumber = ReliabilityTracker::m_alternativeNumber;
}

RMAlternativeSolution2D::~RMAlternativeSolution2D() {
    solution.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void RMAlternativeSolution2D::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;

    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void RMAlternativeSolution2D::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

std::deque<SpMobileObject2D>::iterator RMAlternativeSolution2D::begin() {
    return solution.begin();
}

std::deque<SpMobileObject2D>::iterator RMAlternativeSolution2D::end() {
    return solution.end();
}

unsigned int RMAlternativeSolution2D::size() {
    return solution.size();
}

bool RMAlternativeSolution2D::empty() {
    return solution.empty();
}

void RMAlternativeSolution2D::clear() {
    solution.clear();
}

SpMobileObject2D RMAlternativeSolution2D::operator[](int index) {
    return solution[index];
}


//Inserting Functions
void RMAlternativeSolution2D::insertNewMobileObject(SpMobileObject2D newMobileObject) {
    solution.push_front(newMobileObject);
}

void RMAlternativeSolution2D::insert(std::deque<SpMobileObject2D>::iterator pos, std::deque<SpMobileObject2D>::iterator f, std::deque<SpMobileObject2D>::iterator l) {
    solution.insert(pos,f,l);
}


//Getting Functions
double RMAlternativeSolution2D::getProbability() {
    return alternativeProbability;
}

std::deque<SpMobileObject2D> *RMAlternativeSolution2D::getSolution() {
    return &solution;
}

//Setting Functions
void RMAlternativeSolution2D::setProbability(double p) {
    alternativeProbability = p;
}

//Probability of alternative is calculated by weighting mobiles probability by the number of frames of evidence of their existence
void RMAlternativeSolution2D::setAlternativeProbability() {
    std::deque<SpMobileObject2D>::iterator it;
    double p = 0.0;
    int cur_numFrames, sum_numFrames = 0;

    for(it=begin();it!=end();it++) {
        SpMobileObject2D mobile = *it;
        cur_numFrames = mobile->getNumberOfFramesSinceFirstTimeSeen();
        if(cur_numFrames > 1) {
            p += mobile->getGlobalProbability() * cur_numFrames;
            sum_numFrames += cur_numFrames;
        }
    }

    justFirstFrames = false;

    if(sum_numFrames == 0) {
        alternativeProbability = 0.0;
        justFirstFrames = true;
    } else
        alternativeProbability = p/sum_numFrames;
}

std::deque<SpMobileObject2D>::iterator RMAlternativeSolution2D::erase(std::deque<SpMobileObject2D>::iterator element) {
    return solution.erase(element);
}

std::deque<SpMobileObject2D>::iterator RMAlternativeSolution2D::erase(std::deque<SpMobileObject2D>::iterator first, std::deque<SpMobileObject2D>::iterator last) {
    return solution.erase(first, last);
}



std::ostream& operator<<(std::ostream& out, SpRMAlternativeSolution2D as){
    std::deque<SpMobileObject2D> *solution;
    solution = as->getSolution();
    std::deque<SpMobileObject2D>::iterator it;
    int i = 0;
    out << "\tNumber of mobiles: " << solution->size() << std::endl;

    for(it = solution->begin(); it != solution->end(); it++){
        out << "\t\t" << "Mobile Object " << i << " :"<< std::endl << *it << std::endl;
        i++;
    }

    out << "\t\t" << "Alternative Probability: " << as->getProbability() << std::endl;

    if(as->involvedBlobs != NULL) {
        out<< "\tInvolved Blobs:" << std::endl;
        for(i = 0; i < RMAlternativeSolution2D::m_currentTrackingBlobsNumber; i++)
            out << "\t" << as->involvedBlobs[i];
        out << std::endl;
    }

    if(as->usedBlobs != NULL) {
        out << "\tusedBlobs:" << std::endl;
        for(i = 0; i < as->blobsNumber; i++)
            out << "\t"<< as->usedBlobs[i];
        out << std::endl;
    }

    return out;
}

RMAlternativeSolution2D::mobilePossibility::mobilePossibility(int blobsNumber):score(0.0) {
    blobsCombo = new int[blobsNumber];
}

RMAlternativeSolution2D::mobilePossibility::~mobilePossibility() {
    if(blobsCombo)
        delete[] blobsCombo;
}

bool alternatives2DOrderedByBestProbabilityOperator::operator()(SpRMAlternativeSolution2D a1, SpRMAlternativeSolution2D a2) {
    return a1->getProbability() >= a2->getProbability();
}


