#include "RMAlternativeSolution.h"
#include "ReliabilityTracker.h"


int RMAlternativeSolution::m_currentTrackingBlobsNumber = 0;


RMAlternativeSolution::RMAlternativeSolution():usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), justFirstFrames(true) {
    alternativeNumber = ReliabilityTracker::m_alternativeNumber;
}

RMAlternativeSolution::RMAlternativeSolution(SpRMAlternativeSolution altSolutionToReplicate):usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), survivedToComparison(false), numSurvived(0) {

    std::deque<SpMobileObject>::iterator mobiles_it;
    std::deque<SpMobileObject> *solution = altSolutionToReplicate->getSolution();
    for(mobiles_it = solution->begin(); mobiles_it != solution->end(); mobiles_it++)
        insertNewMobileObject( (*mobiles_it) );
    alternativeNumber = ReliabilityTracker::m_alternativeNumber;
}

RMAlternativeSolution::~RMAlternativeSolution() {
    solution.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void RMAlternativeSolution::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;

    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void RMAlternativeSolution::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

std::deque<SpMobileObject>::iterator RMAlternativeSolution::begin() {
    return solution.begin();
}

std::deque<SpMobileObject>::iterator RMAlternativeSolution::end() {
    return solution.end();
}

unsigned int RMAlternativeSolution::size() {
    return solution.size();
}

bool RMAlternativeSolution::empty() {
    return solution.empty();
}

void RMAlternativeSolution::clear() {
    solution.clear();
}

SpMobileObject RMAlternativeSolution::operator[](int index) {
    return solution[index];
}


//Inserting Functions
void RMAlternativeSolution::insertNewMobileObject(SpMobileObject newMobileObject) {
    solution.push_front(newMobileObject);
}

void RMAlternativeSolution::insert(std::deque<SpMobileObject>::iterator pos, std::deque<SpMobileObject>::iterator f, std::deque<SpMobileObject>::iterator l) {
    solution.insert(pos,f,l);
}


//Getting Functions
double RMAlternativeSolution::getProbability() {
    return alternativeProbability;
}

std::deque<SpMobileObject> *RMAlternativeSolution::getSolution() {
    return &solution;
}

//Setting Functions
void RMAlternativeSolution::setProbability(double p) {
    alternativeProbability = p;
}

//Probability of alternative is calculated by weighting mobiles probability by the number of frames of evidence of their existence
void RMAlternativeSolution::setAlternativeProbability() {
    std::deque<SpMobileObject>::iterator it;
    double p = 0.0;
    int cur_numFrames, sum_numFrames = 0;

    for(it=begin();it!=end();it++) {
        SpMobileObject mobile = *it;
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

std::deque<SpMobileObject>::iterator RMAlternativeSolution::erase(std::deque<SpMobileObject>::iterator element) {
    return solution.erase(element);
}

std::deque<SpMobileObject>::iterator RMAlternativeSolution::erase(std::deque<SpMobileObject>::iterator first, std::deque<SpMobileObject>::iterator last) {
    return solution.erase(first, last);
}



std::ostream& operator<<(std::ostream& out, SpRMAlternativeSolution as){
    std::deque<SpMobileObject> *solution;
    solution = as->getSolution();
    std::deque<SpMobileObject>::iterator it;
    int i = 0;
    out << "\tNumber of mobiles: " << solution->size() << std::endl;

    for(it = solution->begin(); it != solution->end(); it++){
        out << "\t\t" << "Mobile Object " << i << " :"<< std::endl << *it << std::endl;
        i++;
    }

    out << "\t\t" << "Alternative Probability: " << as->getProbability() << std::endl;

    if(as->involvedBlobs != NULL) {
        out<< "\tInvolved Blobs:" << std::endl;
        for(i = 0; i < RMAlternativeSolution::m_currentTrackingBlobsNumber; i++)
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

mobilePossibility::mobilePossibility(int blobsNumber):score(0.0) {
    blobsCombo = new int[blobsNumber];
}

mobilePossibility::~mobilePossibility() {
    if(blobsCombo)
        delete[] blobsCombo;
}

bool alternativesOrderedByBestProbabilityOperator::operator()(SpRMAlternativeSolution a1, SpRMAlternativeSolution a2) {
    return a1->getProbability() >= a2->getProbability();
}


