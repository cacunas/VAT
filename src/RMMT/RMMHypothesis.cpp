#include "RMMHypothesis.h"
#include "RMMTracker.h"


int RMMHypothesis::m_currentTrackingBlobsNumber = 0;


RMMHypothesis::RMMHypothesis():usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), justFirstFrames(true) {
    hypothesisNumber = RMMTracker::m_hypothesisNumber;
}

RMMHypothesis::RMMHypothesis(SpRMMHypothesis hypothesisToReplicate):usedBlobs(NULL), involvedBlobs(NULL), blobsVector(NULL), blobsNumber(m_currentTrackingBlobsNumber), toEliminate(false), survivedToComparison(false), numSurvived(0) {

    std::deque<SpRMMMobileObject>::iterator mobiles_it;
    std::deque<SpRMMMobileObject> *mobiles = hypothesisToReplicate->getMobiles();
    for(mobiles_it = mobiles->begin(); mobiles_it != mobiles->end(); mobiles_it++)
        insertNewMobileObject( (*mobiles_it) );
    hypothesisNumber = RMMTracker::m_hypothesisNumber;
}

RMMHypothesis::~RMMHypothesis() {
    mobiles.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void RMMHypothesis::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;

    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void RMMHypothesis::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

std::deque<SpRMMMobileObject>::iterator RMMHypothesis::begin() {
    return mobiles.begin();
}

std::deque<SpRMMMobileObject>::iterator RMMHypothesis::end() {
    return mobiles.end();
}

unsigned int RMMHypothesis::size() {
    return mobiles.size();
}

bool RMMHypothesis::empty() {
    return mobiles.empty();
}

void RMMHypothesis::clear() {
    mobiles.clear();
}

SpRMMMobileObject RMMHypothesis::operator[](int index) {
    return mobiles[index];
}


//Inserting Functions
void RMMHypothesis::insertNewMobileObject(SpRMMMobileObject newRMMMobileObject) {
    mobiles.push_front(newRMMMobileObject);
}

void RMMHypothesis::insert(std::deque<SpRMMMobileObject>::iterator pos, std::deque<SpRMMMobileObject>::iterator f, std::deque<SpRMMMobileObject>::iterator l) {
    mobiles.insert(pos,f,l);
}


//Getting Functions
double RMMHypothesis::getProbability() {
    return hypothesisProbability;
}

std::deque<SpRMMMobileObject> *RMMHypothesis::getMobiles() {
    return &mobiles;
}

//Setting Functions
void RMMHypothesis::setProbability(double p) {
    hypothesisProbability = p;
}

//Probability of alternative is calculated by weighting mobiles probability by the number of frames of evidence of their existence
void RMMHypothesis::setHypothesisProbability() {
    std::deque<SpRMMMobileObject>::iterator it;
    double p = 0.0;
    int cur_numFrames, sum_numFrames = 0;

    for(it=begin();it!=end();it++) {
        SpRMMMobileObject mobile = *it;
        cur_numFrames = mobile->getNumberOfFramesSinceFirstTimeSeen();
        if(cur_numFrames > 1) {
            p += mobile->getGlobalProbability() * cur_numFrames;
            sum_numFrames += cur_numFrames;
        }
    }

    justFirstFrames = false;

    if(sum_numFrames == 0) {
        hypothesisProbability = 0.0;
        justFirstFrames = true;
    } else
        hypothesisProbability = p/sum_numFrames;
}

std::deque<SpRMMMobileObject>::iterator RMMHypothesis::erase(std::deque<SpRMMMobileObject>::iterator element) {
    return mobiles.erase(element);
}

std::deque<SpRMMMobileObject>::iterator RMMHypothesis::erase(std::deque<SpRMMMobileObject>::iterator first, std::deque<SpRMMMobileObject>::iterator last) {
    return mobiles.erase(first, last);
}



std::ostream& operator<<(std::ostream& out, SpRMMHypothesis hyp){
    std::deque<SpRMMMobileObject> *mobiles;
    mobiles = hyp->getMobiles();
    std::deque<SpRMMMobileObject>::iterator it;
    int i = 0;
    out << "\tNumber of mobiles: " << mobiles->size() << std::endl;

    for(it = mobiles->begin(); it != mobiles->end(); it++){
        out << "\t\t" << "Mobile Object " << i << " :"<< std::endl << *it << std::endl;
        i++;
    }

    out << "\t\t" << "Alternative Probability: " << hyp->getProbability() << std::endl;

    if(hyp->involvedBlobs != NULL) {
        out<< "\tInvolved Blobs:" << std::endl;
        for(i = 0; i < RMMHypothesis::m_currentTrackingBlobsNumber; i++)
            out << "\t" << hyp->involvedBlobs[i];
        out << std::endl;
    }

    if(hyp->usedBlobs != NULL) {
        out << "\tusedBlobs:" << std::endl;
        for(i = 0; i < hyp->blobsNumber; i++)
            out << "\t"<< hyp->usedBlobs[i];
        out << std::endl;
    }

    return out;
}


bool hypothesesOrderedByBestProbabilityOperator::operator()(SpRMMHypothesis h1, SpRMMHypothesis h2) {
    return h1->getProbability() >= h2->getProbability();
}


