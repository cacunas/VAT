#include "RMMHypothesisSet.h"
#include "RMMHypothesis.h"


int RMMHypothesisSet::m_currentTrackingBlobsNumber = 0;

RMMHypothesisSet::RMMHypothesisSet():bestHypothesisIndex(-1), bestMeanProbability(0.0), toErase(false), usedBlobs(NULL), involvedBlobs(NULL) {}


RMMHypothesisSet::~RMMHypothesisSet() {
    hypotheses.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void RMMHypothesisSet::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void RMMHypothesisSet::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

void RMMHypothesisSet::insertNewHypothesis(SpRMMHypothesis h) {
    hypotheses.insert(h);
}

void RMMHypothesisSet::setBestHypothesis() {
    SpRMMHypothesis first = *begin();
    bestMeanProbability = first->getProbability();
    bestHypothesisIndex = 0;
}

double RMMHypothesisSet::getBestProbability() { return bestMeanProbability; }

int RMMHypothesisSet::getBestHypothesisIndex() { return bestHypothesisIndex; }

SpRMMHypothesis RMMHypothesisSet::getBestHypothesis() { return *begin(); }

std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>* RMMHypothesisSet::getHypotheses() { return &hypotheses; }

std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator RMMHypothesisSet::begin() {
    return hypotheses.begin();
}

std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator RMMHypothesisSet::end() {
    return hypotheses.end();
}

void RMMHypothesisSet::erase(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator first,
                                 std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator last) {
    hypotheses.erase(first, last);
}

void RMMHypothesisSet::erase(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator element) {
    hypotheses.erase(element);
}


std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator RMMHypothesisSet::operator[](int k) {
    int i;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator asIt;
    for(i = 0, asIt = hypotheses.begin(); i < k; i++, asIt++);
    return asIt;
}

unsigned int RMMHypothesisSet::size() {
    return hypotheses.size();
}

bool RMMHypothesisSet::empty() {
    return hypotheses.empty();
}

void RMMHypothesisSet::clear() {
    hypotheses.clear();
}

void RMMHypothesisSet::insert(SpRMMHypothesis h) {
    insertNewHypothesis(h);
}

void RMMHypothesisSet::insert(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator last) {
    hypotheses.insert(first, last);
}


std::ostream& operator<<(std::ostream& out,SpRMMHypothesisSet hs) {
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>* h;
    h = hs->getHypotheses();
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator it;
    int i = 0;

    out << "Number of hypotheses: " << hs->size() << std::endl;
    for(it = h->begin(); it != h->end(); it++) {
        out << "\tHypothesis " << i << std::endl;
        out << "\t\tComing from hypothesis: " << (*it)->hypothesisNumber << std::endl;
        out << (*it)<<std::endl;
        i++;
    }
    if(hs->involvedBlobs != NULL) {
        out << "InvolvedBlobs:" << std::endl;
        for(i = 0; i < RMMHypothesisSet::m_currentTrackingBlobsNumber; i++)
            out << "\t" << hs->involvedBlobs[i];
        out<<std::endl;
    }
    return out;
}

