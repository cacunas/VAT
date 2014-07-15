#include "ReliableMobileObject.h"


int ReliableMobileObject::m_currentTrackingBlobsNumber = 0;

ReliableMobileObject::ReliableMobileObject():bestSolutionIndex(-1), bestMeanProbability(0.0), toErase(false), usedBlobs(NULL), involvedBlobs(NULL) {}


ReliableMobileObject::~ReliableMobileObject() {
    alternativeSolutions.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void ReliableMobileObject::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void ReliableMobileObject::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

void ReliableMobileObject::insertNewSolution(SpRMAlternativeSolution solution) {
    alternativeSolutions.insert(solution);
}

void ReliableMobileObject::setBestSolution() {
    SpRMAlternativeSolution first = *begin();
    bestMeanProbability = first->getProbability();
    bestSolutionIndex = 0;
}

double ReliableMobileObject::getBestProbability() { return bestMeanProbability; }

int ReliableMobileObject::getBestSolutionIndex() { return bestSolutionIndex; }

SpRMAlternativeSolution ReliableMobileObject::getBestSolution() { return *begin(); }

std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>* ReliableMobileObject::getAlternativeSolutions() { return &alternativeSolutions; }

std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ReliableMobileObject::begin() {
    return alternativeSolutions.begin();
}

std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ReliableMobileObject::end() {
    return alternativeSolutions.end();
}

void ReliableMobileObject::erase(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator first,
                                 std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator last) {
    alternativeSolutions.erase(first, last);
}

void ReliableMobileObject::erase(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator element) {
    alternativeSolutions.erase(element);
}


std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ReliableMobileObject::operator[](int k) {
    int i;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator asIt;
    for(i = 0, asIt = alternativeSolutions.begin(); i < k; i++, asIt++);
    return asIt;
}

unsigned int ReliableMobileObject::size() {
    return alternativeSolutions.size();
}

bool ReliableMobileObject::empty() {
    return alternativeSolutions.empty();
}

void ReliableMobileObject::clear() {
    alternativeSolutions.clear();
}

void ReliableMobileObject::insert(SpRMAlternativeSolution alternative) {
    insertNewSolution(alternative);
}

void ReliableMobileObject::insert(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator last) {
    alternativeSolutions.insert(first, last);
}


std::ostream& operator<<(std::ostream& out,SpReliableMobileObject rmo) {
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>* as;
    as = rmo->getAlternativeSolutions();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator it;
    int i = 0;

    out << "Number of alternative solutions: " << rmo->size() << std::endl;
    for(it = as->begin(); it != as->end(); it++) {
        out << "\tAlternative solution " << i << std::endl;
        out << "\t\tComing from alternative: " << (*it)->alternativeNumber << std::endl;
        out << (*it)<<std::endl;
        i++;
    }
    if(rmo->involvedBlobs != NULL) {
        out << "InvolvedBlobs:" << std::endl;
        for(i = 0; i < ReliableMobileObject::m_currentTrackingBlobsNumber; i++)
            out << "\t" << rmo->involvedBlobs[i];
        out<<std::endl;
    }
    return out;
}

