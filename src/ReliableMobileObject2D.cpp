#include "ReliableMobileObject2D.h"


int ReliableMobileObject2D::m_currentTrackingBlobsNumber = 0;

ReliableMobileObject2D::ReliableMobileObject2D():bestSolutionIndex(-1), bestMeanProbability(0.0), toErase(false), usedBlobs(NULL), involvedBlobs(NULL) {}


ReliableMobileObject2D::~ReliableMobileObject2D() {
    alternativeSolutions.clear();
    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
}

void ReliableMobileObject2D::initUsedBlobsList() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void ReliableMobileObject2D::initInvolvedBlobs() {
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
}

void ReliableMobileObject2D::insertNewSolution(SpRMAlternativeSolution2D solution) {
    alternativeSolutions.insert(solution);
}

void ReliableMobileObject2D::setBestSolution() {
    SpRMAlternativeSolution2D first = *begin();
    bestMeanProbability = first->getProbability();
    bestSolutionIndex = 0;
}

double ReliableMobileObject2D::getBestProbability() { return bestMeanProbability; }

int ReliableMobileObject2D::getBestSolutionIndex() { return bestSolutionIndex; }

SpRMAlternativeSolution2D ReliableMobileObject2D::getBestSolution() { return *begin(); }

std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>* ReliableMobileObject2D::getAlternativeSolutions() { return &alternativeSolutions; }

std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator ReliableMobileObject2D::begin() {
    return alternativeSolutions.begin();
}

std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator ReliableMobileObject2D::end() {
    return alternativeSolutions.end();
}

void ReliableMobileObject2D::erase(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator first,
                                 std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator last) {
    alternativeSolutions.erase(first, last);
}

void ReliableMobileObject2D::erase(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator element) {
    alternativeSolutions.erase(element);
}


std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator ReliableMobileObject2D::operator[](int k) {
    int i;
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator asIt;
    for(i = 0, asIt = alternativeSolutions.begin(); i < k; i++, asIt++);
    return asIt;
}

unsigned int ReliableMobileObject2D::size() {
    return alternativeSolutions.size();
}

bool ReliableMobileObject2D::empty() {
    return alternativeSolutions.empty();
}

void ReliableMobileObject2D::clear() {
    alternativeSolutions.clear();
}

void ReliableMobileObject2D::insert(SpRMAlternativeSolution2D alternative) {
    insertNewSolution(alternative);
}

void ReliableMobileObject2D::insert(std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator first, std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator last) {
    alternativeSolutions.insert(first, last);
}


std::ostream& operator<<(std::ostream& out,SpReliableMobileObject2D rmo) {
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>* as;
    as = rmo->getAlternativeSolutions();
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator>::iterator it;
    int i = 0;

    out << "Number of alternative solutions: " << rmo->size() << std::endl;
    for(it = as->begin(); it != as->end(); it++) {
        out << "\tAlternative solution " << i << std::endl;
        out << "\t\tComing from alternative: " << (*it)->alternativeNumber << std::endl;
        out << (*it)<<std::endl;
        i++;
    }
    /*std::cerr << "Number of alternative solutions: " << rmo->size() << std::endl;
    for(it = as->begin(); it != as->end(); it++) {
        std::cerr << "\tAlternative solution " << i << std::endl;
        std::cerr << "\t\tComing from alternative: " << (*it)->alternativeNumber << std::endl;
        std::cerr << (*it)<<std::endl;
        i++;
    }*/
    if(rmo->involvedBlobs != NULL) {
        out << "InvolvedBlobs:" << std::endl;
        for(i = 0; i < ReliableMobileObject2D::m_currentTrackingBlobsNumber; i++)
            out << "\t" << rmo->involvedBlobs[i];
        out<<std::endl;
    }
    return out;
}

