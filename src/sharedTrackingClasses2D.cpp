#include "sharedTrackingClasses2D.h"

int bestAlternativesNode2D::numVariables = 0;
std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator *bestAlternativesNode2D::newObjectsListEnds = NULL;
int bestAlternativesNode2D::variablesSum = 0;
int *bestAlternativesNode2D::variablesNumFrames = NULL;

IdBlobPair2D::IdBlobPair2D(long int i_id, Blob *i_blob): id(i_id), blob(i_blob){}

IdBlobPair2D::~IdBlobPair2D() {}

bool IdBlobPair2D::operator<(IdBlobPair2D &P) {
    return this->id < P.id;
}

bestAlternativesNode2D::bestAlternativesNode2D(double i_value, int i_index): value(i_value), mobileIndex(i_index), versionIndex(NULL), added(false), mobileIterators(NULL) {
    mobileIterators = new std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memset(versionIndex, 0, sizeof(int) * numVariables);
}

bestAlternativesNode2D::bestAlternativesNode2D(SpBestAlternativesNode2D toCopy, int indexToMove): value(toCopy->value), mobileIndex(indexToMove), versionIndex(NULL), added(false), mobileIterators(NULL) {

    mobileIterators = new std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memcpy(mobileIterators, toCopy->mobileIterators, sizeof(std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator) * numVariables );
    memcpy(variableContribution, toCopy->variableContribution, sizeof(double) * numVariables );
    memcpy(versionIndex, toCopy->versionIndex, sizeof(int) * numVariables );

    mobileIndex = indexToMove;
    setNextAlternativeValue(indexToMove);

}

bestAlternativesNode2D::~bestAlternativesNode2D() {
    delete[] mobileIterators;
    delete[] variableContribution;
    delete[] versionIndex;
}


void bestAlternativesNode2D::setNextAlternativeValue(int index) {
    if(mobileIterators[index] == newObjectsListEnds[index]) {
        value = -1;
        return;
    }
    mobileIterators[index]++;
    if(mobileIterators[index] == newObjectsListEnds[index]) {
        value = -1;
        return;
    }

    versionIndex[index]++;

    value -= variableContribution[index];
    value += variableContribution[index] = variablesNumFrames[index] * (*mobileIterators[index])->getGlobalProbability() / (double) variablesSum;

}

bool orderedByBestAlternativeProbabilityCooperationOperator2D::operator()(SpBestAlternativesNode2D altNode1, SpBestAlternativesNode2D altNode2) {
    return altNode1->value >= altNode2->value;
}
