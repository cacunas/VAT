#include "sharedTrackingClasses.h"

int bestAlternativesNode::numVariables = 0;
std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *bestAlternativesNode::newObjectsListEnds = NULL;
int bestAlternativesNode::variablesSum = 0;
int *bestAlternativesNode::variablesNumFrames = NULL;

IdBlobPair::IdBlobPair(long int i_id, Blob *i_blob): id(i_id), blob(i_blob){}

IdBlobPair::~IdBlobPair() {}

bool IdBlobPair::operator<(IdBlobPair &P) {
    return this->id < P.id;
}

bestAlternativesNode::bestAlternativesNode(double i_value, int i_index): value(i_value), mobileIndex(i_index), versionIndex(NULL), added(false), mobileIterators(NULL) {
    mobileIterators = new std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memset(versionIndex, 0, sizeof(int) * numVariables);
}

bestAlternativesNode::bestAlternativesNode(SpBestAlternativesNode toCopy, int indexToMove): value(toCopy->value), mobileIndex(indexToMove), versionIndex(NULL), added(false), mobileIterators(NULL) {

    mobileIterators = new std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memcpy(mobileIterators, toCopy->mobileIterators, sizeof(std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator) * numVariables );
    memcpy(variableContribution, toCopy->variableContribution, sizeof(double) * numVariables );
    memcpy(versionIndex, toCopy->versionIndex, sizeof(int) * numVariables );

    mobileIndex = indexToMove;
    setNextAlternativeValue(indexToMove);

}

bestAlternativesNode::~bestAlternativesNode() {
    delete[] mobileIterators;
    delete[] variableContribution;
    delete[] versionIndex;
}


void bestAlternativesNode::setNextAlternativeValue(int index) {
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

bool orderedByBestAlternativeProbabilityCooperationOperator::operator()(SpBestAlternativesNode altNode1, SpBestAlternativesNode altNode2) {
    return altNode1->value >= altNode2->value;
}
