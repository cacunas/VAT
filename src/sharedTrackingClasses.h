#ifndef SHAREDTRACKINGCLASSES_H
#define SHAREDTRACKINGCLASSES_H

#include "MobileObject.h"


class bestAlternativesNode;

typedef QSharedPointer<bestAlternativesNode> SpBestAlternativesNode;

class bestAlternativesNode {
  public:
    static int numVariables;
    static int variablesSum;
    static int *variablesNumFrames;

    static std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *newObjectsListEnds;

    double value;
    int mobileIndex;
    int *versionIndex;
    bool added;
    std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *mobileIterators;
    double *variableContribution;

    bestAlternativesNode(double i_value, int i_index);
    bestAlternativesNode(SpBestAlternativesNode toCopy, int indexToMove);
    ~bestAlternativesNode();

    void setNextAlternativeValue(int index);

};

struct orderedByBestAlternativeProbabilityCooperationOperator {
    bool operator()(SpBestAlternativesNode altNode1, SpBestAlternativesNode altNode2);
};


class IdBlobPair {
  public:
    long id;
    Blob *blob;

    IdBlobPair(long i_id, Blob *i_blob);
    ~IdBlobPair();
    bool operator<(IdBlobPair &P);
};

#endif // SHAREDTRACKINGCLASSES_H
