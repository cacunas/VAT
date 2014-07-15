#ifndef SHAREDTRACKINGCLASSES2D_H
#define SHAREDTRACKINGCLASSES2D_H

#include "MobileObject2D.h"


class bestAlternativesNode2D;

typedef QSharedPointer<bestAlternativesNode2D> SpBestAlternativesNode2D;

class bestAlternativesNode2D {
  public:
    static int numVariables;
    static int variablesSum;
    static int *variablesNumFrames;

    static std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator *newObjectsListEnds;

    double value;
    int mobileIndex;
    int *versionIndex;
    bool added;
    std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator *mobileIterators;
    double *variableContribution;

    bestAlternativesNode2D(double i_value, int i_index);
    bestAlternativesNode2D(SpBestAlternativesNode2D toCopy, int indexToMove);
    ~bestAlternativesNode2D();

    void setNextAlternativeValue(int index);

};

struct orderedByBestAlternativeProbabilityCooperationOperator2D {
    bool operator()(SpBestAlternativesNode2D altNode1, SpBestAlternativesNode2D altNode2);
};


class IdBlobPair2D {
  public:
    long id;
    Blob *blob;

    IdBlobPair2D(long i_id, Blob *i_blob);
    ~IdBlobPair2D();
    bool operator<(IdBlobPair2D &P);
};

#endif // SHAREDTRACKINGCLASSES2D_H
