#ifndef __C_RELIABILITY_MERGE_H__
#define __C_RELIABILITY_MERGE_H__

#include "Datapool.h"
#include "ReliabilityClassification.h"
#include "MODULES/segmentationmodule.h"
#include <iostream>
#include <limits.h>
#include "tree.hh"

class mergeType {
    public:
        mergeType();
        ~mergeType();
        double score;
        ObjectType type;
};

class mergeData {
    public:
        mergeData();
        ~mergeData();
        bool overlaping;
        std::map<ObjectType, double> mergeTypeMap;
        Blob *mergedBlob;
};

class MergedElement {
    protected:
        int blobIndex;
        Blob *mergeResult;

    public:
        bool *usableList;
        bool *usedElements;
        int level;
        int *mergeSequence;
        bool pathUnknown;

        MergedElement(int index, int i_level, Blob *mergedBlob, int blobsNumber);
        ~MergedElement();

        Blob *getMergedBlob();
        int getBlobIndex();
        void initMergeData(bool **initialMergeMap, int *previousMergeSequence, int mergeableNumber, int *reallyMergeableBlobs, bool *forbiddenElements);
};

class AlternativeElement;

typedef QSharedPointer<AlternativeElement> SpAlternativeElement;

typedef QSharedPointer<MergedElement> SMergedElement;

class AlternativeElement {
    public:
        int blob_id;
        Blob *mergedBlob;
        bool classified;
        std::map<int, SpAlternativeElement> mergedElements;
        AlternativeElement();
        AlternativeElement(int i_blob_id);
        ~AlternativeElement();
};

class ReliabilityMerge {
    public:
        enum mergeParameterSet {
            MergeInTracking,
            PreMergeInTracking
        };

        bool initialized;

        //Parameters
        double m_mergeThres;
        double m_mergeDistMax;
        double m_overlapingRatio;
        classificationCriteria m_mergingCriteria;

        //... for premerge
        bool m_eliminateOverlapedClassifiedBlobs;
        double m_eliminateOverlapedClassifiedBlobsOverlapingRatio;
        double m_eliminateOverlapedClassifiedBlobsProbability;
        double m_eliminateOverlapedClassifiedBlobsReliability;

        //Variables
        int *mergeList;
        int elementsToMerge;
        std::map<int, SpAlternativeElement> definedMerges;
        int shortestMergePath;
        int *shortestSequence;
        Blob *shortestSequenceMerge;

        std::map<ObjectType, SpModelInterface> m_mapModels;
        std::map<Blob *, std::map<Blob *, mergeData> > m_mapCorrespondences;

        //  bool m_reducedUnknownBlobsMerge;
        Blob *m_mergeBlob;
        Datapool *m_data;
        SceneModel *m_context;
        QImage *m_segmentation;
        Blob *m_lastMergedBlob;
        ReliabilityClassification *m_rc;

        ReliabilityMerge();
        ReliabilityMerge(Datapool *i_data);
        ~ReliabilityMerge();

        bool prepareRun();
        bool init();

        bool setParameters(QDomNode& config);
        bool setParameters(QDomNode& config, mergeParameterSet i_mergeParameterSet);

	bool defaultParameters(mergeParameterSet i_mergeParameterSet);
	
        bool canBeMerged(Blob *i_blob1, Blob *i_blob2, mergeType *o_mergeType);
        bool getNextCorrespondence(Blob **i_pBlob1, Blob **i_pBlob2);
        bool isAValidCorrespondence(Blob *i_blob1, Blob *i_blob2);

        bool blobOnTopAndNotOnAOIGround(Blob *i_blob1, Blob *i_blob2);
        bool distanceCriteria(Blob *i_blob1, Blob *i_blob2);
        double widthIntersectionRatio(Blob *i_blob1, Blob *i_blob2);
        double heightIntersectionRatio(Blob *i_blob1, Blob *i_blob2);

        void addModel(ObjectType i_blobType, SpModelInterface i_spModel);
        void preMerge(bool **initialMergeMap, std::vector<Blob>& blobsList);
        Blob *getMergedBlob(int *mergeSequence, int sequenceLength, std::vector<Blob>& blobsVector, int blobsNumber, bool classify);

        void computeModelScoresForMerge(std::map<ObjectType, double>& mergeMap, Blob *i_blob1, Blob * i_blob2);
        mergeType computeMergeScoreForType(Blob *i_blob1, Blob *i_blob2, ObjectType blobType);
        void constructCorrespondenceTable();

        //Special Functions for CReliabilityTracker
        void setInitialCorrespondences(bool **initialMergeMap, int blobsNumber, Blob **blobsVector);
        bool setMergesForUsableBlobs(tree<SMergedElement> &mergeTree, tree<SMergedElement>::iterator iter, int level, int parent, bool **initialMergeMap,
                                     int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber);
        int checkValidityOfSubtreesForSequence(int *sequence, int length, bool **initialMergeMap, int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber);
        bool subTreesValidator(tree<SMergedElement> &mergeSubTree, tree<SMergedElement>::iterator iter, int level, int parent, bool **initialMergeMap,
                               int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber, bool *forbiddenElements);

        Blob *getMergedBlob(int *mergeSequence, int sequenceLength, Blob **blobsVector, int blobsNumber, bool classify);
        void clearDefinedMerges();
        void clearDefinedMergesFromStartingKey(int value);

        Blob *justMerge2D(int *mergeSequence, int sequenceLength, Blob **blobsVector);

        bool checkValidityOfPathsForSequence(int *sequence, int length, tree<SMergedElement> &mergeTree, Blob *mergedBlob, int mergeableNumber);
        bool pathValidator(bool *sequenceElements, int length, int num_valid, tree<SMergedElement> &mergeTree, tree<SMergedElement>::iterator iter, Blob *mergedBlob, int mergeableNumber);

        void mergeBlobsInFirstOne(Blob *, Blob *);
        Blob **reduceOverlapedBlobs(Blob **blobsVector, int &blobsNumber, std::vector<Blob>& blobsList);
        int reduceOverlapedBlobsMarkingElimination(bool *eliminationVector, bool *blobWithMergeResult, Blob **blobsVector, int blobsNumber, bool *pertinentElements);

        void setGroup(int groupsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector);
        int setGroups(int groupsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector);

        void orderAscending(int *array, int top, int bottom);
        int getMiddle(int *array, int top, int bottom);

        void mergeBlobs(Blob *, Blob *);
        void updateCorrespondenceTable(Blob *i_blob);
        void removeCorrespondencesWithBlob(Blob *i_blob);
        void removeCorrespondence(Blob *i_blob1, Blob *i_blob2);
        void updateClassificationTable(Blob *i_blob1, Blob *i_blob2);
        bool canBelongToBlob(Blob *blob1, Blob *blob2);
};


#endif 
