#ifndef RELIABILITYTRACKER2D_H
#define RELIABILITYTRACKER2D_H

#include <QDomNode>
#include "Datapool.h"
#include "ReliabilityMerge2D.h"
#include "ReliableMobileObjectList2D.h"
#include "blob.h"
#include "TimeStamp.h"
#include "calibration.h"
#include "sharedTrackingClasses2D.h"

class ReliabilityTracker2D {

public:
    Datapool *m_data;
    ReliabilityMerge2D *m_RMerge;
    ReliabilityMerge2D *m_PreMerge;
    QImage *m_pSegmentation;

  std::deque<IdBlobPair2D> biggestBlobForNewMobile;
  std::set<long int> usedIdsByAlternative;
  std::map< long int, std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D> > mobileAlternativesMap;
  std::map< int, std::deque<long int> > mobileIdsUsingABlob;

  /*!
   * \var CReliableMobileObjectsList rMobilesList
   * \brief pointer to the tracking results
   */
  ReliableMobileObjectList2D rMobilesList;

  /*!
   * \var CReliableMobileObjectsList rMobilesList
   * \brief pointer to the tracking results
   */
  std::map<long int, SpMobileObject2D> mobilesOutList;

    bool parametersInitialised;
    static int m_alternativeNumber;
    static bool m_newOuts;
    int lastMilliSecondsDifference;

    //PARAMETERS:
    bool activatePreMerge;
    int m_BlobBufferSize;

    int m_maximumRetainedAlternativeSolutions;
    int m_maximumGeneratedAlternativesPerMobile;
    int maximumAlternativeMobilePaths;
    QString eliminationStrategy;

    double SpatialCoherenceReliabilityThreshold;
    double SpatialCoherenceProbabilityThreshold;
    double alternativeSolutionsProbabilityThreshold;

    double IgnoreByDimensionalCoherenceThreshold;
    double IgnoreByVelocityCoherenceThreshold;
    double ImportanceRateForBestMobileAlternativeProbability;

    double m_mobile2DCoverageRateToConsiderEqual;
    static double m_highVisualSupportThreshold;
    double m_lowVisualSupportThreshold;
    double m_blobCompletellySupportedThreshold;

    bool m_internalOutputActivated;
    bool m_reducedOutputActivated;

    static int m_meanMillisecondsDifferenceBetweenFrames;

    double m_acceptedPixelError;
    double m_max2DSpeed;

    bool speed_set, object_length;

    //VARIABLES:
    int *g_intersectionAreas;

    Blob **blobsVector;
    bool *usedBlobs;
    int *involvedRMobilesCounter;
    bool **initialMergeMap;
    int *initialGroups;
    bool firstFrame;
    bool initialPreparation;

    bool *checkedMobilePairValidity, *validMobilePair;

    int blobsNumber;

    unsigned long mobile_id_counter;
    unsigned long rmobile_id_counter;
    int currentFrameNumber;
    unsigned long currentTimeMilliSeconds;
    TimeStamp lastTimeStamp;

    //Data used to generate the alternative paths for a mobile
    int g_inserted_for_alternative, g_NumCurrentlyLost;
    SpRMAlternativeSolution2D g_baseAlternative;

    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator> g_newAlternatives;
    std::set<SpRMAlternativeSolution2D, alternatives2DOrderedByBestProbabilityOperator> g_completeAlternatives;
    std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D> g_newMobiles;
    std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D> g_newSpecialMobiles;
    std::deque<Blob *> g_includedBlobsInNewMobiles;
    std::deque<Blob *> g_allocatedBlobs;
    double g_bestGlobalP;
    int g_numberOfNewMobiles, g_NumLocallyAddedAlternatives, *g_numberOfNewMobileVersions, *g_mobileVersionIndex, g_totalNumberOfMobileVersions;
    std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D>::iterator *g_mobileIterators, *g_setBegins;
    std::set<SpBestAlternativesNode2D, orderedByBestAlternativeProbabilityCooperationOperator2D> g_leaves;

    std::map<long int, std::set<SpMobileObject2D, orderedByBestCoherenceOperator2D> > *g_newObjectsList;
    bool *g_acceptable2DCoherenceForMobile;
    double *g_variableContribution;

    double *distanceBetweenBlobs;

    ReliabilityTracker2D(Datapool *i_data);
    ~ReliabilityTracker2D();

    bool setParameters(QDomNode& config);
    static int getBetaDirection(SceneModel *, QImage *);

    /*!
    * \fn Inline void computeInclusiveProbabilities()
    * \brief compute inclusive probabilities for a random linear elimination strategy
    */
    void computeInclusiveProbabilities(std::set<SpRMAlternativeSolution2D,
                                       alternatives2DOrderedByBestProbabilityOperator>* alternativeSolutions,
                                       int k, double sum, bool *mark, double *piT);

    /*!
    * \fn initTracker()
    * \brief constructor of a ReliabilityTracker
    */
    bool init();

    bool initialPrepareRun();

    bool prepareRun();

    void initStaticMobileObject();

    /*!
    * \fn void getMostLikelyMobileObjects(std::deque<SpCMobileObject> *mobileObjectsOutput)
    * \brief clears the current mobiles list and gives the processed list of most likely mobiles
    * \return nothing, it modifies the class pointed by the given parameter
    */
    void getMostLikelyMobileObjects(std::deque<SpMobileObject2D> &mobileObjectsOutput);

    void preMerge(std::vector<Blob>&);

    void preSplit(std::vector<Blob>&);

    void update();

    void eliminateUnlikelyMobiles();

    /*!
    * \fn void run(Blob *new_blobs)
    * \brief reliability tracker process main call
    * \param new_blobs the new detected and classified blobs (time t)
    */
    void run(std::vector<Blob>&);

    void followExistingMobiles();

    void setInvolvedBlobs();

    void createMobilePossibilities();

    void mergeInvolvedRMobiles();

    void mergeReliableMobiles(SpReliableMobileObject2D firstRMobile, SpReliableMobileObject2D secondRMobile);

    void determineInvolvedBlobsForMobile(SpMobileObject2D mobile);

    void insertNewMobiles(SpRMAlternativeSolution2D asolution, SpReliableMobileObject2D rmobile);

    void setBlobsVector(std::vector<Blob>&);

    void freeBlobsVector();

    void presetBlobsVectorAndInitialMergeTable(std::vector<Blob>&);

    void freeBlobsVectorAndInitialMergeTable();

    SpMobileObject2D setNotVisibleBlobMobile(SpMobileObject2D currentMobile, DetectionProblemType dp_type);

    int setGroups(int elementsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector);

    void setGroup(int elementsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector);

    void generateNewMobiles(int blobNumberToAnalyze, int *blobsToAnalyze, int groupsNumber, int *blobGroupVector);

    void generateAlternativesForMobile(int *blobsToMerge, int blobsToMergeNumber);

    void recursiveComboGenerator(int position, int value, int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber);

    bool validBlobMergeConfiguration(int mergeGroupNumber, int mergeLength, int blobsToMergeNumber, int *blobsToMerge, int *alternativesCombo);

    void processMergeVector(int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber);

    void checkConnectivity(bool groupVector[], int elementsNumber, int referencePoint, int *elementsToAnalyzeVector);

    void insertNewMergeSolution(Blob **mergedBlobs, int finalBlobsNumber, bool *usedBlobsMatrix);

    SpMobileObject2D getNewMobileFromBlob(Blob *blob);

    void setCurrentTimeAndFrame();

    void getMergeConnections(bool *listToUpdate, int startingBlobIndex);

    void getRecursiveMergeConnections(bool *mergeList, int blobIndex);

    Blob *generateMostLikelyAlternativeForMobile(SpMobileObject2D mobile);

    //Quicksort
    void orderAscending(int *array, int top, int bottom);
    int getMiddle(int *array, int top, int bottom);

    SpMobileObject2D setNotVisibleBlobMobile(DetectionProblemType dp_type, SpMobileObject2D currentObject);

    void filterUnseenMobiles();

    void filterRepeatedAlternatives(SpReliableMobileObject2D rmobile);

    void separateReliableSolutions();

    double getMaxFocalDistanceToImageCorner();

    double getObjectSizeForMaxReliability(double w, double l, double h);

    double getObjectDistanceForMaxReliability();

    void getNearest2DBlobPointToFocalPoint(Blob *blob, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, double xCenter, double yCenter, double W, double H, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, Rectangle<int> *rect, double *x, double *y);

    void getHighest3DVariations(double x, double y, double X, double Y, double dX, double dY, double *dx, double *dy);

    void determineInvolvedBlobsForMobileObject(SpMobileObject2D mobile);

    bool combinationAlreadyIncluded(Blob **blobsForAlternative, int blobsNumberForAlternative);

    int getMobileId(Blob *blob);

    bool notUsedId(long int id);

    void insertInMobileAlternativesMap(SpMobileObject2D mobile, long int id);

    bool incoherentMobile(SpMobileObject2D mobile);

    void generateAlternativesForMobilePath(int length, int position, int value, int *alternativesCombo,
                                         int blobsToMergeNumber, int *blobsToMerge, SpMobileObject2D mobile);

    void generateMobilePath(SpMobileObject2D mobile);

    bool mobilesCombinationIsValid();


    bool mobilePairValidityChecked(int index1, int index2);

    bool mobilePairIsValid(int index1, int index2);

    bool checkMobilePairValidity(SpMobileObject2D m1, int mindex1, int vindex1, SpMobileObject2D m2, int mindex2, int vindex2);

    bool checkMobilePairValidity(SpMobileObject2D m1, int mindex1, int vindex1, SpMobileObject2D m2, int mindex2, int vindex2, bool firstVerifiable);

    bool checkMobilePairValidity(SpMobileObject2D m1, SpMobileObject2D m2);

    bool checkMobilePairValidity(SpMobileObject2D m1, SpMobileObject2D m2, bool firstVerifiable);

    bool setPairValidityAndGo(int index1, int index2, bool ret_value);

    void initValidityMatrices(int size);

    bool alternativeWith3DInformation(SpRMAlternativeSolution2D alternative);

    bool blobAlreadyIncludedInNewMobilesSet(Blob *blob);

    void generateAlternativesWithBestSolutionsTree();

    void generateNewLeavesFromCurrentBests();

    void buildNewAlternativesFromLeaves(SpRMAlternativeSolution2D currentAlternative);

    double getTentativeAlternativeProbabilityValue(std::set<SpMobileObject2D, orderedByBestCoherenceOperator>::iterator *mobileIterators);

    SpRMAlternativeSolution2D getAlternativeFromNodeIfValid(SpBestAlternativesNode2D node);

    bool equalMobiles(SpRMAlternativeSolution2D alt1, SpRMAlternativeSolution2D alt2);

    bool sameUsed(SpMobileObject2D m1, SpMobileObject2D m2);

    void filterEquallyConvergedMobiles();

    bool highCoverage(SpMobileObject2D m1, SpMobileObject2D m2);

    void filterRepeatedAlternatives();

    bool acceptableInformationForNewMobile(SpMobileObject2D newMobile, SpMobileObject2D oldMobile);

    SpMobileObject2D setSpecialCaseMobile(DetectionProblemType dp_type, SpMobileObject2D currentObject, Blob *blobToAdd);

    SpMobileObject2D generateAndValidateNewMobile(SpMobileObject2D currentMobile, Blob *blobToTest);

    void generateBestPossiblePathsForMobiles(SpRMAlternativeSolution2D alternative);

    void generateMobilePathFromInitialBlob(SpMobileObject2D mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, double *blobSupport, Rectangle<int> *mobileBBox);

    void generateAlternativesForMobilePathFromInitialBlob(int length, int position, int value, int *alternativesCombo, int blobsToMergeNumber, int *blobsToMerge,
                                                        SpMobileObject2D mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, Rectangle<int> *mobileBBox);

    bool equal2DDimensions(SpMobileObject2D m1, SpMobileObject2D m2);

    bool blobCanBeIncludedForMerge(int currentMergeLength, int *mergedBlobIndexes, int currentBlobIndex);

    void cleanByEnsureUsedBlobs(SpRMAlternativeSolution2D newAlternative);

    SpMobileObject2D checkSpecialCases(SpMobileObject2D currentMobile, Blob *blobToTest);

};

typedef QSharedPointer<ReliabilityTracker2D> SpReliabilityTracker2D;


#endif // RELIABILITYTRACKER2D_H
