#ifndef RELIABILITYTRACKER_H
#define RELIABILITYTRACKER_H

#include <QDomNode>
#include "Datapool.h"
#include "ReliabilityMerge.h"
#include "ReliableMobileObjectList.h"
#include "blob.h"
#include "TimeStamp.h"
#include "calibration.h"
#include "sharedTrackingClasses.h"
class ReliabilityClassification;

class ReliabilityTracker {

public:
    Datapool *m_data;
    ReliabilityMerge *m_RMerge;
    ReliabilityMerge *m_PreMerge;
    ReliabilityClassification *m_rclassif;
    SceneModel* smodel;
    QImage *m_pSegmentation;
    bool activatePreMerge;
    double maxObjectSpeed;
    std::map<ObjectType, SpModelInterface> *objectModels;

    int m_BlobBufferSize;
    double meanw, meanl, meanh;
    /*!
    * \var double alternativeSolutionsProbabilityThreshold
    * \brief alternative solutions probability threshold parameter for pre-elimination
    */
    double alternativeSolutionsProbabilityThreshold;

    /*!
    * \var int maximumRetainedAlternativeSolutions
    * \brief The maximun total number of alternative solutions to retain.
    */
    int m_maximumRetainedAlternativeSolutions;

    /*!
    * \var int maximumAlternativesMobilePaths
    * \brief The maximum number of retained generated mobile paths.
    */
    int maximumAlternativeMobilePaths;

    /*!
    * \var int maximumGeneratedAlternativesPerMobile
    * \brief The maximum number of generated alternatives per mobile. This value is multiplied
    *        by the number of mobiles present in the analysed alternative.
    */
    int m_maximumGeneratedAlternativesPerMobile;

    /*!
    * \var std::string eliminationStartegy
    * \brief The elimination strategy to use : worst probability elimination s.,
    *        random linear elimination s. and maximum likelihood estimator s.
    */
    QString eliminationStrategy;
    /*!
    * \fn Inline void computeInclusiveProbabilities()
    * \brief compute inclusive probabilities for a random linear elimination strategy
    */

  std::deque<IdBlobPair> biggestBlobForNewMobile;
  std::set<long int> usedIdsByAlternative;
  std::map< long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> > mobileAlternativesMap;
  std::map< int, std::deque<long int> > mobileIdsUsingABlob;

  /*!
   * \var CReliableMobileObjectsList rMobilesList
   * \brief pointer to the tracking results
   */
  ReliableMobileObjectList rMobilesList;

  /*!
   * \var CReliableMobileObjectsList rMobilesList
   * \brief pointer to the tracking results
   */
  std::map<long int, SpMobileObject> mobilesOutList;

    bool parametersInitialised;
    static int m_alternativeNumber;
    static bool m_newOuts;
    static int acceptedPixelError;
    static double accepted3DFeatureError;
    static double acceptedOrientationError;
    static double m_highVisualSupportThreshold;
    static int m_meanMillisecondsDifferenceBetweenFrames;
    int lastMilliSecondsDifference;

    //PARAMETERS:
    double SpatialCoherenceReliabilityThreshold;
    double SpatialCoherenceProbabilityThreshold;

    double IgnoreByDimensionalCoherenceThreshold;
    double IgnoreByVelocityCoherenceThreshold;

    double ImportanceRateForBestMobileAlternativeProbability;

    bool m_internalOutputActivated;
    bool m_reducedOutputActivated;

    double m_maximal3DBaseOverlapping;
    double m_mobile2DCoverageRateToConsiderEqual;

    double m_lowVisualSupportThreshold;
    double m_blobCompletellySupportedThreshold;

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
    SpRMAlternativeSolution g_baseAlternative;

    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> g_newAlternatives;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> g_completeAlternatives;
    std::set<SpMobileObject, orderedByBestCoherenceOperator> g_newMobiles;
    std::set<SpMobileObject, orderedByBestCoherenceOperator> g_newSpecialMobiles;
    std::deque<Blob *> g_includedBlobsInNewMobiles;
    std::deque<Blob *> g_allocatedBlobs;
    double g_bestGlobalP;
    int g_numberOfNewMobiles, g_NumLocallyAddedAlternatives, *g_numberOfNewMobileVersions, *g_mobileVersionIndex, g_totalNumberOfMobileVersions;
    std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *g_mobileIterators, *g_setBegins;
    std::set<SpBestAlternativesNode, orderedByBestAlternativeProbabilityCooperationOperator> g_leaves;

    std::map<long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> > *g_newObjectsList;
    bool *g_acceptable3DCoherenceForMobile;
    double *g_variableContribution;

    double *distanceBetweenBlobs;

    ReliabilityTracker(Datapool *i_data);
    ~ReliabilityTracker();

    bool setParameters(QDomNode& config);
    static int getBetaDirection(SceneModel *, QImage *);

    void computeInclusiveProbabilities(std::set<SpRMAlternativeSolution,
                                       alternativesOrderedByBestProbabilityOperator>* alternativeSolutions,
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
    void getMostLikelyMobileObjects(std::deque<SpMobileObject> &mobileObjectsOutput);

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

    void mergeReliableMobiles(SpReliableMobileObject firstRMobile, SpReliableMobileObject secondRMobile);

    void determineInvolvedBlobsForMobile(SpMobileObject mobile);

    void insertNewMobiles(SpRMAlternativeSolution asolution, SpReliableMobileObject rmobile);

    void setBlobsVector(std::vector<Blob>&);

    void freeBlobsVector();

    void presetBlobsVectorAndInitialMergeTable(std::vector<Blob>&);

    void freeBlobsVectorAndInitialMergeTable();

    SpMobileObject setNotVisibleBlobMobile(SpMobileObject currentMobile, DetectionProblemType dp_type);

    int setGroups(int elementsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector);

    void setGroup(int elementsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector);

    void generateNewMobiles(int blobNumberToAnalyze, int *blobsToAnalyze, int groupsNumber, int *blobGroupVector);

    void generateAlternativesForMobile(int *blobsToMerge, int blobsToMergeNumber);

    void recursiveComboGenerator(int position, int value, int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber);

    bool validBlobMergeConfiguration(int mergeGroupNumber, int mergeLength, int blobsToMergeNumber, int *blobsToMerge, int *alternativesCombo);

    void processMergeVector(int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber);

    void checkConnectivity(bool groupVector[], int elementsNumber, int referencePoint, int *elementsToAnalyzeVector);

    void insertNewMergeSolution(Blob **mergedBlobs, int finalBlobsNumber, bool *usedBlobsMatrix);

    SpMobileObject getNewMobileFromBlob(Blob *blob);

    void setCurrentTimeAndFrame();

    void getMergeConnections(bool *listToUpdate, int startingBlobIndex);

    void getRecursiveMergeConnections(bool *mergeList, int blobIndex);

    Blob *generateMostLikelyAlternativeForMobile(SpMobileObject mobile);

    //Quicksort
    void orderAscending(int *array, int top, int bottom);
    int getMiddle(int *array, int top, int bottom);

    SpMobileObject setNotVisibleBlobMobile(DetectionProblemType dp_type, SpMobileObject currentObject);

    void filterUnseenMobiles();

    void filterRepeatedAlternatives(SpReliableMobileObject rmobile);

    void separateReliableSolutions();

    double getMaxFocalDistanceToImageCorner();

    double getObjectSizeForMaxReliability(double w, double l, double h);

    double getObjectDistanceForMaxReliability();

    void getNearest2DBlobPointToFocalPoint(Blob *blob, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, double xCenter, double yCenter, double W, double H, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, Rectangle<int> *rect, double *x, double *y);

    void getHighest3DVariations(double x, double y, double X, double Y, double dX, double dY, double *dx, double *dy);

    void determineInvolvedBlobsForMobileObject(SpMobileObject mobile);

    bool combinationAlreadyIncluded(Blob **blobsForAlternative, int blobsNumberForAlternative);

    int getMobileId(Blob *blob);

    bool notUsedId(long int id);

    void insertInMobileAlternativesMap(SpMobileObject mobile, long int id);

    bool incoherentMobile(SpMobileObject mobile);

    void generateAlternativesForMobilePath(int length, int position, int value, int *alternativesCombo,
                                         int blobsToMergeNumber, int *blobsToMerge, SpMobileObject mobile);

    void generateMobilePath(SpMobileObject mobile);

    bool mobilesCombinationIsValid();

    double areaOfIntersection(SpMobileObject m1, SpMobileObject m2);

    double areaOfIntersectionFirstNo3D(SpMobileObject m1, SpMobileObject m2, double *areaOfNo3D);

    int getPairIndex(int mindex, int vindex);

    bool mobilePairValidityChecked(int index1, int index2);

    bool mobilePairIsValid(int index1, int index2);

    bool checkMobilePairValidity(SpMobileObject m1, int mindex1, int vindex1, SpMobileObject m2, int mindex2, int vindex2);

    bool checkMobilePairValidity(SpMobileObject m1, int mindex1, int vindex1, SpMobileObject m2, int mindex2, int vindex2, bool firstVerifiable);

    bool checkMobilePairValidity(SpMobileObject m1, SpMobileObject m2);

    bool checkMobilePairValidity(SpMobileObject m1, SpMobileObject m2, bool firstVerifiable);

    bool setPairValidityAndGo(int index1, int index2, bool ret_value);

    void initValidityMatrices(int size);

    bool alternativeWith3DInformation(SpRMAlternativeSolution alternative);

    bool blobAlreadyIncludedInNewMobilesSet(Blob *blob);

    void generateAlternativesWithBestSolutionsTree();

    void generateNewLeavesFromCurrentBests();

    void buildNewAlternativesFromLeaves(SpRMAlternativeSolution currentAlternative);

    double getTentativeAlternativeProbabilityValue(std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *mobileIterators);

    SpRMAlternativeSolution getAlternativeFromNodeIfValid(SpBestAlternativesNode node);

    bool equalMobiles(SpRMAlternativeSolution alt1, SpRMAlternativeSolution alt2);

    bool sameUsed(SpMobileObject m1, SpMobileObject m2);

    void filterEquallyConvergedMobiles();

    bool highCoverage(SpMobileObject m1, SpMobileObject m2);

    void filterRepeatedAlternatives();

    bool acceptableInformationForNewMobile(SpMobileObject newMobile, SpMobileObject oldMobile);

    SpMobileObject setSpecialCaseMobile(DetectionProblemType dp_type, SpMobileObject currentObject, Blob *blobToAdd);

    SpMobileObject generateAndValidateNewMobile(SpMobileObject currentMobile, Blob *blobToTest);

    void generateBestPossiblePathsForMobiles(SpRMAlternativeSolution alternative);

    void generateMobilePathFromInitialBlob(SpMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, double *blobSupport, Rectangle<int> *mobileBBox);

    void generateAlternativesForMobilePathFromInitialBlob(int length, int position, int value, int *alternativesCombo, int blobsToMergeNumber, int *blobsToMerge,
                                                        SpMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, Rectangle<int> *mobileBBox);

    bool equal2DDimensions(SpMobileObject m1, SpMobileObject m2);

    bool blobCanBeIncludedForMerge(int currentMergeLength, int *mergedBlobIndexes, int currentBlobIndex);

    void cleanByEnsureUsedBlobs(SpRMAlternativeSolution newAlternative);

    SpMobileObject checkSpecialCases(SpMobileObject currentMobile, Blob *blobToTest);

};

typedef QSharedPointer<ReliabilityTracker> SpReliabilityTracker;


#endif // RELIABILITYTRACKER_H
