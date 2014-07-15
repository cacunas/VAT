#ifndef RMMTRACKER_H
#define RMMTRACKER_H

#include <QDomNode>
#include "Datapool.h"
#include "src/ReliabilityMerge.h"
#include "RMMHypothesisSetList.h"
#include "src/blob.h"
#include "src/TimeStamp.h"
#include "src/calibration.h"
#include "src/sharedTrackingClasses.h"

//#define RMTT_OUTPUT

class bestHypothesesNode;

typedef QSharedPointer<bestHypothesesNode> SpBestHypothesesNode;

class bestHypothesesNode {
  public:
    static int numVariables;
    static int variablesSum;
    static int *variablesNumFrames;

    static std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *newObjectsListEnds;

    double value;
    int mobileIndex;
    int *versionIndex;
    bool added;
    std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *mobileIterators;
    double *variableContribution;

    bestHypothesesNode(double i_value, int i_index);
    bestHypothesesNode(SpBestHypothesesNode toCopy, int indexToMove);
    ~bestHypothesesNode();

    void setNextHypothesisValue(int index);

};

struct orderedByBestHypothesisProbabilityCooperationOperator {
    bool operator()(SpBestHypothesesNode altNode1, SpBestHypothesesNode altNode2);
};


/*class IdBlobPair {
  public:
    long id;
    Blob *blob;

    IdBlobPair(long i_id, Blob *i_blob);
    ~IdBlobPair();
    bool operator<(IdBlobPair &P);
};*/



class RMMTracker {

public:
    Datapool *m_data;
    ReliabilityMerge *m_RMerge;
    ReliabilityMerge *m_PreMerge;
    //ReliabilityClassification *m_rclassif;
    SceneModel* smodel;
    QImage *m_pSegmentation;
    bool activatePreMerge;
    double maxObjectSpeed;
//    std::map<ObjectType, SpModelInterface> *objectModels;

    int m_BlobBufferSize;
    double meanw, meanl, meanh;
    /*!
    * \var double hypothesesProbabilityThreshold
    * \brief hypotheses probability threshold parameter for pre-elimination
    */
    double hypothesesProbabilityThreshold;


    int m_maximumGeneratedHypothesesPerMobile;

    /*!
    * \var int maximumRetainedHypotheses
    * \brief The maximun total number of hypotheses to retain.
    */
    int m_maximumRetainedHypotheses;

    /*!
    * \var int maximumMobilePaths
    * \brief The maximum number of retained generated mobile paths.
    */
    int maximumMobilePaths;

    /*!
    * \var int maximumHypothesesPerMobile
    * \brief The maximum number of generated hypotheses per mobile. This value is multiplied
    *        by the number of mobiles present in the analysed hypothesis.
    */
    int m_maximumHypothesesPerMobile;

    /*!
    * \var std::string eliminationStrategy
    * \brief The elimination strategy to use : worst probability elimination s.,
    *        random linear elimination s. and maximum likelihood estimator s.
    */
    QString eliminationStrategy;
    /*!
    * \fn Inline void computeInclusiveProbabilities()
    * \brief compute inclusive probabilities for a random linear elimination strategy
    */

  std::deque<IdBlobPair> biggestBlobForNewMobile;
  std::set<long int> usedIdsByHypothesis;
  std::map< long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> > mobileAlternativesMap;
  std::map< int, std::deque<long int> > mobileIdsUsingABlob;

  /*!
   * \var CReliableMobileObjectsList rMobilesList
   * \brief pointer to the tracking results
   */
  RMMHypothesisSetList hypothesisSets;

  std::map<long int, SpRMMMobileObject> mobilesOutList;

    bool parametersInitialised;

    QString m_MultiModelConfig;

    //Template for building the hierarchy by operator=
    ReliabilityMultiModel         modelTemplate;

    static int m_hypothesisNumber;
    static bool m_newOuts;
    static int acceptedPixelError;
    static double accepted3DFeatureError;
    static double acceptedOrientationError;
    static double m_highVisualSupportThreshold;
    static int m_meanMillisecondsDifferenceBetweenFrames;
    int lastMilliSecondsDifference;

    //PARAMETERS:
    double CoherenceReliabilityThreshold;
    double CoherenceProbabilityThreshold;

    double IgnoreByDimensionalCoherenceThreshold;
    double IgnoreByVelocityCoherenceThreshold;

    double ImportanceRateForBestMobileHypothesisProbability;

    bool m_internalOutputActivated;
    bool m_reducedOutputActivated;

    double m_lowVisualSupportThreshold;
    double m_blobCompletellySupportedThreshold;

    double m_InvolvedVisualEvidenceMultiplyingFactor;
    int m_InvolvedVisualEvidenceSumFactor;
    double m_InvolvedVisualEvidenceMinIntersectionRatio;

    bool m_setObjectLog;
    QString m_objectLogRootName;
    QString m_objectLogModelName;
    bool m_logModelFound;
    bool m_logFirstTime;
    std::map<unsigned long, QString> logFileNames;


    int *g_intersectionAreas;

    Blob **blobsVector;
    bool *usedBlobs;
    int *involvedHypothesisSetsCounter;
    bool **initialMergeMap;
    int *initialGroups;
    bool firstFrame;
    bool initialPreparation;

    bool *checkedMobilePairValidity, *validMobilePair;

    int blobsNumber;

    unsigned long mobile_id_counter;
    unsigned long hset_id_counter;
    int currentFrameNumber;
    unsigned long currentTimeMilliSeconds;
    TimeStamp lastTimeStamp;

    //Data used to generate the alternative paths for a mobile
    int g_inserted_for_hypothesis, g_NumCurrentlyLost;
    SpRMMHypothesis g_baseHypothesis;

    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> g_newHypotheses;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> g_completeHypotheses;
    std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> g_newMobiles;
    std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> g_newSpecialMobiles;
    std::deque<Blob *> g_includedBlobsInNewMobiles;
    std::deque<Blob *> g_allocatedBlobs;
    double g_bestGlobalP;
    int g_numberOfNewMobiles, g_NumLocallyAddedHypotheses, *g_numberOfNewMobileVersions, *g_mobileVersionIndex, g_totalNumberOfMobileVersions;
    std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *g_mobileIterators, *g_setBegins;
    std::set<SpBestHypothesesNode, orderedByBestHypothesisProbabilityCooperationOperator> g_leaves;

    std::map<long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> > *g_newObjectsList;
    double *g_variableContribution;

    double *distanceBetweenBlobs;

    bool m_activatePauseOnLost, m_pauseOnLost;
    bool m_activatePauseOnNew, m_pauseOnNew;

    RMMTracker(Datapool *i_data);
    ~RMMTracker();

    //Reset pause indicators.
    void resetPauseCriterias();

    void activatePause();

    //Implements default multimodel hierarchy if failed to extract multi model  from xml configuration file
    void setDefaultMultiModelTemplate();

    //Sets the buffer sizes according to the maximal buffer defined for a submodel:
    void setBufferSize();
    void setBufferSize(int &max, std::deque<SpReliabilitySingleModelInterface> &models);

    QString initObjectLog(long object_id);
    void writeObjectLog();

    void setRMMMobileObjectTemplates();

    bool setParameters(QDomNode& config);
    bool processModelParameters(QString& filename);

    static int getBetaDirection(SceneModel *, QImage *);

    void computeInclusiveProbabilities(std::set<SpRMMHypothesis,
                                       hypothesesOrderedByBestProbabilityOperator>* hypotheses,
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
    * \fn void getMostLikelyMobileObjects(std::deque<SpRMMMobileObject> *mobileObjectsOutput)
    * \brief clears the current mobiles list and gives the processed list of most likely mobiles
    * \return nothing, it modifies the class pointed by the given parameter
    */
    void getMostLikelyMobileObjects(std::deque<SpRMMMobileObject> &mobileObjectsOutput);

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

    void mergeInvolvedHypothesisSets();

    void mergeHypothesisSets(SpRMMHypothesisSet first, SpRMMHypothesisSet second);

    void determineInvolvedBlobsForMobile(SpRMMMobileObject mobile);

    void insertNewMobiles(SpRMMHypothesis asolution, SpRMMHypothesisSet hset);

    void setBlobsVector(std::vector<Blob>&);

    void freeBlobsVector();

    void presetBlobsVectorAndInitialMergeTable(std::vector<Blob>&);

    void freeBlobsVectorAndInitialMergeTable();

    SpRMMMobileObject setNotVisibleBlobMobile(SpRMMMobileObject currentMobile, DetectionProblemType dp_type);

    int setGroups(int elementsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector);

    void setGroup(int elementsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector);

    void generateNewMobiles(int blobNumberToAnalyze, int *blobsToAnalyze, int groupsNumber, int *blobGroupVector);

    void generateHypothesesForMobile(int *blobsToMerge, int blobsToMergeNumber);

    void recursiveComboGenerator(int position, int value, int *hypothesesCombo, int *blobsToMerge, int blobsToMergeNumber);

    bool validBlobMergeConfiguration(int mergeGroupNumber, int mergeLength, int blobsToMergeNumber, int *blobsToMerge, int *hypothesesCombo);

    void processMergeVector(int *hypothesesCombo, int *blobsToMerge, int blobsToMergeNumber);

    void checkConnectivity(bool groupVector[], int elementsNumber, int referencePoint, int *elementsToAnalyzeVector);

    void insertNewMergeSolution(Blob **mergedBlobs, int finalBlobsNumber, bool *usedBlobsMatrix);

    SpRMMMobileObject getNewMobileFromBlob(Blob *blob);

    void setCurrentTimeAndFrame();

    void getMergeConnections(bool *listToUpdate, int startingBlobIndex);

    void getRecursiveMergeConnections(bool *mergeList, int blobIndex);

    Blob *generateMostLikelyHypothesisForMobile(SpRMMMobileObject mobile);

    //Quicksort
    void orderAscending(int *array, int top, int bottom);
    int getMiddle(int *array, int top, int bottom);

    //SpRMMMobileObject setNotVisibleBlobMobile(DetectionProblemType dp_type, SpRMMMobileObject currentObject);

    void filterUnseenMobiles();

    void filterRepeatedHypotheses(SpRMMHypothesisSet hset);

    void separateReliableHypotheses();

    double getMaxFocalDistanceToImageCorner();

    double getObjectSizeForMaxReliability(double w, double l, double h);

    double getObjectDistanceForMaxReliability();

    void getNearest2DBlobPointToFocalPoint(Blob *blob, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, double xCenter, double yCenter, double W, double H, double *x, double *y);
    void getNearest2DBlobPointToFocalPoint(int position, Rectangle<int> *rect, double *x, double *y);

    void getHighest3DVariations(double x, double y, double X, double Y, double dX, double dY, double *dx, double *dy);

    void determineInvolvedBlobsForMobileObject(SpRMMMobileObject mobile);

    bool combinationAlreadyIncluded(Blob **blobsForHypothesis, int blobsNumberForHypothesis);

    int getMobileId(Blob *blob);

    bool notUsedId(long int id);

    void insertInMobileAlternativesMap(SpRMMMobileObject mobile, long int id);

    bool incoherentMobile(SpRMMMobileObject mobile);

    void generateHypothesesForMobilePath(int length, int position, int value, int *hypothesesCombo,
                                         int blobsToMergeNumber, int *blobsToMerge, SpRMMMobileObject mobile);

    void generateMobilePath(SpRMMMobileObject mobile);

    bool mobilesCombinationIsValid();

    double areaOfIntersection(SpRMMMobileObject m1, SpRMMMobileObject m2);

    double areaOfIntersectionFirstNo3D(SpRMMMobileObject m1, SpRMMMobileObject m2, double *areaOfNo3D);

    int getPairIndex(int mindex, int vindex);

    bool mobilePairValidityChecked(int index1, int index2);

    bool mobilePairIsValid(int index1, int index2);

    bool checkMobilePairValidity(SpRMMMobileObject m1, int mindex1, int vindex1, SpRMMMobileObject m2, int mindex2, int vindex2);

    bool checkMobilePairValidity(SpRMMMobileObject m1, int mindex1, int vindex1, SpRMMMobileObject m2, int mindex2, int vindex2, bool firstVerifiable);

    bool checkMobilePairValidity(SpRMMMobileObject m1, SpRMMMobileObject m2);

    bool checkMobilePairValidity(SpRMMMobileObject m1, SpRMMMobileObject m2, bool firstVerifiable);

    bool setPairValidityAndGo(int index1, int index2, bool ret_value);

    void initValidityMatrices(int size);

    bool hypothesisWith3DInformation(SpRMMHypothesis h);

    bool blobAlreadyIncludedInNewMobilesSet(Blob *blob);

    void generateHypothesesWithBestSolutionsTree();

    void generateNewLeavesFromCurrentBests();

    void buildNewHypothesesFromLeaves(SpRMMHypothesis currentHypothesis);

    double getTentativeHypothesisProbabilityValue(std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *mobileIterators);

    SpRMMHypothesis getHypothesisFromNodeIfValid(SpBestHypothesesNode node);

    bool equalMobiles(SpRMMHypothesis alt1, SpRMMHypothesis alt2);

    bool sameUsed(SpRMMMobileObject m1, SpRMMMobileObject m2);

    void filterEquallyConvergedMobiles();

    bool highCoverage(SpRMMMobileObject m1, SpRMMMobileObject m2);

    void filterRepeatedHypotheses();

    bool acceptableInformationForNewMobile(SpRMMMobileObject newMobile, SpRMMMobileObject oldMobile);

    SpRMMMobileObject setSpecialCaseMobile(DetectionProblemType dp_type, SpRMMMobileObject currentObject, Blob *blobToAdd);

    SpRMMMobileObject generateAndValidateNewMobile(SpRMMMobileObject currentMobile, Blob *blobToTest);

    void generateBestPossiblePathsForMobiles(SpRMMHypothesis hypothesis);
    //Testing version
    void generateBestPossiblePathsForMobiles2(SpRMMHypothesis hypothesis);

    void generateMobilePathFromInitialBlob(SpRMMMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, double *blobSupport, Rectangle<int> *mobileBBox);

    void generateHypothesesForMobilePathFromInitialBlob(int length, int position, int value, int *hypothesesCombo, int blobsToMergeNumber, int *blobsToMerge,
                                                        SpRMMMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, Rectangle<int> *mobileBBox);

    //bool equal2DDimensions(SpRMMMobileObject m1, SpRMMMobileObject m2);

    bool blobCanBeIncludedForMerge(int currentMergeLength, int *mergedBlobIndexes, int currentBlobIndex);

    void cleanByEnsureUsedBlobs(SpRMMHypothesis newHypothesis);

    SpRMMMobileObject checkSpecialCases(SpRMMMobileObject currentMobile, Blob *blobToTest);

};

typedef QSharedPointer<RMMTracker> SpRMMTracker;


#endif // RMMTRACKER_H
