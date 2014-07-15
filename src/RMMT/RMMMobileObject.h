#ifndef __RMM_MOBILE_OBJECT_H__
#define __RMM_MOBILE_OBJECT_H__

#include "MODELS/reliabilitymultimodel.h"
#include "src/ModelInterface.h"
#include "src/trajectory.h"
#include <iostream>
#include <map>
#include <deque>
#include <QImage>

#define MOBILE_DEBUG_DATA 
#define MOBILE_DETAILS 
#define SHOW_BLOB_BUFFER

class RMMMobileObject;

typedef QSharedPointer<RMMMobileObject> SpRMMMobileObject;

struct RMMorderedByBestCoherenceOperator {
    bool operator()(SpRMMMobileObject mobile1, SpRMMMobileObject mobile2);
};

struct RMMorderedByMobileIdOperator {
    bool operator()(SpRMMMobileObject mobile1, SpRMMMobileObject mobile2);
};

class RMMMobileObject {
  public:
    static double ONE_OVER_SQRT_2_TIMES_PI;

    static SceneModel *m_context;

    static double m_maxSpeed;
    static double m_maxKnownSpeed;
    static int m_currentTrackingBlobsNumber;
    static double m_lambda; //Cooling parameter
    static int m_blobsBufferSize;
    static int m_trajectoryMaxSize;
    static double m_knownSolutionThreshold;
    static double m_mobile2DCoverageRateToConsiderEqual;

    static std::map<ObjectType, bool> rigidModel;
    static std::map<ObjectType, int> objectModelMap;
    static std::map<ObjectType, std::map<ObjectSubtype, int> > objectSubModelMap;
    static int m_objectModelsNumber;
    static SpModelInterface *objectModelsList;
    static SpModelInterface **objectSubModelsList;

    static bool m_firstFrame;

    static double m_CoherenceReliabilityThreshold;
    static double m_CoherenceProbabilityThreshold;
    static double m_MinimalHistoryProbability;
    static double m_Maximal3DDimensionChangeSpeed;

    static std::map<ObjectType, ObjectSubtype> lastFoundSubtypeTemplate;

    //Used to find the best type and to update buffer information
    static double m_classifThreshold;

    static double *secDiffSequence;
    static double *secDiffToCurrent;
    static double *coolingValue;

    static double m_probabilityToEnsureMode;
    static int m_2DLevelFrames;

    static double zeroTolerance;
    static double m_minimalTolerance;

    static ReliabilityMultiModel modelTemplate;

    Datapool *m_data;

    unsigned long mobile_id;
    unsigned long hset_id;

    ObjectType best_type;
    ObjectSubtype best_subtype;
    int best_index;

    bool ensureMode; //Flag for indicating when a mobile object becomes "sure" in terms of type.
                     //Important flag for performing coherence validation just for one class type,
                     //optimizing computational cost.
    //    bool estimativeResult; //Marks if estimated are based in a priori information 

    int numberOfFramesNotSeen, numberOfFramesSinceFirstTimeSeen;
    int currentBufferSize;

    Rectangle<int> *bboxesToAnalyze;
    double *visualSupport;
    DetectionProblemType *dpFlags;
    DetectionProblemType currentVisualState;

    bool *foundSupport;
    int numberOfFound;

    double RKnownSolutions, RVKnownSolutions;
    double RFoundSolutions, RVFoundSolutions;

    //Global Probabilities
    double P; //Model Coherence        (tracker probability)
    double R; //Input Data Reliability (tracker reliability)

    //General cooling variables:
    double prevCooling, sumCooling;

    //The multi-model object
    ReliabilityMultiModel multiModel;

    //History of previously seen positions
    Trajectory trajectory3D;
    Trajectory trajectory2D;

    //Variables used by RMM Tracker
    bool *usedBlobs;
    int numUsed;
    bool *involvedBlobs;
    int numInvolved;
    Rectangle<int> iestimator;
    bool accepted_solution;
    int unknownsNumber;
    bool classificationAllowed;
    bool comparedMobile;
    bool toErase;

    RMMMobileObject(Datapool *i_data);
    RMMMobileObject(SpRMMMobileObject m);
    ~RMMMobileObject();

    //Init functions
    void initUsedBlobs();
    void initInvolvedBlobs();
    void initMaps();
    //Inserting Functions
    void insertNewMultiModel(Blob *blob, double lastSecDifference);

    void setInitialMultiModel(Blob *blob);

    //Build new model instances from template and associate pointer to mobile object
    void renewModels();

    void get2DTrajectoryPoint(double &x, double &y, double &R);
    void insert2DTrajectoryPoint();

    //Setting Functions
    void setMobileId(unsigned long i_id);
    void setRMobileId(unsigned long i_id);
    void setNewMobileFromBlob(Blob *blob, unsigned long mobile_id, unsigned long rmobile_id);
    void setNumberOfFramesNotSeen(int num);
    void incrementNumberOfFramesNotSeen();
    void incrementNumberOfFramesSinceFirstTimeSeen();     
    void setGlobalProbability();
    Blob *determineMostLikelyBlob();
    //Getting Functions
    unsigned long getMobileId();
    unsigned long getHypothesisSetId();
    ObjectType getBestType();
    ObjectSubtype getBestSubType();
    int  getNumberOfFramesNotSeen();
    int  getNumberOfFramesSinceFirstTimeSeen();    
    double getGlobalProbability();
    SpReliabilitySingleModelInterface getSubModel(QString);
    SpReliabilitySingleModelInterface getSubModel(QString name, std::deque<SpReliabilitySingleModelInterface> &models);

    //Auxiliar fuctions
    static double NormalizeOrientation(double);
    static double minimalAngularDistance(double alpha1, double alpha2);
    static double NormalizeVelocityAngle(double);
    static double coolingFunction(double x);

    void updateMobilePath(Blob *blob);
    void updateMobileBufferStructures(Blob *blob);
    void updateMobileDynamics();
    void initDynamics();

    void initCooling();

    double DimensionalCoherenceReliability(double sigma_dim, double min, double max);

    //Rendering Functions
    friend std::ostream& operator<<(std::ostream&, SpRMMMobileObject);

    //Incremental Implementation for Updating Mobile Information
    void updateBestTypeInformation();
    void incrementalUpdateDynamics(std::deque<SpReliabilitySingleModelInterface> &dyn);
    void updateCooling();

    void getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox);

    void improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence);

    bool mobileOutOfScene();

    double probabilisticCoherenceReliability(double data, double mean, double sigma, double acuity);

    void setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport);

    void setInitialNormalList(Blob *current_blob);

    Rectangle<int> getVisualEvidenceEstimator();
    Rectangle<int> getVisualEvidenceEstimator(double &R);

    
 };


typedef RMMMobileObject* rmm_mobile_object_t;

#endif	
