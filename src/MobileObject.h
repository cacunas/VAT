#ifndef __MOBILE_OBJECT_H__
#define __MOBILE_OBJECT_H__

#include "blob.h"
#include "calibration.h"
#include "parallelpiped.h"
#include "ModelInterface.h"
#include "MobileObject2D.h"
#include "trajectory.h"
#include <iostream>
#include <map>
#include <deque>
#include <QImage>

#define MOBILE_DEBUG_DATA 
#define MOBILE_DETAILS 
#define SHOW_BLOB_BUFFER

struct IncrementalExtraGeneralData {
    //General
    double prevCooling2D, sumCooling2D, prevCooling3D, sumCooling3D;
    double totalNumClassified;
};

struct IncrementalExtra3DData {
    //For 3D orientation
    double alphaEstimated, sumRDalpha, sumRDValpha;
    //For 3D dimensions
    double wEstimated, sumRDw, sumRDVw, sumPw;
    double lEstimated, sumRDl, sumRDVl, sumPl;
    double hEstimated, sumRDh, sumRDVh, sumPh;
    //For 3D position
    double sumRDxy, sumRDVxy, sumPxy, sumPVxy, xEstimated, yEstimated;
};


typedef struct IncrementalExtraGeneralData IncrementalExtraGeneralData;
typedef struct IncrementalExtra3DData IncrementalExtra3DData;

struct tracking3DimensionalData {
    double alpha,  RDalpha,  RCalpha,  Ralpha,  SDalpha;
    double Valpha, RDValpha, RCValpha, RValpha, SDValpha;

    double w, Pw, Rw, RCw, RDw, SDw;
    double l, Pl, Rl, RCl, RDl, SDl;
    double h, Ph, Rh, RCh, RDh, SDh;

    double Vw, RVw, RCVw, RDVw, SDVw;
    double Vl, RVl, RCVl, RDVl, SDVl;
    double Vh, RVh, RCVh, RDVh, SDVh;

};

typedef struct tracking3DimensionalData tracking3DimensionalData;

struct tracking3DSpatialData {
    double x, SDx, Rx, RCx;    //x 3D coordinate of object position
    double y, SDy, Ry, RCy;    //y 3D coordinate of object position
    double RDxy; //Reliability of position due to mean reliability of parallelepiped base dimensions. 
    double Ppos; //Probability of position due to mean probability of parallelepiped base dimensions. 

    double Vx, SDVx, RVx, RCVx; //Velocity vector component in 3D x coordinates
    double Vy, SDVy, RVy, RCVy; //Velocity vector component in 3D y coordinates
    double RDV; //Reliability of velocity according to positions reliability
    double RCV; //Reliability of velocity according to coherence

    double V, //Velocity Magnitude 
      PV,     //Probability of Velocity given positions probability
      RV,     //Reliability of Velocity
      SDV,    //Standard deviation of velocity magnitude
      MPV;    //Probability of Velocity Magnitude according to best type
    double theta, SDtheta; //Velocity Angle w/r to the xy 3D plane. 

};

typedef struct tracking3DSpatialData tracking3DSpatialData;

struct info3D {
    IncrementalExtraGeneralData iGData;
    IncrementalExtra3DData i3DData;
    tracking3DimensionalData t3DDimData;
    tracking3DSpatialData t3DSpatialData;
};

typedef struct info3D info3D;


class MobileObject;
class ReliabilityClassification;

typedef QSharedPointer<MobileObject> SpMobileObject;

struct orderedByBestCoherenceOperator {
    bool operator()(SpMobileObject mobile1, SpMobileObject mobile2);
};

struct orderedByMobileIdOperator {
    bool operator()(SpMobileObject mobile1, SpMobileObject mobile2);
};

class MobileObject {

  public:
    static double ONE_OVER_SQRT_2_TIMES_PI;

    static SceneModel *m_context;
    static ReliabilityClassification *m_rc;
    static QImage *m_pSegmentation;
    static double m_maxSpeed;
    static double m_maxKnownSpeed;

    static int m_currentTrackingBlobsNumber;
    static double m_lambda; //Cooling parameter
    static int m_blobsBufferSize;
    static int m_trajectoryMaxSize;
    static double m_knownSolutionThreshold;

    static std::map<ObjectType, bool> rigidModel;
    static std::map<ObjectType, int> objectModelMap;
    static std::map<ObjectType, std::map<ObjectSubtype, int> > objectSubModelMap;
    static SpModelInterface *objectModelsList;
    static SpModelInterface **objectSubModelsList;

    static double **objectSubModelMinWidth;
    static double **objectSubModelMeanWidth;
    static double **objectSubModelMaxWidth;
    static double **objectSubModelMinLength;
    static double **objectSubModelMeanLength;
    static double **objectSubModelMaxLength;
    static double **objectSubModelMinHeight;
    static double **objectSubModelMeanHeight;
    static double **objectSubModelMaxHeight;
    static double **objectSubModelMinVelocity;
    static double **objectSubModelMeanVelocity;
    static double **objectSubModelMaxVelocity;

    static double *objectModelMinWidth;
    static double *objectModelMeanWidth;
    static double *objectModelMaxWidth;
    static double *objectModelMinLength;
    static double *objectModelMeanLength;
    static double *objectModelMaxLength;
    static double *objectModelMinHeight;
    static double *objectModelMeanHeight;
    static double *objectModelMaxHeight;
    static double *objectModelMinVelocity;
    static double *objectModelMeanVelocity;
    static double *objectModelMaxVelocity;

    static int m_objectModelsNumber;
    static bool m_firstFrame;
    static int m_numberOfPostures;

    static double m_SpatialCoherenceReliabilityThreshold;
    static double m_SpatialCoherenceProbabilityThreshold;
    static double m_DimensionalCoherenceReliabilityThreshold;
    static double m_DimensionalCoherenceProbabilityThreshold;
    static double m_MinimalHistoryProbability;
    static double m_Maximal3DDimensionChangeSpeed;

    static std::map<ObjectType, ObjectSubtype> lastFoundSubtypeTemplate;

    //Used to find the best type and to update buffer information
    static double m_classifThreshold;
    static double m_maximalAlphaRotationSpeed;

    static double *secDiffSequence;
    static double *secDiffToCurrent;
    static double *coolingValue;

    static double m_maxFocalDistance;
    static double m_objectSizeForMaxReliability;
    static double m_objectDimensionForMaxReliability;
    static double m_objectDistanceForMaxReliability;

    static double *g_postureMinw, *g_postureMinl, *g_postureMinh;
    static double *g_postureMeanw, *g_postureMeanl, *g_postureMeanh;
    static double *g_postureMaxw, *g_postureMaxl, *g_postureMaxh;
    static ObjectSubtype *g_posturesList;

    static double m_probabilityToEnsureMode;
    static int m_2DLevelFrames;

    static double *g_secDiffSequence;
    static double *g_coolingValue;
    static double *g_secDiffToCurrent;
    static tracking2DimensionalData *g_t2DDimData;
    static tracking2DSpatialData    *g_t2DSpatialData;
    static IncrementalExtraGeneralData *g_iGData;
    static IncrementalExtra2DData *g_i2DData;
    static Rectangle<int> *g_newBBoxesToAnalyze;
    static double *g_newVisualSupport;
    static DetectionProblemType *g_newDPFlags;
    static int g_currentBufferSize;
    static double zeroTolerance;
    static double m_minimalTolerance;

  private:
    unsigned long mobile_id;
    unsigned long rmobile_id;
    ObjectType best_type;
    ObjectSubtype best_subtype;

    //Global Probability Measure
    double P;

  public:
    bool ensureMode; //Flag for indicating when a mobile object becomes "sure" in terms of type.
                     //Important flag for performing coherence validation just for one class type,
                     //optimizing computational cost.
    //    bool estimativeResult; //Marks if estimated are based in a priori information 

    bool productionMode;   //Flag for indicating to certain functions to not to add trajectory points
                           //because several alternatives are being tested (if false, it means "Test Mode").
    bool lastUnknown, *lastKnownPerType;
    int best_index;
    int numberOfFramesNotSeen, numberOfFramesSinceFirstTimeSeen;
    int previousBufferSize;
    int currentBufferSize;
    Shape3DData **s3dsToAnalyzeAllTypes;
    Shape3DData **s3dsToAnalyze;
    info3D *i3D;
    Rectangle<int> *bboxesToAnalyze;
    double *visualSupport;
    DetectionProblemType *dpFlags;
    DetectionProblemType currentVisualState;
    bool *classifiedS3ds;
    bool *foundS3ds;

    double *PSumPerType;
    int debug_data_flag;

    int numberOfClassifiedS3ds, numberOfFoundS3ds;
    double RKnownSolutions, RVKnownSolutions;
    double RFoundSolutions, RVFoundSolutions;
    //Global Probabilities
    double P2D, R2D; //2D Dimensional Coherence Probability and Reliability
    double P3D, R3D; //3D Dimensional Coherence Probability and Reliability
    double PVC, RVC;  //Velocity Coherence Probability and Reliability
    double PV2DC, RV2DC;  //Velocity Coherence Probability and Reliability

    //Best type limits
    double current_min_w_model;
    double current_max_w_model;
    double current_min_l_model;
    double current_max_l_model;
    double current_min_h_model;
    double current_max_h_model;
    double current_min_velocity_model;
    double current_max_velocity_model;

    //Encapsulated tracking data
    tracking3DimensionalData t3DDimData;
    tracking2DimensionalData t2DDimData;
    tracking3DSpatialData    t3DSpatialData;
    tracking2DSpatialData    t2DSpatialData;
    //Data for incremental values update
    IncrementalExtraGeneralData iGData;
    IncrementalExtra2DData i2DData;
    IncrementalExtra3DData i3DData;

    //History of previously seen blobs
    blobBuffer blobHistory;

    //History of previously seen blobs
    Trajectory trajectory3D;
    Trajectory trajectory2D;

    //Variables used by Reliability Tracker
    bool *usedBlobs;
    int numUsed;
    bool *involvedBlobs;
    int numInvolved;
    bool accepted_solution;
    std::map<ObjectType, ObjectSubtype> lastFoundSubtype;
    int unknownsNumber;
    bool classificationAllowed;
    bool comparedMobile;
    bool toErase;

    MobileObject();
    MobileObject(SpMobileObject);
    ~MobileObject();

    //Init functions
    void initUsedBlobs();
    void initInvolvedBlobs();
    void initMaps();
    //Inserting Functions
    void insertNewBlob(Blob *blob);
    void insertNewBlob(Blob *blob, int lastMilliSecondsDifference);

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
    unsigned long getRMobileId();
    ObjectType getBestType();
    ObjectSubtype getBestSubType();
    int  getNumberOfFramesNotSeen();
    int  getNumberOfFramesSinceFirstTimeSeen();    
    double getGlobalProbability();

    //Auxiliar fuctions
    static double NormalizeOrientation(double);
    static double minimalAngularDistance(double alpha1, double alpha2);
    static double NormalizeVelocityAngle(double);
    static double coolingFunction(double x);

    void velocityMagnitudeAccordingToModels(Shape3DData *s3d);
    double velocityAngleAccordingToEntranceToScene(Shape3DData *s3d);
    void setAprioriVelocity(Shape3DData *s3d);

    void updateMobilePath(Blob *blob);
    void updateMobileData();

    double DimensionalCoherenceReliability(double sigma_dim, double min, double max);

    //Rendering Functions
    friend std::ostream& operator<<(std::ostream&, SpMobileObject);

    //Data for best types history
    Shape3DData **getS3DBufferToAnalyze(ObjectType type);
    Shape3DData **getS3DBufferToAnalyzeByIndex(int index);
    Shape3DData *getS3DToAnalyze(ObjectType type, int frame);
    Shape3DData *getS3DToAnalyzeByIndex(int index, int frame);
    void setS3DToAnalyzeByIndex(int index, int frame, Shape3DData *s3d);
    void insertS3DToAnalyzeByIndex(int index, Shape3DData *s3d);

    Shape3DData *getRightS3D(std::map<ObjectType, Shape3DData> *list, ObjectType type);
    Shape3DData *getBestPostureCoherentS3DReclassifying(Blob *blob, std::map<ObjectType, Shape3DData> *normal,
                                                        std::map<ObjectType, Shape3DData> *occ, ObjectType type);

    double dimensional2DReliability(double distance2D, double blobW, double blobH);
    double position2DReliability(double distance2D);

    double get2DDistanceToFocalPoint(Blob *blob);
    double get2DDistanceToFocalPoint(Rectangle<int> *rectangle);
    double get2DDistanceToFocalPoint(double X, double Y);

    bool mobile3DCoherenceIsAcceptable();
    bool mobile2DCoherenceIsAcceptable();

    bool mobile3DVelocityCoherenceIsAcceptable();

    Shape3DData *getMostCoherentDataFromMobileAndBBoxLimit(std::map<ObjectType, Shape3DData> *normal,
                                                           std::map<ObjectType, Shape3DData> *occ, Blob *blob);
    
    void initPostureLimits(ObjectType type);

    void freePostureLimits();

    //Incremental Implementation for Updating Mobile Information
    void updateBestType3DInformation();
    void incrementalUpdateCooling(int bufferSize);
    void incrementalUpdateOrientation(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport);
    void incrementalUpdate3DDimensions(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport);
    void incrementalUpdate3DPosition(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport);
    void incrementalUpdate2DPosition(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor);
    void incrementalUpdate2DDimensions(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor);

    void getCurrentBoundingBoxForMobile(Rectangle<int> *bbox);
    void getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox);

    void improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence);
    void generateS3DFrom3DInformation(Shape3DData *outS3D);

    bool mobileOutOfScene(); //ACA!!! MIRAR QUE HACE
    void generateFirstClassifiedSequence();
    void setBestS3DSequence(std::deque<SpBlob>::iterator first_blob_it, int frame_index, int newBufferSize);
    void generateMostCoherentS3DSequence(std::deque<SpBlob>::iterator first_blob_it, Shape3DData *firstClassified, int newBufferSize, int frame_index, ObjectType current_type, int current_index);

    double probabilisticCoherenceReliability(double data, double mean, double sigma, double acuity);

    double initialSDEstimateFor2DHorizontalAttribute();
    double initialSDEstimateFor2DVerticalAttribute();

    void setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport);

    void setInitialNormalList(Blob *current_blob);

    bool coherentWithRespectOfCurrent3DInformation(Shape3DData *s3d);

    void repositionS3D(Shape3DData *s3d, double x, double y);
    
    void orient3DModel(Shape3DData *improved3DInfo, Rectangle<int> *improvedBBox, DetectionProblemType dptype);

    Shape3DData *generateBestLikelyS3D(Blob *blob, ObjectType type);

    Shape3DData *getBestS3D(bool &lastUnknown, Blob *current_blob);

    void getMobile3DTolerances(double *Wtol, double *Htol);
 };


typedef MobileObject* mobile_object_t;

#endif	
