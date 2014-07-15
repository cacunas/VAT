#ifndef __MOBILE_OBJECT_2D_H__
#define __MOBILE_OBJECT_2D_H__

#include "blob.h"
#include "trajectory.h"
#include <iostream>
#include <map>
#include <deque>
#include <QImage>

#define MOBILE2D_DEBUG_DATA
#define MOBILE2D_DETAILS
#define FOR_SPREADSHEET_MOBILE2D
#define SHOW_BLOB2D_BUFFER

//Flag for controlling reliability effect
//#define __NO_RELIABILITY_CODE__

struct IncrementalExtraGeneralData2D {
    //General
    double prevCooling2D, sumCooling2D;
};

struct IncrementalExtra2DData {
    //For 2D position
    double XEstimated, sumRDX, sumRDVX;
    double YEstimated, sumRDY, sumRDVY;
    //For 2D Dimensions
    double WEstimated, sumRDW, sumRDVW;
    double HEstimated, sumRDH, sumRDVH;
};

typedef struct IncrementalExtraGeneralData2D IncrementalExtraGeneralData2D;
typedef struct IncrementalExtra2DData IncrementalExtra2DData;

struct tracking2DimensionalData {
    double W, RW, RDW, RCW, SDW;
    double H, RH, RDH, RCH, SDH;

    double VW, RDVW, RCVW, RVW, SDVW;
    double VH, RDVH, RCVH, RVH, SDVH;
};

typedef struct tracking2DimensionalData tracking2DimensionalData;

struct tracking2DSpatialData {
    double X, SDX, RX, RDX, RCX;    //X 2D coordinate of object position,desviacion,confiavilidad. confialidad espacial,confiavilidad
    double Y, SDY, RY, RDY, RCY;    //Y 2D coordinate of object position

    double VX, SDVX, RVX, RDVX, RCVX; //Velocity vector component in 2D X coordinates
    double VY, SDVY, RVY, RDVY, RCVY; //Velocity vector component in 2D Y coordinates

    double V2D; //2D Velocity Magnitude 
    double RV2D;   //Reliability of 2D Velocity
    double SDV2D;  //Standard deviation of 2D velocity magnitude
    double RDV2D;  //Reliability of velocity according to positions reliability
    double RCV2D;  //Reliability of velocity according to coherence
    double theta2D, SDtheta2D; //Velocity Angle w/r to the XY 2D plane. 
};

typedef struct tracking2DSpatialData tracking2DSpatialData;

class MobileObject2D;

typedef QSharedPointer<MobileObject2D> SpMobileObject2D;

struct orderedByBestCoherenceOperator2D {
    bool operator()(SpMobileObject2D mobile1, SpMobileObject2D mobile2);
};

struct orderedByMobileIdOperator2D {
    bool operator()(SpMobileObject2D mobile1, SpMobileObject2D mobile2);
};

class MobileObject2D {

  public:
    static double ONE_OVER_SQRT_2_TIMES_PI;

    static QImage *m_pSegmentation;

    static int m_currentTrackingBlobsNumber;
    static double m_lambda; //Cooling parameter
    static int m_blobsBufferSize;
    static int m_trajectoryMaxSize;
    static bool m_firstFrame;
    static double m_SpatialCoherenceReliabilityThreshold;
    static double m_SpatialCoherenceProbabilityThreshold;
    static double m_DimensionalCoherenceReliabilityThreshold;
    static double m_DimensionalCoherenceProbabilityThreshold;
    static double m_MinimalHistoryProbability;

    //Used to find the best type and to update buffer information
    static double *secDiffSequence;
    static double *secDiffToCurrent;
    static double *coolingValue;

    static double m_probabilityToEnsureMode;

    static double *g_secDiffSequence;
    static double *g_coolingValue;
    static double *g_secDiffToCurrent;
    static tracking2DimensionalData *g_t2DDimData;
    static tracking2DSpatialData    *g_t2DSpatialData;
    static IncrementalExtraGeneralData2D *g_iGData;
    static IncrementalExtra2DData *g_i2DData;
    static Rectangle<int> *g_newBBoxesToAnalyze;
    static double *g_newVisualSupport;
    static DetectionProblemType *g_newDPFlags;
    static int g_currentBufferSize;
    static double zeroTolerance;
    static double m_minimalTolerance;
    static int m_pixelAcuity;
    static double m_max2DSpeed;
    static double m_reliableObjectLength;

  public:
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
    int numberOfFramesNotSeen, numberOfFramesSinceFirstTimeSeen, numberOfFound;
    int previousBufferSize;
    int currentBufferSize;
    Rectangle<int> *bboxesToAnalyze;
    double *visualSupport;
    DetectionProblemType *dpFlags;
    DetectionProblemType currentVisualState;

    bool *foundSupport;
    int debug_data_flag;
    double RFoundSolutions, RVFoundSolutions;
    //Global Probabilities
    double P2D, R2D; //2D Dimensional Coherence Probability and Reliability
    double PV2DC, RV2DC;  //Velocity Coherence Probability and Reliability

    //Encapsulated tracking data
    tracking2DimensionalData t2DDimData;
    tracking2DSpatialData    t2DSpatialData;
    //Data for incremental values update
    IncrementalExtraGeneralData2D iGData;
    IncrementalExtra2DData i2DData;

    //History of previously seen blobs
    blobBuffer blobHistory;

    //History of previously seen blobs
    Trajectory trajectory2D;

    //Variables used by Reliability Tracker
    bool *usedBlobs;
    int numUsed;
    int usedLength;
    bool *involvedBlobs;
    int numInvolved;
    bool accepted_solution;
    bool comparedMobile;
    bool toErase;

    //Currently merged blobs for this object
    std::deque<int> currentlyMergedBlobs;

    MobileObject2D();
    MobileObject2D(SpMobileObject2D);
    ~MobileObject2D();

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
    int  getNumberOfFramesNotSeen();
    int  getNumberOfFramesSinceFirstTimeSeen();    
    double getGlobalProbability();

    void getMobileTolerances(double&, double&);

    //Auxiliar fuctions
    static double coolingFunction(double x);
    static double NormalizeOrientation(double);
    static double minimalAngularDistance(double alpha1, double alpha2);
    static double NormalizeVelocityAngle(double);

    void updateMobilePath(Blob *blob);
    void updateMobileData();
    double DimensionalCoherenceReliability(double sigma_dim, double min, double max);

    //Rendering Functions
    friend std::ostream& operator<<(std::ostream&, SpMobileObject2D);

    double dimensional2DReliability(double blobW, double blobH);
    double position2DReliability(double distance2D);

    double get2DDistanceToFocalPoint(Blob *blob);
    double get2DDistanceToFocalPoint(Rectangle<int> *rectangle);
    double get2DDistanceToFocalPoint(double X, double Y);
    bool mobile2DCoherenceIsAcceptable();
    bool mobile2DVelocityCoherenceIsAcceptable();

    void incorporateNewInformation();


    //Incremental Implementation for Updating Mobile Information
    void incrementalUpdateCooling(int bufferSize);
    void incrementalUpdateOrientation(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport);
    void incrementalUpdate2DPosition(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor);
    void incrementalUpdate2DDimensions(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor);

    void getCurrentBoundingBoxForMobile(Rectangle<int> *bbox);
    void getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox);

    void improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence);

    bool mobileOutOfScene(); //ACA!!! MIRAR QUE HACE
    void generateFirstClassifiedSequence();
    void generateMostCoherentSequence(std::deque<SpBlob>::iterator first_blob_it, int newBufferSize, int frame_index);

    double probabilisticCoherenceReliability(double data, double mean, double sigma, double acuity);
    double normalisedSigmaCoherenceReliability(double sigma, double acuity);

    void setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport);

    void setInitialNormalList(Blob *current_blob);
    
 };


typedef MobileObject2D* mobile_object_2D_t;

#endif	
