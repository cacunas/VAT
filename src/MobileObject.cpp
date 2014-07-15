#include "MobileObject.h"
#include "ReliabilityTracker.h"
#include "ReliabilityClassification.h"
#include "MathFunctions.h"
#include "geometric.h"
#include <cmath>
//#include <values.h> values.h? //never heard of it before

double MobileObject::ONE_OVER_SQRT_2_TIMES_PI = 1.0 / sqrt(2 * M_PI);

SceneModel *MobileObject::m_context = NULL;
ReliabilityClassification *MobileObject::m_rc = NULL;
QImage *MobileObject::m_pSegmentation = NULL;
double MobileObject::m_maxSpeed = 5000; // 5000 cm/s
double MobileObject::m_maxKnownSpeed = 5000; // 5000 cm/s

int MobileObject::m_currentTrackingBlobsNumber = 0;
double MobileObject::m_lambda = 1000; // constant for cooling function of time difference in seconds
int MobileObject::m_blobsBufferSize = 5;
int MobileObject::m_trajectoryMaxSize = 0; // 0: now limit, >0: stablished limit
double MobileObject::m_knownSolutionThreshold = 0.5;

std::map<ObjectType, bool> MobileObject::rigidModel;
std::map<ObjectType, int> MobileObject::objectModelMap;
std::map<ObjectType, std::map<ObjectSubtype, int> >MobileObject::objectSubModelMap;
SpModelInterface *MobileObject::objectModelsList = NULL;
SpModelInterface **MobileObject::objectSubModelsList = NULL;

double **MobileObject::objectSubModelMinWidth = NULL;
double **MobileObject::objectSubModelMeanWidth = NULL;
double **MobileObject::objectSubModelMaxWidth = NULL;
double **MobileObject::objectSubModelMinLength = NULL;
double **MobileObject::objectSubModelMeanLength = NULL;
double **MobileObject::objectSubModelMaxLength = NULL;
double **MobileObject::objectSubModelMinHeight = NULL;
double **MobileObject::objectSubModelMeanHeight = NULL;
double **MobileObject::objectSubModelMaxHeight = NULL;
double **MobileObject::objectSubModelMinVelocity = NULL;
double **MobileObject::objectSubModelMeanVelocity = NULL;
double **MobileObject::objectSubModelMaxVelocity = NULL;
double *MobileObject::objectModelMinWidth = NULL;
double *MobileObject::objectModelMeanWidth = NULL;
double *MobileObject::objectModelMaxWidth = NULL;
double *MobileObject::objectModelMinLength = NULL;
double *MobileObject::objectModelMeanLength = NULL;
double *MobileObject::objectModelMaxLength = NULL;
double *MobileObject::objectModelMinHeight = NULL;
double *MobileObject::objectModelMeanHeight = NULL;
double *MobileObject::objectModelMaxHeight = NULL;
double *MobileObject::objectModelMinVelocity = NULL;
double *MobileObject::objectModelMeanVelocity = NULL;
double *MobileObject::objectModelMaxVelocity = NULL;

int MobileObject::m_objectModelsNumber = 0;
bool MobileObject::m_firstFrame = true;
int MobileObject::m_numberOfPostures;

double MobileObject::m_SpatialCoherenceReliabilityThreshold = 0.1;
double MobileObject::m_SpatialCoherenceProbabilityThreshold = 0.5;
double MobileObject::m_DimensionalCoherenceReliabilityThreshold = 0.1;
double MobileObject::m_DimensionalCoherenceProbabilityThreshold = 0.5;
double MobileObject::m_Maximal3DDimensionChangeSpeed = 244.0; //[cm/s]

std::map<ObjectType, ObjectSubtype> MobileObject::lastFoundSubtypeTemplate;

double MobileObject::m_classifThreshold = 0.0;
double MobileObject::m_maximalAlphaRotationSpeed = 19.6349540849362077417350480601; //(PI/2)[rad]/0.080[sec]

double *MobileObject::secDiffSequence = NULL;
double *MobileObject::coolingValue = NULL;
double *MobileObject::secDiffToCurrent = NULL;

double MobileObject::m_maxFocalDistance = 0.0;
double MobileObject::m_objectSizeForMaxReliability = 0.0;
double MobileObject::m_objectDimensionForMaxReliability = 0.0;
double MobileObject::m_objectDistanceForMaxReliability = 0.0;

double *MobileObject::g_postureMinw = NULL;
double *MobileObject::g_postureMinl = NULL;
double *MobileObject::g_postureMinh = NULL;
double *MobileObject::g_postureMaxw = NULL;
double *MobileObject::g_postureMaxl = NULL;
double *MobileObject::g_postureMaxh = NULL;
double *MobileObject::g_postureMeanw = NULL;
double *MobileObject::g_postureMeanl = NULL;
double *MobileObject::g_postureMeanh = NULL;
ObjectSubtype *MobileObject::g_posturesList = NULL;

double MobileObject::m_probabilityToEnsureMode = 0.8;
int MobileObject::m_2DLevelFrames = 3;

double *MobileObject::g_secDiffSequence;
double *MobileObject::g_coolingValue;
double *MobileObject::g_secDiffToCurrent;
tracking2DimensionalData *MobileObject::g_t2DDimData;
tracking2DSpatialData    *MobileObject::g_t2DSpatialData;
IncrementalExtraGeneralData *MobileObject::g_iGData;
IncrementalExtra2DData *MobileObject::g_i2DData;
Rectangle<int> *MobileObject::g_newBBoxesToAnalyze;
double *MobileObject::g_newVisualSupport;
DetectionProblemType *MobileObject::g_newDPFlags;
int MobileObject::g_currentBufferSize;
double MobileObject::zeroTolerance = 0.00001;
double MobileObject::m_minimalTolerance = 4;

MobileObject::MobileObject():best_type(UNKNOWN), best_subtype(ST_UNKNOWN), ensureMode(false), lastUnknown(true), best_index(-1),
                             numberOfFramesNotSeen(0), numberOfFramesSinceFirstTimeSeen(1), debug_data_flag(0),
                             blobHistory(m_blobsBufferSize), trajectory3D(m_trajectoryMaxSize), trajectory2D(m_trajectoryMaxSize), usedBlobs(NULL), involvedBlobs(NULL),
                             unknownsNumber(0), classificationAllowed(false) {

    s3dsToAnalyze = NULL;

    memset(&t3DDimData, 0, sizeof(tracking3DimensionalData));
    memset(&t2DDimData, 0, sizeof(tracking2DimensionalData));
    memset(&t3DSpatialData, 0, sizeof(tracking3DSpatialData));
    memset(&t2DSpatialData, 0, sizeof(tracking2DSpatialData));
    memset(&iGData, 0, sizeof(IncrementalExtraGeneralData));
    memset(&i2DData, 0, sizeof(IncrementalExtra2DData));
    memset(&i3DData, 0, sizeof(IncrementalExtra3DData));

    initMaps();
    numUsed = 0;
    unknownsNumber = 0;
    initUsedBlobs();

    s3dsToAnalyzeAllTypes = new Shape3DData *[m_blobsBufferSize*m_objectModelsNumber];
    PSumPerType           = new double[m_objectModelsNumber];
    lastKnownPerType      = new bool[m_objectModelsNumber];
    i3D                   = new info3D[m_objectModelsNumber];

    classifiedS3ds        = new bool[m_blobsBufferSize];
    foundS3ds             = new bool[m_blobsBufferSize];

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags               = new DetectionProblemType[m_blobsBufferSize];

    memset(s3dsToAnalyzeAllTypes, 0, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
    memset(i3D, 0, sizeof(info3D)*m_objectModelsNumber);
    memset(PSumPerType, 0, sizeof(double)*m_objectModelsNumber);
    memset(lastKnownPerType, 0, sizeof(bool)*m_objectModelsNumber);
    memset(bboxesToAnalyze, 0, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memset(visualSupport, 0, sizeof(double)*m_blobsBufferSize);
    memset(dpFlags, 0, sizeof(DetectionProblemType)*m_blobsBufferSize);
    memset(classifiedS3ds, false, sizeof(bool)*m_objectModelsNumber);
    memset(foundS3ds, false, sizeof(bool)*m_objectModelsNumber);
    P2D = 0.0;
    P3D = 0.0;
    PVC = 0.0;
    PV2DC = 0.0;
    R2D = 0.0;
    R3D = 0.0;
    RVC = 0.0;
    RV2DC = 0.0;
}

MobileObject::MobileObject(SpMobileObject to_copy):mobile_id(to_copy->getMobileId()), rmobile_id(to_copy->getRMobileId()), best_type(to_copy->getBestType()), best_subtype(to_copy->getBestSubType()),
                                                   ensureMode(to_copy->ensureMode), lastUnknown(to_copy->lastUnknown), best_index(to_copy->best_index),
                                                   numberOfFramesNotSeen(to_copy->getNumberOfFramesNotSeen()),
                                                   numberOfFramesSinceFirstTimeSeen(to_copy->getNumberOfFramesSinceFirstTimeSeen()+1), debug_data_flag(0),
                                                   blobHistory(m_blobsBufferSize), trajectory3D(m_trajectoryMaxSize), trajectory2D(m_trajectoryMaxSize),
                                                   usedBlobs(NULL), involvedBlobs(NULL),
                                                   unknownsNumber(to_copy->unknownsNumber), classificationAllowed(to_copy->classificationAllowed) {
    s3dsToAnalyze = NULL;

    blobHistory.copyBlobs(&(to_copy->blobHistory));
    trajectory3D.copyPoints(&(to_copy->trajectory3D));
    trajectory2D.copyPoints(&(to_copy->trajectory2D));

    if(ensureMode) {
        current_min_w_model = to_copy->current_min_w_model;
        current_max_w_model = to_copy->current_max_w_model;
        current_min_l_model = to_copy->current_min_l_model;
        current_max_l_model = to_copy->current_max_l_model;
        current_min_h_model = to_copy->current_min_h_model;
        current_max_h_model = to_copy->current_max_h_model;
        current_min_velocity_model = to_copy->current_min_velocity_model;
        current_max_velocity_model = to_copy->current_max_velocity_model;
    }

    lastFoundSubtype  = to_copy->lastFoundSubtype;
    memcpy(&t3DDimData, &(to_copy->t3DDimData), sizeof(tracking3DimensionalData));
    memcpy(&t2DDimData, &(to_copy->t2DDimData), sizeof(tracking2DimensionalData));
    memcpy(&t3DSpatialData, &(to_copy->t3DSpatialData), sizeof(tracking3DSpatialData));
    memcpy(&t2DSpatialData, &(to_copy->t2DSpatialData), sizeof(tracking2DSpatialData));
    memcpy(&iGData, &(to_copy->iGData), sizeof(IncrementalExtraGeneralData));
    memcpy(&i2DData, &(to_copy->i2DData), sizeof(IncrementalExtra2DData));
    memcpy(&i3DData, &(to_copy->i3DData), sizeof(IncrementalExtra3DData));

    initUsedBlobs();
    numUsed = 0;

    s3dsToAnalyzeAllTypes = new Shape3DData *[m_blobsBufferSize*m_objectModelsNumber];
    i3D                   = new info3D[m_objectModelsNumber];
    PSumPerType           = new double[m_objectModelsNumber];
    lastKnownPerType      = new bool[m_objectModelsNumber];

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags               = new DetectionProblemType[m_blobsBufferSize];

    memcpy(s3dsToAnalyzeAllTypes, to_copy->s3dsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
    memcpy(i3D, to_copy->i3D, sizeof(info3D)*m_objectModelsNumber);
    memcpy(PSumPerType, to_copy->PSumPerType, sizeof(double)*m_objectModelsNumber);
    memcpy(lastKnownPerType, to_copy->lastKnownPerType, sizeof(bool)*m_objectModelsNumber);
    memcpy(bboxesToAnalyze, to_copy->bboxesToAnalyze, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memcpy(visualSupport, to_copy->visualSupport, sizeof(double)*m_blobsBufferSize);
    memcpy(dpFlags, to_copy->dpFlags, sizeof(DetectionProblemType)*m_blobsBufferSize);

    classifiedS3ds = new bool[m_blobsBufferSize];
    foundS3ds      = new bool[m_blobsBufferSize];
    memset(classifiedS3ds, false, sizeof(bool)*m_objectModelsNumber);
    memset(foundS3ds, false, sizeof(bool)*m_objectModelsNumber);

    P2D = 0.0;
    P3D = 0.0;
    PVC = 0.0;
    PV2DC = 0.0;
    R2D = 0.0;
    R3D = 0.0;
    RVC = 0.0;
    RV2DC = 0.0;
}

void  MobileObject::initMaps() {
    lastFoundSubtype = lastFoundSubtypeTemplate;
}

MobileObject::~MobileObject() {
    blobHistory.clear();
    trajectory3D.clear();
    trajectory2D.clear();

    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(s3dsToAnalyzeAllTypes)
        delete[] s3dsToAnalyzeAllTypes;
    if(classifiedS3ds)
        delete[] classifiedS3ds;
    if(foundS3ds)
        delete[] foundS3ds;
    if(bboxesToAnalyze)
        delete[] bboxesToAnalyze;
    if(visualSupport)
        delete[] visualSupport;
    if(dpFlags)
        delete[] dpFlags;
    if(PSumPerType)
        delete[] PSumPerType;
    if(lastKnownPerType)
        delete[] lastKnownPerType;
    if(i3D)
        delete[] i3D;
}

void MobileObject::initUsedBlobs() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void MobileObject::initInvolvedBlobs() {
    if(involvedBlobs != NULL)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
    numInvolved = 0;
}


Shape3DData **MobileObject::getS3DBufferToAnalyze(ObjectType type) {
    return getS3DBufferToAnalyzeByIndex(objectModelMap[type]);
}

Shape3DData **MobileObject::getS3DBufferToAnalyzeByIndex(int index) {
    return s3dsToAnalyzeAllTypes + index*m_blobsBufferSize;
}

Shape3DData *MobileObject::getS3DToAnalyze(ObjectType type, int frame) {
    return getS3DToAnalyzeByIndex(objectModelMap[type], frame);
}

Shape3DData *MobileObject::getS3DToAnalyzeByIndex(int index, int frame) {
    return s3dsToAnalyzeAllTypes[index*m_blobsBufferSize + frame];
}

void MobileObject::setS3DToAnalyzeByIndex(int index, int frame, Shape3DData *s3d) {
    s3dsToAnalyzeAllTypes[index*m_blobsBufferSize + frame] = s3d;
}

void MobileObject::insertS3DToAnalyzeByIndex(int index, Shape3DData *s3d) {
    int type_position = index*m_blobsBufferSize;

    //If this condition holds, it means that 3D information was not extracted
    //from a blob and a Shape3DData *structure has been allocated for this purpose.
    //    if(dpFlags[m_blobsBufferSize - 1] & MM_NOT_3D_TRACKABLE_MASK
    //   && s3dsToAnalyzeAllTypes[type_position + m_blobsBufferSize - 1] != NULL)
    //  s3data_free(s3dsToAnalyzeAllTypes[type_position + m_blobsBufferSize - 1]);

    memmove(s3dsToAnalyzeAllTypes + type_position + 1,
            s3dsToAnalyzeAllTypes + type_position,
            sizeof(Shape3DData *)*(m_blobsBufferSize-1));
    s3dsToAnalyzeAllTypes[type_position] = s3d;
}


//New implementation of best type determination and update of frame data to be used (with coherency check for postural objects)
Shape3DData *MobileObject::getRightS3D(std::map<ObjectType, Shape3DData> *list, ObjectType type) {

    if(list == NULL)
        return NULL;

    if(list->size() == 0)
        return NULL;

    return &(*list)[type];
}

//Method for coherency check
//for one type
Shape3DData *MobileObject::generateBestLikelyS3D(Blob *blob, ObjectType type) {
    Shape3DData *bests3d;

    if(!rigidModel[type])
        initPostureLimits(type);

    bests3d =  m_rc->getMostLikelyDataFromMobile(blob, type);
    //bests3d = NULL;
    if(!rigidModel[type])
        freePostureLimits();

    if(bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold)
        S3D_3DBBOX(bests3d)->setReal3DPosition(&bests3d->position3D);

    return bests3d;
}


//Method for postural coherency check
//Implementation for initial 3D sequence generation
Shape3DData *MobileObject::getBestPostureCoherentS3DReclassifying(Blob *blob, std::map<ObjectType, Shape3DData> *normal,
                                                                              std::map<ObjectType, Shape3DData> *occ,
                                                                              ObjectType type) {

    //Get most coherent combination with respect to previous data, given the bounding box of current blob
    //and check if can be good enough to be used
    if(!rigidModel[type])
        initPostureLimits(type);

    BLOB_TYPE(blob) = type;
    Shape3DData *bests3d =  getMostCoherentDataFromMobileAndBBoxLimit(normal, occ, blob);

    if(!rigidModel[type])
        freePostureLimits();

    if(bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold)
        S3D_3DBBOX(bests3d)->setReal3DPosition(&bests3d->position3D);

    return bests3d;
}


void MobileObject::initPostureLimits(ObjectType type) {

    int i = objectModelMap[type], j;

    std::map<ObjectSubtype, int> *posturesMap = &objectSubModelMap[type];
    std::map<ObjectSubtype, int>::iterator postures_it;
    m_numberOfPostures = posturesMap->size();

    g_postureMinw = new double[m_numberOfPostures];
    g_postureMinl = new double[m_numberOfPostures];
    g_postureMinh = new double[m_numberOfPostures];
    g_postureMaxw = new double[m_numberOfPostures];
    g_postureMaxl = new double[m_numberOfPostures];
    g_postureMaxh = new double[m_numberOfPostures];
    g_postureMeanw = new double[m_numberOfPostures];
    g_postureMeanl = new double[m_numberOfPostures];
    g_postureMeanh = new double[m_numberOfPostures];
    g_posturesList = new ObjectSubtype[m_numberOfPostures];

    for(j=0, postures_it = posturesMap->begin(); j<m_numberOfPostures; j++, postures_it++) {
        g_postureMinw[j] = objectSubModelMinWidth[i][j];
        g_postureMeanw[j] = objectSubModelMeanWidth[i][j];
        g_postureMaxw[j] = objectSubModelMaxWidth[i][j];
        g_postureMinl[j] = objectSubModelMinLength[i][j];
        g_postureMeanl[j] = objectSubModelMeanLength[i][j];
        g_postureMaxl[j] = objectSubModelMaxLength[i][j];
        g_postureMinh[j]= objectSubModelMinHeight[i][j];
        g_postureMeanh[j] = objectSubModelMeanHeight[i][j];
        g_postureMaxh[j] = objectSubModelMaxHeight[i][j];

        g_posturesList[j] = (*postures_it).first;
    }

}

void MobileObject::freePostureLimits() {
    delete[] g_postureMinw;
    delete[] g_postureMinl;
    delete[] g_postureMinh;
    delete[] g_postureMaxw;
    delete[] g_postureMaxl;
    delete[] g_postureMaxh;
    delete[] g_postureMeanw;
    delete[] g_postureMeanl;
    delete[] g_postureMeanh;
    delete[] g_posturesList;
}

Shape3DData *MobileObject::getMostCoherentDataFromMobileAndBBoxLimit(std::map<ObjectType, Shape3DData> *normal,
                                                                     std::map<ObjectType, Shape3DData> *occ, Blob *blob) {
    return m_rc->getMostCoherentDataFromMobileAndBBoxLimit(this, normal, occ, blob);
}

//ACA!!!
//The solution to initial 3D generation is to implement a new function considering
//the current available data to guide the classification on finding the parallelepiped.
//  Shape3DData *MobileObject::getMostCoherentDataForMobile(Rectangle<int> *bbox, Blob *blob) {
//  return m_rc->getMostCoherentDataFromMobileAndBBoxLimit(this, normal, occ, blob);
//}


bool MobileObject::mobileOutOfScene() {
    int pixelTolerance = 3;
    double
        X = t2DSpatialData.X,
        Y = t2DSpatialData.Y,
        W_2 = t2DDimData.W / 2.0,
        H_2 = t2DDimData.H / 2.0;
    if(X + W_2 < pixelTolerance || X - W_2 > m_pSegmentation->width() - pixelTolerance || Y + H_2 < pixelTolerance || Y - H_2 > m_pSegmentation->height() - pixelTolerance )
        return true;

    if(ensureMode) {
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), t3DSpatialData.x, t3DSpatialData.y, 0, &X, &Y);
        if(X + W_2 < pixelTolerance || X - W_2 > m_pSegmentation->width() - pixelTolerance || Y + H_2 < pixelTolerance || Y - H_2 > m_pSegmentation->height() - pixelTolerance )
            return true;
    }
    return false;
}


void MobileObject::getCurrentBoundingBoxForMobile(Rectangle<int> *bbox) {
    if(bbox == NULL)
        return;

    if(currentBufferSize == 2) { //After first frame? Then copy the only blob in history
        memcpy(bbox, BLOB_BBOX(*this->blobHistory.begin()), sizeof(Rectangle<int>));
        return;
    }

    //Use 2D information because it represent just the visual evidence.
    //In terms of Visual Support just visible data is interesting.
    double tdiff = secDiffSequence[0];
    int
        W_2 = (int)( (t2DDimData.W + t2DDimData.VW * tdiff) / 2.0),
        H_2 = (int)( (t2DDimData.H + t2DDimData.VH * tdiff) / 2.0),
        X   = (int)(t2DSpatialData.X + t2DSpatialData.VX * tdiff),
        Y   = (int)(t2DSpatialData.Y + t2DSpatialData.VY * tdiff);

    RECT_XLEFT(bbox)   = X - W_2;
    RECT_XRIGHT(bbox)  = X + W_2;
    RECT_YTOP(bbox)    = Y - H_2;
    RECT_YBOTTOM(bbox) = Y + H_2;

    RECT_WIDTH(bbox)    = RECT_XRIGHT(bbox) - RECT_XLEFT(bbox) + 1;
    RECT_HEIGHT(bbox)   = RECT_YBOTTOM(bbox) - RECT_YTOP(bbox) + 1;

}

void MobileObject::getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox) {
    if(bbox == NULL)
        return;

    if(currentBufferSize == 2) { //After first frame? Then copy the only blob in history
        memcpy(bbox, BLOB_BBOX(*this->blobHistory.begin()), sizeof(Rectangle<int>));
        return;
    }

    //Use 2D information because it represent just the visual evidence.
    //In terms of Visual Support just visible data is interesting.
    double tdiff = secDiffSequence[0];
    int
        W_2 = (int)( (t2DDimData.W) / 2.0),
        H_2 = (int)( (t2DDimData.H) / 2.0),
        X   = (int)(t2DSpatialData.X + t2DSpatialData.VX * tdiff),
        Y   = (int)(t2DSpatialData.Y + t2DSpatialData.VY * tdiff);

    RECT_XLEFT(bbox)   = X - W_2;
    RECT_XRIGHT(bbox)  = X + W_2;
    RECT_YTOP(bbox)    = Y - H_2;
    RECT_YBOTTOM(bbox) = Y + H_2;

    RECT_WIDTH(bbox)    = RECT_XRIGHT(bbox) - RECT_XLEFT(bbox) + 1;
    RECT_HEIGHT(bbox)   = RECT_YBOTTOM(bbox) - RECT_YTOP(bbox) + 1;

}


void MobileObject::improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence) {
    int elimit, vlimit, variation, variation2, SD;

    //Initial values for improved bbox (without improvements :D).
    //ACA!!!! Cocina pa no perder!!!
    getCurrentBoundingBoxForMobileKeepingSize(estimatedBBox);
    memcpy(improvedBBox, estimatedBBox, sizeof(Rectangle<int>));

    //For horizontal variation
    if(RECT_XLEFT(estimatedBBox) > RECT_XLEFT(visualEvidence) && RECT_XRIGHT(estimatedBBox) > RECT_XRIGHT(visualEvidence)) { //Check if mobile position can be moved to the left
        SD = (int)(t2DSpatialData.SDX + t2DSpatialData.SDVX*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;
        vlimit = RECT_XLEFT(visualEvidence);
        elimit = RECT_XLEFT(estimatedBBox) - SD;
        variation = RECT_XLEFT(estimatedBBox) - std::max(vlimit, elimit);
        vlimit = RECT_XRIGHT(visualEvidence);
        elimit = RECT_XRIGHT(estimatedBBox) - SD;
        variation2 = RECT_XRIGHT(estimatedBBox) - std::max(vlimit, elimit);
        if(variation2 < variation) //Take minimal variation
            variation = variation2;
        RECT_XLEFT(improvedBBox)  -= variation;
        RECT_XRIGHT(improvedBBox) -= variation;
    } else if (RECT_XLEFT(estimatedBBox) < RECT_XLEFT(visualEvidence) && RECT_XRIGHT(estimatedBBox) < RECT_XRIGHT(visualEvidence)) { //Check if mobile position can be moved to the right
        SD = (int)(t2DSpatialData.SDX + t2DSpatialData.SDVX*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;
        vlimit = RECT_XLEFT(visualEvidence);
        elimit = RECT_XLEFT(estimatedBBox) + SD;
        variation = std::min(vlimit, elimit) - RECT_XLEFT(estimatedBBox);
        vlimit = RECT_XRIGHT(visualEvidence);
        elimit = RECT_XRIGHT(estimatedBBox) + SD;
        variation2 = std::min(vlimit, elimit) - RECT_XRIGHT(estimatedBBox);
        if(variation2 < variation) //Take minimal variation
            variation = variation2;
        RECT_XLEFT(improvedBBox)  += variation;
        RECT_XRIGHT(improvedBBox) += variation;
    }

    //For vertical variation
    if(RECT_YTOP(estimatedBBox) > RECT_YTOP(visualEvidence) && RECT_YBOTTOM(estimatedBBox) > RECT_YBOTTOM(visualEvidence)) { //Check if mobile position can be moved to top
        SD = (int)(t2DSpatialData.SDY + t2DSpatialData.SDVY*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;
        vlimit = RECT_YTOP(visualEvidence);
        elimit = RECT_YTOP(estimatedBBox) - SD;
        variation = RECT_YTOP(estimatedBBox) - std::max(vlimit, elimit);
        vlimit = RECT_YBOTTOM(visualEvidence);
        elimit = RECT_YBOTTOM(estimatedBBox) - SD;
        variation2 = RECT_YBOTTOM(estimatedBBox) - std::max(vlimit, elimit);
        if(variation2 < variation) //Take minimal variation
            variation = variation2;
        RECT_YTOP(improvedBBox)    -= variation;
        RECT_YBOTTOM(improvedBBox) -= variation;
    } else if (RECT_YTOP(estimatedBBox) < RECT_YTOP(visualEvidence) && RECT_YBOTTOM(estimatedBBox) < RECT_YBOTTOM(visualEvidence)) { //Check if mobile position can be moved to the right
        SD = (int)(t2DSpatialData.SDY + t2DSpatialData.SDVY*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;
        vlimit = RECT_YTOP(visualEvidence);
        elimit = RECT_YTOP(estimatedBBox) + SD;
        variation = std::min(vlimit, elimit) - RECT_YTOP(estimatedBBox);
        vlimit = RECT_YBOTTOM(visualEvidence);
        elimit = RECT_YBOTTOM(estimatedBBox) + SD;
        variation2 = std::min(vlimit, elimit) - RECT_YBOTTOM(estimatedBBox);
        if(variation2 < variation) //Take minimal variation
            variation = variation2;
        RECT_YTOP(improvedBBox)    += variation;
        RECT_YBOTTOM(improvedBBox) += variation;
    }

    RECT_WIDTH(improvedBBox)    = RECT_XRIGHT(improvedBBox) - RECT_XLEFT(improvedBBox) + 1;
    RECT_HEIGHT(improvedBBox)   = RECT_YBOTTOM(improvedBBox) - RECT_YTOP(improvedBBox) + 1;

    //Now check 2D W and H for improvement
    if(t2DDimData.SDVW > fabs(t2DDimData.VW)) { //If velocity has significative value
        int vlimit2;
        elimit = (int)(t2DDimData.SDW + t2DDimData.SDVW*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;

        //If both limits are exterior
        if(RECT_XLEFT(improvedBBox) < RECT_XLEFT(visualEvidence) && RECT_XRIGHT(improvedBBox) > RECT_XRIGHT(visualEvidence)) {
            vlimit = RECT_XLEFT(visualEvidence) - RECT_XLEFT(improvedBBox);
            vlimit2 = RECT_XRIGHT(improvedBBox) - RECT_XRIGHT(visualEvidence);
            if(vlimit < vlimit2) {
                variation = std::min(vlimit, elimit/2);
                RECT_XLEFT(improvedBBox)  += variation;
                RECT_XRIGHT(improvedBBox) -= variation;
                elimit -= 2*variation;
                vlimit2 -= variation;
                if(elimit > 0 && vlimit2 > 0) {
                    variation = std::min(vlimit2, elimit);
                    RECT_XRIGHT(improvedBBox) -= variation;
                }
                RECT_WIDTH(improvedBBox) = RECT_XRIGHT(improvedBBox) - RECT_XLEFT(improvedBBox) + 1;
            } else {
                variation = std::min(vlimit2, elimit/2);
                RECT_XLEFT(improvedBBox)  += variation;
                RECT_XRIGHT(improvedBBox) -= variation;
                elimit -= 2*variation;
                vlimit -= variation;
                if(elimit > 0 && vlimit > 0) {
                    variation = std::min(vlimit, elimit);
                    RECT_XLEFT(improvedBBox)  += variation;
                }
                RECT_WIDTH(improvedBBox) = RECT_XRIGHT(improvedBBox) - RECT_XLEFT(improvedBBox) + 1;
            }
        } else if(RECT_XLEFT(improvedBBox) < RECT_XLEFT(visualEvidence) && RECT_XRIGHT(improvedBBox) <= RECT_XRIGHT(visualEvidence)) {
            vlimit = RECT_XLEFT(visualEvidence) - RECT_XLEFT(improvedBBox);
            variation = std::min(vlimit, elimit);
            RECT_XLEFT(improvedBBox)  += variation;
            RECT_WIDTH(improvedBBox) = RECT_XRIGHT(improvedBBox) - RECT_XLEFT(improvedBBox) + 1;
        } else if(RECT_XLEFT(improvedBBox) >= RECT_XLEFT(visualEvidence) && RECT_XRIGHT(improvedBBox) > RECT_XRIGHT(visualEvidence)) {
            vlimit = RECT_XRIGHT(improvedBBox) - RECT_XRIGHT(visualEvidence);
            variation = std::min(vlimit, elimit);
            RECT_XRIGHT(improvedBBox) -= variation;
            RECT_WIDTH(improvedBBox) = RECT_XRIGHT(improvedBBox) - RECT_XLEFT(improvedBBox) + 1;
        }
    }

    if(t2DDimData.SDVH > fabs(t2DDimData.VH)) { //If velocity has significative value
        int vlimit2;
        elimit = (int)(t2DDimData.SDH + t2DDimData.SDVH*secDiffSequence[0]) + ReliabilityTracker::acceptedPixelError;

        //If both limits are exterior
        if(RECT_YTOP(improvedBBox) < RECT_YTOP(visualEvidence) && RECT_YBOTTOM(improvedBBox) > RECT_YBOTTOM(visualEvidence)) {
            vlimit = RECT_YTOP(visualEvidence) - RECT_YTOP(improvedBBox);
            vlimit2 = RECT_YBOTTOM(improvedBBox) - RECT_YBOTTOM(visualEvidence);
            if(vlimit < vlimit2) {
                variation = std::min(vlimit, elimit/2);
                RECT_YTOP(improvedBBox)  += variation;
                RECT_YBOTTOM(improvedBBox) -= variation;
                elimit -= 2*variation;
                vlimit2 -= variation;
                if(elimit > 0 && vlimit2 > 0) {
                    variation = std::min(vlimit2, elimit);
                    RECT_YBOTTOM(improvedBBox) -= variation;
                }
                RECT_HEIGHT(improvedBBox) = RECT_YBOTTOM(improvedBBox) - RECT_YTOP(improvedBBox) + 1;
            } else {
                variation = std::min(vlimit2, elimit/2);
                RECT_YTOP(improvedBBox)  += variation;
                RECT_YBOTTOM(improvedBBox) -= variation;
                elimit -= 2*variation;
                vlimit -= variation;
                if(elimit > 0 && vlimit > 0) {
                    variation = std::min(vlimit, elimit);
                    RECT_YTOP(improvedBBox)  += variation;
                }
                RECT_HEIGHT(improvedBBox) = RECT_YBOTTOM(improvedBBox) - RECT_YTOP(improvedBBox) + 1;
            }
        } else if(RECT_YTOP(improvedBBox) < RECT_YTOP(visualEvidence) && RECT_YBOTTOM(improvedBBox) <= RECT_YBOTTOM(visualEvidence)) {
            vlimit = RECT_YTOP(visualEvidence) - RECT_YTOP(improvedBBox);
            variation = std::min(vlimit, elimit);
            RECT_YTOP(improvedBBox)  += variation;
            RECT_HEIGHT(improvedBBox) = RECT_YBOTTOM(improvedBBox) - RECT_YTOP(improvedBBox) + 1;
        } else if(RECT_YTOP(improvedBBox) >= RECT_YTOP(visualEvidence) && RECT_YBOTTOM(improvedBBox) > RECT_YBOTTOM(visualEvidence)) {
            vlimit = RECT_YBOTTOM(improvedBBox) - RECT_YBOTTOM(visualEvidence);
            variation = std::min(vlimit, elimit);
            RECT_YBOTTOM(improvedBBox) -= variation;
            RECT_HEIGHT(improvedBBox) = RECT_YBOTTOM(improvedBBox) - RECT_YTOP(improvedBBox) + 1;
        }
    }
}


void MobileObject::generateFirstClassifiedSequence() {
    bool firstClassifiedNotFound = true;
    //Start from oldest blob in buffer
    std::deque<SpBlob>::iterator first_classified_it = blobHistory.begin();
    Blob *current_blob;    
    int i, frame_index = currentBufferSize - 1;
    double bkSecDiffSequence[m_blobsBufferSize];
    double bkCoolingValue[m_blobsBufferSize];
    double bkSecDiffToCurrent[m_blobsBufferSize];
    //Encapsulated tracking data
    tracking2DimensionalData bk_t2DDimData;
    tracking2DSpatialData    bk_t2DSpatialData;
    //Data for incremental values update
    IncrementalExtraGeneralData bk_iGData;
    IncrementalExtra2DData bk_i2DData;
    Rectangle<int> bk_BBoxesToAnalyze[m_blobsBufferSize];
    double bk_VisualSupport[m_blobsBufferSize];
    DetectionProblemType bk_DPFlags[m_blobsBufferSize];

    //Store copies of the original information, to recover it
    //when 3D initilialisation process is finished
    memcpy(bkSecDiffSequence,  secDiffSequence, m_blobsBufferSize*sizeof(double));
    memcpy(bkCoolingValue,     coolingValue,    m_blobsBufferSize*sizeof(double));
    memcpy(bkSecDiffToCurrent, secDiffToCurrent, m_blobsBufferSize*sizeof(double));

    memcpy(&bk_t2DDimData, &t2DDimData, sizeof(tracking2DimensionalData));
    memcpy(&bk_t2DSpatialData, &t2DSpatialData, sizeof(tracking2DSpatialData));
    memcpy(&bk_iGData, &iGData, sizeof(IncrementalExtraGeneralData));
    memcpy(&bk_i2DData, &i2DData, sizeof(IncrementalExtra2DData));

    memcpy(bk_BBoxesToAnalyze, bboxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
    memcpy(bk_VisualSupport, visualSupport, m_blobsBufferSize*sizeof(double));
    memcpy(bk_DPFlags, dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));

    g_currentBufferSize = currentBufferSize;

    //Initialize 2D data, and bufferSize as it was the first frame
    //to be processed
    memset(&t2DDimData, 0, sizeof(tracking2DimensionalData));
    memset(&t2DSpatialData, 0, sizeof(tracking2DSpatialData));
    memset(&iGData, 0, sizeof(IncrementalExtraGeneralData));
    memset(&i2DData, 0, sizeof(IncrementalExtra2DData));

    best_type = UNKNOWN;
    best_index = 0;

    currentBufferSize = 1;

    numberOfClassifiedS3ds = numberOfFoundS3ds = 0;
    RKnownSolutions  = RVKnownSolutions = 0.0;

    do {
        current_blob = &(**first_classified_it);

        //If blob have not yet been classified, do it
        if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
            m_rc->setBlob3DFacts(current_blob, m_pSegmentation);

        if(BLOB_P(current_blob) >= m_classifThreshold) {
            firstClassifiedNotFound = false;
        } else { //If current blob is UNKNOWN, prepare
                 //information to check the next blob

            //Set information for next frame, from bk-ed information
            if(currentBufferSize == 1)
                secDiffToCurrent[1] = secDiffSequence[0] = bkSecDiffSequence[frame_index];
            else {
                //Update time
                memmove(secDiffSequence  + 1, secDiffSequence,  sizeof(double)*(m_blobsBufferSize - 1));
                memmove(secDiffToCurrent + 1, secDiffToCurrent, sizeof(double)*(m_blobsBufferSize - 1));
                secDiffToCurrent[1] = secDiffSequence[0] = bkSecDiffSequence[frame_index];
                for(i=2; i<currentBufferSize; i++)
                    secDiffToCurrent[i] += secDiffSequence[0];
                for(i=1; i<currentBufferSize; i++)
                    coolingValue[i] = coolingFunction(secDiffToCurrent[i]);
            }

            //Insert the current information
            int j, n = BLOB_NORMAL_3DDATA(current_blob)->size();
            for(j=0; j < n; j++)
                insertS3DToAnalyzeByIndex(j, NULL);

            memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
            memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);
            memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
            memcpy(&(bboxesToAnalyze[0]), &(bk_BBoxesToAnalyze[frame_index]), sizeof(Rectangle<int>));
            memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
            visualSupport[0] = bk_VisualSupport[frame_index];
            memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
            memcpy(&(dpFlags[0]), &(bk_DPFlags[frame_index]), sizeof(DetectionProblemType));

            numberOfFoundS3ds = 0;
            for(i=0; i<currentBufferSize; i++) {
                if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                    foundS3ds[i] = true;
                    numberOfFoundS3ds++;
                }
            }

            RFoundSolutions  = (double)numberOfFoundS3ds / (double)currentBufferSize;
            RVFoundSolutions = numberOfFoundS3ds < 2 ? 0.0 : (double)(numberOfFoundS3ds - 1)/(double)(currentBufferSize - 1);

            //Update 2D information
            m_rc->setDPForBlob(current_blob, m_pSegmentation);
            currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);
            incrementalUpdateCooling(currentBufferSize);
            incrementalUpdate2DDimensions(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
            incrementalUpdate2DPosition(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);

            frame_index--;
            currentBufferSize++;
            first_classified_it++;
        }
    } while(firstClassifiedNotFound && first_classified_it != blobHistory.end());

    if(firstClassifiedNotFound) { //In this case the object is really UNKNOWN
        best_type = UNKNOWN;
        best_subtype = ST_UNKNOWN;
        best_index = 0;
        current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
        current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;
    } else {
        //Set pointers to original information
        g_secDiffSequence    = bkSecDiffSequence;
        g_coolingValue       = bkCoolingValue;
        g_secDiffToCurrent   = bkSecDiffToCurrent;
        g_t2DDimData         = &bk_t2DDimData;
        g_t2DSpatialData     = &bk_t2DSpatialData;
        g_iGData             = &bk_iGData;
        g_i2DData            = &bk_i2DData;
        g_newBBoxesToAnalyze = bk_BBoxesToAnalyze;
        g_newVisualSupport   = bk_VisualSupport;
        g_newDPFlags  = bk_DPFlags;

        //Check solutions starting from the first classified blob
        setBestS3DSequence(first_classified_it, frame_index, currentBufferSize);
    }

    //Restore initial values
    currentBufferSize = g_currentBufferSize;

    memcpy(secDiffSequence,  bkSecDiffSequence,  m_blobsBufferSize*sizeof(double));
    memcpy(coolingValue,     bkCoolingValue,     m_blobsBufferSize*sizeof(double));
    memcpy(secDiffToCurrent, bkSecDiffToCurrent, m_blobsBufferSize*sizeof(double));

    memcpy(&t2DDimData, &bk_t2DDimData, sizeof(tracking2DimensionalData));
    memcpy(&t2DSpatialData, &bk_t2DSpatialData, sizeof(tracking2DSpatialData));
    memcpy(&i2DData, &bk_i2DData, sizeof(IncrementalExtra2DData));

    memcpy(bboxesToAnalyze, bk_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
    memcpy(visualSupport, bk_VisualSupport, m_blobsBufferSize*sizeof(double));
    memcpy(dpFlags, bk_DPFlags, m_blobsBufferSize*sizeof(DetectionProblemType));

    //Update 2D information

    if(best_type != UNKNOWN)
        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(best_index);
    else {
        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(0);
        //If no valid sequence found, restore the general data
        memcpy(&iGData, &bk_iGData, sizeof(IncrementalExtraGeneralData));
    }

    numberOfClassifiedS3ds = numberOfFoundS3ds = 0;
    memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
    memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);
    //Set the classification flags for data
    for(i = 0; i < currentBufferSize; i++) {
        if(    s3dsToAnalyze[i] != NULL
            && S3D_P(s3dsToAnalyze[i]) >= m_classifThreshold
            && !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            classifiedS3ds[i] = true;
            numberOfClassifiedS3ds++;
        } else {
            unknownsNumber++;
        }

        if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            foundS3ds[i] = true;
            numberOfFoundS3ds++;
        }
    }


    RKnownSolutions  = (double)numberOfClassifiedS3ds/(double)currentBufferSize;
    RFoundSolutions  = (double)numberOfFoundS3ds     /(double)currentBufferSize;

    RVKnownSolutions = numberOfClassifiedS3ds < 2 ? 0.0 : (double)(numberOfClassifiedS3ds - 1)/(double)(currentBufferSize-1);
    RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds      - 1)/(double)(currentBufferSize-1);

    if(numberOfFramesSinceFirstTimeSeen*(1 - m_knownSolutionThreshold) - unknownsNumber >= 0 && best_type != UNKNOWN) { //More than the m_knownSolutionThreshold of data is not unknown type
        if(!rigidModel[best_type]) {
            i = objectModelMap[best_type];
            int j = objectSubModelMap[best_type][best_subtype];

            current_min_w_model        = objectSubModelMinWidth[i][j];
            current_max_w_model        = objectSubModelMaxWidth[i][j];
            current_min_l_model        = objectSubModelMinLength[i][j];
            current_max_l_model        = objectSubModelMaxLength[i][j];
            current_min_h_model        = objectSubModelMinHeight[i][j];
            current_max_h_model        = objectSubModelMaxHeight[i][j];
            current_min_velocity_model = objectSubModelMinVelocity[i][j];
            current_max_velocity_model = objectSubModelMaxVelocity[i][j];
        } else {
            best_subtype = ST_UNKNOWN;
            i = objectModelMap[best_type];

            current_min_w_model       = objectModelMinWidth[i];
            current_max_w_model       = objectModelMaxWidth[i];
            current_min_l_model       = objectModelMinLength[i];
            current_max_l_model       = objectModelMaxLength[i];
            current_min_h_model       = objectModelMinHeight[i];
            current_max_h_model       = objectModelMaxHeight[i];
            current_min_velocity_model = objectModelMinVelocity[i];
            current_max_velocity_model = objectModelMaxVelocity[i];
        }
    } else {
        best_type = UNKNOWN;
        best_subtype = ST_UNKNOWN;
        current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
        current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;

        current_blob = &(*blobHistory.back());
        DetectionProblemType not_visible_mask     = (DetectionProblemType) (BLOB_DP_TYPE(current_blob) & MM_NOT_VISIBLE_MASK);
        if(not_visible_mask)
            unknownsNumber++;
        else
            setNumberOfFramesNotSeen(0);
        m_rc->setDPForBlob(current_blob, m_pSegmentation);
        currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);
        //If not classified, the previous data is considered, then the last blob info is not yet included
        incrementalUpdateCooling(currentBufferSize);
        P3D = R3D = PVC = RVC = 0.0;
    }

    //Update 2D information with the last frame
    incrementalUpdate2DDimensions(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
    incrementalUpdate2DPosition(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);

}

void MobileObject::setBestS3DSequence(std::deque<SpBlob>::iterator first_blob_it, int frame_index, int newBufferSize) {
    //Determine best for each participating type and update probability in time
    ObjectType local_best_type;
    ObjectSubtype local_best_subtype;
    int local_best_index;

    Blob *current_blob;
    std::map<ObjectType, Shape3DData> *occ, *normal;
    bool occlusion;
    double bestP = - 1.0;
    std::map<ObjectType, int>::iterator modelMap_it;
    int current_index;
    ObjectType current_type;
    int i, j, k, local_frame_index = frame_index, index;
    bool foundSolution = false;
    Shape3DData *bestS3Ds[m_blobsBufferSize];
    struct info3D bestInfo3D;
    double
      initialSecDiffSequence[m_blobsBufferSize],
      initialCoolingValue[m_blobsBufferSize],
      initialSecDiffToCurrent[m_blobsBufferSize];
    Shape3DData *initialS3DsToAnalyzeAllTypes[m_blobsBufferSize*m_objectModelsNumber];
    Shape3DData bestS3DsAllTypes[m_blobsBufferSize*m_objectModelsNumber];
    bool notNullS3DsAllTypes[m_blobsBufferSize*m_objectModelsNumber];
    Shape3DData *current_S3D_list;
    bool *current_bool_list;
    tracking2DimensionalData initial_t2DDimData;
    tracking2DSpatialData    initial_t2DSpatialData;
    IncrementalExtraGeneralData initial_iGData;
    IncrementalExtra2DData initial_i2DData;
    Rectangle<int> initial_BBoxesToAnalyze[m_blobsBufferSize];
    double initial_VisualSupport[m_blobsBufferSize];
    DetectionProblemType initial_dpFlags[m_blobsBufferSize];
    Trajectory bestTrajectory3D(m_trajectoryMaxSize);

    memset(bestS3Ds, 0, m_blobsBufferSize*sizeof(Shape3DData *));
    memset(bestS3DsAllTypes, 0, m_blobsBufferSize*m_objectModelsNumber*sizeof(Shape3DData));
    memset(notNullS3DsAllTypes, 0, m_blobsBufferSize*m_objectModelsNumber*sizeof(bool));


    //Save data from starting point
    memcpy(initialSecDiffSequence,  secDiffSequence, m_blobsBufferSize*sizeof(double));
    memcpy(initialCoolingValue,     coolingValue,    m_blobsBufferSize*sizeof(double));
    memcpy(initialSecDiffToCurrent, secDiffToCurrent, m_blobsBufferSize*sizeof(double));
    memcpy(initialS3DsToAnalyzeAllTypes, s3dsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
    memcpy(&initial_t2DDimData, &t2DDimData, sizeof(tracking2DimensionalData));
    memcpy(&initial_t2DSpatialData, &t2DSpatialData, sizeof(tracking2DSpatialData));
    memcpy(&initial_iGData, &iGData, sizeof(IncrementalExtraGeneralData));
    memcpy(&initial_i2DData, &i2DData, sizeof(IncrementalExtra2DData));
    memcpy(initial_BBoxesToAnalyze, bboxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
    memcpy(initial_VisualSupport, visualSupport, m_blobsBufferSize*sizeof(double));
    memcpy(initial_dpFlags, dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));

    do {
        current_blob = &(**first_blob_it);
        //DetectionProblemType not_visible_mask      = (DetectionProblemType) (BLOB_DP_TYPE(current_blob) & MM_NOT_VISIBLE_MASK);

        //Classify currently initial blob if this is not done yet
        //This happens when previous starting frame did not give
        //any coherent/classifiable solution
        if(local_frame_index != frame_index && BLOB_NORMAL_3DDATA(current_blob) == NULL) //If they are equal, the blob has been already classified in previous function
            m_rc->setBlob3DFacts(current_blob, m_pSegmentation);

        normal = BLOB_NORMAL_3DDATA(current_blob);
        occ = BLOB_OCC_3DDATA(current_blob);
        occlusion = (occ == NULL) ? false : true;
        Shape3DData *normal_s3d, *occ_s3d = NULL;
        std::map<ObjectType, Shape3DData>::iterator n_it, n_end = normal->end();
        //For each best valid solution in blob for normal and occlusion solutions
        //check if a coherent sequence exist.
        int m, nummod;
        for(nummod = 0, n_it = normal->begin(); n_it != n_end && nummod < m_objectModelsNumber; nummod++, n_it++) {
            current_type = n_it->first;
            m = objectModelMap[current_type];
            current_index = m;

            s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(current_index);

            normal_s3d = &n_it->second;
            if(occlusion)
                occ_s3d = getRightS3D(BLOB_OCC_3DDATA(current_blob), current_type);

            //Check normal solution
            if(S3D_P(normal_s3d) >= m_classifThreshold) {
                if(rigidModel[current_type]) { //If analysed model is a rigid type...
                    //Initialize data to current blob
                    memcpy(secDiffSequence, initialSecDiffSequence,  m_blobsBufferSize*sizeof(double));
                    memcpy(coolingValue,   initialCoolingValue,      m_blobsBufferSize*sizeof(double));
                    memcpy(secDiffToCurrent, initialSecDiffToCurrent, m_blobsBufferSize*sizeof(double));
                    memcpy(s3dsToAnalyzeAllTypes, initialS3DsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
                    memcpy(&t2DDimData, &initial_t2DDimData, sizeof(tracking2DimensionalData));
                    memcpy(&t2DSpatialData, &initial_t2DSpatialData, sizeof(tracking2DSpatialData));
                    memcpy(&iGData, &initial_iGData, sizeof(IncrementalExtraGeneralData));
                    memcpy(&i2DData, &initial_i2DData, sizeof(IncrementalExtra2DData));
                    memcpy(bboxesToAnalyze, initial_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
                    memcpy(visualSupport, initial_VisualSupport, m_blobsBufferSize*sizeof(double));
                    memcpy(dpFlags, initial_dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));
                    //Reset 3D information
                    memset(&t3DDimData, 0, sizeof(tracking3DimensionalData));
                    memset(&t3DSpatialData, 0, sizeof(tracking3DSpatialData));
                    memset(&i3DData, 0, sizeof(IncrementalExtra3DData));
                    trajectory3D.clear();

                    i = objectModelMap[current_type];

                    current_min_w_model       = objectModelMinWidth[i];
                    current_max_w_model       = objectModelMaxWidth[i];
                    current_min_l_model       = objectModelMinLength[i];
                    current_max_l_model       = objectModelMaxLength[i];
                    current_min_h_model       = objectModelMinHeight[i];
                    current_max_h_model       = objectModelMaxHeight[i];
                    current_min_velocity_model = objectModelMinVelocity[i];
                    current_max_velocity_model = objectModelMaxVelocity[i];

                    generateMostCoherentS3DSequence(first_blob_it, normal_s3d, newBufferSize,
                                                local_frame_index, current_type, current_index);

                    //store 3D data
                    memcpy(&(i3D[current_index].iGData), &iGData, sizeof(IncrementalExtraGeneralData));
                    memcpy(&(i3D[current_index].i3DData), &i3DData, sizeof(IncrementalExtra3DData));
                    memcpy(&(i3D[current_index].t3DDimData), &t3DDimData, sizeof(tracking3DimensionalData));
                    memcpy(&(i3D[current_index].t3DSpatialData), &t3DSpatialData, sizeof(tracking3DSpatialData));

                    //update the best one
                    if(mobile3DVelocityCoherenceIsAcceptable() && mobile3DCoherenceIsAcceptable()) {
                        foundSolution = true;

                        current_S3D_list = bestS3DsAllTypes + current_index*m_blobsBufferSize;
                        current_bool_list = notNullS3DsAllTypes + current_index*m_blobsBufferSize;
                        for(index = 0; index<this->g_currentBufferSize; index++) {
                            if(s3dsToAnalyze[index] != NULL) {
                                memcpy(current_S3D_list + index, s3dsToAnalyze[index], sizeof(Shape3DData));
                                *(current_bool_list + index) = true;
                            }
                        }

                        if(P > bestP) {
                            bestP = P;
                            for(index = 0; index<this->g_currentBufferSize; index++)
                                if(*(current_bool_list + index) == true)
                                    bestS3Ds[index] = current_S3D_list + index;
                                else
                                    bestS3Ds[index] = NULL;
                            local_best_type = current_type;
                            local_best_subtype = ST_UNKNOWN;
                            local_best_index = current_index;

                            memcpy(&(bestInfo3D.iGData), &(i3D[current_index].iGData), sizeof(IncrementalExtraGeneralData));
                            memcpy(&(bestInfo3D.i3DData), &(i3D[current_index].i3DData), sizeof(IncrementalExtra3DData));
                            memcpy(&(bestInfo3D.t3DDimData), &(i3D[current_index].t3DDimData), sizeof(tracking3DimensionalData));
                            memcpy(&(bestInfo3D.t3DSpatialData), &(i3D[current_index].t3DSpatialData), sizeof(tracking3DSpatialData));
                            bestTrajectory3D = trajectory3D;
                        }
                    }
                } else {//For postural models postural change coherency must be checked
                    std::map<ObjectSubtype, int> *posturesMap = &objectSubModelMap[current_type];
                    std::map<ObjectSubtype, int>::iterator postures_it;
                    std::map<ObjectSubtype, Shape3DData> *postures = S3D_SUBTYPES_LIST(normal_s3d);
                    std::map<ObjectSubtype, Shape3DData>::iterator p_s3d_it, p_s3d_it_end = postures->end();
                    Shape3DData *current_posture, *bestS3DsForPosture[m_blobsBufferSize];
                    ObjectSubtype current_subtype, local_subbest;
                    double bestPForPosture = - 1.0;
                    bool foundPostureSolution = false;
                    Trajectory localBestTrajectory3D(m_trajectoryMaxSize);
                    memset(bestS3DsForPosture, 0, m_blobsBufferSize*sizeof(Shape3DData *));

                    for(j=0, postures_it = posturesMap->begin(), p_s3d_it = postures->begin();
                        p_s3d_it != p_s3d_it_end; j++, postures_it++, p_s3d_it++) {
                        current_subtype = (*postures_it).first;
                        current_posture = &(*p_s3d_it).second;
                        if(S3D_P(current_posture) >= m_classifThreshold) {
                            //Initialize data
                            memcpy(secDiffSequence, initialSecDiffSequence,  m_blobsBufferSize*sizeof(double));
                            memcpy(coolingValue,   initialCoolingValue,      m_blobsBufferSize*sizeof(double));
                            memcpy(secDiffToCurrent, initialSecDiffToCurrent, m_blobsBufferSize*sizeof(double));
                            memcpy(s3dsToAnalyzeAllTypes, initialS3DsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
                            memcpy(&t2DDimData, &initial_t2DDimData, sizeof(tracking2DimensionalData));
                            memcpy(&t2DSpatialData, &initial_t2DSpatialData, sizeof(tracking2DSpatialData));
                            memcpy(&iGData, &initial_iGData, sizeof(IncrementalExtraGeneralData));
                            memcpy(&i2DData, &initial_i2DData, sizeof(IncrementalExtra2DData));
                            memcpy(bboxesToAnalyze, initial_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
                            memcpy(visualSupport, initial_VisualSupport, m_blobsBufferSize*sizeof(double));
                            memcpy(dpFlags, initial_dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));
                            //Reset 3D information
                            memset(&t3DDimData, 0, sizeof(tracking3DimensionalData));
                            memset(&t3DSpatialData, 0, sizeof(tracking3DSpatialData));
                            memset(&i3DData, 0, sizeof(IncrementalExtra3DData));
                            trajectory3D.clear();

                            i = objectModelMap[current_type];
                            k = objectSubModelMap[current_type][current_subtype];
                            current_min_w_model        = objectSubModelMinWidth[i][k];
                            current_max_w_model        = objectSubModelMaxWidth[i][k];
                            current_min_l_model        = objectSubModelMinLength[i][k];
                            current_max_l_model        = objectSubModelMaxLength[i][k];
                            current_min_h_model        = objectSubModelMinHeight[i][k];
                            current_max_h_model        = objectSubModelMaxHeight[i][k];
                            current_min_velocity_model = objectSubModelMinVelocity[i][k];
                            current_max_velocity_model = objectSubModelMaxVelocity[i][k];

                            generateMostCoherentS3DSequence(first_blob_it, current_posture, newBufferSize,
                                                            local_frame_index, current_type, current_index);

                            if(mobile3DVelocityCoherenceIsAcceptable() && mobile3DCoherenceIsAcceptable()) {
                                foundPostureSolution = true;
                                if(P > bestPForPosture) {
                                    bestPForPosture = P;
                                    memcpy(bestS3DsForPosture, s3dsToAnalyze, sizeof(Shape3DData *)*m_blobsBufferSize);
                                    localBestTrajectory3D = trajectory3D;
                                    local_subbest = current_subtype;
                                    memcpy(&(i3D[current_index].iGData), &iGData, sizeof(IncrementalExtraGeneralData));
                                    memcpy(&(i3D[current_index].i3DData), &i3DData, sizeof(IncrementalExtra3DData));
                                    memcpy(&(i3D[current_index].t3DDimData), &t3DDimData, sizeof(tracking3DimensionalData));
                                    memcpy(&(i3D[current_index].t3DSpatialData), &t3DSpatialData, sizeof(tracking3DSpatialData));
                                }
                            }
                        }                        
                    }

                    if(foundPostureSolution && bestPForPosture > bestP) {
                        bestP = bestPForPosture;
                        foundSolution = true;

                        current_S3D_list = bestS3DsAllTypes + current_index*m_blobsBufferSize;
                        current_bool_list = notNullS3DsAllTypes + current_index*m_blobsBufferSize;
                        for(index = 0; index<this->g_currentBufferSize; index++) {
                            if(bestS3DsForPosture[index] != NULL) {
                                memcpy(current_S3D_list + index, bestS3DsForPosture[index], sizeof(Shape3DData));
                                *(current_bool_list + index) = true;
                            }
                        }

                        for(index = 0; index<this->g_currentBufferSize; index++)
                            if(*(current_bool_list + index) == true)
                                bestS3Ds[index] = current_S3D_list + index;
                            else
                                bestS3Ds[index] = NULL;

                        local_best_type = current_type;
                        local_best_index = current_index;
                        local_best_subtype = local_subbest;
                        memcpy(&(bestInfo3D.iGData), &(i3D[current_index].iGData), sizeof(IncrementalExtraGeneralData));
                        memcpy(&(bestInfo3D.i3DData), &(i3D[current_index].i3DData), sizeof(IncrementalExtra3DData));
                        memcpy(&(bestInfo3D.t3DDimData), &(i3D[current_index].t3DDimData), sizeof(tracking3DimensionalData));
                        memcpy(&(bestInfo3D.t3DSpatialData), &(i3D[current_index].t3DSpatialData), sizeof(tracking3DSpatialData));
                        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(current_index);
                        memcpy(s3dsToAnalyze, bestS3Ds, sizeof(Shape3DData *)*m_blobsBufferSize);
                        bestTrajectory3D = localBestTrajectory3D;
                    }
                }
            }



            //Check occlusion solution if it exists
            if(occ != NULL && S3D_P(occ_s3d) >= m_classifThreshold) {
                if(rigidModel[current_type]) { //If analysed model is a rigid type...
                //Initialize data
                    memcpy(secDiffSequence, initialSecDiffSequence,  m_blobsBufferSize*sizeof(double));
                    memcpy(coolingValue,   initialCoolingValue,      m_blobsBufferSize*sizeof(double));
                    memcpy(secDiffToCurrent, initialSecDiffToCurrent, m_blobsBufferSize*sizeof(double));
                    memcpy(s3dsToAnalyzeAllTypes, initialS3DsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
                    memcpy(&t2DDimData, &initial_t2DDimData, sizeof(tracking2DimensionalData));
                    memcpy(&t2DSpatialData, &initial_t2DSpatialData, sizeof(tracking2DSpatialData));
                    memcpy(&iGData, &initial_iGData, sizeof(IncrementalExtraGeneralData));
                    memcpy(&i2DData, &initial_i2DData, sizeof(IncrementalExtra2DData));
                    memcpy(bboxesToAnalyze, initial_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
                    memcpy(visualSupport, initial_VisualSupport, m_blobsBufferSize*sizeof(double));
                    memcpy(dpFlags, initial_dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));
                    //Reset 3D information
                    memset(&t3DDimData, 0, sizeof(tracking3DimensionalData));
                    memset(&t3DSpatialData, 0, sizeof(tracking3DSpatialData));
                    memset(&i3DData, 0, sizeof(IncrementalExtra3DData));

                    i = objectModelMap[current_type];
                    current_min_w_model       = objectModelMinWidth[i];
                    current_max_w_model       = objectModelMaxWidth[i];
                    current_min_l_model       = objectModelMinLength[i];
                    current_max_l_model       = objectModelMaxLength[i];
                    current_min_h_model       = objectModelMinHeight[i];
                    current_max_h_model       = objectModelMaxHeight[i];
                    current_min_velocity_model = objectModelMinVelocity[i];
                    current_max_velocity_model = objectModelMaxVelocity[i];

                    generateMostCoherentS3DSequence(first_blob_it, occ_s3d, newBufferSize,
                                                    local_frame_index, current_type, current_index);
                    //store 3D data and update the best one.
                    if(mobile3DVelocityCoherenceIsAcceptable() && mobile3DCoherenceIsAcceptable()) {
                        foundSolution = true;

                        current_S3D_list = bestS3DsAllTypes + current_index*m_blobsBufferSize;
                        current_bool_list = notNullS3DsAllTypes + current_index*m_blobsBufferSize;
                        for(index = 0; index<this->g_currentBufferSize; index++) {
                            if(s3dsToAnalyze[index] != NULL) {
                                memcpy(current_S3D_list + index, s3dsToAnalyze[index], sizeof(Shape3DData));
                                *(current_bool_list + index) = true;
                            }
                        }

                        if(P > bestP) {
                            bestP = P;
                            for(index = 0; index<this->g_currentBufferSize; index++)
                                if(*(current_bool_list + index) == true)
                                    bestS3Ds[index] = current_S3D_list + index;
                                else
                                    bestS3Ds[index] = NULL;
                            local_best_type = current_type;
                            local_best_subtype = ST_UNKNOWN;
                            local_best_index = current_index;
                            memcpy(&(bestInfo3D.iGData), &(i3D[current_index].iGData), sizeof(IncrementalExtraGeneralData));
                            memcpy(&(bestInfo3D.i3DData), &(i3D[current_index].i3DData), sizeof(IncrementalExtra3DData));
                            memcpy(&(bestInfo3D.t3DDimData), &(i3D[current_index].t3DDimData), sizeof(tracking3DimensionalData));
                            memcpy(&(bestInfo3D.t3DSpatialData), &(i3D[current_index].t3DSpatialData), sizeof(tracking3DSpatialData));
                        }
                    }
                } else {//For postural models postural change coherency must be checked
                    std::map<ObjectSubtype, int> *posturesMap = &objectSubModelMap[current_type];
                    std::map<ObjectSubtype, int>::iterator postures_it;
                    std::map<ObjectSubtype, Shape3DData> *postures = S3D_SUBTYPES_LIST(occ_s3d);
                    std::map<ObjectSubtype, Shape3DData>::iterator p_s3d_it, p_s3d_it_end = postures->end();
                    Shape3DData *current_posture, *bestS3DsForPosture[m_blobsBufferSize];
                    ObjectSubtype current_subtype, local_subbest;
                    double bestPForPosture = - 1.0;
                    bool foundPostureSolution = false;

                    memset(bestS3DsForPosture, 0, m_blobsBufferSize*sizeof(Shape3DData *));

                    for(j=0, postures_it = posturesMap->begin(), p_s3d_it = postures->begin();
                        p_s3d_it != p_s3d_it_end; j++, postures_it++, p_s3d_it++) {
                        current_subtype = (*postures_it).first;
                        current_posture = &(*p_s3d_it).second;
                        if(S3D_P(current_posture) >= m_classifThreshold) {
                            //Initialize data
                            memcpy(secDiffSequence, initialSecDiffSequence,  m_blobsBufferSize*sizeof(double));
                            memcpy(coolingValue,   initialCoolingValue,      m_blobsBufferSize*sizeof(double));
                            memcpy(secDiffToCurrent, initialSecDiffToCurrent, m_blobsBufferSize*sizeof(double));
                            memcpy(s3dsToAnalyzeAllTypes, initialS3DsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
                            memcpy(&t2DDimData, &initial_t2DDimData, sizeof(tracking2DimensionalData));
                            memcpy(&t2DSpatialData, &initial_t2DSpatialData, sizeof(tracking2DSpatialData));
                            memcpy(&iGData, &initial_iGData, sizeof(IncrementalExtraGeneralData));
                            memcpy(&i2DData, &initial_i2DData, sizeof(IncrementalExtra2DData));
                            memcpy(bboxesToAnalyze, initial_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
                            memcpy(visualSupport, initial_VisualSupport, m_blobsBufferSize*sizeof(double));
                            memcpy(dpFlags, initial_dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));
                            //Reset 3D information
                            memset(&t3DDimData, 0, sizeof(tracking3DimensionalData));
                            memset(&t3DSpatialData, 0, sizeof(tracking3DSpatialData));
                            memset(&i3DData, 0, sizeof(IncrementalExtra3DData));

                            i = objectModelMap[current_type];
                            k = objectSubModelMap[current_type][current_subtype];
                            current_min_w_model        = objectSubModelMinWidth[i][k];
                            current_max_w_model        = objectSubModelMaxWidth[i][k];
                            current_min_l_model        = objectSubModelMinLength[i][k];
                            current_max_l_model        = objectSubModelMaxLength[i][k];
                            current_min_h_model        = objectSubModelMinHeight[i][k];
                            current_max_h_model        = objectSubModelMaxHeight[i][k];
                            current_min_velocity_model = objectSubModelMinVelocity[i][k];
                            current_max_velocity_model = objectSubModelMaxVelocity[i][k];

                            generateMostCoherentS3DSequence(first_blob_it, current_posture, newBufferSize,
                                                            local_frame_index, current_type, current_index);

                            if(mobile3DVelocityCoherenceIsAcceptable() && mobile3DCoherenceIsAcceptable()) {
                                foundPostureSolution = true;
                                if(P > bestPForPosture) {
                                    bestPForPosture = P;
                                    memcpy(bestS3DsForPosture, s3dsToAnalyze, sizeof(Shape3DData *)*m_blobsBufferSize);

                                    local_subbest = current_subtype;
                                    memcpy(&(i3D[current_index].iGData), &iGData, sizeof(IncrementalExtraGeneralData));
                                    memcpy(&(i3D[current_index].i3DData), &i3DData, sizeof(IncrementalExtra3DData));
                                    memcpy(&(i3D[current_index].t3DDimData), &t3DDimData, sizeof(tracking3DimensionalData));
                                    memcpy(&(i3D[current_index].t3DSpatialData), &t3DSpatialData, sizeof(tracking3DSpatialData));
                                }
                            }
                        }                        
                    }

                    if(foundPostureSolution && bestPForPosture > bestP) {
                        bestP = bestPForPosture;
                        foundSolution = true;

                        current_S3D_list = bestS3DsAllTypes + current_index*m_blobsBufferSize;
                        current_bool_list = notNullS3DsAllTypes + current_index*m_blobsBufferSize;
                        for(index = 0; index<this->g_currentBufferSize; index++) {
                            if(bestS3DsForPosture[index] != NULL) {
                                memcpy(current_S3D_list + index, bestS3DsForPosture[index], sizeof(Shape3DData));
                                *(current_bool_list + index) = true;
                            }
                        }

                        for(index = 0; index<this->g_currentBufferSize; index++)
                            if(*(current_bool_list + index) == true)
                                bestS3Ds[index] = current_S3D_list + index;
                            else
                                bestS3Ds[index] = NULL;

                        local_best_type = current_type;
                        local_best_index = current_index;
                        local_best_subtype = local_subbest;
                        memcpy(&(bestInfo3D.iGData), &(i3D[current_index].iGData), sizeof(IncrementalExtraGeneralData));
                        memcpy(&(bestInfo3D.i3DData), &(i3D[current_index].i3DData), sizeof(IncrementalExtra3DData));
                        memcpy(&(bestInfo3D.t3DDimData), &(i3D[current_index].t3DDimData), sizeof(tracking3DimensionalData));
                        memcpy(&(bestInfo3D.t3DSpatialData), &(i3D[current_index].t3DSpatialData), sizeof(tracking3DSpatialData));
                        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(current_index);
                        memcpy(s3dsToAnalyze, bestS3Ds, sizeof(Shape3DData *)*m_blobsBufferSize);
                    }
                }
            }
        } //end main for

        //If no coherent solution found, add default information and pass to the next blob
        if(!foundSolution) {
            best_type = UNKNOWN;
            best_subtype = ST_UNKNOWN;
            best_index = 0;
            current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
            current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;

            if(local_frame_index == 0) //If this blob corresponds to the current one, no more possibilities to analyze
                break;

            //Initialize data
            memcpy(secDiffSequence, initialSecDiffSequence,  m_blobsBufferSize*sizeof(double));
            memcpy(coolingValue,   initialCoolingValue,      m_blobsBufferSize*sizeof(double));
            memcpy(secDiffToCurrent, initialSecDiffToCurrent, m_blobsBufferSize*sizeof(double));
            memcpy(s3dsToAnalyzeAllTypes, initialS3DsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
            memcpy(&t2DDimData, &initial_t2DDimData, sizeof(tracking2DimensionalData));
            memcpy(&t2DSpatialData, &initial_t2DSpatialData, sizeof(tracking2DSpatialData));
            memcpy(&iGData, &initial_iGData, sizeof(IncrementalExtraGeneralData));
            memcpy(&i2DData, &initial_i2DData, sizeof(IncrementalExtra2DData));
            memcpy(bboxesToAnalyze, initial_BBoxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
            memcpy(visualSupport, initial_VisualSupport, m_blobsBufferSize*sizeof(double));
            memcpy(dpFlags, initial_dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));
            memset(bestS3DsAllTypes, 0, m_blobsBufferSize*m_objectModelsNumber*sizeof(Shape3DData));
            memset(notNullS3DsAllTypes, 0, m_blobsBufferSize*m_objectModelsNumber*sizeof(bool));

            //Update time
            if(newBufferSize == 1)
                secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
            else {
                memmove(secDiffSequence  + 1, secDiffSequence,  sizeof(double)*(m_blobsBufferSize - 1));
                memmove(secDiffToCurrent + 1, secDiffToCurrent, sizeof(double)*(m_blobsBufferSize - 1));
                secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
                for(i=2; i<currentBufferSize; i++)
                    secDiffToCurrent[i] += secDiffSequence[0];
                for(i=1; i<currentBufferSize; i++)
                    coolingValue[i] = coolingFunction(secDiffToCurrent[i]);
            }

            //Update general info with current blob
            memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
            memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
            memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);
            numberOfClassifiedS3ds = 0;
            numberOfFoundS3ds = 0;
            memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
            visualSupport[0] = g_newVisualSupport[local_frame_index];
            memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
            memcpy(&(bboxesToAnalyze[0]), &(g_newBBoxesToAnalyze[local_frame_index]), sizeof(Rectangle<int>));

            unknownsNumber = 0;
            for(i=0; i<newBufferSize; i++) {
                if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                    foundS3ds[i] = true;
                    numberOfFoundS3ds++;
                } else
                    unknownsNumber++;
            }

            RFoundSolutions  = (double)numberOfFoundS3ds     /(double)newBufferSize;
            RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds - 1)/(double)(newBufferSize-1);
            RKnownSolutions  = RVKnownSolutions = 0.0;

            int j;
            for(j=0 ; j < m_objectModelsNumber ; j++)
                insertS3DToAnalyzeByIndex(j, NULL);

            //Update mobile info
            m_rc->setDPForBlob(current_blob, m_pSegmentation);
            currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);

            incrementalUpdateCooling(newBufferSize);
            incrementalUpdate2DDimensions(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
            incrementalUpdate2DPosition(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);

            //Save data from new starting point
            memcpy(initialSecDiffSequence,  secDiffSequence, m_blobsBufferSize*sizeof(double));
            memcpy(initialCoolingValue,     coolingValue,    m_blobsBufferSize*sizeof(double));
            memcpy(initialSecDiffToCurrent, secDiffToCurrent, m_blobsBufferSize*sizeof(double));
            memcpy(initialS3DsToAnalyzeAllTypes, s3dsToAnalyzeAllTypes, sizeof(Shape3DData *)*m_blobsBufferSize*m_objectModelsNumber);
            memcpy(&initial_t2DDimData, &t2DDimData, sizeof(tracking2DimensionalData));
            memcpy(&initial_t2DSpatialData, &t2DSpatialData, sizeof(tracking2DSpatialData));
            memcpy(&initial_iGData, &iGData, sizeof(IncrementalExtraGeneralData));
            memcpy(&initial_i2DData, &i2DData, sizeof(IncrementalExtra2DData));
            memcpy(initial_BBoxesToAnalyze, bboxesToAnalyze, m_blobsBufferSize*sizeof(Rectangle<int>));
            memcpy(initial_VisualSupport, visualSupport, m_blobsBufferSize*sizeof(double));
            memcpy(initial_dpFlags, dpFlags, m_blobsBufferSize*sizeof(DetectionProblemType));

            //Set information for analyzing next blob
            local_frame_index--;
            first_blob_it++;
            newBufferSize++;
        }

        local_frame_index--;
    } while(foundSolution == false && local_frame_index >= 0);

    if(foundSolution) {
        //Set best 3D information
        memcpy(&iGData, &(bestInfo3D.iGData), sizeof(IncrementalExtraGeneralData));
        memcpy(&i3DData, &(bestInfo3D.i3DData), sizeof(IncrementalExtra3DData));
        memcpy(&t3DDimData, &(bestInfo3D.t3DDimData), sizeof(tracking3DimensionalData));
        memcpy(&t3DSpatialData, &(bestInfo3D.t3DSpatialData), sizeof(tracking3DSpatialData));
        memcpy(s3dsToAnalyze, bestS3Ds, sizeof(Shape3DData *)*m_blobsBufferSize);

        for(i=0; i<g_currentBufferSize; i++)
            if(s3dsToAnalyze[i] != NULL && S3D_P(s3dsToAnalyze[i]) >= m_classifThreshold) {
                best_type = S3D_TYPE(s3dsToAnalyze[i]);
                best_subtype = S3D_SUBTYPE(s3dsToAnalyze[i]);
                best_index = objectModelMap[best_type];
                break;
            }

        std::deque<SpBlob>::reverse_iterator blobsBegin = blobHistory.rbegin();
        Blob *lastBlob;

        int m, nummod;
        Shape3DData *normal_s3d;

        for(i=0; i<g_currentBufferSize; i++, blobsBegin++) {
            lastBlob = &(**blobsBegin);
            if(BLOB_NORMAL_3DDATA(lastBlob) != NULL) {
                delete BLOB_NORMAL_3DDATA(lastBlob);
                BLOB_NORMAL_3DDATA(lastBlob) = NULL;
                setInitialNormalList(lastBlob);
            }

            normal = BLOB_NORMAL_3DDATA(lastBlob);
            std::map<ObjectType, Shape3DData>::iterator n_it, n_end = normal->end();

            for(nummod = 0, n_it = normal->begin(); n_it != n_end && nummod < m_objectModelsNumber; nummod++, n_it++) {
                m = objectModelMap[n_it->first];
                if(notNullS3DsAllTypes[m*m_blobsBufferSize + i]) {
                    normal_s3d = &n_it->second;

                    memcpy(normal_s3d, bestS3DsAllTypes + m*m_blobsBufferSize + i, sizeof(Shape3DData));
                    S3D_SUBTYPES_LIST(normal_s3d) = NULL;

                    s3dsToAnalyzeAllTypes[m*m_blobsBufferSize + i] = normal_s3d;
                } else
                    s3dsToAnalyzeAllTypes[m*m_blobsBufferSize + i] = NULL;
            }
        }

        //update S3Ds to analyze best information
        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(best_index);

        //Set limits for current model
        if(rigidModel[best_type]) {
            i = objectModelMap[best_type];
            current_min_w_model       = objectModelMinWidth[i];
            current_max_w_model       = objectModelMaxWidth[i];
            current_min_l_model       = objectModelMinLength[i];
            current_max_l_model       = objectModelMaxLength[i];
            current_min_h_model       = objectModelMinHeight[i];
            current_max_h_model       = objectModelMaxHeight[i];
            current_min_velocity_model = objectModelMinVelocity[i];
            current_max_velocity_model = objectModelMaxVelocity[i];
        } else {
            i = objectModelMap[best_type];
            k = objectSubModelMap[best_type][best_subtype];
            current_min_w_model        = objectSubModelMinWidth[i][k];
            current_max_w_model        = objectSubModelMaxWidth[i][k];
            current_min_l_model        = objectSubModelMinLength[i][k];
            current_max_l_model        = objectSubModelMaxLength[i][k];
            current_min_h_model        = objectSubModelMinHeight[i][k];
            current_max_h_model        = objectSubModelMaxHeight[i][k];
            current_min_velocity_model = objectSubModelMinVelocity[i][k];
            current_max_velocity_model = objectSubModelMaxVelocity[i][k];
        }
    } else {
        best_type = UNKNOWN;
        best_subtype = ST_UNKNOWN;
        best_index = 0;
        current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
        current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;
        //update S3Ds to analyze best information
        s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(0);
    }
}


void MobileObject::generateMostCoherentS3DSequence(std::deque<SpBlob>::iterator first_blob_it, Shape3DData *firstClassified, int newBufferSize,
                                                  int frame_index, ObjectType current_type, int current_index) {
    //Update general info with current blob
    int i, j;
    Blob *current_blob = &(**first_blob_it);
    Shape3DData *nextClassified;
    std::map<ObjectType, Shape3DData> *normal, *occ;
    std::deque<SpBlob>::iterator next_blob_it = first_blob_it;
    //Update time
    if(newBufferSize == 1)
        secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
    else {
        memmove(secDiffSequence  + 1, secDiffSequence,  sizeof(double)*(m_blobsBufferSize - 1));
        memmove(secDiffToCurrent + 1, secDiffToCurrent, sizeof(double)*(m_blobsBufferSize - 1));
        secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
        for(i=2; i<currentBufferSize; i++)
            secDiffToCurrent[i] += secDiffSequence[0];
        for(i=1; i<currentBufferSize; i++)
            coolingValue[i] = coolingFunction(secDiffToCurrent[i]);
    }

    //Update general info with current blob
    memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
    memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
    memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);
    numberOfClassifiedS3ds = 0;
    numberOfFoundS3ds = 0;
    memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
    visualSupport[0] = g_newVisualSupport[frame_index];
    memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
    memcpy(&(bboxesToAnalyze[0]), &(g_newBBoxesToAnalyze[frame_index]), sizeof(Rectangle<int>));

    classifiedS3ds[0] = true;
    numberOfClassifiedS3ds++;

    //update s3dsToAnalyze

    unknownsNumber = 0;
    for(i=0; i<newBufferSize; i++) {
        if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            foundS3ds[i] = true;
            numberOfFoundS3ds++;
        } else
            unknownsNumber++;
    }

    RFoundSolutions  = (double)numberOfFoundS3ds/(double)newBufferSize;
    RVFoundSolutions = numberOfFoundS3ds < 2 ? 0.0 : (double)(numberOfFoundS3ds - 1)/(double)(newBufferSize-1);
    RKnownSolutions  = 1.0 / (double)newBufferSize;
    RVKnownSolutions = 0.0;

    //insert the S3D in the S3Ds list
    insertS3DToAnalyzeByIndex(current_index, firstClassified);

    s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(current_index);
    best_type = current_type;
    best_subtype = S3D_SUBTYPE(s3dsToAnalyze[0]);
    best_index = current_index;

    //Update mobile info including 3D info
    currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);

    incrementalUpdateCooling(newBufferSize);
    incrementalUpdate2DDimensions(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
    incrementalUpdate2DPosition(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
    incrementalUpdateOrientation(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
    incrementalUpdate3DDimensions(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
    incrementalUpdate3DPosition(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);

    //Complete with most coherent information
    for(j = --frame_index; j >= 0; j--) {
        newBufferSize++;
        next_blob_it++;
        current_blob = &(**next_blob_it);

        m_rc->prepareBlobForRClassification(current_blob, m_pSegmentation);

        if(BLOB_NORMAL_3DDATA(current_blob) == NULL) { //If blob does not have list allocated, copy the list from first
                                                       //blob, which is classified for sure
            normal = BLOB_NORMAL_3DDATA(current_blob) = Shape3DData::copyList(BLOB_NORMAL_3DDATA((*first_blob_it)));
            if(BLOB_OCC_3DDATA((*first_blob_it)) != NULL)
                occ = BLOB_OCC_3DDATA(current_blob) = Shape3DData::copyList(BLOB_OCC_3DDATA((*first_blob_it)));
            else
                occ = BLOB_OCC_3DDATA(current_blob) = NULL;
        } else {
            normal = BLOB_NORMAL_3DDATA(current_blob);
            occ = BLOB_OCC_3DDATA(current_blob);
        }

        //Update time
        if(newBufferSize == 1)
            secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
        else {
            memmove(secDiffSequence  + 1, secDiffSequence,  sizeof(double)*(m_blobsBufferSize - 1));
            memmove(secDiffToCurrent + 1, secDiffToCurrent, sizeof(double)*(m_blobsBufferSize - 1));
            secDiffToCurrent[1] = secDiffSequence[0] = g_secDiffSequence[frame_index];
            for(i=2; i<newBufferSize; i++)
                secDiffToCurrent[i] += secDiffSequence[0];
            for(i=1; i<newBufferSize; i++)
                coolingValue[i] = coolingFunction(secDiffToCurrent[i]);
        }

        memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
        memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
        memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);
        numberOfClassifiedS3ds = 0;
        numberOfFoundS3ds = 0;
        memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
        visualSupport[0] = g_newVisualSupport[frame_index];
        memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
        memcpy(&(bboxesToAnalyze[0]), &(g_newBBoxesToAnalyze[frame_index]), sizeof(Rectangle<int>));

        nextClassified = getBestPostureCoherentS3DReclassifying(current_blob, normal, occ, current_type);
        //nextClassified = generateBestLikelys3d(current_blob, current_type);

        //insert the s3d in the s3ds list
        insertS3DToAnalyzeByIndex(current_index, nextClassified);

        //Update best_subtype
        if(!rigidModel[best_type] && s3dsToAnalyze[0] != NULL && S3D_P(s3dsToAnalyze[0]) >= m_classifThreshold)
            best_subtype = S3D_SUBTYPE(s3dsToAnalyze[0]);

        for(i=0; i<newBufferSize; i++) {
            if(    s3dsToAnalyze[i] != NULL && (S3D_P(s3dsToAnalyze[i]) >= m_classifThreshold)
                && ( !(S3D_DP_TYPE(s3dsToAnalyze[i]) & MM_NOT_VISIBLE_MASK) ) ) {
                classifiedS3ds[i] = true;
                numberOfClassifiedS3ds++;
            }
            if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                foundS3ds[i] = true;
                numberOfFoundS3ds++;
            }
        }

        RKnownSolutions  = (double)numberOfClassifiedS3ds/(double)newBufferSize;
        RFoundSolutions  = (double)numberOfFoundS3ds     /(double)newBufferSize;
        if(!rigidModel[best_type]) {
            i = objectSubModelMap[best_type][best_subtype];

            current_min_w_model        = objectSubModelMinWidth[best_index][i];
            current_max_w_model        = objectSubModelMaxWidth[best_index][i];
            current_min_l_model        = objectSubModelMinLength[best_index][i];
            current_max_l_model        = objectSubModelMaxLength[best_index][i];
            current_min_h_model        = objectSubModelMinHeight[best_index][i];
            current_max_h_model        = objectSubModelMaxHeight[best_index][i];
            current_min_velocity_model = objectSubModelMinVelocity[best_index][i];
            current_max_velocity_model = objectSubModelMaxVelocity[best_index][i];
        }

        if(newBufferSize == 1)
            RVFoundSolutions = RVKnownSolutions = 0.0;
        else {
            RVKnownSolutions = numberOfClassifiedS3ds < 2 ? 0.0 : (double)(numberOfClassifiedS3ds-1)/(double)(newBufferSize-1);
            RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds     -1)/(double)(newBufferSize-1);
        }

        currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);

        //Update mobile info including 3D info
        incrementalUpdateCooling(newBufferSize);
        incrementalUpdate2DDimensions(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
        incrementalUpdate2DPosition(newBufferSize, bboxesToAnalyze, dpFlags, visualSupport, current_blob->maxDistanceFactor);
        incrementalUpdateOrientation(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
        incrementalUpdate3DDimensions(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
        incrementalUpdate3DPosition(newBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
    }

    //Set global probability
    setGlobalProbability();
}

void MobileObject::generateS3DFrom3DInformation(Shape3DData *outs3d) {
    int beta_direction = ReliabilityTracker::getBetaDirection(m_context, this->m_pSegmentation);
    double
        w = t3DDimData.SDVw > t3DDimData.Vw
          ? t3DDimData.w
          : t3DDimData.w + t3DDimData.RVw*(t3DDimData.Vw * secDiffSequence[0]),
        l = t3DDimData.SDVl > t3DDimData.Vl
          ? t3DDimData.l
          : t3DDimData.l + t3DDimData.RVl*(t3DDimData.Vl * secDiffSequence[0]),
        h = t3DDimData.SDVh > t3DDimData.Vh
          ? t3DDimData.h
          : t3DDimData.h + t3DDimData.RVh*(t3DDimData.Vh * secDiffSequence[0]),
        x = t3DSpatialData.SDVx > t3DSpatialData.Vx
          ? t3DSpatialData.x
          : t3DSpatialData.x + t3DSpatialData.Vx * secDiffSequence[0],
        y = t3DSpatialData.SDVy > t3DSpatialData.Vy
          ? t3DSpatialData.y
          : t3DSpatialData.y + t3DSpatialData.Vy * secDiffSequence[0],
        alpha = t3DDimData.SDValpha > t3DDimData.Valpha
              ? t3DDimData.alpha
              : t3DDimData.alpha + t3DDimData.Valpha * secDiffSequence[0];
    double sina = sin(alpha), cosa = cos(alpha);

    if(l < objectModelMinLength[best_index])
        l = objectModelMinLength[best_index];
    else if(l > objectModelMaxLength[best_index])
        l = objectModelMaxLength[best_index];

    if(w < objectModelMinWidth[best_index])
        w = objectModelMinWidth[best_index];
    else if(w > objectModelMaxWidth[best_index])
        w = objectModelMaxWidth[best_index];

    if(h < objectModelMinHeight[best_index])
        h = objectModelMinHeight[best_index];
    else if(h > objectModelMaxHeight[best_index])
        h = objectModelMaxHeight[best_index];

    memset(outs3d, 0, sizeof(Shape3DData));

    S3D_W(outs3d) = w;
    S3D_L(outs3d) = l;
    S3D_H(outs3d) = h;
    S3D_3D_X(outs3d) = x;
    S3D_3D_Y(outs3d) = y;
    S3D_ALPHA(outs3d) = alpha;
    S3D_3DBBOX(outs3d)->getFromInitial3DPoint(m_context, S3D_BBOX(outs3d),
                                              x + beta_direction*(w*sina - l*cosa)/2.0,
                                              y - (w*cosa + l*sina)/2.0,
                                              1, alpha, beta_direction, w, l, h);

}

void MobileObject::getMobile3DTolerances(double *Wtol, double *Htol) {
    //If not enough 3D information is available, use the 2D information
    if(numberOfClassifiedS3ds < 2) {
        double tdiff = secDiffSequence[0];
        *Wtol = 2*(t2DDimData.SDW + t2DDimData.SDVW*tdiff);
        *Htol = 2*(t2DDimData.SDH + t2DDimData.SDVH*tdiff);
        if(*Wtol < m_minimalTolerance) *Wtol = m_minimalTolerance;
        if(*Htol < m_minimalTolerance) *Htol = m_minimalTolerance;
        return;
    }
    double Wmean = 0.0, Hmean = 0.0;
    int i, Ws[currentBufferSize], Hs[currentBufferSize];

    //Mean estimated 2D Dimensions from 3D data
    for (i=0;i<currentBufferSize;i++)
        if(classifiedS3ds[i] == true) {
            Wmean += Ws[i] = S3D_WIDTH(s3dsToAnalyze[i]);
            Hmean += Hs[i] = S3D_HEIGHT(s3dsToAnalyze[i]);
        }
    Wmean /= (double)numberOfClassifiedS3ds;
    Hmean /= (double)numberOfClassifiedS3ds;

    //Tolerances calculated as the double of the s.d.
    *Wtol = *Htol = 0.0;
    double aux;
    for (i=0;i<currentBufferSize;i++)
        if(classifiedS3ds[i] == true) {
            aux = Ws[i] - Wmean;
            *Wtol += aux*aux;
            aux = Hs[i] - Hmean;
            *Htol += aux*aux;
        }
    *Wtol = 2*sqrt(*Wtol/(double)numberOfClassifiedS3ds);
    *Htol = 2*sqrt(*Htol/(double)numberOfClassifiedS3ds);
    if(*Wtol < m_minimalTolerance) *Wtol = m_minimalTolerance;
    if(*Htol < m_minimalTolerance) *Htol = m_minimalTolerance;

}


void MobileObject::setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport) {

    int
        Xc = RECT_XCENTER(bboxResult),
        Yc = RECT_YCENTER(bboxResult),
        Xb = RECT_XCENTER(realBBox),
        Yb = RECT_YCENTER(realBBox),
        Wc = RECT_WIDTH(bboxResult),
        Hc = RECT_HEIGHT(bboxResult),
        Wb = RECT_WIDTH(realBBox),
        Hb = RECT_HEIGHT(realBBox);
    double
        visualComplement = 1.0 - visualSupport,
        X  = visualSupport * Xc + visualComplement * Xb,
        Y  = visualSupport * Yc + visualComplement * Yb,
        W  = visualSupport * Wc + visualComplement * Wb,
        H  = visualSupport * Hc + visualComplement * Hb;

    RECT_XRIGHT(bboxResult)  = (int) ( (2.0*X + W - 1) / 2.0 );
    RECT_XLEFT(bboxResult)   = (int) ( RECT_XRIGHT(bboxResult) - W + 1 );
    RECT_WIDTH(bboxResult)   = (int) W;

    RECT_YBOTTOM(bboxResult) = (int) ( (2.0*Y + H - 1) / 2.0 );
    RECT_YTOP(bboxResult)    = (int) ( RECT_YBOTTOM(bboxResult) - H + 1 );
    RECT_HEIGHT(bboxResult)  = (int) H;
}

void MobileObject::orient3DModel(Shape3DData *improved3DInfo, Rectangle<int> *improvedBBox, DetectionProblemType occtype) {

    Rectangle<int> *improved3DBBox = S3D_BBOX(improved3DInfo);
    Parallelpiped *par =  S3D_3DBBOX(improved3DInfo);
    double
      x3D = (PARALL_X_i(par, 0) + PARALL_X_i(par, 2)) / 2.0,
      y3D = (PARALL_Y_i(par, 0) + PARALL_Y_i(par, 2)) / 2.0,
      X2D, Y2D, distToXLeft, distToYTop, distToXRight, distToYBottom, newX2D, newY2D;
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3D, y3D, 0.0, &X2D, &Y2D);
    distToXLeft    = X2D - RECT_XLEFT(improved3DBBox);
    distToYTop     = Y2D - RECT_YTOP (improved3DBBox);
    distToXRight   = RECT_XRIGHT (improved3DBBox) - X2D;
    distToYBottom  = RECT_YBOTTOM(improved3DBBox) - Y2D;

    //If occlusion on both sides, position it at center
    if(RECT_XLEFT(improved3DBBox) != RECT_XLEFT(improvedBBox) && RECT_XRIGHT(improved3DBBox) != RECT_XRIGHT(improvedBBox)) {
      if(    ( (RECT_XLEFT(improved3DBBox) > RECT_XLEFT(improvedBBox)) && RECT_XRIGHT(improved3DBBox) < RECT_XRIGHT(improvedBBox))
          || ( (RECT_XLEFT(improved3DBBox) < RECT_XLEFT(improvedBBox)) && RECT_XRIGHT(improved3DBBox) > RECT_XRIGHT(improvedBBox) ) ) {
        int xc = RECT_XCENTER(improvedBBox);
        double
          W_2 = RECT_WIDTH(improved3DBBox)  / 2.0;

        RECT_XLEFT(improved3DBBox) = (int)(xc - W_2);
        RECT_XRIGHT(improved3DBBox) =(int)(xc + W_2);

      } else if ( (RECT_XLEFT(improved3DBBox) < RECT_XLEFT(improvedBBox)) && (RECT_XRIGHT(improved3DBBox) < RECT_XRIGHT(improvedBBox)) ) {
        int dLL = RECT_XLEFT(improvedBBox) - RECT_XLEFT(improved3DBBox), dRR = RECT_XRIGHT(improvedBBox) - RECT_XRIGHT(improved3DBBox);
        if(dLL > dRR) {
          RECT_XRIGHT(improved3DBBox) = RECT_XRIGHT(improvedBBox);
          RECT_XLEFT(improved3DBBox) = RECT_XRIGHT(improvedBBox) - RECT_WIDTH(improved3DBBox) + 1;
        } else {
          RECT_XLEFT(improved3DBBox) = RECT_XLEFT(improvedBBox);
          RECT_XRIGHT(improved3DBBox) = RECT_XLEFT(improvedBBox) + RECT_WIDTH(improved3DBBox) - 1;
        }
      } else { //Both from 3D to the right
        int dLL = RECT_XLEFT(improved3DBBox) - RECT_XLEFT(improvedBBox), dRR = RECT_XRIGHT(improved3DBBox) - RECT_XRIGHT(improvedBBox);
        if(dLL > dRR) {
          RECT_XRIGHT(improved3DBBox) = RECT_XRIGHT(improvedBBox);
          RECT_XLEFT(improved3DBBox) = RECT_XRIGHT(improvedBBox) - RECT_WIDTH(improved3DBBox) + 1;
        } else {
          RECT_XLEFT(improved3DBBox) = RECT_XLEFT(improvedBBox);
          RECT_XRIGHT(improved3DBBox) = RECT_XLEFT(improvedBBox) + RECT_WIDTH(improved3DBBox) - 1;
        }
      }

      if( ( (occtype & MM_LEFT_OCCL) && (occtype & MM_RIGHT_OCCL) ) || !(occtype & MM_HORIZONTAL_OCCL_MASK) )
        newX2D = RECT_XLEFT(improved3DBBox) + distToXLeft;
      else if (occtype & MM_LEFT_OCCL) { //Right is trustable
        newX2D = RECT_XRIGHT(improved3DBBox) - distToXRight;
      } else //RIGHT OCCLUSION (LEFT IS TRUSTABLE)
        newX2D = RECT_XLEFT(improved3DBBox) + distToXLeft;
    } else
      newX2D = X2D;

    //Vertical
    if(RECT_YTOP(improved3DBBox) != RECT_YTOP(improvedBBox) && RECT_YBOTTOM(improved3DBBox) != RECT_YBOTTOM(improvedBBox)) {
      if(    ( (RECT_YTOP(improved3DBBox) > RECT_YTOP(improvedBBox)) && RECT_YBOTTOM(improved3DBBox) < RECT_YBOTTOM(improvedBBox))
             || ( (RECT_YTOP(improved3DBBox) < RECT_YTOP(improvedBBox)) && RECT_YBOTTOM(improved3DBBox) > RECT_YBOTTOM(improvedBBox) ) ) {
        int yc = RECT_YCENTER(improvedBBox);
        double
          H_2 = RECT_HEIGHT(improved3DBBox)  / 2.0;
        RECT_YTOP(improved3DBBox)    = (int) (yc - H_2);
        RECT_YBOTTOM(improved3DBBox) = (int) (yc + H_2);
      } else if ( (RECT_YTOP(improved3DBBox) < RECT_YTOP(improvedBBox)) && (RECT_YBOTTOM(improved3DBBox) < RECT_YBOTTOM(improvedBBox)) ) {
        int dTT = RECT_YTOP(improvedBBox) - RECT_YTOP(improved3DBBox), dBB = RECT_YBOTTOM(improvedBBox) - RECT_YBOTTOM(improved3DBBox);
        if(dTT > dBB) {
          RECT_YBOTTOM(improved3DBBox) = RECT_YBOTTOM(improvedBBox);
          RECT_YTOP(improved3DBBox) = RECT_YBOTTOM(improvedBBox) - RECT_HEIGHT(improved3DBBox) + 1;
        } else {
          RECT_YTOP(improved3DBBox) = RECT_YTOP(improvedBBox);
          RECT_YBOTTOM(improved3DBBox) = RECT_YTOP(improvedBBox) + RECT_HEIGHT(improved3DBBox) - 1;
        }
      } else { //Both from 3D to the right
        int dTT = RECT_YTOP(improved3DBBox) - RECT_YTOP(improvedBBox), dBB = RECT_YBOTTOM(improved3DBBox) - RECT_YBOTTOM(improvedBBox);
        if(dTT > dBB) {
          RECT_YBOTTOM(improved3DBBox) = RECT_YBOTTOM(improvedBBox);
          RECT_YTOP(improved3DBBox) = RECT_YBOTTOM(improvedBBox) - RECT_HEIGHT(improved3DBBox) + 1;
        } else {
          RECT_YTOP(improved3DBBox) = RECT_YTOP(improvedBBox);
          RECT_YBOTTOM(improved3DBBox) = RECT_YTOP(improvedBBox) + RECT_HEIGHT(improved3DBBox) - 1;
        }
      }

      if( ( (occtype & MM_TOP_OCCL) && (occtype & MM_BOTTOM_OCCL) ) || !(occtype & MM_VERTICAL_OCCL_MASK) )
        newY2D = RECT_YTOP(improved3DBBox) + distToYTop;
      else if(occtype & MM_TOP_OCCL)
        newY2D = RECT_YBOTTOM(improved3DBBox) - distToYBottom;
      else
        newY2D = RECT_YTOP(improved3DBBox) + distToYTop;
    } else
      newY2D = Y2D;
    //Reposition Center or Parallelepiped
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), newX2D, newY2D, 0, &x3D, &y3D);

    repositionS3D(improved3DInfo, x3D, y3D);
}


void MobileObject::repositionS3D(Shape3DData *s3d, double x, double y) {
    int beta_direction = ReliabilityTracker::getBetaDirection(m_context, m_pSegmentation);
    double
        w = S3D_W(s3d),
        l = S3D_L(s3d),
        h = S3D_H(s3d),
        alpha = S3D_ALPHA(s3d);
    double sina = sin(alpha), cosa = cos(alpha);

    if(l < objectModelMinLength[best_index])
        l = objectModelMinLength[best_index];
    else if(l > objectModelMaxLength[best_index])
        l = objectModelMaxLength[best_index];

    if(w < objectModelMinWidth[best_index])
        w = objectModelMinWidth[best_index];
    else if(w > objectModelMaxWidth[best_index])
        w = objectModelMaxWidth[best_index];

    if(h < objectModelMinHeight[best_index])
        h = objectModelMinHeight[best_index];
    else if(h > objectModelMaxHeight[best_index])
        h = objectModelMaxHeight[best_index];

    S3D_3DBBOX(s3d)->getFromInitial3DPoint(m_context, S3D_BBOX(s3d),
                                           x + beta_direction*(w*sina - l*cosa)/2.0,
                                           y - (w*cosa + l*sina)/2.0,
                                           1, alpha, beta_direction, w, l, h);
    S3D_3D_X(s3d) = x;
    S3D_3D_Y(s3d) = y;
}

//Last Best Type Update Version, including Visual Support Analysis, 3D Analysis Initializer and Most Coherent 3D
//solution construction.
void MobileObject::updateBestType3DInformation() {

//    std::map<ObjectType, int>::iterator modelMap_it;
//    std::map<ObjectType, Shape3DData> *normal, *occ;
//    int current_index;
//    ObjectType current_type;
    Blob *current_blob = &(*blobHistory.back());
    int i, j;

//    debug_data_flag = -1;

    //    if(mobile_id == 1)
    //  std::cout << "Este me interesa!!!" << std::endl;

    //Authorized to add points to trajectory buffers
    productionMode = true;

    memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
    currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);

    memset(classifiedS3ds, false, sizeof(bool)*m_blobsBufferSize);
    memset(foundS3ds, false, sizeof(bool)*m_blobsBufferSize);

    numberOfClassifiedS3ds = 0;
    numberOfFoundS3ds = 0;

    DetectionProblemType not_visible_mask      = (DetectionProblemType) (BLOB_DP_TYPE(current_blob) & MM_NOT_VISIBLE_MASK);
    DetectionProblemType non_3D_trackable_mask = (DetectionProblemType) (BLOB_DP_TYPE(current_blob) & MM_NOT_3D_TRACKABLE_MASK);

//    if(!ensureMode && non_3D_trackable_mask && numberOfFramesSinceFirstTimeSeen > m_2DLevelFrames) {
        //In this case type information will not change,
        //so just the current data must be updated
//        memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
//        Rectangle<int> mobileBBox, improvedBBox;
//        getCurrentBoundingBoxForMobile(&mobileBBox);

        //Update bbox to analyze and displace the visual support buffer.
//        memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
//        improveBBoxSupport(&improvedBBox, &mobileBBox, &(current_blob->bbox));

//        if(currentVisualState & MM_PART_OF_BIGGER)
            //support rate here is the coverage rate of the mobile bbox estimation over the current blob
//            visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &improvedBBox);
//        else if(currentVisualState & MM_PARTIALLY_DETECTED)
        //support rate here is the coverage rate of the current blob over the mobile bbox estimation
//            visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&improvedBBox, &(current_blob->bbox));
//        else //This case should not happen as
//            visualSupport[0] = 1.0;

//        memcpy(&(bboxesToAnalyze[0]), &improvedBBox, sizeof(Rectangle<int>));
        //setSpecialBBoxToAnalyze(&(bboxesToAnalyze[0]), &(current_blob->bbox), visualSupport[0]);

//        for(i=0; i<currentBufferSize; i++) {
//            if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
//                foundS3ds[i] = true;
//                numberOfFoundS3ds++;
//            }
//        }

//        RKnownSolutions  = (double)numberOfClassifiedS3ds/(double)currentBufferSize;
//        RFoundSolutions  = (double)numberOfFoundS3ds     /(double)currentBufferSize;

//        RVKnownSolutions = numberOfClassifiedS3ds < 2 ? 0.0 : (double)(numberOfClassifiedS3ds-1)/(double)(currentBufferSize-1);
//        RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds     -1)/(double)(currentBufferSize-1);

//        return;
//    }

    //Just 2D info.
//    if(numberOfFramesSinceFirstTimeSeen <= m_2DLevelFrames) {

//        debug_data_flag = 0;

        //If blob has not been classified yet, obtain information about its possible occlusions
//        if(BLOB_NORMAL_3DDATA(current_blob) == NULL) {
//            m_rc->setOcclussionForBlob(current_blob, m_pSegmentation);
//            currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);
//        }

        //Update bbox to analyze and displace the visual support buffer.
        memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
        memcpy(&(bboxesToAnalyze[0]), &(current_blob->bbox), sizeof(Rectangle<int>));
        memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
        visualSupport[0] = 1.0;
//        best_type = UNKNOWN;
//        best_index = 0;
//        best_subtype = ST_UNKNOWN;
//        current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
//        current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;

        //Set the classification flags for data to be analysed
        for(i=0;i<currentBufferSize; i++)
            if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                foundS3ds[i] = true;
                numberOfFoundS3ds++;
        }

        RFoundSolutions  = (double)numberOfFoundS3ds     /(double)currentBufferSize;
        RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds     -1)/(double)(currentBufferSize-1);

        if(not_visible_mask)
            unknownsNumber++;
        else
            setNumberOfFramesNotSeen(0);

//        m_rc->setOcclussionForBlob(current_blob, m_pSegmentation);

        lastUnknown = true;

//        return;
//    }
    }
/*    //For a frame number out of range of pure 2D tracking
    if(classificationAllowed == false) { //First time to allow analysis of 3D classification information

        debug_data_flag = 1;

        //Not authorized to add points to trajectory buffers
        productionMode = false;
        classificationAllowed = true;
        memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
        memcpy(&(bboxesToAnalyze[0]), &(current_blob->bbox), sizeof(Rectangle<int>));
        memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
        visualSupport[0] = 1.0;

        lastUnknown = classifiedS3ds[0]  ? false : true;

        //Function for generation of first s3dsToAnalyze
        generateFirstClassifiedSequence();

        double sum;
        Shape3DData **s3ds;

        for(i=0; i<m_objectModelsNumber; i++) {
            sum = 0;
            s3ds = getS3DBufferToAnalyzeByIndex(i);
            for(j=0; j<currentBufferSize; j++)
                sum += s3ds[j] != NULL ? S3D_P(s3ds[j]) : 0.0;
            PSumPerType[i] = sum;
        }

        return;
    }


    //Update bbox to analyze and displace the visual support buffer.
    memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
    memcpy(&(bboxesToAnalyze[0]), &(current_blob->bbox), sizeof(Rectangle<int>));
    memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));

    if(not_visible_mask) {

        unknownsNumber++;
        lastUnknown = true;
        visualSupport[0] = 0.0;

        if(ensureMode) {
            debug_data_flag = 2;

            insertS3DToAnalyzeByIndex(best_index, NULL);
            s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(best_index);
        } else {
            debug_data_flag = 3;
            int n = m_objectModelsNumber;
            std::cout << "Verifica:" << n << std::endl;
            int j;            
            for(j = 0; j < n; j++) {
                insertS3DToAnalyzeByIndex(j, NULL);
                lastKnownPerType[j] = false;
            }
            lastUnknown = true;

            best_index = (best_type != UNKNOWN) ? objectModelMap[best_type] : 0;
            s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(best_index);

            //Determine if the type of mobile will continue to be the best one, or UNKNOWN, based on a threshold.
            if(best_type == UNKNOWN || numberOfFramesSinceFirstTimeSeen*(1 - m_knownSolutionThreshold) - unknownsNumber < 0) { //More than the m_knownSolutionThreshold of data is unknown
                best_type = UNKNOWN;
                best_subtype = ST_UNKNOWN;
                best_index = 0;
                current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
                current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;
            } else {
                if(!rigidModel[best_type]) {
                    i = objectModelMap[best_type];
                    j = objectSubModelMap[best_type][best_subtype];
                    current_min_w_model        = objectSubModelMinWidth[i][j];
                    current_max_w_model        = objectSubModelMaxWidth[i][j];
                    current_min_l_model        = objectSubModelMinLength[i][j];
                    current_max_l_model        = objectSubModelMaxLength[i][j];
                    current_min_h_model        = objectSubModelMinHeight[i][j];
                    current_max_h_model        = objectSubModelMaxHeight[i][j];
                    current_min_velocity_model = objectSubModelMinVelocity[i][j];
                    current_max_velocity_model = objectSubModelMaxVelocity[i][j];
                } else {
                    best_subtype = ST_UNKNOWN;
                    i = objectModelMap[best_type];
                    current_min_w_model       = objectModelMinWidth[i];
                    current_max_w_model       = objectModelMaxWidth[i];
                    current_min_l_model       = objectModelMinLength[i];
                    current_max_l_model       = objectModelMaxLength[i];
                    current_min_h_model       = objectModelMinHeight[i];
                    current_max_h_model       = objectModelMaxHeight[i];
                    current_min_velocity_model = objectModelMinVelocity[i];
                    current_max_velocity_model = objectModelMaxVelocity[i];
                }
            }
        }

        //Set the classification flags for data to be analysed
        for(i=0; i<currentBufferSize; i++) {
            if(    s3dsToAnalyze[i] != NULL
                && S3D_P(s3dsToAnalyze[i]) >= m_classifThreshold
                && !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                classifiedS3ds[i] = true;
                numberOfClassifiedS3ds++;
            }
            if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
                foundS3ds[i] = true;
                numberOfFoundS3ds++;
            }
        }

        RKnownSolutions  = (double)numberOfClassifiedS3ds/(double)currentBufferSize;
        RFoundSolutions  = (double)numberOfFoundS3ds     /(double)currentBufferSize;

        RVKnownSolutions = numberOfClassifiedS3ds < 2 ? 0.0 : (double)(numberOfClassifiedS3ds-1)/(double)(currentBufferSize-1);
        RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds     -1)/(double)(currentBufferSize-1);

        return;
    }

    setNumberOfFramesNotSeen(0);
    visualSupport[0] = 1.0;

    //Set blob flags for occlusion and growth limits
    m_rc->prepareBlobForRClassification(current_blob, m_pSegmentation);

    ObjectType best = UNKNOWN;
    ObjectSubtype subbest = ST_UNKNOWN;
    Shape3DData* bests3d = NULL;
    double current_sum, max_sum = -1.0;

    //Determine best for each participating type and update probability in time
    bool occlusion;

    //If the frame corresponds to the first for an object, initialize model s3ds list and probability with
    //best classification results.
    if(numberOfFramesSinceFirstTimeSeen == 1 || m_blobsBufferSize == 1) {

        Shape3DData *normal_s3d, *occ_s3d;
        debug_data_flag = 4;

        //If blob is not classified yet, do it
        if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
            m_rc->setBlob3DFacts(current_blob, m_pSegmentation);
        occ = BLOB_OCC_3DDATA(current_blob);
        occlusion = (occ == NULL) ? false : true;

        normal = BLOB_NORMAL_3DDATA(current_blob);

        for(modelMap_it = objectModelMap.begin(), normal = BLOB_NORMAL_3DDATA(current_blob); modelMap_it != objectModelMap.end(); modelMap_it++) {
            current_type = (*modelMap_it).first;
            current_index = (*modelMap_it).second;

            //In case that order of modelMap has not the same order or s3d lists of blob, the right data must be collected
            normal_s3d = getRightS3D(normal, current_type);
            if(occlusion)
                occ_s3d = getRightS3D(occ, current_type);

            //Now the lists of best frame results are stored and global probability data is stored
            if(occlusion) {
                if(S3D_P(normal_s3d) > S3D_P(occ_s3d) && S3D_P(normal_s3d) > m_classifThreshold)
                    bests3d = normal_s3d;
                else if (S3D_P(occ_s3d) > m_classifThreshold)
                    bests3d = occ_s3d;
                else
                    bests3d = NULL;
            } else if(S3D_P(normal_s3d) > m_classifThreshold)
                bests3d = normal_s3d;
            else
                bests3d = NULL;

            setS3DToAnalyzeByIndex(current_index, 0, bests3d);
            if(bests3d != NULL)
                PSumPerType[current_index] = current_sum = S3D_P(bests3d);
            else
                PSumPerType[current_index] = current_sum = 0.0;
            //Store best result
            if(current_sum > max_sum) {
                max_sum = current_sum;
                best = current_type;
                best_index = current_index;
                if(!rigidModel[best])
                    subbest = S3D_SUBTYPE(bests3d);
            }
        }

        if(!rigidModel[best])
            lastFoundSubtype[best] = subbest;

        lastUnknown = (bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold) ? false : true;

    } else { //In this case, there is more than one element to analyze, so coherency between frames must be analysed

        if(ensureMode) {

            //If currently the mobile is partially detected, part of a bigger size visual evidence, or not detected al all,
            //Generate the best 3D estimate and reflect the situation in reliability calculation
            if(non_3D_trackable_mask || not_visible_mask) {

                Shape3DData improved3DInfo;
                Rectangle<int> realBBox;

                generateS3DFrom3DInformation(&improved3DInfo);
                //double
                //    blob_support   = Rectangle<int>::rectangleIntersectRatio(&(improved3DInfo.bbox), &(current_blob->bbox)),
                //    mobile_support = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &(improved3DInfo.bbox));

                S3D_TYPE(&improved3DInfo) = best_type;
                if(rigidModel[best_type])
                    S3D_SUBTYPE(&improved3DInfo) = ST_UNKNOWN;
                else
                    S3D_SUBTYPE(&improved3DInfo) = best_subtype;
                orient3DModel(&improved3DInfo, &(current_blob->bbox), BLOB_DP_TYPE(current_blob));

                if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
                    setInitialNormalList(current_blob);

                bests3d = &(*BLOB_NORMAL_3DDATA(current_blob))[best_type];
                improved3DInfo.copy(bests3d);
                improved3DInfo.copyShape3DDataToBlob(current_blob);

                if(rigidModel[best_type])
                    objectModelsList[objectModelMap[best_type]]->computeScore(current_blob, m_context);
                else
                    objectSubModelsList[objectModelMap[best_type]][objectSubModelMap[best_type][best_subtype]]->computeScore(current_blob, m_context);
                bests3d->copyBlobToShape3DData(current_blob);

                //Reflect reliability according to type of problem.
                if(not_visible_mask) {
                    //Controled in site, by taking previous dimensional reliability weakened by time until
                    //a minimal threshold
                    visualSupport[0] = 1.0;
                } else {
                    if(currentVisualState & MM_PART_OF_BIGGER)
                        //support rate here is the coverage rate of the mobile bbox estimation over the current blob
                        visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &(improved3DInfo.bbox));
                    else if(currentVisualState & MM_PARTIALLY_DETECTED)
                        //support rate here is the coverage rate of the current blob over the mobile bbox estimation
                        visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&(improved3DInfo.bbox), &(current_blob->bbox));
                    else //This case should not happen...
                        visualSupport[0] = 1.0;
                }
                lastUnknown = false;

            } else {
                //If tracked mobile comes from an abnormal situation, reclassify if possible
                if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
                    setInitialNormalList(current_blob);
                if (    lastUnknown
                     || (currentBufferSize > 1 && (dpFlags[1] & MM_NOT_VISIBLE_MASK || dpFlags[1] & MM_NOT_3D_TRACKABLE_MASK) ) ) {
                    bests3d = generateBestLikelyS3D(current_blob, best_type);
                    lastUnknown = (bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold) ? false : true;
                } else {
                    bests3d = getBestPostureCoherentS3DReclassifying(current_blob, BLOB_NORMAL_3DDATA(current_blob), BLOB_OCC_3DDATA(current_blob), best_type);
                    if(bests3d == NULL)
                        bests3d = generateBestLikelyS3D(current_blob, best_type);
                    lastUnknown = (bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold) ? false : true;
                }

                //ACA VERIFICAR FUNCIONAMIENTO!!!
                if(false && bests3d == NULL) { //If  can not find a solution, generate it with the 3D data and reposition it

                    debug_data_flag = 5;

                    //with respect to the new bbox
                    Shape3DData improved3DInfo;
                    Rectangle<int> realBBox;

                    generateS3DFrom3DInformation(&improved3DInfo);
                    double
                        blob_support   = Rectangle<int>::rectangleIntersectRatio(&(improved3DInfo.bbox), &(current_blob->bbox)),
                        mobile_support = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &(improved3DInfo.bbox));

                    if(    blob_support >= ReliabilityTracker::m_highVisualSupportThreshold
                        && mobile_support >= ReliabilityTracker::m_highVisualSupportThreshold) {//The model itself will be able to cope with the loss of data on this frame
                        bests3d = NULL;
                        lastUnknown = true;
                    } else {
                        S3D_TYPE(&improved3DInfo) = best_type;
                        if(rigidModel[best_type])
                            S3D_SUBTYPE(&improved3DInfo) = ST_UNKNOWN;
                        else
                            S3D_SUBTYPE(&improved3DInfo) = best_subtype;
                        orient3DModel(&improved3DInfo, &(current_blob->bbox), BLOB_DP_TYPE(current_blob));

                        if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
                            setInitialNormalList(current_blob);

                        bests3d = &(*BLOB_NORMAL_3DDATA(current_blob))[best_type];
                        improved3DInfo.copy(bests3d);
                        improved3DInfo.copyShape3DDataToBlob(current_blob);

                        if(rigidModel[best_type])
                            objectModelsList[objectModelMap[best_type]]->computeScore(current_blob, m_context);
                        else
                            objectSubModelsList[objectModelMap[best_type]][objectSubModelMap[best_type][best_subtype]]->computeScore(current_blob, m_context);
                        bests3d->copyBlobToShape3DData(current_blob);

                        lastUnknown = true;
                        visualSupport[0] = blob_support < mobile_support ? blob_support : mobile_support;
                    }
                }
            }

            insertS3DToAnalyzeByIndex(best_index, bests3d);

            //Update postural information if necessary
            if(bests3d != NULL && !rigidModel[best_type]) {
                subbest = S3D_SUBTYPE(bests3d);
                if(best_subtype != subbest) {
                    best_subtype = subbest;
                    i = objectModelMap[best_type];
                    j = objectSubModelMap[best_type][best_subtype];

                    current_min_w_model        = objectSubModelMinWidth[i][j];
                    current_max_w_model        = objectSubModelMaxWidth[i][j];
                    current_min_l_model        = objectSubModelMinLength[i][j];
                    current_max_l_model        = objectSubModelMaxLength[i][j];
                    current_min_h_model        = objectSubModelMinHeight[i][j];
                    current_max_h_model        = objectSubModelMaxHeight[i][j];
                    current_min_velocity_model = objectSubModelMinVelocity[i][j];
                    current_max_velocity_model = objectSubModelMaxVelocity[i][j];
                }
            }

            if(bests3d != NULL)
                max_sum += S3D_P(bests3d);
        } else {

            debug_data_flag = 7;

            int l;

            if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
                setInitialNormalList(current_blob);

            bool bestFound = false;
            Shape3DData *current_s3d;
            normal = BLOB_NORMAL_3DDATA(current_blob);
            std::map<ObjectType, Shape3DData>::iterator n_it, n_it_end = normal->end();

            for(l = 0, n_it = normal->begin(); n_it != n_it_end; l++, n_it++) {

                current_type = n_it->first;
                current_index = l;
                current_s3d = NULL;

                if(PSumPerType[current_index] < m_classifThreshold) //If this type has been never detected
                    current_s3d = generateBestLikelyS3D(current_blob, current_type);
//                    current_s3d = NULL;
                else {
                    s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(current_index);
                    //If previous blob was not classified
                    if(s3dsToAnalyze[0] != NULL && S3D_P(s3dsToAnalyze[0]) >= m_classifThreshold) {
                        current_s3d = getBestPostureCoherentS3DReclassifying(current_blob, BLOB_NORMAL_3DDATA(current_blob), BLOB_OCC_3DDATA(current_blob), current_type);
                        if(current_s3d == NULL)
                            current_s3d = generateBestLikelyS3D(current_blob, current_type);
                    } else
                        current_s3d = generateBestLikelyS3D(current_blob, current_type);
                }

                insertS3DToAnalyzeByIndex(current_index, current_s3d);

                current_sum = PSumPerType[current_index] += current_s3d != NULL ? S3D_P(current_s3d) : 0.0;

                //Store best result
                if(current_s3d != NULL && current_sum > max_sum) {
                    max_sum = current_sum;
                    best = current_type;
                    best_index = current_index;
                    bests3d = current_s3d;
                    if(!rigidModel[best])
                        subbest = S3D_SUBTYPE(current_s3d);
                    bestFound = true;
                }
            }
            lastUnknown = bestFound && S3D_P(bests3d) >= m_classifThreshold ? false : true;
        }
    }

    s3dsToAnalyze = getS3DBufferToAnalyzeByIndex(best_index);

    if(s3dsToAnalyze[0] == NULL || S3D_P(s3dsToAnalyze[0]) < m_classifThreshold)
        unknownsNumber++;

    //Set the classification flags for data to be analysed
    for(i=0; i<currentBufferSize; i++) {
        if(    s3dsToAnalyze[i] != NULL
            && (S3D_P(s3dsToAnalyze[i]) >= m_classifThreshold)
            && ( !(S3D_OCCL_TYPE(s3dsToAnalyze[i]) & MM_NOT_VISIBLE_MASK) ) ) {
            classifiedS3ds[i] = true;
            numberOfClassifiedS3ds++;
        }
        if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            foundS3ds[i] = true;
            numberOfFoundS3ds++;
        }
    }

//    if(   s3dsToAnalyze[0] != NULL && S3D_P(s3dsToAnalyze[0]) >= m_classifThreshold
//       && numberOfClassifiedS3ds < 3)
//            classificationAllowed
    if(currentBufferSize == 1)
        RVFoundSolutions = RVKnownSolutions = 0.0;
    else {
        RVKnownSolutions = numberOfClassifiedS3ds < 2 ? 0.0 : (double)(numberOfClassifiedS3ds-1)/(double)(currentBufferSize-1);
        RVFoundSolutions = numberOfFoundS3ds      < 2 ? 0.0 : (double)(numberOfFoundS3ds     -1)/(double)(currentBufferSize-1);
    }


    //Determine if the type of mobile will be the best one, or UNKNOWN, based on a threshold.
    if(!ensureMode) {
        if(numberOfFramesSinceFirstTimeSeen*(1 - m_knownSolutionThreshold) - unknownsNumber >= 0 || RVKnownSolutions == 1.0) { //More than the m_knownSolutionThreshold of data is not unknown type
            if(best != UNKNOWN)
                best_type = best;

            if(!rigidModel[best_type]) {
                best_subtype = subbest != ST_UNKNOWN ? subbest : best_subtype;
                i = objectModelMap[best_type];
                if(best_subtype != ST_UNKNOWN) {
                    j = objectSubModelMap[best_type][best_subtype];
                    current_min_w_model        = objectSubModelMinWidth[i][j];
                    current_max_w_model        = objectSubModelMaxWidth[i][j];
                    current_min_l_model        = objectSubModelMinLength[i][j];
                    current_max_l_model        = objectSubModelMaxLength[i][j];
                    current_min_h_model        = objectSubModelMinHeight[i][j];
                    current_max_h_model        = objectSubModelMaxHeight[i][j];
                    current_min_velocity_model = objectSubModelMinVelocity[i][j];
                    current_max_velocity_model = objectSubModelMaxVelocity[i][j];
                } else {
                    current_min_w_model       = objectModelMinWidth[i];
                    current_max_w_model       = objectModelMaxWidth[i];
                    current_min_l_model       = objectModelMinLength[i];
                    current_max_l_model       = objectModelMaxLength[i];
                    current_min_h_model       = objectModelMinHeight[i];
                    current_max_h_model       = objectModelMaxHeight[i];
                    current_min_velocity_model = objectModelMinVelocity[i];
                    current_max_velocity_model = objectModelMaxVelocity[i];
                }
            } else {
                best_subtype = ST_UNKNOWN;
                i = objectModelMap[best_type];

                current_min_w_model       = objectModelMinWidth[i];
                current_max_w_model       = objectModelMaxWidth[i];
                current_min_l_model       = objectModelMinLength[i];
                current_max_l_model       = objectModelMaxLength[i];
                current_min_h_model       = objectModelMinHeight[i];
                current_max_h_model       = objectModelMaxHeight[i];
                current_min_velocity_model = objectModelMinVelocity[i];
                current_max_velocity_model = objectModelMaxVelocity[i];
            }
        } else {
            best_type = UNKNOWN;
            best_subtype = ST_UNKNOWN;
            current_min_w_model = current_min_l_model = current_min_h_model = current_min_velocity_model = 0.0;
            current_max_w_model = current_max_l_model = current_max_h_model = current_max_velocity_model = DBL_MAX;
        }
    }
}
*/
void MobileObject::setInitialNormalList(Blob *current_blob) {

    //If no blob has a non null normal list, generate one.
    Shape3DData *current_s3d;
    ObjectType current_type;
    std::map<ObjectType, Shape3DData> *S3D_list = new std::map<ObjectType, Shape3DData>();
    std::map<ObjectType, int>::iterator models_iter;
    for(models_iter = objectModelMap.begin(); models_iter != objectModelMap.end(); models_iter++) {
        current_type = (*models_iter).first;
        current_s3d = &(*S3D_list)[current_type];
        memset(current_s3d, 0, sizeof(Shape3DData));
        S3D_TYPE(current_s3d) = current_type;
//        (*S3D_list)
    }

    BLOB_NORMAL_3DDATA(current_blob) = S3D_list;


}

bool MobileObject::coherentWithRespectOfCurrent3DInformation(Shape3DData *s3d) {
    return true;
}


//Inserting Functions
void MobileObject::insertNewBlob(Blob *blob) {
    Blob *new_blob;
    previousBufferSize = blobHistory.size();
    blobHistory.insert((new_blob = blob->copyWithLists()));
    currentBufferSize = blobHistory.size();
    if(numberOfFramesSinceFirstTimeSeen <= m_2DLevelFrames)
        m_rc->setExtraPixelInfo(new_blob, BLOB_DDATA(new_blob), m_pSegmentation);
}

void MobileObject::insertNewBlob(Blob *blob, int lastMilliSecondsDifference) {
    Blob *new_blob = blob->copyWithLists();
    BLOB_TIME_DIFF_MSEC(new_blob) = lastMilliSecondsDifference;
    previousBufferSize = blobHistory.size();
    blobHistory.insert(new_blob);
    currentBufferSize = blobHistory.size();
}


//Setting Functions

void MobileObject::setGlobalProbability() {
    if(R2D + R3D + RVC + RV2DC > 0.0)
        P = (P2D*R2D + P3D*R3D + PVC*RVC + PV2DC*RV2DC) / (R2D + RV2DC + R3D + RVC);
    else
        P = 0.0;
}

void MobileObject::setMobileId(unsigned long i_id) {
    mobile_id = i_id;
}

void MobileObject::setRMobileId(unsigned long i_id) {
    rmobile_id = i_id;
}

double MobileObject::coolingFunction(double x) {
    return exp(-m_lambda*x);
}

void MobileObject::setNewMobileFromBlob(Blob *blob, unsigned long mobile_id, unsigned long rmobile_id) {
    //Functions order IS important:
    //First initialize memory of some internal processing lists
    setMobileId(mobile_id);
    setRMobileId(rmobile_id);
    setNumberOfFramesNotSeen(0);
    numberOfFramesSinceFirstTimeSeen = 1;

    //Insert the first blob into the mobile's blob buffer
    insertNewBlob(blob);

    updateMobileData();

    setGlobalProbability();

}

void MobileObject::updateMobileData(){
    P2D = 0.0;
    R2D = 0.0;
    PV2DC = 0.0;
    RV2DC = 0.0;
    //Cambiar nombre!!
    updateBestType3DInformation();

    //if(productionMode) {

        //Update General Cooling Function Values
        incrementalUpdateCooling(currentBufferSize);

        //    if(currentVisualState & MM_NOT_3D_TRACKABLE_MASK)
        //  incrementalUpdateForPartOfNonTrackableEvidence();
        //else {
        //Update the mobile 2D data
        incrementalUpdate2DDimensions(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, blobHistory.back()->maxDistanceFactor);
        incrementalUpdate2DPosition(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, blobHistory.back()->maxDistanceFactor);

        //Update the mobile 3D data if the object has passed the 2D initial test
//        if(numberOfFramesSinceFirstTimeSeen > m_2DLevelFrames && best_type != UNKNOWN) {
//        if(numberOfFramesSinceFirstTimeSeen > m_2DLevelFrames || lastUnknown == false) {
//              incrementalUpdateOrientation(currentBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
//            incrementalUpdate3DDimensions(currentBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
//            incrementalUpdate3DPosition(currentBufferSize, s3dsToAnalyze, dpFlags, visualSupport);
 //       } else //No 3D information will be considered
 //       P3D = R3D = PVC = RVC = 0.0;
    //}

}

void MobileObject::updateMobilePath(Blob *blob) {
    insertNewBlob(blob);

    updateMobileData();

    setGlobalProbability();

    //Set ensureMode = true if criterias are accomplished
    if(!ensureMode && best_type != UNKNOWN && numberOfClassifiedS3ds >= m_blobsBufferSize && P > m_probabilityToEnsureMode )
        ensureMode = true;

}

void MobileObject::incrementalUpdateCooling(int bufferSize) {

    if(bufferSize == 1) {
        iGData.sumCooling2D = foundS3ds[0] ? 1.0 : 0.0;
//        iGData.sumCooling3D = classifiedS3ds[0] ? 1.0 : 0.0;
//        if(classifiedS3ds[0])
//            iGData.totalNumClassified++;
        iGData.sumCooling3D = 0.0;
        return;
    }

    if(foundS3ds[0]) {
        iGData.prevCooling2D = iGData.sumCooling2D;
        iGData.sumCooling2D = 1.0 + coolingValue[1] * iGData.sumCooling2D;
    } else {
        iGData.prevCooling2D = iGData.sumCooling2D;
        iGData.sumCooling2D = 0.0 + coolingValue[1] * iGData.sumCooling2D;
    }

    if(iGData.prevCooling2D >= zeroTolerance && iGData.sumCooling2D < zeroTolerance)
        iGData.sumCooling2D = zeroTolerance;

//    if(classifiedS3ds[0]) {
//        iGData.totalNumClassified++;
//        iGData.prevCooling3D = iGData.sumCooling3D;
//        iGData.sumCooling3D = 1.0 + coolingValue[1] * iGData.sumCooling3D;
//    } else {
        iGData.prevCooling3D = iGData.sumCooling3D;
        iGData.sumCooling3D = 0.0 + coolingValue[1] * iGData.sumCooling3D;
//    }

//    if(iGData.prevCooling3D >= zeroTolerance && iGData.sumCooling3D < zeroTolerance)
//        iGData.sumCooling3D = zeroTolerance;

}

void MobileObject::incrementalUpdateOrientation(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport) {

    Shape3DData *current;

    //If first frame is being processed, there is no previous information about the object
    if(bufferSize == 1) {
        current = data[0];
        i3DData.alphaEstimated = t3DDimData.alpha = NormalizeOrientation(S3D_ALPHA(current));
        t3DDimData.RCalpha = 0.0;
        t3DDimData.Valpha = t3DDimData.RDValpha = t3DDimData.RCValpha = t3DDimData.RValpha = 0.0;
        t3DDimData.SDalpha = t3DDimData.SDValpha = M_PI;
        t3DDimData.RDalpha = i3DData.sumRDalpha = (S3D_RW(current) + S3D_RL(current)) / 2.0;
        t3DDimData.Ralpha = t3DDimData.RDalpha / 2.0;
        //t3DDimData.RValpha = 0.0; //BORRAR
        return;
    }

    double auxSum, curSD, curDiff;
    double RalphaEstimated;
    double currentCooling = coolingValue[1];
    double alphaData, RalphaData;
    //If last element to be added has been classified
    if(classifiedS3ds[0]) {
        current = data[0];
        alphaData = NormalizeOrientation(S3D_ALPHA(current));
        //Dimensional Reliability for Alpha
        RalphaData = visualSupport[0]*(S3D_RW(current) + S3D_RL(current)) / 2.0;

        if(t3DDimData.Ralpha < 0.0000001) { //If no classified elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            i3DData.alphaEstimated = alphaData;
            t3DDimData.RCalpha = 0.0;
            t3DDimData.SDalpha = M_PI;
            t3DDimData.RDalpha = i3DData.sumRDalpha = RalphaData;
            RalphaEstimated = t3DDimData.RDalpha / 2.0;
        } else {
            //Incremental step for denominator of alphaEstimated
            auxSum = i3DData.sumRDalpha;
            i3DData.sumRDalpha = RalphaData + currentCooling * auxSum;
            t3DDimData.RDalpha = i3DData.sumRDalpha / iGData.sumCooling3D;

            //Incremental SD Formula uses old alphaEstimated value, so computation order IS important.
            curDiff = (alphaData - i3DData.alphaEstimated) / visualSupport[0];
            curSD = t3DDimData.SDalpha;
            t3DDimData.SDalpha = sqrt( (currentCooling*auxSum/i3DData.sumRDalpha) * (curSD*curSD + RalphaData*(curDiff*curDiff/i3DData.sumRDalpha)) );

            i3DData.alphaEstimated = (alphaData*RalphaData + currentCooling*i3DData.alphaEstimated*auxSum) / i3DData.sumRDalpha;

            t3DDimData.RCalpha = DimensionalCoherenceReliability(t3DDimData.SDalpha, 0.0, M_PI);
            RalphaEstimated = RKnownSolutions * (t3DDimData.RDalpha + t3DDimData.RCalpha) / 2.0;
        }
    } else { //The current element has not been classified
        if(t3DDimData.Ralpha < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i3DData.sumRDalpha = t3DDimData.RDalpha = t3DDimData.RCalpha = RalphaEstimated = 0.0;
            t3DDimData.SDalpha = M_PI;
        } else { //If not classified and we have previous information
            //Incremental step for denominator of alphaEstimated
            i3DData.sumRDalpha = currentCooling * i3DData.sumRDalpha; //Reliability in data is diminished in time
            t3DDimData.RDalpha = i3DData.sumRDalpha / iGData.sumCooling3D;
            //alphaEstimated and SD Value will not change in this case, so RC coherence also will not change.
            RalphaEstimated = RKnownSolutions * (t3DDimData.RDalpha + t3DDimData.RCalpha) / 2.0;
        }
    }

    double
        alphaExpected  = t3DDimData.alpha + t3DDimData.Valpha*secDiffSequence[0],
        RalphaExpected = (t3DDimData.Ralpha + t3DDimData.RValpha) / 2.0;

    t3DDimData.Ralpha = RalphaExpected + RalphaEstimated;
    if(t3DDimData.Ralpha > 0.0)
        t3DDimData.alpha =  (alphaExpected*RalphaExpected + i3DData.alphaEstimated*RalphaEstimated) / t3DDimData.Ralpha;
    else if(classifiedS3ds[0])
        t3DDimData.alpha = alphaData;
    t3DDimData.Ralpha /= 2.0;

    //Data calculation related to Valpha
    double ValphaData, RDValphaData;

    //If last element to be added has been classified and we have other element too
    if(classifiedS3ds[0] && numberOfClassifiedS3ds>= 2) {
        int i;
        Shape3DData *previous;
        current = data[0];
        for(i=1; i<m_blobsBufferSize; i++)
            if(classifiedS3ds[i]) {
                previous = data[i];
                break;
            }
        ValphaData = ( alphaData - NormalizeOrientation(S3D_ALPHA(previous)) ) / secDiffToCurrent[i];
        //Dimensional Reliability for Alpha
        RDValphaData = ( RalphaData + visualSupport[i] * (S3D_RW(previous) + S3D_RL(previous)) / 2.0 ) / 2.0;

        if(t3DDimData.RValpha < 0.0000001) { //If no classified elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t3DDimData.Valpha = ValphaData;
            t3DDimData.RCValpha = 0.0;
            t3DDimData.SDValpha = M_PI;
            t3DDimData.RDValpha = i3DData.sumRDValpha = RDValphaData;
            t3DDimData.RValpha = t3DDimData.RDValpha / 2.0;
        } else {
            //Incremental step for denominator of alphaEstimated
            auxSum = i3DData.sumRDValpha;
            i3DData.sumRDValpha = RDValphaData + currentCooling * auxSum;
            t3DDimData.RDValpha = i3DData.sumRDValpha / iGData.sumCooling3D;

            //Incremental SD Formula uses old Valpha value, so computation order IS important.
            curDiff = (ValphaData - t3DDimData.Valpha) / visualSupport[0];
            curSD = t3DDimData.SDValpha;
            t3DDimData.SDValpha = sqrt( (currentCooling*auxSum/i3DData.sumRDValpha) * (curSD*curSD + RDValphaData*(curDiff*curDiff/i3DData.sumRDValpha)) );

            t3DDimData.Valpha = (ValphaData*RDValphaData + currentCooling*t3DDimData.Valpha*auxSum) / i3DData.sumRDValpha;

            t3DDimData.RCValpha = DimensionalCoherenceReliability(t3DDimData.SDValpha, 0.0, M_PI);
            t3DDimData.RValpha = RVKnownSolutions * (t3DDimData.RDValpha + t3DDimData.RCValpha) / 2.0;
        }
    } else { //The current element has not been classified or there is no enough information for determining Valpha

        if(t3DDimData.RValpha < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t3DDimData.SDalpha >  fabs(t3DDimData.Valpha))  //If standard deviation is higher than velocity
                                                               //estimation is very likely that velocity value
                                                               //is not significant. Better to take null velocity
                                                               //in this case. Else Valpha won't change.
                t3DDimData.Valpha = 0.0;

            i3DData.sumRDValpha = t3DDimData.RDValpha = t3DDimData.RCValpha = t3DDimData.RValpha = 0.0;
            t3DDimData.SDValpha = M_PI;
        } else { //If not classified and we have previous information
            //Incremental step for denominator of alphaEstimated
            auxSum = i3DData.sumRDValpha;
            i3DData.sumRDValpha = currentCooling * auxSum; //Reliability in data is diminished in time
            t3DDimData.RDValpha = i3DData.sumRDValpha / iGData.sumCooling3D;
            //Valpha and SD Value will not change in this case, so RC coherence also will not change.
            t3DDimData.RValpha = RVKnownSolutions * (t3DDimData.RDValpha + t3DDimData.RCValpha) / 2.0;
        }
    }
    //t3DDimData.RValpha = 0.0;//BORRAR

}


double MobileObject::NormalizeOrientation(double alpha) {
    return alpha < 0 ? fmod(alpha, M_PI) + M_PI : fmod(alpha, M_PI);
}

double MobileObject::minimalAngularDistance(double alpha1, double alpha2) {
    double
        na1 = NormalizeOrientation(alpha1),
        na2 = NormalizeOrientation(alpha2),
        d1 = fabs(na1 - na2), d2 = fabs( (na1 + M_PI) - na2);

    if(d1 < d2)
        return d1;

    return d2;
}


double MobileObject::NormalizeVelocityAngle(double theta) {
    return theta < 0 ? fmod(theta, 2*M_PI) + 2*M_PI : fmod(theta, 2*M_PI);
}


void MobileObject::velocityMagnitudeAccordingToModels(Shape3DData *s3d) {

    if (classifiedS3ds[0] == false) {
        t3DSpatialData.V = t3DSpatialData.SDV = m_maxSpeed;
        return;
    }

    std::map<ObjectType, SpModelInterface> *objectModels = (std::map<ObjectType, SpModelInterface> *) &(m_rc->getModelsMap());

    SpGaussianFunction gaussian = ( (*objectModels)[S3D_TYPE(s3d)]->m_mapCriteria)["velocity"].spFunction;

    t3DSpatialData.V   = gaussian->getMean();
    t3DSpatialData.SDV = gaussian->getSigma();
    t3DSpatialData.RV = 0.0;
    t3DSpatialData.PV = 1.0;
}


double MobileObject::velocityAngleAccordingToEntranceToScene(Shape3DData *s3d) {
    double x1, y1, x2, y2; // 2D coordinates to convert
    double x2_3d, y2_3d;
    DetectionProblemType camoccmask;

    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), S3D_3D_X(s3d), S3D_3D_Y(s3d), 0, &x1, &y1);

    if((camoccmask = (DetectionProblemType) (S3D_DP_TYPE(s3d) & MM_CAM_OCCL_MASK))) { //if occluded by an image border, it is easier to know the nearest borders
        y2 = y1 - 2*(S3D_DP_TYPE(s3d) & MM_CAM_OCCL_BOTTOM) + 2*(S3D_DP_TYPE(s3d) & MM_CAM_OCCL_TOP);
        x2 = x1 - 2*(S3D_DP_TYPE(s3d) & MM_CAM_OCCL_RIGHT)  + 2*(S3D_DP_TYPE(s3d) & MM_CAM_OCCL_LEFT);
    } else { //Check distance to image borders
        double ImgW = m_pSegmentation->width();
        double ImgH = m_pSegmentation->height();
        double near_criteria_W = 0.02 * ImgW;
        double near_criteria_H = 0.02 * ImgH;
        int bottom_nearer = (ImgH < 2*y1 )? 1 : 0; //condition means: ImgHeight - y1 < y1 - 0 => y1 nearer to bottom than to top
        int right_nearer  = (ImgW  < 2*x1 )? 1 : 0; //condition means: ImgWidth  - x1 < x1 - 0 => x1 nearer to right than to left
        y2 = y1 - 2*bottom_nearer*(ImgH - y1 < near_criteria_H) ? 1 : 0 + 2*(1 - bottom_nearer)*(y1 < near_criteria_H) ? 1 : 0;
        x2 = y1 - 2* right_nearer*(ImgW - x1 < near_criteria_W) ? 1 : 0 + 2*(1 - right_nearer) *(x1 < near_criteria_W) ? 1 : 0;
    }

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2, y2, 0, &x2_3d, &y2_3d);

    return fmod(atan2(y2_3d - S3D_3D_Y(s3d), x2_3d - S3D_3D_X(s3d)), 2*M_PI);
}

void MobileObject::setAprioriVelocity(Shape3DData *s3d) {
    t3DSpatialData.theta = velocityAngleAccordingToEntranceToScene(s3d);
    t3DSpatialData.SDtheta = 2*M_PI;
    velocityMagnitudeAccordingToModels(s3d);
    t3DSpatialData.Vx = t3DSpatialData.V * cos(t3DSpatialData.theta);
    t3DSpatialData.Vy = t3DSpatialData.V * sin(t3DSpatialData.theta);
    t3DSpatialData.SDVx = t3DSpatialData.SDV * t3DSpatialData.Vx/t3DSpatialData.V;
    t3DSpatialData.SDVy = t3DSpatialData.SDV * t3DSpatialData.Vy/t3DSpatialData.V;
}

//Incremental version for 3D Velocity and Position update
void MobileObject::incrementalUpdate3DPosition(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport) {

    Shape3DData *current;

    //Trivial case for an initial blobHistory
    if(bufferSize == 1) {
        current = data[0];

        i3DData.xEstimated = t3DSpatialData.x = S3D_3D_X(current);
        i3DData.yEstimated = t3DSpatialData.y = S3D_3D_Y(current);

        trajectory3D.insert(t3DSpatialData.x, t3DSpatialData.y);

        t3DSpatialData.RVx = t3DSpatialData.RVy = 0.0;
        t3DSpatialData.RCx = t3DSpatialData.RCy = t3DSpatialData.RCVx = t3DSpatialData.RCVy = 0.0;
        i3DData.sumRDVxy = t3DSpatialData.RDV = 0.0;
        t3DSpatialData.SDVx = t3DSpatialData.SDVy = t3DSpatialData.SDV = m_maxSpeed;

        if(secDiffSequence[0] > 0.0)
            t3DSpatialData.SDx = t3DSpatialData.SDy = (current_max_velocity_model - current_min_velocity_model)
                               * secDiffSequence[0];
        else
            t3DSpatialData.SDx = t3DSpatialData.SDy = (current_max_velocity_model - current_min_velocity_model)
                               * ReliabilityTracker::m_meanMillisecondsDifferenceBetweenFrames/1000.0;

        i3DData.sumRDxy = t3DSpatialData.RDxy = (S3D_RW(current) + S3D_RL(current)) / 2.0;
        t3DSpatialData.Rx = t3DSpatialData.Ry = t3DSpatialData.RDxy / 2.0;

        i3DData.sumPxy = t3DSpatialData.Ppos = (S3D_PW(current) + S3D_PL(current)) / 2.0;

        if(m_firstFrame) { //The object was already at the scene, nothing can be said
                       //about its velocity
            t3DSpatialData.theta = 0;
            t3DSpatialData.SDtheta = 2*M_PI;
            t3DSpatialData.V = t3DSpatialData.SDV = t3DSpatialData.SDVx = t3DSpatialData.SDVy = m_maxSpeed;
            t3DSpatialData.Vx = t3DSpatialData.Vy = 0;
        } else
            setAprioriVelocity(current);
        t3DSpatialData.RV = 0.0;

        PVC = 0.0;
        RVC = 0.0;
        return;
    }

    double xData, yData, RData, PData, auxSum, curSD, curDiff;
    double RxEstimated, RyEstimated;
    double min_displacement, max_displacement, currentCooling = coolingValue[1];
    min_displacement = current_min_velocity_model * secDiffSequence[0];
    max_displacement = current_max_velocity_model * secDiffSequence[0];
    //If last element to be added has been classified
    if(classifiedS3ds[0]) {
        current = data[0];
        xData = S3D_3D_X(current);
        yData = S3D_3D_Y(current);
        //Dimensional Reliability for x, and y.
        RData = visualSupport[0] * (S3D_RW(current) + S3D_RL(current)) / 2.0;
        //Dimensional Probability for x, and y.
        PData = (S3D_PW(current) + S3D_PL(current)) / 2.0;

        if(t3DSpatialData.RDxy < 0.0000001) { //If no classified elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            i3DData.xEstimated = xData;
            i3DData.yEstimated = yData;
            t3DSpatialData.RCx = t3DSpatialData.RCy = 0.0;
            //So SD must be initialized to 0.0 to restart the incremental SD function
            t3DSpatialData.SDx = t3DSpatialData.SDy = 0.0;
            t3DSpatialData.Ppos = i3DData.sumPxy = PData;
            t3DSpatialData.RDxy = i3DData.sumRDxy = RData;
            RxEstimated = RyEstimated = t3DSpatialData.RDxy / 2.0;
        } else {
            //Incremental step for denominator of xEstimated and yEstimated
            auxSum = i3DData.sumRDxy;
            i3DData.sumRDxy = RData + currentCooling * auxSum;
            t3DSpatialData.RDxy = i3DData.sumRDxy / iGData.sumCooling3D;

            i3DData.sumPxy = PData + currentCooling * i3DData.sumPxy;
            t3DSpatialData.Ppos = i3DData.sumPxy / iGData.sumCooling3D;

            //Incremental SD Formula uses old xEstimated value, so computation order IS important.
            curDiff = (xData - i3DData.xEstimated) / visualSupport[0];
            curSD = t3DSpatialData.SDx;
            t3DSpatialData.SDx = sqrt( (currentCooling*auxSum/i3DData.sumRDxy) * (curSD*curSD + RData*(curDiff*curDiff/i3DData.sumRDxy)) );
            //Incremental SD Formula uses old yEstimated value, so computation order IS important.
            curDiff = (yData - i3DData.yEstimated) / visualSupport[0];
            curSD = t3DSpatialData.SDy;
            t3DSpatialData.SDy = sqrt( (currentCooling*auxSum/i3DData.sumRDxy) * (curSD*curSD + RData*(curDiff*curDiff/i3DData.sumRDxy)) );

            i3DData.xEstimated = (xData*RData + currentCooling*i3DData.xEstimated*auxSum) / i3DData.sumRDxy;
            i3DData.yEstimated = (yData*RData + currentCooling*i3DData.yEstimated*auxSum) / i3DData.sumRDxy;

            t3DSpatialData.RCx = DimensionalCoherenceReliability(t3DSpatialData.SDx, min_displacement, max_displacement);
            t3DSpatialData.RCy = DimensionalCoherenceReliability(t3DSpatialData.SDy, min_displacement, max_displacement);

            RxEstimated = RKnownSolutions * (t3DSpatialData.RDxy + t3DSpatialData.RCx) / 2.0;
            RyEstimated = RKnownSolutions * (t3DSpatialData.RDxy + t3DSpatialData.RCy) / 2.0;
        }
    } else { //The current element has not been classified
        if(t3DSpatialData.RDxy < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i3DData.sumPxy = t3DSpatialData.Ppos = i3DData.sumRDxy = t3DSpatialData.RDxy = t3DSpatialData.RCx = RxEstimated = t3DSpatialData.RCy = RyEstimated = 0.0;
            t3DSpatialData.SDx = t3DSpatialData.SDy = max_displacement - min_displacement;
        } else { //If not classified and we have previous information
            //Incremental step for denominator of xEstimated and yEstimated
            i3DData.sumRDxy = currentCooling * i3DData.sumRDxy; //Reliability in data is diminished in time
            t3DSpatialData.RDxy = i3DData.sumRDxy / iGData.sumCooling3D;

            i3DData.sumPxy = currentCooling * i3DData.sumPxy; //Probability in data is diminished in time
            t3DSpatialData.Ppos = i3DData.sumPxy / iGData.sumCooling3D;

            //{x,y}Estimated and SD Value will not change in this case, so RC coherence also will not change.
            RxEstimated = RKnownSolutions * (t3DSpatialData.RDxy + t3DSpatialData.RCx) / 2.0;
            RyEstimated = RKnownSolutions * (t3DSpatialData.RDxy + t3DSpatialData.RCy) / 2.0;
        }
    }

    double
        tdiff = secDiffSequence[0],
        xExpected  = t3DSpatialData.x + t3DSpatialData.Vx * tdiff,
        RxExpected = (t3DSpatialData.Rx + t3DSpatialData.RVx) / 2.0,
        yExpected  = t3DSpatialData.y + t3DSpatialData.Vy * tdiff,
        RyExpected = (t3DSpatialData.Ry + t3DSpatialData.RVy) / 2.0;

    t3DSpatialData.Rx = RxExpected + RxEstimated;
    if(t3DSpatialData.Rx > 0.0)
        t3DSpatialData.x = (xExpected*RxExpected + i3DData.xEstimated*RxEstimated) / t3DSpatialData.Rx;
    else if(t3DSpatialData.Rx == 0.0 && classifiedS3ds[0])
        t3DSpatialData.x = xData;

    t3DSpatialData.Rx /= 2.0;

    t3DSpatialData.Ry = RyExpected + RyEstimated;
    if(t3DSpatialData.Ry > 0.0)
        t3DSpatialData.y =  (yExpected*RyExpected + i3DData.yEstimated*RyEstimated) / t3DSpatialData.Ry;
    else if(t3DSpatialData.Ry == 0.0 && classifiedS3ds[0])
        t3DSpatialData.y = yData;

    t3DSpatialData.Ry /= 2.0;

    trajectory3D.insert(t3DSpatialData.x, t3DSpatialData.y);

    //Data calculation related to Vx, and Vy
    double VxData, VyData, RDVData, PVData;

    //If last element to be added has been classified and we have other element too
    if(classifiedS3ds[0] && numberOfClassifiedS3ds>= 2) {
        int i;
        Shape3DData *previous;
        current = data[0];
        for(i=1; i<m_blobsBufferSize; i++)
            if(classifiedS3ds[i]) {
                previous = data[i];
                break;
            }
        VxData = ( xData - S3D_3D_X(previous) ) / secDiffToCurrent[i];
        VyData = ( yData - S3D_3D_Y(previous) ) / secDiffToCurrent[i];

        //Dimensional Reliability for Vw, Vl, Vh
        RDVData = ( RData + visualSupport[i] * (S3D_RW(previous) + S3D_RL(previous)) / 2.0 ) / 2.0;
        PVData  = ( PData + (S3D_PW(previous) + S3D_PL(previous)) / 2.0 ) / 2.0;

        if(t3DSpatialData.RDV < 0.0000001) { //If no classified elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t3DSpatialData.Vx = VxData;
            t3DSpatialData.Vy = VyData;
            t3DSpatialData.RCVx = t3DSpatialData.RCVy = 0.0;
            t3DSpatialData.SDVx = t3DSpatialData.SDVy = 0.0; //SD is initialized to restart recursion
            t3DSpatialData.RDV = i3DData.sumRDVxy = RDVData;
            t3DSpatialData.PV = i3DData.sumPVxy = PVData;
            t3DSpatialData.RVx = t3DSpatialData.RVy = RDVData / 2.0;
        } else {
            auxSum = i3DData.sumRDVxy;
            i3DData.sumRDVxy = RDVData + currentCooling * auxSum;
            t3DSpatialData.RDV = i3DData.sumRDVxy / iGData.sumCooling3D;

            i3DData.sumPVxy = PVData + currentCooling * i3DData.sumPVxy;
            t3DSpatialData.PV = i3DData.sumPVxy / iGData.sumCooling3D;

            //Incremental SD Formula uses old Vx value, so computation order IS important.
            curDiff = (VxData - t3DSpatialData.Vx) / visualSupport[0];
            curSD = t3DSpatialData.SDVx;
            t3DSpatialData.SDVx = sqrt( (currentCooling*auxSum/i3DData.sumRDVxy) * (curSD*curSD + RDVData*(curDiff*curDiff/i3DData.sumRDVxy)) );
            //Incremental SD Formula uses old Vy value, so computation order IS important.
            curDiff = (VyData - t3DSpatialData.Vy) / visualSupport[0];
            curSD = t3DSpatialData.SDVy;
            t3DSpatialData.SDVy = sqrt( (currentCooling*auxSum/i3DData.sumRDVxy) * (curSD*curSD + RDVData*(curDiff*curDiff/i3DData.sumRDVxy)) );

            t3DSpatialData.Vx = (VxData*RDVData + currentCooling*t3DSpatialData.Vx*auxSum) / i3DData.sumRDVxy;
            t3DSpatialData.Vy = (VyData*RDVData + currentCooling*t3DSpatialData.Vy*auxSum) / i3DData.sumRDVxy;

            t3DSpatialData.RCVx = DimensionalCoherenceReliability(t3DSpatialData.SDVx, current_min_velocity_model, current_max_velocity_model);
            t3DSpatialData.RCVy = DimensionalCoherenceReliability(t3DSpatialData.SDVy, current_min_velocity_model, current_max_velocity_model);

            t3DSpatialData.RVx = RVKnownSolutions * (t3DSpatialData.RDV + t3DSpatialData.RCVx) / 2.0;
            t3DSpatialData.RVy = RVKnownSolutions * (t3DSpatialData.RDV + t3DSpatialData.RCVy) / 2.0;
        }
    } else { //The current element has not been classified or there is no enough information for determining Vw, Vl, and Vh

        if(t3DSpatialData.RDV < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t3DSpatialData.SDx > fabs(t3DSpatialData.Vx))  //If standard deviation is higher than velocity
              //estimation is very likely that velocity value
              //is not significant. Better to take null velocity
              //in this case. Else Vx won't change.
              t3DSpatialData.Vx = 0.0;
            if(t3DSpatialData.SDy > fabs(t3DSpatialData.Vy))  //If standard deviation is higher than velocity
              //estimation is very likely that velocity value
              //is not significant. Better to take null velocity
              //in this case. Else Vy won't change.
              t3DSpatialData.Vy = 0.0;

            i3DData.sumRDVxy = t3DSpatialData.RCVx = t3DSpatialData.RVx = t3DSpatialData.RCVy = t3DSpatialData.RVy = 0.0;
            t3DSpatialData.SDVx = t3DSpatialData.SDVy = current_max_velocity_model - current_min_velocity_model;
        } else { //If not classified and we have previous information
            auxSum = i3DData.sumRDVxy;
            i3DData.sumRDVxy = currentCooling * auxSum; //Reliability in data is diminished in time
            t3DSpatialData.RDV = i3DData.sumRDVxy / iGData.sumCooling3D;

            i3DData.sumPVxy = currentCooling * i3DData.sumPVxy; //Probability in data is diminished in time
            t3DSpatialData.PV = i3DData.sumPVxy / iGData.sumCooling3D;

            //V{x,y} and SD Value will not change in this case, so RC coherence also will not change.
            t3DSpatialData.RVx = RVKnownSolutions * (t3DSpatialData.RDV + t3DSpatialData.RCVx) / 2.0;
            t3DSpatialData.RVy = RVKnownSolutions * (t3DSpatialData.RDV + t3DSpatialData.RCVy) / 2.0;
        }
    }

    //Decrement object speed if object is lost for long time
    if( numberOfFramesNotSeen >= 5*m_blobsBufferSize ) {
        t3DSpatialData.Vx /= 2.0;
        t3DSpatialData.Vy /= 2.0;
    }

    //Complete general Velocity information and General Coherence Probability and Reliability
    t3DSpatialData.V = sqrt(t3DSpatialData.Vx*t3DSpatialData.Vx + t3DSpatialData.Vy*t3DSpatialData.Vy);
    t3DSpatialData.RCV = (t3DSpatialData.RCVx + t3DSpatialData.RCVy) / 2.0;
    t3DSpatialData.RV = (t3DSpatialData.RCV + t3DSpatialData.RDV) / 2.0;
    t3DSpatialData.theta = NormalizeVelocityAngle(atan2(t3DSpatialData.Vy, t3DSpatialData.Vx));

    SpGaussianFunction gaussian;

    if(rigidModel[best_type])
        gaussian = (objectModelsList[objectModelMap[best_type]]->m_mapCriteria)["velocity"].spFunction;
    else
        gaussian = (objectSubModelsList[objectModelMap[best_type]][objectSubModelMap[best_type][best_subtype]]->m_mapCriteria)["velocity"].spFunction;

    if(t3DSpatialData.V < gaussian->getMin() || t3DSpatialData.V > gaussian->getMax())
        t3DSpatialData.MPV = 0.0;
    else
        t3DSpatialData.MPV = gaussian->getValue(t3DSpatialData.V);

    if(t3DSpatialData.V == 0.0) {
        t3DSpatialData.SDtheta = 0.0;
        t3DSpatialData.SDV     = (t3DSpatialData.SDVx + t3DSpatialData.SDVy) / 2.0;
    } else {
        t3DSpatialData.SDtheta = fabs(t3DSpatialData.SDVy*t3DSpatialData.Vx - t3DSpatialData.SDVx*t3DSpatialData.Vy) / (t3DSpatialData.V*t3DSpatialData.V);
        t3DSpatialData.SDV     = (t3DSpatialData.SDVx*fabs(t3DSpatialData.Vx) + t3DSpatialData.SDVy*fabs(t3DSpatialData.Vy)) / t3DSpatialData.V;
    }

    //If there is no two velocities to compare, velocities can not be trusted yet
    if(numberOfClassifiedS3ds < 3) {
        PVC = t3DSpatialData.MPV*t3DSpatialData.PV;
        RVC = 0.0;
        return;
    }

    PVC = (t3DSpatialData.MPV + t3DSpatialData.PV + t3DSpatialData.RCV) / 3.0;
    RVC = RVKnownSolutions * t3DSpatialData.RDV;
    //PV2DC = 0.0;//BORRAR
    //RV2DC = 0.0;//BORRAR
}

double MobileObject::probabilisticCoherenceReliability(double data, double mean, double sigma, double acuity) {
    if(sigma == 0.0)
        return fabs(data - mean) <= acuity ? 1.0 : 0.0;

    double diff = data - mean;
    return exp ( - diff*diff/(2*sigma*sigma) );
}

//Incremental version for 2D Position update
void MobileObject::incrementalUpdate2DPosition(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor) {

}


double MobileObject::initialSDEstimateFor2DHorizontalAttribute() {
    double Xc = t2DSpatialData.X, Yc = t2DSpatialData.Y, xc, yc, x, y, X, Y;
    double d1, d2, theta;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc, Yc, 0.0, &xc, &yc);

    //Check right image direction in 3D coordinates
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc + 5, Yc, 0.0, &x, &y);
    theta = atan2(y - yc, x - xc);
    x = xc + m_maxSpeed * cos(theta);
    y = yc + m_maxSpeed * sin(theta);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x, y, 0.0, &X, &Y);
    d1 = X - Xc;

    //Check left image direction in 3D coordinates
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc - 5, Yc, 0.0, &x, &y);
    theta = atan2(y - yc, x - xc);
    x = xc + m_maxSpeed * cos(theta);
    y = yc + m_maxSpeed * sin(theta);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x, y, 0.0, &X, &Y);
    d2 = Xc - X;

    return d1 > d2 ? d1 : d2;
}


double MobileObject::initialSDEstimateFor2DVerticalAttribute() {
    double Xc = t2DSpatialData.X, Yc = t2DSpatialData.Y, xc, yc, x, y, X, Y;
    double d1, d2, theta;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc, Yc, 0.0, &xc, &yc);

    //Check bottom image direction in 3D coordinates
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc, Yc + 5, 0.0, &x, &y);
    theta = atan2(y - yc, x - xc);
    x = xc + m_maxSpeed * cos(theta);
    y = yc + m_maxSpeed * sin(theta);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x, y, 0.0, &X, &Y);
    d1 = Y - Yc;

    //Check left image direction in 3D coordinates
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), Xc, Yc - 5, 0.0, &x, &y);
    theta = atan2(y - yc, x - xc);
    x = xc + m_maxSpeed * cos(theta);
    y = yc + m_maxSpeed * sin(theta);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x, y, 0.0, &X, &Y);
    d2 = Yc - Y;

    return d1 > d2 ? d1 : d2;
}


//Incremental version for 2D Dimensions update
void MobileObject::incrementalUpdate2DDimensions(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor = 1.0) {

}

//Incremental update of 3D dimensions
void MobileObject::incrementalUpdate3DDimensions(int bufferSize, Shape3DData **data, DetectionProblemType *dpFlags, double *visualSupport) {

    Shape3DData *current;

    //If first frame is being processed, there is no previous information about the object
    if(bufferSize == 1) {

        current = data[0];

        //For w dimension
        i3DData.wEstimated = t3DDimData.w = S3D_W(current);
        t3DDimData.Vw = t3DDimData.RDVw = t3DDimData.RCVw = t3DDimData.RVw = 0.0;
        t3DDimData.RDw = i3DData.sumRDw = visualSupport[0]*S3D_RW(current);
        t3DDimData.RCw = 0.0;
        t3DDimData.Rw = t3DDimData.RDw / 2.0;
        t3DDimData.SDVw = 0.0; //Initial incremental step
        t3DDimData.SDw = (current_max_w_model - current_min_w_model)/2.0;
        t3DDimData.Pw = i3DData.sumPw = S3D_PW(current);

        //For l dimension
        i3DData.lEstimated = t3DDimData.l = S3D_L(current);
        t3DDimData.Vl = t3DDimData.RDVl = t3DDimData.RCVl = t3DDimData.RVl = 0.0;
        t3DDimData.RDl = i3DData.sumRDl = visualSupport[0]*S3D_RL(current);
        t3DDimData.RCl = 0.0;
        t3DDimData.Rl = t3DDimData.RDl / 2.0;
        t3DDimData.SDVl = 0.0; //Initial incremental step
        t3DDimData.SDl = (current_max_l_model - current_min_l_model)/2.0;
        t3DDimData.Pl = i3DData.sumPl = S3D_PL(current);

        //For h dimension
        i3DData.hEstimated = t3DDimData.h = S3D_H(current);
        t3DDimData.Vh = t3DDimData.RDVh = t3DDimData.RCVh = t3DDimData.RVh = 0.0;
        t3DDimData.RDh = i3DData.sumRDh = visualSupport[0]*S3D_RH(current);
        t3DDimData.RCh = 0.0;
        t3DDimData.Rh = t3DDimData.RDh / 2.0;
        t3DDimData.SDVh = 0.0; //Initial incremental step
        t3DDimData.SDh = (current_max_h_model - current_min_h_model)/2.0;
        t3DDimData.Ph = i3DData.sumPh = S3D_PH(current);

        //Global Probability and reliability Measures for 3D Dimensions
        P3D = 0.0;
        R3D = 0.0;
        return;
    }

    double wData = 0, RwData, PwData, lData = 0, RlData, PlData, hData = 0, RhData, PhData, auxSum, curSD, curDiff;
    if(rigidModel[best_type]) {
        //If last element to be added has been classified
        if(classifiedS3ds[0]) {
            current = data[0];
            wData = S3D_W(current);
            lData = S3D_L(current);
            hData = S3D_H(current);
            //Dimensional Reliability for w, l, and h.
            RwData = visualSupport[0]*S3D_RW(current);
            RlData = visualSupport[0]*S3D_RL(current);
            RhData = visualSupport[0]*S3D_RH(current);
            //Dimensional Probability for w, l, and h.
            PwData = S3D_PW(current);
            PlData = S3D_PL(current);
            PhData = S3D_PH(current);

            //For w
            if(t3DDimData.Rw < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.w = wData;
                t3DDimData.RCw = 0.0;
                t3DDimData.SDw = (current_max_w_model - current_min_w_model)/2.0;
                t3DDimData.RDw = i3DData.sumRDw = RwData;
                t3DDimData.Rw = t3DDimData.RDw / 2.0;
                t3DDimData.Pw = i3DData.sumPw = PwData;
            } else {
                //Incremental step for denominator of wEstimated
                auxSum = i3DData.sumRDw;
                i3DData.sumRDw = RwData + auxSum;
                t3DDimData.RDw = i3DData.sumRDw / iGData.totalNumClassified;

                i3DData.sumPw = PwData + i3DData.sumPw;
                t3DDimData.Pw = i3DData.sumPw / iGData.totalNumClassified;

                //Incremental SD Formula uses old wEstimated value, so computation order IS important.
                curDiff = (wData - t3DDimData.w) / visualSupport[0];
                curSD = t3DDimData.SDw;
                t3DDimData.SDw = sqrt( (auxSum/i3DData.sumRDw) * (curSD*curSD + RwData*(curDiff*curDiff/i3DData.sumRDw)) );

                t3DDimData.w = (wData*RwData + t3DDimData.w*auxSum) / i3DData.sumRDw;

                t3DDimData.RCw = DimensionalCoherenceReliability(t3DDimData.SDw, current_min_w_model, current_max_w_model);
                t3DDimData.Rw = RKnownSolutions * (t3DDimData.RDw + t3DDimData.RCw) / 2.0;
            }

            //For l
            if(t3DDimData.Rl < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.l = lData;
                t3DDimData.RCl = 0.0;
                t3DDimData.SDl = (current_max_l_model - current_min_l_model)/2.0;
                t3DDimData.RDl = i3DData.sumRDl = RlData;
                t3DDimData.Rl = t3DDimData.RDl / 2.0;
                t3DDimData.Pl = i3DData.sumPl = PlData;
            } else {
                //Incremental step for denominator of lEstimated
                auxSum = i3DData.sumRDl;
                i3DData.sumRDl = RlData + auxSum;
                t3DDimData.RDl = i3DData.sumRDl / iGData.totalNumClassified;

                i3DData.sumPl = PlData + i3DData.sumPl;
                t3DDimData.Pl = i3DData.sumPl / iGData.totalNumClassified;

                //Incremental SD Formula uses old lEstimated value, so computation order IS important.
                curDiff = (lData - t3DDimData.l) / visualSupport[0];
                curSD = t3DDimData.SDl;
                t3DDimData.SDl = sqrt( (auxSum/i3DData.sumRDl) * (curSD*curSD + RlData*(curDiff*curDiff/i3DData.sumRDl)) );

                i3DData.lEstimated = (lData*RlData + t3DDimData.l*auxSum) / i3DData.sumRDl;

                t3DDimData.RCl = DimensionalCoherenceReliability(t3DDimData.SDl, current_min_l_model, current_max_l_model);
                t3DDimData.Rl = RKnownSolutions * (t3DDimData.RDl + t3DDimData.RCl) / 2.0;
            }

            //For h
            if(t3DDimData.Rh < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.h = hData;
                t3DDimData.RCh = 0.0;
                t3DDimData.SDh = (current_max_h_model - current_min_h_model)/2.0;
                t3DDimData.RDh = i3DData.sumRDh = RhData;
                t3DDimData.Rh = t3DDimData.RDh / 2.0;
                t3DDimData.Ph = i3DData.sumPh = PhData;
            } else {
                //Incremental step for denominator of hEstimated
                auxSum = i3DData.sumRDh;
                i3DData.sumRDh = RhData + auxSum;
                t3DDimData.RDh = i3DData.sumRDh / iGData.totalNumClassified;

                i3DData.sumPh = PhData + i3DData.sumPh;
                t3DDimData.Ph = i3DData.sumPh / iGData.totalNumClassified;

                //Incremental SD Formula uses old hEstimated value, so computation order IS important.
                curDiff = (hData - t3DDimData.h) / visualSupport[0];
                curSD = t3DDimData.SDh;
                t3DDimData.SDh = sqrt( (auxSum/i3DData.sumRDh) * (curSD*curSD + RhData*(curDiff*curDiff/i3DData.sumRDh)) );

                t3DDimData.h = (hData*RhData + t3DDimData.h*auxSum) / i3DData.sumRDh;

                t3DDimData.RCh = DimensionalCoherenceReliability(t3DDimData.SDh, current_min_h_model, current_max_h_model);
                t3DDimData.Rh = RKnownSolutions * (t3DDimData.RDh + t3DDimData.RCh) / 2.0;
            }
        } else { //The current element has not been classified
            //For w
            if(t3DDimData.Rw < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                t3DDimData.w = wData;
                i3DData.sumPw = t3DDimData.Pw = i3DData.sumRDw = t3DDimData.RDw = t3DDimData.RCw = t3DDimData.Rw = 0.0;
                t3DDimData.SDw = (current_max_w_model - current_min_w_model)/2.0;
            }
            //For l
            if(t3DDimData.Rl < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                t3DDimData.l = lData;
                i3DData.sumPl = t3DDimData.Pl = i3DData.sumRDl = t3DDimData.RDl = t3DDimData.RCl = t3DDimData.Rl = 0.0;
                t3DDimData.SDl = (current_max_l_model - current_min_l_model)/2.0;
            }
            //For h
            if(t3DDimData.Rh < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                t3DDimData.h = hData;
                i3DData.sumPh = t3DDimData.Ph = i3DData.sumRDh = t3DDimData.RDh = t3DDimData.RCh = t3DDimData.Rh = 0.0;
                t3DDimData.SDh = (current_max_h_model - current_min_h_model)/2.0;
            }
        }

        //No dimension change speed information in rigid model
        t3DDimData.Vw   = t3DDimData.RVw  = t3DDimData.RCVw = t3DDimData.RDVw = 0.0;
        t3DDimData.Vl   = t3DDimData.RVl  = t3DDimData.RCVl = t3DDimData.RDVl = 0.0;
        t3DDimData.Vh   = t3DDimData.RVh  = t3DDimData.RCVh = t3DDimData.RDVh = 0.0;
        t3DDimData.SDVw = t3DDimData.SDVl = t3DDimData.SDVh = 0.0;
    } else { //Postural dimensions
        double RwEstimated, RlEstimated, RhEstimated;
        double currentCooling = coolingValue[1];
        //If last element to be added has been classified
        if(classifiedS3ds[0]) {
            current = data[0];
            wData = S3D_W(current);
            lData = S3D_L(current);
            hData = S3D_H(current);
            //Dimensional Reliability for w, l, and h.
            RwData = visualSupport[0]*S3D_RW(current);
            RlData = visualSupport[0]*S3D_RL(current);
            RhData = visualSupport[0]*S3D_RH(current);
            //Dimensional Probability for w, l, and h.
            PwData = S3D_PW(current);
            PlData = S3D_PL(current);
            PhData = S3D_PH(current);

            //For w
            if(t3DDimData.Rw < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                i3DData.wEstimated = wData;
                t3DDimData.RCw = 0.0;
                t3DDimData.SDw = (current_max_w_model - current_min_w_model)/2.0; //Reinitialize incremental value
                t3DDimData.RDw = i3DData.sumRDw = RwData;
                t3DDimData.Pw = i3DData.sumPw = PwData;
                RwEstimated = t3DDimData.RDw / 2.0;
            } else {
                //Incremental step for denominator of wEstimated
                auxSum = i3DData.sumRDw;
                i3DData.sumRDw = RwData + currentCooling * auxSum;
                t3DDimData.RDw = i3DData.sumRDw / iGData.sumCooling3D;

                i3DData.sumPw = PwData + currentCooling * i3DData.sumPw;
                t3DDimData.Pw = i3DData.sumPw / iGData.sumCooling3D;

                //Incremental SD Formula uses old wEstimated value, so computation order IS important.
                curDiff = (wData - i3DData.wEstimated) / visualSupport[0];
                curSD = t3DDimData.SDw;
                t3DDimData.SDw = sqrt( (currentCooling*auxSum/i3DData.sumRDw) * (curSD*curSD + RwData*(curDiff*curDiff/i3DData.sumRDw)) );

                i3DData.wEstimated = (wData*RwData + currentCooling*i3DData.wEstimated*auxSum) / i3DData.sumRDw;

                t3DDimData.RCw = DimensionalCoherenceReliability(t3DDimData.SDw, current_min_w_model, current_max_w_model);
                RwEstimated = RKnownSolutions * (t3DDimData.RDw + t3DDimData.RCw) / 2.0;
            }

            //For l
            if(t3DDimData.Rl < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                i3DData.lEstimated = lData;
                t3DDimData.RCl = 0.0;
                t3DDimData.SDl = (current_max_l_model - current_min_l_model)/2.0; //Reinitialize incremental value
                t3DDimData.RDl = i3DData.sumRDl = RlData;
                t3DDimData.Pl = i3DData.sumPl = PlData;
                RlEstimated = t3DDimData.RDl / 2.0;
            } else {
                //Incremental step for denominator of lEstimated
                auxSum = i3DData.sumRDl;
                i3DData.sumRDl = RlData + currentCooling * auxSum;
                t3DDimData.RDl = i3DData.sumRDl / iGData.sumCooling3D;

                i3DData.sumPl = PlData + currentCooling * i3DData.sumPl;
                t3DDimData.Pl = i3DData.sumPl / iGData.sumCooling3D;

                //Incremental SD Formula uses old lEstimated value, so computation order IS important.
                curDiff = (lData - i3DData.lEstimated) / visualSupport[0];
                curSD = t3DDimData.SDl;
                t3DDimData.SDl = sqrt( (currentCooling*auxSum/i3DData.sumRDl) * (curSD*curSD + RlData*(curDiff*curDiff/i3DData.sumRDl)) );

                i3DData.lEstimated = (lData*RlData + currentCooling*i3DData.lEstimated*auxSum) / i3DData.sumRDl;

                t3DDimData.RCl = DimensionalCoherenceReliability(t3DDimData.SDl, current_min_l_model, current_max_l_model);
                RlEstimated = RKnownSolutions * (t3DDimData.RDl + t3DDimData.RCl) / 2.0;
            }

            //For h
            if(t3DDimData.Rh < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                i3DData.hEstimated = hData;
                t3DDimData.RCh = 0.0;
                t3DDimData.SDh = (current_max_h_model - current_min_h_model)/2.0; //Reinitialize incremental value
                t3DDimData.RDh = i3DData.sumRDh = RhData;
                t3DDimData.Ph = i3DData.sumPh = PhData;
                RhEstimated = t3DDimData.RDh / 2.0;
            } else {
                //Incremental step for denominator of hEstimated
                auxSum = i3DData.sumRDh;
                i3DData.sumRDh = RhData + currentCooling * auxSum;
                t3DDimData.RDh = i3DData.sumRDh / iGData.sumCooling3D;

                i3DData.sumPh = PhData + currentCooling * i3DData.sumPh;
                t3DDimData.Ph = i3DData.sumPh / iGData.sumCooling3D;

                //Incremental SD Formula uses old hEstimated value, so computation order IS important.
                curDiff = (hData - i3DData.hEstimated) / visualSupport[0];
                curSD = t3DDimData.SDh;
                t3DDimData.SDh = sqrt( (currentCooling*auxSum/i3DData.sumRDh) * (curSD*curSD + RhData*(curDiff*curDiff/i3DData.sumRDh)) );

                i3DData.hEstimated = (hData*RhData + currentCooling*i3DData.hEstimated*auxSum) / i3DData.sumRDh;

                t3DDimData.RCh = DimensionalCoherenceReliability(t3DDimData.SDh, current_min_h_model, current_max_h_model);
                RhEstimated = RKnownSolutions * (t3DDimData.RDh + t3DDimData.RCh) / 2.0;
            }
        } else { //The current element has not been classified
            //For w
            if(t3DDimData.Rw < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                i3DData.sumPw = t3DDimData.Pw = i3DData.sumRDw = t3DDimData.RDw = t3DDimData.RCw = RwEstimated = 0.0;
                t3DDimData.SDw = (current_max_w_model - current_min_w_model)/2.0;
            } else { //If not classified and we have previous information
                //Incremental step for denominator of wEstimated
                i3DData.sumRDw = currentCooling * i3DData.sumRDw; //Reliability in data is diminished in time
                t3DDimData.RDw = i3DData.sumRDw / iGData.sumCooling3D;

                i3DData.sumPw = currentCooling * i3DData.sumPw; //Probability in data is diminished in time
                t3DDimData.Pw = i3DData.sumPw / iGData.sumCooling3D;

                //wEstimated and SD Value will not change in this case, so RC coherence also will not change.
                RwEstimated = RKnownSolutions * (t3DDimData.RDw + t3DDimData.RCw) / 2.0;
            }
            //For l
            if(t3DDimData.Rl < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                i3DData.sumPl = t3DDimData.Pl = i3DData.sumRDl = t3DDimData.RDl = t3DDimData.RCl = RlEstimated = 0.0;
                t3DDimData.SDl = (current_max_l_model - current_min_l_model)/2.0;
            } else { //If not classified and we have previous information
                //Incremental step for denominator of lEstimated
                i3DData.sumRDl = currentCooling * i3DData.sumRDl; //Reliability in data is diminished in time
                t3DDimData.RDl = i3DData.sumRDl / iGData.sumCooling3D;

                i3DData.sumPl = currentCooling * i3DData.sumPl; //Probability in data is diminished in time
                t3DDimData.Pl = i3DData.sumPl / iGData.sumCooling3D;

                //lEstimated and SD Value will not change in this case, so RC coherence also will not change.
                RlEstimated = RKnownSolutions * (t3DDimData.RDl + t3DDimData.RCl) / 2.0;
            }
            //For h
            if(t3DDimData.Rh < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                i3DData.sumPh = t3DDimData.Ph = i3DData.sumRDh = t3DDimData.RDh = t3DDimData.RCh = RhEstimated = 0.0;
                t3DDimData.SDh = (current_max_h_model - current_min_h_model)/2.0;
            } else { //If not classified and we have previous information
                //Incremental step for denominator of hEstimated
                i3DData.sumRDh = currentCooling * i3DData.sumRDh; //Reliability in data is diminished in time
                t3DDimData.RDh = i3DData.sumRDh / iGData.sumCooling3D;

                i3DData.sumPh = currentCooling * i3DData.sumPh; //Probability in data is diminished in time
                t3DDimData.Ph = i3DData.sumPh / iGData.sumCooling3D;

                //hEstimated and SD Value will not change in this case, so RC coherence also will not change.
                RhEstimated = RKnownSolutions * (t3DDimData.RDh + t3DDimData.RCh) / 2.0;
            }
        }

        double
            tdiff = secDiffSequence[0],
            wExpected  = t3DDimData.w + t3DDimData.Vw * tdiff,
            RwExpected = (t3DDimData.Rw + t3DDimData.RVw) / 2.0,
            lExpected  = t3DDimData.l + t3DDimData.Vl * tdiff,
            RlExpected = (t3DDimData.Rl + t3DDimData.RVl) / 2.0,
            hExpected  = t3DDimData.h + t3DDimData.Vh * tdiff,
            RhExpected = (t3DDimData.Rh + t3DDimData.RVh) / 2.0;

        t3DDimData.Rw = RwExpected + RwEstimated;
        if(t3DDimData.Rw > 0.0)
            t3DDimData.w =  (wExpected*RwExpected + i3DData.wEstimated*RwEstimated) / t3DDimData.Rw;
        else if(t3DDimData.Rw == 0.0 && classifiedS3ds[0])
            t3DDimData.w = wData;
        t3DDimData.Rw /= 2.0;

        if(t3DDimData.w < objectModelMinWidth[best_index])
            t3DDimData.w = objectModelMinWidth[best_index];
        if(t3DDimData.w > objectModelMaxWidth[best_index])
            t3DDimData.w = objectModelMaxWidth[best_index];

        t3DDimData.Rl = RlExpected + RlEstimated;
        if(t3DDimData.Rl > 0.0)
            t3DDimData.l =  (lExpected*RlExpected + i3DData.lEstimated*RlEstimated) / t3DDimData.Rl;
        else if(t3DDimData.Rl == 0.0 && classifiedS3ds[0])
            t3DDimData.l = lData;

        t3DDimData.Rl /= 2.0;

        if(t3DDimData.l < objectModelMinLength[best_index])
            t3DDimData.l = objectModelMinLength[best_index];
        if(t3DDimData.l > objectModelMaxLength[best_index])
            t3DDimData.l = objectModelMaxLength[best_index];

        t3DDimData.Rh = RhExpected + RhEstimated;
        if(t3DDimData.Rh > 0.0)
            t3DDimData.h =  (hExpected*RhExpected + i3DData.hEstimated*RhEstimated) / t3DDimData.Rh;
        else if(t3DDimData.Rh == 0.0 && classifiedS3ds[0])
            t3DDimData.h = hData;
        t3DDimData.Rh /= 2.0;

        if(t3DDimData.h < objectModelMinHeight[best_index])
            t3DDimData.h = objectModelMinHeight[best_index];
        if(t3DDimData.h > objectModelMaxHeight[best_index])
            t3DDimData.h = objectModelMaxHeight[best_index];

        //Data calculation related to Vw, Vl and Vh
        double VwData, RDVwData, VlData, RDVlData, VhData, RDVhData;

        //If last element to be added has been classified and we have other element too
        if(classifiedS3ds[0] && numberOfClassifiedS3ds>= 2) {

            int i;
            Shape3DData *previous;
            current = data[0];
            for(i=1; i<m_blobsBufferSize; i++)
                if(classifiedS3ds[i]) {
                    previous = data[i];
                    break;
                }

            VwData = ( wData - S3D_W(previous) ) / secDiffToCurrent[i];
            VlData = ( lData - S3D_L(previous) ) / secDiffToCurrent[i];
            VhData = ( hData - S3D_H(previous) ) / secDiffToCurrent[i];
            //Dimensional Reliability for Vw, Vl, Vh
            RDVwData = ( RwData + visualSupport[i]*S3D_RW(previous) ) / 2.0;
            RDVlData = ( RlData + visualSupport[i]*S3D_RL(previous) ) / 2.0;
            RDVhData = ( RhData + visualSupport[i]*S3D_RH(previous) ) / 2.0;

            //For w
            if(t3DDimData.RVw < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.Vw = VwData;
                t3DDimData.RDVw = t3DDimData.RCVw = 0.0;
                t3DDimData.SDVw = 0.0; //Reinitialize incremental SD
                i3DData.sumRDVw = RDVwData;
                t3DDimData.RVw = RDVwData / 2.0;
            } else {
                auxSum = i3DData.sumRDVw;
                i3DData.sumRDVw = RDVwData + currentCooling * auxSum;
                t3DDimData.RDVw = i3DData.sumRDVw / iGData.sumCooling3D;

                //Incremental SD Formula uses old Vw value, so computation order IS important.
                curDiff = (VwData - t3DDimData.Vw) / visualSupport[0];
                curSD = t3DDimData.SDVw;
                t3DDimData.SDVw = sqrt( (currentCooling*auxSum/i3DData.sumRDVw) * (curSD*curSD + RDVwData*(curDiff*curDiff/i3DData.sumRDVw)) );

                t3DDimData.Vw = (VwData*RDVwData + currentCooling*t3DDimData.Vw*auxSum) / i3DData.sumRDVw;

                t3DDimData.RCVw = DimensionalCoherenceReliability(t3DDimData.SDVw, 0.0, m_Maximal3DDimensionChangeSpeed);
                t3DDimData.RVw = RVKnownSolutions * (t3DDimData.RDVw + t3DDimData.RCVw) / 2.0;
            }

            //For l
            if(t3DDimData.RVl < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.Vl = VlData;
                t3DDimData.RDVl = t3DDimData.RCVl = 0.0;
                t3DDimData.SDVl = 0.0; //Reinitialize incremental SD
                i3DData.sumRDVl = RDVlData;
                t3DDimData.RVl = RDVlData / 2.0;
            } else {
                auxSum = i3DData.sumRDVl;
                i3DData.sumRDVl = RDVlData + currentCooling * auxSum;
                t3DDimData.RDVl = i3DData.sumRDVl / iGData.sumCooling3D;

                //Incremental SD Formula uses old Vl value, so computation order IS important.
                curDiff = (VlData - t3DDimData.Vl) / visualSupport[0];
                curSD = t3DDimData.SDVl;
                t3DDimData.SDVl = sqrt( (currentCooling*auxSum/i3DData.sumRDVl) * (curSD*curSD + RDVlData*(curDiff*curDiff/i3DData.sumRDVl)) );

                t3DDimData.Vl = (VlData*RDVlData + currentCooling*t3DDimData.Vl*auxSum) / i3DData.sumRDVl;

                t3DDimData.RCVl = DimensionalCoherenceReliability(t3DDimData.SDVl, 0.0, m_Maximal3DDimensionChangeSpeed);
                t3DDimData.RVl = RVKnownSolutions * (t3DDimData.RDVl + t3DDimData.RCVl) / 2.0;
            }

            //For h
            if(t3DDimData.RVh < 0.0000001) { //If no classified elements has been already added in later frames
                //In case of new classified element after a period of no classification, the new classified value is considered.
                t3DDimData.Vh = VhData;
                t3DDimData.RDVh = t3DDimData.RCVh = 0.0;
                t3DDimData.SDVh = 0.0; //Reinitialize incremental SD
                i3DData.sumRDVh = RDVhData;
                t3DDimData.RVh = RDVhData / 2.0;
            } else {
                auxSum = i3DData.sumRDVh;
                i3DData.sumRDVh = RDVhData + currentCooling * auxSum;
                t3DDimData.RDVh = i3DData.sumRDVh / iGData.sumCooling3D;

                //Incremental SD Formula uses old Vh value, so computation order IS important.
                curDiff = (VhData - t3DDimData.Vh) / visualSupport[0];
                curSD = t3DDimData.SDVh;
                t3DDimData.SDVh = sqrt( (currentCooling*auxSum/i3DData.sumRDVh) * (curSD*curSD + RDVhData*(curDiff*curDiff/i3DData.sumRDVh)) );

                t3DDimData.Vh = (VhData*RDVhData + currentCooling*t3DDimData.Vh*auxSum) / i3DData.sumRDVh;

                t3DDimData.RCVh = DimensionalCoherenceReliability(t3DDimData.SDVh, 0.0, m_Maximal3DDimensionChangeSpeed);
                t3DDimData.RVh = RVKnownSolutions * (t3DDimData.RDVh + t3DDimData.RCVh) / 2.0;
            }
        } else { //The current element has not been classified or there is no enough information for determining Vw, Vl, and Vh

            //For w
            if(t3DDimData.RVw < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                if(t3DDimData.SDw > fabs(t3DDimData.Vw))  //If standard deviation is higher than velocity
                    //estimation is very likely that velocity value
                    //is not significant. Better to take null velocity
                    //in this case. Else Vw won't change.
                    t3DDimData.Vw = 0.0;

                i3DData.sumRDVw = t3DDimData.RDVw = t3DDimData.RCVw = t3DDimData.RVw = 0.0;
                t3DDimData.SDVw = m_Maximal3DDimensionChangeSpeed;
            } else { //If not classified and we have previous information
                auxSum = i3DData.sumRDVw;
                i3DData.sumRDVw = currentCooling * auxSum; //Reliability in data is diminished in time
                t3DDimData.RDVw = i3DData.sumRDVw / iGData.sumCooling3D;
                //Vw and SD Value will not change in this case, so RC coherence also will not change.
                t3DDimData.RVw = RVKnownSolutions * (t3DDimData.RDVw + t3DDimData.RCVw) / 2.0;
            }

            //For l
            if(t3DDimData.RVl < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                if(t3DDimData.SDl > fabs(t3DDimData.Vl))  //If standard deviation is higher than velocity
                    //estimation is very likely that velocity value
                    //is not significant. Better to take null velocity
                    //in this case. Else Vl won't change.
                    t3DDimData.Vl = 0.0;

                i3DData.sumRDVl = t3DDimData.RDVl = t3DDimData.RCVl = t3DDimData.RVl = 0.0;
                t3DDimData.SDVl = m_Maximal3DDimensionChangeSpeed;
            } else { //If not classified and we have previous information
                auxSum = i3DData.sumRDVl;
                i3DData.sumRDVl = currentCooling * auxSum; //Reliability in data is diminished in time
                t3DDimData.RDVl = i3DData.sumRDVl / iGData.sumCooling3D;
                //Vl and SD Value will not change in this case, so RC coherence also will not change.
                t3DDimData.RVl = RVKnownSolutions * (t3DDimData.RDVl + t3DDimData.RCVl) / 2.0;
            }

            //For h
            if(t3DDimData.RVh < 0.0000001) { //If no classified elements has been already added in later frames
                //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
                //data obtention
                if(t3DDimData.SDh > fabs(t3DDimData.Vh))  //If standard deviation is higher than velocity
                    //estimation is very likely that velocity value
                    //is not significant. Better to take null velocity
                    //in this case. Else Vh won't change.
                    t3DDimData.Vh = 0.0;

                i3DData.sumRDVh = t3DDimData.RDVh = t3DDimData.RCVh = t3DDimData.RVh = 0.0;
                t3DDimData.SDVh = m_Maximal3DDimensionChangeSpeed;
            } else { //If not classified and we have previous information
                auxSum = i3DData.sumRDVh;
                i3DData.sumRDVh = currentCooling * auxSum; //Reliability in data is diminished in time
                t3DDimData.RDVh = i3DData.sumRDVh / iGData.sumCooling3D;
                //Vh and SD Value will not change in this case, so RC coherence also will not change.
                t3DDimData.RVh = RVKnownSolutions * (t3DDimData.RDVh + t3DDimData.RCVh) / 2.0;
            }
        }
    }

    //Determine 3D criteria for global probability
    R3D = t3DDimData.RDl + t3DDimData.RDw + t3DDimData.RDh;
    if(R3D == 0.0 || RKnownSolutions == 0.0)
        P3D = R3D = 0.0;
    else {
        //    Mean model probability * Mean coherence probability * Mean Model Reliability
        P3D = (   (t3DDimData.Pl + t3DDimData.RCl)*t3DDimData.RDl
                + (t3DDimData.Pw + t3DDimData.RCw)*t3DDimData.RDw
                + (t3DDimData.Ph + t3DDimData.RCh)*t3DDimData.RDh ) / (2.0 * R3D);
        R3D /= 3.0;
        R3D *= RKnownSolutions;
    }
    //R3D = 0.0; //BORRAR
}


double MobileObject::DimensionalCoherenceReliability(double sigma_dim, double min, double max) {
    return  1.0 - std::min(1.0, sigma_dim/(max - min));
}

double MobileObject::dimensional2DReliability(double distance2D, double blobW, double blobH) {
    //The farest the object is, it is more likely to loose detected pixels.
    double Rdistance = 1.0 - std::min(1.0, distance2D/m_maxFocalDistance);
    //Biggest objects are less affected by segmentation errors.
    double Rsize =  std::min(1.0, blobW*blobH/m_objectSizeForMaxReliability);
    return (Rdistance + Rsize) / 2.0 ;
}

double MobileObject::position2DReliability(double distance2D) {
    //The farest the object is, it is more likely to loose precision.
    //The value is normalized with a constant representing the maximal posible 2D displacement
    //for the quickest known object (m_objectDistanceForMaxReliability)
    if (distance2D == 0.0)
        return 1.0;
    double Rdistance = std::min(1.0, m_objectDistanceForMaxReliability/distance2D);
    return Rdistance;
}


double MobileObject::get2DDistanceToFocalPoint(Blob *blob) {
    double Xf = SM_CAMERA_X2D_FOC_POINT(m_context), Yf = SM_CAMERA_Y2D_FOC_POINT(m_context);
    int W = m_pSegmentation->width(), H = m_pSegmentation->height();
    double dX, dY;

    if(Yf < 0) {
        Xf = W/2.0;
        Yf = 0.0;
    } else if (Yf > H) {
        Xf = W/2.0;
        Yf = H;
    }

    //Get Nearest Point
    switch(BLOB_POSITION(blob)) {
        case 0:
            dX = BLOB_XRIGHT(blob) - Xf;
            dY = BLOB_YBOTTOM(blob) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 1:
            return fabs(BLOB_YBOTTOM(blob) - Yf);
            break;
        case 2:
            dX = BLOB_XLEFT(blob) - Xf;
            dY = BLOB_YBOTTOM(blob) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 3:
            return fabs(BLOB_XRIGHT(blob) - Xf);
            break;
        case 4:
            return 0.0;
            break;
        case 5:
            return fabs(BLOB_XLEFT(blob) - Xf);
            break;
        case 6:
            dX = BLOB_XRIGHT(blob) - Xf;
            dY = BLOB_YTOP(blob) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 7:
            return fabs(BLOB_YTOP(blob) - Yf);
            break;
        case 8:
            dX = BLOB_XLEFT(blob) - Xf;
            dY = BLOB_YTOP(blob) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        default:
            return 0.0;
    }

    return 0.0;
}

double MobileObject::get2DDistanceToFocalPoint(Rectangle<int> *rectangle) {
    double Xf = SM_CAMERA_X2D_FOC_POINT(m_context), Yf = SM_CAMERA_Y2D_FOC_POINT(m_context);
    double dX, dY;
    int position = rectangle->getPositionRelativeToCamera(m_context);
    //Get Nearest Point
    switch(position) {
        case 0:
            dX = RECT_XRIGHT(rectangle) - Xf;
            dY = RECT_YBOTTOM(rectangle) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 1:
            return fabs(RECT_YBOTTOM(rectangle) - Yf);
            break;
        case 2:
            dX = RECT_XLEFT(rectangle) - Xf;
            dY = RECT_YBOTTOM(rectangle) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 3:
            return fabs(RECT_XRIGHT(rectangle) - Xf);
            break;
        case 4:
            return 0.0;
            break;
        case 5:
            return fabs(RECT_XLEFT(rectangle) - Xf);
            break;
        case 6:
            dX = RECT_XRIGHT(rectangle) - Xf;
            dY = RECT_YTOP(rectangle) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        case 7:
            return fabs(RECT_YTOP(rectangle) - Yf);
            break;
        case 8:
            dX = RECT_XLEFT(rectangle) - Xf;
            dY = RECT_YTOP(rectangle) - Yf;
            return sqrt(dX*dX + dY*dY);
            break;
        default:
            return 0.0;
    }

    return 0.0;
}


double MobileObject::get2DDistanceToFocalPoint(double X, double Y) {
    double
        Xf = SM_CAMERA_X2D_FOC_POINT(m_context),
        Yf = SM_CAMERA_Y2D_FOC_POINT(m_context),
        dX = Xf - X,
        dY = Yf - Y;

    return sqrt(dX*dX + dY*dY);
}

void MobileObject::setNumberOfFramesNotSeen(int num) {
    numberOfFramesNotSeen = num;
}

void MobileObject::incrementNumberOfFramesNotSeen() {
    numberOfFramesNotSeen++;
}

//2D bounding box information infered from best quality mobile information
Blob *MobileObject::determineMostLikelyBlob() {
    Blob *newBlob = ((*blobHistory.rbegin()))->copyWithLists();

    //Trustable 3D information, infer best blob from 3D information
    if(    RVC >= m_SpatialCoherenceReliabilityThreshold && PVC >= m_SpatialCoherenceProbabilityThreshold
        && R3D >= m_DimensionalCoherenceReliabilityThreshold && P3D >= m_DimensionalCoherenceProbabilityThreshold) {
        double sina, cosa, x, y;

        sina = sin(t3DDimData.alpha);
        cosa = cos(t3DDimData.alpha);

        x = t3DSpatialData.x + t3DSpatialData.Vx*secDiffSequence[0];
        y = t3DSpatialData.y + t3DSpatialData.Vy*secDiffSequence[0];

        int beta_direction = ReliabilityTracker::getBetaDirection(m_context, m_pSegmentation);

        //Starting point is the point 1 of the 3D bounding box, obtained repositioning from the blob 3D position.
        BLOB_3DBBOX(newBlob)->getFromInitial3DPoint(m_context, BLOB_BBOX(newBlob),
                                                    x + beta_direction*(t3DDimData.w*sina - t3DDimData.l*cosa)/2.0,
                                                    y - (t3DDimData.w*cosa + t3DDimData.l*sina)/2.0,
                                                    1, t3DDimData.alpha, beta_direction, t3DDimData.w, t3DDimData.l, t3DDimData.h);
        BLOB_POSITION(newBlob) = newBlob->getPositionRelativeToCamera(m_context);
        BLOB_3D_WIDTH(newBlob) = t3DDimData.w;
        BLOB_3D_HEIGHT(newBlob) = t3DDimData.h;
        BLOB_3D_LENGTH(newBlob) = t3DDimData.l;
        BLOB_ALPHA(newBlob) = t3DDimData.alpha;

        BLOB_3D_X(newBlob) = x;
        BLOB_3D_Y(newBlob) = y;

        BLOB_PW(newBlob) = t3DDimData.Pw;
        BLOB_PH(newBlob) = t3DDimData.Ph;
        BLOB_PL(newBlob) = t3DDimData.Pl;
        BLOB_RW(newBlob) = t3DDimData.RDw;
        BLOB_RH(newBlob) = t3DDimData.RDh;
        BLOB_RL(newBlob) = t3DDimData.RDl;
        BLOB_P(newBlob) = t3DDimData.Pw * t3DDimData.Pl * t3DDimData.Ph;

        //Update pertinent s3d data
        //Just one list is enough, because is just for completion of information.
        BLOB_DP_TYPE(newBlob) = MM_DP_NONE;
        if(BLOB_OCC_3DDATA(newBlob) != NULL) {
            delete BLOB_OCC_3DDATA(newBlob);
            BLOB_OCC_3DDATA(newBlob) = NULL ;
        }

        std::map<ObjectType, Shape3DData> *normal = BLOB_NORMAL_3DDATA(newBlob);
        std::map<ObjectType, Shape3DData>::iterator n_it, n_it_end = normal->end();
        std::map<ObjectSubtype, Shape3DData> *subnormal;
        std::map<ObjectSubtype, Shape3DData>::iterator sn_it, sn_it_end;
        //In this case just copy 2D most likely mobile information
        if(best_type == UNKNOWN) {
            BLOB_TYPE(newBlob) = UNKNOWN;
            BLOB_SUBTYPE(newBlob) = ST_UNKNOWN;
            for(n_it = normal->begin(); n_it != n_it_end; n_it++) {
                subnormal = S3D_SUBTYPES_LIST(&n_it->second);
                if(subnormal != NULL) {
                    sn_it_end = subnormal->end();
                    for(sn_it = subnormal->begin(); sn_it != sn_it_end; sn_it++)
                        sn_it->second.setNull(newBlob);
                }
                n_it->second.setNull(newBlob);
            }
        } else { //Classified blob
            BLOB_TYPE(newBlob) = best_type;
            BLOB_SUBTYPE(newBlob) = best_subtype;
            for(n_it = normal->begin(); n_it != n_it_end; n_it++) {
                if(!rigidModel[n_it->first]) {
                    subnormal = S3D_SUBTYPES_LIST(&n_it->second);
                    if(subnormal != NULL) {
                        sn_it_end = subnormal->end();
                        for(sn_it = subnormal->begin(); sn_it != sn_it_end; sn_it++)
                            if(sn_it->first == best_subtype)
                                sn_it->second.copyBlobToShape3DData(newBlob);
                            else
                                sn_it->second.setNull(newBlob);
                    }
                }

                if(n_it->first == best_type)
                    n_it->second.copyBlobToShape3DData(newBlob);
                else
                    n_it->second.setNull(newBlob);
            }
        }
    } else { //Use 2D position information
        double X, Y,
        //Use 2D Width and Height information, redardless its coherence, because it is all we have
        W_2 = t2DDimData.W / 2.0,
        H_2 = t2DDimData.H / 2.0;

        //Use 2D velocity information if trustable enough
        if ( RV2DC > m_SpatialCoherenceReliabilityThreshold && PV2DC > m_SpatialCoherenceProbabilityThreshold ) {
            X = t2DSpatialData.X + t2DSpatialData.VX * secDiffSequence[0];
            Y = t2DSpatialData.Y + t2DSpatialData.VY * secDiffSequence[0];
        } else { //Use last 2D position information
            X = t2DSpatialData.X;
            Y = t2DSpatialData.Y;
        }

        BLOB_YBOTTOM(newBlob) = (int) (Y + H_2);
        BLOB_XLEFT(newBlob)   = (int) (X - W_2);
        BLOB_XRIGHT(newBlob)  = (int) (X + W_2);
        BLOB_YTOP(newBlob)    = (int) (Y - H_2);
        BLOB_WIDTH(newBlob)   = BLOB_XRIGHT(newBlob)  - BLOB_XLEFT(newBlob) + 1;
        BLOB_HEIGHT(newBlob)  = BLOB_YBOTTOM(newBlob) - BLOB_YTOP(newBlob)  + 1;

        BLOB_POSITION(newBlob) = newBlob->getPositionRelativeToCamera(m_context);
        BLOB_3D_WIDTH(newBlob) = BLOB_3D_HEIGHT(newBlob) = BLOB_3D_LENGTH(newBlob) = BLOB_ALPHA(newBlob) = 0.0;
        BLOB_3D_X(newBlob) = BLOB_3D_Y(newBlob) = 0.0;
        BLOB_PW(newBlob) = BLOB_PH(newBlob) = BLOB_PL(newBlob) = BLOB_RW(newBlob) = BLOB_RH(newBlob) = BLOB_RL(newBlob) = BLOB_P(newBlob) = 0.0;

        //Update pertinent s3d data
        //Just one list is enough, because is just for completion of information.
        BLOB_DP_TYPE(newBlob) = MM_DP_NONE;
        if(BLOB_OCC_3DDATA(newBlob) != NULL) {
            delete BLOB_OCC_3DDATA(newBlob);
            BLOB_OCC_3DDATA(newBlob) = NULL;
        }

        std::map<ObjectType, Shape3DData> *normal = BLOB_NORMAL_3DDATA(newBlob);
        std::map<ObjectType, Shape3DData>::iterator n_it, n_it_end = normal->end();
        std::map<ObjectSubtype, Shape3DData> *subnormal;
        std::map<ObjectSubtype, Shape3DData>::iterator sn_it, sn_it_end;
        BLOB_TYPE(newBlob) = UNKNOWN;
        BLOB_SUBTYPE(newBlob) = ST_UNKNOWN;
        if(normal)
            for(n_it = normal->begin(); n_it != n_it_end; n_it++) {
                subnormal = S3D_SUBTYPES_LIST(&n_it->second);
                if(subnormal != NULL) {
                    sn_it_end = subnormal->end();
                    for(sn_it = subnormal->begin(); sn_it != sn_it_end; sn_it++)
                        sn_it->second.setNull(newBlob);
                }
                n_it->second.setNull(newBlob);
            }
    }

    return newBlob;
}

//Getting Functions
double MobileObject::getGlobalProbability() {
    return P;
}

unsigned long MobileObject::getMobileId() {
    return mobile_id;
}

unsigned long MobileObject::getRMobileId() {
    return rmobile_id;
}

ObjectType MobileObject::getBestType(){
    return best_type;
}

ObjectSubtype MobileObject::getBestSubType(){
    return best_subtype;
}

int MobileObject::getNumberOfFramesNotSeen() {
    return numberOfFramesNotSeen;
}

int MobileObject::getNumberOfFramesSinceFirstTimeSeen() {
    return numberOfFramesSinceFirstTimeSeen;
}


bool MobileObject::mobile3DCoherenceIsAcceptable() {
    if (    R3D >= m_DimensionalCoherenceReliabilityThreshold
         && P3D >= m_DimensionalCoherenceProbabilityThreshold )
        return true;

    return false;

}

bool MobileObject::mobile2DCoherenceIsAcceptable() {
    if (    R2D >= m_DimensionalCoherenceReliabilityThreshold
         && P2D >= m_DimensionalCoherenceProbabilityThreshold )
        return true;
    return false;
}


bool MobileObject::mobile3DVelocityCoherenceIsAcceptable() {
    if (    RVC >= m_SpatialCoherenceReliabilityThreshold
         && PVC >= m_SpatialCoherenceProbabilityThreshold )
        return true;
    return false;
}

bool orderedByBestCoherenceOperator::operator()(SpMobileObject mobile1, SpMobileObject mobile2) {
    return mobile1->getGlobalProbability() >= mobile2->getGlobalProbability();
}

bool orderedByMobileIdOperator::operator()(SpMobileObject mobile1, SpMobileObject mobile2) {
    return mobile1->getMobileId() < mobile2->getMobileId();
}

std::ostream& operator<<(std::ostream& out,const SpMobileObject mo){
    int i;
    Shape3DData *current_s3d;
    bool rigid = false;

    out << "\t\t\tMobile id        : " << mo->mobile_id << std::endl;
#ifdef MOBILE_DEBUG_DATA 
    out << "\t\t\tDebug Flag: " << mo->debug_data_flag;
    out << "\tDetailed Occlusion: " << Blob::getDPNameFromTypeDetailed(mo->currentVisualState);
    out << "\tLast Unknown: " << (mo->lastUnknown ? "true" : "false") << std::endl;
    //out << std::endl;
#endif
#ifdef MOBILE_DETAILS 
    out << "\t\t\tBest Type        : " << Blob::getNameFromType(mo->best_type) << std::endl;
    out << "\t\t\tBest Sub Type        : " << Blob::getNameFromSubtype(mo->best_subtype) << std::endl;

    out << "\t\t\tPosition(x, y) -> (SDx, SDy) : (Ppos) : (" << mo->t3DSpatialData.x << ", " << mo->t3DSpatialData.y << ")"
        << " -> (" << mo->t3DSpatialData.SDx << ", " << mo->t3DSpatialData.SDy << ") : (" << mo->t3DSpatialData.Ppos << ")" << std::endl;
    out << "\t\t\tPos. Rel.(Rx, Ry) ; (RCx, RCy, RDxy) : (" << mo->t3DSpatialData.Rx << ", " << mo->t3DSpatialData.Ry << ")"
        << " ; (" << mo->t3DSpatialData.RCx << ", " << mo->t3DSpatialData.RCy << ", " << mo->t3DSpatialData.RDxy << ")" << std::endl;
    out << "\t\t\tVelocity(Vx, Vy) -> (SDVx, SDVy)  : (" << mo->t3DSpatialData.Vx << ", " << mo->t3DSpatialData.Vy << ")"
        << " -> (" << mo->t3DSpatialData.SDVx << ", " << mo->t3DSpatialData.SDVy << ")" << std::endl;
    out << "\t\t\tVelocity(theta, V) -> (SDtheta, SDV)  : (" << mo->t3DSpatialData.theta << ", " << mo->t3DSpatialData.V << ")"
        << " -> (" << mo->t3DSpatialData.SDtheta << ", " << mo->t3DSpatialData.SDV << ")" << std::endl;
    out << "\t\t\tVelocity Rel.(RVx, RVy) ; (RCVx, RCVy)  : (" << mo->t3DSpatialData.RVx << ", " << mo->t3DSpatialData.RVy << ")"
        << " -> ("  << mo->t3DSpatialData.RCVx << ", " << mo->t3DSpatialData.RCVy << ")" << std::endl;
    out << "\t\t\tVelocity Others(PV, RDV, RCV, RV, MPV)  : (" << mo->t3DSpatialData.PV << ", " << mo->t3DSpatialData.RDV << ", " << mo->t3DSpatialData.RCV << ", " << mo->t3DSpatialData.RV << ", " << mo->t3DSpatialData.MPV << ")" << std::endl;

    out << "\t\t\tOrientation(M,SD)                 : (" << mo->t3DDimData.alpha << ", " << mo->t3DDimData.SDalpha << ")" << std::endl;
    out << "\t\t\tOrientation Reliabilities(RD,RC,R): (" << mo->t3DDimData.RDalpha << ", " << mo->t3DDimData.RCalpha << ", "  << mo->t3DDimData.Ralpha << ")" << std::endl;
    out << "\t\t\tOrientation Velocity (M,SD)       : (" << mo->t3DDimData.Valpha << ", " << mo->t3DDimData.SDValpha << ")" << std::endl;
    out << "\t\t\tOri. Velo. Reliabilities(RD,RC,R) : (" << mo->t3DDimData.RDValpha << ", " << mo->t3DDimData.RCValpha << ", " << mo->t3DDimData.RValpha << ")" << std::endl;
    out << "\t\t\t3D Height(M,SD,P,R) : (" << mo->t3DDimData.h << ", " << mo->t3DDimData.SDh<<", " << mo->t3DDimData.Ph << ", " << mo->t3DDimData.Rh << ")" << std::endl;

    out << "\t\t\t3D Dimensions (w, l, h) -> (SDw, SDl, SDh)     : (" << mo->t3DDimData.w << ", " << mo->t3DDimData.l << ", " << mo->t3DDimData.h << ") -> (" << mo->t3DDimData.SDw << ", " << mo->t3DDimData.SDl << ", " << mo->t3DDimData.SDh << ")" << std::endl;
    out << "\t\t\t3D Dimensions Probabilities (Pw, Pl, Ph) : (" << mo->t3DDimData.Pw << ", " << mo->t3DDimData.Pl << ", " << mo->t3DDimData.Ph << ")" << std::endl;
    out << "\t\t\t3D Width   Reliabilities(RDw, RCw, Rw): (" << mo->t3DDimData.RDw << ", " << mo->t3DDimData.RCw << ", " << mo->t3DDimData.Rw << ")" << std::endl;
    out << "\t\t\t3D Length  Reliabilities(RDl, RCl, Rl): (" << mo->t3DDimData.RDl << ", " << mo->t3DDimData.RCl << ", " << mo->t3DDimData.Rl << ")" << std::endl;
    out << "\t\t\t3D Height  Reliabilities(RDh, RCh, Rh): (" << mo->t3DDimData.RDh << ", " << mo->t3DDimData.RCh << ", " << mo->t3DDimData.Rh << ")" << std::endl;

    out << "\t\t\t3D Dimensions Change Speed (Vw, Vl, Vh) -> (SDVw, SDVl, SDVh)     : (" << mo->t3DDimData.Vw << ", " << mo->t3DDimData.Vl << ", " << mo->t3DDimData.Vh << ") -> (" << mo->t3DDimData.SDVw << ", " << mo->t3DDimData.SDVl << ", " << mo->t3DDimData.SDVh << ")" << std::endl;
    out << "\t\t\t3D Width  Speed Reliabilities(RDVw, RCVw, RVw): (" << mo->t3DDimData.RDVw << ", " << mo->t3DDimData.RCVw << ", " << mo->t3DDimData.RVw << ")" << std::endl;
    out << "\t\t\t3D Length Speed Reliabilities(RDVl, RCVl, RVl): (" << mo->t3DDimData.RDVl << ", " << mo->t3DDimData.RCVl << ", " << mo->t3DDimData.RVl << ")" << std::endl;
    out << "\t\t\t3D Height Speed Reliabilities(RDVh, RCVh, RVh): (" << mo->t3DDimData.RDVh << ", " << mo->t3DDimData.RCVh << ", " << mo->t3DDimData.RVh << ")" << std::endl;

    out << "\t\t\t2D Position (X,Y) -> (SDX,SDY)     : (" << mo->t2DSpatialData.X << ", " << mo->t2DSpatialData.Y << ") -> (" << mo->t2DSpatialData.SDX << ", " << mo->t2DSpatialData.SDY << ")" << std::endl;
    out << "\t\t\t2D X Reliabilities(RDX, RCX, RX): (" << mo->t2DSpatialData.RDX <<", " << mo->t2DSpatialData.RCX << ", " << mo->t2DSpatialData.RX<<")" << std::endl;
    out << "\t\t\t2D Y Reliabilities(RDY, RCY, RY): (" << mo->t2DSpatialData.RDY <<", " << mo->t2DSpatialData.RCY << ", " << mo->t2DSpatialData.RY<<")" << std::endl;
    out << "\t\t\t2D Velocities (VX,VY) -> (SDVX,SDVY)     : (" << mo->t2DSpatialData.VX << ", " << mo->t2DSpatialData.VY << ") -> (" << mo->t2DSpatialData.SDVX << ", " << mo->t2DSpatialData.SDVY << ")" << std::endl;
    out << "\t\t\t2D VX Reliabilities (RDVX, RCVX, RVX): (" << mo->t2DSpatialData.RDVX << ", " << mo->t2DSpatialData.RCVX << ", " << mo->t2DSpatialData.RVX << ")" << std::endl;
    out << "\t\t\t2D VY Reliabilities (RDVY, RCVY, RVY): (" << mo->t2DSpatialData.RDVY << ", " << mo->t2DSpatialData.RCVY << ", " << mo->t2DSpatialData.RVY << ")" << std::endl;
    out << "\t\t\t2D Position Velocity (V2D,theta2D) -> (SDV2D,SDtheta2D) : (" << mo->t2DSpatialData.V2D << ", " << mo->t2DSpatialData.theta2D << ") -> (" << mo->t2DSpatialData.SDV2D << ", " << mo->t2DSpatialData.SDtheta2D << ")" << std::endl;
    out << "\t\t\t2D Velocity Reliabilities(RDV2D, RCV2D, RV2D): (" << mo->t2DSpatialData.RDV2D << ", " << mo->t2DSpatialData.RCV2D << ", " << mo->t2DSpatialData.RV2D << ")" << std::endl;

    out << "\t\t\t2D Dimensions (W, H) -> (SDW, SDH)     : (" << mo->t2DDimData.W << ", " << mo->t2DDimData.H << ") -> (" << mo->t2DDimData.SDW << ", " << mo->t2DDimData.SDH<<")" << std::endl;
    out << "\t\t\t2D Width  Reliabilities(RDW, RCW, RW): (" << mo->t2DDimData.RDW << ", " << mo->t2DDimData.RCW << ", " << mo->t2DDimData.RW << ")" << std::endl;
    out << "\t\t\t2D Height Reliabilities(RDH, RCH, RH): (" << mo->t2DDimData.RDH << ", " << mo->t2DDimData.RCH << ", " << mo->t2DDimData.RH << ")" << std::endl;
    out << "\t\t\t2D Dimensions Velocity (VW, VH) -> (SDVW, SDVH)     : (" << mo->t2DDimData.VW << ", " << mo->t2DDimData.VH << ") -> (" << mo->t2DDimData.SDVW << ", " << mo->t2DDimData.SDVH << ")" << std::endl;
    out << "\t\t\t2D Width Velocity Reliabilities (RDVW, RCVW, RVW): (" << mo->t2DDimData.RDVW << ", " << mo->t2DDimData.RCVW << ", " << mo->t2DDimData.RVW << ")" << std::endl;
    out << "\t\t\t2D Height Velocity Reliabilities(RDVH, RCVH, RVH): (" << mo->t2DDimData.RDVH << ", " << mo->t2DDimData.RCVH << ", " << mo->t2DDimData.RVH << ")" << std::endl;
#endif
    //    out << "\t\t\t2D Velocity(VX2D,VY2D) : (" << mo->t2DSpatialData.VX2D<<" , " << mo->t2DSpatialData.VY2D<<")" << std::endl;
    //   out << "\t\t\tBlob History     : " << std::endl<<mo->blobHistory<<std::endl;
    //Better to show current chosen s3dsToAnalyzeList
    #ifdef SHOW_BLOB_BUFFER
    if( (mo->best_type == UNKNOWN && mo->rigidModel[(*mo->objectModelMap.begin()).first]) || mo->rigidModel[mo->best_type])
        rigid = true;
    if(mo->best_type == UNKNOWN) {
        Rectangle<int> *current_rect;
        for(i=0; i<mo->currentBufferSize; i++) {
            current_rect = &(mo->bboxesToAnalyze[i]);
            out << "\t\t\t\t\t" << " (W,L) : (" << RECT_WIDTH(current_rect) << ", " << RECT_HEIGHT(current_rect) << ")" << std::endl;
            out << "\t\t\t\t\t" << " (X,Y) : (" << RECT_XCENTER(current_rect) << ", " << RECT_YCENTER(current_rect) << ")" << std::endl;
        }
    } else {
        for(i=0; i<mo->currentBufferSize; i++) {
            current_s3d = mo->s3dsToAnalyze[i];
            if(current_s3d != NULL) {
                if(rigid) {
                    out << "\t\t\t\tFrame Data " << i << " (type,P,w,l,h,alpha) : (";
                    out << Blob::getNameFromType(S3D_TYPE(current_s3d)) << ", " << S3D_P(current_s3d) << ", ";
                    out << S3D_W(current_s3d) << ", " << S3D_L(current_s3d) << ", " << S3D_H(current_s3d);
                    out << ", " << S3D_ALPHA(current_s3d) << ")" << std::endl;
                    out << "\t\t\t\t\t (x,y,z)           : (" << S3D_3D_X(current_s3d);
                    out << ", " << S3D_3D_Y(current_s3d) << ", " << S3D_3D_Z(current_s3d) << ")" << std::endl;
                } else {
                    out << "\t\t\t\tFrame Data " << i << " (type,subtype,P,w,l,h,alpha) : (";
                    out << Blob::getNameFromType(S3D_TYPE(current_s3d)) << ", " << Blob::getNameFromSubtype(S3D_SUBTYPE(current_s3d)) << ", " << S3D_P(current_s3d) << ", ";
                    out << S3D_W(current_s3d) << ", " << S3D_L(current_s3d) << ", " << S3D_H(current_s3d);
                    out << ", " << S3D_ALPHA(current_s3d) << ")" << std::endl;
                    out << "\t\t\t\t\t (x,y,z)           : (" << S3D_3D_X(current_s3d);
                    out << ", " << S3D_3D_Y(current_s3d) << ", " << S3D_3D_Z(current_s3d) << ")" << std::endl;
                }
                out << "\t\t\t\t\t (W,L) : (" << S3D_WIDTH(current_s3d) << ", " << S3D_HEIGHT(current_s3d) << ")" << std::endl;
                out << "\t\t\t\t\t (X,Y) : (" << S3D_XCENTER(current_s3d) << ", " << S3D_YCENTER(current_s3d) << ")" << std::endl;
            }
        }
    }
#endif

    if(mo->currentVisualState & MM_PART_OF_BIGGER)
        out << "\t\t\tCurrent Visual State: PART OF BIGGER" << std::endl;
    else if(mo->currentVisualState & MM_PARTIALLY_DETECTED)
        out << "\t\t\tCurrent Visual State: PARTIALLY DETECTED" << std::endl;
    else if(mo->currentVisualState & MM_NOT_VISIBLE_MASK)
        out << "\t\t\tCurrent Visual State: LOST" << std::endl;
    else
        out << "\t\t\tCurrent Visual State: NORMAL" << std::endl;

    out << "\t\t\t#Number of frames since first one : " << mo->numberOfFramesSinceFirstTimeSeen << std::endl;
    out << "\t\t\t#Unknown frames : " << mo->unknownsNumber << std::endl;
    out << "\t\t\t#frames not seen : " << mo->numberOfFramesNotSeen << std::endl;
    out << "\t\t\t2D Coherence(P2D,R2D) : \t(" << mo->P2D << ", " << mo->R2D<<")" << std::endl;
    out << "\t\t\t3D Coherence(P3D,R3D) : \t(" << mo->P3D << ", " << mo->R3D<<")" << std::endl;
    out << "\t\t\t2D Velocity Coherence(PV2DC,RV2DC) : \t(" << mo->PV2DC << ", " << mo->RV2DC << ")" << std::endl;
    out << "\t\t\t3D Velocity Coherence(PVC,RVC) : \t(" << mo->PVC << ", " << mo->RVC << ")" << std::endl;
    out << "\t\t\tGlobal Probability : \t\t" << mo->P << std::endl;

    if(mo->involvedBlobs != NULL) {
        out <<  "InvolvedBlobs:" << std::endl;
        for(int i=0; i<MobileObject::m_currentTrackingBlobsNumber; i++)
            out << "\t" << mo->involvedBlobs[i];
        out << std::endl;
    }

    if(mo->usedBlobs != NULL) {
        out <<  "UsedBlobs:" << std::endl;
        for(int i=0; i<MobileObject::m_currentTrackingBlobsNumber; i++)
            out << "\t" << mo->usedBlobs[i];
        out << std::endl;
    }

    return out;
}


//memcpy(&realBBox, &(current_blob->bbox), sizeof(Rectangle<int>));
//Stablish cooperation schemes between 2D and 3D:
/*	if(bests3d != NULL && S3D_P(bests3d) >= m_classifThreshold ) { //&& coherentWithRespectOfCurrent3DInformation(bests3d)//3D can help
// In this case, support of 3D is 1.0 as a solution has been found.
//S o suppress the non trackable mask. Also trust in 2D evidence.
if(non_3D_trackable_mask)
rect_intersection(&(current_blob->bbox), &(bests3d->bbox), &(bboxesToAnalyze[0]));
currentVisualState = dpFlags[0] = (DetectionProblemType)(dpFlags[0] - non_3D_trackable_mask);
} else { //3D needs help
//Check support with visual evidence and the best supported will orient the other
//	  if(best_type != UNKNOWN) { //Check both supports
Rectangle<int> mobileBBox, improvedBBox;

if(non_3D_trackable_mask) {

//In this case type information will not change,
//so just the current data must be updated
getCurrentBoundingBoxForMobile(&mobileBBox);

//Update bbox to analyze and displace the visual support buffer.
improveBBoxSupport(&improvedBBox, &mobileBBox, &(current_blob->bbox)); 

if(currentVisualState & MM_PART_OF_BIGGER)
//support rate here is the coverage rate of the mobile bbox estimation over the current blob 
visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &improvedBBox);
else if(currentVisualState & MM_PARTIALLY_DETECTED)
//support rate here is the coverage rate of the current blob over the mobile bbox estimation
visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&improvedBBox, &(current_blob->bbox));
else //This case should not happen as
visualSupport[0] = 1.0;

memcpy(&(bboxesToAnalyze[0]), &improvedBBox, sizeof(Rectangle<int>));
} else {
memcpy(&improvedBBox, &(current_blob->bbox), sizeof(Rectangle<int>));
memcpy(&(bboxesToAnalyze[0]), &improvedBBox, sizeof(Rectangle<int>));
}

Shape3DData improved3DInfo;
Rectangle<int> realBBox;
memcpy(&realBBox, BLOB_BBOX(current_blob), sizeof(Rectangle<int>));
memcpy(BLOB_BBOX(current_blob), &improvedBBox, sizeof(Rectangle<int>));

bests3d = getBestPostureCoherents3dReclassifying(current_blob, BLOB_NORMAL_3DDATA(current_blob), BLOB_OCC_3DDATA(current_blob), best_type, best_index);

memcpy(BLOB_BBOX(current_blob), &realBBox, sizeof(Rectangle<int>));

if(bests3d == NULL) { //If still can not find a solution, generate it with the 3D data and reposition it
//with respect to the new bbox 
generateS3DFrom3DInformation(&improved3DInfo);
S3D_TYPE(&improved3DInfo) = best_type;
        if(rigidModel[best_type])
          S3D_SUBTYPE(&improved3DInfo) = ST_UNKNOWN;
        else
          S3D_SUBTYPE(&improved3DInfo) = best_subtype;
        orient3DModel(&improved3DInfo, &improvedBBox, BLOB_DP_TYPE(current_blob), current_blob);

        if(BLOB_NORMAL_3DDATA(current_blob) == NULL)
          setInitialNormalList(current_blob);
        Shape3DData *next, previous, normal = BLOB_NORMAL_3DDATA(current_blob);

        while(normal != NULL) {
          if(S3D_TYPE(normal) == best_type) {

            next = S3D_NEXT(normal);
            previous = S3D_PREVIOUS(normal);
            copy_s3data(&improved3DInfo, normal);
            S3D_NEXT(normal) = next;
            S3D_PREVIOUS(normal) = previous;
            bests3d = normal;
            break;
          }
          normal = S3D_NEXT(normal);
        }

        copy_S3D_to_blob(&improved3DInfo, current_blob);
        if(rigidModel[best_type])
          objectModelsList[objectModelMap[best_type]]->computeScore(current_blob, m_context, NULL);
        else
          objectSubModelsList[objectModelMap[best_type]][objectSubModelMap[best_type][best_subtype]]->computeScore(current_blob, m_context, NULL);
        copy_blob_to_s3d(current_blob, bests3d);
        memcpy(&realBBox, &(current_blob->bbox), sizeof(Rectangle<int>));

        double
          v1 = Rectangle<int>::rectangleIntersectRatio(&improvedBBox, &(current_blob->bbox)),
          v2 = Rectangle<int>::rectangleIntersectRatio(&(bests3d->bbox), &(current_blob->bbox));
        if(v2 > v1) {
          memcpy(&(bboxesToAnalyze[0]), &(bests3d->bbox), sizeof(Rectangle<int>));
          if(currentVisualState & MM_PART_OF_BIGGER)
            //support rate here is the coverage rate of the mobile bbox estimation over the current blob
            visualSupport[0] = Rectangle<int>::rectangleIntersectRatio(&(current_blob->bbox), &(bests3d->bbox));
          else if(currentVisualState & MM_PARTIALLY_DETECTED)
            //support rate here is the coverage rate of the current blob over the mobile bbox estimation
            visualSupport[0] = v2;
          else //This case should not happen as
            visualSupport[0] = 1.0;
        }
      }

      }*/
