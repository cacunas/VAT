#include "RMMMobileObject.h"
#include "RMMTracker.h"
#include "VideoAnalysis.h"
#include "src/MathFunctions.h"
#include "src/geometric.h"
#include <cmath>
//#include <values.h> values.h? //never heard of it before

double RMMMobileObject::ONE_OVER_SQRT_2_TIMES_PI = 1.0 / sqrt(2 * M_PI);

SceneModel *RMMMobileObject::m_context = NULL;
double RMMMobileObject::m_maxSpeed = 5000; // 5000 cm/s
double RMMMobileObject::m_maxKnownSpeed = 5000; // 5000 cm/s

int RMMMobileObject::m_currentTrackingBlobsNumber = 0;
double RMMMobileObject::m_lambda = 1000; // constant for cooling function of time difference in seconds
int RMMMobileObject::m_blobsBufferSize = 5;
int RMMMobileObject::m_trajectoryMaxSize = 0; // 0: now limit, >0: stablished limit
double RMMMobileObject::m_knownSolutionThreshold = 0.5;
double RMMMobileObject::m_mobile2DCoverageRateToConsiderEqual = 0.95;

std::map<ObjectType, bool> RMMMobileObject::rigidModel;
std::map<ObjectType, int> RMMMobileObject::objectModelMap;
std::map<ObjectType, std::map<ObjectSubtype, int> >RMMMobileObject::objectSubModelMap;
SpModelInterface *RMMMobileObject::objectModelsList = NULL;
SpModelInterface **RMMMobileObject::objectSubModelsList = NULL;

ReliabilityMultiModel         RMMMobileObject::modelTemplate;

bool RMMMobileObject::m_firstFrame = true;

double RMMMobileObject::m_CoherenceReliabilityThreshold = 0.1;
double RMMMobileObject::m_CoherenceProbabilityThreshold = 0.5;

std::map<ObjectType, ObjectSubtype> RMMMobileObject::lastFoundSubtypeTemplate;

double RMMMobileObject::m_classifThreshold = 0.0;

double *RMMMobileObject::secDiffSequence = NULL;
double *RMMMobileObject::coolingValue = NULL;
double *RMMMobileObject::secDiffToCurrent = NULL;

double RMMMobileObject::m_probabilityToEnsureMode = 0.8;
int RMMMobileObject::m_2DLevelFrames = 3;

double RMMMobileObject::zeroTolerance = 0.00001;
double RMMMobileObject::m_minimalTolerance = 4;
int RMMMobileObject::m_objectModelsNumber = 0;



RMMMobileObject::RMMMobileObject(Datapool *i_data):
                             m_data(i_data), best_type(UNKNOWN), best_subtype(ST_UNKNOWN), ensureMode(false),
                             numberOfFramesNotSeen(0), numberOfFramesSinceFirstTimeSeen(1),
                             currentBufferSize(1),
                             multiModel(m_data), trajectory3D(m_trajectoryMaxSize), trajectory2D(m_trajectoryMaxSize), usedBlobs(NULL), involvedBlobs(NULL),
                             unknownsNumber(0), classificationAllowed(false) {

    numUsed = 0;
    unknownsNumber = 0;
    initUsedBlobs();

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags               = new DetectionProblemType[m_blobsBufferSize];
    foundSupport          = new bool[m_blobsBufferSize];

    memset(bboxesToAnalyze, 0, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memset(visualSupport, 0, sizeof(double)*m_blobsBufferSize);
    memset(dpFlags, 0, sizeof(DetectionProblemType)*m_blobsBufferSize);
    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);

    memset(&iestimator, 0, sizeof(Rectangle<int>));

    P = 0.0;
    R = 0.0;

    //Init multimodel instance
    multiModel = modelTemplate;
    multiModel.mobile = this;

    currentVisualState = MM_DP_NONE;

}

RMMMobileObject::RMMMobileObject(SpRMMMobileObject to_copy):
       m_data(to_copy->m_data), mobile_id(to_copy->getMobileId()), hset_id(to_copy->getHypothesisSetId()), best_type(to_copy->getBestType()), best_subtype(to_copy->getBestSubType()),
       ensureMode(to_copy->ensureMode), best_index(to_copy->best_index),
       numberOfFramesNotSeen(to_copy->getNumberOfFramesNotSeen()),
       numberOfFramesSinceFirstTimeSeen(to_copy->getNumberOfFramesSinceFirstTimeSeen()+1),
       currentBufferSize(to_copy->currentBufferSize), multiModel(m_data),
       trajectory3D(m_trajectoryMaxSize), trajectory2D(m_trajectoryMaxSize), usedBlobs(NULL), involvedBlobs(NULL),
       unknownsNumber(to_copy->unknownsNumber), classificationAllowed(to_copy->classificationAllowed) {

    multiModel = to_copy->multiModel;

    trajectory3D.copyPoints(&(to_copy->trajectory3D));
    trajectory2D.copyPoints(&(to_copy->trajectory2D));

    initUsedBlobs();
    numUsed = 0;

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags        = new DetectionProblemType[m_blobsBufferSize];
    foundSupport          = new bool[m_blobsBufferSize];

    memcpy(&iestimator, &(to_copy->iestimator), sizeof(Rectangle<int>));

    memcpy(bboxesToAnalyze, to_copy->bboxesToAnalyze, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memcpy(visualSupport, to_copy->visualSupport, sizeof(double)*m_blobsBufferSize);
    memcpy(dpFlags, to_copy->dpFlags, sizeof(DetectionProblemType)*m_blobsBufferSize);
    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);

    P = 0.0;
    R = 0.0;

    currentVisualState = to_copy->currentVisualState;
    prevCooling = to_copy->prevCooling;
    sumCooling = to_copy->sumCooling;
}


RMMMobileObject::~RMMMobileObject() {
    trajectory3D.clear();
    trajectory2D.clear();

    if(usedBlobs)
        delete[] usedBlobs;
    if(involvedBlobs)
        delete[] involvedBlobs;
    if(bboxesToAnalyze)
        delete[] bboxesToAnalyze;
    if(visualSupport)
        delete[] visualSupport;
    if(dpFlags)
        delete[] dpFlags;
    if(foundSupport)
        delete[] foundSupport;
}

void RMMMobileObject::initUsedBlobs() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
}

void RMMMobileObject::initInvolvedBlobs() {
    if(involvedBlobs != NULL)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
    numInvolved = 0;
}


bool RMMMobileObject::mobileOutOfScene() {
    int pixelTolerance = 3;
    double R;
    Rectangle<int> r = multiModel.binterface.bbox;
    int
        X = RECT_XCENTER(&r),
        Y = RECT_YCENTER(&r),
        W_2 = RECT_WIDTH(&r) / 2,
        H_2 = RECT_HEIGHT(&r) / 2;
    if(X + W_2 < pixelTolerance || X - W_2 > m_data->currentImage->width() - pixelTolerance || Y + H_2 < pixelTolerance || Y - H_2 > m_data->currentImage->height() - pixelTolerance )
        return true;

    //If some while present, but lost for a while, assume it is out.
    if(   numberOfFramesSinceFirstTimeSeen > m_blobsBufferSize
        && numberOfFramesNotSeen > numberOfFramesSinceFirstTimeSeen  )
        return true;
    return false;
}

void RMMMobileObject::getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox) {
    if(bbox == NULL)
        return;

    if(currentBufferSize == 2) { //After first frame? Then copy the only blob in history
        //ACAAA!!!!
        //memcpy(bbox, BLOB_BBOX(*this->multiModelHistory.begin()), sizeof(Rectangle<int>));
        return;
    }

    //Use 2D information because it represent just the visual evidence.
    //In terms of Visual Support just visible data is interesting.
    double tdiff = secDiffSequence[0];
/*    int
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
*/
}


void RMMMobileObject::improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence) {
    int elimit, vlimit, variation, variation2, SD;

    //Initial values for improved bbox (without improvements :D).
    //ACA!!!! Cocina pa no perder!!!
    getCurrentBoundingBoxForMobileKeepingSize(estimatedBBox);
    memcpy(improvedBBox, estimatedBBox, sizeof(Rectangle<int>));

    //For horizontal variation
/*    if(RECT_XLEFT(estimatedBBox) > RECT_XLEFT(visualEvidence) && RECT_XRIGHT(estimatedBBox) > RECT_XRIGHT(visualEvidence)) { //Check if mobile position can be moved to the left
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
    }*/
}


void RMMMobileObject::setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport) {

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

Rectangle<int> RMMMobileObject::getVisualEvidenceEstimator(double &R) {
    return multiModel.getVisualEvidenceEstimator(R);
}

Rectangle<int> RMMMobileObject::getVisualEvidenceEstimator() {
    return multiModel.getVisualEvidenceEstimator(R);
}


void RMMMobileObject::setInitialNormalList(Blob *current_blob) {

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


void RMMMobileObject::get2DTrajectoryPoint(double &x, double &y, double &R) {
    multiModel.set2DTrajectoryPoint(x, y, R);

}

void RMMMobileObject::insert2DTrajectoryPoint() {
    double x, y, R;
    get2DTrajectoryPoint(x, y, R);
    trajectory2D.insert(x, y);
}

void RMMMobileObject::insertNewMultiModel(Blob *blob, double lastSecDifference) {
    multiModel.currentTimeDiffSeconds = lastSecDifference;
    multiModel.update(blob);
    multiModel.feedback();
}

//Build new single model instances from template
void RMMMobileObject::renewModels() {
    std::deque<SpReliabilitySingleModelInterface>::iterator it, it2, it3, it_end = modelTemplate.multiModelDAG.end(), it_end2;
    std::map<QString, SpReliabilitySingleModelInterface> modelsMap;
    //Only independent models init the process.
    multiModel.multiModelDAG.clear();

    multiModel.forwardIndependentModels = modelTemplate.forwardIndependentModels;
    multiModel.backwardIndependentModels = modelTemplate.backwardIndependentModels;

    for(it=modelTemplate.multiModelDAG.begin(); it != it_end; it++) {
        SpReliabilitySingleModelInterface templateModel = *it;
        SpReliabilitySingleModelInterface newModel(VideoAnalysis::modelConstructor[templateModel->model_name]());

        newModel->copy_general_structure(templateModel);
        newModel->m_mobile = this;
        newModel->init();
        multiModel.multiModelDAG.push_back(newModel);
        modelsMap[newModel->name] = newModel;
    }

    //Rebuid dependences
    for(it=modelTemplate.multiModelDAG.begin(), it3=multiModel.multiModelDAG.begin(); it != it_end; it++) {
        SpReliabilitySingleModelInterface templateModel = *it;
        SpReliabilitySingleModelInterface newModel = *it3;

        if(!templateModel->dependences.empty()) {
            newModel->dependences.clear();
            it_end2 = templateModel->dependences.end();
            for(it2=templateModel->dependences.begin(); it2 != it_end2; it2++)
                newModel->dependences.push_back(modelsMap[(*it2)->name]);
        }
        if(!templateModel->dependants.empty()) {
            newModel->dependants.clear();
            it_end2 = templateModel->dependants.end();
            for(it2=templateModel->dependants.begin(); it2 != it_end2; it2++)
                newModel->dependants.push_back(modelsMap[(*it2)->name]);
        }
    }

}

void RMMMobileObject::setInitialMultiModel(Blob *blob) {
    renewModels();
    multiModel.update(blob);
    multiModel.feedback();
}


//Setting Functions

void RMMMobileObject::setGlobalProbability() {
    multiModel.getGlobalProbability(R, P);
}

void RMMMobileObject::setMobileId(unsigned long i_id) {
    mobile_id = i_id;
}

void RMMMobileObject::setRMobileId(unsigned long i_id) {
    hset_id = i_id;
}

double RMMMobileObject::coolingFunction(double x) {
    return exp(-m_lambda*x);
}

void RMMMobileObject::setNewMobileFromBlob(Blob *blob, unsigned long mobile_id, unsigned long hset_id) {
    //Functions order IS important:
    //First initialize memory of some internal processing lists
    setMobileId(mobile_id);
    setRMobileId(hset_id);
    setNumberOfFramesNotSeen(0);
    numberOfFramesSinceFirstTimeSeen = 1;

    //The template contains a multi-model with size one buffers with
    //non-initialized single models. So execution of setInitialMultiModel
    //after is compulsory
    multiModel = modelTemplate;

    //Insert the first multi model from blob into the mobile's blob buffer
    currentBufferSize = 1;

    initCooling(); //Init general cooling accumulators

    updateMobileBufferStructures(blob);

    setInitialMultiModel(blob);

    setGlobalProbability();
    multiModel.binterface.bbox = getVisualEvidenceEstimator();
    insert2DTrajectoryPoint();

}

void RMMMobileObject::updateMobileBufferStructures(Blob *blob) {

    memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
    currentVisualState = dpFlags[0] = BLOB_DP_TYPE(blob);

    DetectionProblemType not_visible_mask = (DetectionProblemType) (BLOB_DP_TYPE(blob) & MM_NOT_VISIBLE_MASK);

    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);
    numberOfFound = 0;

    //Update bbox to analyze and displace the visual support buffer.
    memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
    memcpy(&(bboxesToAnalyze[0]), &(blob->bbox), sizeof(Rectangle<int>));

    //Insert new visual support
    memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
    visualSupport[0] = 1.0;

    //Set the flags for data to be analysed
    for(int i=0;i<currentBufferSize; i++)
        if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            foundSupport[i] = true;
            numberOfFound++;
        }

    RFoundSolutions  = (double)numberOfFound/(double)currentBufferSize;
    RVFoundSolutions = numberOfFound < 2 ? 0.0 : (double)(numberOfFound-1)/(double)(currentBufferSize-1);

    if(!not_visible_mask)
        setNumberOfFramesNotSeen(0);
}



void RMMMobileObject::initCooling() {
    sumCooling = 1.0;
    prevCooling = 0.0;

}


void RMMMobileObject::updateMobilePath(Blob *blob) {
    P = 0.0;
    R = 0.0;

    if(currentBufferSize < m_blobsBufferSize)
        currentBufferSize ++;

    updateMobileBufferStructures(blob);

    if(numberOfFramesSinceFirstTimeSeen == 1) {
        initCooling(); //Init general cooling accumulators
        setInitialMultiModel(blob);
    } else {
        updateCooling();
        insertNewMultiModel(blob, secDiffSequence[0]);
    }

    setGlobalProbability();
    multiModel.binterface.bbox = getVisualEvidenceEstimator();
    insert2DTrajectoryPoint();


    //ACA!!!!
    //Set ensureMode = true if criterias are accomplished
    //if(!ensureMode && P > m_probabilityToEnsureMode )
    //    ensureMode = true;

}

void RMMMobileObject::updateCooling() {
    prevCooling = sumCooling;
    sumCooling = 1.0 + coolingValue[1] * sumCooling;

//    sumCooling = foundSupport[0] ? 1.0 + coolingValue[1] * sumCooling
//                                 : 0.0 + coolingValue[1] * sumCooling;

    //If cooling sum descending to a tolerance, establish it as minimum
    if(prevCooling >= zeroTolerance && sumCooling < zeroTolerance)
        sumCooling = zeroTolerance;
}


void RMMMobileObject::setNumberOfFramesNotSeen(int num) {
    numberOfFramesNotSeen = num;
}

void RMMMobileObject::incrementNumberOfFramesNotSeen() {
    numberOfFramesNotSeen++;
}

//2D bounding box information infered from best quality mobile information
Blob *RMMMobileObject::determineMostLikelyBlob() {
    Blob *newBlob = new Blob(&multiModel.binterface);

    //Trustable information
    if( R >= m_CoherenceReliabilityThreshold && P >= m_CoherenceProbabilityThreshold ) {
        double sina, cosa, x, y;

    }

    return newBlob;
}

//Getting Functions
double RMMMobileObject::getGlobalProbability() {
    return P;
}

SpReliabilitySingleModelInterface RMMMobileObject::getSubModel(QString name) {
    return multiModel.getSingleModelByName(name);
}


unsigned long RMMMobileObject::getMobileId() {
    return mobile_id;
}

unsigned long RMMMobileObject::getHypothesisSetId() {
    return hset_id;
}

ObjectType RMMMobileObject::getBestType(){
    return best_type;
}

ObjectSubtype RMMMobileObject::getBestSubType(){
    return best_subtype;
}

int RMMMobileObject::getNumberOfFramesNotSeen() {
    return numberOfFramesNotSeen;
}

int RMMMobileObject::getNumberOfFramesSinceFirstTimeSeen() {
    return numberOfFramesSinceFirstTimeSeen;
}

bool RMMorderedByBestCoherenceOperator::operator()(SpRMMMobileObject mobile1, SpRMMMobileObject mobile2) {
    return mobile1->getGlobalProbability() >= mobile2->getGlobalProbability();
}

bool RMMorderedByMobileIdOperator::operator()(SpRMMMobileObject mobile1, SpRMMMobileObject mobile2) {
    return mobile1->getMobileId() < mobile2->getMobileId();
}

std::ostream& operator<<(std::ostream& out,const SpRMMMobileObject mo){
    int i;
    Shape3DData *current_s3d;
    bool rigid = false;

    out << "\t\t\tMobile id        : " << mo->mobile_id << std::endl;
#ifdef MOBILE_DEBUG_DATA 
    out << "\tDetailed Detection Problem: " << Blob::getDPNameFromTypeDetailed(mo->currentVisualState);
    out << std::endl;
#endif

    //    out << "\t\t\t2D Velocity(VX2D,VY2D) : (" << mo->t2DSpatialData.VX2D<<" , " << mo->t2DSpatialData.VY2D<<")" << std::endl;
    //   out << "\t\t\tBlob History     : " << std::endl<<mo->multiModelHistory<<std::endl;
    //Better to show current chosen s3dsToAnalyzeList

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
    out << "\t\t\tModel Coherence  P: " << mo->P << std::endl;
    out << "\t\t\tData Reliability R: " << mo->R << std::endl;

    if(mo->involvedBlobs != NULL) {
        out <<  "InvolvedBlobs:" << std::endl;
        for(int i=0; i<RMMMobileObject::m_currentTrackingBlobsNumber; i++)
            out << "\t" << mo->involvedBlobs[i];
        out << std::endl;
    }

    if(mo->usedBlobs != NULL) {
        out <<  "UsedBlobs:" << std::endl;
        for(int i=0; i<RMMMobileObject::m_currentTrackingBlobsNumber; i++)
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
        orient3DModel(&improved3DInfo, &improvedBBox, BLOB_OCCL_TYPE(current_blob), current_blob);

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
