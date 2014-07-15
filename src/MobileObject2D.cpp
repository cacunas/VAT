#include "MobileObject2D.h"
#include "ReliabilityTracker2D.h"
//#include "ReliabilityClassification.h"
#include "MathFunctions.h"
#include "geometric.h"
#include <cmath>
//#include <values.h> values.h? //never heard of it before

double MobileObject2D::ONE_OVER_SQRT_2_TIMES_PI = 1.0 / sqrt(2 * M_PI);

QImage *MobileObject2D::m_pSegmentation = NULL;

int MobileObject2D::m_currentTrackingBlobsNumber = 0;
double MobileObject2D::m_lambda = 1000; // constant for cooling function of time difference in seconds
int MobileObject2D::m_blobsBufferSize = 5;
int MobileObject2D::m_trajectoryMaxSize = 0; // 0: now limit, >0: stablished limit
bool MobileObject2D::m_firstFrame = true;
double MobileObject2D::m_SpatialCoherenceReliabilityThreshold = 0.1;
double MobileObject2D::m_SpatialCoherenceProbabilityThreshold = 0.5;
double MobileObject2D::m_DimensionalCoherenceReliabilityThreshold = 0.1;
double MobileObject2D::m_DimensionalCoherenceProbabilityThreshold = 0.5;

double *MobileObject2D::secDiffSequence = NULL;
double *MobileObject2D::coolingValue = NULL;
double *MobileObject2D::secDiffToCurrent = NULL;

double MobileObject2D::m_probabilityToEnsureMode = 0.8;

double *MobileObject2D::g_secDiffSequence;
double *MobileObject2D::g_coolingValue;
double *MobileObject2D::g_secDiffToCurrent;
tracking2DimensionalData *MobileObject2D::g_t2DDimData;
tracking2DSpatialData    *MobileObject2D::g_t2DSpatialData;
IncrementalExtraGeneralData2D *MobileObject2D::g_iGData;
IncrementalExtra2DData *MobileObject2D::g_i2DData;
Rectangle<int> *MobileObject2D::g_newBBoxesToAnalyze;
double *MobileObject2D::g_newVisualSupport;
DetectionProblemType *MobileObject2D::g_newDPFlags;
int MobileObject2D::g_currentBufferSize;
double MobileObject2D::zeroTolerance = 0.00001;
double MobileObject2D::m_minimalTolerance = 4;

int MobileObject2D::m_pixelAcuity = 3;
double MobileObject2D::m_max2DSpeed = 100;
double MobileObject2D::m_reliableObjectLength = 50;


MobileObject2D::MobileObject2D():ensureMode(false), numberOfFramesNotSeen(0), numberOfFramesSinceFirstTimeSeen(1), debug_data_flag(0),
                             blobHistory(m_blobsBufferSize), trajectory2D(m_trajectoryMaxSize), usedBlobs(NULL), involvedBlobs(NULL) {

    memset(&t2DDimData, 0, sizeof(tracking2DimensionalData));
    memset(&t2DSpatialData, 0, sizeof(tracking2DSpatialData));
    memset(&iGData, 0, sizeof(IncrementalExtraGeneralData));
    memset(&i2DData, 0, sizeof(IncrementalExtra2DData));

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags               = new DetectionProblemType[m_blobsBufferSize];
    foundSupport          = new bool[m_blobsBufferSize];

    memset(bboxesToAnalyze, 0, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memset(visualSupport, 0, sizeof(double)*m_blobsBufferSize);
    memset(dpFlags, 0, sizeof(DetectionProblemType)*m_blobsBufferSize);
    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);

    numUsed = 0;
    initUsedBlobs();

    P2D = 0.0;
    PV2DC = 0.0;

#ifdef __NO_RELIABILITY_CODE__
    R2D = 1.0;
    RV2DC = 1.0;
#else
    R2D = 0.0;
    RV2DC = 0.0;
#endif
}

MobileObject2D::MobileObject2D(SpMobileObject2D to_copy):mobile_id(to_copy->getMobileId()), rmobile_id(to_copy->getRMobileId()),
                                                   ensureMode(to_copy->ensureMode), numberOfFramesNotSeen(to_copy->getNumberOfFramesNotSeen()),
                                                   numberOfFramesSinceFirstTimeSeen(to_copy->getNumberOfFramesSinceFirstTimeSeen()+1), debug_data_flag(0),
                                                   blobHistory(m_blobsBufferSize), trajectory2D(m_trajectoryMaxSize), usedBlobs(NULL), involvedBlobs(NULL) {
    blobHistory.copyBlobs(&(to_copy->blobHistory));
    trajectory2D.copyPoints(&(to_copy->trajectory2D));

    memcpy(&t2DDimData, &(to_copy->t2DDimData), sizeof(tracking2DimensionalData));
    memcpy(&t2DSpatialData, &(to_copy->t2DSpatialData), sizeof(tracking2DSpatialData));
    memcpy(&iGData, &(to_copy->iGData), sizeof(IncrementalExtraGeneralData));
    memcpy(&i2DData, &(to_copy->i2DData), sizeof(IncrementalExtra2DData));

    bboxesToAnalyze       = new Rectangle<int>[m_blobsBufferSize];
    visualSupport         = new double[m_blobsBufferSize];
    dpFlags        = new DetectionProblemType[m_blobsBufferSize];
    foundSupport          = new bool[m_blobsBufferSize];

    memcpy(bboxesToAnalyze, to_copy->bboxesToAnalyze, sizeof(Rectangle<int>)*m_blobsBufferSize);
    memcpy(visualSupport, to_copy->visualSupport, sizeof(double)*m_blobsBufferSize);
    memcpy(dpFlags, to_copy->dpFlags, sizeof(DetectionProblemType)*m_blobsBufferSize);
    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);

    initUsedBlobs();
    numUsed = 0;

    P2D = 0.0;
    PV2DC = 0.0;

#ifdef __NO_RELIABILITY_CODE__
    R2D = 1.0;
    RV2DC = 1.0;
#else
    R2D = 0.0;
    RV2DC = 0.0;
#endif
}


MobileObject2D::~MobileObject2D() {
    blobHistory.clear();
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

void MobileObject2D::initUsedBlobs() {
    if(usedBlobs)
        delete[] usedBlobs;
    if(m_currentTrackingBlobsNumber) {
        usedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(usedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        usedBlobs = NULL;
    usedLength = m_currentTrackingBlobsNumber;
}

void MobileObject2D::initInvolvedBlobs() {
    if(involvedBlobs != NULL)
        delete[] involvedBlobs;
    if(m_currentTrackingBlobsNumber) {
        involvedBlobs = new bool[m_currentTrackingBlobsNumber];
        memset(involvedBlobs, false, sizeof(bool)*m_currentTrackingBlobsNumber);
    } else
        involvedBlobs = NULL;
    numInvolved = 0;
}


//ACA!!!
//The solution to initial 3D generation is to implement a new function considering
//the current available data to guide the classification on finding the parallelepiped.
//  Shape3DData *MobileObject2D::getMostCoherentDataForMobile(Rectangle<int> *bbox, Blob *blob) {
//  return m_rc->getMostCoherentDataFromMobileAndBBoxLimit(this, normal, occ, blob);
//}


bool MobileObject2D::mobileOutOfScene() {
    int pixelTolerance = 3;
    double
        X = t2DSpatialData.X,
        Y = t2DSpatialData.Y,
        W_2 = t2DDimData.W / 2.0,
        H_2 = t2DDimData.H / 2.0;
    if(X + W_2 < pixelTolerance || X - W_2 > m_pSegmentation->width() - pixelTolerance || Y + H_2 < pixelTolerance || Y - H_2 > m_pSegmentation->height() - pixelTolerance )
        return true;

    return false;
}


void MobileObject2D::getCurrentBoundingBoxForMobile(Rectangle<int> *bbox) {
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

    if(W_2 <= m_minimalTolerance)
        W_2 = m_minimalTolerance;
    if(H_2 <= m_minimalTolerance)
        H_2 = m_minimalTolerance;


    RECT_XLEFT(bbox)   = X - W_2;
    RECT_XRIGHT(bbox)  = X + W_2;
    RECT_YTOP(bbox)    = Y - H_2;
    RECT_YBOTTOM(bbox) = Y + H_2;

    RECT_WIDTH(bbox)    = RECT_XRIGHT(bbox) - RECT_XLEFT(bbox) + 1;
    RECT_HEIGHT(bbox)   = RECT_YBOTTOM(bbox) - RECT_YTOP(bbox) + 1;

}

void MobileObject2D::getCurrentBoundingBoxForMobileKeepingSize(Rectangle<int> *bbox) {
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


void MobileObject2D::improveBBoxSupport(Rectangle<int> *improvedBBox, Rectangle<int> *estimatedBBox, Rectangle<int> *visualEvidence) {
    int elimit, vlimit, variation, variation2, SD;

    //Initial values for improved bbox (without improvements :D).
    getCurrentBoundingBoxForMobileKeepingSize(estimatedBBox);
    memcpy(improvedBBox, estimatedBBox, sizeof(Rectangle<int>));

    //For horizontal variation
    if(RECT_XLEFT(estimatedBBox) > RECT_XLEFT(visualEvidence) && RECT_XRIGHT(estimatedBBox) > RECT_XRIGHT(visualEvidence)) { //Check if mobile position can be moved to the left
        SD = (int)(t2DSpatialData.SDX + t2DSpatialData.SDVX*secDiffSequence[0]) + m_pixelAcuity;
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
        SD = (int)(t2DSpatialData.SDX + t2DSpatialData.SDVX*secDiffSequence[0]) + m_pixelAcuity;
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
        SD = (int)(t2DSpatialData.SDY + t2DSpatialData.SDVY*secDiffSequence[0]) + m_pixelAcuity;
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
        SD = (int)(t2DSpatialData.SDY + t2DSpatialData.SDVY*secDiffSequence[0]) + m_pixelAcuity;
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
        elimit = (int)(t2DDimData.SDW + t2DDimData.SDVW*secDiffSequence[0]) + m_pixelAcuity;

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
        elimit = (int)(t2DDimData.SDH + t2DDimData.SDVH*secDiffSequence[0]) + m_pixelAcuity;

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


void MobileObject2D::getMobileTolerances(double& Wtol, double& Htol) {
    double tdiff = secDiffSequence[0];
    Wtol = 2*(t2DDimData.SDW + t2DDimData.SDVW*tdiff);
    Htol = 2*(t2DDimData.SDH + t2DDimData.SDVH*tdiff);
    if(Wtol < m_minimalTolerance) Wtol = m_minimalTolerance;
    if(Htol < m_minimalTolerance) Htol = m_minimalTolerance;
}


void MobileObject2D::setSpecialBBoxToAnalyze(Rectangle<int> *bboxResult, Rectangle<int> *realBBox, double visualSupport) {

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

//2D mobile information preparation
void MobileObject2D::incorporateNewInformation() {

    int i;
    Blob *current_blob = &(*blobHistory.back());

    //  debug_data_flag = -1;

    //  if(mobile_id == 1)
    //      std::cout << "Este me interesa!!!" << std::endl;

    //Authorized to add points to trajectory buffers
    productionMode = true;

    memmove(dpFlags + 1, dpFlags, sizeof(DetectionProblemType)*(m_blobsBufferSize-1));
    currentVisualState = dpFlags[0] = BLOB_DP_TYPE(current_blob);
    memset(foundSupport, false, sizeof(bool)*m_blobsBufferSize);
    numberOfFound = 0;
    DetectionProblemType not_visible_mask      = (DetectionProblemType) (BLOB_DP_TYPE(current_blob) & MM_NOT_VISIBLE_MASK);

    //Update bbox to analyze and displace the visual support buffer.
    memmove(bboxesToAnalyze + 1, bboxesToAnalyze, sizeof(Rectangle<int>)*(m_blobsBufferSize-1));
    memcpy(&(bboxesToAnalyze[0]), &(current_blob->bbox), sizeof(Rectangle<int>));

    //Insert new visual support
    memmove(visualSupport + 1, visualSupport, sizeof(double)*(m_blobsBufferSize-1));
    visualSupport[0] = 1.0;

    //Set the flags for data to be analysed
    for(i=0;i<currentBufferSize; i++)
        if( !(dpFlags[i] & MM_NOT_VISIBLE_MASK) ) {
            foundSupport[i] = true;
            numberOfFound++;
    }

    RFoundSolutions  = (double)numberOfFound/(double)currentBufferSize;
    RVFoundSolutions = numberOfFound < 2 ? 0.0 : (double)(numberOfFound-1)/(double)(currentBufferSize-1);

    if(!not_visible_mask)
        setNumberOfFramesNotSeen(0);

}

//Inserting Functions
void MobileObject2D::insertNewBlob(Blob *blob) {
    Blob *new_blob;
    previousBufferSize = blobHistory.size();
    blobHistory.insert((new_blob = blob->copyWithLists()));
    currentBufferSize = blobHistory.size();
}

void MobileObject2D::insertNewBlob(Blob *blob, int lastMilliSecondsDifference) {
    Blob *new_blob = blob->copyWithLists();
    BLOB_TIME_DIFF_MSEC(new_blob) = lastMilliSecondsDifference;
    previousBufferSize = blobHistory.size();
    blobHistory.insert(new_blob);
    currentBufferSize = blobHistory.size();
}


//Setting Functions
void MobileObject2D::setGlobalProbability() {
    if(R2D + RV2DC > 0.0)
        P = (P2D*R2D + PV2DC*RV2DC) / (R2D + RV2DC);
    else
        P = 0.0;
}

void MobileObject2D::setMobileId(unsigned long i_id) {
    mobile_id = i_id;
}

void MobileObject2D::setRMobileId(unsigned long i_id) {
    rmobile_id = i_id;
}

double MobileObject2D::coolingFunction(double x) {
    return exp(-m_lambda*x);
}

void MobileObject2D::setNewMobileFromBlob(Blob *blob, unsigned long mobile_id, unsigned long rmobile_id) {
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

void MobileObject2D::updateMobileData(){
    P2D = 0.0;
    PV2DC = 0.0;
#ifdef __NO_RELIABILITY_CODE__
    R2D = 1.0;
    RV2DC = 1.0;
#else
    R2D = 0.0;
    RV2DC = 0.0;
#endif

    incorporateNewInformation();

    //Update General Cooling Function Values
    incrementalUpdateCooling(currentBufferSize);

    //Update the mobile 2D data
    incrementalUpdate2DDimensions(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, blobHistory.back()->maxDistanceFactor);
    incrementalUpdate2DPosition(currentBufferSize, bboxesToAnalyze, dpFlags, visualSupport, blobHistory.back()->maxDistanceFactor);
    trajectory2D.insert(t2DSpatialData.X, t2DSpatialData.Y);
}

void MobileObject2D::updateMobilePath(Blob *blob) {
    insertNewBlob(blob);

    updateMobileData();

    setGlobalProbability();

    //Set ensureMode = true if criterias are accomplished
    //if(!ensureMode && numberOfFound >= m_blobsBufferSize && P > m_probabilityToEnsureMode )
      //  ensureMode = true;

}

void MobileObject2D::incrementalUpdateCooling(int bufferSize) {

    if(bufferSize == 1) {
        iGData.sumCooling2D = foundSupport[0] ? 1.0 : 0.0;
        return;
    }

    if(foundSupport[0]) {
        iGData.prevCooling2D = iGData.sumCooling2D;
        iGData.sumCooling2D = 1.0 + coolingValue[1] * iGData.sumCooling2D;
    } else {
        iGData.prevCooling2D = iGData.sumCooling2D;
        iGData.sumCooling2D = 0.0 + coolingValue[1] * iGData.sumCooling2D;
    }

    if(iGData.prevCooling2D >= zeroTolerance && iGData.sumCooling2D < zeroTolerance)
        iGData.sumCooling2D = zeroTolerance;
}


double MobileObject2D::probabilisticCoherenceReliability(double data, double mean, double sigma, double acuity) {
    if(sigma == 0.0)
        return fabs(data - mean) <= acuity ? 1.0 : 0.0;

    double diff = data - mean;
#ifdef __NO_RELIABILITY_CODE__
    return 1.0;
#else
    return exp ( - diff*diff/(2*sigma*sigma) );
#endif
}

double MobileObject2D::normalisedSigmaCoherenceReliability(double sigma, double acuity) {
#ifdef __NO_RELIABILITY_CODE__
    return 1.0;
#else
    return  sigma<acuity ? 1.0 : acuity/sigma;
#endif
}


//Incremental version for 2D Position update
void MobileObject2D::incrementalUpdate2DPosition(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor) {
    Rectangle<int> *current_bbox = &bboxesToAnalyze[0];
    double tdiff = secDiffSequence[0], Rdistance;

    //Trivial case for an initial blobHistory
    if(bufferSize == 1) {
        t2DSpatialData.X = RECT_XCENTER(current_bbox);
        t2DSpatialData.Y = RECT_YCENTER(current_bbox);

        t2DSpatialData.SDX = t2DSpatialData.SDY = m_pixelAcuity; //Initial incremental SD

#ifdef __NO_RELIABILITY_CODE__
        double Rdistance = 1.0;
#else
        double Rdistance = dimensional2DReliability(RECT_WIDTH(current_bbox), RECT_HEIGHT(current_bbox));
#endif
        //Evaluate reliability as mean between difficulty of obtained data (Rdim) and coherence with previous one (Rcoh (initially 1))
#ifdef __NO_RELIABILITY_CODE__
        i2DData.sumRDX = t2DSpatialData.RDX = 1.0;
        i2DData.sumRDY = t2DSpatialData.RDY = 1.0;
        t2DSpatialData.RCX = t2DSpatialData.RCY = 1.0;
        t2DSpatialData.RX = t2DSpatialData.RY = 1.0;
#else
        i2DData.sumRDX = t2DSpatialData.RDX = Rdistance;
        i2DData.sumRDY = t2DSpatialData.RDY = Rdistance;
        t2DSpatialData.RCX = t2DSpatialData.RCY = 1.0;
        t2DSpatialData.RX = (t2DSpatialData.RDX + 1.0) / 2.0;
        t2DSpatialData.RY = (t2DSpatialData.RDY + 1.0) / 2.0;
#endif
        i2DData.sumRDVX = i2DData.sumRDVY = 0.0;
        t2DSpatialData.VX = t2DSpatialData.VY = 0.0;
        t2DSpatialData.SDVX = t2DSpatialData.SDVY = 0.0;

        PV2DC = 0.0;
#ifdef __NO_RELIABILITY_CODE__
        RV2DC = 1.0;
#else
        RV2DC = 0.0;
#endif
        return;
    }

    double XData, YData, Xcur, Ycur, RXData, RYData, auxSum, curSD, curDiff;
    double currentCooling = coolingValue[1];
    double initialSDX = 0.0, initialSDY = 0.0;

    //If last element to be added has been found
    if(foundSupport[0]) {
        if(t2DSpatialData.RX < 0.0000001)
            initialSDX = m_pixelAcuity;
        if(t2DSpatialData.RY < 0.0000001)
            initialSDY = m_pixelAcuity;

        //DATA
        //Get current estimation, according to model.
        Xcur = t2DSpatialData.X + t2DSpatialData.VX*tdiff;
        Ycur = t2DSpatialData.Y + t2DSpatialData.VY*tdiff;

        //Get current data
        XData = RECT_XCENTER(current_bbox);
        YData = RECT_YCENTER(current_bbox);

        //RELIABILITY
        //Set data reliability: Add reliability related to object size
#ifdef __NO_RELIABILITY_CODE__
        RXData = RYData = 1.0;
#else
        RXData = RYData = 1.0;
#endif
        //Cool old reliability X
        auxSum = i2DData.sumRDX;
        i2DData.sumRDX = RXData + currentCooling*auxSum;
#ifdef __NO_RELIABILITY_CODE__
        t2DSpatialData.RDX = 1.0;
#else
        t2DSpatialData.RDX = i2DData.sumRDX / iGData.sumCooling2D;
#endif
        //Incremental SD Formula uses Xcur value, storing failures in estimation on real data, representing COHERENCE
        curDiff = (XData - Xcur);
        curSD = t2DSpatialData.SDX;
        t2DSpatialData.SDX = sqrt( (currentCooling*auxSum/i2DData.sumRDX) * (curSD*curSD + RXData*(curDiff*curDiff/i2DData.sumRDX)) );
#ifdef __NO_RELIABILITY_CODE__
        t2DSpatialData.RCX = 1.0;
#else
        t2DSpatialData.RCX = normalisedSigmaCoherenceReliability(t2DSpatialData.SDX, m_pixelAcuity);
#endif
        t2DSpatialData.X = (XData*RXData + currentCooling*auxSum*Xcur) / i2DData.sumRDX;

        //Cool old reliability Y
        auxSum = i2DData.sumRDY;
        i2DData.sumRDY = RYData + currentCooling*auxSum;

#ifdef __NO_RELIABILITY_CODE__
        t2DSpatialData.RDY = 1.0;
#else
        t2DSpatialData.RDY = i2DData.sumRDY / iGData.sumCooling2D;
#endif
        //Incremental SD Formula uses Ycur value, storing failures in estimation on real data, representing COHERENCE
        curDiff = (YData - Ycur);
        curSD = t2DSpatialData.SDY;
        t2DSpatialData.SDY = sqrt( (currentCooling*auxSum/i2DData.sumRDY) * (curSD*curSD + RYData*(curDiff*curDiff/i2DData.sumRDY)) );
#ifdef __NO_RELIABILITY_CODE__
        t2DSpatialData.RCY = 1.0;
#else
        t2DSpatialData.RCY = normalisedSigmaCoherenceReliability(t2DSpatialData.SDY, m_pixelAcuity);
#endif
        t2DSpatialData.Y = (YData*RYData + currentCooling*auxSum*Ycur) / i2DData.sumRDY;


    } else { //The current element has not been found
        //Get current estimation, according to model.
        Xcur = t2DSpatialData.X + t2DSpatialData.VX*tdiff;
        Ycur = t2DSpatialData.Y + t2DSpatialData.VY*tdiff;

        //For X
        if(t2DSpatialData.RX < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i2DData.sumRDX = t2DSpatialData.RDX = t2DSpatialData.RCX = 0.0;
            t2DSpatialData.SDX = m_pixelAcuity;
        } else { //If not classified and we have previous information
            //Incremental step
            auxSum = i2DData.sumRDX;
            i2DData.sumRDX = currentCooling * i2DData.sumRDX; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDX = 1.0;
#else
            t2DSpatialData.RDX = i2DData.sumRDX / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses Xcur value, storing failures in estimation on real data, representing COHERENCE
            curSD = t2DSpatialData.SDX;
            t2DSpatialData.SDX = sqrt( (currentCooling*auxSum/i2DData.sumRDX) * (curSD*curSD) );
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RCX = 1.0;
#else
            t2DSpatialData.RCX = normalisedSigmaCoherenceReliability(t2DSpatialData.SDX, m_pixelAcuity);
#endif
            t2DSpatialData.X = Xcur; //Use estimation
        }

        //For Y
        if(t2DSpatialData.RY < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i2DData.sumRDY = t2DSpatialData.RDY = t2DSpatialData.RCY = 0.0;
            t2DSpatialData.SDY = m_pixelAcuity;
        } else { //If not classified and we have previous information
            //Incremental step
            auxSum = i2DData.sumRDY;
            i2DData.sumRDY = currentCooling * i2DData.sumRDY; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDY = 1.0;
#else
            t2DSpatialData.RDY = i2DData.sumRDY / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses Xcur value, storing failures in estimation on real data, representing COHERENCE
            curSD = t2DSpatialData.SDY;
            t2DSpatialData.SDY = sqrt( (currentCooling*auxSum/i2DData.sumRDY) * (curSD*curSD) );
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RCY = 1.0;
#else
            t2DSpatialData.RCY = normalisedSigmaCoherenceReliability(t2DSpatialData.SDY, m_pixelAcuity);
#endif
            t2DSpatialData.Y = Ycur; //Use estimation
        }
    }
#ifdef __NO_RELIABILITY_CODE__
    t2DSpatialData.RX = t2DSpatialData.RY = 1.0;
#else
    t2DSpatialData.RX = (t2DSpatialData.RDX + t2DSpatialData.RCX) / 2.0;
    t2DSpatialData.RY = (t2DSpatialData.RDY + t2DSpatialData.RCY) / 2.0;
#endif
    //Data calculation related to VX, and VY
    double VXData, VYData, RDVXData, RDVYData;
    double max_2Dchange = m_pixelAcuity / secDiffSequence[0];

    //If last element to be added has been found and we have other element too
    if(foundSupport[0] && numberOfFound >= 2) {
        int i;
        for(i=1; i<m_blobsBufferSize; i++)
            if(foundSupport[i])
                break;
        Rectangle<int> *previous_bbox = &bboxesToAnalyze[i];

        double sdiff = secDiffToCurrent[i];

        VXData = ( XData - RECT_XCENTER(previous_bbox) ) / secDiffToCurrent[i];
        VYData = ( YData - RECT_YCENTER(previous_bbox) ) / secDiffToCurrent[i];

        //Dimensional Reliability for VX, and VY
#ifdef __NO_RELIABILITY_CODE__
        Rdistance = RDVXData = RDVYData = 1.0;
#else
        Rdistance = 1.0;
        RDVXData = ( RXData + ( Rdistance + (1.0 - (dpFlags[i] & MM_HORIZONTAL_OCCL_MASK) ? 1.0 : 0.0) ) / 2.0 ) / 2.0;
        RDVYData = ( RYData + ( Rdistance + (1.0 - (dpFlags[i] & MM_VERTICAL_OCCL_MASK)   ? 1.0 : 0.0) ) / 2.0 ) / 2.0;
#endif
        //For VX
        if(t2DSpatialData.RVX < 0.0000001) { //If no found elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t2DSpatialData.VX = VXData;
            t2DSpatialData.RCVX = 1.0;
            t2DSpatialData.SDVX = max_2Dchange; //SD is initialized to restart recursion
            t2DSpatialData.RDVX = i2DData.sumRDVX = RDVXData;
            t2DSpatialData.RVX = (RDVXData + 1.0) / 2.0;
        } else {
            auxSum = i2DData.sumRDVX;
            i2DData.sumRDVX = RDVXData + currentCooling * auxSum;
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDVX = 1.0;
#else
            t2DSpatialData.RDVX = i2DData.sumRDVX / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses old VX value, so computation order IS important.
            curDiff = (VXData - t2DSpatialData.VX) / visualSupport[0];
            curSD = t2DSpatialData.SDVX;
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RCVX = 1.0;
#else
            t2DSpatialData.RCVX =     normalisedSigmaCoherenceReliability(t2DSpatialData.SDVX, m_pixelAcuity/tdiff);
#endif
            t2DSpatialData.SDVX = sqrt( (currentCooling*auxSum/i2DData.sumRDVX) * (curSD*curSD + RDVXData*(curDiff*curDiff/i2DData.sumRDVX)) );
            if(!(currentVisualState & MM_OBJECT_LOST)) //Freeze Data if object lost
                t2DSpatialData.VX = (VXData*RDVXData + currentCooling*t2DSpatialData.VX*auxSum) / i2DData.sumRDVX;
            //t2DSpatialData.RCVX = DimensionalCoherenceReliability(t2DSpatialData.SDVX, 0.0, max_2Dchange);
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RVX = 1.0;
#else
            t2DSpatialData.RVX = RVFoundSolutions * (t2DSpatialData.RDVX + t2DSpatialData.RCVX) / 2.0;
#endif
        }

        //For VY
        if(t2DSpatialData.RVY < 0.0000001) { //If no found elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t2DSpatialData.VY = VYData;
            t2DSpatialData.RCVY = 0.0;
            t2DSpatialData.SDVY = 0.0; //SD is initialized to restart recursion
            t2DSpatialData.RDVY = i2DData.sumRDVY = RDVYData;
            t2DSpatialData.RVY = RDVYData / 2.0;
        } else {
            auxSum = i2DData.sumRDVY;
            i2DData.sumRDVY = RDVYData + currentCooling * auxSum;
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDVY = 1.0;
#else
            t2DSpatialData.RDVY = i2DData.sumRDVY / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses old VY value, so computation order IS important.
            curDiff = (VYData - t2DSpatialData.VY) / visualSupport[0];
            curSD = t2DSpatialData.SDVY;
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RCVY = 1.0;
#else
            t2DSpatialData.RCVY = normalisedSigmaCoherenceReliability(t2DSpatialData.SDVY, m_pixelAcuity/tdiff);
#endif
            t2DSpatialData.SDVY = sqrt( (currentCooling*auxSum/i2DData.sumRDVY) * (curSD*curSD + RDVYData*(curDiff*curDiff/i2DData.sumRDVY)) );
            if(!(currentVisualState & MM_OBJECT_LOST)) //Freeze Data if object lost
                t2DSpatialData.VY = (VYData*RDVYData + currentCooling*t2DSpatialData.VY*auxSum) / i2DData.sumRDVY;
            //t2DSpatialData.RCVY = DimensionalCoherenceReliability(t2DSpatialData.SDVY, 0.0, max_2Dchange);
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RVY = 1.0;
#else
            t2DSpatialData.RVY = RVFoundSolutions * (t2DSpatialData.RDVY + t2DSpatialData.RCVY) / 2.0;
#endif
        }
    } else { //The current element has not been classified or there is no enough information for determining VX, and VY
        //For VX
        if(t2DSpatialData.RDVX < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t2DSpatialData.SDX > fabs(t2DSpatialData.VX))  //If standard deviation is higher than velocity
                //estimation is very likely that velocity value
                //is not significant. Better to take null velocity
                //in this case. Else VX won't change.
                t2DSpatialData.VX = 0.0;
            i2DData.sumRDVX = t2DSpatialData.RCVX = t2DSpatialData.RVX = 0.0;
            t2DSpatialData.SDVX = max_2Dchange;
        } else { //If not classified and we have previous information
            auxSum = i2DData.sumRDVX;
            i2DData.sumRDVX = currentCooling * auxSum; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDVX = 1.0;
            t2DSpatialData.RCVX = 1.0;
            t2DSpatialData.RVX = 1.0;
#else
            t2DSpatialData.RDVX = i2DData.sumRDVX / iGData.sumCooling2D;
            //VX and SD Value will not change in this case, so RC coherence also will not change.
            t2DSpatialData.RVX = RVFoundSolutions * (t2DSpatialData.RDVX + t2DSpatialData.RCVX) / 2.0;
#endif
        }

        //For VY
        if(t2DSpatialData.RDVY < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t2DSpatialData.SDY > fabs(t2DSpatialData.VY))  //If standard deviation is higher than velocity
                //estimation is very likely that velocity value
                //is not significant. Better to take null velocity
                //in this case. Else VY won't change.
                t2DSpatialData.VY = 0.0;
            i2DData.sumRDVY = t2DSpatialData.RCVY = t2DSpatialData.RVY = 0.0;
            t2DSpatialData.SDVY = max_2Dchange;
        } else { //If not classified and we have previous information
            auxSum = i2DData.sumRDVY;
            i2DData.sumRDVY = currentCooling * auxSum; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DSpatialData.RDVY = 1.0;
            t2DSpatialData.RCVY = 1.0;
            t2DSpatialData.RVY = 1.0;
#else
            t2DSpatialData.RDVY = i2DData.sumRDVY / iGData.sumCooling2D;
            //VY and SD Value will not change in this case, so RC coherence also will not change.
            t2DSpatialData.RVY = RVFoundSolutions * (t2DSpatialData.RDVY + t2DSpatialData.RCVY) / 2.0;
#endif
        }
    }

    //Complete General Coherence Probability and Reliability
    PV2DC = RFoundSolutions * (t2DSpatialData.RCX + t2DSpatialData.RCY) / 2.0;
#ifdef __NO_RELIABILITY_CODE__
    RV2DC = 1.0;
#else
    RV2DC = RFoundSolutions * (t2DSpatialData.RDX + t2DSpatialData.RDY) / 2.0;
#endif


}

//Incremental version for 2D Dimensions update
void MobileObject2D::incrementalUpdate2DDimensions(int bufferSize, Rectangle<int> *bboxesToAnalyze, DetectionProblemType *dpFlags, double *visualSupport, double maxDistanceFactor = 1.0) {

    Rectangle<int> *current_bbox = &bboxesToAnalyze[0];
    double tdiff = secDiffSequence[0], Rdistance;

    //Trivial case for an initial blobHistory
    if(bufferSize == 1) {
        t2DDimData.W = RECT_WIDTH(current_bbox);
        t2DDimData.H = RECT_HEIGHT(current_bbox);

        t2DDimData.SDW = t2DDimData.SDH = m_pixelAcuity; //Initial incremental SD

#ifdef __NO_RELIABILITY_CODE__
        i2DData.sumRDW = t2DDimData.RDW = 1.0;
        i2DData.sumRDH = t2DDimData.RDH = 1.0;
        t2DDimData.RCW = t2DDimData.RCH = 1.0;
        t2DDimData.RW = t2DDimData.RH   = 1.0;
#else
        double Rdistance = dimensional2DReliability(t2DDimData.W, t2DDimData.H); //Rdim
        //Evaluate reliability as mean between difficulty of obtained data (Rdim) and coherence with previous one (Rcoh (initially 1))
        i2DData.sumRDW = t2DDimData.RDW = Rdistance;
        i2DData.sumRDH = t2DDimData.RDH = Rdistance;
        t2DDimData.RCW = t2DDimData.RCH = 1.0;
        t2DDimData.RW = (t2DDimData.RDW + 1.0) / 2.0;
        t2DDimData.RH = (t2DDimData.RDH + 1.0) / 2.0;        
//        i2DData.sumRDW = t2DDimData.RDW = visualSupport[0]*( Rdistance + (1.0 - (dpFlags[0] & MM_HORIZONTAL_OCCL_MASK) ? 1.0 : 0.0) ) / 2.0;
//        i2DData.sumRDH = t2DDimData.RDH = visualSupport[0]*( Rdistance + (1.0 - (dpFlags[0] & MM_VERTICAL_OCCL_MASK)   ? 1.0 : 0.0) ) / 2.0;
#endif
        i2DData.sumRDVW = i2DData.sumRDVH = t2DDimData.VW = t2DDimData.VH = 0.0;
        t2DDimData.SDVW = t2DDimData.SDVH = 0.0;

        P2D = 0.0;
#ifdef __NO_RELIABILITY_CODE__
        R2D = 1.0;
#else
        R2D = 0.0;
#endif
        return;
    }

    double WData, HData, Wcur, Hcur, RWData, RHData, auxSum, curSD, curDiff;
    double currentCooling = coolingValue[1];
    double initialSDW = 0.0, initialSDH = 0.0;

    //If last element to be added has been found
    if(foundSupport[0]) {
        if(t2DDimData.RW < 0.0000001)
            initialSDW = m_pixelAcuity;
        if(t2DDimData.RH < 0.0000001)
            initialSDH = m_pixelAcuity;

        //DATA
        //Get current estimation, according to model.
        Wcur = t2DDimData.W + t2DDimData.VW*tdiff;
        Hcur = t2DDimData.H + t2DDimData.VH*tdiff;

        //Get current data
        WData = RECT_WIDTH(current_bbox);
        HData = RECT_HEIGHT(current_bbox);

        //RELIABILITY
        //Set data reliability: Add COHERENCE!!!
#ifdef __NO_RELIABILITY_CODE__
        RHData = RWData = 1.0;
#else
        RHData = RWData = dimensional2DReliability(WData, HData);
#endif
        //Cool old reliability W
        auxSum = i2DData.sumRDW;
        i2DData.sumRDW = RWData + currentCooling*auxSum;
#ifdef __NO_RELIABILITY_CODE__
        t2DDimData.RDW = 1.0;
#else
        t2DDimData.RDW = i2DData.sumRDW / iGData.sumCooling2D;
#endif
        //Incremental SD Formula uses Wcur value, storing failures in estimation on real data, representing COHERENCE
        curDiff = (WData - Wcur);
        curSD = t2DDimData.SDW;
        t2DDimData.SDW = sqrt( (currentCooling*auxSum/i2DData.sumRDW) * (curSD*curSD + RWData*(curDiff*curDiff/i2DData.sumRDW)) );
#ifdef __NO_RELIABILITY_CODE__
        t2DDimData.RCW = 1.0;
#else
        t2DDimData.RCW = normalisedSigmaCoherenceReliability(t2DDimData.SDW, m_pixelAcuity);
#endif
        t2DDimData.W = (WData*RWData + currentCooling*auxSum*Wcur) / i2DData.sumRDW;

        //Cool old reliability H
        auxSum = i2DData.sumRDH;
        i2DData.sumRDH = RHData + currentCooling*auxSum;
#ifdef __NO_RELIABILITY_CODE__
        t2DDimData.RDH = 1.0;
#else
        t2DDimData.RDH = i2DData.sumRDH / iGData.sumCooling2D;
#endif
        //Incremental SD Formula uses Wcur value, storing failures in estimation on real data, representing COHERENCE
        curDiff = (HData - Hcur);
        curSD = t2DDimData.SDH;
        t2DDimData.SDH = sqrt( (currentCooling*auxSum/i2DData.sumRDH) * (curSD*curSD + RHData*(curDiff*curDiff/i2DData.sumRDH)) );
#ifdef __NO_RELIABILITY_CODE__
        t2DDimData.RCH = 1.0;
#else
        t2DDimData.RCH = normalisedSigmaCoherenceReliability(t2DDimData.SDH, m_pixelAcuity);
#endif
        t2DDimData.H = (HData*RHData + currentCooling*auxSum*Hcur) / i2DData.sumRDH;

    } else { //The current element has not been found
        //Freeze current dimensional estimation, as object is lost.
        Wcur = t2DDimData.W;
        Hcur = t2DDimData.H;
        //Wcur = t2DDimData.W + t2DDimData.VW*tdiff;
        //Hcur = t2DDimData.H + t2DDimData.VH*tdiff;
        if(Wcur < 2)
            Wcur = t2DDimData.W;
        if(Hcur < 2)
            Hcur = t2DDimData.H;

        //For W
        if(t2DDimData.RW < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i2DData.sumRDW = t2DDimData.RDW = t2DDimData.RCW = 0.0;
            t2DDimData.SDW = m_pixelAcuity;
        } else { //If not classified and we have previous information
            //Incremental step
            auxSum = i2DData.sumRDW;
            i2DData.sumRDW = currentCooling * i2DData.sumRDW; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDW = 1.0;
#else
            t2DDimData.RDW = i2DData.sumRDW / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses Wcur value, storing failures in estimation on real data, representing COHERENCE
            curSD = t2DDimData.SDW;
            t2DDimData.SDW = sqrt( (currentCooling*auxSum/i2DData.sumRDW) * (curSD*curSD) );
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RCW = 1.0;
#else
            t2DDimData.RCW = normalisedSigmaCoherenceReliability(t2DDimData.SDW, m_pixelAcuity);
#endif
            t2DDimData.W = Wcur; //Trust the estimate
        }

        //For H
        if(t2DDimData.RH < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            i2DData.sumRDH = t2DDimData.RDH = t2DDimData.RCH = 0.0;
            t2DDimData.SDH = m_pixelAcuity;
        } else { //If not classified and we have previous information
            //Incremental step
            auxSum = i2DData.sumRDH;
            i2DData.sumRDH = currentCooling * i2DData.sumRDH; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDH = 1.0;
#else
            t2DDimData.RDH = i2DData.sumRDH / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses Hcur value, storing failures in estimation on real data, representing COHERENCE
            curSD = t2DDimData.SDH;
            t2DDimData.SDH = sqrt( (currentCooling*auxSum/i2DData.sumRDH) * (curSD*curSD) );
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RCH = 1.0;
#else
            t2DDimData.RCH = normalisedSigmaCoherenceReliability(t2DDimData.SDH, m_pixelAcuity);
#endif
            t2DDimData.H = Hcur; //Trust the estimate
        }
    }
#ifdef __NO_RELIABILITY_CODE__
    t2DDimData.RW = t2DDimData.RH = 1.0;
#else
    t2DDimData.RW = (t2DDimData.RDW + t2DDimData.RCW) / 2.0;
    t2DDimData.RH = (t2DDimData.RDH + t2DDimData.RCH) / 2.0;
#endif
    //Data calculation related to VW, and VH
    double VWData, VHData, RDVWData, RDVHData;
    double max_2Dchange = m_pixelAcuity / secDiffSequence[0];

    //If last element to be added has been found and we have other element too
    if(foundSupport[0] && numberOfFound >= 2) {
        int i;
        for(i=1; i<m_blobsBufferSize; i++)
            if(foundSupport[i])
                break;
        Rectangle<int> *previous_bbox = &bboxesToAnalyze[i];

        VWData = ( WData - RECT_WIDTH  (previous_bbox) ) / secDiffToCurrent[i];
        VHData = ( HData - RECT_HEIGHT (previous_bbox) ) / secDiffToCurrent[i];

#ifdef __NO_RELIABILITY_CODE__
        //Dimensional Reliability for VW, and VH
        Rdistance = RDVWData = RDVHData = 1.0;
#else
        //Dimensional Reliability for VW, and VH
        Rdistance = dimensional2DReliability(RECT_WIDTH(previous_bbox), RECT_HEIGHT(previous_bbox));
        RDVWData = ( RWData + ( Rdistance + (1.0 - (dpFlags[i] & MM_HORIZONTAL_OCCL_MASK) ? 1.0 : 0.0) ) / 2.0 ) / 2.0;
        RDVHData = ( RHData + ( Rdistance + (1.0 - (dpFlags[i] & MM_VERTICAL_OCCL_MASK)   ? 1.0 : 0.0) ) / 2.0 ) / 2.0;
#endif
        //For VW
        if(t2DDimData.RVW < 0.0000001) { //If no found elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t2DDimData.VW = VWData;
            t2DDimData.RCVW = 1.0;
            t2DDimData.SDVW = max_2Dchange; //SD is initialized to restart recursion
            t2DDimData.RDVW = i2DData.sumRDVW = RDVWData;
            t2DDimData.RVW = (RDVWData + 1.0) / 2.0;
        } else {
            auxSum = i2DData.sumRDVW;
            i2DData.sumRDVW = RDVWData + currentCooling * auxSum;
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDVW = 1.0;
#else
            t2DDimData.RDVW = i2DData.sumRDVW / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses old VW value, so computation order IS important.
            curDiff = (VWData - t2DDimData.VW) / visualSupport[0];
            curSD = t2DDimData.SDVW;
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RCVW = 1.0;
#else
            t2DDimData.RCVW =     normalisedSigmaCoherenceReliability(t2DDimData.SDVW, m_pixelAcuity/tdiff);
#endif
            t2DDimData.SDVW = sqrt( (currentCooling*auxSum/i2DData.sumRDVW) * (curSD*curSD + RDVWData*(curDiff*curDiff/i2DData.sumRDVW)) );
            if(!(currentVisualState & MM_OBJECT_LOST)) //Freeze Data if object lost
                t2DDimData.VW = (VWData*RDVWData + currentCooling*t2DDimData.VW*auxSum) / i2DData.sumRDVW;
            //t2DDimData.RCVW = DimensionalCoherenceReliability(t2DDimData.SDVW, 0.0, max_2Dchange);
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RVW = 1.0;
#else
            t2DDimData.RVW = RVFoundSolutions * (t2DDimData.RDVW + t2DDimData.RCVW) / 2.0;
#endif
        }

        //For VH
        if(t2DDimData.RVH < 0.0000001) { //If no found elements has been already added in later frames
            //In case of new classified element after a period of no classification, the new classified value is considered.
            t2DDimData.VH = VHData;
            t2DDimData.RCVH = 1.0;
            t2DDimData.SDVH = 0.0; //SD is initialized to restart recursion
            t2DDimData.RDVH = i2DData.sumRDVH = RDVHData;
            t2DDimData.RVH = RDVHData / 2.0;
        } else {
            auxSum = i2DData.sumRDVH;
            i2DData.sumRDVH = RDVHData + currentCooling * auxSum;
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDVH = 1.0;
#else
            t2DDimData.RDVH = i2DData.sumRDVH / iGData.sumCooling2D;
#endif
            //Incremental SD Formula uses old VH value, so computation order IS important.
            curDiff = (VHData - t2DDimData.VH) / visualSupport[0];
            curSD = t2DDimData.SDVH;
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RCVH = 1.0;
#else
            t2DDimData.RCVH = normalisedSigmaCoherenceReliability(t2DDimData.SDVH, m_pixelAcuity/tdiff);
#endif
            t2DDimData.SDVH = sqrt( (currentCooling*auxSum/i2DData.sumRDVH) * (curSD*curSD + RDVHData*(curDiff*curDiff/i2DData.sumRDVH)) );
            if(!(currentVisualState & MM_OBJECT_LOST)) //Freeze Data if object lost
                t2DDimData.VH = (VHData*RDVHData + currentCooling*t2DDimData.VH*auxSum) / i2DData.sumRDVH;
            //t2DDimData.RCVH = DimensionalCoherenceReliability(t2DDimData.SDVH, 0.0, max_2Dchange);
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RVH = 1.0;
#else
            t2DDimData.RVH = RVFoundSolutions * (t2DDimData.RDVH + t2DDimData.RCVH) / 2.0;
#endif
        }
    } else { //The current element has not been classified or there is no enough information for determining VW, and VH
        //For VW
        if(t2DDimData.RDVW < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t2DDimData.SDW > fabs(t2DDimData.VW))  //If standard deviation is higher than velocity
                //estimation is very likely that velocity value
                //is not significant. Better to take null velocity
                //in this case. Else VX won't change.
                t2DDimData.VW = 0.0;
            i2DData.sumRDVW = t2DDimData.RCVW = t2DDimData.RVW = 0.0;
            t2DDimData.SDVW = max_2Dchange;
        } else { //If not classified and we have previous information
            auxSum = i2DData.sumRDVW;
            i2DData.sumRDVW = currentCooling * auxSum; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDVW = t2DDimData.RCVW = t2DDimData.RVW = 1.0;
#else
            t2DDimData.RDVW = i2DData.sumRDVW / iGData.sumCooling2D;
            //VW and SD Value will not change in this case, so RC coherence also will not change.
            t2DDimData.RVW = RVFoundSolutions * (t2DDimData.RDVW + t2DDimData.RCVW) / 2.0;
#endif
        }

        //For VH
        if(t2DDimData.RDVH < 0.0000001) { //If no classified elements has been already added in later frames
            //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
            //data obtention
            if(t2DDimData.SDH > fabs(t2DDimData.VH))  //If standard deviation is higher than velocity
                //estimation is very likely that velocity value
                //is not significant. Better to take null velocity
                //in this case. Else VY won't change.
                t2DDimData.VH = 0.0;
            i2DData.sumRDVH = t2DDimData.RCVH = t2DDimData.RVH = 0.0;
            t2DDimData.SDVH = max_2Dchange;
        } else { //If not classified and we have previous information
            auxSum = i2DData.sumRDVH;
            i2DData.sumRDVH = currentCooling * auxSum; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
            t2DDimData.RDVH = t2DDimData.RCVH = t2DDimData.RVH = 1.0;
#else
            t2DDimData.RDVH = i2DData.sumRDVH / iGData.sumCooling2D;
            //VH and SD Value will not change in this case, so RC coherence also will not change.
            t2DDimData.RVH = RVFoundSolutions * (t2DDimData.RDVH + t2DDimData.RCVH) / 2.0;
#endif
        }
    }

    //Complete General Coherence Probability and Reliability
    P2D = RFoundSolutions * (t2DDimData.RCW + t2DDimData.RCH) / 2.0;
#ifdef __NO_RELIABILITY_CODE__
    R2D = 1.0;
#else
    R2D = RFoundSolutions * (t2DDimData.RDW + t2DDimData.RDH) / 2.0;
#endif
}

double MobileObject2D::NormalizeOrientation(double alpha) {
    return alpha < 0 ? fmod(alpha, M_PI) + M_PI : fmod(alpha, M_PI);
}

double MobileObject2D::minimalAngularDistance(double alpha1, double alpha2) {
    double
        na1 = NormalizeOrientation(alpha1),
        na2 = NormalizeOrientation(alpha2),
        d1 = fabs(na1 - na2), d2 = fabs( (na1 + M_PI) - na2);

    if(d1 < d2)
        return d1;

    return d2;
}


double MobileObject2D::NormalizeVelocityAngle(double theta) {
    return theta < 0 ? fmod(theta, 2*M_PI) + 2*M_PI : fmod(theta, 2*M_PI);
}

double MobileObject2D::DimensionalCoherenceReliability(double sigma_dim, double min, double max) {
#ifdef __NO_RELIABILITY_CODE__
    return 1.0;
#else
    return  1.0 - std::min(1.0, sigma_dim/(max - min));
#endif
}

double MobileObject2D::dimensional2DReliability(double blobW, double blobH) {
    //The farest the object is, it is more likely to loose detected pixels.
    //double Rdistance = 1.0 - std::min(1.0, distance2D/m_maxFocalDistance);
    //Biggest objects are less affected by segmentation errors.
#ifdef __NO_RELIABILITY_CODE__
    return 1.0;
#else
    return std::min(1.0, blobW*blobH/(m_reliableObjectLength*m_reliableObjectLength));
#endif
}

double MobileObject2D::position2DReliability(double distance2D) {
    //The farest the object is, it is more likely to loose precision.
    //The value is normalized with a constant representing the maximal posible 2D displacement
    //for the quickest known object (m_pixelAcuity)
    if (distance2D == 0.0)
        return 1.0;
    double Rdistance = std::min(1.0, m_pixelAcuity/distance2D);
#ifdef __NO_RELIABILITY_CODE__
    return 1.0;
#else
    return Rdistance;
#endif
}

void MobileObject2D::setNumberOfFramesNotSeen(int num) {
    numberOfFramesNotSeen = num;
}

void MobileObject2D::incrementNumberOfFramesNotSeen() {
    numberOfFramesNotSeen++;
}

//2D bounding box information infered from best quality mobile information
Blob *MobileObject2D::determineMostLikelyBlob() {
    Blob *newBlob = (*blobHistory.rbegin())->copyWithLists();

    //Use 2D Width and Height information, redardless its coherence, because it is all we have
    double X, Y,
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

    return newBlob;
}

//Getting Functions
double MobileObject2D::getGlobalProbability() {
    return P;
}

unsigned long MobileObject2D::getMobileId() {
    return mobile_id;
}

unsigned long MobileObject2D::getRMobileId() {
    return rmobile_id;
}

int MobileObject2D::getNumberOfFramesNotSeen() {
    return numberOfFramesNotSeen;
}

int MobileObject2D::getNumberOfFramesSinceFirstTimeSeen() {
    return numberOfFramesSinceFirstTimeSeen;
}

bool MobileObject2D::mobile2DCoherenceIsAcceptable() {
    if (    R2D >= m_DimensionalCoherenceReliabilityThreshold
         && P2D >= m_DimensionalCoherenceProbabilityThreshold )
        return true;
    return false;
}

bool MobileObject2D::mobile2DVelocityCoherenceIsAcceptable() {
    if (    RV2DC >= m_SpatialCoherenceReliabilityThreshold
         && PV2DC >= m_SpatialCoherenceProbabilityThreshold )
        return true;
    return false;
}


bool orderedByBestCoherenceOperator2D::operator()(SpMobileObject2D mobile1, SpMobileObject2D mobile2) {
    return mobile1->getGlobalProbability() >= mobile2->getGlobalProbability();
}

bool orderedByMobileIdOperator2D::operator()(SpMobileObject2D mobile1, SpMobileObject2D mobile2) {
    return mobile1->getMobileId() < mobile2->getMobileId();
}

std::ostream& operator<<(std::ostream& out,const SpMobileObject2D mo){
    int i;
    Shape3DData *current_s3d;
    bool rigid = false;

    out << "\t\t\tMobile id        : " << mo->mobile_id << std::endl;
#ifdef MOBILE_DEBUG_DATA 
    out << "\t\t\tDebug Flag: " << mo->debug_data_flag;
    out << "\tDetailed Detection Problem: " << Blob::getDPNameFromTypeDetailed(mo->currentVisualState);
    out << std::endl;
#endif
#ifdef MOBILE2D_DETAILS
#ifndef FOR_SPREADSHEET_MOBILE2D
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
#else //FOR_SPREDSHEET_MOBILE2D
    out << std::endl;
    out << "ID\tX\tSDX\tRDX\tRCX\tRX\tY\tSDY\tRDY\tRCY\tRY";
    out <<   "\tVX\tSDVX\tRDVX\tRCVX\tRVX\tVY\tSDVY\tRDVY\tRCVY\tRVY\tV2D\tSDV2D\tRDV2D\tRCV2D\tRV2D\ttheta2D\tSDtheta2D";
    out << "\tW\tSDW\tRDW\tRCW\tRW\tH\tSDH\tRDH\tRCH\tRH)";
    out << "\tVW\tSDVW\tRDVW\tRCVW\tRVW\tVH\tSDVH\tRDVH\tRCVH\tRVH)" << std::endl;

    //same
    out << "\t" << mo->mobile_id;
    out << "\t" << mo->t2DSpatialData.X << "\t" << mo->t2DSpatialData.SDX
        << "\t" << mo->t2DSpatialData.RDX << "\t" << mo->t2DSpatialData.RCX << "\t" << mo->t2DSpatialData.RX;
    out << "\t" << mo->t2DSpatialData.Y << "\t" << mo->t2DSpatialData.SDY
        << "\t" << mo->t2DSpatialData.RDY << "\t" << mo->t2DSpatialData.RCY << "\t" << mo->t2DSpatialData.RY;
    out << "\t" << mo->t2DSpatialData.VX << "\t" << mo->t2DSpatialData.SDVX
        << "\t" << mo->t2DSpatialData.RDVX << "\t" << mo->t2DSpatialData.RCVX << "\t" << mo->t2DSpatialData.RVX;
    out << "\t" << mo->t2DSpatialData.VY << "\t" << mo->t2DSpatialData.SDVY
        << "\t" << mo->t2DSpatialData.RDVY << "\t" << mo->t2DSpatialData.RCVY << "\t" << mo->t2DSpatialData.RVY;
    out << "\t" << mo->t2DSpatialData.V2D << "\t" << mo->t2DSpatialData.SDV2D
        << "\t" << mo->t2DSpatialData.RDV2D << "\t" << mo->t2DSpatialData.RCV2D << "\t" << mo->t2DSpatialData.RV2D;
    out << "\t" << mo->t2DSpatialData.theta2D << "\t" << mo->t2DSpatialData.SDtheta2D;
    out << "\t" << mo->t2DDimData.W << "\t" << mo->t2DDimData.SDW
        << "\t" << mo->t2DDimData.RDW << "\t" << mo->t2DDimData.RCW << "\t" << mo->t2DDimData.RW;
    out << "\t" << mo->t2DDimData.H << "\t" << mo->t2DDimData.SDH
        << "\t" << mo->t2DDimData.RDH << "\t" << mo->t2DDimData.RCH << "\t" << mo->t2DDimData.RH;
    out << "\t" << mo->t2DDimData.VW << "\t" << mo->t2DDimData.SDVW
        << "\t" << mo->t2DDimData.RDVW << "\t" << mo->t2DDimData.RCVW << "\t" << mo->t2DDimData.RVW;
    out << "\t" << mo->t2DDimData.VH << "\t" << mo->t2DDimData.SDVH
        << "\t" << mo->t2DDimData.RDVH << "\t" << mo->t2DDimData.RCVH << "\t" << mo->t2DDimData.RVH;
#endif//FOR_SPREDSHEET_MOBILE2D
#endif
    //    out << "\t\t\t2D Velocity(VX2D,VY2D) : (" << mo->t2DSpatialData.VX2D<<" , " << mo->t2DSpatialData.VY2D<<")" << std::endl;
    //   out << "\t\t\tBlob History     : " << std::endl<<mo->blobHistory<<std::endl;
    //Better to show current chosen s3dsToAnalyzeList
#ifdef SHOW_BLOB_BUFFER
    Rectangle<int> *current_rect;
    for(i=0; i<mo->currentBufferSize; i++) {
        current_rect = &(mo->bboxesToAnalyze[i]);
        out << "\t\t\t\t\t" << " (W,L) : (" << RECT_WIDTH(current_rect) << ", " << RECT_HEIGHT(current_rect) << ")" << std::endl;
        out << "\t\t\t\t\t" << " (X,Y) : (" << RECT_XCENTER(current_rect) << ", " << RECT_YCENTER(current_rect) << ")" << std::endl;
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
    out << "\t\t\t#frames not seen : " << mo->numberOfFramesNotSeen << std::endl;
    out << "\t\t\t2D Coherence(P2D,R2D) : \t(" << mo->P2D << ", " << mo->R2D<<")" << std::endl;
    out << "\t\t\t2D Velocity Coherence(PV2DC,RV2DC) : \t(" << mo->PV2DC << ", " << mo->RV2DC << ")" << std::endl;
    out << "\t\t\tGlobal Probability : \t\t" << mo->P << std::endl;

    if(mo->involvedBlobs != NULL) {
        out <<  "InvolvedBlobs:" << std::endl;
        for(int i=0; i<MobileObject2D::m_currentTrackingBlobsNumber; i++)
            out << "\t" << mo->involvedBlobs[i];
        out << std::endl;
    }

    if(mo->usedBlobs != NULL) {
        out <<  "UsedBlobs:" << std::endl;
        for(int i=0; i<mo->usedLength; i++)
            out << "\t" << mo->usedBlobs[i];
        out << std::endl;
    }

    return out;
}
