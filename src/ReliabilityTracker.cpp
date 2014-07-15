#include "ReliabilityClassification.h"
#include "ReliabilityTracker.h"
#include "ReliableMobileObjectList.h"
#include "VideoAnalysis.h"
#include "ImageHeader.h"
#include <cmath>
#include <deque>
#include "geometric.h"
#include "blob.h"
#include <QMessageBox>
#include <QSharedPointer>

int ReliabilityTracker::m_alternativeNumber = 0;
int ReliabilityTracker::acceptedPixelError = 0;
double ReliabilityTracker::accepted3DFeatureError = 0.0;
double ReliabilityTracker::acceptedOrientationError = 0.0;

double ReliabilityTracker::m_highVisualSupportThreshold = 0.95;

int ReliabilityTracker::m_meanMillisecondsDifferenceBetweenFrames = 110;

ReliabilityTracker::ReliabilityTracker(Datapool *i_data) : m_data(i_data), parametersInitialised(false) {
    m_RMerge = new ReliabilityMerge(m_data);
    m_PreMerge = new ReliabilityMerge(m_data);    
    m_PreMerge->m_rc = m_RMerge->m_rc = MobileObject::m_rc = m_rclassif = new ReliabilityClassification(m_data);
    initialPreparation = true;
    checkedMobilePairValidity = NULL;
    validMobilePair = NULL;
}

void ReliabilityTracker::getMostLikelyMobileObjects(std::deque<SpMobileObject> &mobileObjectsOutput) {
    std::deque<SpReliableMobileObject>::iterator rmobile_it, rend = rMobilesList.end();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralternative_it;
    std::deque<SpMobileObject>::iterator mobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> *asolutions;
    SpRMAlternativeSolution bestAlternative;
    bool justUnknownSolutions = true;

    mobileObjectsOutput.clear();

    if(rMobilesList.empty())
        return;

    for(rmobile_it = rMobilesList.begin(); rmobile_it != rend; rmobile_it++) {
        asolutions = (*rmobile_it)->getAlternativeSolutions();
        justUnknownSolutions = true;
        for(ralternative_it = asolutions->begin(); ralternative_it != asolutions->end(); ralternative_it++) {
            bestAlternative = (*ralternative_it);
            if(alternativeWith3DInformation(bestAlternative)) {
                justUnknownSolutions = false;
                break;
            }
        }

        if(justUnknownSolutions)
            bestAlternative = *(asolutions->begin());

        for(mobile_it = bestAlternative->begin(); mobile_it != bestAlternative->end(); mobile_it++)
            mobileObjectsOutput.push_back(*mobile_it);
    }
}


bool ReliabilityTracker::initialPrepareRun() {
    if( m_data->fgImage == NULL) {
        AppendToLog("ReliabilityTracker: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }

    MobileObject::m_pSegmentation = m_pSegmentation = m_data->fgImage;
    initialPreparation = false;

    MobileObject::m_maxFocalDistance = getMaxFocalDistanceToImageCorner();
    //A quarter of the image surface
    MobileObject::m_objectSizeForMaxReliability = getObjectSizeForMaxReliability(meanw, meanl, meanh);
    MobileObject::m_objectDimensionForMaxReliability = sqrt(MobileObject::m_objectSizeForMaxReliability);

    return true;
}

bool ReliabilityTracker::prepareRun() {

    if( m_data->fgImage == NULL) {
        AppendToLog("ReliabilityTracker: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }
    MobileObject::m_pSegmentation = m_pSegmentation = m_data->fgImage;
    if (activatePreMerge && !m_PreMerge->prepareRun()) {
        AppendToLog("ReliabilityTracker: error: Error preparing execution for preMerge step.");
        return false;
    }

    if (!m_RMerge->prepareRun()) {
        AppendToLog("ReliabilityTracker: error: Error preparing execution for ReliabilityMerge capabilities.");
        return false;
    }

    if (!m_rclassif->prepareRun()) {
        AppendToLog("ReliabilityTracker: error: Error preparing execution for ReliabilityClassification capabilities.");
        return false;
    }
    return true;
}

bool ReliabilityTracker::init() {
    //TODO: Set as parameter
    //maxObjectSpeed = 200;
    //maxObjectSpeed = i_maxObjectSpeed;
    //   std::cout << "SPEED:" << maxObjectSpeed << "\n";

    if(m_data->objectModels.size() == 0) {
        AppendToLog("ReliabilityTracker: error: At least one ObjectModel definition is required (ContextModule does it). See file 'config/parallelpiped-models.xml' as an example.");
        return false;
    }

    if( (smodel = m_data->sceneModel) == NULL) {
        AppendToLog("ReliabilityTracker: error: SceneModel is NULL. Initialization required (ContextModule does it). ");
        return false;
    }

    m_RMerge->init();
    m_PreMerge->init();
    m_rclassif->init();

    objectModels = &m_data->objectModels;

    checkedMobilePairValidity = NULL;
    validMobilePair = NULL;

    initStaticMobileObject();
    mobileAlternativesMap.clear();
    mobile_id_counter = 0;
    rmobile_id_counter = 0;
    currentTimeMilliSeconds = 0;
    //For detecting first frame
    lastTimeStamp.millisecond = -1;
    lastMilliSecondsDifference = 0;
    MobileObject::m_firstFrame = firstFrame = true;

    return true;
}

void ReliabilityTracker::initStaticMobileObject() {

    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
    ObjectType cur_type;
    SpModelInterface cur_model;
    bool cur_rigid;
    int i, j, numSubModels, numModels = objectModels->size();


    MobileObject::m_context = smodel;
    MobileObject::m_maxSpeed = maxObjectSpeed;
    MobileObject::rigidModel.clear();
    MobileObject::lastFoundSubtypeTemplate.clear();

    MobileObject::objectModelMap.clear();
    MobileObject::objectSubModelMap.clear();
    MobileObject::objectModelsList = new SpModelInterface[numModels];
    MobileObject::objectSubModelsList = new SpModelInterface*[numModels];

    MobileObject::objectSubModelMinWidth = new double*[numModels];
    MobileObject::objectSubModelMeanWidth = new double*[numModels];
    MobileObject::objectSubModelMaxWidth = new double*[numModels];
    MobileObject::objectSubModelMinLength = new double*[numModels];
    MobileObject::objectSubModelMeanLength = new double*[numModels];
    MobileObject::objectSubModelMaxLength = new double*[numModels];
    MobileObject::objectSubModelMinHeight = new double*[numModels];
    MobileObject::objectSubModelMeanHeight = new double*[numModels];
    MobileObject::objectSubModelMaxHeight = new double*[numModels];
    MobileObject::objectSubModelMinVelocity = new double*[numModels];
    MobileObject::objectSubModelMeanVelocity = new double*[numModels];
    MobileObject::objectSubModelMaxVelocity = new double*[numModels];

    MobileObject::objectModelMinWidth = new double[numModels];
    MobileObject::objectModelMeanWidth = new double[numModels];
    MobileObject::objectModelMaxWidth = new double[numModels];
    MobileObject::objectModelMinLength = new double[numModels];
    MobileObject::objectModelMeanLength = new double[numModels];
    MobileObject::objectModelMaxLength = new double[numModels];
    MobileObject::objectModelMinHeight = new double[numModels];
    MobileObject::objectModelMeanHeight = new double[numModels];
    MobileObject::objectModelMaxHeight = new double[numModels];
    MobileObject::objectModelMinVelocity = new double[numModels];
    MobileObject::objectModelMeanVelocity = new double[numModels];
    MobileObject::objectModelMaxVelocity = new double[numModels];

    MobileObject::m_objectModelsNumber = numModels;

    double maxVolume = 0.0, maxSpeed = 0.0, speed, mw, ml, mh, volume;

    for(modelsIt = objectModels->begin(), i = 0; i < numModels; modelsIt++, i++) {
        cur_type = (*modelsIt).first;
        MobileObject::objectModelsList[i] = cur_model = (*modelsIt).second;
        MobileObject::rigidModel[cur_type] = cur_rigid = cur_model->IsRigid;
        MobileObject::objectModelMap[cur_type] = i;

        MobileObject::objectModelMinWidth[i] = ( ( cur_model->m_mapCriteria )["width"].spFunction ) -> getMin();
        mw = MobileObject::objectModelMeanWidth[i] = ( ( cur_model->m_mapCriteria )["width"].spFunction ) -> getMean();
        MobileObject::objectModelMaxWidth[i] = ( ( cur_model->m_mapCriteria )["width"].spFunction ) -> getMax();
        MobileObject::objectModelMinLength[i] = ( ( cur_model->m_mapCriteria )["depth"].spFunction ) -> getMin();
        ml = MobileObject::objectModelMeanLength[i] = ( ( cur_model->m_mapCriteria )["depth"].spFunction ) -> getMean();
        MobileObject::objectModelMaxLength[i] = ( ( cur_model->m_mapCriteria )["depth"].spFunction ) -> getMax();
        MobileObject::objectModelMinHeight[i] = ( ( cur_model->m_mapCriteria )["height"].spFunction ) -> getMin();
        mh = MobileObject::objectModelMeanHeight[i] = ( ( cur_model->m_mapCriteria )["height"].spFunction ) -> getMean();
        MobileObject::objectModelMaxHeight[i] = ( ( cur_model->m_mapCriteria )["height"].spFunction ) -> getMax();
        MobileObject::objectModelMinVelocity[i] = ( ( cur_model->m_mapCriteria )["velocity"].spFunction ) -> getMin();
        MobileObject::objectModelMeanVelocity[i] = ( ( cur_model->m_mapCriteria )["velocity"].spFunction ) -> getMean();
        MobileObject::objectModelMaxVelocity[i] = ( ( cur_model->m_mapCriteria )["velocity"].spFunction ) -> getMax();

        volume = mw * ml * mh;
        speed = MobileObject::objectModelMaxVelocity[i];
        if(volume > maxVolume) {
            maxVolume = volume;
            meanw = mw;
            meanl = ml;
            meanh = mh;
        }

        if(speed > maxSpeed)
            maxSpeed = speed;

        MobileObject::lastFoundSubtypeTemplate[cur_type] = ST_UNKNOWN;

        if(cur_model->IsRigid)
            MobileObject::objectSubModelsList[i] = NULL;
        else { //Postural objects
            numSubModels = cur_model->m_mapPostures.size();
            MobileObject::objectSubModelsList[i] = new SpModelInterface[numSubModels];

            MobileObject::objectSubModelMinWidth[i] = new double[numSubModels];
            MobileObject::objectSubModelMeanWidth[i] = new double[numSubModels];
            MobileObject::objectSubModelMaxWidth[i] = new double[numSubModels];
            MobileObject::objectSubModelMinLength[i] = new double[numSubModels];
            MobileObject::objectSubModelMeanLength[i] = new double[numSubModels];
            MobileObject::objectSubModelMaxLength[i] = new double[numSubModels];
            MobileObject::objectSubModelMinHeight[i] = new double[numSubModels];
            MobileObject::objectSubModelMeanHeight[i] = new double[numSubModels];
            MobileObject::objectSubModelMaxHeight[i] = new double[numSubModels];
            MobileObject::objectSubModelMinVelocity[i] = new double[numSubModels];
            MobileObject::objectSubModelMeanVelocity[i] = new double[numSubModels];
            MobileObject::objectSubModelMaxVelocity[i] = new double[numSubModels];

            for(subModelsIt = cur_model->m_mapPostures.begin(), j = 0; j < numSubModels; subModelsIt++, j++) {
                MobileObject::objectSubModelsList[i][j] = (*subModelsIt).second;
                MobileObject::objectSubModelMap[cur_type][(*subModelsIt).first] = j;

                MobileObject::objectSubModelMinWidth[i][j]    = ( ( ((*subModelsIt).second)->m_mapCriteria )["width"].spFunction ) -> getMin();
                MobileObject::objectSubModelMeanWidth[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["width"].spFunction ) -> getMean();
                MobileObject::objectSubModelMaxWidth[i][j]    = ( ( ((*subModelsIt).second)->m_mapCriteria )["width"].spFunction ) -> getMax();
                MobileObject::objectSubModelMinLength[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["depth"].spFunction ) -> getMin();
                MobileObject::objectSubModelMeanLength[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["depth"].spFunction ) -> getMean();
                MobileObject::objectSubModelMaxLength[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["depth"].spFunction ) -> getMax();
                MobileObject::objectSubModelMinHeight[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["height"].spFunction ) -> getMin();
                MobileObject::objectSubModelMeanHeight[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["height"].spFunction ) -> getMean();
                MobileObject::objectSubModelMaxHeight[i][j]   = ( ( ((*subModelsIt).second)->m_mapCriteria )["height"].spFunction ) -> getMax();
                MobileObject::objectSubModelMinVelocity[i][j] = ( ( ((*subModelsIt).second)->m_mapCriteria )["velocity"].spFunction ) -> getMin();
                MobileObject::objectSubModelMeanVelocity[i][j] = ( ( ((*subModelsIt).second)->m_mapCriteria )["velocity"].spFunction ) -> getMean();
                MobileObject::objectSubModelMaxVelocity[i][j] = ( ( ((*subModelsIt).second)->m_mapCriteria )["velocity"].spFunction ) -> getMax();

            }
        }
    }

    MobileObject::m_classifThreshold = m_rclassif->m_classifThresInter;
    MobileObject::m_maxKnownSpeed = maxSpeed;

}

int ReliabilityTracker::getBetaDirection(SceneModel *smodel, QImage *m_pSegmentation) {
    double x0, y0, x1, y1;
    double angle, beta;

    //Problems getting the right angle difference
    return 1;

    double width = m_pSegmentation->width(), height = m_pSegmentation->height();

    SceneModel::imgToWorldCoordsGivenHeight(smodel->p_matrix, width/2.0, height/2.0, 0, &x0, &y0);
    SceneModel::imgToWorldCoordsGivenHeight(smodel->p_matrix, width/2.0 + 5, height/2.0, 0, &x1, &y1);

    if(x1 - x0 == 0) {
        if(y1 - y0 > 0)
            beta =   M_PI/2;
        else
            beta = - M_PI/2;
    } else
        beta = atan2(y1 - y0, x1 - x0);

    //Calculate 45 degrees in image plane angle, to understand the sense of the growth of beta in the xy world plane
    SceneModel::imgToWorldCoordsGivenHeight(smodel->p_matrix, width/2.0 + 5, height/2.0 - 5, 0, &x1, &y1);
    if(x1 - x0 == 0) {
        if(y1 - y0 > 0)
            angle =   M_PI/2;
        else
            angle = - M_PI/2;
    } else
        angle = atan2(y1 - y0, x1 - x0);

    if(angle > beta)
        return 1;

    return -1;

}


double ReliabilityTracker::getObjectDistanceForMaxReliability () {

    //For first frame there is no time difference, so we return the mean distance from border to center of the image

    return (m_pSegmentation->width()/2.0 + m_pSegmentation->height()/2.0) / 2.0;

    int W = m_pSegmentation->width(), H = m_pSegmentation->height();
    double xf = SM_CAMERA_X_FOC_POINT(smodel), yf = SM_CAMERA_Y_FOC_POINT(smodel);
    double Xf = SM_CAMERA_X2D_FOC_POINT(smodel), Yf = SM_CAMERA_Y2D_FOC_POINT(smodel);

    if(Yf < 0) {
        Xf = W/2.0;
        Yf = 0.0;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), Xf, Yf, 0.0, &xf, &yf);
    } else if (Yf > H) {
        Xf = W/2.0;
        Yf = H;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), Xf, Yf, 0.0, &xf, &yf);
    }

    //Taking an image displacement of 45 degrees
    double x2, y2, X2 = Xf + 5, Y2 = Yf + 5;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X2, Y2, 0.0, &x2, &y2);

    double alpha = atan2(y2 - yf, x2 - xf);
    double distance = MobileObject::m_maxKnownSpeed*MobileObject::secDiffSequence[0];

    x2 = xf + distance*cos(alpha);
    y2 = yf + distance*sin(alpha);

    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x2, y2, 0.0, &X2, &Y2);
    double
        dX = Xf - X2,
        dY = Yf - Y2;

    return sqrt(dX*dX + dY*dY);

}

double ReliabilityTracker::getObjectSizeForMaxReliability(double w, double l, double h) {
    double xf = SM_CAMERA_X_FOC_POINT(smodel), yf = SM_CAMERA_Y_FOC_POINT(smodel);
    double Xf = SM_CAMERA_X2D_FOC_POINT(smodel), Yf = SM_CAMERA_Y2D_FOC_POINT(smodel);
    double X2 = Xf + 5, Y2 = Yf + 5;
    double x2, y2;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X2, Y2, 0.0, &x2, &y2);
    double alpha = atan2(y2 - yf, x2 - xf);

    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    bb3D.getFromInitial3DPoint(smodel, &bb2D, xf, yf, 1, alpha, getBetaDirection(smodel, m_pSegmentation), w, l, h);
    return RECT_WIDTH(&bb2D)*RECT_HEIGHT(&bb2D);
}

double ReliabilityTracker::getMaxFocalDistanceToImageCorner() {
    double Xf = SM_CAMERA_X2D_FOC_POINT(smodel), Yf = SM_CAMERA_Y2D_FOC_POINT(smodel);
    //start by upper left corner (0,0)
    double auxX, auxY, dist, maxDistance = sqrt(Xf*Xf + Yf*Yf);

    //Now upper right corner (SM_IMG_WIDTH,0)
    auxX = m_pSegmentation->width() - Xf;
    dist = sqrt(auxX*auxX + Yf*Yf);
    if(dist > maxDistance)
        maxDistance = dist;

    //Now bottom left corner (0, SM_IMG_HEIGHT)
    auxY = m_pSegmentation->height() - Yf;
    dist = sqrt(Xf*Xf + auxY*auxY);
    if(dist > maxDistance)
        maxDistance = dist;

    //Finally bottom right corner (SM_IMG_WIDTH, SM_IMG_HEIGHT)
    auxX = m_pSegmentation->width() - Xf;
    auxY = m_pSegmentation->height() - Yf;
    dist = sqrt(auxX*auxX + auxY*auxY);
    if(dist > maxDistance)
        return dist;

    return maxDistance;
}

ReliabilityTracker::~ReliabilityTracker() {

    MobileObject::rigidModel.clear();
    MobileObject::lastFoundSubtypeTemplate.clear();

    biggestBlobForNewMobile.clear();
    usedIdsByAlternative.clear();

    delete[] MobileObject::objectModelsList;

    for(int i = 0; i < MobileObject::m_objectModelsNumber; i++)
        if(MobileObject::objectSubModelsList[i])
            delete[] MobileObject::objectSubModelsList[i];


    delete[] MobileObject::objectSubModelsList;


    if(MobileObject::secDiffSequence)
        delete[] MobileObject::secDiffSequence;

    if(MobileObject::coolingValue)
        delete[] MobileObject::coolingValue;

    if(MobileObject::secDiffToCurrent)
        delete[] MobileObject::secDiffToCurrent;

    if(checkedMobilePairValidity != NULL)
        delete[] checkedMobilePairValidity;

    if(validMobilePair != NULL)
        delete[] validMobilePair;
}

bool ReliabilityTracker::alternativeWith3DInformation(SpRMAlternativeSolution alternative) {
    std::deque<SpMobileObject>::iterator mobile_it;
    for(mobile_it = alternative->begin(); mobile_it != alternative->end(); mobile_it++)
        if((*mobile_it)-> getBestType() != UNKNOWN)
            return true;
    return false;
}


void ReliabilityTracker::preMerge(std::vector<Blob>& blobs) {

    if(activatePreMerge) {
        presetBlobsVectorAndInitialMergeTable(blobs);

        unsigned int i;
        std::cout << "Before order:" << std::endl;
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;
        Blob::orderByProximityToPoint(blobs, SM_CAMERA_X2D_FOC_POINT(smodel), SM_CAMERA_Y2D_FOC_POINT(smodel));
        std::cout << "After order:" << std::endl;
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;
        m_PreMerge->preMerge(initialMergeMap, blobs);
        std::cout << "After pre-merge:" << std::endl;
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;

        freeBlobsVectorAndInitialMergeTable();
    }
}

//TODO: Implement Split
void ReliabilityTracker::preSplit(std::vector<Blob>& blobs) {
}

void ReliabilityTracker::checkConnectivity(bool groupVector[], int elementsNumber, int referencePoint, int *elementsToAnalyzeVector) {
    int i;

    groupVector[referencePoint] = true;

    for(i=1; i<elementsNumber; i++)
        if(initialMergeMap[elementsToAnalyzeVector[referencePoint]][elementsToAnalyzeVector[i]] && groupVector[i] == false)
            checkConnectivity(groupVector, elementsNumber, i, elementsToAnalyzeVector);

}


bool ReliabilityTracker::validBlobMergeConfiguration(int mergeGroupNumber, int mergeLength, int blobsToMergeNumber, int *blobsToMerge, int *alternativesCombo) {
    bool validMerges[mergeLength];

    memset(validMerges, false, sizeof(bool)*mergeLength);

    int i, j, blobToMergeArray[mergeLength], trueConnections=0;

    for(i=0, j=0; i<blobsToMergeNumber; i++) {
        if(alternativesCombo[i] == mergeGroupNumber)
            blobToMergeArray[j++] = blobsToMerge[i];
        if(j == mergeLength)
            break;
    }

    validMerges[0] = true;
    for(j=1; j<mergeLength; j++) {
        if(initialMergeMap[blobToMergeArray[0]][blobToMergeArray[j]] && validMerges[j] == false)
            checkConnectivity(validMerges, mergeLength, j, blobToMergeArray);
    }

    for(i=0; i<mergeLength; i++)
        if(validMerges[i])
            trueConnections++;

    if(trueConnections == mergeLength)
        return true;

    return false;
}


bool ReliabilityTracker::blobCanBeIncludedForMerge(int currentMergeLength, int *mergedBlobIndexes, int currentBlobIndex) {
    int j;

    for(j=0; j<currentMergeLength; j++) {
        if(initialMergeMap[currentBlobIndex][mergedBlobIndexes[j]])
            return true;
    }

    return false;
}

void ReliabilityTracker::processMergeVector(int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber) {
    //First set length of each merged blob:
    int i, mergeLength[blobsToMergeNumber], finalBlobsNumber = 0;;

    for(i=0; i<blobsToMergeNumber; i++)
        mergeLength[i] = 0;

    for(i=0; i<blobsToMergeNumber; i++) {
        mergeLength[alternativesCombo[i]]++;
        if(alternativesCombo[i] > finalBlobsNumber)
            finalBlobsNumber = alternativesCombo[i];
    }

    finalBlobsNumber++;

    //Check validity of analyzed combination
    bool thereIsMerge =  finalBlobsNumber < blobsToMergeNumber ? true : false;

    if(thereIsMerge && mergeLength[0] < blobsToMergeNumber)
        for(i=0; i<blobsToMergeNumber && mergeLength[i]>0 ; i++)
            if(mergeLength[i] > 1 && !validBlobMergeConfiguration(i,mergeLength[i],blobsToMergeNumber,blobsToMerge,alternativesCombo))
                return;

    //Creating or getting merged blobs
    Blob *mergedBlobs[finalBlobsNumber];
    bool usedBlobsMatrix[finalBlobsNumber*blobsNumber];
    int j, k;
    memset(usedBlobsMatrix, false, finalBlobsNumber*blobsNumber*sizeof(bool));
    for(i=0; i<finalBlobsNumber; i++) {
        if(mergeLength[i] == 1) {
            for(j=0; j<blobsToMergeNumber; j++)
                if(alternativesCombo[j] == i) {
                    usedBlobsMatrix[blobsNumber*i + blobsToMerge[i]] = true;
                    mergedBlobs[i] =  blobsVector[blobsToMerge[i]];
                    break;
                }
        } else {
            int j, blobToMergeArray[mergeLength[i]];
            bool ascending = true;

            for(k=0, j=0; k<blobsToMergeNumber; k++) {
                if(alternativesCombo[k] == i) {
                    blobToMergeArray[j++] = blobsToMerge[k];
                    usedBlobsMatrix[blobsNumber*i + blobsToMerge[k]] = true;
                }
                if (j>1) //Check if blobs are ordered in ascending order
                    if(blobToMergeArray[j-2] > blobToMergeArray[j-1])
                        ascending = false;
                if(j == mergeLength[i])
                    break;
            }

            if(!ascending)
                orderAscending(blobToMergeArray, 0, mergeLength[i] - 1);

            //A merged blob is demanded but without being classified, to accelerate the process
            //Classification will be performed if necessary in case of a good 2D coherence after
            //a certain number of frames to obtain better computational performance.
            mergedBlobs[i] = m_RMerge->getMergedBlob(blobToMergeArray, mergeLength[i], blobsVector, blobsNumber, false);
        }
    }

    //Check if the combination contains any blob completly inside another one, in this case the alternative
    //is not considered because there will be another one representing the same alternative.
    if(thereIsMerge && combinationAlreadyIncluded(mergedBlobs, finalBlobsNumber) )
        return;

    //Generate an alternative solution for each set of blob merges
    insertNewMergeSolution(mergedBlobs, finalBlobsNumber, usedBlobsMatrix);
}

bool ReliabilityTracker::combinationAlreadyIncluded(Blob **blobsForAlternative, int blobsNumberForAlternative) {
    int i,j;
    if(blobsNumberForAlternative == 1)
        return false;

    for(i=0; i<blobsNumberForAlternative - 1; i++)
        for(j=i+1; j<blobsNumberForAlternative; j++)
            if(   Blob::isBlob1InsideBlob2(blobsForAlternative[j], blobsForAlternative[i])
               || Blob::isBlob1InsideBlob2(blobsForAlternative[i], blobsForAlternative[j]) )
                return true;
    return false;
}

void ReliabilityTracker::insertNewMergeSolution(Blob **mergedBlobs, int finalBlobsNumber, bool *usedBlobsMatrix) {
    SpRMAlternativeSolution newAltSolution;
    bool completing = false;
    int i, j, size = 0;
    if(g_baseAlternative != NULL) { //In case of an analyzed alternative solution of an already existing reliable mobile, duplicate mobile references in alternative solution
        SpRMAlternativeSolution altSolutionCopy(new RMAlternativeSolution(g_baseAlternative));
        newAltSolution = altSolutionCopy;
        size = newAltSolution->size();
        std::cerr << "from 1 size: " << size << std::endl;
        completing = true;
    } else { //New alternative solution for new mobile
        SpRMAlternativeSolution altSolutionCopy(new RMAlternativeSolution());
        newAltSolution = altSolutionCopy;
        size = newAltSolution->size();
        std::cerr << "from 2 size: " << size << std::endl;
    }


    usedIdsByAlternative.clear();
    if(completing) {
        bool valid;
        bool comparable[size];
        SpMobileObject trackedMobiles[size];
        std::deque<SpMobileObject>::iterator trackedMobiles_it;
        for(j=0, trackedMobiles_it = newAltSolution->begin(); j<size; j++, trackedMobiles_it++) {
            trackedMobiles[j] = *trackedMobiles_it;
            comparable[j] = trackedMobiles[j]->mobile3DCoherenceIsAcceptable();
        }

        for(i=0; i<finalBlobsNumber; i++) {
            //	std::cout << i << ": " << get_name_from_type(S3D_TYPE(mergedBlobs[i])) << std::endl;
            SpMobileObject newMobile = getNewMobileFromBlob(mergedBlobs[i]);

            newMobile->numUsed = 0;
            for(j=0; j < blobsNumber; j++) {
                newMobile->usedBlobs[j] = usedBlobsMatrix[blobsNumber*i + j];
                if(newMobile->usedBlobs[j])
                    newMobile->numUsed++;
            }

            valid = true;
            //New mobile object will be comparable if it is classified
            if(BLOB_TYPE(mergedBlobs[i]) != UNKNOWN && BLOB_P(mergedBlobs[i]) >= MobileObject::m_classifThreshold) {
                for(j=0; j<size; j++) {
                    if(comparable[j]) {
                        if(checkMobilePairValidity(newMobile, trackedMobiles[j]))
                            continue;
                        valid = false;
                        break;
                    } else {
                        if(checkMobilePairValidity(newMobile, trackedMobiles[j], true))
                            continue;
                        valid = false;
                        break;
                    }
                }
            } else {
                for(j=0; j<size; j++) {
                    if(comparable[j]) {
                        if(checkMobilePairValidity(newMobile, trackedMobiles[j], false))
                            continue;
                        valid = false;
                        break;
                    }
                }
            }
            if(valid)
                newAltSolution->insertNewMobileObject(newMobile);
        }
    } else { //Global Solution
        SpMobileObject newMobile;
        for(i=0; i<finalBlobsNumber; i++) {
            //std::cout << i << ": " << get_name_from_type(S3D_TYPE(mergedBlobs[i])) << std::endl;
            newAltSolution->insertNewMobileObject(newMobile = getNewMobileFromBlob(mergedBlobs[i]));
            newMobile->numUsed = 0;
            for(j=0; j < blobsNumber; j++) {
                newMobile->usedBlobs[j] = usedBlobsMatrix[blobsNumber*i + j];
                if(newMobile->usedBlobs[j])
                    newMobile->numUsed++;
            }
        }
    }

    newAltSolution->setAlternativeProbability();
    g_inserted_for_alternative++;
    g_newAlternatives.insert(newAltSolution);

}

void ReliabilityTracker::insertInMobileAlternativesMap(SpMobileObject mobile, long int id) {
    std::map< long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> >::iterator existing_set;

    if((existing_set = mobileAlternativesMap.find(id)) == mobileAlternativesMap.end()) {
        std::set<SpMobileObject, orderedByBestCoherenceOperator> newSet;
        newSet.insert(mobile);
        mobileAlternativesMap[id] = newSet;
        return;
    }

    existing_set->second.insert(mobile);
}

SpMobileObject ReliabilityTracker::getNewMobileFromBlob(Blob *blob) {
    SpMobileObject newMobile(new MobileObject());
    int id = getMobileId(blob);
    //    std::cout << "Mobile ID: " << id << std::endl;
    newMobile->setNewMobileFromBlob(blob, id, rmobile_id_counter);
    insertInMobileAlternativesMap(newMobile, id);

    return newMobile;
}

bool ReliabilityTracker::notUsedId(long int id) {
    return usedIdsByAlternative.find(id) != usedIdsByAlternative.end() ? false : true;
}

int ReliabilityTracker::getMobileId(Blob *blob) {
    std::deque<IdBlobPair>::iterator p_iter = biggestBlobForNewMobile.begin();
    int i, size = biggestBlobForNewMobile.size();
    Blob *current_blob;
    long int current_id;
    for(i=0; i<size; i++, p_iter++) {
        current_blob = p_iter->blob;
        current_id = p_iter->id;
        //If new mobile type has different type compare with current mobile, it cannot have this id
        if(BLOB_TYPE(blob) != BLOB_TYPE(current_blob))
            continue;

      //If new blob enters inside maximal blob for current mobile, the id will be the same
        if(Blob::isBlob1InsideBlob2(blob, current_blob) && notUsedId(current_id)) {
            usedIdsByAlternative.insert(current_id);
            return current_id;
        }
        //If maximal blob for current mobile enters inside new blob, maximal blob must be updated
        if(Blob::isBlob1InsideBlob2(current_blob, blob) && notUsedId(current_id)) {
            p_iter->blob = blob;
            usedIdsByAlternative.insert(current_id);
            return current_id;
        }
    }

    //If loop have not returned any id, a new id must be assigned
    mobile_id_counter++;

    IdBlobPair p(mobile_id_counter, blob);
    biggestBlobForNewMobile.push_back(p);
    usedIdsByAlternative.insert(mobile_id_counter);

    return mobile_id_counter;
}

int ReliabilityTracker::getMiddle(int *array, int top, int bottom) {
    int x = array[top], i = top - 1, j = bottom + 1, temp;

    do {
        do {
            j --;
        }while (x > array[j]);

        do {
            i++;
        } while (x < array[i]);

        if (i < j) {
            temp = array[i];    // switch elements at positions i and j
            array[i] = array[j];
            array[j] = temp;
        }
    } while(i < j);

    return j;           // returns middle index
}

//Quicksort
void ReliabilityTracker::orderAscending(int *array, int top, int bottom) {
    int middle;
    if(top < bottom) {
        middle = getMiddle(array, top, bottom);
        orderAscending(array, top, middle);   // sort top partition
        orderAscending(array, middle+1, bottom);    // sort bottom partition
    }
}

void ReliabilityTracker::recursiveComboGenerator(int position, int value, int *alternativesCombo, int *blobsToMerge, int blobsToMergeNumber) {
    int i;

    if(position == blobsToMergeNumber) {
        processMergeVector(alternativesCombo, blobsToMerge, blobsToMergeNumber);
    } else
        for(i=0; i <= value; i++) {
            alternativesCombo[position] = i;
            recursiveComboGenerator(position+1, (value > i+1) ? value : i+1, alternativesCombo, blobsToMerge, blobsToMergeNumber);
        }
}


void ReliabilityTracker::generateAlternativesForMobile(int *blobsToMerge, int blobsToMergeNumber) {
    int alternativesCombo[blobsToMergeNumber];
    alternativesCombo[0] = 0;
    recursiveComboGenerator(1, 1, alternativesCombo, blobsToMerge, blobsToMergeNumber);
}

void ReliabilityTracker::generateNewMobiles(int blobNumberToAnalyze, int *blobsToAnalyze, int groupsNumber, int *blobGroupVector) {
    int i, j;
    int blobsToMerge[blobNumberToAnalyze], blobsToMergeNumber;
    bool globalInsertion = false;
    if(g_baseAlternative == NULL)
        globalInsertion = true;

    for(i=0; i<groupsNumber; i++) {
        //Set the blobs to merge per group
        blobsToMergeNumber = 0;
        for(j=0; j<blobNumberToAnalyze; j++)
            if(blobGroupVector[j] == i)
                blobsToMerge[blobsToMergeNumber++] = blobsToAnalyze[j];

        SpReliableMobileObject newRMobile;

        if(globalInsertion) {
            SpReliableMobileObject auxRMobile(new ReliableMobileObject());
            newRMobile = auxRMobile;
            g_newAlternatives.clear();
            rmobile_id_counter++;
        }

        generateAlternativesForMobile(blobsToMerge, blobsToMergeNumber);

        if(globalInsertion) {
            newRMobile->alternativeSolutions = g_newAlternatives;
            newRMobile->setBestSolution();
            rMobilesList.insert(newRMobile);
        }
    }
}

void ReliabilityTracker::insertNewMobiles(SpRMAlternativeSolution asolution, SpReliableMobileObject rmobile) {

    int i, j, blobNumberToAnalyze = 0, groupsNumber;
    int blobsToAnalyze[blobsNumber];
    int blobGroupVector[blobsNumber];

    if(asolution != NULL) {
        g_baseAlternative = asolution;
        bool ensureUsedBlobs[blobsNumber];
        int size = 0;
        std::deque<SpMobileObject>::iterator mobile_it;

        memset(ensureUsedBlobs, false, blobsNumber*sizeof(bool));
        mobile_it = g_baseAlternative->begin();
        size = g_baseAlternative->size();

        for(i=0; i< size; i++, mobile_it++) {
            if((*mobile_it)->ensureMode) {
                for(j=0; j<blobsNumber; j++)
                    ensureUsedBlobs[j] |= (*mobile_it)->usedBlobs[j];
            }
        }

        for(i=0; i<blobsNumber; i++)
            if(rmobile->usedBlobs[i] && !asolution->usedBlobs[i] && !ensureUsedBlobs[i]) //Consider blobs used in some of the rmobile alternatives, that are not used in the analyzed alternative
                blobsToAnalyze[blobNumberToAnalyze++] = i;
    } else { //Globally not used blobs
        g_baseAlternative = QSharedPointer<RMAlternativeSolution>();
        for(i=0; i<blobsNumber; i++)
            if(!usedBlobs[i])
                blobsToAnalyze[blobNumberToAnalyze++] = i;
    }

    g_intersectionAreas = new int[blobNumberToAnalyze*blobNumberToAnalyze];

    for(i=0; i<blobNumberToAnalyze-1; i++)
        for(j=i+1; j<blobNumberToAnalyze; j++)
            g_intersectionAreas[i*blobNumberToAnalyze + j] = Rectangle<int>::overlappingArea(&(blobsVector[blobsToAnalyze[i]]->bbox), &(blobsVector[blobsToAnalyze[j]]->bbox));

    /*    std::cout << "Overlaping Areas for " << blobNumberToAnalyze << " blobs:" << std::endl;
    for(i=0; i<blobNumberToAnalyze-1; i++) {
        for(j=i+1; j<blobNumberToAnalyze; j++)
            std::cout << " " << g_intersectionAreas[i*blobNumberToAnalyze + j];
        std::cout << std::endl;
    }
    std::cout << std::endl;*/

    if(blobNumberToAnalyze > 0) {
        groupsNumber = setGroups(blobGroupVector, initialMergeMap, blobNumberToAnalyze, blobsToAnalyze);
        generateNewMobiles(blobNumberToAnalyze, blobsToAnalyze, groupsNumber, blobGroupVector);
    }

    delete[] g_intersectionAreas;
}

void ReliabilityTracker::setCurrentTimeAndFrame() {
    int i;
    TimeStamp *ts = &m_data->currentHeader->ts;

    //For first frame
    if(lastTimeStamp.millisecond < 0)
      currentTimeMilliSeconds = 0;
    else {
      int sec_diff = ts->second - lastTimeStamp.second;
      //Supposing that maximal frame rate will produce a change between frames in second level, we will just analyze at this level
      if(sec_diff < 0) //A minute change
        sec_diff = 60 + ts->second - lastTimeStamp.second;

      lastMilliSecondsDifference = sec_diff*1000 +  ts->millisecond - lastTimeStamp.millisecond;

      currentTimeMilliSeconds += lastMilliSecondsDifference;
    }

    //Update sequence of second times
    memmove(MobileObject::secDiffSequence  + 1, MobileObject::secDiffSequence,  sizeof(double)*(m_BlobBufferSize-1));
    memmove(MobileObject::secDiffToCurrent + 1, MobileObject::secDiffToCurrent, sizeof(double)*(m_BlobBufferSize-1));

    //In case of a frame jump, consider the last seconds difference (IntervalDiskInput Acquisition Method)
    if(lastTimeStamp.millisecond >= 0 && ts->frame_id - currentFrameNumber > 1) {
        if(MobileObject::secDiffSequence[1] == 0.0)
            MobileObject::secDiffToCurrent[1] = MobileObject::secDiffSequence[0]  = m_meanMillisecondsDifferenceBetweenFrames;
        else
            MobileObject::secDiffToCurrent[1] = MobileObject::secDiffSequence[0]  = MobileObject::secDiffSequence[1];
        lastMilliSecondsDifference = (int)(1000*MobileObject::secDiffSequence[0]);
    } else
        MobileObject::secDiffToCurrent[1] = MobileObject::secDiffSequence[0] = lastMilliSecondsDifference/1000.0;

    for(i=2; i<m_BlobBufferSize; i++)
        MobileObject::secDiffToCurrent[i] += MobileObject::secDiffSequence[0];
    for(i=1; i<m_BlobBufferSize; i++)
        MobileObject::coolingValue[i] = MobileObject::coolingFunction(MobileObject::secDiffToCurrent[i]);

    MobileObject::m_objectDistanceForMaxReliability = getObjectDistanceForMaxReliability();

    currentFrameNumber = ts->frame_id;

    memcpy(&lastTimeStamp, ts, sizeof(TimeStamp));

    if(m_internalOutputActivated) {
        std::cout << ts->frame_id << ": " << ts->day<< "/" << ts->month<< "/" << ts->year << "\t" << ts->hour << ":"  << ts->minute << ":"  << ts->second << ","  << ts->millisecond << std::endl;
        std::cout << "msec:\t" << currentTimeMilliSeconds << std::endl;
        std::cout << "msec diff:\t" << lastMilliSecondsDifference << std::endl;
    }
}



void ReliabilityTracker::update(){

    if(blobsNumber == 0 && rMobilesList.empty())
        return;

    std::cout << "Update 1: numBlobs:" << blobsNumber << std::endl;

    //Complete the information on existing mobiles for time t
    if(!rMobilesList.empty()) {
        followExistingMobiles();

        if(blobsNumber > 0) {
            //Set new mobiles for remaining blobs for each ReliableMobile
            std::deque<SpReliableMobileObject>::iterator rmobile_it;
            std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralternative_it, ralt_it;
            std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> *asolutions;
            int j;
            std::cout << "rMobiles size: " << rMobilesList.size() << std::endl;
            for(j=0, rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); j++, rmobile_it++);
            std::cout << "rMobiles real: " << j << std::endl;

            //Complete with new mobiles the incomplete solutions
            for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {

                std::cout << "Beginning - Mobile size: " << (*rmobile_it)->size() << std::endl;
                for(j=0, ralt_it=(*rmobile_it)->begin(); ralt_it!=(*rmobile_it)->end(); j++, ralt_it++);
                std::cout << "Beginning - Mobile size real: " << j << std::endl;


                //If incomplete alternatives exist
                if((*rmobile_it)->incompleteAlternatives > 0) {
                    g_newAlternatives.clear();
                    g_completeAlternatives.clear();
                    biggestBlobForNewMobile.clear();

                    asolutions = (*rmobile_it)->getAlternativeSolutions();
                    for(ralternative_it = asolutions->begin(); ralternative_it != asolutions->end(); ralternative_it++) {
                        if((*ralternative_it)->incompleteAlternative) {
                            g_inserted_for_alternative = 0;
                            insertNewMobiles((*ralternative_it), (*rmobile_it));
                            if(g_inserted_for_alternative == 0) //If no alternative added, the base alternative will be lost, so add it
                                g_completeAlternatives.insert(*ralternative_it);
                            (*rmobile_it)->incompleteAlternatives--;
                        } else
                            g_completeAlternatives.insert(*ralternative_it);
                    }

                    //	    std::cout << "Ids List:" << std::endl;
                    std::deque<IdBlobPair>::iterator p_iter = biggestBlobForNewMobile.begin();

                    //In case of solution completion, regenerate alternatives list for rmobile with new alternatives
                    //if there are.
                    if(g_newAlternatives.size() > 0 || g_completeAlternatives.size() > 0) {
                        (*rmobile_it)->clear();
                        int j;
                        std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralt_it;

                        std::cout << "Inside - Mobile size: " << (*rmobile_it)->size() << std::endl;
                        std::cout << "Inside - complete size: " << g_completeAlternatives.size() << std::endl;
                        for(j=0, ralt_it=g_completeAlternatives.begin(); ralt_it!=g_completeAlternatives.end(); j++, ralt_it++);
                        std::cout << "Inside - complete size real: " << j << std::endl;

                        std::cout << "Inside - new size: " << g_newAlternatives.size() << std::endl;
                        for(j=0, ralt_it=g_newAlternatives.begin(); ralt_it!=g_newAlternatives.end(); j++, ralt_it++);
                        std::cout << "Inside - new size real: " << j << std::endl;

                        if(g_completeAlternatives.size() > 0)
                            (*rmobile_it)->alternativeSolutions = g_completeAlternatives;
                        std::cout << "Inside - Mobile size first: " << (*rmobile_it)->size() << std::endl;
                        for(j=0, ralt_it=(*rmobile_it)->begin(); ralt_it!=(*rmobile_it)->end(); j++, ralt_it++);
                        std::cout << "Inside - Mobile size real: " << j << std::endl;

                        if(g_newAlternatives.size() > 0) {
                            if((*rmobile_it)->size() == 0)
                                (*rmobile_it)->alternativeSolutions = g_newAlternatives;
                            else
                                (*rmobile_it)->insert(g_newAlternatives.begin(), g_newAlternatives.end());
                        }
                        std::cout << "Inside - Mobile size second: " << (*rmobile_it)->size() << std::endl;
                        for(j=0, ralt_it=(*rmobile_it)->begin(); ralt_it!=(*rmobile_it)->end(); j++, ralt_it++);
                        std::cout << "Inside - Mobile size real: " << j << std::endl;
                        (*rmobile_it)->setBestSolution();
                    }
                }
            }
        }
    }

    if(blobsNumber > 0) {
        biggestBlobForNewMobile.clear();
        g_inserted_for_alternative = 0;
        g_newAlternatives.clear();
        insertNewMobiles(QSharedPointer<RMAlternativeSolution>(), QSharedPointer<ReliableMobileObject>());
    }
}

void ReliabilityTracker::mergeReliableMobiles(SpReliableMobileObject firstRMobile, SpReliableMobileObject secondRMobile) {
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> newList;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator altFromFirst = firstRMobile->begin(), altFromSecond = secondRMobile->begin();
    int i;
    bool *involved_new, *involved_first, *involved_second;

    for(; altFromFirst!=firstRMobile->end(); altFromFirst++) {
        for(; altFromSecond!=secondRMobile->end(); altFromSecond++) {
            SpRMAlternativeSolution newAlternative(new RMAlternativeSolution());
            newAlternative->insert(newAlternative->end(), (*altFromFirst)->begin(), (*altFromFirst)->end());
            newAlternative->insert(newAlternative->end(), (*altFromSecond)->begin(), (*altFromSecond)->end());
            newAlternative->setAlternativeProbability();
            newAlternative->initInvolvedBlobs();
            involved_new    = newAlternative  ->involvedBlobs;
            involved_first  = (*altFromFirst) ->involvedBlobs;
            involved_second = (*altFromSecond)->involvedBlobs;
            for(i=0;i<blobsNumber;i++)
                involved_new[i] = involved_first[i] | involved_second[i];

            newList.insert(newAlternative);
        }
    }
    firstRMobile->clear();
    firstRMobile->insert(newList.begin(),newList.end());
    firstRMobile->setBestSolution();

    newList.clear();
}


void ReliabilityTracker::mergeInvolvedRMobiles() {

    int i, num_rmobiles = rMobilesList.size();

    if (num_rmobiles < 2) //Not enough rmobiles for possible conflicts
        return;

    int first_conflict = -1;

    for(i=0; i<blobsNumber; i++)
        if(involvedRMobilesCounter[i] > 1) {
            first_conflict = i;
            break;
        }

    if(first_conflict < 0) //No conflicts found
        return;

    std::deque<SpReliableMobileObject>::iterator first_rmobile, second_rmobile;
    bool *involved_first, *involved_second;
    int current_conflict = first_conflict;
    bool remaining_conflicts = true;

    int j, k, numToErase = 0;

    while(remaining_conflicts) {

        for(first_rmobile = rMobilesList.begin(), i=0; i < num_rmobiles-1; first_rmobile++, i++) {
            if(!(*first_rmobile)->toErase && (*first_rmobile)->involvedBlobs[current_conflict]) {

                involved_first = (*first_rmobile)->involvedBlobs;

                for(second_rmobile = first_rmobile + 1, j = i + 1; j < num_rmobiles; second_rmobile++, j++) {

                    if(!(*second_rmobile)->toErase && (*second_rmobile)->involvedBlobs[current_conflict]) {

                        involved_second = (*second_rmobile)->involvedBlobs;
                        involvedRMobilesCounter[current_conflict]--;

                        //Merge two RMobiles and store in first one.
                        mergeReliableMobiles((*first_rmobile), (*second_rmobile));

                        //Correct the involved blobs for resulting rmobile (the first one)
                        for(k=current_conflict+1; k<blobsNumber; k++) {
                            if(involved_second[k]) {
                                if(involved_first[k])
                                    involvedRMobilesCounter[k]--;
                                else
                                    involved_first[k] = true;
                            }
                        }

                        (*second_rmobile)->toErase = true;
                        numToErase++;
                    }

                    if(involvedRMobilesCounter[current_conflict] < 2)
                        break;
                }
            }

            if(involvedRMobilesCounter[current_conflict] < 2)
                break;
        }

        remaining_conflicts = false;
        for(i=current_conflict+1; i<blobsNumber; i++)
            if(involvedRMobilesCounter[i]>1) {
                current_conflict = i;
                remaining_conflicts = true;
                break;
            }

    }

    if(numToErase > 0) {
        for(first_rmobile = rMobilesList.begin(); first_rmobile != rMobilesList.end(); ) {
            if((*first_rmobile)->toErase) {
                first_rmobile = rMobilesList.erase(first_rmobile);
                numToErase--;
            } else
                first_rmobile++;

            if(numToErase == 0)
                break;

        }
    }
}

void ReliabilityTracker::createMobilePossibilities() {
    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralternative_it;
    int alternativesToEliminateCount, rmobilesToEliminateCount = 0;

    m_RMerge->clearDefinedMerges();

    for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
        (*rmobile_it)->initUsedBlobsList();
        alternativesToEliminateCount = 0;

        for(ralternative_it = (*rmobile_it)->begin(); ralternative_it != (*rmobile_it)->end(); ralternative_it++) {
            generateBestPossiblePathsForMobiles(*ralternative_it);
            if((*ralternative_it)->newObjectsList.empty()) {
                (*ralternative_it)->toEliminate = true;
                alternativesToEliminateCount++;
            }
        }

        //Eliminate empty alternatives (with just lost weak mobiles)
        if(alternativesToEliminateCount > 0) {
            std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> newList;
            for(ralternative_it = (*rmobile_it)->begin(); ralternative_it != (*rmobile_it)->end(); ralternative_it++)
                if(!(*ralternative_it)->toEliminate)
                    newList.insert(*ralternative_it);

                (*rmobile_it)->clear();

                if(newList.empty()) {
                    (*rmobile_it)->toErase = true;
                    rmobilesToEliminateCount++;
                } else {
                    (*rmobile_it)->insert(newList.begin(), newList.end());
                    (*rmobile_it)->setBestSolution();
                }
        }
    }


    //Eliminate empty alternatives (with just lost weak mobiles)
    if(rmobilesToEliminateCount > 0) {
        int numEliminated = 0;

        for(rmobile_it = rMobilesList.begin(); numEliminated < rmobilesToEliminateCount && rmobile_it != rMobilesList.end(); ) {
            if((*rmobile_it)->toErase == true) {
                rmobile_it = rMobilesList.erase(rmobile_it);
                numEliminated++;
            } else
                rmobile_it++;

        }
    }
}

//Path generation for mobiles in an alternative, considering:
//- First searching the blob with the best coverage (visual support) of estimated mobile 2D bounding box
//  (estimated from the most reliable information)
//- Generate mobile alternatives progressively merging with near blobs, until a too overcovering merge.
//- If the best possibility has a too low coverage, mark the object as MM_PARTIALLY_DETECTED and use its
//  coverage rate to give a reliability measure for attributes coherence.
//- If the best possibility largelly exceeds the coverage needed for the mobile, mark the object as
//  MM_PART_OF_BIGGER and use the coverage rate the mobile in the blob to give a reliability measure
//  for attributes coherence.
void ReliabilityTracker::generateBestPossiblePathsForMobiles(SpRMAlternativeSolution alternative) {
    //    std::map< long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> >::iterator mobileAlternatives_iter;
    std::deque<SpMobileObject>::iterator mobiles_iter = alternative->begin();
    int i, j, size = alternative->size();
    SpMobileObject currentMobile;
    //    std::cout << std::endl << "For Alternative: " << alternative->getProbability() << std::endl;

    alternative->newObjectsList.clear();
    alternative->numCurrentlyLost = 0;
    for(i=0; i<size; i++, mobiles_iter++) {

        currentMobile = *mobiles_iter;
        g_newMobiles.clear();
        g_newSpecialMobiles.clear();

        if(currentMobile->currentBufferSize <= 2) { //For first and second frames information, the only information
                                                    //is about initial dimensions of the 2D bounding box and very
                                                    //totally unreliable velocity information,
                                                    //then generation of mobile alternatives cannot be too
                                                    //constrained.

            //In this list the included blobs are stored, to verify if the same blob is included twice
            g_includedBlobsInNewMobiles.clear();
            g_bestGlobalP = 0.0;

            //Generate involved blobs merges to generate the mobile paths
            generateMobilePath(currentMobile);

            //Set lost mobile if no solution has been found
            if(g_newMobiles.empty()) {
                g_newMobiles.insert(setNotVisibleBlobMobile(MM_OBJECT_LOST, *mobiles_iter));
                alternative->numCurrentlyLost++;
            }

            //Filter incoherent alternatives for mobile, with respect to the best one
            if(g_newMobiles.size() > 1) {
                std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newMobiles.begin();
                //Start from second, because the first is the best
                for(j=2, newMobiles_iter++; newMobiles_iter != g_newMobiles.end(); newMobiles_iter++, j++)
                    //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                    //remaining mobiles.
                    if(j > maximumAlternativeMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileAlternativeProbability) {
                        g_newMobiles.erase(newMobiles_iter, g_newMobiles.end());
                        break;
                    }
            }
        } else { //With more information, we generate the initial bounding box guess for the mobile
               //From 3 frames we already have for sure information about 2D dimensions and velocity
               //allowing to estimate where the object should be in the next frame
               //and which size it would have.

            //Test the visual support of involved blobs
            if (currentMobile->numInvolved == 0) {//The object is LOST for sure
                Blob *specialBlob = new Blob();
                Rectangle<int> mobileBBox;
                currentMobile->getCurrentBoundingBoxForMobile(&mobileBBox);
                memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
                BLOB_POSITION(specialBlob) = specialBlob->getPositionRelativeToCamera(smodel);
                g_newMobiles.insert(setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob));
                delete specialBlob;
            } else if(currentMobile->numInvolved == 1) { //Nothing to compare, just one possibility for this mobile
                Blob *blobToTest;

                for(j=0; j<blobsNumber; j++)
                    if(currentMobile->involvedBlobs[j]) {
                        blobToTest = blobsVector[j];
                        break;
                    }
                g_bestGlobalP = 0.0;

                //Generate new mobile
                SpMobileObject newMobile = generateAndValidateNewMobile(currentMobile, blobToTest);
                if(newMobile != NULL) {
                    g_newMobiles.insert(newMobile);
                    //Check other cases, if 3D information is trustable enough
                    if(currentMobile->ensureMode || currentMobile->numberOfFramesSinceFirstTimeSeen > 2*MobileObject::m_blobsBufferSize) {
                        SpMobileObject specialMobile = checkSpecialCases(currentMobile, blobToTest);
                        if(specialMobile != NULL)
                            g_newMobiles.insert(specialMobile);
                    }
                } else {
                    Blob *specialBlob = new Blob();
                    Rectangle<int> mobileBBox;
                    currentMobile->getCurrentBoundingBoxForMobile(&mobileBBox);
                    memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
                    BLOB_POSITION(specialBlob) = specialBlob->getPositionRelativeToCamera(smodel);
                    g_newSpecialMobiles.insert( (newMobile = setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob)) );
                    delete specialBlob;
                }

                currentMobile->accepted_solution = true;

                //Set used blob for mobile
                if( !(newMobile->currentVisualState & MM_OBJECT_LOST) ) {
                    newMobile->usedBlobs[j] = true;
                    newMobile->numUsed = 1;
                }
            } else { //More than one possible combination, then compare the possibilities

                double blobSupport[blobsNumber], mobileSupport[blobsNumber];
                bool usedBlobs[blobsNumber];
                int numUsed = 0, bestIndex;
                Rectangle<int> mobileBBox;
                Blob *initialBlob = NULL, *mergedBlob = NULL;
                double totalSupport = 0.0, bestSupport = -1.0;

                memset(usedBlobs, false, blobsNumber*sizeof(bool));
                memset(blobSupport,   0, blobsNumber*sizeof(double));
                memset(mobileSupport, 0, blobsNumber*sizeof(double));

                currentMobile->getCurrentBoundingBoxForMobile(&mobileBBox);

                //Get the initial blob merge with the blobs totally supported by the mobile, which are connected
                //with the most supported blob.
                for(j=0; j<blobsNumber; j++) {
                    if(currentMobile->involvedBlobs[j]) {
                        //Visual support is 1.0 when mobile bbox is completelly covered by the analysed blob
                        blobSupport[j] = Rectangle<int>::rectangleIntersectRatio(&mobileBBox, &(blobsVector[j]->bbox));
                        mobileSupport[j] = Rectangle<int>::rectangleIntersectRatio(&(blobsVector[j]->bbox), &mobileBBox);
                        if(mobileSupport[j] >= m_blobCompletellySupportedThreshold && blobSupport[j] > bestSupport) {
                            bestSupport = blobSupport[j];
                            initialBlob = blobsVector[j];
                            bestIndex = j;
                        }
                    }
                }
                //If an initial blob evidence have been found, complete with connected blobs inside the mobile estimated bbox
                if(initialBlob != NULL) {
                    int mergedBlobsList[blobsNumber];
                    bool someIncluded;
                    mergedBlobsList[0] = bestIndex;
                    numUsed = 1;
                    usedBlobs[bestIndex] = true;
                    do {
                        someIncluded = false;
                        for(j=0; j<blobsNumber; j++) {
                            if(currentMobile->involvedBlobs[j] && !usedBlobs[j]) {
                                if( blobCanBeIncludedForMerge(numUsed, mergedBlobsList, j) && blobSupport[j] > 0.0 && mobileSupport[j] >= m_blobCompletellySupportedThreshold ) {
                                    someIncluded = true;
                                    if(!Blob::isBlob1InsideBlob2(blobsVector[j], initialBlob)) { //If analyzed blob is not already contained in initial blob merge, merge it.
                                        mergedBlob = Blob::mergeBlob(initialBlob, blobsVector[j]);
                                        if(numUsed > 1) //If blob does not correspond to one on the blobsVector list, free it
                                            delete initialBlob;
                                        initialBlob = mergedBlob;
                                        totalSupport = Rectangle<int>::rectangleIntersectRatio(&mobileBBox, &(initialBlob->bbox));
                                    }
                                    mergedBlobsList[numUsed] = j;
                                    usedBlobs[j] = true;
                                    numUsed++;
                                }
                            }
                        }
                    } while (someIncluded);
                }

                //If initialBlob is still NULL, it could mean that mobile bounding box is part of a bigger
                //blob (maybe a group of mobiles), so we search for the blob with the highest mobile bbox support.
                if(initialBlob == NULL) {
                    int usedIndex;
                    for(j=0; j<blobsNumber; j++) {
                        if(currentMobile->involvedBlobs[j] && blobSupport[j] > totalSupport) {
                            initialBlob = blobsVector[j];
                            usedIndex = j;
                            totalSupport = blobSupport[j];
                        }
                    }

                    if(initialBlob != NULL) {
                        usedBlobs[usedIndex] = true;
                        numUsed = 1;
                    }
                }

                //If initialBlob is STILL NULL, the mobile is considered LOST
                if(initialBlob == NULL) {
                    Blob *specialBlob = new Blob();
                    Rectangle<int> mobileBBox;
                    currentMobile->getCurrentBoundingBoxForMobile(&mobileBBox);
                    memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
                    BLOB_POSITION(specialBlob) = specialBlob->getPositionRelativeToCamera(smodel);
                    g_newSpecialMobiles.insert(setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob));
                    delete specialBlob;
                } else { //There is an initialSolution to add and to be used as a starting point for searching new ones
                    //Add the initial solution
                    g_bestGlobalP = 0.0;
                    //Generate new mobile
                    SpMobileObject
                        specialMobile = QSharedPointer<MobileObject>(),
                        newMobile = generateAndValidateNewMobile(currentMobile, initialBlob);

                    if(!newMobile.isNull())
                        g_newMobiles.insert(newMobile);

                    if(currentMobile->ensureMode || currentMobile->numberOfFramesSinceFirstTimeSeen > 2*MobileObject::m_blobsBufferSize) {
                        specialMobile = checkSpecialCases(currentMobile, initialBlob);
                        if(!specialMobile.isNull())
                            g_newSpecialMobiles.insert(specialMobile);
                    } else
                        specialMobile = QSharedPointer<MobileObject>();

                    currentMobile->accepted_solution = true;

                    //Set used blob for initial mobile
                    if( newMobile != NULL ) {
                        memcpy(newMobile->usedBlobs, usedBlobs, blobsNumber*sizeof(bool));
                        newMobile->numUsed = numUsed;
                    }

                    if( specialMobile != NULL ) {
                        memcpy(specialMobile->usedBlobs, usedBlobs, blobsNumber*sizeof(bool));
                        specialMobile->numUsed = numUsed;
                    }

                    if(totalSupport < 0.99999 && numUsed < currentMobile->numInvolved) { //if total mobile visual support using initialBlob is not 1.0
                        //(floating point 1.0), and if all involved have not yet been
                        //included in initial solution, is worthy to test other blobs
                        //to merge.

                        //Construct new solutions from initialBlob.
                        //Initialize global probability with
                        if(newMobile != NULL)
                            g_bestGlobalP = newMobile->getGlobalProbability();
                        else
                            g_bestGlobalP = 0;

                        //In this list the included blobs are stored, to verify if the same blob is included twice
                        g_includedBlobsInNewMobiles.clear();
                        g_allocatedBlobs.clear();
                        g_includedBlobsInNewMobiles.push_front(initialBlob);
                        if(numUsed > 1)
                            g_allocatedBlobs.push_front(initialBlob);

                        //Generate involved blobs merges to generate the mobile paths
                        generateMobilePathFromInitialBlob(currentMobile, initialBlob, numUsed, usedBlobs, blobSupport, &mobileBBox);

                        //Filter incoherent alternatives for mobile, with respect to the best one
                        if(g_newMobiles.size() > 1) {
                            std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newMobiles.begin();
                            //Start from second, because the first is the best
                            for(j=2, newMobiles_iter++; newMobiles_iter != g_newMobiles.end(); newMobiles_iter++, j++)
                                //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                                //remaining mobiles.
                                if(j > maximumAlternativeMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileAlternativeProbability) {
                                    g_newMobiles.erase(newMobiles_iter, g_newMobiles.end());
                                    break;
                                }
                        } else if (g_newSpecialMobiles.size() > 1) { //If no new normal mobiles, try the special cases list
                            std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newSpecialMobiles.begin();
                            //Start from second, because the first is the best
                            for(j=2, newMobiles_iter++; newMobiles_iter != g_newSpecialMobiles.end(); newMobiles_iter++, j++)
                                //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                                //remaining mobiles.
                                if(j > maximumAlternativeMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileAlternativeProbability) {
                                    g_newSpecialMobiles.erase(newMobiles_iter, g_newSpecialMobiles.end());
                                    break;
                                }
                        }

                        //Free allocated blobs
                        std::deque<Blob *>::iterator allocated;
                        int alloc_size = g_allocatedBlobs.size();
                        for(j=0, allocated = g_allocatedBlobs.begin(); j < alloc_size; j++, allocated++)
                            delete *allocated;

                    }
                }
                if(mergedBlob != NULL)
                    delete mergedBlob;
            }
        }

        if(!g_newMobiles.empty())
            alternative->newObjectsList[currentMobile->getMobileId()] = g_newMobiles;
        else if(!g_newSpecialMobiles.empty())
            alternative->newObjectsList[currentMobile->getMobileId()] = g_newSpecialMobiles;

    }

}

bool ReliabilityTracker::acceptableInformationForNewMobile(SpMobileObject newMobile, SpMobileObject oldMobile) {

    double dimension, variation, dt = MobileObject::secDiffSequence[0];
    //    Blob *currentOldBlob = (*(oldMobile->rbegin())->getBlob();
    //Blob *currentNewBlob = (*(newMobile->blobHistory.rbegin()))->getBlob();

    //Test for 2D is relaxed because 2D is not an invariant on the scene, and it is done with
    //extracted 2D blobs, because the estimation will always try to fit with this data
    //2D Position Coordinate X Test.
    dimension = oldMobile->t2DSpatialData.X + dt*oldMobile->t2DSpatialData.VX;
    variation = 2*(oldMobile->t2DSpatialData.SDX + dt*oldMobile->t2DSpatialData.SDVX) + acceptedPixelError;
    //    if(newMobile->t2DSpatialData.X > dimension + variation || newMobile->t2DSpatialData.X < dimension - variation)
    if(newMobile->t2DSpatialData.X > dimension + variation || newMobile->t2DSpatialData.X < dimension - variation)
        return false;
    //2D Position Coordinate Y Test.
    dimension = oldMobile->t2DSpatialData.Y + dt*oldMobile->t2DSpatialData.VY;
    variation = 2*(oldMobile->t2DSpatialData.SDY + dt*oldMobile->t2DSpatialData.SDVY) + acceptedPixelError;
    //    if(newMobile->t2DSpatialData.Y > dimension + variation || newMobile->t2DSpatialData.Y < dimension - variation)
    if(newMobile->t2DSpatialData.Y > dimension + variation || newMobile->t2DSpatialData.Y < dimension - variation)
        return false;
    //2D Dimension W Test.
    dimension = oldMobile->t2DDimData.W + dt*oldMobile->t2DDimData.VW;
    variation = 2*(oldMobile->t2DDimData.SDW + dt*oldMobile->t2DDimData.SDVW) + acceptedPixelError;
    //    if(newMobile->t2DDimData.W > dimension + variation || newMobile->t2DDimData.W < dimension - variation)
    if(newMobile->t2DDimData.W > dimension + variation || newMobile->t2DDimData.W < dimension - variation)
        return false;
    //2D Dimension H Test.
    dimension = oldMobile->t2DDimData.H + dt*oldMobile->t2DDimData.VH;
    variation = 2*(oldMobile->t2DDimData.SDH + dt*oldMobile->t2DDimData.SDVH) + acceptedPixelError;
    //if(newMobile->t2DDimData.H > dimension + variation || newMobile->t2DDimData.H < dimension - variation)
    if(newMobile->t2DDimData.H > dimension + variation || newMobile->t2DDimData.H < dimension - variation)
        return false;

    /*
    //If newMobile and oldMobile has trustable 3D information
    if(oldMobile->ensureMode && newMobile->ensureMode) {
        //3D Position Coordinate x Test.
        dimension = oldMobile->t3DSpatialData.x + dt*oldMobile->t3DSpatialData.Vx;
        variation = oldMobile->t3DSpatialData.SDx + dt*oldMobile->t3DSpatialData.SDVx + accepted3DFeatureError;
        if(newMobile->t3DSpatialData.x > dimension + variation || newMobile->t3DSpatialData.x < dimension - variation)
            return false;
        //3D Position Coordinate y Test.
        dimension = oldMobile->t3DSpatialData.y + dt*oldMobile->t3DSpatialData.Vy;
        variation = oldMobile->t3DSpatialData.SDy + dt*oldMobile->t3DSpatialData.SDVy + accepted3DFeatureError;
        if(newMobile->t3DSpatialData.y > dimension + variation || newMobile->t3DSpatialData.y < dimension - variation)
            return false;

        //3D Dimension w Test.
        dimension = oldMobile->t3DDimData.w + dt*oldMobile->t3DDimData.Vw;
        variation = oldMobile->t3DDimData.SDw + dt*oldMobile->t3DDimData.SDVw + accepted3DFeatureError;
        if(newMobile->t3DDimData.w > dimension + variation || newMobile->t3DDimData.w < dimension - variation)
            return false;
        //3D Dimension l Test.
        dimension = oldMobile->t3DDimData.l + dt*oldMobile->t3DDimData.Vl;
        variation = oldMobile->t3DDimData.SDl + dt*oldMobile->t3DDimData.SDVl + accepted3DFeatureError;
        if(newMobile->t3DDimData.l > dimension + variation || newMobile->t3DDimData.l < dimension - variation)
            return false;
        //3D Dimension h Test.
        dimension = oldMobile->t3DDimData.h + dt*oldMobile->t3DDimData.Vh;
        variation = oldMobile->t3DDimData.SDh + dt*oldMobile->t3DDimData.SDVh + accepted3DFeatureError;
        if(newMobile->t3DDimData.h > dimension + variation || newMobile->t3DDimData.h < dimension - variation)
            return false;
        //3D base orientation alpha Test.
        dimension = oldMobile->t3DDimData.alpha + dt*oldMobile->t3DDimData.Valpha;
        variation = oldMobile->t3DDimData.SDalpha + dt*oldMobile->t3DDimData.SDValpha + acceptedOrientationError;
        if(variation < M_PI / 2.0) { //so much variation invalidates the test as the length of the valid interval es between 0 and PI
            if(dimension + variation >= M_PI) {
                if(newMobile->t3DDimData.alpha > MobileObject::NormalizeOrientation(dimension + variation) && newMobile->t3DDimData.alpha < dimension - variation)
                    return false;
            } else if(dimension - variation < 0.0) {
                if(newMobile->t3DDimData.alpha > dimension + variation && newMobile->t3DDimData.alpha < MobileObject::NormalizeOrientation(dimension - variation))
                    return false;
            } else {
                if(newMobile->t3DDimData.alpha > dimension + variation || newMobile->t3DDimData.alpha < dimension - variation)
                    return false;
            }
        }
    }
    */
    return true;
}

bool ReliabilityTracker::mobilesCombinationIsValid() {
    int i, j;
    SpMobileObject m1, m2;
    for(i = 0; i < g_numberOfNewMobiles - 1; i++) {
        m1 = *g_mobileIterators[i];
        for(j = i + 1; j < g_numberOfNewMobiles; j++) {
            //If both objects are unknown, they cannot be validated by 3D model base.
            if( !g_acceptable3DCoherenceForMobile[i] && !g_acceptable3DCoherenceForMobile[j] )
                continue;

            m2 = *g_mobileIterators[j];

            if( g_acceptable3DCoherenceForMobile[i] && g_acceptable3DCoherenceForMobile[j] ) {
                if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j]))
                    continue;
                return false;
            } else if( g_acceptable3DCoherenceForMobile[i] ) {
                //The first one has 3D information (see the boolean flag of function)
                if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j], true))
                    continue;
                return false;
            } else {
                //The second one has 3D information (see the boolean flag of function)
                if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j], false))
                    continue;
                return false;
            }
        }
    }

    return true;
}

int ReliabilityTracker::getPairIndex(int mindex, int vindex) {
    int i, index = 0;
    for(i=0; i<mindex; i++)
        index += g_numberOfNewMobileVersions[i];
    index += vindex;

    return index;
}

bool ReliabilityTracker::mobilePairValidityChecked(int index1, int index2) {
    return checkedMobilePairValidity[g_totalNumberOfMobileVersions*index1 + index2];
}

bool ReliabilityTracker::mobilePairIsValid(int index1, int index2) {
    return validMobilePair[g_totalNumberOfMobileVersions*index1 + index2];
}

bool ReliabilityTracker::checkMobilePairValidity(SpMobileObject m1, int mindex1, int vindex1, SpMobileObject m2, int mindex2, int vindex2) {

    int index1 = getPairIndex(mindex1, vindex1), index2 = getPairIndex(mindex2, vindex2);

    if( mobilePairValidityChecked(index1, index2) )
        return mobilePairIsValid(index1, index2);

    double area = areaOfIntersection(m1, m2);

    if(area == 0.0)
        return setPairValidityAndGo(index1, index2, true);

    if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
        || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
        return setPairValidityAndGo(index1, index2, false);

    return setPairValidityAndGo(index1, index2, true);
}

bool ReliabilityTracker::checkMobilePairValidity(SpMobileObject m1, int mindex1, int vindex1, SpMobileObject m2, int mindex2, int vindex2, bool firstVerifiable) {

    int index1 = getPairIndex(mindex1, vindex1), index2 = getPairIndex(mindex2, vindex2);

    if( mobilePairValidityChecked(index1, index2) )
        return mobilePairIsValid(index1, index2);

    double area, nvarea;

    if(firstVerifiable)
        area = areaOfIntersectionFirstNo3D(m2, m1, &nvarea);
    else
        area = areaOfIntersectionFirstNo3D(m1, m2, &nvarea);

    if(area == 0.0)
        return setPairValidityAndGo(index1, index2, true);

    if(firstVerifiable) {
        if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
            || area / nvarea > m_maximal3DBaseOverlapping )
            return setPairValidityAndGo(index1, index2, false);
    } else {
        if(    area / nvarea > m_maximal3DBaseOverlapping
            || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
            return setPairValidityAndGo(index1, index2, false);
    }

    return setPairValidityAndGo(index1, index2, true);
}

bool ReliabilityTracker::checkMobilePairValidity(SpMobileObject m1, SpMobileObject m2) {

    double area = areaOfIntersection(m1, m2);

    if(area == 0.0)
        return true;

    if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
        || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
        return false;

    return true;
}

bool ReliabilityTracker::checkMobilePairValidity(SpMobileObject m1, SpMobileObject m2, bool firstVerifiable) {

    double area, nvarea;

    if(firstVerifiable)
        area = areaOfIntersectionFirstNo3D(m2, m1, &nvarea);
    else
        area = areaOfIntersectionFirstNo3D(m1, m2, &nvarea);

    if(area == 0.0)
        return true;

    if(firstVerifiable) {
        if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
            || area / nvarea > m_maximal3DBaseOverlapping )
            return false;
    } else {
        if(    area / nvarea > m_maximal3DBaseOverlapping
            || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
            return false;
    }

    return true;
}


bool ReliabilityTracker::setPairValidityAndGo(int index1, int index2, bool ret_value) {

    checkedMobilePairValidity[g_totalNumberOfMobileVersions*index1 + index2] = true;
    validMobilePair[g_totalNumberOfMobileVersions*index1 + index2] = ret_value;

    return ret_value;
}

double ReliabilityTracker::areaOfIntersection(SpMobileObject m1, SpMobileObject m2) {
    int i, n1 = 0, n2 = 0, numUsed1 = m1->numUsed, numUsed2 = m2->numUsed;
    bool possible_conflict = false, *used1 = m1->usedBlobs, *used2 = m2->usedBlobs;

    for(i=0; i<blobsNumber; i++) {
        if(used1[i] && used2[i]) { //There might be a conflict
            possible_conflict = true;
            break;
        } else if(used1[i])
            n1++;
        else if(used2[i])
            n2++;
        if(n1 == numUsed1)
            break;
        if(n2 == numUsed2)
            break;
    }

    if(!possible_conflict)
        return 0.0;

    polygon2D<double> *p1 = new polygon2D<double>(4), *p2 = new polygon2D<double>(4), *r;
    double alpha, sina, cosa;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    int beta_direction = getBetaDirection(smodel, m_pSegmentation);

    alpha = m1->t3DDimData.alpha;
    sina = sin(alpha);
    cosa = cos(alpha);
    bb3D.getFromInitial3DPoint(smodel, &bb2D,
                               m1->t3DSpatialData.x + beta_direction*(m1->t3DDimData.w*sina - m1->t3DDimData.l*cosa)/2.0,
                               m1->t3DSpatialData.y - (m1->t3DDimData.w*cosa + m1->t3DDimData.l*sina)/2.0,
                               1, alpha, beta_direction, m1->t3DDimData.w, m1->t3DDimData.l, m1->t3DDimData.h);

    for(i = 0; i < 4; i++) {
        p1->points[i].x = PARALL_X_i(&bb3D, i);
        p1->points[i].y = PARALL_Y_i(&bb3D, i);
    }
    p1->computeBoundingRectangle();

    alpha = m2->t3DDimData.alpha;
    sina = sin(alpha);
    cosa = cos(alpha);
    bb3D.getFromInitial3DPoint(smodel, &bb2D,
                               m2->t3DSpatialData.x + beta_direction*(m2->t3DDimData.w*sina - m2->t3DDimData.l*cosa)/2.0,
                               m2->t3DSpatialData.y - (m2->t3DDimData.w*cosa + m2->t3DDimData.l*sina)/2.0,
                               1, alpha, beta_direction, m2->t3DDimData.w, m2->t3DDimData.l, m2->t3DDimData.h);

    for(i = 0; i < 4; i++) {
        p2->points[i].x = PARALL_X_i(&bb3D, i);
        p2->points[i].y = PARALL_Y_i(&bb3D, i);
    }
    p2->computeBoundingRectangle();

    r = polygon2D<double>::intersectionPolygon(p1, p2);

    if(r == NULL) {
        delete p1;
        delete p2;
        return 0.0;
    }

    double area = r->polygonArea();

    delete r;
    delete p1;
    delete p2;

    return area;
}

double ReliabilityTracker::areaOfIntersectionFirstNo3D(SpMobileObject m1, SpMobileObject m2, double *areaOfNo3D) {
    int i, n1 = 0, n2 = 0, numUsed1 = m1->numUsed, numUsed2 = m2->numUsed;
    bool possible_conflict = false, *used1 = m1->usedBlobs, *used2 = m2->usedBlobs;
    int beta_direction = getBetaDirection(smodel, m_pSegmentation);
    for(i=0; i<blobsNumber; i++) {
        if(used1[i] && used2[i]) { //There might be a conflict
            possible_conflict = true;
            break;
        } else if(used1[i])
            n1++;
        else if(used2[i])
            n2++;
        if(n1 == numUsed1)
            break;
        if(n2 == numUsed2)
            break;
    }

    if(!possible_conflict)
        return 0.0;

    polygon2D<double> *p1 = new polygon2D<double>(4), *p2 = new polygon2D<double>(4), *r;
    double alpha, sina, cosa, leftmost, rightmost, upmost, downmost;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;

    double W_2 = m1->t2DDimData.W/2.0, H_2 = m1->t2DDimData.H/2.0, X = m1->t2DSpatialData.X, Y = m1->t2DSpatialData.Y;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X - W_2, Y + H_2, 0, &(p1->points[0].x), &(p1->points[0].y));
    leftmost = rightmost = p1->points[0].x;
    upmost = downmost = p1->points[0].y;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X + W_2, Y + H_2, 0, &(p1->points[1].x), &(p1->points[1].y));
    if (p1->points[1].x < leftmost)  leftmost  = p1->points[1].x;
    if (p1->points[1].x > rightmost) rightmost = p1->points[1].x;
    if (p1->points[1].y < upmost)    upmost    = p1->points[1].y;
    if (p1->points[1].y > downmost)  downmost  = p1->points[1].y;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X + W_2, Y - H_2, 0, &(p1->points[2].x), &(p1->points[2].y));
    if (p1->points[2].x < leftmost)  leftmost  = p1->points[2].x;
    if (p1->points[2].x > rightmost) rightmost = p1->points[2].x;
    if (p1->points[2].y < upmost)    upmost    = p1->points[2].y;
    if (p1->points[2].y > downmost)  downmost  = p1->points[2].y;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X - W_2, Y - H_2, 0, &(p1->points[3].x), &(p1->points[3].y));
    if (p1->points[3].x < leftmost)  leftmost  = p1->points[3].x;
    if (p1->points[3].x > rightmost) rightmost = p1->points[3].x;
    if (p1->points[3].y < upmost)    upmost    = p1->points[3].y;
    if (p1->points[3].y > downmost)  downmost  = p1->points[3].y;

    POLYGON_2D_BB_X(p1) = leftmost;
    POLYGON_2D_BB_Y(p1) = upmost;
    POLYGON_2D_BB_WIDTH(p1) = (rightmost - leftmost);
    POLYGON_2D_BB_HEIGHT(p1) = (downmost - upmost);
    POLYGON_2D_BB_XRIGHT(p1) = rightmost;
    POLYGON_2D_BB_YBOTTOM(p1) = downmost;
    POLYGON_2D_BB_DONE(p1) = true;

    *areaOfNo3D = p1->polygonArea();

    alpha = m2->t3DDimData.alpha;
    sina = sin(alpha);
    cosa = cos(alpha);
    bb3D.getFromInitial3DPoint(smodel, &bb2D,
                                m2->t3DSpatialData.x + beta_direction*(m2->t3DDimData.w*sina - m2->t3DDimData.l*cosa)/2.0,
                                m2->t3DSpatialData.y - (m2->t3DDimData.w*cosa + m2->t3DDimData.l*sina)/2.0,
                                1, alpha, beta_direction, m2->t3DDimData.w, m2->t3DDimData.l, m2->t3DDimData.h);

    for(i = 0; i < 4; i++) {
        p2->points[i].x = PARALL_X_i(&bb3D, i);
        p2->points[i].y = PARALL_Y_i(&bb3D, i);
    }
    p2->computeBoundingRectangle();

    r = polygon2D<double>::intersectionPolygon(p1, p2);

    if(r == NULL) {
        delete p1;
        delete p2;
        return 0.0;
    }

    delete p1;
    delete p2;

    double area = r->polygonArea();

    delete r;

    return area;
}

void ReliabilityTracker::generateNewLeavesFromCurrentBests() {
    std::set<SpBestAlternativesNode, orderedByBestAlternativeProbabilityCooperationOperator>::iterator leaves = g_leaves.begin();
    std::set<SpBestAlternativesNode, orderedByBestAlternativeProbabilityCooperationOperator> new_leaves;
    int i;
    for(; leaves != g_leaves.end() && (*leaves)->added == true; leaves++) {
        for(i=(*leaves)->mobileIndex; i<g_numberOfNewMobiles; i++) {
            if (bestAlternativesNode::variablesNumFrames[i]>0) {
                SpBestAlternativesNode newNode(new bestAlternativesNode((*leaves), i));
                if(newNode->value < 0)
                    continue;
                new_leaves.insert(newNode);
            }
        }
    }

    do { g_leaves.erase(g_leaves.begin()); } while(g_leaves.begin() != g_leaves.end() && (*g_leaves.begin())->added == true);

    g_leaves.insert(new_leaves.begin(), new_leaves.end());
    new_leaves.clear();
}


void ReliabilityTracker::buildNewAlternativesFromLeaves(SpRMAlternativeSolution currentAlternative) {

    int localMaximumNumber = g_numberOfNewMobiles * m_maximumGeneratedAlternativesPerMobile;

    std::set<SpBestAlternativesNode, orderedByBestAlternativeProbabilityCooperationOperator>::iterator leaves;
    double best;

    while(g_NumLocallyAddedAlternatives < localMaximumNumber && g_NumLocallyAddedAlternatives < m_maximumRetainedAlternativeSolutions && g_leaves.size() > 0) {

        generateNewLeavesFromCurrentBests();

        if(g_leaves.size() > 0) {
            leaves = g_leaves.begin();
            best = (*leaves)->value;
            do {
                SpRMAlternativeSolution newAlternative;
                if((newAlternative = getAlternativeFromNodeIfValid(*leaves)) != NULL) {
                    cleanByEnsureUsedBlobs(newAlternative);
                    currentAlternative->newAlternatives.insert(newAlternative);
                    g_NumLocallyAddedAlternatives++;
                }

                (*leaves)->added = true;
                leaves++;
                if(leaves == g_leaves.end())
                    break;
            } while( best == (*leaves)->value );
        }
    }
}

double ReliabilityTracker::getTentativeAlternativeProbabilityValue(std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator *mobileIterators) {

    int i, sum = 0, num;
    double total = 0.0;
    for(i=0; i<g_numberOfNewMobiles; i++) {
        g_variableContribution[i] = 0;
        bestAlternativesNode::variablesNumFrames[i] = 0;
        if(g_numberOfNewMobileVersions[i] > 0) {
            sum +=  bestAlternativesNode::variablesNumFrames[i] = num = (*mobileIterators[i])->getNumberOfFramesSinceFirstTimeSeen();
            total += g_variableContribution[i] = num * (*mobileIterators[i])->getGlobalProbability();
        }
    }

    for(i=0; i<g_numberOfNewMobiles; i++)
        g_variableContribution[i] /= (double) sum;

    bestAlternativesNode::variablesSum = sum;

    return total/(double) sum;
}

SpRMAlternativeSolution ReliabilityTracker::getAlternativeFromNodeIfValid(SpBestAlternativesNode node) {
    int i;
    bool acceptable3DCoherenceForMobile[g_numberOfNewMobiles];

    g_mobileIterators = node->mobileIterators;

    for(i=0; i<g_numberOfNewMobiles; i++) {
        if(bestAlternativesNode::variablesNumFrames[i] > 0)
            acceptable3DCoherenceForMobile[i] = (*(g_mobileIterators[i]))->mobile3DCoherenceIsAcceptable();
        else
            acceptable3DCoherenceForMobile[i] = true;
    }
    g_acceptable3DCoherenceForMobile = acceptable3DCoherenceForMobile;
    g_mobileVersionIndex = node->versionIndex;

    if(!mobilesCombinationIsValid())
        return QSharedPointer<RMAlternativeSolution>();

    SpMobileObject currentMobile;
    SpRMAlternativeSolution newAlternative(new RMAlternativeSolution());
    int j;

    newAlternative->initUsedBlobsList();

    for(i=0; i<g_numberOfNewMobiles; i++) {
        if(bestAlternativesNode::variablesNumFrames[i]>0) {
            currentMobile = *(g_mobileIterators[i]);
            newAlternative->insertNewMobileObject(currentMobile);
            for(j=0; j < blobsNumber; j++)
                newAlternative->usedBlobs[j] |= currentMobile->usedBlobs[j];
        }
    }

    newAlternative->setAlternativeProbability();

    newAlternative->numCurrentlyLost = g_NumCurrentlyLost;

    return newAlternative;
}

void ReliabilityTracker::cleanByEnsureUsedBlobs(SpRMAlternativeSolution newAlternative) {
    std::deque<SpMobileObject>::iterator mobile_it;
    bool ensureUsedBlobs[blobsNumber];
    int i, j, size, numEnsured = 0;

    memset(ensureUsedBlobs, false, blobsNumber*sizeof(bool));
    mobile_it = newAlternative->begin();
    size = newAlternative->size();

    //Get used blobs for ensured mobiles
    for(i=0; i< size; i++, mobile_it++) {
        if((*mobile_it)->ensureMode) {
            numEnsured++;
            for(j=0; j<blobsNumber; j++)
                ensureUsedBlobs[j] = (*mobile_it)->usedBlobs[j] ? true : false;
        }
    }

    //There is no discrimination in this case
    if(numEnsured == 0 || numEnsured == size)
        return;

    bool modified = false;

    for(mobile_it = newAlternative->begin(); mobile_it != newAlternative->end(); mobile_it++) {
        (*mobile_it)->toErase = false;
        if((*mobile_it)-> getBestType() == UNKNOWN)
            for(j=0; j<blobsNumber; j++)
                if(ensureUsedBlobs[j] && (*mobile_it)->usedBlobs[j]) {
                    (*mobile_it)->toErase = modified = true;
                    break;
                }
    }
    if(modified) {
        SpRMAlternativeSolution goodMobilesCopy(new RMAlternativeSolution());
        for(mobile_it = newAlternative->begin(); mobile_it != newAlternative->end(); mobile_it++)
            if((*mobile_it)->toErase == false)
                goodMobilesCopy->insertNewMobileObject(*mobile_it);

        newAlternative->clear();
        newAlternative->insert(newAlternative->end(),goodMobilesCopy->begin(),goodMobilesCopy->end());
        goodMobilesCopy->clear();

        newAlternative->setAlternativeProbability();
    }
}


void ReliabilityTracker::generateAlternativesWithBestSolutionsTree() {
    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator alternative_it, newalt_it;
    std::map<long int, std::set<SpMobileObject, orderedByBestCoherenceOperator> >::iterator newObj_it;
    int i, numberOfMobiles, totalNumMobileVersions;
    SpRMAlternativeSolution currentAlternative;

    //for output
    std::deque<SpMobileObject>::iterator mobile_it;

    //Generate Alternatives for each rmobile
    for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
        (*rmobile_it)->initUsedBlobsList();

        for(m_alternativeNumber = 0, alternative_it = (*rmobile_it)->begin(); alternative_it != (*rmobile_it)->end(); m_alternativeNumber++, alternative_it++) {

            currentAlternative = (*alternative_it);
            currentAlternative->newAlternatives.clear();
            g_NumLocallyAddedAlternatives = 0;

            //Second proposed method (smart method: check alternatives as a tree, immediatelly storing best alternatives and generating
            //leaves according to best found solutions in previous leaves).
            //Useful when the maximum number of alternative solutions retained is less than the total possible number of mobile combinations.

            numberOfMobiles = currentAlternative->newObjectsList.size();

            int numMobileVersions[numberOfMobiles], numFramesVariables[numberOfMobiles];
            std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator setBegins[numberOfMobiles], setEnds[numberOfMobiles], mobileIterators[numberOfMobiles];
            double variableContribution[numberOfMobiles];

            g_newObjectsList = &currentAlternative->newObjectsList;
            g_NumCurrentlyLost = currentAlternative->numCurrentlyLost;
            totalNumMobileVersions = 0;
            for(i = 0, newObj_it = g_newObjectsList->begin(); i<numberOfMobiles; i++, newObj_it++) {
                totalNumMobileVersions += numMobileVersions[i] = newObj_it->second.size();
                setBegins[i] = newObj_it->second.begin();
                setEnds[i]   = newObj_it->second.end();
            }
            //Information for managing validity matrices for mobile pairs
            g_numberOfNewMobiles = numberOfMobiles;
            g_numberOfNewMobileVersions = numMobileVersions;
            g_mobileIterators = mobileIterators;
            g_totalNumberOfMobileVersions = totalNumMobileVersions;
            initValidityMatrices(totalNumMobileVersions);

            g_setBegins = setBegins;

            //Generate initial leave with best solution
            bestAlternativesNode::numVariables = numberOfMobiles;
            bestAlternativesNode::newObjectsListEnds = setEnds;
            bestAlternativesNode::variablesNumFrames = numFramesVariables;

            g_leaves.clear();
            g_variableContribution = variableContribution;
            SpBestAlternativesNode root(new bestAlternativesNode(getTentativeAlternativeProbabilityValue(g_setBegins), 0));
            memcpy(root->mobileIterators, g_setBegins, sizeof(std::set<SpMobileObject, orderedByBestCoherenceOperator>::iterator) * numberOfMobiles );
            memcpy(root->variableContribution, g_variableContribution, sizeof(double) * numberOfMobiles );

            root->added = true;

            g_leaves.insert(root);

            SpRMAlternativeSolution newAlternative;

            /*	std::cout << "\tFor Alternative of probability " << currentAlternative->getProbability() << " containing:" << std::endl;
            mobile_it = currentAlternative->begin();
            size = currentAlternative->size();
            for(i=0; i< size; i++, mobile_it++)
                std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
                          << " of type "          << get_name_from_type((*mobile_it)->getBestType())
                          << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
            */
            if((newAlternative = getAlternativeFromNodeIfValid(root)) != NULL) {
                cleanByEnsureUsedBlobs(newAlternative);
                currentAlternative->newAlternatives.insert(newAlternative);
                g_NumLocallyAddedAlternatives++;
            }

            buildNewAlternativesFromLeaves(currentAlternative);

            /*
            size = g_newAlternatives.size();
            newalt_it = g_newAlternatives.begin();
            for(i=0; i< size; i++, newalt_it++) {
                std::cout << "\tAlternative " << i << " (P: " << (*newalt_it)->getProbability() << ") containing:" << std::endl;
                mobile_it = (*newalt_it)->begin();
                size2 = (*newalt_it)->size();
                for(j=0; j < size2; j++, mobile_it++)
                    std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
                              << " of type "          << get_name_from_type((*mobile_it)->getBestType())
                              << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
            }
            std::cout << std::endl;
            */
        }

        //Filter repeated alternatives between different sets of new alternatives
        //      filterRepeatedAlternatives(*rmobile_it);

        //Insert remaining alternatives for mobile
        g_newAlternatives.clear();
        for(alternative_it = (*rmobile_it)->begin(); alternative_it != (*rmobile_it)->end(); alternative_it++)
            g_newAlternatives.insert((*alternative_it)->newAlternatives.begin(), (*alternative_it)->newAlternatives.end());

        SpRMAlternativeSolution bestPrevious = *(*rmobile_it)->begin();
        (*rmobile_it)->clear();

        if((int)g_newAlternatives.size() > m_maximumRetainedAlternativeSolutions) {
            std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ending_iterator;
            for(i=0, ending_iterator = g_newAlternatives.begin(); i < m_maximumRetainedAlternativeSolutions; i++, ending_iterator++);

            (*rmobile_it)->insert(g_newAlternatives.begin(), ++ending_iterator);
        } else
            (*rmobile_it)->insert(g_newAlternatives.begin(), g_newAlternatives.end());

        //std::cout << "\tAfter Solutions Elimination:" << std::endl;
        //size = (*rmobile_it)->size();
        //newalt_it = (*rmobile_it)->begin();
        //for(i=0; i< size; i++, newalt_it++) {
        //std::cout << "\tAlternative " << i << " (P: " << (*newalt_it)->getProbability() << ") containing:" << std::endl;
        //mobile_it = (*newalt_it)->begin();
        //size2 = (*newalt_it)->size();
        //for(j=0; j < size2; j++, mobile_it++)
        //  std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
        //	    << " of type "          << get_name_from_type((*mobile_it)->getBestType())
        //	    << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
        //}
        //std::cout << std::endl;

        if((*rmobile_it)->size() == 0) {
            numberOfMobiles = bestPrevious->newObjectsList.size();
            g_newObjectsList = &bestPrevious->newObjectsList;

            int j;
            SpRMAlternativeSolution newAlternative(new RMAlternativeSolution());
            newAlternative->initUsedBlobsList();

            SpMobileObject currentMobile;
            for(i = 0, newObj_it = g_newObjectsList->begin(); i<numberOfMobiles; i++, newObj_it++) {
                currentMobile = *newObj_it->second.begin();
                newAlternative->insertNewMobileObject(currentMobile);
                for(j=0; j < blobsNumber; j++)
                    newAlternative->usedBlobs[j] |= currentMobile->usedBlobs[j];
            }

            newAlternative->setAlternativeProbability();
            (*rmobile_it)->insert(newAlternative);

        }

        (*rmobile_it)->setBestSolution();

        //Set used blobs lists
        (*rmobile_it)->numUsed = 0;
        for(alternative_it = (*rmobile_it)->begin(); alternative_it != (*rmobile_it)->end(); alternative_it++) {
            (*alternative_it)->numUsed = 0;
            for(i=0; i<blobsNumber; i++) {
                if((*alternative_it)->usedBlobs[i]) {
                    (*alternative_it)->numUsed++;
                    if((*rmobile_it)->usedBlobs[i] == false) {
                        (*rmobile_it)->numUsed++;
                        (*rmobile_it)->usedBlobs[i] = true;
                    }
                }
            }
        }

        //Set incomplete alternatives flags
        (*rmobile_it)->incompleteAlternatives = 0;
        for(alternative_it = (*rmobile_it)->begin(); alternative_it != (*rmobile_it)->end(); alternative_it++) {
            (*alternative_it)->incompleteAlternative = false;
            if((*alternative_it)->numUsed < (*rmobile_it)->numUsed) {
                (*alternative_it)->incompleteAlternative = true;
                (*rmobile_it)->incompleteAlternatives++;
            }
        }

        for(i=0; i<blobsNumber; i++)
            usedBlobs[i] |= (*rmobile_it)->usedBlobs[i];
    }
}

void ReliabilityTracker::initValidityMatrices(int size) {
    if(checkedMobilePairValidity != NULL)
        delete[] checkedMobilePairValidity;
    if(validMobilePair != NULL)
        delete[] validMobilePair;

    int pairValiditySize = size*size;

    checkedMobilePairValidity = new bool[pairValiditySize];
    validMobilePair = new bool[pairValiditySize];

    memset(checkedMobilePairValidity, false, pairValiditySize*sizeof(bool));
    memset(validMobilePair, false, pairValiditySize*sizeof(bool));
}


void ReliabilityTracker::followExistingMobiles() {

    //1. Set involved blobs for each reliable mobile (implies involved blobs for each alternative solution and mobile)
    //   and global relations of blobs
    setInvolvedBlobs();

    //2. Merge reliable mobiles involved in non-empty intersection of involved blobs.
    mergeInvolvedRMobiles();

    //2. Create possible movements for each mobile
    createMobilePossibilities();

    //3. Generate Alternatives with upper number limit and coherency bound
    generateAlternativesWithBestSolutionsTree();

    if(m_internalOutputActivated)
        std::cout << "After Path creation:" << std::endl << rMobilesList;
}

void ReliabilityTracker::setInvolvedBlobs() {
    int i;
    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralternative_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> *asolutions;
    std::deque<SpMobileObject>::iterator mobile_it;

    for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
        (*rmobile_it) -> initInvolvedBlobs();
        asolutions = (*rmobile_it)->getAlternativeSolutions();
        for(ralternative_it = asolutions->begin(); ralternative_it != asolutions->end(); ralternative_it++) {
            (*ralternative_it) -> initInvolvedBlobs();
            for(mobile_it = (*ralternative_it)->begin(); mobile_it != (*ralternative_it)->end(); mobile_it++) {
                (*mobile_it) -> initInvolvedBlobs();
                determineInvolvedBlobsForMobileObject(*mobile_it);
                for(i=0; i<blobsNumber; i++)
                    (*ralternative_it)->involvedBlobs[i] |= (*mobile_it) -> involvedBlobs[i];
            }
            for(i=0; i<blobsNumber; i++)
                (*rmobile_it)->involvedBlobs[i] |= (*ralternative_it) -> involvedBlobs[i];
        }

        for(i=0; i<blobsNumber; i++)
            if((*rmobile_it)->involvedBlobs[i])
                involvedRMobilesCounter[i]++;
    }

    if(m_internalOutputActivated) {
        std::cout << "\nInvolved RMobiles counter:\n";
        for(i=0; i<blobsNumber; i++)
            std::cout << involvedRMobilesCounter[i] << "\t";
        std::cout << std::endl;
    }
}

void ReliabilityTracker::getNearest2DBlobPointToFocalPoint(int position, double xCenter, double yCenter, double W, double H, double *x, double *y) {
    switch(position) {
        case 0:
            *x = xCenter + W/2;
            *y = yCenter + H/2;
            return;
        case 1:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = yCenter + H/2;
            return;
        case 2:
            *x = xCenter - W/2;
            *y = yCenter + H/2;
            return;
        case 3:
            *x = xCenter + W/2;
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 4:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 5:
            *x = xCenter - W/2;
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 6:
            *x = xCenter + W/2;
            *y = yCenter - H/2;
            return;
        case 7:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = yCenter - H/2;
            return;
        case 8:
            *x = xCenter - W/2;
            *y = yCenter - H/2;
            return;
        default:
            return;
    }
    return;
}

void ReliabilityTracker::getNearest2DBlobPointToFocalPoint(int position, Rectangle<int> *rect, double *x, double *y) {

    int
        W_2 = RECT_WIDTH(rect)  / 2,
        H_2 = RECT_HEIGHT(rect) / 2,
        xCenter = RECT_XLEFT(rect) + W_2,
        yCenter = RECT_YTOP(rect)  + H_2;

    switch(position) {
        case 0:
            *x = xCenter + W_2;
            *y = yCenter + H_2;
            return;
        case 1:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = yCenter + H_2;
            return;
        case 2:
            *x = xCenter - W_2;
            *y = yCenter + H_2;
            return;
        case 3:
            *x = xCenter + W_2;
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 4:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 5:
            *x = xCenter - W_2;
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 6:
            *x = xCenter + W_2;
            *y = yCenter - H_2;
            return;
        case 7:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = yCenter - H_2;
            return;
        case 8:
            *x = xCenter - W_2;
            *y = yCenter - H_2;
            return;
        default:
            return;
    }
    return;
}

void ReliabilityTracker::getNearest2DBlobPointToFocalPoint(Blob *blob, double *x, double *y) {

    switch(BLOB_POSITION(blob)) {
        case 0:
            *x = BLOB_XRIGHT(blob);
            *y = BLOB_YBOTTOM(blob);
            return;
        case 1:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = BLOB_YBOTTOM(blob);
            return;
        case 2:
            *x = BLOB_XLEFT(blob);
            *y = BLOB_YBOTTOM(blob);
            return;
        case 3:
            *x = BLOB_XRIGHT(blob);
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 4:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 5:
            *x = BLOB_XLEFT(blob);
            *y = SM_CAMERA_Y2D_FOC_POINT(smodel);
            return;
        case 6:
            *x = BLOB_XRIGHT(blob);
            *y = BLOB_YTOP(blob);
            return;
        case 7:
            *x = SM_CAMERA_X2D_FOC_POINT(smodel);
            *y = BLOB_YTOP(blob);
            return;
        case 8:
            *x = BLOB_XLEFT(blob);
            *y = BLOB_YTOP(blob);
            return;
        default:
            return;
    }

    return;
}

//TODO!!! ACA!!!: Modify involved blobs determination by using intersections with the higher possible
//rectangle determined with current mobile information.x
void ReliabilityTracker::determineInvolvedBlobsForMobileObject(SpMobileObject mobile) {
    int i, j;
    ObjectType best_type = mobile->getBestType();
    double V, xm, ym, xm_fp, ym_fp, dx, dy, x2d, y2d, xb, yb;
    bool mergeList[blobsNumber];
    bool useRadius = false, noPosInformation = false, noV2D = false, noV3D = false, noP2D = false, noP3D = false;

    //At least two classified s3ds are needed to know something about  3D velocity
    if(best_type == UNKNOWN || mobile->numberOfClassifiedS3ds < 2 || mobile->t3DSpatialData.RCV < SpatialCoherenceReliabilityThreshold)
        noV3D = true;
    //At least two found (not lost) s3ds are needed to know something about 2D velocity
    if(mobile->numberOfFoundS3ds < 2 || mobile->t2DSpatialData.RCV2D < SpatialCoherenceReliabilityThreshold)
        noV2D = true;

    //At least one classified s3d is needed to know something about 3D position
    if(best_type == UNKNOWN || mobile->numberOfClassifiedS3ds == 0 || mobile->t3DSpatialData.RCx < SpatialCoherenceReliabilityThreshold || mobile->t3DSpatialData.RCy < SpatialCoherenceReliabilityThreshold )
        noP3D = true;
    //At least one found (not lost) s3d is needed to know something about 2D velocity
    if(mobile->numberOfFoundS3ds == 0 || mobile->t2DSpatialData.RCX < SpatialCoherenceReliabilityThreshold || mobile->t2DSpatialData.RCY < SpatialCoherenceReliabilityThreshold )
        noP2D = true;

    //If no trustable velocity information, analyze the radius near the mobile
    if (noV3D && noV2D)
        useRadius = true;

    //If no trustable position information, use last not lost blob
    if (noP3D && noP2D)
        noPosInformation = true;

    //Search for solutions in a radius of action, because velocity information is not trustable
    if(useRadius) {
        int i;
        //Get the central point of the acceptation circle
        if(noPosInformation) {
            Rectangle<int> *lastNotLost = NULL;
            //Get last not lost 2D information
            for(i=0; i<mobile->currentBufferSize; i++) {
                if(!(mobile->dpFlags[i] & MM_OBJECT_LOST)) {
                    lastNotLost = &(mobile->bboxesToAnalyze[i]);
                    break;
                }
            }
            if(lastNotLost == NULL)
                lastNotLost = &(mobile->bboxesToAnalyze[0]);

            getNearest2DBlobPointToFocalPoint(lastNotLost->getPositionRelativeToCamera(smodel), lastNotLost, &x2d, &y2d);
            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xm, &ym);
        } else if(noP3D) { //Not trustable 3D information
            getNearest2DBlobPointToFocalPoint(mobile->bboxesToAnalyze[0].getPositionRelativeToCamera(smodel), mobile->t2DSpatialData.X, mobile->t2DSpatialData.Y, mobile->t2DDimData.W, mobile->t2DDimData.H, &x2d, &y2d);
            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xm, &ym);
        } else { //Trustable 3D information
            xm = mobile->t3DSpatialData.x;
            ym = mobile->t3DSpatialData.y;
            getNearest2DBlobPointToFocalPoint(mobile->bboxesToAnalyze[0].getPositionRelativeToCamera(smodel), mobile->t2DSpatialData.X, mobile->t2DSpatialData.Y, mobile->t2DDimData.W, mobile->t2DDimData.H, &x2d, &y2d);
            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xm_fp, &ym_fp);
        }

        //As we don't have trustable velocity information, determine velocity depending on
        //apriori knowledge
        if(best_type == UNKNOWN)
            V = maxObjectSpeed;
        else       //If the type is known take maximal model speed
            V = MobileObject::objectModelMaxVelocity   [MobileObject::objectModelMap[best_type]];

        double dist_radius = V * MobileObject::secDiffSequence[0], d_2;
        d_2 = dist_radius*dist_radius;

        for (i=0; i<blobsNumber; i++) {
            if(mobile->involvedBlobs[i] == false) { //blob not analyzed or included yet

                if(noP3D) {
                    getNearest2DBlobPointToFocalPoint(blobsVector[i], &x2d, &y2d);
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xb, &yb);
                    dx = xm - xb;
                    dy = ym - yb;
                } else { //Trustable position information from both sources
                    if(BLOB_P(blobsVector[i]) >= MobileObject::m_classifThreshold) {
                        dx = xm - BLOB_3D_X(blobsVector[i]);
                        dy = ym - BLOB_3D_Y(blobsVector[i]);
                    } else {
                        getNearest2DBlobPointToFocalPoint(blobsVector[i], &x2d, &y2d);
                        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xb, &yb);
                        dx = xm_fp - xb;
                        dy = ym_fp - yb;
                    }
                }

                if(d_2 > dx*dx + dy*dy) {
                    getMergeConnections(mergeList, i);

                    for(j=0; j<blobsNumber; j++)
                        if(mergeList[j] == true)
                            mobile -> involvedBlobs[j] = true;

                }
            }
        }

        //Added to avoid problems
        Rectangle<int> augumentedBBox;
        if(mobile->getNumberOfFramesSinceFirstTimeSeen() > 1) {
            double
                X = mobile->t2DSpatialData.X + mobile->t2DSpatialData.VX*MobileObject::secDiffSequence[0],
                Y = mobile->t2DSpatialData.Y + mobile->t2DSpatialData.VY*MobileObject::secDiffSequence[0],
                SDW = mobile->t2DDimData.SDW + fabs(mobile->t2DDimData.VW + mobile->t2DDimData.SDVW)*MobileObject::secDiffSequence[0],
                SDH = mobile->t2DDimData.SDH + fabs(mobile->t2DDimData.VH + mobile->t2DDimData.SDVH)*MobileObject::secDiffSequence[0],
                W_2 = mobile->t2DDimData.W / 2.0,
                H_2 = mobile->t2DDimData.H / 2.0;
            RECT_XLEFT(&augumentedBBox)   = (int) (X - W_2 - SDW);
            RECT_XRIGHT(&augumentedBBox)  = (int) (X + W_2 + SDW);
            RECT_YTOP(&augumentedBBox)    = (int) (Y - H_2 - SDH);
            RECT_YBOTTOM(&augumentedBBox) = (int) (Y + H_2 + SDH);
        } else { //Triplicated size of bbox to find correspondences
            double
                X = mobile->t2DSpatialData.X,
                Y = mobile->t2DSpatialData.Y,
                W = mobile->t2DDimData.W,
                H = mobile->t2DDimData.H;
            RECT_XLEFT(&augumentedBBox)   = (int) (X - W);
            RECT_XRIGHT(&augumentedBBox)  = (int) (X + W);
            RECT_YTOP(&augumentedBBox)    = (int) (Y - H);
            RECT_YBOTTOM(&augumentedBBox) = (int) (Y + H);
        }
        RECT_WIDTH(&augumentedBBox)  = RECT_XRIGHT(&augumentedBBox) - RECT_XLEFT(&augumentedBBox) + 1;
        RECT_HEIGHT(&augumentedBBox) = RECT_YBOTTOM(&augumentedBBox) - RECT_YTOP(&augumentedBBox) + 1;

        //Now check if current blobs intersect the augumented bbox
        for (i=0; i<blobsNumber; i++)
            if(    mobile->involvedBlobs[i] == false //blob not analyzed or included yet
                && Rectangle<int>::thereIsIntersection(&augumentedBBox, BLOB_BBOX(blobsVector[i])) ) {
                getMergeConnections(mergeList, i);
                for(j=0; j<blobsNumber; j++)
                    if(mergeList[j] == true)
                        mobile -> involvedBlobs[j] = true;
            }

        for(j=0; j<blobsNumber; j++)
            if(mobile -> involvedBlobs[j] == true)
                mobile -> numInvolved++;

        return;
    }

    double SDxm, SDym, max_displacement;

    //In this case, we have reliable velocity information
    //Get the central point of the acceptation square using available information
    if(noPosInformation) {

        Rectangle<int> *lastNotLost = NULL;
        //Get last not lost 2D information
        for(i=0; i<mobile->currentBufferSize; i++) {
            if(!(mobile->dpFlags[i] & MM_OBJECT_LOST)) {
                lastNotLost = &(mobile->bboxesToAnalyze[i]);
                break;
            }
        }
        if(lastNotLost == NULL)
            lastNotLost = &(mobile->bboxesToAnalyze[0]);

        getNearest2DBlobPointToFocalPoint(lastNotLost->getPositionRelativeToCamera(smodel), lastNotLost, &x2d, &y2d);
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), mobile->t2DSpatialData.X, mobile->t2DSpatialData.Y, 0, &xm, &ym);

        //This value will be changed for sure, in a later validation
        SDxm = SDym = 0.0;
    } else if(noP3D) { //Not trustable 3D information
        getNearest2DBlobPointToFocalPoint(mobile->bboxesToAnalyze[0].getPositionRelativeToCamera(smodel),
                                          mobile->t2DSpatialData.X  + mobile->t2DSpatialData.VX * MobileObject::secDiffSequence[0],
                                          mobile->t2DSpatialData.Y  + mobile->t2DSpatialData.VY * MobileObject::secDiffSequence[0],
                                          mobile->t2DDimData.W, mobile->t2DDimData.H, &x2d, &y2d);
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xm, &ym);
        //Total variation of xm and ym must be taken from the maximal possible variation
        //Extracted from the four limit points considering +/-(SDX+SDVX), and +/-(SDY+SDVY);
        getHighest3DVariations(xm, ym, x2d, y2d,
                               mobile->t2DSpatialData.SDX + mobile->t2DSpatialData.SDVX*MobileObject::secDiffSequence[0],
                               mobile->t2DSpatialData.SDY + mobile->t2DSpatialData.SDVY*MobileObject::secDiffSequence[0],
                               &SDxm, &SDym);
    } else { //Trustable 3D point information
        xm = mobile->t3DSpatialData.x + mobile->t3DSpatialData.Vx*MobileObject::secDiffSequence[0];
        ym = mobile->t3DSpatialData.y + mobile->t3DSpatialData.Vy*MobileObject::secDiffSequence[0];

        SDxm = mobile->t3DSpatialData.SDx + mobile->t3DSpatialData.SDVx*MobileObject::secDiffSequence[0];
        SDym = mobile->t3DSpatialData.SDy + mobile->t3DSpatialData.SDVy*MobileObject::secDiffSequence[0];

        getNearest2DBlobPointToFocalPoint(mobile->bboxesToAnalyze[0].getPositionRelativeToCamera(smodel), mobile->t2DSpatialData.X, mobile->t2DSpatialData.Y, mobile->t2DDimData.W, mobile->t2DDimData.H, &x2d, &y2d);
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xm_fp, &ym_fp);
    }

    //Calculate the maximal SD as the maximal possible displacement.
    if(best_type == UNKNOWN ) //If the type is unknown take the default maximal speed value
        V = maxObjectSpeed;
    else     //If the type is known take maximal model speed.
        V = MobileObject::objectModelMaxVelocity[MobileObject::objectModelMap[best_type]];

    max_displacement = V * MobileObject::secDiffSequence[0];

    if(SDxm < max_displacement )
        SDxm = max_displacement;
    if(SDym < max_displacement )
        SDym = max_displacement;

    for (i=0; i<blobsNumber; i++) {
        if(mobile -> involvedBlobs[i] == false) { //blob not analyzed or included yet
            if(noP3D) {
                getNearest2DBlobPointToFocalPoint(blobsVector[i], &x2d, &y2d);
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xb, &yb);
                dx = xm - xb;
                dy = ym - yb;
            } else { //Trustable position information from both sources
                if(BLOB_P(blobsVector[i]) >= MobileObject::m_classifThreshold) {
                    dx = xm - BLOB_3D_X(blobsVector[i]);
                    dy = ym - BLOB_3D_Y(blobsVector[i]);
                } else {
                    getNearest2DBlobPointToFocalPoint(blobsVector[i], &x2d, &y2d);
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), x2d, y2d, 0, &xb, &yb);
                    dx = xm_fp - xb;
                    dy = ym_fp - yb;
                }
            }

            if(fabs(dx) <= fabs(SDxm) && fabs(dy) <= fabs(SDym)) {
                getMergeConnections(mergeList, i);

                for(j=0; j<blobsNumber; j++)
                    if(mergeList[j] == true)
                        mobile -> involvedBlobs[j] = true;
            }
        }
    }

    //Added to avoid problems
    Rectangle<int> augumentedBBox;
    if(mobile->getNumberOfFramesSinceFirstTimeSeen() > 1) {
        double
            X = mobile->t2DSpatialData.X + mobile->t2DSpatialData.VX*MobileObject::secDiffSequence[0],
            Y = mobile->t2DSpatialData.Y + mobile->t2DSpatialData.VY*MobileObject::secDiffSequence[0],
            SDW = mobile->t2DDimData.SDW + fabs(mobile->t2DDimData.VW + mobile->t2DDimData.SDVW)*MobileObject::secDiffSequence[0],
            SDH = mobile->t2DDimData.SDH + fabs(mobile->t2DDimData.VH + mobile->t2DDimData.SDVH)*MobileObject::secDiffSequence[0],
            W_2 = mobile->t2DDimData.W / 2.0,
            H_2 = mobile->t2DDimData.H / 2.0;
        RECT_XLEFT(&augumentedBBox)   = (int) (X - W_2 - SDW);
        RECT_XRIGHT(&augumentedBBox)  = (int) (X + W_2 + SDW);
        RECT_YTOP(&augumentedBBox)    = (int) (Y - H_2 - SDH);
        RECT_YBOTTOM(&augumentedBBox) = (int) (Y + H_2 + SDH);
    } else { //Triplicated size of bbox to find correspondences
        double
            X = mobile->t2DSpatialData.X,
            Y = mobile->t2DSpatialData.Y,
            W = mobile->t2DDimData.W,
            H = mobile->t2DDimData.H;
        RECT_XLEFT(&augumentedBBox)   = (int) (X - W);
        RECT_XRIGHT(&augumentedBBox)  = (int) (X + W);
        RECT_YTOP(&augumentedBBox)    = (int) (Y - H);
        RECT_YBOTTOM(&augumentedBBox) = (int) (Y + H);
    }
    RECT_WIDTH(&augumentedBBox)  = RECT_XRIGHT(&augumentedBBox) - RECT_XLEFT(&augumentedBBox) + 1;
    RECT_HEIGHT(&augumentedBBox) = RECT_YBOTTOM(&augumentedBBox) - RECT_YTOP(&augumentedBBox) + 1;

    //Now check if current blobs intersect the augumented bbox
    for (i=0; i<blobsNumber; i++)
        if(    mobile->involvedBlobs[i] == false //blob not analyzed or included yet
               && Rectangle<int>::thereIsIntersection(&augumentedBBox, BLOB_BBOX(blobsVector[i])) ) {
            getMergeConnections(mergeList, i);
            for(j=0; j<blobsNumber; j++)
                if(mergeList[j] == true)
                    mobile -> involvedBlobs[j] = true;
        }

    for(j=0; j<blobsNumber; j++)
        if(mobile -> involvedBlobs[j] == true)
            mobile -> numInvolved++;

}

void ReliabilityTracker::getHighest3DVariations(double x, double y, double X, double Y, double dX, double dY, double *dx, double *dy) {
    double max_dx = 0, max_dy = 0, cur_dx, cur_dy, x3d, y3d;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X + dX, Y + dY, 0, &x3d, &y3d);
    cur_dx = fabs(x - x3d);    cur_dy = fabs(y - y3d);
    if(cur_dx > max_dx) max_dx = cur_dx;
    if(cur_dy > max_dy) max_dy = cur_dy;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X + dX, Y - dY, 0, &x3d, &y3d);
    cur_dx = fabs(x - x3d);    cur_dy = fabs(y - y3d);
    if(cur_dx > max_dx) max_dx = cur_dx;
    if(cur_dy > max_dy) max_dy = cur_dy;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X - dX, Y + dY, 0, &x3d, &y3d);
    cur_dx = fabs(x - x3d);    cur_dy = fabs(y - y3d);
    if(cur_dx > max_dx) max_dx = cur_dx;
    if(cur_dy > max_dy) max_dy = cur_dy;

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X - dX, Y - dY, 0, &x3d, &y3d);
    cur_dx = fabs(x - x3d);    cur_dy = fabs(y - y3d);
    if(cur_dx > max_dx) max_dx = cur_dx;
    if(cur_dy > max_dy) max_dy = cur_dy;

    *dx = max_dx;
    *dy = max_dy;

}

void ReliabilityTracker::getMergeConnections(bool *mergeList, int startingBlobIndex) {
    int i;

    memset(mergeList, false, sizeof(bool)*blobsNumber);
    mergeList[startingBlobIndex] = true;

    for(i=0; i<blobsNumber; i++)
        if(i != startingBlobIndex && initialMergeMap[startingBlobIndex][i])
            getRecursiveMergeConnections(mergeList, i);
}

void ReliabilityTracker::getRecursiveMergeConnections(bool *mergeList, int blobIndex) {
    int i;

    if(mergeList[blobIndex])
        return;

    mergeList[blobIndex] = true;

    for(i = blobIndex + 1; i<blobsNumber; i++)
        if(initialMergeMap[blobIndex][i])
            getRecursiveMergeConnections(mergeList, i);
}


//TODO: Make difference with a MM_TOTAL_OCCLUSION situation.
SpMobileObject ReliabilityTracker::setNotVisibleBlobMobile(DetectionProblemType dp_type, SpMobileObject currentObject) {

    SpMobileObject specialMobile(new MobileObject(currentObject));
    Blob *currentBlob = generateMostLikelyAlternativeForMobile(currentObject);
    bool noBest = false;

    if(currentBlob == NULL) {
        noBest = true;
        currentBlob = (currentObject->blobHistory.back())->copyWithLists();
        BLOB_TYPE(currentBlob) = UNKNOWN;
    }

    specialMobile->incrementNumberOfFramesNotSeen();

    //Blob's data is totally unreliable
    BLOB_R(currentBlob) = BLOB_RW(currentBlob) = BLOB_RL(currentBlob) = BLOB_RH(currentBlob) = 0;
    BLOB_DP_TYPE(currentBlob) = dp_type;

    std::map<ObjectType, Shape3DData> *normal_data = BLOB_NORMAL_3DDATA(currentBlob);
    if(normal_data) {
        std::map<ObjectType, Shape3DData>::iterator it, it_end = normal_data->end();
        for(it = normal_data->begin(); it != it_end; it++)
            S3D_DP_TYPE(&(*it).second) = dp_type;
    }
    specialMobile->insertNewBlob(currentBlob, lastMilliSecondsDifference);

    specialMobile->updateMobileData();

    specialMobile->setGlobalProbability();

    //if(noBest)
    delete currentBlob;

    return specialMobile;

}

SpMobileObject ReliabilityTracker::checkSpecialCases(SpMobileObject currentMobile, Blob *blobToTest) {
    //First check if obtained solution does not respond sufficiently to predictions,
    Rectangle<int> mobileBBox;
    currentMobile->getCurrentBoundingBoxForMobile(&mobileBBox);
    double
        //Visual support is 1.0 when mobile bbox is completelly covered by the analysed blob
        support1 = Rectangle<int>::rectangleIntersectRatio(&mobileBBox, &(blobToTest->bbox)),
        //Visual support is 1.0 when analysed blob is completelly covered by the mobile bbox
        support2 = Rectangle<int>::rectangleIntersectRatio(&(blobToTest->bbox), &mobileBBox);
    //If both support values are big, it means that used 2D bounding box fits to predictions
    if(support1 >= m_highVisualSupportThreshold && support2 >= m_highVisualSupportThreshold)
        return QSharedPointer<MobileObject>();

    //Else check for special cases:
    //1. The 2D width, and 2D height of both blob and prediction are similar, or fit to
    //   expected change but, the coverage is not good, it could be:
    //   a. that the visual evidence is displaced with respect to the expected position, so
    //      this case is normal and can be considered as covered by the new mobile.
    double
        Wdiff = abs(RECT_WIDTH(&mobileBBox)  - BLOB_WIDTH(blobToTest)),
        Hdiff = abs(RECT_HEIGHT(&mobileBBox) - BLOB_HEIGHT(blobToTest)),
        Wtol, Htol;

    currentMobile->getMobile3DTolerances(&Wtol, &Htol);
    if(Wdiff < Wtol && Hdiff < Htol)
        return QSharedPointer<MobileObject>();

    //   b. TODO: Weird case:
    //      The object is partially detected and another object is detected at the same place
    //      making seem similar to the expected size. This case should be analysed at hypothesis
    //      generation level, but it is not a priority in implementation.

    //2. There exist a remarkable difference in expected size of visual evidence. Depending on these differences
    //   different cases can arise:
    //   a.
    SpMobileObject specialMobile;
    double
        mobileArea = RECT_WIDTH(&mobileBBox)*RECT_HEIGHT(&mobileBBox),
        blobArea =  BLOB_WIDTH(blobToTest)*BLOB_HEIGHT(blobToTest);

    if(support2 >= m_highVisualSupportThreshold  && mobileArea > blobArea) //PARTIALLY DETECTED MOBILE
        specialMobile = setSpecialCaseMobile(MM_PARTIALLY_DETECTED, currentMobile, blobToTest);
    else if(support1 < m_lowVisualSupportThreshold && support2 < m_highVisualSupportThreshold) { //LOST MOBILE
        Blob *specialBlob = new Blob();
        memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
        BLOB_POSITION(specialBlob) = specialBlob->getPositionRelativeToCamera(smodel);
        specialMobile = setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob);
        delete specialBlob;
    } else if (support1 >= m_highVisualSupportThreshold &&  blobArea > mobileArea) //PART OF BIGGER
        specialMobile = setSpecialCaseMobile(MM_PART_OF_BIGGER, currentMobile, blobToTest);

    return specialMobile;
}


SpMobileObject ReliabilityTracker::generateAndValidateNewMobile(SpMobileObject currentMobile, Blob *blobToTest) {

    SpMobileObject newMobile(new MobileObject(currentMobile));
    Rectangle<int> mobileBBox;

    newMobile->updateMobilePath(blobToTest);

    if(incoherentMobile(newMobile)) // || (!newMobile->ensureMode && !acceptableInformationForNewMobile(newMobile, currentMobile)))
        return QSharedPointer<MobileObject>();

    return newMobile;
}

SpMobileObject ReliabilityTracker::setSpecialCaseMobile(DetectionProblemType dp_type, SpMobileObject currentObject, Blob *blobToAdd) {

    SpMobileObject specialMobile(new MobileObject(currentObject));

    Blob *blobForMobile = blobToAdd->copyWithLists();

    if(dp_type == MM_OBJECT_LOST) {
        specialMobile->incrementNumberOfFramesNotSeen();
        //Blob's 3Ddata is totally unreliable
        BLOB_R(blobForMobile) = BLOB_RW(blobForMobile) = BLOB_RL(blobForMobile) = BLOB_RH(blobForMobile) = 0.0;
    }

    BLOB_DP_TYPE(blobForMobile) = dp_type;

    std::map<ObjectType, Shape3DData> *normal_data = BLOB_NORMAL_3DDATA(blobForMobile);
    if(normal_data != NULL) {
        std::map<ObjectType, Shape3DData>::iterator it, it_end = normal_data->end();
        for(it = normal_data->begin(); it != it_end; it++)
            S3D_DP_TYPE(&(*it).second) = dp_type;
    }

    specialMobile->insertNewBlob(blobForMobile, lastMilliSecondsDifference);
    specialMobile->updateMobileData();
    specialMobile->setGlobalProbability();

    delete blobForMobile;

    return specialMobile;
}

//Function for generation of new mobiles, using tracking information. This function takes into account
//an initial solution to serve as starting point of future merges of involved blobs with currently analyzed mobile.
//Solutions
void ReliabilityTracker::generateMobilePathFromInitialBlob(SpMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, double *blobSupport, Rectangle<int> *mobileBBox) {

    // 1. Set vector of blobs to analyze according to involved, not used blobs with support on the mobile estimated bbox
    int i;
    int validBlobsVector[blobsNumber], validBlobsNumber = 0;

    for(i=0; i<blobsNumber; i++) {
        if(mobile->involvedBlobs[i] && !usedBlobs[i] && blobSupport[i] > 0.0)
            validBlobsVector[validBlobsNumber++] = i;
    }
    if(validBlobsNumber > 0) {
        // 3. Generate blobs for the valid blobs
        int j;
        int blobsToMerge[validBlobsNumber], blobsToMergeNumber;

        //Set the blobs to merge per group
        blobsToMergeNumber = 0;
        for(j=0; j<validBlobsNumber; j++)
            blobsToMerge[blobsToMergeNumber++] = validBlobsVector[j];

        int alternativesCombo[blobsToMergeNumber];
        for(j=1; j<=validBlobsNumber; j++)
            generateAlternativesForMobilePathFromInitialBlob(j, 0, 0, alternativesCombo, blobsToMergeNumber, blobsToMerge,
                                                             mobile, initialBlob, numUsed, usedBlobs, mobileBBox);
    }
}


void ReliabilityTracker::generateMobilePath(SpMobileObject mobile) {
    int i;

    // 1. Set vector of blobs to analyze according to involved, not used blobs
    int validBlobsVector[blobsNumber], validBlobsNumber = 0;
    for(i=0; i<blobsNumber; i++)
        if(mobile->involvedBlobs[i])
            validBlobsVector[validBlobsNumber++] = i;

    if(validBlobsNumber > 0) {
        // 2. Generate possible groups of blobs
        int groupsVector[blobsNumber];
        int groupsNumber = setGroups(groupsVector, initialMergeMap, validBlobsNumber, validBlobsVector);

        // 3. Generate blob for each group
        int j;
        int blobsToMerge[validBlobsNumber], blobsToMergeNumber;

        mobile->accepted_solution = false;
        //Set global variables used by the recursive function

        for(i=0; i<groupsNumber; i++) {

            //Set the blobs to merge per group
            blobsToMergeNumber = 0;
            for(j=0; j<validBlobsNumber; j++)
                if(groupsVector[j] == i)
                    blobsToMerge[blobsToMergeNumber++] = validBlobsVector[j];

            int alternativesCombo[blobsToMergeNumber];
            for(j=1; j<=validBlobsNumber; j++)
                generateAlternativesForMobilePath(j, 0, 0, alternativesCombo, blobsToMergeNumber, blobsToMerge, mobile);
        }
    }
}


bool ReliabilityTracker::incoherentMobile(SpMobileObject mobile) {
    bool
        M3D
        =    (mobile->ensureMode && mobile->numberOfClassifiedS3ds > 1 && mobile->PVC < IgnoreByVelocityCoherenceThreshold)
        ? false : true,
        M2D
        =    (mobile->numberOfFoundS3ds > 2 && mobile->PV2DC >= IgnoreByVelocityCoherenceThreshold)
        ||   mobile->numberOfFoundS3ds <= 2 ? true : false,
        S3D
        =    (mobile->ensureMode && mobile->numberOfClassifiedS3ds > 1 && mobile->P3D < IgnoreByDimensionalCoherenceThreshold)
        ? false : true,
        S2D
        =    (mobile->numberOfFoundS3ds > 1 && mobile->P2D >= IgnoreByDimensionalCoherenceThreshold)
        ||   mobile->numberOfFoundS3ds <= 1 ? true : false;

    //Movement Criteria: Accomplish with all criteria accomplished for being coherent
    if(    M3D && M2D
    //Size Criteria: Accomplish with at least one criteria if available for being coherent
        && S3D && S2D
    //Probability Criteria: Coherent enough with respect to best one found
        && ( mobile->getGlobalProbability() >= g_bestGlobalP*ImportanceRateForBestMobileAlternativeProbability) )
        return false;
    return true;
}


bool ReliabilityTracker::blobAlreadyIncludedInNewMobilesSet(Blob *blob) {
    std::deque<Blob *>::iterator blob_it = g_includedBlobsInNewMobiles.begin();
    int i, size = g_includedBlobsInNewMobiles.size();

    for(i=0; i < size; i++, blob_it++)
        if(Blob::same2DBlob(blob, *blob_it))
            return true;

    return false;
}

void ReliabilityTracker::generateAlternativesForMobilePathFromInitialBlob(int length, int position, int value, int *alternativesCombo,
                                                                          int blobsToMergeNumber, int *blobsToMerge, SpMobileObject mobile,
                                                                          Blob *initialBlob, int numUsed, bool *usedBlobs, Rectangle<int> *mobileBBox) {

    int i;

    if(position == length) {

        //Generate the blob alternative for mobile
        if(m_internalOutputActivated) {
            std::cout << "For Mobile: " <<mobile->getMobileId() << std::endl;
            for(i=0; i < length; i++)
                std::cout << alternativesCombo[i] << " ";
                std::cout << std::endl;
        }

        //Get new merged blob
        Blob *merged_blob = initialBlob, *new_blob, *curr_blob;
        bool currently_used[length], thereIsIntersecting;
        int distance, minDistance, maxDistance = 0, normalization,
            minDistanceIndex, hd, vd, minAreaIndex = -1, minArea = INT_MAX;
        double maxDistanceFactor = 1.0;
        Rectangle<int> *maxRect, *curRect;
        memset(currently_used, false, length*sizeof(int));

        //Build merge blob with nearest blob in distance between possible blobs and current merge blob.
        //Calculate the intersection between all blobs.  to the determine current merge blob and mobile bbox to calculate distance.

        int k;
        for(k=0; k < length; k++) { //Merge each blob one by one
        
            //Get next blob to merge as the nearest to current merge blob
            //If there is intersected, take least area first
            minArea = INT_MAX;
            minDistance = INT_MAX;
            thereIsIntersecting = false;
            for(i=0; i < length; i++) {
                if(!currently_used[i]) {
                    curRect = &(blobsVector[alternativesCombo[i]]->bbox);
                    //Check first minimal area if there are intersected blobs
                    //as it will allow to properly calculate future distances
                    if(Rectangle<int>::thereIsIntersection(&(merged_blob->bbox),curRect)) {
                        thereIsIntersecting = true;
                        if(curRect->width*curRect->height < minArea) {
                            minArea = curRect->width*curRect->height;
                            minAreaIndex = i;
                            minDistance = 0;
                        }
                    } else if(!thereIsIntersecting) { //Check minimal distance if no intersections
                                                      //already found.
                        hd = Rectangle<int>::horizontalDistance(&(merged_blob->bbox),curRect);
                        vd = Rectangle<int>::verticalDistance(&(merged_blob->bbox),curRect);
                        distance = std::max(hd, vd);
                        if(distance < minDistance) {
                            minDistance = distance;
                            minDistanceIndex = i;
                        }
                    }
                }
            }
            if(thereIsIntersecting) {
                curr_blob = blobsVector[alternativesCombo[minAreaIndex]];
                currently_used[minAreaIndex] = true;
            } else {
                curr_blob = blobsVector[alternativesCombo[minDistanceIndex]];
                currently_used[minDistanceIndex] = true;
            }

            if(minDistance > maxDistance)
                maxDistance = minDistance;

            if(!Blob::isBlob1InsideBlob2(curr_blob, merged_blob)) { //If analyzed blob is not already contained in initial blob merge, merge it.
                new_blob = Blob::mergeBlob(merged_blob, curr_blob);
                g_allocatedBlobs.push_front(new_blob);
                merged_blob = new_blob;
            }
        }

        //maxDistanceFactor is a measure of separability of visual evidences.
        //maxDistanceFactor = 1.0 if visual evidence are intersecting each other.
        //maxDistanceFactor --> 0.0 if visual evidences are too far between them.

        if(maxDistance == 0)
            maxDistanceFactor = 1.0;
        else {
            //normalisation factor as the maximal difference between width or height of maximal rectangle and initial blob rectangle.
            maxRect = &(merged_blob->bbox);
            curRect = &(initialBlob->bbox);
            normalization = std::max(maxRect->width - curRect->width, maxRect->height - curRect->height);
            maxDistanceFactor = std::max(0.000001, 1.0 - (double)maxDistance/(double)normalization);
        }
        merged_blob->maxDistanceFactor = maxDistanceFactor;
        g_includedBlobsInNewMobiles.push_front(merged_blob);

        //Generate new mobile
        SpMobileObject newMobile = generateAndValidateNewMobile(mobile, merged_blob);

        bool specialCase = false;

        if (newMobile == NULL) {
            if(mobile->ensureMode || mobile->numberOfFramesSinceFirstTimeSeen > 2*MobileObject::m_blobsBufferSize) {
                newMobile = checkSpecialCases(mobile, initialBlob);
                if(newMobile != NULL)
                    specialCase = true;
            }
        }

        //If minimal coherence is not achieved, and we already have at least one normal solution
        //we can filter other solutions
        if( newMobile == NULL || (g_newMobiles.size() > 1 && ( incoherentMobile(newMobile) || specialCase )) )
            return;

        if (newMobile->getGlobalProbability() > g_bestGlobalP)
            g_bestGlobalP = newMobile->getGlobalProbability();

        //Set initial used blobs for mobile
        memcpy(newMobile->usedBlobs, usedBlobs, blobsNumber*sizeof(bool));
        //Set new used blobs for mobile
        for(i=0; i < length; i++)
            newMobile->usedBlobs[alternativesCombo[i]] = true;
        newMobile->numUsed = length + numUsed;

        if(specialCase)
            g_newSpecialMobiles.insert(newMobile);
        else
            g_newMobiles.insert(newMobile);
    } else {
        int limit = blobsToMergeNumber - length + position;
        for(i=value; i <= limit; i++) {
            alternativesCombo[position] = blobsToMerge[i];
            generateAlternativesForMobilePathFromInitialBlob(length, position+1, i+1, alternativesCombo,
                                                             blobsToMergeNumber, blobsToMerge, mobile,
                                                             initialBlob, numUsed, usedBlobs, mobileBBox);
        }
    }
}


void ReliabilityTracker::generateAlternativesForMobilePath(int length, int position, int value, int *alternativesCombo,
                                                           int blobsToMergeNumber, int *blobsToMerge, SpMobileObject mobile) {
    int i, j;

    if(position == length) {

        //Generate the blob alternative for mobile
        if(m_internalOutputActivated) {
            std::cout << "For Mobile: " <<mobile->getMobileId() << std::endl;
            for(i=0; i < length; i++)
                std::cout << alternativesCombo[i] << " ";
            std::cout << std::endl;
        }

        //Check blobs proximity
        Blob *curr_blob;
        bool blobTooFar = false, allfar;
        double distance, meanDim[length];

        if(length > 1) {
            for(i=0; i < length; i++) {
                curr_blob = blobsVector[alternativesCombo[i]];
                meanDim[i] = (BLOB_WIDTH(curr_blob) + BLOB_HEIGHT(curr_blob)) / 2.0;
            }

            for(i=0; i < length; i++) {
                allfar = true;
                for(j=0; j < length; j++) {
                    if(i != j) {
                        distance = Rectangle<int>::rectangleDistance(&(blobsVector[alternativesCombo[i]]->bbox), &(blobsVector[alternativesCombo[j]]->bbox));
                        if(distance < meanDim[i] && distance < meanDim[j])
                            allfar = false;
                    }
                    if(allfar) {
                        blobTooFar = true;
                        break;
                    }
                }
            }
        }

        if(blobTooFar)
            return;

        //Get new merged blob
        Blob *merged_blob;
        if(length > 1)
            //Classification is controlled at MobileObject level
            merged_blob = m_RMerge->getMergedBlob(alternativesCombo, length, blobsVector, blobsNumber, false);
        else
            merged_blob = blobsVector[alternativesCombo[0]];

        if(blobAlreadyIncludedInNewMobilesSet(merged_blob))
            return;

        g_includedBlobsInNewMobiles.push_front(merged_blob);

        //Generate new mobile
        SpMobileObject newMobile(new MobileObject(mobile));
        newMobile->updateMobilePath(merged_blob);

        //If minimal coherence is not achieved, solution will not be taken into account
        if(incoherentMobile(newMobile))
            return;

        if (newMobile->getGlobalProbability() > g_bestGlobalP)
            g_bestGlobalP = newMobile->getGlobalProbability();

        //Set used blobs for mobile
        for(i=0; i < length; i++)
            newMobile->usedBlobs[alternativesCombo[i]] = true;
        newMobile->numUsed = length;

        g_newMobiles.insert(newMobile);
        mobile->accepted_solution = true;

    } else {
        int limit = blobsToMergeNumber - length + position;
        for(i=value; i <= limit; i++) {
            alternativesCombo[position] = blobsToMerge[i];
            generateAlternativesForMobilePath(length, position+1, i+1, alternativesCombo,
                                              blobsToMergeNumber, blobsToMerge, mobile);
        }
    }
}

Blob *ReliabilityTracker::generateMostLikelyAlternativeForMobile(SpMobileObject mobile) {
    return mobile->determineMostLikelyBlob();
}

void ReliabilityTracker::eliminateUnlikelyMobiles() {
    //for each reliable mobile do
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator asIt;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>* alternativeSolutions;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> remainingAlternatives;
    std::deque<SpReliableMobileObject>::iterator it;
    int n = m_maximumRetainedAlternativeSolutions;
    bool changed;
    for(it = rMobilesList.begin(); it != rMobilesList.end();it++){
        alternativeSolutions = (*it)->getAlternativeSolutions();
        remainingAlternatives.clear();

        changed = false;

        //      std::cout << "Number of alternatives before: " << alternativeSolutions->size() << std::endl;

        double sum = 0, bestP = (*it)->getBestProbability();
        int N = alternativeSolutions->size();
        //if the probability of an alternative solution is less than a threshold remove it
        asIt = alternativeSolutions->begin();
        remainingAlternatives.insert(*asIt);
        for(asIt++;asIt!=alternativeSolutions->end();asIt++){
            if( (*asIt)->justFirstFrames ) {
                remainingAlternatives.insert(*asIt);
                N--;
            } else if( (*asIt)->getProbability()/ bestP < alternativeSolutionsProbabilityThreshold ){
                changed = true;
                N--;
            } else {
                sum += (*asIt)->getProbability();
                remainingAlternatives.insert(*asIt);
            }
        }

        //If modified, regenerate the list
        if(changed) {
            alternativeSolutions->clear();
            *alternativeSolutions = remainingAlternatives;
            //	std::cout << "Number of stored alternatives: " << remainingAlternatives.size() << std::endl;
        }

        //      std::cout << "Number of alternatives after: " << alternativeSolutions->size() << std::endl;


        //Now retain only most reliable solutions if necessary according to the elimination strategy
        //Three strategy are supported : Least Reliable Elimination and Maximun Likelihood Estimator Elimination

        if(N > n){
            if(eliminationStrategy=="LeastReliable"){
                //As now alternatives are ordered by Probability, is not necessary to order them
                if(N > n) {
                    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator asIt2;
                    asIt = (**it)[n];
                    for(asIt2 = asIt, asIt2++; asIt2 != alternativeSolutions->end(); asIt2++)
                        if((*asIt2)->justFirstFrames) //If there is just new elements do not erase (they have P = 0.0 so
                                                      //they will be at the end of the list).
                            break;

                    alternativeSolutions->erase(++asIt, asIt2);
                }
            } else if(eliminationStrategy=="MaximumLikelihoodEstimator"){
                //compute the mean
                double mean = sum / N;
                //compute the variance
                double sigma = 0, pi;
                for(asIt=alternativeSolutions->begin();asIt!=alternativeSolutions->end();asIt++){
                    pi = (*asIt)->getProbability();
                    sigma += (pi - mean)*(pi -mean);
                }
                sigma /= N - 1;
                sigma = std::sqrt(sigma);
                //Eliminate solutions behind mean +/- standard deviation
                for(asIt=alternativeSolutions->begin();asIt!=alternativeSolutions->end();){
                    double pi = (*asIt)->getProbability();
                    //The elements are ordered by probability
                    if(pi<mean-sigma && !(*asIt)->justFirstFrames) {
                        std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator asIt2;
                        for(asIt2 = asIt, asIt2++; asIt2 != alternativeSolutions->end(); asIt2++)
                            if((*asIt2)->justFirstFrames) //If there is just new elements do not erase (they have P = 0.0 so
                                                          //they will be at the end of the list).
                                break;
                        alternativeSolutions->erase(asIt, asIt2);
                        break;
                    } else if ((*asIt)->justFirstFrames)
                        break;
                    else
                        asIt++;
                }
            }   else if(eliminationStrategy != "None") //Bad definition
                std::cout << "Elimination Strategy '" << eliminationStrategy.toStdString() << "' not defined. Skipping alternatives elimination process";
        }
    }
}

void ReliabilityTracker::computeInclusiveProbabilities(std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>* alternativeSolutions,int k,double sum,bool *mark,double *piT) {

    double newSum = sum;
    bool overflow = false;
    int i;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator asIt;
    for(i = 0,asIt=alternativeSolutions->begin();asIt != alternativeSolutions->end();asIt++,i++){
        if(mark[i]) continue;
        piT[i] = (k * (*asIt)->getProbability()) / sum;
        if(piT[i] > 1) {
            overflow = true;
            mark[i] = true;
            piT[i] = 1;
            newSum -= (*asIt)->getProbability();
        }
    }
    if(overflow)
        computeInclusiveProbabilities(alternativeSolutions,k,newSum,mark,piT);
}


void ReliabilityTracker::run(std::vector<Blob>& blobs) {

    std::cout << "Tracker en frame: " << m_data->frameNumber << std::endl;
    std::cout << "Before processing:" << std::endl << rMobilesList;

    setCurrentTimeAndFrame();

    preMerge(blobs);

    setBlobsVector(blobs);

    //TODO: To implement when a good point tracker exists. :D
    //new_blobs = preSplit (new_blobs);

   update();

   std::cout << "After updating:" << std::endl << rMobilesList;


//    if(m_internalOutputActivated)
//        std::cout << "After New Mobiles:" << std::endl << rMobilesList;

/*    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralt_it, alt_end;

    int i, j;
    if(rMobilesList.size()>0) {
        for(rmobile_it = rMobilesList.begin(), j=0; rmobile_it != rMobilesList.end(); rmobile_it++, j++) {
                alt_end = (*rmobile_it)->end();
                for(ralt_it = (*rmobile_it)->begin(), i=0; ralt_it != alt_end; ralt_it++, i++);
                std::cout << "Mobile " << j << " before eliminateUnlikelyMobiles - Alternatives: " << (*rmobile_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/


    eliminateUnlikelyMobiles();

    std::cout << "After eliminateUnlikelyMobiles:" << std::endl << rMobilesList;


/*    if(rMobilesList.size()>0) {
        for(rmobile_it = rMobilesList.begin(), j=0; rmobile_it != rMobilesList.end(); rmobile_it++, j++) {
                alt_end = (*rmobile_it)->end();
                for(ralt_it = (*rmobile_it)->begin(), i=0; ralt_it != alt_end; ralt_it++, i++);
                std::cout << "Mobile " << j << " before filterUnseenMobiles - Alternatives: " << (*rmobile_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/


    filterUnseenMobiles();
    std::cout << "After filterUnseenMobiles:" << std::endl << rMobilesList;

/*    if(rMobilesList.size()>0) {
        for(rmobile_it = rMobilesList.begin(), j=0; rmobile_it != rMobilesList.end(); rmobile_it++, j++) {
                alt_end = (*rmobile_it)->end();
                for(ralt_it = (*rmobile_it)->begin(), i=0; ralt_it != alt_end; ralt_it++, i++);
                std::cout << "Mobile " << j << " before filterEquallyConvergedMobiles - Alternatives: " << (*rmobile_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/
    filterEquallyConvergedMobiles();
    std::cout << "After filterEquallyConvergedMobiles:" << std::endl << rMobilesList;


/*    if(rMobilesList.size()>0) {
        for(rmobile_it = rMobilesList.begin(), j=0; rmobile_it != rMobilesList.end(); rmobile_it++, j++) {
                alt_end = (*rmobile_it)->end();
                for(ralt_it = (*rmobile_it)->begin(), i=0; ralt_it != alt_end; ralt_it++, i++);
                std::cout << "Mobile " << j << " before filterRepeatedAlternatives - Alternatives: " << (*rmobile_it)->size()<< ", Real: " << i << std::endl;
        }
    }
    */

    filterRepeatedAlternatives();
    std::cout << "After filterRepeatedAlternatives:" << std::endl << rMobilesList;


    separateReliableSolutions();
    std::cout << "After separateReliableSolutions:" << std::endl << rMobilesList;

    freeBlobsVector();

//    if(m_internalOutputActivated)
  //      std::cout << "After Elimination:" << std::endl << rMobilesList;

    MobileObject::m_firstFrame = firstFrame = false;
}


bool ReliabilityTracker::equal2DDimensions(SpMobileObject m1, SpMobileObject m2) {
    if(    fabs(m1->t2DDimData.W - m2->t2DDimData.W) <= acceptedPixelError
        && fabs(m1->t2DDimData.H - m2->t2DDimData.H) <= acceptedPixelError
        && fabs(m1->t2DSpatialData.X - m2->t2DSpatialData.X) <= acceptedPixelError
        && fabs(m1->t2DSpatialData.Y - m2->t2DSpatialData.Y) <= acceptedPixelError )
        return true;

    return false;
}

void ReliabilityTracker::filterEquallyConvergedMobiles() {

    SpReliableMobileObject rmobile;
    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralt_it, alt_end;
    std::deque<SpMobileObject>::iterator mobile_it1, mobile_it2, mobiles_end, bestEqual;
    std::deque<std::deque<SpMobileObject>::iterator> equalMobiles;
    std::deque<std::deque<SpMobileObject>::iterator>::iterator equal_it;
    int numAlternatives, numMobiles, i;
    double bestP;
    int maxNumberOfFrames;

    for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
        rmobile = (*rmobile_it);
        alt_end = rmobile->end();
        numAlternatives = rmobile->size();

        for(ralt_it = rmobile->begin(); ralt_it != alt_end; ralt_it++) {
            numMobiles = (*ralt_it)->size();
            if(numMobiles == 1)
                continue;
            //Check if there is equal mobiles in the same alternative
            mobiles_end = (*ralt_it)->end();
            //Initialize equality and deletion flags
            for(mobile_it1 = (*ralt_it)->begin(), i=0; i < numMobiles; mobile_it1++, i++)
                (*mobile_it1)->comparedMobile = (*mobile_it1)->toErase = false;

            //Analyze equal pairs
            for(mobile_it1 = (*ralt_it)->begin(), i=0; i < numMobiles-1; mobile_it1++, i++) {
                if((*mobile_it1)->comparedMobile)
                    continue;
                equalMobiles.clear();
                (*mobile_it1)->comparedMobile = true;
                bestP = (*mobile_it1)->getGlobalProbability();
                maxNumberOfFrames = (*mobile_it1)->getNumberOfFramesSinceFirstTimeSeen();
                bestEqual = mobile_it1;
                equalMobiles.push_back(mobile_it1);
                for(mobile_it2 = mobile_it1, mobile_it2++; mobile_it2 != mobiles_end; mobile_it2++) {
                    if((*mobile_it2)->comparedMobile)
                        continue;

                    //To consider two mobiles equal, first they have to come from the same origin (same rmobile_id)
                    //Then they have also to use the same visual evidence in current frame, or to have a very
                    //high overlapping on their 2D features.
                    if(    (    (*mobile_it1)->getRMobileId() == (*mobile_it2)->getRMobileId()
                             && ( sameUsed(*mobile_it1, *mobile_it2) || highCoverage(*mobile_it1, *mobile_it2) ) )
                        || (    (*mobile_it1)->getRMobileId() != (*mobile_it2)->getRMobileId()
                             && sameUsed(*mobile_it1, *mobile_it2)
                             && highCoverage(*mobile_it1, *mobile_it2)
                             && equal2DDimensions(*mobile_it1, *mobile_it2) ) ) {
                        equalMobiles.push_back(mobile_it2);
                        (*mobile_it2)->comparedMobile = true;
                        if(maxNumberOfFrames < (*mobile_it2)->getNumberOfFramesSinceFirstTimeSeen()) {
                            maxNumberOfFrames = (*mobile_it2)->getNumberOfFramesSinceFirstTimeSeen();
                            bestP = (*mobile_it2)->getGlobalProbability();
                            bestEqual = mobile_it2;
                        } else if(maxNumberOfFrames == (*mobile_it2)->getNumberOfFramesSinceFirstTimeSeen()){
                            if(bestP < (*mobile_it2)->getGlobalProbability()) {
                                bestP = (*mobile_it2)->getGlobalProbability();
                                bestEqual = mobile_it2;
                            }
                        }
                    }
                }

                //Analyzed mobiles was different to all the others
                if(equalMobiles.size() == 1)
                    continue;

                //Mark Mobiles to Eliminate
                for(equal_it = equalMobiles.begin(); equal_it != equalMobiles.end(); equal_it++)
                    if(*equal_it != bestEqual)
                        (**equal_it)->toErase = true;
            }

            //Eliminate marked mobiles
            for(mobile_it1 = (*ralt_it)->begin(); mobile_it1 != (*ralt_it)->end(); ) {
                if( (*mobile_it1)->toErase )
                    mobile_it1 = (*ralt_it)->erase(mobile_it1);
                else
                    mobile_it1++;
            }
        }
    }
}


void ReliabilityTracker::filterRepeatedAlternatives(SpReliableMobileObject rmobile) {

    int numAlternatives = rmobile->size();
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator *alternativesToCompare[numAlternatives],
        ralt_it, rnewalt_it, alt_end = rmobile->end(),
        rsec_it, rnewsec_it;

    bool notCompletelyChecked = true, newFound, moreThanOneToCompare;
    SpRMAlternativeSolution currentOldAlternative, currentNewAlternative;
    int currentFirst, newSize, currentNew, currentSecond, numToCompare, bestIndex;
    double best;
    ralt_it = rmobile->begin();
    currentFirst = 0;

    while(notCompletelyChecked) {

        //First, set the first alternative to compare, not yet compared
        newFound = false;
        for(; ralt_it != alt_end; ralt_it++, currentFirst++) {
            newSize = (*ralt_it)->newAlternatives.size();
            if(newSize > 0 && (*ralt_it)->numSurvived < newSize)
                for(rnewalt_it = (*ralt_it)->newAlternatives.begin(), currentNew = 0; currentNew < newSize; currentNew++, rnewalt_it++)
                    if(!(*rnewalt_it)->survivedToComparison) {
                        newFound = true;
                        break;
                    }
            if(newFound)
                break;
        }

        //Second, to fill the list with equivalent alternatives generated from other alternatives
        if(newFound) {
            alternativesToCompare[currentFirst] = &rnewalt_it;
            bestIndex = currentFirst;
            best = (*rnewalt_it)->getProbability();
            moreThanOneToCompare = false;
            numToCompare = 1;

            rsec_it = ralt_it;
            for(rsec_it++, currentSecond = currentFirst+1; rsec_it != alt_end; rsec_it++, currentSecond++) {
                newSize = (*rsec_it)->newAlternatives.size();
                newFound = false;
                if(newSize > 0 && (*rsec_it)->numSurvived < newSize)
                    for(rnewsec_it = (*rsec_it)->newAlternatives.begin(), currentNew = 0; currentNew < newSize; currentNew++, rnewsec_it++)
                        if(!(*rnewsec_it)->survivedToComparison && equalMobiles(*rnewalt_it, *rnewsec_it)) {
                            newFound = true;
                            moreThanOneToCompare = true;
                            numToCompare++;
                            if((*rnewsec_it)->getProbability() > best) {
                                bestIndex = currentSecond;
                                best = (*rnewsec_it)->getProbability();
                            }
                            alternativesToCompare[currentSecond] = &rnewsec_it;
                            break;
                        }

                if(newFound == false)
                    alternativesToCompare[currentSecond] = NULL;
            }

            if(moreThanOneToCompare) {
                //Erase the not best elements
                rsec_it = ralt_it;
                for(currentSecond = currentFirst; rsec_it != alt_end; rsec_it++, currentSecond++)
                    if(currentSecond != bestIndex && alternativesToCompare[currentSecond] != NULL)
                        (*rsec_it)->newAlternatives.erase(*alternativesToCompare[currentSecond]);
                    else if(currentSecond == bestIndex) { //Save the best element
                        (**alternativesToCompare[currentSecond])->survivedToComparison = true;
                        (*rsec_it)->numSurvived++;
                    }
            } else {
                (*rnewalt_it)->survivedToComparison = true;
                (*ralt_it)->numSurvived++;
            }
        } else
            notCompletelyChecked = false;
    }

}


void ReliabilityTracker::filterRepeatedAlternatives() {

    SpReliableMobileObject rmobile;
    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralt_it1, ralt_it2, bestEqual, alt_end;
    std::deque<std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator> equalAlternatives;
    std::deque<std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator>::iterator equal_it;
    int numAlternatives, i, j;
    double bestP;
    bool changed;
    j=-1;
    for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
        rmobile = *rmobile_it;
        j++;
        if((numAlternatives = rmobile->size()) == 1)
            continue;
        //std::cout << "Alternatives mobile " << j << ": " << numAlternatives << std::endl;
        alt_end = rmobile->end();

        for(ralt_it1 = (*rmobile_it)->begin(), i=0; ralt_it1 != alt_end; ralt_it1++, i++);
        //std::cout << "In filterRepeatedAlternatives - Real Alternatives: " << i << std::endl;

        changed = false;

        for(ralt_it1 = rmobile->begin(); ralt_it1 != alt_end; ralt_it1++)
            (*ralt_it1)->toEliminate = (*ralt_it1)->comparedAlternative = false;

        for(ralt_it1 = rmobile->begin(), i=0; i < numAlternatives-1; ralt_it1++, i++) {
            if((*ralt_it1)->comparedAlternative || (*ralt_it1)->toEliminate)
                continue;
            equalAlternatives.clear();
            (*ralt_it1)->comparedAlternative = true;
            bestP = (*ralt_it1)->getProbability();
            bestEqual = ralt_it1;
            equalAlternatives.push_back(ralt_it1);
            for(ralt_it2 = ralt_it1, ralt_it2++; ralt_it2 != alt_end; ralt_it2++) {
                if((*ralt_it2)->comparedAlternative || (*ralt_it1)->toEliminate)
                    continue;
                if(equalMobiles(*ralt_it1, *ralt_it2)) {
                    equalAlternatives.push_back(ralt_it2);
                    (*ralt_it2)->comparedAlternative = true;
                    if(bestP < (*ralt_it2)->getProbability()) {
                        bestP = (*ralt_it2)->getProbability();
                        bestEqual = ralt_it2;
                    }
                }
            }

            //Analyzed alternative was different to all the others
            if(equalAlternatives.size() == 1)
                continue;

            //Mark Alternatives to Eliminate
            for(equal_it = equalAlternatives.begin(); equal_it != equalAlternatives.end(); equal_it++)
                if(*equal_it != bestEqual) {
                    (*ralt_it1)->toEliminate = true;
                    changed = true;
                }
        }

        //Eliminate marked alternatives
        if(changed) {
            std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> newAlternatives;
            for(ralt_it1 = rmobile->begin(), i=0; i < numAlternatives; ralt_it1++, i++)
                if((*ralt_it1)->toEliminate == false)
                    newAlternatives.insert(*ralt_it1);

            rmobile->alternativeSolutions.clear();
            rmobile->alternativeSolutions = newAlternatives;
        }
    }
}

bool ReliabilityTracker::equalMobiles(SpRMAlternativeSolution alt1, SpRMAlternativeSolution alt2) {

    //If different number of mobiles they are different
    if(alt1->size() != alt2->size())
        return false;

    std::deque<SpMobileObject>::iterator m1_it, m2_it;
    int i, j, size = alt1->size();
    bool used_alt1[size], used_alt2[size];
    unsigned long id;
    bool found;

    memset(used_alt1, false, size*sizeof(bool));
    memset(used_alt2, false, size*sizeof(bool));

    for(m1_it = alt1->begin(), i = 0; i < size; i++, m1_it++) {
        if(used_alt1[i] == false) {
            id = (*m1_it)->getMobileId();
            found = false;

            for(m2_it = alt2->begin(), j = 0; j < size; j++, m2_it++) {
                if(used_alt2[j] == false) {
                    if(id == (*m2_it)->getMobileId()) { //Id found in second mobiles list
                        if(!sameUsed(*m1_it, *m2_it) && !highCoverage(*m1_it, *m2_it)) //If a pair of mobiles with same id use
                            //different blobs and their coverage is low they are different
                            return false;
                        found = true; //Match found with same id
                        used_alt1[i] = used_alt2[j] = true;
                        break;
                    }
                }
            }

            if(found == false) { //Id not found in second mobiles list, search for matches regardless ids
                for(m2_it = alt2->begin(), j = 0; j < size; j++, m2_it++) {
                    if(used_alt2[j] == false) {
                        if(sameUsed(*m1_it, *m2_it) || highCoverage(*m1_it, *m2_it)) {
                            found = true; //Now found in the sense of matching, regardless the id
                            used_alt1[i] = used_alt2[j] = true;
                            break;
                        }
                    }
                }
            }

            if(found == false) //No match found for current mobile in second alternative
                return false;
        }
    }


    /* Old version for equal ids
       for(m1_it = alt1->begin(), i = 0; i < size; i++, m1_it++) {
       id = (*m1_it)->getMobileId();
       found = false;

       for(m2_it = alt2->begin(), j = 0; j < size; j++, m2_it++) {
       if(id == (*m2_it)->getMobileId()) { //Id found in second mobiles list
       found = true;
       if(!sameUsed(*m1_it, *m2_it) && !highCoverage(*m1_it, *m2_it)) //If a pair of mobiles with same id use
       //different blobs and their coverage is low they are different
       return false;
       }
       }

       if(found == false) //Id not found in second mobiles list
       return false;

       } */

    return true;

}

bool ReliabilityTracker::highCoverage(SpMobileObject m1, SpMobileObject m2) {
    Rectangle<int> bb1, bb2;
    bb1.initRectangle((int)m1->t2DSpatialData.X, (int)m1->t2DSpatialData.Y, (int)m1->t2DDimData.W, (int)m1->t2DDimData.H);
    bb2.initRectangle((int)m2->t2DSpatialData.X, (int)m2->t2DSpatialData.Y, (int)m2->t2DDimData.W, (int)m2->t2DDimData.H);

    if(Rectangle<int>::rectangleIntersectRatio(&bb2, &bb1) < m_mobile2DCoverageRateToConsiderEqual)
        return false;

    if(Rectangle<int>::rectangleIntersectRatio(&bb1, &bb2) < m_mobile2DCoverageRateToConsiderEqual)
        return false;

    return true;
}


bool ReliabilityTracker::sameUsed(SpMobileObject m1, SpMobileObject m2) {
    if(m1->numUsed != m2->numUsed)
        return false;

    if( (m1->currentVisualState & MM_PART_OF_BIGGER) || (m2->currentVisualState & MM_PART_OF_BIGGER) )
        return false;

    if(memcmp(m1->usedBlobs, m2->usedBlobs, sizeof(bool)*blobsNumber) == 0)
        return true;

    return false;
}

void ReliabilityTracker::filterUnseenMobiles() {

    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator>::iterator ralternative_it;
    std::deque<SpMobileObject>::iterator mobile_it;
    unsigned int numToErase;
    bool alternativesToErase;
    if(!rMobilesList.empty()) {

        for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end();) {
            alternativesToErase = false;
            numToErase = 0;
            for(ralternative_it = (*rmobile_it)->begin(); ralternative_it != (*rmobile_it)->end(); ralternative_it++) {
                for(mobile_it = (*ralternative_it)->begin(); mobile_it != (*ralternative_it)->end(); ) {
                    if((*mobile_it)->mobileOutOfScene()) {
                        std::map<long int, SpMobileObject>::iterator previous;
                        if( (previous = mobilesOutList.find((*mobile_it)->getMobileId())) != mobilesOutList.end() ) {
                            if((*previous).second->getGlobalProbability() < (*mobile_it)->getGlobalProbability())
                                mobilesOutList[(*mobile_it)->getMobileId()] = *mobile_it;
                        } else {
                            mobilesOutList[(*mobile_it)->getMobileId()] = *mobile_it;
                            //		std::cout << "Mobile " << (*mobile_it)->getMobileId() << " stored in list of Mobiles out of scene." << std::endl;
                        }
                        mobile_it = (*ralternative_it)->erase(mobile_it);
                    } else if(    (    (*mobile_it)->getNumberOfFramesNotSeen() >= std::max(MobileObject::m_blobsBufferSize, (*mobile_it)->getNumberOfFramesSinceFirstTimeSeen() - (*mobile_it)->getNumberOfFramesNotSeen())
                                    && (*mobile_it)->getBestType() == UNKNOWN )
                                  //		       || (    (*mobile_it)->getMobileId() == 6 )
                               || (    (*mobile_it)->getNumberOfFramesSinceFirstTimeSeen() < m_BlobBufferSize
                                    && (*mobile_it)->currentVisualState & (MM_OBJECT_LOST | MM_NOT_VISIBLE_MASK | MM_NOT_3D_TRACKABLE_MASK) )
                               || (    (*mobile_it)->getNumberOfFramesSinceFirstTimeSeen() < 2*m_BlobBufferSize
                                    && (*mobile_it)->getNumberOfFramesNotSeen() >= MobileObject::m_blobsBufferSize )
                               || (    (*mobile_it)->getNumberOfFramesNotSeen() >= 10*MobileObject::m_blobsBufferSize ) )
                        mobile_it = (*ralternative_it)->erase(mobile_it);
                    else
                        mobile_it++;
                }

                (*ralternative_it)->toEliminate = false;
                if((*ralternative_it)->size() == 0) {
                    (*ralternative_it)->toEliminate = alternativesToErase = true;
                    numToErase++;
                }
            }

            if(numToErase == (*rmobile_it)->size())
                rmobile_it = rMobilesList.erase(rmobile_it);
            else if (numToErase > 0) {
                //Erase alternatives to erase
                std::set<SpRMAlternativeSolution, alternativesOrderedByBestProbabilityOperator> newAlternatives;
                for(ralternative_it = (*rmobile_it)->begin(); ralternative_it != (*rmobile_it)->end(); ralternative_it++)
                    if((*ralternative_it)->toEliminate == false)
                        newAlternatives.insert(*ralternative_it);
                (*rmobile_it)->alternativeSolutions.clear();
                (*rmobile_it)->alternativeSolutions = newAlternatives;
                rmobile_it++;
            } else
                rmobile_it++;

        }
    }
}


void ReliabilityTracker::separateReliableSolutions() {

    std::deque<SpReliableMobileObject>::iterator rmobile_it;
    std::deque<SpReliableMobileObject> newRMobiles;
    SpRMAlternativeSolution ralternative;
    std::deque<SpMobileObject>::iterator mobile_it;

    if(!rMobilesList.empty()) {
        for(rmobile_it = rMobilesList.begin(); rmobile_it != rMobilesList.end(); rmobile_it++) {
            if((*rmobile_it)->size() == 1) { //Just one alternative
                ralternative = (*(*rmobile_it)->begin());
                if(ralternative->size() > 1) { //More than one mobile
                    for(mobile_it = ralternative->begin() + 1; mobile_it != ralternative->end(); mobile_it++) {
                        SpRMAlternativeSolution newAlternative(new RMAlternativeSolution());
                        newAlternative->insertNewMobileObject(*mobile_it);
                        newAlternative->setAlternativeProbability();

                        SpReliableMobileObject newRMobile(new ReliableMobileObject());
                        newRMobile->insert(newAlternative);
                        newRMobile->setBestSolution();

                        newRMobiles.push_front(newRMobile);
                    }
                    ralternative->erase(ralternative->begin() + 1, ralternative->end());
                }
            }
        }

        if(newRMobiles.size() > 0) {
            rMobilesList.insert(rMobilesList.end(), newRMobiles.begin(), newRMobiles.end());
            newRMobiles.clear();
        }
    }
}


void ReliabilityTracker::presetBlobsVectorAndInitialMergeTable(std::vector<Blob>& blobs) {
    int i;

    blobsVector = NULL;
    initialMergeMap = NULL;

    blobsNumber = blobs.size();

    if(blobsNumber == 0)
        return;

    blobsVector = new Blob*[blobsNumber];
    initialMergeMap = new bool*[blobsNumber];

    std::vector<Blob>::iterator it, it_end = blobs.end();
    for(i = 0, it = blobs.begin(); it != it_end; it++, i++) {
        blobsVector[i] = &(*it);
        initialMergeMap[i] = new bool[blobsNumber];
        memset(initialMergeMap[i], false, sizeof(bool)*blobsNumber);
    }

    m_RMerge->setInitialCorrespondences(initialMergeMap, blobsNumber, blobsVector);

}

void ReliabilityTracker::freeBlobsVectorAndInitialMergeTable() {

    if(blobsVector)
        delete[] blobsVector;

    if(initialMergeMap) {
        for(int i = 0; i < blobsNumber; i++)
            delete[] initialMergeMap[i];

        delete[] initialMergeMap;
    }
}


void ReliabilityTracker::setBlobsVector(std::vector<Blob>& blobs) {
    int j;
    int *elementsToAnalyzeVector;

    blobsNumber = blobs.size();

    //Static blobs number for tracking setting, for concerned classes
    ReliableMobileObject::m_currentTrackingBlobsNumber = blobsNumber;
    RMAlternativeSolution::m_currentTrackingBlobsNumber = blobsNumber;
    MobileObject::m_currentTrackingBlobsNumber = blobsNumber;

    blobsVector = NULL;
    usedBlobs = NULL;
    initialMergeMap = NULL;
    initialGroups =  NULL;
    elementsToAnalyzeVector =  NULL;
    involvedRMobilesCounter = NULL;
    m_RMerge->clearDefinedMerges();

    if(blobsNumber == 0)
        return;

    int i;
    blobsVector = new Blob *[blobsNumber];
    usedBlobs = new bool[blobsNumber];
    initialMergeMap = new bool*[blobsNumber];
    initialGroups = new int[blobsNumber];
    elementsToAnalyzeVector = new int[blobsNumber];
    involvedRMobilesCounter = new int[blobsNumber];

    memset(usedBlobs, false, sizeof(bool)*blobsNumber);
    memset(involvedRMobilesCounter, 0, sizeof(int)*blobsNumber);

    for(i = 0; i < blobsNumber; i++)
        elementsToAnalyzeVector[i] = i;

    std::vector<Blob>::iterator it, it_end = blobs.end();
    for(i = 0, it = blobs.begin(); it != it_end; it++, i++) {
        blobsVector[i] = &(*it);
        BLOB_POSITION(blobsVector[i]) = blobsVector[i]->getPositionRelativeToCamera(smodel);
        BLOB_FRAME_NUMBER(blobsVector[i]) = currentFrameNumber;
        BLOB_TIME_DIFF_MSEC(blobsVector[i]) = lastMilliSecondsDifference;

        initialMergeMap[i] = new bool[blobsNumber];
        memset(initialMergeMap[i], false, sizeof(bool)*blobsNumber);
    }

    m_RMerge->setInitialCorrespondences(initialMergeMap, blobsNumber, blobsVector);

    setGroups(initialGroups, initialMergeMap, blobsNumber, elementsToAnalyzeVector);

    if(elementsToAnalyzeVector)
        delete[] elementsToAnalyzeVector;

    if(m_internalOutputActivated) {

        std::cout << "Blobs Vector <-> Index:\n\t";
        for(i = 0; i < blobsNumber; i++)
            std::cout << i << "\t";
        std::cout << "\n\t";
        for(i = 0; i < blobsNumber; i++)
            std::cout << BLOB_IDENT(blobsVector[i]) << "\t";

        std::cout << "\nInitial Merge Table\n\t";
        for(i = 0; i < blobsNumber; i++)
            std::cout << i << "\t";
        for(i = 0; i < blobsNumber; i++) {
            std::cout << std::endl << i << "\t";
            for(j = 0; j < blobsNumber; j++)
                std::cout << initialMergeMap[i][j] << "\t";
        }
        std::cout << std::endl;
        std::cout << "\nRelations list\n\t";
        for(i = 0; i < blobsNumber; i++)
            std::cout << i << "\t";
        std::cout << "\n\t";
        for(i = 0; i < blobsNumber; i++)
            std::cout << initialGroups[i] << "\t";
        std::cout << std::endl;
    }
}

void ReliabilityTracker::freeBlobsVector() {

    if(blobsVector)
        delete[] blobsVector;
    if(usedBlobs)
        delete[] usedBlobs;
    if(initialMergeMap) {
        for(int i = 0; i < blobsNumber; i++)
            delete[] initialMergeMap[i];

        delete[] initialMergeMap;
    }

    if(initialGroups)
        delete[] initialGroups;

    if(involvedRMobilesCounter)
        delete[] involvedRMobilesCounter;

    m_RMerge->clearDefinedMerges();
}

bool ReliabilityTracker::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined

        AppendToLog("ReliabilityTrackingModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry ReliabilityTrackingModule. Taking defaults...");
        m_RMerge->defaultParameters(ReliabilityMerge::MergeInTracking);
        m_rclassif->defaultParameters();
        activatePreMerge = false;
        m_BlobBufferSize = 4;

        MobileObject::m_blobsBufferSize = m_BlobBufferSize;
        MobileObject::secDiffSequence = new double[m_BlobBufferSize];
        MobileObject::coolingValue    = new double[m_BlobBufferSize];
        MobileObject::secDiffToCurrent = new double[m_BlobBufferSize];

        memset(MobileObject::secDiffSequence,  0, sizeof(double)*m_BlobBufferSize);
        memset(MobileObject::coolingValue,     0, sizeof(double)*m_BlobBufferSize);
        memset(MobileObject::secDiffToCurrent, 0, sizeof(double)*m_BlobBufferSize);

        MobileObject::coolingValue[0]     = 1.0;

        MobileObject::m_trajectoryMaxSize = 0;
        m_maximumRetainedAlternativeSolutions = 10;
        m_maximumGeneratedAlternativesPerMobile = 4;
        maximumAlternativeMobilePaths = 3;
        MobileObject::m_2DLevelFrames = 3;
        eliminationStrategy = "LeastReliable";
        MobileObject::m_lambda = 5.0;
        SpatialCoherenceReliabilityThreshold = MobileObject::m_SpatialCoherenceReliabilityThreshold = 0.1;
        SpatialCoherenceProbabilityThreshold = MobileObject::m_SpatialCoherenceProbabilityThreshold = 0.5;
        MobileObject::m_DimensionalCoherenceReliabilityThreshold = 0.1;
        MobileObject::m_DimensionalCoherenceProbabilityThreshold = 0.5;
        MobileObject::m_knownSolutionThreshold = 0.5;
        alternativeSolutionsProbabilityThreshold = 0.8;
        IgnoreByDimensionalCoherenceThreshold = 0.1;
        IgnoreByVelocityCoherenceThreshold = 0.1;
        ImportanceRateForBestMobileAlternativeProbability = 0.9;
        MobileObject::m_probabilityToEnsureMode = 0.9;
        m_maximal3DBaseOverlapping = 0.3;
        m_mobile2DCoverageRateToConsiderEqual = 0.95;
        m_highVisualSupportThreshold = 0.95;
        m_lowVisualSupportThreshold = 0.05;
        m_blobCompletellySupportedThreshold = 0.95;
        m_internalOutputActivated = true;
        m_reducedOutputActivated = false;
        MobileObject::m_maximalAlphaRotationSpeed = 2*1.755; //Twice the empirical maximal rotation speed of a person 1.755[rad/sec]
        m_meanMillisecondsDifferenceBetweenFrames = 80;
        MobileObject::m_Maximal3DDimensionChangeSpeed = 244.0;
        acceptedPixelError = 2;
        accepted3DFeatureError = 10.0; //[cm]
        acceptedOrientationError = 0.1745329252; //10[deg]
        maxObjectSpeed = 200.0; //[cm/sec]

        return true;
    }

    //Check each parameter
    if( ( n = XmlCommon::getParameterNode("Merge", config) ).isNull() )  {
        AppendToLog("ReliabilityTracker Warning: 'Merge' not found. Taking default parameters for Merge in Tracking.\n");
        m_RMerge->defaultParameters(ReliabilityMerge::MergeInTracking);
    } else {
        if(!m_RMerge->setParameters(n, ReliabilityMerge::MergeInTracking)) {
            AppendToLog("ReliabilityTracker: error: In 'Merge': Error in parameters for Merge in Tracking.\n");
            return false;
        }
    }

    if( ( n = XmlCommon::getParameterNode("PreMerge", config) ).isNull() )  {
        AppendToLog("ReliabilityTracker Warning: 'PreMerge' not found. Taking default parameters for PreMerge in Tracking.\n");
        activatePreMerge = false;
    } else {
        activatePreMerge = XmlCommon::getParameterValue(n) == "true" ? true : false;
        if(activatePreMerge) {
            if(!m_PreMerge->setParameters(n, ReliabilityMerge::PreMergeInTracking)) {
                AppendToLog("ReliabilityTracker: error: In 'PreMerge': Error in parameters for PreMerge in Tracking.\n");
                return false;
            }
        }
    }

    if( ( n = XmlCommon::getParameterNode("Classification", config) ).isNull() )  {
        AppendToLog("ReliabilityTracker Warning: 'Classification' not found. Taking default parameters for Classification.\n");
        m_rclassif->defaultParameters();
    } else {
        m_rclassif->setParameters(n);
        if(!m_rclassif->setParameters(n)) {
            AppendToLog("ReliabilityTracker: error: In 'Classification': Error in parameters for Classification in Tracking.\n");
            return false;
        }
    }


    if( ( n = XmlCommon::getParameterNode("BlobBufferSize", config) ).isNull() ) {
        m_BlobBufferSize = 4;
        AppendToLog("ReliabilityTracker Warning: 'BlobBufferSize' not defined. Taking Default (4).\n");
    } else {
        m_BlobBufferSize = XmlCommon::getParameterValue(n).toInt();
        if(m_BlobBufferSize <= 0) {
            AppendToLog("ReliabilityTracker Error: 'BlobBufferSize' value must be a positive integer > 1. Taking Default (5).\n");
            m_BlobBufferSize = 4;
        }
    }

    MobileObject::m_blobsBufferSize = m_BlobBufferSize;
    MobileObject::secDiffSequence = new double[m_BlobBufferSize];
    MobileObject::coolingValue    = new double[m_BlobBufferSize];
    MobileObject::secDiffToCurrent = new double[m_BlobBufferSize];

    memset(MobileObject::secDiffSequence,  0, sizeof(double)*m_BlobBufferSize);
    memset(MobileObject::coolingValue,     0, sizeof(double)*m_BlobBufferSize);
    memset(MobileObject::secDiffToCurrent, 0, sizeof(double)*m_BlobBufferSize);

    MobileObject::coolingValue[0]     = 1.0;

    if( ( n = XmlCommon::getParameterNode("TrajectoryMaxSize", config) ).isNull() ) {
        MobileObject::m_trajectoryMaxSize = 0;
        AppendToLog("ReliabilityTracker Warning: 'TrajectoryMaxSize' not defined. Taking Default 0 (No Max Limit).\n");
    } else {
        MobileObject::m_trajectoryMaxSize = XmlCommon::getParameterValue(n).toInt();

        if(MobileObject::m_trajectoryMaxSize < 0) {
            AppendToLog("ReliabilityTracker Error: 'TrajectoryMaxSize' value must be an integer >= 0. Taking Default 0 (No Max Limit).\n");
            MobileObject::m_trajectoryMaxSize = 0;
        }
    }

    if( ( n = XmlCommon::getParameterNode("MaximumRetainedAlternativeSolutions", config) ).isNull() ) {
        m_maximumRetainedAlternativeSolutions = 10;
        AppendToLog("ReliabilityTracker Warning: 'MaximumRetainedAlternativeSolutions' not defined. Taking Default (10).\n");
    } else {
        m_maximumRetainedAlternativeSolutions = XmlCommon::getParameterValue(n).toInt();
    }

    if( ( n = XmlCommon::getParameterNode("MaximalNumberOfPathAlternativesPerMobile", config) ).isNull() ) {
        m_maximumGeneratedAlternativesPerMobile = 4;
        AppendToLog("ReliabilityTracker Warning: 'MaximalNumberOfPathAlternativesPerMobile' not defined. Taking Default (4).\n");
    } else {
        if( (m_maximumGeneratedAlternativesPerMobile = XmlCommon::getParameterValue(n).toInt()) < 1 ) {
            m_maximumGeneratedAlternativesPerMobile = 4;
            AppendToLog("ReliabilityTracker Warning: 'MaximalNumberOfPathAlternativesPerMobile' must be an integer higher than 0. Taking Default (4).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("MaximumAlternativeMobilePaths", config) ).isNull() ) {
        maximumAlternativeMobilePaths = 3;
        AppendToLog("ReliabilityTracker Warning: 'MaximumAlternativeMobilePaths' not defined. Taking Default (3).\n");
    } else {
        if( (maximumAlternativeMobilePaths = XmlCommon::getParameterValue(n).toInt()) < 1) {
            maximumAlternativeMobilePaths = 3;
            AppendToLog("ReliabilityTracker Warning: 'MaximumAlternativeMobilePaths' must be an integer higher than 0. Taking Default (3).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("_2DLevelFrames", config) ).isNull() ) {
        MobileObject::m_2DLevelFrames = 3;
        AppendToLog("ReliabilityTracker Warning: '_2DLevelFrames' not defined. Taking Default (3).\n");
    } else {
        if( (MobileObject::m_2DLevelFrames = XmlCommon::getParameterValue(n).toInt()) < 1 ) {
            MobileObject::m_2DLevelFrames = 3;
            AppendToLog("ReliabilityTracker Warning: '_2DLevelFrames' must be an integer higher than 0. Taking Default (3).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("EliminationStrategy", config) ).isNull() ) {
        eliminationStrategy = "LeastReliable";
        AppendToLog("ReliabilityTracker Warning: 'EliminationStrategy' not defined. Taking Default (LeastReliable).\n");
    } else {
        eliminationStrategy = XmlCommon::getParameterValue(n);
    }

    if( ( n = XmlCommon::getParameterNode("CoolingFunctionLambda", config) ).isNull() ) {
        MobileObject::m_lambda = 5.0;
        AppendToLog("ReliabilityTracker Warning: 'CoolingFunctionLambda' not defined. Taking Default (5.0).\n");
    } else
        MobileObject::m_lambda = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("SpatialCoherenceReliabilityThreshold", config) ).isNull() ) {
        SpatialCoherenceReliabilityThreshold = MobileObject::m_SpatialCoherenceReliabilityThreshold = 0.1;
        AppendToLog("ReliabilityTracker Warning: 'SpatialCoherenceReliabilityThreshold' not defined. Taking Default (0.1).\n");
    } else
        MobileObject::m_SpatialCoherenceReliabilityThreshold = SpatialCoherenceReliabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("SpatialCoherenceProbabilityThreshold", config) ).isNull() ) {
        SpatialCoherenceProbabilityThreshold = MobileObject::m_SpatialCoherenceProbabilityThreshold = 0.5;
        AppendToLog("ReliabilityTracker Warning: 'SpatialCoherenceProbabilityThreshold' not defined. Taking Default (0.5).\n");
    } else
        SpatialCoherenceProbabilityThreshold = MobileObject::m_SpatialCoherenceProbabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("DimensionalCoherenceReliabilityThreshold", config) ).isNull() ) {
        MobileObject::m_DimensionalCoherenceReliabilityThreshold = 0.1;
        AppendToLog("ReliabilityTracker Warning: 'DimensionalCoherenceReliabilityThreshold' not defined. Taking Default (0.1).\n");
    } else
        MobileObject::m_DimensionalCoherenceReliabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("DimensionalCoherenceProbabilityThreshold", config) ).isNull() ) {
        MobileObject::m_DimensionalCoherenceProbabilityThreshold = 0.5;
        AppendToLog("ReliabilityTracker Warning: 'DimensionalCoherenceProbabilityThreshold' not defined. Taking Default (0.5).\n");
    } else
        MobileObject::m_DimensionalCoherenceProbabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("KnownSolutionThreshold", config) ).isNull() ) {
        MobileObject::m_knownSolutionThreshold = 0.5;
        AppendToLog("ReliabilityTracker Warning: 'KnownSolutionThreshold' not defined. Taking Default (0.5).\n");
    } else
        MobileObject::m_knownSolutionThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("AlternativeSolutionsProbabilityThreshold", config) ).isNull() ) {
        alternativeSolutionsProbabilityThreshold = 0.8;
        AppendToLog("ReliabilityTracker Warning: 'AlternativeSolutionsProbabilityThreshold' not defined. Taking Default (0.8).\n");
    } else
        alternativeSolutionsProbabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("IgnoreByDimensionalCoherenceThreshold", config) ).isNull() ) {
        IgnoreByDimensionalCoherenceThreshold = 0.1;
        AppendToLog("ReliabilityTracker Warning: 'IgnoreByDimensionalCoherenceThreshold' not defined. Taking Default (0.1).\n");
    } else
        IgnoreByDimensionalCoherenceThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("IgnoreByVelocityCoherenceThreshold", config) ).isNull() ) {
        IgnoreByVelocityCoherenceThreshold = 0.1;
        AppendToLog("ReliabilityTracker Warning: 'IgnoreByVelocityCoherenceThreshold' not defined. Taking Default (0.1).\n");
    } else
        IgnoreByVelocityCoherenceThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("ImportanceRateForBestMobileAlternativeProbability", config) ).isNull() ) {
        ImportanceRateForBestMobileAlternativeProbability = 0.9;
        AppendToLog("ReliabilityTracker Warning: 'ImportanceRateForBestMobileAlternativeProbability' not defined. Taking Default (0.9).\n");
    } else
        ImportanceRateForBestMobileAlternativeProbability = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("ProbabilityToEnsureMode", config) ).isNull() ) {
        MobileObject::m_probabilityToEnsureMode = 0.9;
        AppendToLog("ReliabilityTracker Warning: 'ProbabilityToEnsureMode' not defined. Taking Default (0.9).\n");
    } else
        MobileObject::m_probabilityToEnsureMode = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Maximal3DBaseOverlapping", config) ).isNull() ) {
        m_maximal3DBaseOverlapping = 0.3;
        AppendToLog("ReliabilityTracker Warning: 'Maximal3DBaseOverlapping' not defined. Taking Default (0.3).\n");
    } else
        m_maximal3DBaseOverlapping = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Mobile2DCoverageRateToConsiderEqual", config) ).isNull() ) {
        m_mobile2DCoverageRateToConsiderEqual = 0.95;
        AppendToLog("ReliabilityTracker Warning: 'Mobile2DCoverageRateToConsiderEqual' not defined. Taking Default (0.95).\n");
    } else
        m_mobile2DCoverageRateToConsiderEqual = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("HighVisualSupportThreshold", config) ).isNull() ) {
        m_highVisualSupportThreshold = 0.95;
        AppendToLog("ReliabilityTracker Warning: 'HighVisualSupportThreshold' not defined. Taking Default (0.85).\n");
    } else
        m_highVisualSupportThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("LowVisualSupportThreshold", config) ).isNull() ) {
        m_lowVisualSupportThreshold = 0.05;
        AppendToLog("ReliabilityTracker Warning: 'LowVisualSupportThreshold' not defined. Taking Default (0.15).\n");
    } else
        m_lowVisualSupportThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("BlobCompletellySupportedThreshold", config) ).isNull() ) {
        m_blobCompletellySupportedThreshold = 0.95;
        AppendToLog("ReliabilityTracker Warning: 'BlobCompletellySupportedThreshold' not defined. Taking Default (0.95).\n");
    } else
        m_blobCompletellySupportedThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("InternalOutputActivated", config) ).isNull() ) {
        m_internalOutputActivated = true;
        AppendToLog("ReliabilityTracker Warning: 'InternalOutputActivated' not defined. Taking Default (true).\n");
    } else
        m_internalOutputActivated = XmlCommon::getParameterValue(n) == "true" ? true : false;

    if( ( n = XmlCommon::getParameterNode("ReducedOutputActivated", config) ).isNull() ) {
        m_reducedOutputActivated = false;
        AppendToLog("ReliabilityTracker Warning: 'ReducedOutputActivated' not defined. Taking Default (false).\n");
    } else
        m_reducedOutputActivated = XmlCommon::getParameterValue(n) == "true" ? true : false;

    if( ( n = XmlCommon::getParameterNode("MaximalAlphaRotationSpeed", config) ).isNull() ) {
        MobileObject::m_maximalAlphaRotationSpeed = 2*1.755; //Twice the empirical maximal rotation speed of a person 1.755[rad/sec]
        AppendToLog("ReliabilityTracker Warning: 'MaximalAlphaRotationSpeed' not defined. Taking Default 2*1.755[rad]/[sec]. Where 1.755[rad]/[sec] corresponds to the empirical calculation of the rotation speed of a person\n");
    } else
        MobileObject::m_maximalAlphaRotationSpeed = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("MeanMillisecondsDifferenceBetweenFrames", config) ).isNull() ) {
        m_meanMillisecondsDifferenceBetweenFrames = 80;
        AppendToLog("ReliabilityTracker Warning: 'MeanMillisecondsDifferenceBetweenFrames' not defined. Taking Default (80).\n");
    } else {
        if( (m_meanMillisecondsDifferenceBetweenFrames = XmlCommon::getParameterValue(n).toInt()) <= 0) {
            m_meanMillisecondsDifferenceBetweenFrames = 80;
            AppendToLog("ReliabilityTracker Warning: 'MeanMillisecondsDifferenceBetweenFrames' must be an integer higher than 0. Taking Default (80).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("Maximal3DDimensionChangeSpeed", config) ).isNull() ) {
        MobileObject::m_Maximal3DDimensionChangeSpeed = 244.0;
        AppendToLog("ReliabilityTracker Warning: 'Maximal3DDimensionChangeSpeed' not defined. Taking Default (244.0 [cm/s]).\n");
    } else
        MobileObject::m_Maximal3DDimensionChangeSpeed = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("AcceptedPixelError", config) ).isNull() ) {
        acceptedPixelError = 2;
        AppendToLog("ReliabilityTracker Warning: 'AcceptedPixelError' not defined. Taking Default (2).\n");
    } else
        acceptedPixelError = XmlCommon::getParameterValue(n).toInt();

    if( ( n = XmlCommon::getParameterNode("Accepted3DFeatureError", config) ).isNull() ) {
        accepted3DFeatureError = 10.0; //[cm]
        AppendToLog("ReliabilityTracker Warning: 'Accepted3DFeatureError' not defined. Taking Default (10[cm]).\n");
    } else
        accepted3DFeatureError = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("AcceptedOrientationError", config) ).isNull() ) {
        acceptedOrientationError = 0.1745329252; //10[deg]
        AppendToLog("ReliabilityTracker Warning: 'AcceptedOrientationError' not defined. Taking Default (0.1745329252[rad] (10[deg]) ).\n");
    } else
        acceptedOrientationError = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("MaxObjectSpeed", config) ).isNull() ) {
         maxObjectSpeed = 200.0; //[cm/sec]
        AppendToLog("ReliabilityTracker Warning: 'MaxObjectSpeed' not defined. Taking Default (200[cm/sec]).\n");
    } else
        maxObjectSpeed = XmlCommon::getParameterValue(n).toDouble();


    return true;
}


void ReliabilityTracker::setGroup(int groupsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector) {
    int i;

    groupsVector[referencePoint] = groupId;

    for(i=startPoint; i<elementsNumber; i++)
        if(relations[elementsToAnalyzeVector[referencePoint]][elementsToAnalyzeVector[i]] && groupsVector[i] < 0)
            setGroup(groupsVector, relations, elementsNumber, groupId, startPoint, i, elementsToAnalyzeVector);

}

int ReliabilityTracker::setGroups(int groupsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector) {
    int groupId = -1;
    int i, j;

    for(i=0; i<elementsNumber; i++)
        groupsVector[i] = -1;

    for(i=0; i<elementsNumber; i++) {
        if(groupsVector[i] < 0) {
            groupId++;
            groupsVector[i] = groupId;
            for(j=i+1; j<elementsNumber; j++) {
                if(relations[elementsToAnalyzeVector[i]][elementsToAnalyzeVector[j]] && groupsVector[j] < 0)
                    setGroup(groupsVector, relations, elementsNumber, groupId, i+1, j, elementsToAnalyzeVector);
            }
        }
    }

    return groupId + 1;
}


