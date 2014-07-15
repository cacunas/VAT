//#include "ReliabilityClassification.h"
#include <cmath>
#include <deque>
#include <QMessageBox>
#include <QSharedPointer>
#include <QTextStream>

#include "RMMTracker.h"
#include "VideoAnalysis.h"
#include "src/ImageHeader.h"
#include "src/geometric.h"
#include "src/blob.h"
#include "MODELS/Blob2DFromBGSubstractionModel.h"

int RMMTracker::m_hypothesisNumber = 0;
int RMMTracker::acceptedPixelError = 0;
double RMMTracker::accepted3DFeatureError = 0.0;
double RMMTracker::acceptedOrientationError = 0.0;

double RMMTracker::m_highVisualSupportThreshold = 0.95;

int RMMTracker::m_meanMillisecondsDifferenceBetweenFrames = 110;

int bestHypothesesNode::numVariables = 0;
int bestHypothesesNode::variablesSum = 0;
int *bestHypothesesNode::variablesNumFrames = 0;
std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *bestHypothesesNode::newObjectsListEnds;

RMMTracker::RMMTracker(Datapool *i_data) : m_data(i_data), parametersInitialised(false),
                                           modelTemplate(m_data) {
    m_RMerge = new ReliabilityMerge(m_data);
    m_PreMerge = new ReliabilityMerge(m_data);
    //m_PreMerge->m_rc = m_RMerge->m_rc = RMMMobileObject::m_rc = m_rclassif = new ReliabilityClassification(m_data);

    initialPreparation = true;
    checkedMobilePairValidity = NULL;
    validMobilePair = NULL;
    m_setObjectLog = false;
    m_objectLogModelName = "Blob2DFromBGSubstractionModel";
    m_logModelFound = false;
    m_logFirstTime = true;





    m_activatePauseOnLost = m_pauseOnLost = m_activatePauseOnNew = m_pauseOnNew = false;
}

void RMMTracker::getMostLikelyMobileObjects(std::deque<SpRMMMobileObject> &mobileObjectsOutput) {
    std::deque<SpRMMHypothesisSet>::iterator hset_it, hsend = hypothesisSets.end();
    std::deque<SpRMMMobileObject>::iterator mobile_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> *hypotheses;
    SpRMMHypothesis bestHypothesis;

    mobileObjectsOutput.clear();

    if(hypothesisSets.empty())
        return;

    for(hset_it = hypothesisSets.begin(); hset_it != hsend; hset_it++) {
        hypotheses = (*hset_it)->getHypotheses();
        bestHypothesis = (*hypotheses->begin());

        for(mobile_it = bestHypothesis->begin(); mobile_it != bestHypothesis->end(); mobile_it++)
            mobileObjectsOutput.push_back(*mobile_it);
    }
}


bool RMMTracker::initialPrepareRun() {
    if( m_data->fgImage == NULL) {
        AppendToLog("RMMTracker: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }

    initialPreparation = false;

    return true;
}

bool RMMTracker::prepareRun() {

    if( m_data->fgImage == NULL) {
        AppendToLog("RMMTracker: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }

    if (activatePreMerge && !m_PreMerge->prepareRun()) {
        AppendToLog("RMMTracker: error: Error preparing execution for preMerge step.");
        return false;
    }

    if (!m_RMerge->prepareRun()) {
        AppendToLog("RMMTracker: error: Error preparing execution for ReliabilityMerge capabilities.");
        return false;
    }

    return true;
}

bool RMMTracker::init() {
    //TODO: Set as parameter
    //maxObjectSpeed = 200;
    //maxObjectSpeed = i_maxObjectSpeed;
    //   std::cout << "SPEED:" << maxObjectSpeed << "\n";

    if(m_data->objectModels.size() == 0) {
        AppendToLog("RMMTracker: error: At least one ObjectModel definition is required (ContextModule does it). See file 'config/parallelpiped-models.xml' as an example.");
        return false;
    }

    if( (smodel = m_data->sceneModel) == NULL) {
        AppendToLog("RMMTracker: error: SceneModel is NULL. Initialization required (ContextModule does it). ");
        return false;
    }

    m_RMerge->init();
    m_PreMerge->init();

    checkedMobilePairValidity = NULL;
    validMobilePair = NULL;

    initStaticMobileObject();
    mobileAlternativesMap.clear();
    mobile_id_counter = 0;
    hset_id_counter = 0;
    currentTimeMilliSeconds = 0;
    //For detecting first frame
    lastTimeStamp.millisecond = -1;
    lastMilliSecondsDifference = 0;
    RMMMobileObject::m_firstFrame = firstFrame = true;

    return true;
}

QString RMMTracker::initObjectLog(long object_id) {
    QString name = m_objectLogRootName + "-" + QString::number(object_id)
            + "-" + QString::number(m_data->frameNumber) + "-"
            + Datapool::getCurrentDateTimeString() + ".csv";

    QFile file(name);
    if( file.open(QIODevice::ReadOnly) ) {
        file.close();
        QFile::remove(name);
    }
    file.close();

    logFileNames[object_id] = name;

    return name;

}


void RMMTracker::writeObjectLog() {

    if(m_logFirstTime) {
        m_logFirstTime = false;

        if(RMMTracker::modelTemplate.findSingleModelByName(m_objectLogModelName))
            m_logModelFound = true;
        else
            AppendToLog("RMMTracker: Warning: LogModelInstanceName not found. Ignoring Object Instance Log.");

    }

    if(!m_logModelFound) {
        AppendToLog("RMMTracker: Warning: Log files not generated due to name definition errors.");
        return;
    }

    if(m_data->RMMobjects.size() > 0) {
        std::deque<SpRMMMobileObject>::iterator it, it_end = m_data->RMMobjects.end();
        SpRMMMobileObject o;
        QString filename;
        bool first = false;
        for(it=m_data->RMMobjects.begin(); it!=it_end; it++) {
            o =  *it;
            if(logFileNames.count(o->mobile_id) == 0) {
                filename = initObjectLog(o->mobile_id);
                first = true;
            } else
                filename = logFileNames[o->mobile_id];
            QFile m_file(filename);
            if (!m_file.open(QIODevice::Append | QIODevice::Text)) {
                AppendToLog("RMMTracker Error: Unable to open file '"+ filename +"' for append.");
                return;
            }

            QTextStream logStream(&m_file);
            logStream.setRealNumberPrecision(2);
            logStream.setRealNumberNotation(QTextStream::FixedNotation);
            if(first) {
                logStream << "Object\t" << o->mobile_id << endl << endl;
                if(m_logModelFound) {
                    logStream << "\t\tBlob\t\tObject\t\t\t\tVelocity\t\t\t\tAcceleration" << endl;
                    logStream << "Frame\tAttribute\tValue\tRD\tValue\tSD\tRC\tRD\tValue\tSD\tRC\tRD\tValue\tSD\tRC\tRD" << endl;
                } else {
                    logStream << "\t\tBlob" << endl;
                    logStream << "Frame\tAttribute\tValue\tRD" << endl;
                }
            }
            logStream << m_data->frameNumber;

            SpReliabilitySingleModelInterface model;
            if(m_logModelFound) {
                model = o->multiModel.getSingleModelByName(m_objectLogModelName);

                std::map <QString, AttributeDupletValue>::iterator inst_it, inst_end = model->instances.front().attributes.end();
                std::map <QString, ReliabilityDynamicsAttribute>::iterator dyn_it, dyn_end = model->dynamics.dynamics.end();

                for(inst_it = model->instances.front().attributes.begin(); inst_it != inst_end; inst_it++) {
                    AttributeDupletValue &att = (*inst_it).second;
                    logStream << "\t" << (*inst_it).first;
                    logStream << "\t" << att.value;
                    logStream << "\t" << att.RD;

                    dyn_it = model->dynamics.dynamics.find((*inst_it).first);
                    if(dyn_it != dyn_end) {
                        ReliabilityDynamicsAttribute &datt = dyn_it->second;
                        logStream << "\t" << datt.att.value;
                        logStream << "\t" << datt.att.VR.SD;
                        logStream << "\t" << datt.att.VR.RC;
                        logStream << "\t" << datt.att.VR.RD;
                        logStream << "\t" << datt.V.value;
                        logStream << "\t" << datt.V.VR.SD;
                        logStream << "\t" << datt.V.VR.RC;
                        logStream << "\t" << datt.V.VR.RD;
                        logStream << "\t" << datt.A.value;
                        logStream << "\t" << datt.A.VR.SD;
                        logStream << "\t" << datt.A.VR.RC;
                        logStream << "\t" << datt.A.VR.RD << endl;
                    }
                }
            }

            m_file.close();

        }
    }

}

void RMMTracker::initStaticMobileObject() {

    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;

    RMMMobileObject::m_context = smodel;
    RMMMobileObject::m_maxSpeed = maxObjectSpeed;
    RMMMobileObject::rigidModel.clear();
    RMMMobileObject::lastFoundSubtypeTemplate.clear();

    RMMMobileObject::objectModelMap.clear();
    RMMMobileObject::objectSubModelMap.clear();
}

int RMMTracker::getBetaDirection(SceneModel *smodel, QImage *m_pSegmentation) {
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


double RMMTracker::getObjectDistanceForMaxReliability () {

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
    double distance = RMMMobileObject::m_maxKnownSpeed*RMMMobileObject::secDiffSequence[0];

    x2 = xf + distance*cos(alpha);
    y2 = yf + distance*sin(alpha);

    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x2, y2, 0.0, &X2, &Y2);
    double
        dX = Xf - X2,
        dY = Yf - Y2;

    return sqrt(dX*dX + dY*dY);

}

double RMMTracker::getObjectSizeForMaxReliability(double w, double l, double h) {
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

double RMMTracker::getMaxFocalDistanceToImageCorner() {
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

RMMTracker::~RMMTracker() {

    RMMMobileObject::rigidModel.clear();
    RMMMobileObject::lastFoundSubtypeTemplate.clear();

    biggestBlobForNewMobile.clear();
    usedIdsByHypothesis.clear();

    delete[] RMMMobileObject::objectModelsList;

    if(RMMMobileObject::m_objectModelsNumber > 0)
        for(int i = 0; i < RMMMobileObject::m_objectModelsNumber; i++)
            if(RMMMobileObject::objectSubModelsList[i])
                delete[] RMMMobileObject::objectSubModelsList[i];


    delete[] RMMMobileObject::objectSubModelsList;

    if(RMMMobileObject::secDiffSequence)
        delete[] RMMMobileObject::secDiffSequence;

    if(RMMMobileObject::coolingValue)
        delete[] RMMMobileObject::coolingValue;

    if(RMMMobileObject::secDiffToCurrent)
        delete[] RMMMobileObject::secDiffToCurrent;

    if(checkedMobilePairValidity != NULL)
        delete[] checkedMobilePairValidity;

    if(validMobilePair != NULL)
        delete[] validMobilePair;
}


void RMMTracker::preMerge(std::vector<Blob>& blobs) {

    if(activatePreMerge) {
        presetBlobsVectorAndInitialMergeTable(blobs);

        unsigned int i;
#ifdef RMTT_OUTPUT
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;
#endif
        Blob::orderByProximityToPoint(blobs, SM_CAMERA_X2D_FOC_POINT(smodel), SM_CAMERA_Y2D_FOC_POINT(smodel));
#ifdef RMTT_OUTPUT
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;
#endif
        m_PreMerge->preMerge(initialMergeMap, blobs);
#ifdef RMTT_OUTPUT
        std::cout << "After pre-merge:" << std::endl;
        for(i=0; i<blobs.size(); i++)
            std::cout << blobs[i] << std::endl;
#endif
        freeBlobsVectorAndInitialMergeTable();
    }
}

//TODO: Implement Split
//void RMMTracker::preSplit(std::vector<Blob>& blobs) {
//}

void RMMTracker::checkConnectivity(bool groupVector[], int elementsNumber, int referencePoint, int *elementsToAnalyzeVector) {
    int i;

    groupVector[referencePoint] = true;

    for(i=1; i<elementsNumber; i++)
        if(initialMergeMap[elementsToAnalyzeVector[referencePoint]][elementsToAnalyzeVector[i]] && groupVector[i] == false)
            checkConnectivity(groupVector, elementsNumber, i, elementsToAnalyzeVector);

}


bool RMMTracker::validBlobMergeConfiguration(int mergeGroupNumber, int mergeLength, int blobsToMergeNumber, int *blobsToMerge, int *hypothesesCombo) {
    bool validMerges[mergeLength];

    memset(validMerges, false, sizeof(bool)*mergeLength);

    int i, j, blobToMergeArray[mergeLength], trueConnections=0;

    for(i=0, j=0; i<blobsToMergeNumber; i++) {
        if(hypothesesCombo[i] == mergeGroupNumber)
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


bool RMMTracker::blobCanBeIncludedForMerge(int currentMergeLength, int *mergedBlobIndexes, int currentBlobIndex) {
    int j;

    for(j=0; j<currentMergeLength; j++) {
        if(initialMergeMap[currentBlobIndex][mergedBlobIndexes[j]])
            return true;
    }

    return false;
}

void RMMTracker::processMergeVector(int *hypothesesCombo, int *blobsToMerge, int blobsToMergeNumber) {
    //First set length of each merged blob:
    int i, mergeLength[blobsToMergeNumber], finalBlobsNumber = 0;

    for(i=0; i<blobsToMergeNumber; i++)
        mergeLength[i] = 0;

    for(i=0; i<blobsToMergeNumber; i++) {
        mergeLength[hypothesesCombo[i]]++;
        if(hypothesesCombo[i] > finalBlobsNumber)
            finalBlobsNumber = hypothesesCombo[i];
    }

    finalBlobsNumber++;

    //Check validity of analyzed combination
    bool thereIsMerge =  finalBlobsNumber < blobsToMergeNumber ? true : false;

    if(thereIsMerge && mergeLength[0] < blobsToMergeNumber)
        for(i=0; i<blobsToMergeNumber && mergeLength[i]>0 ; i++)
            if(mergeLength[i] > 1 && !validBlobMergeConfiguration(i,mergeLength[i],blobsToMergeNumber,blobsToMerge,hypothesesCombo))
                return;

    //Creating or getting merged blobs
    Blob *mergedBlobs[finalBlobsNumber];
    bool usedBlobsMatrix[finalBlobsNumber*blobsNumber];
    int j, k;
    memset(usedBlobsMatrix, false, finalBlobsNumber*blobsNumber*sizeof(bool));
    for(i=0; i<finalBlobsNumber; i++) {
        if(mergeLength[i] == 1) {
            for(j=0; j<blobsToMergeNumber; j++)
                if(hypothesesCombo[j] == i) {
                    usedBlobsMatrix[blobsNumber*i + blobsToMerge[i]] = true;
                    mergedBlobs[i] =  blobsVector[blobsToMerge[i]];
                    break;
                }
        } else {
            int j, blobToMergeArray[mergeLength[i]];
            bool ascending = true;

            for(k=0, j=0; k<blobsToMergeNumber; k++) {
                if(hypothesesCombo[k] == i) {
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

            //A merged blob is demanded

            mergedBlobs[i] = m_RMerge->getMergedBlob(blobToMergeArray, mergeLength[i], blobsVector, blobsNumber, false);
        }
    }

    //Check if the combination contains any blob completly inside another one, in this case the hypothesis
    //is not considered because there will be another one representing the same hypothesis.
    if(thereIsMerge && combinationAlreadyIncluded(mergedBlobs, finalBlobsNumber) )
        return;

    //Generate a hypothesis solution for each set of blob merges
    insertNewMergeSolution(mergedBlobs, finalBlobsNumber, usedBlobsMatrix);
}

bool RMMTracker::combinationAlreadyIncluded(Blob **blobsForHypothesis, int blobsNumberForHypothesis) {
    int i,j;
    if(blobsNumberForHypothesis == 1)
        return false;

    for(i=0; i<blobsNumberForHypothesis - 1; i++)
        for(j=i+1; j<blobsNumberForHypothesis; j++)
            if(   Blob::isBlob1InsideBlob2(blobsForHypothesis[j], blobsForHypothesis[i])
               || Blob::isBlob1InsideBlob2(blobsForHypothesis[i], blobsForHypothesis[j]) )
                return true;
    return false;
}

void RMMTracker::insertNewMergeSolution(Blob **mergedBlobs, int finalBlobsNumber, bool *usedBlobsMatrix) {
    SpRMMHypothesis newHypothesis;
    bool completing = false;
    int i, j, size = 0;
    if(g_baseHypothesis != NULL) { //In case of an analyzed hypothesis of an already existing hypothesis set, duplicate mobile references in hypothesis
        SpRMMHypothesis hypothesisCopy(new RMMHypothesis(g_baseHypothesis));
        newHypothesis = hypothesisCopy;
        //size = newHypothesis->size();
        //std::cerr << "from 1 size: " << size << std::endl;
        completing = true;
    } else { //New hypothesis for new mobile
        SpRMMHypothesis hypothesisCopy(new RMMHypothesis());
        newHypothesis = hypothesisCopy;
        //size = newAltSolution->size();
        //std::cerr << "from 2 size: " << size << std::endl;
    }

    usedIdsByHypothesis.clear();
    if(completing) {
        bool valid;
        bool comparable[size];
        SpRMMMobileObject trackedMobiles[size];
        std::deque<SpRMMMobileObject>::iterator trackedMobiles_it;
        for(j=0, trackedMobiles_it = newHypothesis->begin(); j<size; j++, trackedMobiles_it++) {
            trackedMobiles[j] = *trackedMobiles_it;
            comparable[j] = true; //ACA!!!!! Modificar
            //comparable[j] = trackedMobiles[j]->mobile3DCoherenceIsAcceptable();
        }

        for(i=0; i<finalBlobsNumber; i++) {
            //	std::cout << i << ": " << get_name_from_type(S3D_TYPE(mergedBlobs[i])) << std::endl;
            SpRMMMobileObject newMobile = getNewMobileFromBlob(mergedBlobs[i]);

            newMobile->numUsed = 0;
            for(j=0; j < blobsNumber; j++) {
                newMobile->usedBlobs[j] = usedBlobsMatrix[blobsNumber*i + j];
                if(newMobile->usedBlobs[j])
                    newMobile->numUsed++;
            }

            valid = true;
            //New mobile object will be comparable if it is classified

            //ACAAAAAA!!! Adaptar codigo

            /*if(BLOB_TYPE(mergedBlobs[i]) != UNKNOWN && BLOB_P(mergedBlobs[i]) >= RMMMobileObject::m_classifThreshold) {
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
            } else {*/
            if(1) {
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
                newHypothesis->insertNewMobileObject(newMobile);
        }

    } else { //Global Solution
        SpRMMMobileObject newMobile;
        for(i=0; i<finalBlobsNumber; i++) {
            //std::cout << i << ": " << get_name_from_type(S3D_TYPE(mergedBlobs[i])) << std::endl;
            newHypothesis->insertNewMobileObject(newMobile = getNewMobileFromBlob(mergedBlobs[i]));
            newMobile->numUsed = 0;
            for(j=0; j < blobsNumber; j++) {
                newMobile->usedBlobs[j] = usedBlobsMatrix[blobsNumber*i + j];
                if(newMobile->usedBlobs[j])
                    newMobile->numUsed++;
            }
        }
    }

    newHypothesis->setHypothesisProbability();
    g_inserted_for_hypothesis++;
    g_newHypotheses.insert(newHypothesis);

}

void RMMTracker::insertInMobileAlternativesMap(SpRMMMobileObject mobile, long int id) {
    std::map< long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> >::iterator existing_set;

    if((existing_set = mobileAlternativesMap.find(id)) == mobileAlternativesMap.end()) {
        std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> newSet;
        newSet.insert(mobile);
        mobileAlternativesMap[id] = newSet;
        return;
    }

    existing_set->second.insert(mobile);
}

SpRMMMobileObject RMMTracker::getNewMobileFromBlob(Blob *blob) {
    SpRMMMobileObject newMobile(new RMMMobileObject(m_data));
    int id = getMobileId(blob);
    m_pauseOnNew = true;
    //    std::cout << "Mobile ID: " << id << std::endl;
    newMobile->setNewMobileFromBlob(blob, id, hset_id_counter);
    insertInMobileAlternativesMap(newMobile, id);

    return newMobile;
}

bool RMMTracker::notUsedId(long int id) {
    return usedIdsByHypothesis.find(id) != usedIdsByHypothesis.end() ? false : true;
}

int RMMTracker::getMobileId(Blob *blob) {
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
            usedIdsByHypothesis.insert(current_id);
            return current_id;
        }
        //If maximal blob for current mobile enters inside new blob, maximal blob must be updated
        if(Blob::isBlob1InsideBlob2(current_blob, blob) && notUsedId(current_id)) {
            p_iter->blob = blob;
            usedIdsByHypothesis.insert(current_id);
            return current_id;
        }
    }

    //If loop have not returned any id, a new id must be assigned
    mobile_id_counter++;

    IdBlobPair p(mobile_id_counter, blob);
    biggestBlobForNewMobile.push_back(p);
    usedIdsByHypothesis.insert(mobile_id_counter);

    return mobile_id_counter;
}

int RMMTracker::getMiddle(int *array, int top, int bottom) {
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
void RMMTracker::orderAscending(int *array, int top, int bottom) {
    int middle;
    if(top < bottom) {
        middle = getMiddle(array, top, bottom);
        orderAscending(array, top, middle);   // sort top partition
        orderAscending(array, middle+1, bottom);    // sort bottom partition
    }
}

void RMMTracker::recursiveComboGenerator(int position, int value, int *hypothesesCombo, int *blobsToMerge, int blobsToMergeNumber) {
    int i;

    if(position == blobsToMergeNumber) {
        processMergeVector(hypothesesCombo, blobsToMerge, blobsToMergeNumber);
    } else
        for(i=0; i <= value; i++) {
            hypothesesCombo[position] = i;
            recursiveComboGenerator(position+1, (value > i+1) ? value : i+1, hypothesesCombo, blobsToMerge, blobsToMergeNumber);
        }
}


void RMMTracker::generateHypothesesForMobile(int *blobsToMerge, int blobsToMergeNumber) {
    int hypothesesCombo[blobsToMergeNumber];
    hypothesesCombo[0] = 0;
    recursiveComboGenerator(1, 1, hypothesesCombo, blobsToMerge, blobsToMergeNumber);
}

void RMMTracker::generateNewMobiles(int blobNumberToAnalyze, int *blobsToAnalyze, int groupsNumber, int *blobGroupVector) {
    int i, j;
    int blobsToMerge[blobNumberToAnalyze], blobsToMergeNumber;
    bool globalInsertion = false;
    if(g_baseHypothesis == NULL)
        globalInsertion = true;

    for(i=0; i<groupsNumber; i++) {
        //Set the blobs to merge per group
        blobsToMergeNumber = 0;
        for(j=0; j<blobNumberToAnalyze; j++)
            if(blobGroupVector[j] == i)
                blobsToMerge[blobsToMergeNumber++] = blobsToAnalyze[j];

        SpRMMHypothesisSet newHSets;

        //If global, prepare a new hypothesis set
        if(globalInsertion) {
            SpRMMHypothesisSet auxHSet(new RMMHypothesisSet());
            newHSets = auxHSet;
            g_newHypotheses.clear();
            hset_id_counter++;
        }

        //Generate the hypotheses for new mobile
        generateHypothesesForMobile(blobsToMerge, blobsToMergeNumber);

        if(globalInsertion) {
            newHSets->hypotheses = g_newHypotheses;
            newHSets->setBestHypothesis();
            hypothesisSets.insert(newHSets);
        }
    }
}

void RMMTracker::insertNewMobiles(SpRMMHypothesis hypothesis, SpRMMHypothesisSet hset) {
    //ACAAAA: Volver a revisar
    int i, j, blobNumberToAnalyze = 0, groupsNumber;
    int blobsToAnalyze[blobsNumber];
    int blobGroupVector[blobsNumber];

    if(!hypothesis.isNull()) {
        g_baseHypothesis = hypothesis;
        bool ensureUsedBlobs[blobsNumber];
        int size = 0;
        std::deque<SpRMMMobileObject>::iterator mobile_it;

        memset(ensureUsedBlobs, false, blobsNumber*sizeof(bool));
        mobile_it = g_baseHypothesis->begin();
        size = g_baseHypothesis->size();

        /*
        for(i=0; i< size; i++, mobile_it++) {
            if((*mobile_it)->ensureMode) {
                for(j=0; j<blobsNumber; j++)
                    ensureUsedBlobs[j] |= (*mobile_it)->usedBlobs[j];
            }
        }*/

        for(i=0; i<blobsNumber; i++)
            //if(hset->usedBlobs[i] && !hypothesis->usedBlobs[i] && !ensureUsedBlobs[i]) //Consider blobs used in some of the hypothesis sets, that are not used in the analyzed hypothesis
            if(hset->usedBlobs[i] && !hypothesis->usedBlobs[i]) //Consider blobs used in some of the hypothesis sets, that are not used in the analyzed hypothesis
                blobsToAnalyze[blobNumberToAnalyze++] = i;
    } else { //Globally not used blobs
        g_baseHypothesis = QSharedPointer<RMMHypothesis>();
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

void RMMTracker::setCurrentTimeAndFrame() {
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
    memmove(RMMMobileObject::secDiffSequence  + 1, RMMMobileObject::secDiffSequence,  sizeof(double)*(m_BlobBufferSize-1));
    memmove(RMMMobileObject::secDiffToCurrent + 1, RMMMobileObject::secDiffToCurrent, sizeof(double)*(m_BlobBufferSize-1));

    //In case of a frame jump, consider the last seconds difference (IntervalDiskInput Acquisition Method)
    if(lastTimeStamp.millisecond >= 0 && ts->frame_id - currentFrameNumber > 1) {
        if(RMMMobileObject::secDiffSequence[1] == 0.0)
            RMMMobileObject::secDiffToCurrent[1] = RMMMobileObject::secDiffSequence[0]  = m_meanMillisecondsDifferenceBetweenFrames/1000.0;
        else
            RMMMobileObject::secDiffToCurrent[1] = RMMMobileObject::secDiffSequence[0]  = RMMMobileObject::secDiffSequence[1];
        lastMilliSecondsDifference = (int)(1000*RMMMobileObject::secDiffSequence[0]);
    } else
        RMMMobileObject::secDiffToCurrent[1] = RMMMobileObject::secDiffSequence[0] = lastMilliSecondsDifference/1000.0;

    for(i=2; i<m_BlobBufferSize; i++)
        RMMMobileObject::secDiffToCurrent[i] += RMMMobileObject::secDiffSequence[0];
    double value;
    for(i=1; i<m_BlobBufferSize; i++) {
        value = RMMMobileObject::secDiffToCurrent[i];
        value = RMMMobileObject::coolingFunction(RMMMobileObject::secDiffToCurrent[i]);
        RMMMobileObject::coolingValue[i] = value;
    }
    //ACAAAA!!!: Unificar
    //RMMMobileObject::m_objectDistanceForMaxReliability = getObjectDistanceForMaxReliability();

    currentFrameNumber = ts->frame_id;

    memcpy(&lastTimeStamp, ts, sizeof(TimeStamp));

    if(m_internalOutputActivated) {
        std::cout << ts->frame_id << ": " << ts->day<< "/" << ts->month<< "/" << ts->year << "\t" << ts->hour << ":"  << ts->minute << ":"  << ts->second << ","  << ts->millisecond << std::endl;
        std::cout << "msec:\t" << currentTimeMilliSeconds << std::endl;
        std::cout << "msec diff:\t" << lastMilliSecondsDifference << std::endl;
    }
}



void RMMTracker::update(){

    if(blobsNumber == 0 && hypothesisSets.empty())
        return;
#ifdef RMTT_OUTPUT
    std::cout << "Update 1: numBlobs:" << blobsNumber << std::endl;
#endif
    //Complete the information on existing mobiles for time t
    if(!hypothesisSets.empty()) {
        followExistingMobiles();

        if(blobsNumber > 0) {
            //Set new mobiles for remaining blobs for each ReliableMobile
            std::deque<SpRMMHypothesisSet>::iterator hset_it;
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypothesis_it, hyp_it;
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> *hypotheses;
            int j;
#ifdef RMTT_OUTPUT
            std::cout << "Hypothesis set size: " << hypothesisSets.size() << std::endl;

            for(j=0, hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); j++, hset_it++);
            std::cout << "Hypothesis set real: " << j << std::endl;
#endif
            //Complete with new mobiles the incomplete solutions
            for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {

#ifdef RMTT_OUTPUT
                std::cout << "Beginning - Mobile size: " << (*hset_it)->size() << std::endl;
                for(j=0, hyp_it=(*hset_it)->begin(); hyp_it!=(*hset_it)->end(); j++, hyp_it++);
                std::cout << "Beginning - Mobile size real: " << j << std::endl;
#endif

                //If incomplete hypotheses exist
                if((*hset_it)->incompleteHypotheses > 0) {
                    g_newHypotheses.clear();
                    g_completeHypotheses.clear();
                    biggestBlobForNewMobile.clear();

                    hypotheses = (*hset_it)->getHypotheses();
                    for(hypothesis_it = hypotheses->begin(); hypothesis_it != hypotheses->end(); hypothesis_it++) {
                        if((*hypothesis_it)->incompleteHypothesis) {
                            g_inserted_for_hypothesis = 0;
                            insertNewMobiles((*hypothesis_it), (*hset_it));
                            if(g_inserted_for_hypothesis == 0) //If no hypothesis added, the base hypothesis will be lost, so add it
                                g_completeHypotheses.insert(*hypothesis_it);
                            (*hset_it)->incompleteHypotheses--;
                        } else
                            g_completeHypotheses.insert(*hypothesis_it);
                    }

                    //In case of solution completion, regenerate hypotheses list for hypothesis set with new hypotheses
                    //if there are.
                    if(g_newHypotheses.size() > 0 || g_completeHypotheses.size() > 0) {
                        (*hset_it)->clear();
                        int j;
                        std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hyp_it;
#ifdef RMTT_OUTPUT
                        if(g_completeHypotheses.size() > 0) {
                            std::cout << "Inside - Mobile size: " << (*hset_it)->size() << std::endl;
                            std::cout << "Inside - complete size: " << g_completeHypotheses.size() << std::endl;
                            for(j=0, hyp_it=g_completeHypotheses.begin(); hyp_it!=g_completeHypotheses.end(); j++, hyp_it++);
                            std::cout << "Inside - complete size real: " << j << std::endl;
                        }
                        if(g_newHypotheses.size() > 0) {
                            std::cout << "Inside - new size: " << g_newHypotheses.size() << std::endl;
                            for(j=0, hyp_it=g_newHypotheses.begin(); hyp_it!=g_newHypotheses.end(); j++, hyp_it++);
                            std::cout << "Inside - new size real: " << j << std::endl;
                        }
#endif
                        if(g_completeHypotheses.size() > 0) {
                            (*hset_it)->hypotheses = g_completeHypotheses;
#ifdef RMTT_OUTPUT
                            std::cout << "Inside - Mobile size first: " << (*hset_it)->size() << std::endl;
                            for(j=0, hyp_it=(*hset_it)->begin(); hyp_it!=(*hset_it)->end(); j++, hyp_it++);
                            std::cout << "Inside - Mobile size real: " << j << std::endl;
#endif
                        }
                        if(g_newHypotheses.size() > 0) {
                            if((*hset_it)->size() == 0)
                                (*hset_it)->hypotheses = g_newHypotheses;
                            else {
                                std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hyp_it, hyp_end = g_newHypotheses.end();
                                for(hyp_it = g_newHypotheses.begin(); hyp_it != hyp_end; hyp_it++ )
                                    (*hset_it)->insert(*hyp_it);
                            }
                        }
#ifdef RMTT_OUTPUT
                        std::cout << "Inside - Mobile size second: " << (*hset_it)->size() << std::endl;
                        for(j=0, hyp_it=(*hset_it)->begin(); hyp_it!=(*hset_it)->end(); j++, hyp_it++);
                        std::cout << "Inside - Mobile size real: " << j << std::endl;
#endif
                        (*hset_it)->setBestHypothesis();
                    }
                }
            }
        }
    }

    if(blobsNumber > 0) {
        biggestBlobForNewMobile.clear();
        g_inserted_for_hypothesis = 0;
        g_newHypotheses.clear();
        insertNewMobiles(QSharedPointer<RMMHypothesis>(), QSharedPointer<RMMHypothesisSet>());
    }
}

void RMMTracker::mergeHypothesisSets(SpRMMHypothesisSet firstHSet, SpRMMHypothesisSet secondHSet) {
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> newList;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypFromFirst = firstHSet->begin(), hypFromSecond = secondHSet->begin();
    int i;
    bool *involved_new, *involved_first, *involved_second;

//    if(m_data->frameNumber == 951) {
//        std::cerr << "Aquí nos detenemos!!" << std::endl;
//    }

    for(; hypFromFirst!=firstHSet->end(); hypFromFirst++) {
        for(hypFromSecond = secondHSet->begin(); hypFromSecond!=secondHSet->end(); hypFromSecond++) {
            SpRMMHypothesis newHypothesis(new RMMHypothesis());
            newHypothesis->mobiles = (*hypFromFirst)->mobiles;
            newHypothesis->insert(newHypothesis->end(), (*hypFromSecond)->begin(), (*hypFromSecond)->end());
            newHypothesis->setHypothesisProbability();
            newHypothesis->initInvolvedBlobs();
            involved_new    = newHypothesis   ->involvedBlobs;
            involved_first  = (*hypFromFirst) ->involvedBlobs;
            involved_second = (*hypFromSecond)->involvedBlobs;
            for(i=0;i<blobsNumber;i++)
                involved_new[i] = involved_first[i] | involved_second[i];

            newList.insert(newHypothesis);
        }
    }
    firstHSet->clear();
    firstHSet->hypotheses = newList;
    firstHSet->setBestHypothesis();

    newList.clear();
}


void RMMTracker::mergeInvolvedHypothesisSets() {

    int i, num_hsets = hypothesisSets.size();

    if (num_hsets < 2) //Not enough rmobiles for possible conflicts
        return;

    int first_conflict = -1;

    for(i=0; i<blobsNumber; i++)
        if(involvedHypothesisSetsCounter[i] > 1) {
            first_conflict = i;
            break;
        }

    if(first_conflict < 0) //No conflicts found
        return;

    std::deque<SpRMMHypothesisSet>::iterator first_hset, second_hset;
    bool *involved_first, *involved_second;
    int current_conflict = first_conflict;
    bool remaining_conflicts = true;

    int j, k, numToErase = 0;

    while(remaining_conflicts) {

        for(first_hset = hypothesisSets.begin(), i=0; i < num_hsets-1; first_hset++, i++) {
            if(!(*first_hset)->toErase && (*first_hset)->involvedBlobs[current_conflict]) {

                involved_first = (*first_hset)->involvedBlobs;

                for(second_hset = first_hset + 1, j = i + 1; j < num_hsets; second_hset++, j++) {

                    if(!(*second_hset)->toErase && (*second_hset)->involvedBlobs[current_conflict]) {

                        involved_second = (*second_hset)->involvedBlobs;
                        involvedHypothesisSetsCounter[current_conflict]--;

                        //Merge two RMobiles and store in first one.
                        mergeHypothesisSets((*first_hset), (*second_hset));

                        //Correct the involved blobs for resulting rmobile (the first one)
                        for(k=current_conflict+1; k<blobsNumber; k++) {
                            if(involved_second[k]) {
                                if(involved_first[k])
                                    involvedHypothesisSetsCounter[k]--;
                                else
                                    involved_first[k] = true;
                            }
                        }

                        (*second_hset)->toErase = true;
                        numToErase++;
                    }

                    if(involvedHypothesisSetsCounter[current_conflict] < 2)
                        break;
                }
            }

            if(involvedHypothesisSetsCounter[current_conflict] < 2)
                break;
        }

        remaining_conflicts = false;
        for(i=current_conflict+1; i<blobsNumber; i++)
            if(involvedHypothesisSetsCounter[i]>1) {
                current_conflict = i;
                remaining_conflicts = true;
                break;
            }

    }

    if(numToErase > 0) {
        for(first_hset = hypothesisSets.begin(); first_hset != hypothesisSets.end(); ) {
            if((*first_hset)->toErase) {
                first_hset = hypothesisSets.erase(first_hset);
                numToErase--;
            } else
                first_hset++;

            if(numToErase == 0)
                break;

        }
    }
}

void RMMTracker::createMobilePossibilities() {
    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypothesis_it;
    int hypothesesToEliminateCount, hsetsToEliminateCount = 0;

    //ACAAAAA!!! Redefinir
    m_RMerge->clearDefinedMerges();

    for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
        (*hset_it)->initUsedBlobsList();
        hypothesesToEliminateCount = 0;

        for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++) {
            generateBestPossiblePathsForMobiles2(*hypothesis_it);
            if((*hypothesis_it)->newObjectsList.empty()) {
                (*hypothesis_it)->toEliminate = true;
                hypothesesToEliminateCount++;
            }
        }

        //Eliminate empty hypotheses (with just lost weak mobiles)
        if(hypothesesToEliminateCount > 0) {
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> newList;
            for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++)
                if(!(*hypothesis_it)->toEliminate)
                    newList.insert(*hypothesis_it);

                (*hset_it)->clear();

                if(newList.empty()) {
                    (*hset_it)->toErase = true;
                    hsetsToEliminateCount++;
                } else {
                    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator new_it, new_end = newList.end();
                    for(new_it = newList.begin(); new_it != new_end; new_it++)
                        (*hset_it)->insert(*new_it);
                    (*hset_it)->setBestHypothesis();
                }
        }
    }


    //Eliminate empty hypothesis (with just lost weak mobiles)
    if(hsetsToEliminateCount > 0) {
        int numEliminated = 0;

        for(hset_it = hypothesisSets.begin(); numEliminated < hsetsToEliminateCount && hset_it != hypothesisSets.end(); ) {
            if((*hset_it)->toErase == true) {
                hset_it = hypothesisSets.erase(hset_it);
                numEliminated++;
            } else
                hset_it++;

        }
    }
}

void RMMTracker::generateBestPossiblePathsForMobiles2(SpRMMHypothesis hypothesis) {
    //    std::map< long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> >::iterator mobileHypotheses_iter;
    std::deque<SpRMMMobileObject>::iterator mobiles_iter = hypothesis->begin();
    int i, j, size = hypothesis->size();
    SpRMMMobileObject currentMobile;
    //    std::cout << std::endl << "For hypothesis: " << hypothesis->getProbability() << std::endl;

    hypothesis->newObjectsList.clear();
    hypothesis->numCurrentlyLost = 0;
    for(i=0; i<size; i++, mobiles_iter++) {

        currentMobile = *mobiles_iter;
        g_newMobiles.clear();
        g_newSpecialMobiles.clear();

        double R;
        //Test the visual support of involved blobs
        if (currentMobile->numInvolved == 0) {//The object is LOST for sure
            Blob *specialBlob = new Blob();
            Rectangle<int> mobileBBox = currentMobile->getVisualEvidenceEstimator(R);
            memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
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
            SpRMMMobileObject newMobile = generateAndValidateNewMobile(currentMobile, blobToTest);
            if(newMobile != NULL)
                g_newMobiles.insert(newMobile);
            else {
                Blob *specialBlob = new Blob();
                Rectangle<int> mobileBBox =currentMobile->getVisualEvidenceEstimator(R);
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

            mobileBBox = currentMobile->getVisualEvidenceEstimator(R);

            //Get the initial blob merge with the blobs totally supported by the mobile, which are connected
            //with the most supported blob.
            for(j=0; j<blobsNumber; j++) {
                if(currentMobile->involvedBlobs[j]) {
                    //Visual support is 1.0 when mobile bbox is completelly covered by the analysed blob
                    mobileSupport[j] = Rectangle<int>::rectangleIntersectRatio(&mobileBBox, &(blobsVector[j]->bbox));
                    blobSupport[j] = Rectangle<int>::rectangleIntersectRatio(&(blobsVector[j]->bbox), &mobileBBox);
                    if(mobileSupport[j] >= bestSupport) {
                        totalSupport = bestSupport = mobileSupport[j];
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
                            if( blobCanBeIncludedForMerge(numUsed, mergedBlobsList, j) && mobileSupport[j] > 0.0 && blobSupport[j] >= m_blobCompletellySupportedThreshold ) {
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
                    if(currentMobile->involvedBlobs[j] && mobileSupport[j] > 0 && blobSupport[j] > totalSupport) {
                        initialBlob = blobsVector[j];
                        usedIndex = j;
                        totalSupport = mobileSupport[j];
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
                mobileBBox = currentMobile->getVisualEvidenceEstimator(R);
                memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
                g_newSpecialMobiles.insert(setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob));
                delete specialBlob;
            } else { //There is an initialSolution to add and to be used as a starting point for searching new ones
                //Add the initial solution
                g_bestGlobalP = 0.0;
                //Generate new mobile
                SpRMMMobileObject
                    specialMobile = QSharedPointer<RMMMobileObject>(),
                    newMobile = generateAndValidateNewMobile(currentMobile, initialBlob);

                if(!newMobile.isNull())
                    g_newMobiles.insert(newMobile);

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

                    //Filter incoherent hypotheses for mobile, with respect to the best one
                    if(g_newMobiles.size() > 1) {
                        std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newMobiles.begin();
                        //Start from second, because the first is the best
                        for(j=2, newMobiles_iter++; newMobiles_iter != g_newMobiles.end(); newMobiles_iter++, j++)
                            //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                            //remaining mobiles.
                            if(j > maximumMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) {
                                g_newMobiles.erase(newMobiles_iter, g_newMobiles.end());
                                break;
                            }
                    } else if (g_newSpecialMobiles.size() > 1) { //If no new normal mobiles, try the special cases list
                        std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newSpecialMobiles.begin();
                        //Start from second, because the first is the best
                        for(j=2, newMobiles_iter++; newMobiles_iter != g_newSpecialMobiles.end(); newMobiles_iter++, j++)
                            //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                            //remaining mobiles.
                            if(j > maximumMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) {
                                g_newSpecialMobiles.erase(newMobiles_iter, g_newSpecialMobiles.end());
                                break;
                            }
                    }

                    //Free allocated blobs
                    std::deque<Blob *>::iterator allocated;
                    int alloc_size = g_allocatedBlobs.size();
                    for(j=0, allocated = g_allocatedBlobs.begin(); j < alloc_size; j++, allocated++)
                        delete *allocated;

                } else if (mergedBlob != NULL)
                    delete mergedBlob;
            }
        }

        if(!g_newMobiles.empty())
            hypothesis->newObjectsList[currentMobile->getMobileId()] = g_newMobiles;
        else if(!g_newSpecialMobiles.empty())
            hypothesis->newObjectsList[currentMobile->getMobileId()] = g_newSpecialMobiles;

    }


}

//Path generation for mobiles in a hypothesis, considering:
//- First searching the blob with the best coverage (visual support) of estimated mobile 2D bounding box
//  (estimated from the most reliable information)
//- Generate mobile hypothesiss progressively merging with near blobs, until a too overcovering merge.
//- If the best possibility has a too low coverage, mark the object as MM_PARTIALLY_DETECTED and use its
//  coverage rate to give a reliability measure for attributes coherence.
//- If the best possibility largelly exceeds the coverage needed for the mobile, mark the object as
//  MM_PART_OF_BIGGER and use the coverage rate the mobile in the blob to give a reliability measure
//  for attributes coherence.
void RMMTracker::generateBestPossiblePathsForMobiles(SpRMMHypothesis hypothesis) {
    //    std::map< long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> >::iterator mobileHypotheses_iter;
    std::deque<SpRMMMobileObject>::iterator mobiles_iter = hypothesis->begin();
    int i, j, size = hypothesis->size();
    SpRMMMobileObject currentMobile;
    //    std::cout << std::endl << "For hypothesis: " << hypothesis->getProbability() << std::endl;

    hypothesis->newObjectsList.clear();
    hypothesis->numCurrentlyLost = 0;
    for(i=0; i<size; i++, mobiles_iter++) {

        currentMobile = *mobiles_iter;
        g_newMobiles.clear();
        g_newSpecialMobiles.clear();

        if(currentMobile->numberOfFramesSinceFirstTimeSeen <= 2) { //For first and second frames information, the only information
                                                    //is about initial dimensions of the 2D bounding box and very
                                                    //totally unreliable velocity information,
                                                    //then generation of mobile hypotheses cannot be too
                                                    //constrained.

            //In this list the included blobs are stored, to verify if the same blob is included twice
            g_includedBlobsInNewMobiles.clear();
            g_bestGlobalP = 0.0;

            //Generate involved blobs merges to generate the mobile paths
            generateMobilePath(currentMobile);

            //Set lost mobile if no solution has been found
            if(g_newMobiles.empty()) {
                g_newMobiles.insert(setNotVisibleBlobMobile(*mobiles_iter, MM_OBJECT_LOST));
                hypothesis->numCurrentlyLost++;
            }

            //Filter incoherent hypotheses for mobile, with respect to the best one
            if(g_newMobiles.size() > 1) {
                std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newMobiles.begin();
                //Start from second, because the first is the best
                for(j=2, newMobiles_iter++; newMobiles_iter != g_newMobiles.end(); newMobiles_iter++, j++)
                    //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                    //remaining mobiles.
                    if(j > maximumMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) {
                        g_newMobiles.erase(newMobiles_iter, g_newMobiles.end());
                        break;
                    }
            }
        } else { //With more information, we generate the initial bounding box guess for the mobile
               //From 3 frames we already have for sure information about 2D dimensions and velocity
               //allowing to estimate where the object should be in the next frame
               //and which size it would have.

            double R;
            //Test the visual support of involved blobs
            if (currentMobile->numInvolved == 0) {//The object is LOST for sure
                Blob *specialBlob = new Blob();
                Rectangle<int> mobileBBox = currentMobile->getVisualEvidenceEstimator(R);
                memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
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
                SpRMMMobileObject newMobile = generateAndValidateNewMobile(currentMobile, blobToTest);
                if(newMobile != NULL) {
                    g_newMobiles.insert(newMobile);
                    //Check other cases, if 3D information is trustable enough
                    if(currentMobile->ensureMode || currentMobile->numberOfFramesSinceFirstTimeSeen > 2*RMMMobileObject::m_blobsBufferSize) {
                        SpRMMMobileObject specialMobile = checkSpecialCases(currentMobile, blobToTest);
                        if(specialMobile != NULL)
                            g_newMobiles.insert(specialMobile);
                    }
                } else {
                    Blob *specialBlob = new Blob();
                    Rectangle<int> mobileBBox =currentMobile->getVisualEvidenceEstimator(R);
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

                mobileBBox = currentMobile->getVisualEvidenceEstimator(R);

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
                    mobileBBox = currentMobile->getVisualEvidenceEstimator(R);
                    memcpy(&(specialBlob->bbox), &mobileBBox, sizeof(Rectangle<int>));
                    g_newSpecialMobiles.insert(setSpecialCaseMobile(MM_OBJECT_LOST, currentMobile, specialBlob));
                    delete specialBlob;
                } else { //There is an initialSolution to add and to be used as a starting point for searching new ones
                    //Add the initial solution
                    g_bestGlobalP = 0.0;
                    //Generate new mobile
                    SpRMMMobileObject
                        specialMobile = QSharedPointer<RMMMobileObject>(),
                        newMobile = generateAndValidateNewMobile(currentMobile, initialBlob);

                    if(!newMobile.isNull())
                        g_newMobiles.insert(newMobile);

                    if(currentMobile->ensureMode || currentMobile->numberOfFramesSinceFirstTimeSeen > 2*RMMMobileObject::m_blobsBufferSize) {
                        specialMobile = checkSpecialCases(currentMobile, initialBlob);
                        if(!specialMobile.isNull())
                            g_newSpecialMobiles.insert(specialMobile);
                    } else
                        specialMobile = QSharedPointer<RMMMobileObject>();

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

                        //Filter incoherent hypotheses for mobile, with respect to the best one
                        if(g_newMobiles.size() > 1) {
                            std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newMobiles.begin();
                            //Start from second, because the first is the best
                            for(j=2, newMobiles_iter++; newMobiles_iter != g_newMobiles.end(); newMobiles_iter++, j++)
                                //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                                //remaining mobiles.
                                if(j > maximumMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) {
                                    g_newMobiles.erase(newMobiles_iter, g_newMobiles.end());
                                    break;
                                }
                        } else if (g_newSpecialMobiles.size() > 1) { //If no new normal mobiles, try the special cases list
                            std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator newMobiles_iter = g_newSpecialMobiles.begin();
                            //Start from second, because the first is the best
                            for(j=2, newMobiles_iter++; newMobiles_iter != g_newSpecialMobiles.end(); newMobiles_iter++, j++)
                                //If this criteria is accomplished, we know that remaining elements are of inferior quality, so we eliminate all
                                //remaining mobiles.
                                if(j > maximumMobilePaths || (*newMobiles_iter)->getGlobalProbability() < g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) {
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
            hypothesis->newObjectsList[currentMobile->getMobileId()] = g_newMobiles;
        else if(!g_newSpecialMobiles.empty())
            hypothesis->newObjectsList[currentMobile->getMobileId()] = g_newSpecialMobiles;

    }

}

bool RMMTracker::acceptableInformationForNewMobile(SpRMMMobileObject newMobile, SpRMMMobileObject oldMobile) {

    //ACAAAAA!!! REDEFINIR!!

    double dimension, variation, dt = RMMMobileObject::secDiffSequence[0];
    //    Blob *currentOldBlob = (*(oldMobile->rbegin())->getBlob();
    //Blob *currentNewBlob = (*(newMobile->blobHistory.rbegin()))->getBlob();

    //Test for 2D is relaxed because 2D is not an invariant on the scene, and it is done with
    //extracted 2D blobs, because the estimation will always try to fit with this data
    //2D Position Coordinate X Test.
    //dimension = oldMobile->t2DSpatialData.X + dt*oldMobile->t2DSpatialData.VX;
    //variation = 2*(oldMobile->t2DSpatialData.SDX + dt*oldMobile->t2DSpatialData.SDVX) + acceptedPixelError;
    //    if(newMobile->t2DSpatialData.X > dimension + variation || newMobile->t2DSpatialData.X < dimension - variation)
    //if(newMobile->t2DSpatialData.X > dimension + variation || newMobile->t2DSpatialData.X < dimension - variation)
    //    return false;
    //2D Position Coordinate Y Test.
    //dimension = oldMobile->t2DSpatialData.Y + dt*oldMobile->t2DSpatialData.VY;
    //variation = 2*(oldMobile->t2DSpatialData.SDY + dt*oldMobile->t2DSpatialData.SDVY) + acceptedPixelError;
    //    if(newMobile->t2DSpatialData.Y > dimension + variation || newMobile->t2DSpatialData.Y < dimension - variation)
    //if(newMobile->t2DSpatialData.Y > dimension + variation || newMobile->t2DSpatialData.Y < dimension - variation)
    //    return false;
    //2D Dimension W Test.
    //dimension = oldMobile->t2DDimData.W + dt*oldMobile->t2DDimData.VW;
    //variation = 2*(oldMobile->t2DDimData.SDW + dt*oldMobile->t2DDimData.SDVW) + acceptedPixelError;
    //    if(newMobile->t2DDimData.W > dimension + variation || newMobile->t2DDimData.W < dimension - variation)
    //if(newMobile->t2DDimData.W > dimension + variation || newMobile->t2DDimData.W < dimension - variation)
    //    return false;
    //2D Dimension H Test.
    //dimension = oldMobile->t2DDimData.H + dt*oldMobile->t2DDimData.VH;
    //variation = 2*(oldMobile->t2DDimData.SDH + dt*oldMobile->t2DDimData.SDVH) + acceptedPixelError;
    //if(newMobile->t2DDimData.H > dimension + variation || newMobile->t2DDimData.H < dimension - variation)
    //if(newMobile->t2DDimData.H > dimension + variation || newMobile->t2DDimData.H < dimension - variation)
    //    return false;

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
                if(newMobile->t3DDimData.alpha > RMMMobileObject::NormalizeOrientation(dimension + variation) && newMobile->t3DDimData.alpha < dimension - variation)
                    return false;
            } else if(dimension - variation < 0.0) {
                if(newMobile->t3DDimData.alpha > dimension + variation && newMobile->t3DDimData.alpha < RMMMobileObject::NormalizeOrientation(dimension - variation))
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

bool RMMTracker::mobilesCombinationIsValid() {
    int i, j;
    SpRMMMobileObject m1, m2;
    for(i = 0; i < g_numberOfNewMobiles - 1; i++) {
        m1 = *g_mobileIterators[i];
        for(j = i + 1; j < g_numberOfNewMobiles; j++) {
            //If both objects are unknown, they cannot be validated by 3D model base.

            //ACA!!! Redefinir con coherencia en general

            //if( !g_acceptable3DCoherenceForMobile[i] && !g_acceptable3DCoherenceForMobile[j] )
            //    continue;

            m2 = *g_mobileIterators[j];

            //if( g_acceptable3DCoherenceForMobile[i] && g_acceptable3DCoherenceForMobile[j] ) {
            //    if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j]))
            //        continue;
            //    return false;
            //} else if( g_acceptable3DCoherenceForMobile[i] ) {
                //The first one has 3D information (see the boolean flag of function)
            //    if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j], true))
            //        continue;
            //    return false;
            //} else {
                //The second one has 3D information (see the boolean flag of function)
                if(checkMobilePairValidity(m1, i, g_mobileVersionIndex[i], m2, j, g_mobileVersionIndex[j], false))
                    continue;
                return false;
            //}
        }
    }

    return true;
}

int RMMTracker::getPairIndex(int mindex, int vindex) {
    int i, index = 0;
    for(i=0; i<mindex; i++)
        index += g_numberOfNewMobileVersions[i];
    index += vindex;

    return index;
}

bool RMMTracker::mobilePairValidityChecked(int index1, int index2) {
    return checkedMobilePairValidity[g_totalNumberOfMobileVersions*index1 + index2];
}

bool RMMTracker::mobilePairIsValid(int index1, int index2) {
    return validMobilePair[g_totalNumberOfMobileVersions*index1 + index2];
}

bool RMMTracker::checkMobilePairValidity(SpRMMMobileObject m1, int mindex1, int vindex1, SpRMMMobileObject m2, int mindex2, int vindex2) {

    int index1 = getPairIndex(mindex1, vindex1), index2 = getPairIndex(mindex2, vindex2);

    if( mobilePairValidityChecked(index1, index2) )
        return mobilePairIsValid(index1, index2);

    double area = areaOfIntersection(m1, m2);

    if(area == 0.0)
        return setPairValidityAndGo(index1, index2, true);

    //if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
    //    || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
    //    return setPairValidityAndGo(index1, index2, false);

    return setPairValidityAndGo(index1, index2, true);
}

bool RMMTracker::checkMobilePairValidity(SpRMMMobileObject m1, int mindex1, int vindex1, SpRMMMobileObject m2, int mindex2, int vindex2, bool firstVerifiable) {

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

/*    if(firstVerifiable) {
        if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
            || area / nvarea > m_maximal3DBaseOverlapping )
            return setPairValidityAndGo(index1, index2, false);
    } else {
        if(    area / nvarea > m_maximal3DBaseOverlapping
            || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
            return setPairValidityAndGo(index1, index2, false);
    }*/

    return setPairValidityAndGo(index1, index2, true);
}

bool RMMTracker::checkMobilePairValidity(SpRMMMobileObject m1, SpRMMMobileObject m2) {

    double area = areaOfIntersection(m1, m2);

    if(area == 0.0)
        return true;

//    if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
//        || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
//        return false;

    return true;
}

bool RMMTracker::checkMobilePairValidity(SpRMMMobileObject m1, SpRMMMobileObject m2, bool firstVerifiable) {

    double area, nvarea;

    if(firstVerifiable)
        area = areaOfIntersectionFirstNo3D(m2, m1, &nvarea);
    else
        area = areaOfIntersectionFirstNo3D(m1, m2, &nvarea);

    if(area == 0.0)
        return true;

/*    if(firstVerifiable) {
        if(    area / (m1->t3DDimData.w * m1->t3DDimData.l) > m_maximal3DBaseOverlapping
            || area / nvarea > m_maximal3DBaseOverlapping )
            return false;
    } else {
        if(    area / nvarea > m_maximal3DBaseOverlapping
            || area / (m2->t3DDimData.w * m2->t3DDimData.l) > m_maximal3DBaseOverlapping )
            return false;
    }
*/
    return true;
}


bool RMMTracker::setPairValidityAndGo(int index1, int index2, bool ret_value) {

    checkedMobilePairValidity[g_totalNumberOfMobileVersions*index1 + index2] = true;
    validMobilePair[g_totalNumberOfMobileVersions*index1 + index2] = ret_value;

    return ret_value;
}

double RMMTracker::areaOfIntersection(SpRMMMobileObject m1, SpRMMMobileObject m2) {
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

    /*polygon2D<double> *p1 = new polygon2D<double>(4), *p2 = new polygon2D<double>(4), *r;
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
*/
    double area = 0;
    return area;
}

double RMMTracker::areaOfIntersectionFirstNo3D(SpRMMMobileObject m1, SpRMMMobileObject m2, double *areaOfNo3D) {
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

    //if(!possible_conflict)
        return 0.0;
/*
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
    */
}

void RMMTracker::generateNewLeavesFromCurrentBests() {
    if(!g_leaves.empty()) {
        std::set<SpBestHypothesesNode, orderedByBestHypothesisProbabilityCooperationOperator>::iterator leaves = g_leaves.begin();
        std::set<SpBestHypothesesNode, orderedByBestHypothesisProbabilityCooperationOperator> new_leaves;
        int i;
        for(; leaves != g_leaves.end() && (*leaves)->added == true; leaves++) {
            for(i=(*leaves)->mobileIndex; i<g_numberOfNewMobiles; i++) {
                if (bestHypothesesNode::variablesNumFrames[i] > 0) {
                    SpBestHypothesesNode newNode(new bestHypothesesNode((*leaves), i));
                    if(newNode->value < 0)
                        continue;
                    new_leaves.insert(newNode);
                }
            }
        }

        while(g_leaves.begin() != g_leaves.end() && (*g_leaves.begin())->added == true) {
            SpBestHypothesesNode aux = *(g_leaves.begin());
            g_leaves.erase(g_leaves.begin());
        }

        if(!new_leaves.empty()) {
            leaves = new_leaves.begin();
            for(; leaves != new_leaves.end(); leaves++)
                g_leaves.insert(*leaves);
            new_leaves.clear();
        }
    }
}


void RMMTracker::buildNewHypothesesFromLeaves(SpRMMHypothesis currentHypothesis) {

    int localMaximumNumber = g_numberOfNewMobiles * m_maximumGeneratedHypothesesPerMobile;

    std::set<SpBestHypothesesNode, orderedByBestHypothesisProbabilityCooperationOperator>::iterator leaves;
    double best;

    while(g_NumLocallyAddedHypotheses < localMaximumNumber && g_NumLocallyAddedHypotheses < m_maximumRetainedHypotheses && g_leaves.size() > 0) {

        generateNewLeavesFromCurrentBests();

        if(!g_leaves.empty()) {
            leaves = g_leaves.begin();
            best = (*leaves)->value;
            do {
                SpRMMHypothesis newHypothesis;
                if((newHypothesis = getHypothesisFromNodeIfValid(*leaves)) != NULL) {
                    cleanByEnsureUsedBlobs(newHypothesis);
                    currentHypothesis->newHypotheses.insert(newHypothesis);
                    g_NumLocallyAddedHypotheses++;
                }

                (*leaves)->added = true;
                leaves++;
                if(leaves == g_leaves.end())
                    break;
            } while( best == (*leaves)->value );
        }
    }
}

double RMMTracker::getTentativeHypothesisProbabilityValue(std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator *mobileIterators) {

    int i, sum = 0, num;
    double total = 0.0;
    for(i=0; i<g_numberOfNewMobiles; i++) {
        g_variableContribution[i] = 0;
        bestHypothesesNode::variablesNumFrames[i] = 0;
        if(g_numberOfNewMobileVersions[i] > 0) {
            sum +=  bestHypothesesNode::variablesNumFrames[i] = num = (*mobileIterators[i])->getNumberOfFramesSinceFirstTimeSeen();
            total += g_variableContribution[i] = num * (*mobileIterators[i])->getGlobalProbability();
        }
    }

    for(i=0; i<g_numberOfNewMobiles; i++)
        g_variableContribution[i] /= (double) sum;

    bestHypothesesNode::variablesSum = sum;

    return total/(double) sum;
}

SpRMMHypothesis RMMTracker::getHypothesisFromNodeIfValid(SpBestHypothesesNode node) {
    int i;
    //ACAAA!!! Tratar coherencia global
    bool acceptable3DCoherenceForMobile[g_numberOfNewMobiles];

    g_mobileIterators = node->mobileIterators;

    for(i=0; i<g_numberOfNewMobiles; i++) {
//        if(bestHypothesesNode::variablesNumFrames[i] > 0)
//            acceptable3DCoherenceForMobile[i] = (*(g_mobileIterators[i]))->mobile3DCoherenceIsAcceptable();
//        else
            acceptable3DCoherenceForMobile[i] = true;
    }
    //g_acceptable3DCoherenceForMobile = acceptable3DCoherenceForMobile;
    g_mobileVersionIndex = node->versionIndex;

    if(!mobilesCombinationIsValid())
        return QSharedPointer<RMMHypothesis>();

    SpRMMMobileObject currentMobile;
    SpRMMHypothesis newHypothesis(new RMMHypothesis());
    int j;

    newHypothesis->initUsedBlobsList();

    for(i=0; i<g_numberOfNewMobiles; i++) {
        if(bestHypothesesNode::variablesNumFrames[i]>0) {
            currentMobile = *(g_mobileIterators[i]);
            newHypothesis->insertNewMobileObject(currentMobile);
            for(j=0; j < blobsNumber; j++)
                newHypothesis->usedBlobs[j] |= currentMobile->usedBlobs[j];
        }
    }

    newHypothesis->setHypothesisProbability();

    newHypothesis->numCurrentlyLost = g_NumCurrentlyLost;

    return newHypothesis;
}

void RMMTracker::cleanByEnsureUsedBlobs(SpRMMHypothesis newHypothesis) {
    std::deque<SpRMMMobileObject>::iterator mobile_it;
    bool ensureUsedBlobs[blobsNumber];
    int i, j, size, numEnsured = 0;

    memset(ensureUsedBlobs, false, blobsNumber*sizeof(bool));
    mobile_it = newHypothesis->begin();
    size = newHypothesis->size();

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

    for(mobile_it = newHypothesis->begin(); mobile_it != newHypothesis->end(); mobile_it++) {
        (*mobile_it)->toErase = false;
        if((*mobile_it)-> getBestType() == UNKNOWN)
            for(j=0; j<blobsNumber; j++)
                if(ensureUsedBlobs[j] && (*mobile_it)->usedBlobs[j]) {
                    (*mobile_it)->toErase = modified = true;
                    break;
                }
    }
    if(modified) {
        if(!newHypothesis->empty()) {
            SpRMMHypothesis goodMobilesCopy(new RMMHypothesis());
            for(mobile_it = newHypothesis->begin(); mobile_it != newHypothesis->end(); mobile_it++)
                if((*mobile_it)->toErase == false)
                    goodMobilesCopy->insertNewMobileObject(*mobile_it);

            newHypothesis->clear();
            newHypothesis->mobiles = goodMobilesCopy->mobiles;
            goodMobilesCopy->clear();
        }
        newHypothesis->setHypothesisProbability();
    }
}


void RMMTracker::generateHypothesesWithBestSolutionsTree() {
    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypothesis_it, newhyp_it;
    std::map<long int, std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator> >::iterator newObj_it;
    int i, numberOfMobiles, totalNumMobileVersions;
    SpRMMHypothesis currentHypothesis;

    //for output
    std::deque<SpRMMMobileObject>::iterator mobile_it;

    if(hypothesisSets.empty())
        return;

    //Generate hypotheses for each mobile
    for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
        (*hset_it)->initUsedBlobsList();

        for(m_hypothesisNumber = 0, hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); m_hypothesisNumber++, hypothesis_it++) {

            currentHypothesis = (*hypothesis_it);
            currentHypothesis->newHypotheses.clear();
            g_NumLocallyAddedHypotheses = 0;

            //Second proposed method (smart method: check hypotheses as a tree, immediatelly storing best alternatives and generating
            //leaves according to best found solutions in previous leaves).
            //Useful when the maximum number of hypotheses retained is less than the total possible number of mobile combinations.

            numberOfMobiles = currentHypothesis->newObjectsList.size();

            int numMobileVersions[numberOfMobiles], numFramesVariables[numberOfMobiles];
            std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator setBegins[numberOfMobiles], setEnds[numberOfMobiles], mobileIterators[numberOfMobiles];
            double variableContribution[numberOfMobiles];

            g_newObjectsList = &currentHypothesis->newObjectsList;
            g_NumCurrentlyLost = currentHypothesis->numCurrentlyLost;
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
            bestHypothesesNode::numVariables = numberOfMobiles;
            bestHypothesesNode::newObjectsListEnds = setEnds;
            bestHypothesesNode::variablesNumFrames = numFramesVariables;

            g_leaves.clear();
            g_variableContribution = variableContribution;
            SpBestHypothesesNode root(new bestHypothesesNode(getTentativeHypothesisProbabilityValue(g_setBegins), 0));
            memcpy(root->mobileIterators, g_setBegins, sizeof(std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator) * numberOfMobiles );
            memcpy(root->variableContribution, g_variableContribution, sizeof(double) * numberOfMobiles );

            root->added = true;

            g_leaves.insert(root);

            SpRMMHypothesis newHypothesis;

            /*	std::cout << "\tFor Hypothesis of probability " << currentHypothesis->getProbability() << " containing:" << std::endl;
            mobile_it = currentHypothesis->begin();
            size = currentHypothesis->size();
            for(i=0; i< size; i++, mobile_it++)
                std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
                          << " of type "          << get_name_from_type((*mobile_it)->getBestType())
                          << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
            */
            if((newHypothesis = getHypothesisFromNodeIfValid(root)) != NULL) {
                cleanByEnsureUsedBlobs(newHypothesis);
                currentHypothesis->newHypotheses.insert(newHypothesis);
                g_NumLocallyAddedHypotheses++;
            }

            buildNewHypothesesFromLeaves(currentHypothesis);

            /*
            size = g_newHypotheses.size();
            newhyp_it = g_newHypotheses.begin();
            for(i=0; i< size; i++, newhyp_it++) {
                std::cout << "\tHypothesis " << i << " (P: " << (*newhyp_it)->getProbability() << ") containing:" << std::endl;
                mobile_it = (*newhyp_it)->begin();
                size2 = (*newhyp_it)->size();
                for(j=0; j < size2; j++, mobile_it++)
                    std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
                              << " of type "          << get_name_from_type((*mobile_it)->getBestType())
                              << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
            }
            std::cout << std::endl;
            */
        }

        //Filter repeated hypothesess between different sets of new hypotheses
        //      filterRepeatedHypotheses(*hset_it);

        //Insert remaining hypotheses for mobile
        g_newHypotheses.clear();
        for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++) {
            if(g_newHypotheses.empty())
                g_newHypotheses = (*hypothesis_it)->newHypotheses;
            else {
                std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator new_it, new_end = (*hypothesis_it)->newHypotheses.end();
                for(new_it = (*hypothesis_it)->newHypotheses.begin(); new_it != new_end; new_it++)
                    g_newHypotheses.insert(*new_it);
            }
        }
        SpRMMHypothesis bestPrevious = *(*hset_it)->begin();
        (*hset_it)->clear();

        if((int)g_newHypotheses.size() > m_maximumRetainedHypotheses) {
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator ending_iterator;
            for(i=0, ending_iterator = g_newHypotheses.begin(); i < m_maximumRetainedHypotheses; i++, ending_iterator++);

            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator new_it;
            ++ending_iterator;
            for(new_it = g_newHypotheses.begin(); new_it != ending_iterator; new_it++)
                (*hset_it)->insert(*new_it);
        } else {
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator new_it, new_end = g_newHypotheses.end();
            for(new_it = g_newHypotheses.begin(); new_it != new_end; new_it++)
                (*hset_it)->insert(*new_it);
        }

        //std::cout << "\tAfter Solutions Elimination:" << std::endl;
        //size = (*hset_it)->size();
        //newhyp_it = (*hset_it)->begin();
        //for(i=0; i< size; i++, newhyp_it++) {
        //std::cout << "\tHypothesis " << i << " (P: " << (*newhyp_it)->getProbability() << ") containing:" << std::endl;
        //mobile_it = (*newhyp_it)->begin();
        //size2 = (*newhyp_it)->size();
        //for(j=0; j < size2; j++, mobile_it++)
        //  std::cout << "\t\tMobile "       << (*mobile_it)->getMobileId()
        //	    << " of type "          << get_name_from_type((*mobile_it)->getBestType())
        //	    << " and probability " << (*mobile_it)->getGlobalProbability() << std::endl;
        //}
        //std::cout << std::endl;

        if((*hset_it)->size() == 0) {
            numberOfMobiles = bestPrevious->newObjectsList.size();
            g_newObjectsList = &bestPrevious->newObjectsList;

            int j;
            SpRMMHypothesis newHypothesis(new RMMHypothesis());
            newHypothesis->initUsedBlobsList();

            SpRMMMobileObject currentMobile;
            for(i = 0, newObj_it = g_newObjectsList->begin(); i<numberOfMobiles; i++, newObj_it++) {
                currentMobile = *newObj_it->second.begin();
                newHypothesis->insertNewMobileObject(currentMobile);
                for(j=0; j < blobsNumber; j++)
                    newHypothesis->usedBlobs[j] |= currentMobile->usedBlobs[j];
            }

            newHypothesis->setHypothesisProbability();
            (*hset_it)->insert(newHypothesis);

        }

        (*hset_it)->setBestHypothesis();

        //Set used blobs lists
        (*hset_it)->numUsed = 0;
        for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++) {
            (*hypothesis_it)->numUsed = 0;
            for(i=0; i<blobsNumber; i++) {
                if((*hypothesis_it)->usedBlobs[i]) {
                    (*hypothesis_it)->numUsed++;
                    if((*hset_it)->usedBlobs[i] == false) {
                        (*hset_it)->numUsed++;
                        (*hset_it)->usedBlobs[i] = true;
                    }
                }
            }
        }

        //Set incomplete hypotheses flags
        (*hset_it)->incompleteHypotheses = 0;
        for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++) {
            (*hypothesis_it)->incompleteHypothesis = false;
            if((*hypothesis_it)->numUsed < (*hset_it)->numUsed) {
                (*hypothesis_it)->incompleteHypothesis = true;
                (*hset_it)->incompleteHypotheses++;
            }
        }

        for(i=0; i<blobsNumber; i++)
            usedBlobs[i] |= (*hset_it)->usedBlobs[i];
    }
}

void RMMTracker::initValidityMatrices(int size) {
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


void RMMTracker::followExistingMobiles() {

    //1. Set involved blobs for each reliable mobile (implies involved blobs for each hypothesis and mobile)
    //   and global relations of blobs
    setInvolvedBlobs();

    //2. Merge reliable mobiles involved in non-empty intersection of involved blobs.
    mergeInvolvedHypothesisSets();

    //2. Create possible movements for each mobile
    createMobilePossibilities();

    //3. Generate Hypotheses with upper number limit and coherency bound
    generateHypothesesWithBestSolutionsTree();

    if(m_internalOutputActivated)
        std::cout << "After Path creation:" << std::endl << hypothesisSets;
}

void RMMTracker::setInvolvedBlobs() {
    int i;
    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypothesis_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> *hypotheses;
    std::deque<SpRMMMobileObject>::iterator mobile_it;

    for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
        (*hset_it) -> initInvolvedBlobs();
        hypotheses = (*hset_it)->getHypotheses();
        for(hypothesis_it = hypotheses->begin(); hypothesis_it != hypotheses->end(); hypothesis_it++) {
            (*hypothesis_it) -> initInvolvedBlobs();
            for(mobile_it = (*hypothesis_it)->begin(); mobile_it != (*hypothesis_it)->end(); mobile_it++) {
                (*mobile_it) -> initInvolvedBlobs();
                determineInvolvedBlobsForMobileObject(*mobile_it);
                for(i=0; i<blobsNumber; i++)
                    (*hypothesis_it)->involvedBlobs[i] |= (*mobile_it) -> involvedBlobs[i];
            }
            for(i=0; i<blobsNumber; i++)
                (*hset_it)->involvedBlobs[i] |= (*hypothesis_it) -> involvedBlobs[i];
        }

        for(i=0; i<blobsNumber; i++)
            if((*hset_it)->involvedBlobs[i])
                involvedHypothesisSetsCounter[i]++;
    }

    if(m_internalOutputActivated) {
        std::cout << "\nInvolved RMobiles counter:\n";
        for(i=0; i<blobsNumber; i++)
            std::cout << involvedHypothesisSetsCounter[i] << "\t";
        std::cout << std::endl;
    }
}

void RMMTracker::getNearest2DBlobPointToFocalPoint(int position, double xCenter, double yCenter, double W, double H, double *x, double *y) {
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

void RMMTracker::getNearest2DBlobPointToFocalPoint(int position, Rectangle<int> *rect, double *x, double *y) {

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

void RMMTracker::getNearest2DBlobPointToFocalPoint(Blob *blob, double *x, double *y) {

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
void RMMTracker::determineInvolvedBlobsForMobileObject(SpRMMMobileObject mobile) {
    int i, j, tosumw, tosumh;
    double R, R_;
    bool mergeList[blobsNumber];
    memset(mergeList, false, blobsNumber*sizeof(bool));
    mobile->iestimator = mobile->getVisualEvidenceEstimator(R);
    Rectangle<int> &bestimator = mobile->iestimator;
    R_ = 1.0 - R;

    //tosumw = R_*(   m_InvolvedVisualEvidenceMultiplyingFactor * bestimator.width
    //              + m_InvolvedVisualEvidenceSumFactor - bestimator.width)/2.0;
    //tosumh = R_*(   m_InvolvedVisualEvidenceMultiplyingFactor * bestimator.height
    //              + m_InvolvedVisualEvidenceSumFactor - bestimator.height)/2.0;
    tosumw = R_*(m_InvolvedVisualEvidenceMultiplyingFactor * bestimator.width)/2.0
           + m_InvolvedVisualEvidenceSumFactor;
    tosumh = R_*(m_InvolvedVisualEvidenceMultiplyingFactor * bestimator.height)/2.0
           + m_InvolvedVisualEvidenceSumFactor;

    bestimator.xleft -= tosumw;
    bestimator.xright += tosumw;
    bestimator.ytop -= tosumh;
    bestimator.ybottom += tosumh;
    bestimator.width = bestimator.xright - bestimator.xleft + 1;
    bestimator.height = bestimator.ybottom - bestimator.ytop + 1;

    for (i=0; i<blobsNumber; i++) {
        if(mobile->involvedBlobs[i] == false) { //blob not analyzed or included yet
            if(    bestimator.rectangleIntersectRatio(&blobsVector[i]->bbox) >= m_InvolvedVisualEvidenceMinIntersectionRatio
                || blobsVector[i]->bbox.rectangleIntersectRatio(&bestimator) >= m_InvolvedVisualEvidenceMinIntersectionRatio) {
                getMergeConnections(mergeList, i);
                for(j=0; j<blobsNumber; j++)
                    if(mergeList[j] == true)
                        mobile->involvedBlobs[j] = true;
            }
        }
    }

    for(j=0; j<blobsNumber; j++)
        if(mobile->involvedBlobs[j] == true)
            mobile->numInvolved++;

}

void RMMTracker::getHighest3DVariations(double x, double y, double X, double Y, double dX, double dY, double *dx, double *dy) {
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

void RMMTracker::getMergeConnections(bool *mergeList, int startingBlobIndex) {
    int i;

    memset(mergeList, false, sizeof(bool)*blobsNumber);
    mergeList[startingBlobIndex] = true;

    for(i=0; i<blobsNumber; i++)
        if(i != startingBlobIndex && initialMergeMap[startingBlobIndex][i])
            getRecursiveMergeConnections(mergeList, i);
}

void RMMTracker::getRecursiveMergeConnections(bool *mergeList, int blobIndex) {
    int i;

    if(mergeList[blobIndex])
        return;

    mergeList[blobIndex] = true;

    for(i = blobIndex + 1; i<blobsNumber; i++)
        if(initialMergeMap[blobIndex][i])
            getRecursiveMergeConnections(mergeList, i);
}


//TODO: Make difference with a MM_TOTAL_OCCLUSION situation.
SpRMMMobileObject RMMTracker::setNotVisibleBlobMobile(SpRMMMobileObject currentObject, DetectionProblemType dp_type) {

    SpRMMMobileObject specialMobile(new RMMMobileObject(currentObject));
    Blob *currentBlob = generateMostLikelyHypothesisForMobile(currentObject);
    bool noBest = false;

    if(currentBlob == NULL) {
        noBest = true;
        //ACAA
        //currentBlob = (currentObject->blobHistory.back())->copyWithLists();
        BLOB_TYPE(currentBlob) = UNKNOWN;
    }

    specialMobile->incrementNumberOfFramesNotSeen();

    //Blob's data is totally unreliable
    BLOB_R(currentBlob) = BLOB_RW(currentBlob) = BLOB_RL(currentBlob) = BLOB_RH(currentBlob) = 0;
    BLOB_DP_TYPE(currentBlob) = dp_type;

    specialMobile->updateMobilePath(currentBlob);

    if(noBest)
        delete currentBlob;

    return specialMobile;

}

SpRMMMobileObject RMMTracker::checkSpecialCases(SpRMMMobileObject currentMobile, Blob *blobToTest) {
    //First check if obtained solution does not respond sufficiently to predictions,
    double R;
    Rectangle<int> mobileBBox = currentMobile->getVisualEvidenceEstimator(R);

    double
        //Visual support is 1.0 when mobile bbox is completelly covered by the analysed blob
        support1 = Rectangle<int>::rectangleIntersectRatio(&mobileBBox, &(blobToTest->bbox)),
        //Visual support is 1.0 when analysed blob is completelly covered by the mobile bbox
        support2 = Rectangle<int>::rectangleIntersectRatio(&(blobToTest->bbox), &mobileBBox);
    //If both support values are big, it means that used 2D bounding box fits to predictions
    if(support1 >= m_highVisualSupportThreshold && support2 >= m_highVisualSupportThreshold)
        return QSharedPointer<RMMMobileObject>();

    //Else check for special cases:
    //1. The 2D width, and 2D height of both blob and prediction are similar, or fit to
    //   expected change but, the coverage is not good, it could be:
    //   a. that the visual evidence is displaced with respect to the expected position, so
    //      this case is normal and can be considered as covered by the new mobile.
    double
        Wdiff = abs(RECT_WIDTH(&mobileBBox)  - BLOB_WIDTH(blobToTest)),
        Hdiff = abs(RECT_HEIGHT(&mobileBBox) - BLOB_HEIGHT(blobToTest)),
        Wtol, Htol;

    //ACAAAA!!
    //currentMobile->getMobile3DTolerances(&Wtol, &Htol);
    //if(Wdiff < Wtol && Hdiff < Htol)
    //    return QSharedPointer<RMMMobileObject>();

    //   b. TODO: Weird case:
    //      The object is partially detected and another object is detected at the same place
    //      making seem similar to the expected size. This case should be analysed at hypothesis
    //      generation level, but it is not a priority in implementation.

    //2. There exist a remarkable difference in expected size of visual evidence. Depending on these differences
    //   different cases can arise:
    //   a.
    SpRMMMobileObject specialMobile;
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


SpRMMMobileObject RMMTracker::generateAndValidateNewMobile(SpRMMMobileObject currentMobile, Blob *blobToTest) {

    SpRMMMobileObject newMobile(new RMMMobileObject(currentMobile));
    Rectangle<int> mobileBBox;

    newMobile->updateMobilePath(blobToTest);

    if(incoherentMobile(newMobile)) // || (!newMobile->ensureMode && !acceptableInformationForNewMobile(newMobile, currentMobile)))
        return QSharedPointer<RMMMobileObject>();

    return newMobile;
}

SpRMMMobileObject RMMTracker::setSpecialCaseMobile(DetectionProblemType dp_type, SpRMMMobileObject currentObject, Blob *blobToAdd) {

    SpRMMMobileObject specialMobile(new RMMMobileObject(currentObject));

    Blob *blobForMobile = blobToAdd->copyWithLists();

    if(dp_type == MM_OBJECT_LOST) {
        specialMobile->incrementNumberOfFramesNotSeen();
        //Blob's 3Ddata is totally unreliable
        BLOB_R(blobForMobile) = BLOB_RW(blobForMobile) = BLOB_RL(blobForMobile) = BLOB_RH(blobForMobile) = 0.0;
    }

    BLOB_DP_TYPE(blobForMobile) = dp_type;

    specialMobile->updateMobilePath(blobForMobile);

    delete blobForMobile;

    return specialMobile;
}

//Function for generation of new mobiles, using tracking information. This function takes into account
//an initial solution to serve as starting point of future merges of involved blobs with currently analyzed mobile.
//Solutions
void RMMTracker::generateMobilePathFromInitialBlob(SpRMMMobileObject mobile, Blob *initialBlob, int numUsed, bool *usedBlobs, double *blobSupport, Rectangle<int> *mobileBBox) {

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

        int hypothesesCombo[blobsToMergeNumber];
        for(j=1; j<=validBlobsNumber; j++)
            generateHypothesesForMobilePathFromInitialBlob(j, 0, 0, hypothesesCombo, blobsToMergeNumber, blobsToMerge,
                                                             mobile, initialBlob, numUsed, usedBlobs, mobileBBox);
    }
}


void RMMTracker::generateMobilePath(SpRMMMobileObject mobile) {
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

            int hypothesesCombo[blobsToMergeNumber];
            for(j=1; j<=validBlobsNumber; j++)
                generateHypothesesForMobilePath(j, 0, 0, hypothesesCombo, blobsToMergeNumber, blobsToMerge, mobile);
        }
    }
}


bool RMMTracker::incoherentMobile(SpRMMMobileObject mobile) {
//ACA!!!!
/*    bool
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
        && ( mobile->getGlobalProbability() >= g_bestGlobalP*ImportanceRateForBestMobileHypothesisProbability) )
        return false;*/
    return false;
}


bool RMMTracker::blobAlreadyIncludedInNewMobilesSet(Blob *blob) {
    std::deque<Blob *>::iterator blob_it = g_includedBlobsInNewMobiles.begin();
    int i, size = g_includedBlobsInNewMobiles.size();

    for(i=0; i < size; i++, blob_it++)
        if(Blob::same2DBlob(blob, *blob_it))
            return true;

    return false;
}

void RMMTracker::generateHypothesesForMobilePathFromInitialBlob(int length, int position, int value, int *hypothesesCombo,
                                                                          int blobsToMergeNumber, int *blobsToMerge, SpRMMMobileObject mobile,
                                                                          Blob *initialBlob, int numUsed, bool *usedBlobs, Rectangle<int> *mobileBBox) {

    int i;

    if(position == length) {

        //Generate the blob hypothesis for mobile
        if(m_internalOutputActivated) {
            std::cout << "For Mobile: " <<mobile->getMobileId() << std::endl;
            for(i=0; i < length; i++)
                std::cout << hypothesesCombo[i] << " ";
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
                    curRect = &(blobsVector[hypothesesCombo[i]]->bbox);
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
                curr_blob = blobsVector[hypothesesCombo[minAreaIndex]];
                currently_used[minAreaIndex] = true;
            } else {
                curr_blob = blobsVector[hypothesesCombo[minDistanceIndex]];
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
        SpRMMMobileObject newMobile = generateAndValidateNewMobile(mobile, merged_blob);

        bool specialCase = false;

        if (newMobile == NULL) {
            if(mobile->ensureMode || mobile->numberOfFramesSinceFirstTimeSeen > 2*RMMMobileObject::m_blobsBufferSize) {
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
            newMobile->usedBlobs[hypothesesCombo[i]] = true;
        newMobile->numUsed = length + numUsed;

        if(specialCase)
            g_newSpecialMobiles.insert(newMobile);
        else
            g_newMobiles.insert(newMobile);
    } else {
        int limit = blobsToMergeNumber - length + position;
        for(i=value; i <= limit; i++) {
            hypothesesCombo[position] = blobsToMerge[i];
            generateHypothesesForMobilePathFromInitialBlob(length, position+1, i+1, hypothesesCombo,
                                                             blobsToMergeNumber, blobsToMerge, mobile,
                                                             initialBlob, numUsed, usedBlobs, mobileBBox);
        }
    }
}


void RMMTracker::generateHypothesesForMobilePath(int length, int position, int value, int *hypothesesCombo,
                                                           int blobsToMergeNumber, int *blobsToMerge, SpRMMMobileObject mobile) {
    int i, j;

    if(position == length) {

        //Generate the blob hypothesis for mobile
        if(m_internalOutputActivated) {
            std::cout << "For Mobile: " <<mobile->getMobileId() << std::endl;
            for(i=0; i < length; i++)
                std::cout << hypothesesCombo[i] << " ";
            std::cout << std::endl;
        }

        //Check blobs proximity
        Blob *curr_blob;
        bool blobTooFar = false, allfar;
        double distance, meanDim[length];

        if(length > 1) {
            for(i=0; i < length; i++) {
                curr_blob = blobsVector[hypothesesCombo[i]];
                meanDim[i] = (BLOB_WIDTH(curr_blob) + BLOB_HEIGHT(curr_blob)) / 2.0;
            }

            for(i=0; i < length; i++) {
                allfar = true;
                for(j=0; j < length; j++) {
                    if(i != j) {
                        distance = Rectangle<int>::rectangleDistance(&(blobsVector[hypothesesCombo[i]]->bbox), &(blobsVector[hypothesesCombo[j]]->bbox));
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

        //ACA Arreglar el metodo de merge

/*        if(length > 1)
            //Classification is controlled at RMMMobileObject level
            merged_blob = m_RMerge->getMergedBlob(hypothesesCombo, length, blobsVector, blobsNumber, false);
        else*/
        merged_blob = blobsVector[hypothesesCombo[0]];

        if(blobAlreadyIncludedInNewMobilesSet(merged_blob))
            return;

        g_includedBlobsInNewMobiles.push_front(merged_blob);

        //Generate new mobile
        SpRMMMobileObject newMobile(new RMMMobileObject(mobile));
        newMobile->updateMobilePath(merged_blob);

        //If minimal coherence is not achieved, solution will not be taken into account
        if(incoherentMobile(newMobile))
            return;

        if (newMobile->getGlobalProbability() > g_bestGlobalP)
            g_bestGlobalP = newMobile->getGlobalProbability();

        //Set used blobs for mobile
        for(i=0; i < length; i++)
            newMobile->usedBlobs[hypothesesCombo[i]] = true;
        newMobile->numUsed = length;

        g_newMobiles.insert(newMobile);
        mobile->accepted_solution = true;

    } else {
        int limit = blobsToMergeNumber - length + position;
        for(i=value; i <= limit; i++) {
            hypothesesCombo[position] = blobsToMerge[i];
            generateHypothesesForMobilePath(length, position+1, i+1, hypothesesCombo,
                                              blobsToMergeNumber, blobsToMerge, mobile);
        }
    }
}

Blob *RMMTracker::generateMostLikelyHypothesisForMobile(SpRMMMobileObject mobile) {
    return mobile->determineMostLikelyBlob();
}

void RMMTracker::eliminateUnlikelyMobiles() {
    //for each reliable mobile do
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypIt;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>* hypotheses;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> remainingHypotheses;
    std::deque<SpRMMHypothesisSet>::iterator it;
    int n = m_maximumRetainedHypotheses;
    bool changed;
    for(it = hypothesisSets.begin(); it != hypothesisSets.end();it++){
        hypotheses = (*it)->getHypotheses();
        remainingHypotheses.clear();

        changed = false;

        //      std::cout << "Number of hypotheses before: " << hypotheses->size() << std::endl;

        double sum = 0, bestP = (*it)->getBestProbability();
        int N = hypotheses->size();
        //if the probability of a hypothesis solution is less than a threshold remove it
        hypIt = hypotheses->begin();
        remainingHypotheses.insert(*hypIt);
        for(hypIt++;hypIt!=hypotheses->end();hypIt++){
            if( (*hypIt)->justFirstFrames ) {
                remainingHypotheses.insert(*hypIt);
                N--;
            } else if( (*hypIt)->getProbability()/ bestP < hypothesesProbabilityThreshold ){
                changed = true;
                N--;
            } else {
                sum += (*hypIt)->getProbability();
                remainingHypotheses.insert(*hypIt);
            }
        }

        //If modified, regenerate the list
        if(changed) {
            hypotheses->clear();
            *hypotheses = remainingHypotheses;
            //	std::cout << "Number of stored hypotheses: " << remainingHypotheses.size() << std::endl;
        }

        //      std::cout << "Number of hypotheses after: " << hypotheses->size() << std::endl;


        //Now retain only most reliable solutions if necessary according to the elimination strategy
        //Three strategy are supported : Least Reliable Elimination and Maximun Likelihood Estimator Elimination

        if(N > n){
            if(eliminationStrategy=="LeastReliable"){
                //As now hypotheses are ordered by Probability, is not necessary to order them
                if(N > n) {
                    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypIt2;
                    hypIt = (**it)[n];
                    for(hypIt2 = hypIt, hypIt2++; hypIt2 != hypotheses->end(); hypIt2++)
                        if((*hypIt2)->justFirstFrames) //If there is just new elements do not erase (they have P = 0.0 so
                                                      //they will be at the end of the list).
                            break;

                    hypotheses->erase(++hypIt, hypIt2);
                }
            } else if(eliminationStrategy=="MaximumLikelihoodEstimator"){
                //compute the mean
                double mean = sum / N;
                //compute the variance
                double sigma = 0, pi;
                for(hypIt=hypotheses->begin();hypIt!=hypotheses->end();hypIt++){
                    pi = (*hypIt)->getProbability();
                    sigma += (pi - mean)*(pi -mean);
                }
                sigma /= N - 1;
                sigma = std::sqrt(sigma);
                //Eliminate solutions behind mean +/- standard deviation
                for(hypIt=hypotheses->begin();hypIt!=hypotheses->end();){
                    double pi = (*hypIt)->getProbability();
                    //The elements are ordered by probability
                    if(pi<mean-sigma && !(*hypIt)->justFirstFrames) {
                        std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypIt2;
                        for(hypIt2 = hypIt, hypIt2++; hypIt2 != hypotheses->end(); hypIt2++)
                            if((*hypIt2)->justFirstFrames) //If there is just new elements do not erase (they have P = 0.0 so
                                                          //they will be at the end of the list).
                                break;
                        hypotheses->erase(hypIt, hypIt2);
                        break;
                    } else if ((*hypIt)->justFirstFrames)
                        break;
                    else
                        hypIt++;
                }
            }   else if(eliminationStrategy != "None") //Bad definition
                std::cout << "Elimination Strategy '" << eliminationStrategy.toStdString() << "' not defined. Skipping hypothesis elimination process";
        }
    }
}

void RMMTracker::computeInclusiveProbabilities(std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>* hypotheses,int k,double sum,bool *mark,double *piT) {

    double newSum = sum;
    bool overflow = false;
    int i;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypIt;
    for(i = 0,hypIt=hypotheses->begin();hypIt != hypotheses->end();hypIt++,i++){
        if(mark[i]) continue;
        piT[i] = (k * (*hypIt)->getProbability()) / sum;
        if(piT[i] > 1) {
            overflow = true;
            mark[i] = true;
            piT[i] = 1;
            newSum -= (*hypIt)->getProbability();
        }
    }
    if(overflow)
        computeInclusiveProbabilities(hypotheses,k,newSum,mark,piT);
}

void RMMTracker::resetPauseCriterias() {
    m_pauseOnLost = m_pauseOnNew = false;
}


void RMMTracker::run(std::vector<Blob>& blobs) {
#ifdef RMTT_OUTPUT
    std::cout << "Tracker en frame: " << m_data->frameNumber << std::endl;
    std::cout << "Before processing:" << std::endl << hypothesisSets;
#endif
    std::cerr << "Frame: " << m_data->frameNumber << std::endl;

//    if(m_data->frameNumber == 951)
//        std::cerr << "Aquí nos detenemos!!" << std::endl;
    resetPauseCriterias();


    setCurrentTimeAndFrame();

    preMerge(blobs);

    setBlobsVector(blobs);

    //TODO: To implement when a good point tracker exists. :D
    //new_blobs = preSplit (new_blobs);


    if(m_data->frameNumber == 671)
        std::cout << "Aca paramos:" << m_data->frameNumber
                  << std::endl;

   update();
#ifdef RMTT_OUTPUT
   std::cout << "After updating:" << std::endl << hypothesisSets;
#endif

//    if(m_internalOutputActivated)
//        std::cout << "After New Mobiles:" << std::endl << hypothesisSets;

/*    std::deque<SpRMMHypothesis>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hyp_it, alt_end;

    int i, j;
    if(hypothesisSets.size()>0) {
        for(hset_it = hypothesisSets.begin(), j=0; hset_it != hypothesisSets.end(); hset_it++, j++) {
                alt_end = (*hset_it)->end();
                for(hyp_it = (*hset_it)->begin(), i=0; hyp_it != alt_end; hyp_it++, i++);
                std::cout << "Mobile " << j << " before eliminateUnlikelyMobiles - Hypotheses: " << (*hset_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/


    eliminateUnlikelyMobiles();
#ifdef RMTT_OUTPUT
    std::cout << "After eliminateUnlikelyMobiles:" << std::endl << hypothesisSets;
#endif

/*    if(hypothesisSets.size()>0) {
        for(hset_it = hypothesisSets.begin(), j=0; hset_it != hypothesisSets.end(); hset_it++, j++) {
                alt_end = (*hset_it)->end();
                for(hyp_it = (*hset_it)->begin(), i=0; hyp_it != alt_end; hyp_it++, i++);
                std::cout << "Mobile " << j << " before filterUnseenMobiles - Hypotheses: " << (*hset_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/


    filterUnseenMobiles();
#ifdef RMTT_OUTPUT
    std::cout << "After filterUnseenMobiles:" << std::endl << hypothesisSets;
#endif
/*    if(hypothesisSets.size()>0) {
        for(hset_it = hypothesisSets.begin(), j=0; hset_it != hypothesisSets.end(); hset_it++, j++) {
                alt_end = (*hset_it)->end();
                for(hyp_it = (*hset_it)->begin(), i=0; hyp_it != alt_end; hyp_it++, i++);
                std::cout << "Mobile " << j << " before filterEquallyConvergedMobiles - Hypotheses: " << (*hset_it)->size()<< ", Real: " << i << std::endl;
        }
    }*/
    filterEquallyConvergedMobiles();
#ifdef RMTT_OUTPUT
    std::cout << "After filterEquallyConvergedMobiles:" << std::endl << hypothesisSets;
#endif

/*    if(hypothesisSets.size()>0) {
        for(hset_it = hypothesisSets.begin(), j=0; hset_it != hypothesisSets.end(); hset_it++, j++) {
                alt_end = (*hset_it)->end();
                for(hyp_it = (*hset_it)->begin(), i=0; hyp_it != alt_end; hyp_it++, i++);
                std::cout << "Mobile " << j << " before filterRepeatedHypotheses - Hypotheses: " << (*hset_it)->size()<< ", Real: " << i << std::endl;
        }
    }
    */

    filterRepeatedHypotheses();
#ifdef RMTT_OUTPUT
    std::cout << "After filterRepeatedHypotheses:" << std::endl << hypothesisSets;
#endif

    separateReliableHypotheses();
#ifdef RMTT_OUTPUT
    std::cout << "After separateReliableHypotheses:" << std::endl << hypothesisSets;
#endif

    freeBlobsVector();

//    if(m_internalOutputActivated)
  //      std::cout << "After Elimination:" << std::endl << hypothesisSets;

    activatePause();

    RMMMobileObject::m_firstFrame = firstFrame = false;
}


//bool RMMTracker::equal2DDimensions(SpRMMMobileObject m1, SpRMMMobileObject m2) {
//ACAAAAAAA: Generalizar

    /*    if(    fabs(m1->t2DDimData.W - m2->t2DDimData.W) <= acceptedPixelError
        && fabs(m1->t2DDimData.H - m2->t2DDimData.H) <= acceptedPixelError
        && fabs(m1->t2DSpatialData.X - m2->t2DSpatialData.X) <= acceptedPixelError
        && fabs(m1->t2DSpatialData.Y - m2->t2DSpatialData.Y) <= acceptedPixelError )
        return true;*/

//    return false;
//}

void RMMTracker::activatePause() {
    if(    (m_activatePauseOnLost && m_pauseOnLost)
        || (m_activatePauseOnNew  && m_pauseOnNew) )
           pauseApp();
}

void RMMTracker::filterEquallyConvergedMobiles() {

    SpRMMHypothesisSet hset;
    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hyp_it, alt_end;
    std::deque<SpRMMMobileObject>::iterator mobile_it1, mobile_it2, mobiles_end, bestEqual;
    std::deque<std::deque<SpRMMMobileObject>::iterator> equalMobiles;
    std::deque<std::deque<SpRMMMobileObject>::iterator>::iterator equal_it;
    int numHypotheses, numMobiles, i;
    double bestP;
    int maxNumberOfFrames;

    for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
        hset = (*hset_it);
        alt_end = hset->end();
        numHypotheses = hset->size();

        for(hyp_it = hset->begin(); hyp_it != alt_end; hyp_it++) {
            numMobiles = (*hyp_it)->size();
            if(numMobiles == 1)
                continue;
            //Check if there is equal mobiles in the same alternative
            mobiles_end = (*hyp_it)->end();
            //Initialize equality and deletion flags
            for(mobile_it1 = (*hyp_it)->begin(), i=0; i < numMobiles; mobile_it1++, i++)
                (*mobile_it1)->comparedMobile = (*mobile_it1)->toErase = false;

            //Analyze equal pairs
            for(mobile_it1 = (*hyp_it)->begin(), i=0; i < numMobiles-1; mobile_it1++, i++) {
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

                    //To consider two mobiles equal, first they have to come from the same origin (same hset_id)
                    //Then they have also to use the same visual evidence in current frame, or to have a very
                    //high overlapping on their 2D features.
                    if(    (    (*mobile_it1)->getHypothesisSetId() == (*mobile_it2)->getHypothesisSetId()
                             && ( sameUsed(*mobile_it1, *mobile_it2) || highCoverage(*mobile_it1, *mobile_it2) ) )
                        || (    (*mobile_it1)->getHypothesisSetId() != (*mobile_it2)->getHypothesisSetId()
                             && sameUsed(*mobile_it1, *mobile_it2)
                             && highCoverage(*mobile_it1, *mobile_it2) ) ) {
                        //ACAAAA!!! Generalizar
                             //&& equal2DDimensions(*mobile_it1, *mobile_it2) ) ) {
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
            for(mobile_it1 = (*hyp_it)->begin(); mobile_it1 != (*hyp_it)->end(); ) {
                if( (*mobile_it1)->toErase )
                    mobile_it1 = (*hyp_it)->erase(mobile_it1);
                else
                    mobile_it1++;
            }
        }
    }
}


void RMMTracker::filterRepeatedHypotheses(SpRMMHypothesisSet hset) {

    int numHypotheses = hset->size();
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator *alternativesToCompare[numHypotheses],
        hyp_it, rnewhyp_it, alt_end = hset->end(),
        rsec_it, rnewsec_it;

    bool notCompletelyChecked = true, newFound, moreThanOneToCompare;
    SpRMMHypothesis currentOldAlternative, currentnewHypothesis;
    int currentFirst, newSize, currentNew, currentSecond, numToCompare, bestIndex;
    double best;
    hyp_it = hset->begin();
    currentFirst = 0;

    while(notCompletelyChecked) {

        //First, set the first alternative to compare, not yet compared
        newFound = false;
        for(; hyp_it != alt_end; hyp_it++, currentFirst++) {
            newSize = (*hyp_it)->newHypotheses.size();
            if(newSize > 0 && (*hyp_it)->numSurvived < newSize)
                for(rnewhyp_it = (*hyp_it)->newHypotheses.begin(), currentNew = 0; currentNew < newSize; currentNew++, rnewhyp_it++)
                    if(!(*rnewhyp_it)->survivedToComparison) {
                        newFound = true;
                        break;
                    }
            if(newFound)
                break;
        }

        //Second, to fill the list with equivalent alternatives generated from other alternatives
        if(newFound) {
            alternativesToCompare[currentFirst] = &rnewhyp_it;
            bestIndex = currentFirst;
            best = (*rnewhyp_it)->getProbability();
            moreThanOneToCompare = false;
            numToCompare = 1;

            rsec_it = hyp_it;
            for(rsec_it++, currentSecond = currentFirst+1; rsec_it != alt_end; rsec_it++, currentSecond++) {
                newSize = (*rsec_it)->newHypotheses.size();
                newFound = false;
                if(newSize > 0 && (*rsec_it)->numSurvived < newSize)
                    for(rnewsec_it = (*rsec_it)->newHypotheses.begin(), currentNew = 0; currentNew < newSize; currentNew++, rnewsec_it++)
                        if(!(*rnewsec_it)->survivedToComparison && equalMobiles(*rnewhyp_it, *rnewsec_it)) {
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
                rsec_it = hyp_it;
                for(currentSecond = currentFirst; rsec_it != alt_end; rsec_it++, currentSecond++)
                    if(currentSecond != bestIndex && alternativesToCompare[currentSecond] != NULL)
                        (*rsec_it)->newHypotheses.erase(*alternativesToCompare[currentSecond]);
                    else if(currentSecond == bestIndex) { //Save the best element
                        (**alternativesToCompare[currentSecond])->survivedToComparison = true;
                        (*rsec_it)->numSurvived++;
                    }
            } else {
                (*rnewhyp_it)->survivedToComparison = true;
                (*hyp_it)->numSurvived++;
            }
        } else
            notCompletelyChecked = false;
    }

}


void RMMTracker::filterRepeatedHypotheses() {

    SpRMMHypothesisSet hset;
    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hyp_it1, hyp_it2, bestEqual, hyp_end;
    std::deque<std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator> equalHypotheses;
    std::deque<std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator>::iterator equal_it;
    int numHypotheses, i, j;
    double bestP;
    bool changed;

    if(m_data->frameNumber == 311)
        std::cerr << "Aqui paramos: " << m_data->frameNumber << std::endl;

    j=-1;
    for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
        hset = *hset_it;
        j++;
        if((numHypotheses = hset->size()) == 1)
            continue;
        //std::cout << "Alternatives mobile " << j << ": " << numHypotheses << std::endl;
        hyp_end = hset->end();

        for(hyp_it1 = hset->begin(), i=0; hyp_it1 != hyp_end; hyp_it1++, i++);
        //std::cout << "In filterRepeatedHypotheses - Real Alternatives: " << i << std::endl;

        changed = false;

        for(hyp_it1 = hset->begin(); hyp_it1 != hyp_end; hyp_it1++)
            (*hyp_it1)->toEliminate = (*hyp_it1)->comparedHypothesis = false;

        for(hyp_it1 = hset->begin(), i=0; i < numHypotheses-1; hyp_it1++, i++) {
            if((*hyp_it1)->comparedHypothesis || (*hyp_it1)->toEliminate)
                continue;
            equalHypotheses.clear();
            (*hyp_it1)->comparedHypothesis = true;
            bestP = (*hyp_it1)->getProbability();
            bestEqual = hyp_it1;
            equalHypotheses.push_back(hyp_it1);
            for(hyp_it2 = hyp_it1, hyp_it2++; hyp_it2 != hyp_end; hyp_it2++) {
                if((*hyp_it2)->comparedHypothesis || (*hyp_it1)->toEliminate)
                    continue;
                if(equalMobiles(*hyp_it1, *hyp_it2)) {
                    equalHypotheses.push_back(hyp_it2);
                    (*hyp_it2)->comparedHypothesis = true;
                    if(bestP < (*hyp_it2)->getProbability()) {
                        bestP = (*hyp_it2)->getProbability();
                        bestEqual = hyp_it2;
                    }
                }
            }

            //Analyzed hypothesis was different to all the others
            if(equalHypotheses.size() <= 1)
                continue;

            //Mark Alternatives to Eliminate
            for(equal_it = equalHypotheses.begin(); equal_it != equalHypotheses.end(); equal_it++)
                if(*equal_it != bestEqual) {
                    (*(*equal_it))->toEliminate = true;
                    changed = true;
                }
        }

        //Eliminate marked hypotheses
        if(changed) {
            std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> newHypotheses;
            for(hyp_it1 = hset->begin(), i=0; i < numHypotheses; hyp_it1++, i++)
                if((*hyp_it1)->toEliminate == false)
                    newHypotheses.insert(*hyp_it1);

            hset->hypotheses.clear();
            hset->hypotheses = newHypotheses;
        }
    }
}

bool RMMTracker::equalMobiles(SpRMMHypothesis alt1, SpRMMHypothesis alt2) {

    //If different number of mobiles they are different
    if(alt1->size() != alt2->size())
        return false;

    std::deque<SpRMMMobileObject>::iterator m1_it, m2_it;
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

bool RMMTracker::highCoverage(SpRMMMobileObject m1, SpRMMMobileObject m2) {
    Rectangle<int> bb1, bb2;

    //ACA!!!
    //Unificar

    //bb1.initRectangle((int)m1->t2DSpatialData.X, (int)m1->t2DSpatialData.Y, (int)m1->t2DDimData.W, (int)m1->t2DDimData.H);
    //bb2.initRectangle((int)m2->t2DSpatialData.X, (int)m2->t2DSpatialData.Y, (int)m2->t2DDimData.W, (int)m2->t2DDimData.H);

    //if(Rectangle<int>::rectangleIntersectRatio(&bb2, &bb1) < m_mobile2DCoverageRateToConsiderEqual)
    //    return false;

    //if(Rectangle<int>::rectangleIntersectRatio(&bb1, &bb2) < m_mobile2DCoverageRateToConsiderEqual)
    //    return false;

    return true;
}


bool RMMTracker::sameUsed(SpRMMMobileObject m1, SpRMMMobileObject m2) {
    if(m1->numUsed != m2->numUsed)
        return false;

    if( (m1->currentVisualState & MM_PART_OF_BIGGER) || (m2->currentVisualState & MM_PART_OF_BIGGER) )
        return false;

    if(memcmp(m1->usedBlobs, m2->usedBlobs, sizeof(bool)*blobsNumber) == 0)
        return true;

    return false;
}

void RMMTracker::filterUnseenMobiles() {

    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator>::iterator hypothesis_it;
    std::deque<SpRMMMobileObject>::iterator mobile_it;
    unsigned int numToErase;
    bool alternativesToErase;
    if(!hypothesisSets.empty()) {

        for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end();) {
            alternativesToErase = false;
            numToErase = 0;
            for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++) {
                for(mobile_it = (*hypothesis_it)->begin(); mobile_it != (*hypothesis_it)->end(); ) {
                    SpRMMMobileObject m = (*mobile_it);

                    if((*mobile_it)->mobileOutOfScene()) {
                        std::map<long int, SpRMMMobileObject>::iterator previous;
                        if( (previous = mobilesOutList.find((*mobile_it)->getMobileId())) != mobilesOutList.end() ) {
                            if((*previous).second->getGlobalProbability() < (*mobile_it)->getGlobalProbability())
                                mobilesOutList[(*mobile_it)->getMobileId()] = *mobile_it;
                        } else {
                            mobilesOutList[(*mobile_it)->getMobileId()] = *mobile_it;
                            //		std::cout << "Mobile " << (*mobile_it)->getMobileId() << " stored in list of Mobiles out of scene." << std::endl;
                        }
                        mobile_it = (*hypothesis_it)->erase(mobile_it);
                    } else
                        mobile_it++;
                }

                (*hypothesis_it)->toEliminate = false;
                if((*hypothesis_it)->size() == 0) {
                    (*hypothesis_it)->toEliminate = alternativesToErase = true;
                    numToErase++;
                }
            }

            if(numToErase == (*hset_it)->size())
                hset_it = hypothesisSets.erase(hset_it);
            else if (numToErase > 0) {
                //Erase alternatives to erase
                std::set<SpRMMHypothesis, hypothesesOrderedByBestProbabilityOperator> newHypotheses;
                for(hypothesis_it = (*hset_it)->begin(); hypothesis_it != (*hset_it)->end(); hypothesis_it++)
                    if((*hypothesis_it)->toEliminate == false)
                        newHypotheses.insert(*hypothesis_it);
                (*hset_it)->hypotheses.clear();
                (*hset_it)->hypotheses = newHypotheses;
                hset_it++;
            } else
                hset_it++;

        }
    }
}


void RMMTracker::separateReliableHypotheses() {

    std::deque<SpRMMHypothesisSet>::iterator hset_it;
    std::deque<SpRMMHypothesisSet> newHSets;
    SpRMMHypothesis hypothesis;
    std::deque<SpRMMMobileObject>::iterator mobile_it;

    if(!hypothesisSets.empty()) {
        for(hset_it = hypothesisSets.begin(); hset_it != hypothesisSets.end(); hset_it++) {
            if((*hset_it)->size() == 1) { //Just one alternative
                hypothesis = (*(*hset_it)->begin());
                if(hypothesis->size() > 1) { //More than one mobile
                    for(mobile_it = hypothesis->begin() + 1; mobile_it != hypothesis->end(); mobile_it++) {
                        SpRMMHypothesis newHypothesis(new RMMHypothesis());
                        newHypothesis->insertNewMobileObject(*mobile_it);
                        newHypothesis->setHypothesisProbability();

                        SpRMMHypothesisSet newHypothesisSet(new RMMHypothesisSet());
                        newHypothesisSet->insert(newHypothesis);
                        newHypothesisSet->setBestHypothesis();

                        newHSets.push_front(newHypothesisSet);
                    }
                    hypothesis->erase(hypothesis->begin() + 1, hypothesis->end());
                }
            }
        }

        if(newHSets.size() > 0) {
            hypothesisSets.insert(hypothesisSets.end(), newHSets.begin(), newHSets.end());
            newHSets.clear();
        }
    }
}


void RMMTracker::presetBlobsVectorAndInitialMergeTable(std::vector<Blob>& blobs) {
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


    //ACAA!! Unificar
    m_RMerge->setInitialCorrespondences(initialMergeMap, blobsNumber, blobsVector);

}

void RMMTracker::freeBlobsVectorAndInitialMergeTable() {

    if(blobsVector)
        delete[] blobsVector;

    if(initialMergeMap) {
        for(int i = 0; i < blobsNumber; i++)
            delete[] initialMergeMap[i];

        delete[] initialMergeMap;
    }
}


void RMMTracker::setBlobsVector(std::vector<Blob>& blobs) {
    int j;
    int *elementsToAnalyzeVector;

    blobsNumber = blobs.size();

    //Static blobs number for tracking setting, for concerned classes
    RMMHypothesisSet::m_currentTrackingBlobsNumber = blobsNumber;
    RMMHypothesis::m_currentTrackingBlobsNumber = blobsNumber;
    RMMMobileObject::m_currentTrackingBlobsNumber = blobsNumber;

    blobsVector = NULL;
    usedBlobs = NULL;
    initialMergeMap = NULL;
    initialGroups =  NULL;
    elementsToAnalyzeVector =  NULL;
    involvedHypothesisSetsCounter = NULL;

    //ACAAAA!! Unificar
    m_RMerge->clearDefinedMerges();

    if(blobsNumber == 0)
        return;

    int i;
    blobsVector = new Blob *[blobsNumber];
    usedBlobs = new bool[blobsNumber];
    initialMergeMap = new bool*[blobsNumber];
    initialGroups = new int[blobsNumber];
    elementsToAnalyzeVector = new int[blobsNumber];
    involvedHypothesisSetsCounter = new int[blobsNumber];

    memset(usedBlobs, false, sizeof(bool)*blobsNumber);
    memset(involvedHypothesisSetsCounter, 0, sizeof(int)*blobsNumber);

    for(i = 0; i < blobsNumber; i++)
        elementsToAnalyzeVector[i] = i;

    std::vector<Blob>::iterator it, it_end = blobs.end();
    for(i = 0, it = blobs.begin(); it != it_end; it++, i++) {
        blobsVector[i] = &(*it);
        if(smodel->pmatrix_filled)
            BLOB_POSITION(blobsVector[i]) = blobsVector[i]->getPositionRelativeToCamera(smodel);
        BLOB_FRAME_NUMBER(blobsVector[i]) = currentFrameNumber;
        BLOB_TIME_DIFF_MSEC(blobsVector[i]) = lastMilliSecondsDifference;

        initialMergeMap[i] = new bool[blobsNumber];
        memset(initialMergeMap[i], false, sizeof(bool)*blobsNumber);
    }

    //ACAAA!!: Unificar
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

void RMMTracker::freeBlobsVector() {

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

    if(involvedHypothesisSetsCounter)
        delete[] involvedHypothesisSetsCounter;

    //ACAAA!!!: Unificar
    m_RMerge->clearDefinedMerges();
}

//Function defaults multi-model to single 2D blob model from BG segmentation with buffer of size 5
void RMMTracker::setDefaultMultiModelTemplate() {
    modelTemplate.multiModelDAG.clear();

    SpReliabilitySingleModelInterface defInstance(VideoAnalysis::modelConstructor["Blob2DFromBGSubstractionModel"]());
    defInstance->initFromXML(m_data, 5, "DefaultModel");
    defInstance->initInstanceAttributes();
    defInstance->initDynamicsAttributes();

    modelTemplate.multiModelDAG.push_back(defInstance);

    modelTemplate.forwardIndependentModels.insert("DefaultModel");

}

bool RMMTracker::processModelParameters(QString& filename) {
    QDomDocument xmlConfig( "MULTI_MODEL" );

    QFile file( filename );
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("Error opening multi model configuration file '" + filename + "' for reading. Execution will be aborted.");
        return false;
    }

    QString error;
    int line, column;
    if( !xmlConfig.setContent( &file, &error, &line, &column ) ) {

        AppendToLog(  "Error opening multi model configuration file '" + filename + "'. XML content could not be extracted. Execution will be aborted.\n"
                    + "Error in line " + QString::number(line) + " and column " + QString::number(column) + ": " + error);
        file.close();
        return false;
    }
    file.close();

    if(!modelTemplate.setParameters(xmlConfig))
        return false;

    xmlConfig.setContent( &file );
    file.close();

    return true;
}


bool RMMTracker::setParameters(QDomNode& config) {
    QDomNode n, m;

    if(config.isNull()) { //Parameter set for module not defined

        AppendToLog("ReliabilityMultiModelTrackingModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry ReliabilityMultiModelTrackingModule. Taking defaults...");

        //ACAAA!!!: Unificar
        m_RMerge->defaultParameters(ReliabilityMerge::MergeInTracking);
        m_MultiModelConfig = "";
        setDefaultMultiModelTemplate();
        setRMMMobileObjectTemplates();
        setBufferSize();

        activatePreMerge = false;

        RMMMobileObject::m_blobsBufferSize = m_BlobBufferSize;
        RMMMobileObject::secDiffSequence = new double[m_BlobBufferSize];
        RMMMobileObject::coolingValue    = new double[m_BlobBufferSize];
        RMMMobileObject::secDiffToCurrent = new double[m_BlobBufferSize];

        memset(RMMMobileObject::secDiffSequence,  0, sizeof(double)*m_BlobBufferSize);
        memset(RMMMobileObject::coolingValue,     0, sizeof(double)*m_BlobBufferSize);
        memset(RMMMobileObject::secDiffToCurrent, 0, sizeof(double)*m_BlobBufferSize);

        RMMMobileObject::coolingValue[0]     = 1.0;

        RMMMobileObject::m_trajectoryMaxSize = 0;
        m_maximumRetainedHypotheses = 10;
        m_maximumGeneratedHypothesesPerMobile = 4;
        maximumMobilePaths = 3;
        RMMMobileObject::m_2DLevelFrames = 3;
        eliminationStrategy = "LeastReliable";
        RMMMobileObject::m_lambda = 5.0;
        CoherenceReliabilityThreshold = RMMMobileObject::m_CoherenceReliabilityThreshold = 0.1;
        CoherenceProbabilityThreshold = RMMMobileObject::m_CoherenceProbabilityThreshold = 0.5;
        RMMMobileObject::m_knownSolutionThreshold = 0.5;
        hypothesesProbabilityThreshold = 0.8;
        IgnoreByDimensionalCoherenceThreshold = 0.1;
        IgnoreByVelocityCoherenceThreshold = 0.1;
        ImportanceRateForBestMobileHypothesisProbability = 0.9;
        RMMMobileObject::m_probabilityToEnsureMode = 0.9;

        //ACAA
        //m_maximal3DBaseOverlapping = 0.3;
        m_highVisualSupportThreshold = 0.95;
        m_lowVisualSupportThreshold = 0.05;
        m_blobCompletellySupportedThreshold = 0.95;
        m_internalOutputActivated = true;
        m_reducedOutputActivated = false;
        //ACAA!!!: Unificar
        //RMMMobileObject::m_maximalAlphaRotationSpeed = 2*1.755; //Twice the empirical maximal rotation speed of a person 1.755[rad/sec]
        m_meanMillisecondsDifferenceBetweenFrames = 80;
        //ACAA!!!: Unificar
        //RMMMobileObject::m_Maximal3DDimensionChangeSpeed = 244.0;
        acceptedPixelError = 2;
        accepted3DFeatureError = 10.0; //[cm]
        acceptedOrientationError = 0.1745329252; //10[deg]
        maxObjectSpeed = 200.0; //[cm/sec]

        m_InvolvedVisualEvidenceMultiplyingFactor = 1.0;
        m_InvolvedVisualEvidenceSumFactor = 100;
        m_InvolvedVisualEvidenceMinIntersectionRatio = 0.1;
        m_setObjectLog = false;
        m_objectLogRootName = "";
        m_objectLogModelName = "Blob2DFromBGSubstractionModelDynamics";
        m_logModelFound = false;

        m_activatePauseOnLost = m_activatePauseOnNew = false;

        return true;
    }

    //Check each parameter
    if( ! ( n = XmlCommon::getParameterNode("ActivatePauseCriteria", config) ).isNull() )  {

        if(XmlCommon::getParameterValue(n) == "yes") {
           if( ! ( m = XmlCommon::getParameterNode("ActivatePauseOnNew", n) ).isNull() )
                m_activatePauseOnNew = (XmlCommon::getParameterValue(m) == "yes") ? true : false;
           if( ! ( m = XmlCommon::getParameterNode("ActivatePauseOnLost", n) ).isNull() )
                m_activatePauseOnLost = (XmlCommon::getParameterValue(m) == "yes") ? true : false;
        }
    }



    if( ( n = XmlCommon::getParameterNode("MultiModelConfig", config) ).isNull() )  {
        m_MultiModelConfig = "";
        setDefaultMultiModelTemplate();

        AppendToLog("RMMTracker Warning: 'MultiModelConfig' parameter not found. Taking default multi model: single 2D blob model.\n");
    } else {
        m_MultiModelConfig = XmlCommon::getParameterValue(n);
        //sets activation criteria for each model: reliability on input (distance, bad data),
        //   needs (occlusion, priority),
        //sets priority of models (hierarchy).

        if(   !processModelParameters(m_MultiModelConfig) )
            setDefaultMultiModelTemplate();

    }


    if( ( n = XmlCommon::getParameterNode("Merge", config) ).isNull() )  {
        AppendToLog("RMMTracker Warning: 'Merge' not found. Taking default parameters for Merge in Tracking.\n");

        //ACAA!!!: Unificar
        m_RMerge->defaultParameters(ReliabilityMerge::MergeInTracking);
    } else {
        //ACAA!!!: Unificar
        if(!m_RMerge->setParameters(n, ReliabilityMerge::MergeInTracking)) {
            AppendToLog("RMMTracker: error: In 'Merge': Error in parameters for Merge in Tracking.\n");
            return false;
        }
    }

    if( ( n = XmlCommon::getParameterNode("PreMerge", config) ).isNull() )  {
        AppendToLog("RMMTracker Warning: 'PreMerge' not found. Taking default parameters for PreMerge in Tracking.\n");
        activatePreMerge = false;
    } else {
        activatePreMerge = XmlCommon::getParameterValue(n) == "true" ? true : false;
        if(activatePreMerge) {
            //ACAA!!!: Unificar
            if(!m_PreMerge->setParameters(n, ReliabilityMerge::PreMergeInTracking)) {
                AppendToLog("RMMTracker: error: In 'PreMerge': Error in parameters for PreMerge in Tracking.\n");
                return false;
            }
        }
    }

    if( ( n = XmlCommon::getParameterNode("BlobBufferSize", config) ).isNull() ) {
        m_BlobBufferSize = 4;
        AppendToLog("RMMTracker Warning: 'BlobBufferSize' not defined. Taking Default (4).\n");
    } else {
        m_BlobBufferSize = XmlCommon::getParameterValue(n).toInt();
        if(m_BlobBufferSize <= 0) {
            AppendToLog("RMMTracker Error: 'BlobBufferSize' value must be a positive integer > 1. Taking Default (5).\n");
            m_BlobBufferSize = 4;
        }
    }

    setRMMMobileObjectTemplates();
    setBufferSize();

    RMMMobileObject::m_blobsBufferSize = m_BlobBufferSize;
    RMMMobileObject::secDiffSequence = new double[m_BlobBufferSize];
    RMMMobileObject::coolingValue    = new double[m_BlobBufferSize];
    RMMMobileObject::secDiffToCurrent = new double[m_BlobBufferSize];

    memset(RMMMobileObject::secDiffSequence,  0, sizeof(double)*m_BlobBufferSize);
    memset(RMMMobileObject::coolingValue,     0, sizeof(double)*m_BlobBufferSize);
    memset(RMMMobileObject::secDiffToCurrent, 0, sizeof(double)*m_BlobBufferSize);

    RMMMobileObject::coolingValue[0]     = 1.0;

    if( ( n = XmlCommon::getParameterNode("TrajectoryMaxSize", config) ).isNull() ) {
        RMMMobileObject::m_trajectoryMaxSize = 0;
        AppendToLog("RMMTracker Warning: 'TrajectoryMaxSize' not defined. Taking Default 0 (No Max Limit).\n");
    } else {
        RMMMobileObject::m_trajectoryMaxSize = XmlCommon::getParameterValue(n).toInt();

        if(RMMMobileObject::m_trajectoryMaxSize < 0) {
            AppendToLog("RMMTracker Error: 'TrajectoryMaxSize' value must be an integer >= 0. Taking Default 0 (No Max Limit).\n");
            RMMMobileObject::m_trajectoryMaxSize = 0;
        }
    }

    if( ( n = XmlCommon::getParameterNode("MaximumRetainedHypotheses", config) ).isNull() ) {
        m_maximumRetainedHypotheses = 10;
        AppendToLog("RMMTracker Warning: 'MaximumRetainedHypotheses' not defined. Taking Default (10).\n");
    } else {
        m_maximumRetainedHypotheses = XmlCommon::getParameterValue(n).toInt();
    }

    if( ( n = XmlCommon::getParameterNode("MaximalNumberOfPathHypothesesPerMobile", config) ).isNull() ) {
        m_maximumGeneratedHypothesesPerMobile = 4;
        AppendToLog("RMMTracker Warning: 'MaximalNumberOfPathHypothesesPerMobile' not defined. Taking Default (4).\n");
    } else {
        if( (m_maximumGeneratedHypothesesPerMobile = XmlCommon::getParameterValue(n).toInt()) < 1 ) {
            m_maximumGeneratedHypothesesPerMobile = 4;
            AppendToLog("RMMTracker Warning: 'MaximalNumberOfPathHypothesesPerMobile' must be an integer higher than 0. Taking Default (4).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("maximumMobilePaths", config) ).isNull() ) {
        maximumMobilePaths = 3;
        AppendToLog("RMMTracker Warning: 'maximumMobilePaths' not defined. Taking Default (3).\n");
    } else {
        if( (maximumMobilePaths = XmlCommon::getParameterValue(n).toInt()) < 1) {
            maximumMobilePaths = 3;
            AppendToLog("RMMTracker Warning: 'maximumMobilePaths' must be an integer higher than 0. Taking Default (3).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("_2DLevelFrames", config) ).isNull() ) {
        RMMMobileObject::m_2DLevelFrames = 3;
        AppendToLog("RMMTracker Warning: '_2DLevelFrames' not defined. Taking Default (3).\n");
    } else {
        if( (RMMMobileObject::m_2DLevelFrames = XmlCommon::getParameterValue(n).toInt()) < 1 ) {
            RMMMobileObject::m_2DLevelFrames = 3;
            AppendToLog("RMMTracker Warning: '_2DLevelFrames' must be an integer higher than 0. Taking Default (3).\n");
        }
    }

    if( ( n = XmlCommon::getParameterNode("EliminationStrategy", config) ).isNull() ) {
        eliminationStrategy = "LeastReliable";
        AppendToLog("RMMTracker Warning: 'EliminationStrategy' not defined. Taking Default (LeastReliable).\n");
    } else {
        eliminationStrategy = XmlCommon::getParameterValue(n);
    }

    if( ( n = XmlCommon::getParameterNode("CoolingFunctionLambda", config) ).isNull() ) {
        RMMMobileObject::m_lambda = 5.0;
        AppendToLog("RMMTracker Warning: 'CoolingFunctionLambda' not defined. Taking Default (5.0).\n");
    } else
        RMMMobileObject::m_lambda = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("CoherenceReliabilityThreshold", config) ).isNull() ) {
        CoherenceReliabilityThreshold = RMMMobileObject::m_CoherenceReliabilityThreshold = 0.1;
        AppendToLog("RMMTracker Warning: 'CoherenceReliabilityThreshold' not defined. Taking Default (0.1).\n");
    } else
        RMMMobileObject::m_CoherenceReliabilityThreshold = CoherenceReliabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("CoherenceProbabilityThreshold", config) ).isNull() ) {
        CoherenceProbabilityThreshold = RMMMobileObject::m_CoherenceProbabilityThreshold = 0.5;
        AppendToLog("RMMTracker Warning: 'CoherenceProbabilityThreshold' not defined. Taking Default (0.5).\n");
    } else
        CoherenceProbabilityThreshold = RMMMobileObject::m_CoherenceProbabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("KnownSolutionThreshold", config) ).isNull() ) {
        RMMMobileObject::m_knownSolutionThreshold = 0.5;
        AppendToLog("RMMTracker Warning: 'KnownSolutionThreshold' not defined. Taking Default (0.5).\n");
    } else
        RMMMobileObject::m_knownSolutionThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("HypothesesProbabilityThreshold", config) ).isNull() ) {
        hypothesesProbabilityThreshold = 0.8;
        AppendToLog("RMMTracker Warning: 'HypothesesProbabilityThreshold' not defined. Taking Default (0.8).\n");
    } else
        hypothesesProbabilityThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("IgnoreByDimensionalCoherenceThreshold", config) ).isNull() ) {
        IgnoreByDimensionalCoherenceThreshold = 0.1;
        AppendToLog("RMMTracker Warning: 'IgnoreByDimensionalCoherenceThreshold' not defined. Taking Default (0.1).\n");
    } else
        IgnoreByDimensionalCoherenceThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("IgnoreByVelocityCoherenceThreshold", config) ).isNull() ) {
        IgnoreByVelocityCoherenceThreshold = 0.1;
        AppendToLog("RMMTracker Warning: 'IgnoreByVelocityCoherenceThreshold' not defined. Taking Default (0.1).\n");
    } else
        IgnoreByVelocityCoherenceThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("ImportanceRateForBestMobileHypothesisProbability", config) ).isNull() ) {
        ImportanceRateForBestMobileHypothesisProbability = 0.9;
        AppendToLog("RMMTracker Warning: 'ImportanceRateForBestMobileHypothesisProbability' not defined. Taking Default (0.9).\n");
    } else
        ImportanceRateForBestMobileHypothesisProbability = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("ProbabilityToEnsureMode", config) ).isNull() ) {
        RMMMobileObject::m_probabilityToEnsureMode = 0.9;
        AppendToLog("RMMTracker Warning: 'ProbabilityToEnsureMode' not defined. Taking Default (0.9).\n");
    } else
        RMMMobileObject::m_probabilityToEnsureMode = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Maximal3DBaseOverlapping", config) ).isNull() ) {
        //ACAA!!!: Unificar
        //m_maximal3DBaseOverlapping = 0.3;
        AppendToLog("RMMTracker Warning: 'Maximal3DBaseOverlapping' not defined. Taking Default (0.3).\n");
    } //else
        //ACAA!!!: Unificar
        //m_maximal3DBaseOverlapping = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Mobile2DCoverageRateToConsiderEqual", config) ).isNull() ) {
        //ACAA!!!: Unificar
        RMMMobileObject::m_mobile2DCoverageRateToConsiderEqual = 0.95;
        AppendToLog("RMMTracker Warning: 'Mobile2DCoverageRateToConsiderEqual' not defined. Taking Default (0.95).\n");
    } else
        //ACAA!!!: Unificar
        RMMMobileObject::m_mobile2DCoverageRateToConsiderEqual = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("HighVisualSupportThreshold", config) ).isNull() ) {
        m_highVisualSupportThreshold = 0.95;
        AppendToLog("RMMTracker Warning: 'HighVisualSupportThreshold' not defined. Taking Default (0.85).\n");
    } else
        m_highVisualSupportThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("LowVisualSupportThreshold", config) ).isNull() ) {
        m_lowVisualSupportThreshold = 0.05;
        AppendToLog("RMMTracker Warning: 'LowVisualSupportThreshold' not defined. Taking Default (0.15).\n");
    } else
        m_lowVisualSupportThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("BlobCompletellySupportedThreshold", config) ).isNull() ) {
        m_blobCompletellySupportedThreshold = 0.95;
        AppendToLog("RMMTracker Warning: 'BlobCompletellySupportedThreshold' not defined. Taking Default (0.95).\n");
    } else
        m_blobCompletellySupportedThreshold = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("InternalOutputActivated", config) ).isNull() ) {
        m_internalOutputActivated = true;
        AppendToLog("RMMTracker Warning: 'InternalOutputActivated' not defined. Taking Default (true).\n");
    } else
        m_internalOutputActivated = XmlCommon::getParameterValue(n) == "true" ? true : false;

    if( ( n = XmlCommon::getParameterNode("ReducedOutputActivated", config) ).isNull() ) {
        m_reducedOutputActivated = false;
        AppendToLog("RMMTracker Warning: 'ReducedOutputActivated' not defined. Taking Default (false).\n");
    } else
        m_reducedOutputActivated = XmlCommon::getParameterValue(n) == "true" ? true : false;

    //ACAAA!!!: Unificar
    if( ( n = XmlCommon::getParameterNode("MaximalAlphaRotationSpeed", config) ).isNull() ) {
        //RMMMobileObject::m_maximalAlphaRotationSpeed = 2*1.755; //Twice the empirical maximal rotation speed of a person 1.755[rad/sec]
        AppendToLog("RMMTracker Warning: 'MaximalAlphaRotationSpeed' not defined. Taking Default 2*1.755[rad]/[sec]. Where 1.755[rad]/[sec] corresponds to the empirical calculation of the rotation speed of a person\n");
    } //else
    //    RMMMobileObject::m_maximalAlphaRotationSpeed = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("MeanMillisecondsDifferenceBetweenFrames", config) ).isNull() ) {
        m_meanMillisecondsDifferenceBetweenFrames = 80;
        AppendToLog("RMMTracker Warning: 'MeanMillisecondsDifferenceBetweenFrames' not defined. Taking Default (80).\n");
    } else {
        if( (m_meanMillisecondsDifferenceBetweenFrames = XmlCommon::getParameterValue(n).toInt()) <= 0) {
            m_meanMillisecondsDifferenceBetweenFrames = 80;
            AppendToLog("RMMTracker Warning: 'MeanMillisecondsDifferenceBetweenFrames' must be an integer higher than 0. Taking Default (80).\n");
        }
    }

    /*if( ( n = XmlCommon::getParameterNode("Maximal3DDimensionChangeSpeed", config) ).isNull() ) {
        RMMMobileObject::m_Maximal3DDimensionChangeSpeed = 244.0;
        AppendToLog("RMMTracker Warning: 'Maximal3DDimensionChangeSpeed' not defined. Taking Default (244.0 [cm/s]).\n");
    } else
        RMMMobileObject::m_Maximal3DDimensionChangeSpeed = XmlCommon::getParameterValue(n).toDouble();
*/

    if( ( n = XmlCommon::getParameterNode("AcceptedPixelError", config) ).isNull() ) {
        acceptedPixelError = 2;
        AppendToLog("RMMTracker Warning: 'AcceptedPixelError' not defined. Taking Default (2).\n");
    } else
        acceptedPixelError = XmlCommon::getParameterValue(n).toInt();

    if( ( n = XmlCommon::getParameterNode("Accepted3DFeatureError", config) ).isNull() ) {
        accepted3DFeatureError = 10.0; //[cm]
        AppendToLog("RMMTracker Warning: 'Accepted3DFeatureError' not defined. Taking Default (10[cm]).\n");
    } else
        accepted3DFeatureError = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("AcceptedOrientationError", config) ).isNull() ) {
        acceptedOrientationError = 0.1745329252; //10[deg]
        AppendToLog("RMMTracker Warning: 'AcceptedOrientationError' not defined. Taking Default (0.1745329252[rad] (10[deg]) ).\n");
    } else
        acceptedOrientationError = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("MaxObjectSpeed", config) ).isNull() ) {
         maxObjectSpeed = 200.0; //[cm/sec]
        AppendToLog("RMMTracker Warning: 'MaxObjectSpeed' not defined. Taking Default (200[cm/sec]).\n");
    } else
        maxObjectSpeed = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("InvolvedVisualEvidenceMultiplyingFactor", config) ).isNull() ) {
        m_InvolvedVisualEvidenceMultiplyingFactor = 2.0;
        AppendToLog("RMMTracker Warning: 'InvolvedVisualEvidenceMultiplyingFactor' not defined. Taking Default (2.0).\n");
    } else
        m_InvolvedVisualEvidenceMultiplyingFactor = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("InvolvedVisualEvidenceSumFactor", config) ).isNull() ) {
        m_InvolvedVisualEvidenceSumFactor = 100;
        AppendToLog("RMMTracker Warning: 'InvolvedVisualEvidenceSumFactor' not defined. Taking Default (1.0).\n");
    } else
        m_InvolvedVisualEvidenceSumFactor = XmlCommon::getParameterValue(n).toInt();

    if( ( n = XmlCommon::getParameterNode("InvolvedVisualEvidenceMinIntersectionRatio", config) ).isNull() ) {
        m_InvolvedVisualEvidenceMinIntersectionRatio = 0.1;
        AppendToLog("RMMTracker Warning: 'InvolvedVisualEvidenceMinIntersectionRatio' not defined. Taking Default (0.1).\n");
    } else
        m_InvolvedVisualEvidenceMinIntersectionRatio = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("ObjectLog", config) ).isNull() )  {
        AppendToLog("RMMTracker Warning: 'ObjectLog' not found. Taking default parameter value: false.\n");
        m_setObjectLog = false;
    } else {
        m_setObjectLog = XmlCommon::getParameterValue(n) == "true" ? true : false;
        if(m_setObjectLog) {
            if( ( m = XmlCommon::getParameterNode("LogRootName", n) ).isNull() )  {
                AppendToLog("RMMTracker Warning: 'ObjectLog::LogRootName' not found. Taking default parameter value: 'VAT-Object'.\n");
                m_objectLogRootName = "VAT-Object";
            } else {
                m_objectLogRootName = XmlCommon::getParameterValue(m);
                if( m_objectLogRootName == "" )  {
                    AppendToLog("RMMTracker Warning: 'ObjectLog::LogRootName' not found. Taking default parameter value: 'VAT-Object'.\n");
                    m_objectLogRootName = "VAT-Object";
                }
            }

            if( ( m = XmlCommon::getParameterNode("LogModelName", n) ).isNull() )  {
                AppendToLog("RMMTracker Warning: 'ObjectLog::LogModelName' not found. Taking default parameter value: 'Blob2DFromBGSubstractionModel'.\n");
                m_objectLogModelName = "Blob2DFromBGSubstractionModel";
            } else {
                m_objectLogModelName = XmlCommon::getParameterValue(m);
                if( m_objectLogModelName == "" )  {
                    AppendToLog("RMMTracker Warning: 'ObjectLog::LogModelName' not found. Taking default parameter value: 'Blob2DFromBGSubstractionModel'.\n");
                    m_objectLogModelName = "Blob2DFromBGSubstractionModel";
                }
            }
        }

    }

    return true;
}

void RMMTracker::setRMMMobileObjectTemplates() {
    RMMMobileObject::modelTemplate = modelTemplate;
}

void RMMTracker::setBufferSize() {
    int max = -1;

    setBufferSize(max, modelTemplate.multiModelDAG);

    if(max > m_BlobBufferSize)
        RMMMobileObject::m_blobsBufferSize = m_BlobBufferSize = max;
}

void RMMTracker::setBufferSize(int &max, std::deque<SpReliabilitySingleModelInterface> &models) {

    if(models.empty())
        return;
    int mmax;
    std::deque<SpReliabilitySingleModelInterface>::iterator it, it_end = models.end();

    for(it=models.begin(); it != it_end; it++) {
        mmax = (*it)->instances.maxLength;
        if(mmax > max)
            max = mmax;
    }

}


void RMMTracker::setGroup(int groupsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector) {
    int i;

    groupsVector[referencePoint] = groupId;

    for(i=startPoint; i<elementsNumber; i++)
        if(relations[elementsToAnalyzeVector[referencePoint]][elementsToAnalyzeVector[i]] && groupsVector[i] < 0)
            setGroup(groupsVector, relations, elementsNumber, groupId, startPoint, i, elementsToAnalyzeVector);

}

int RMMTracker::setGroups(int groupsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector) {
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


bestHypothesesNode::bestHypothesesNode(double i_value, int i_index): value(i_value), mobileIndex(i_index), versionIndex(NULL), added(false), mobileIterators(NULL) {
    mobileIterators = new std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memset(versionIndex, 0, sizeof(int) * numVariables);
}

bestHypothesesNode::bestHypothesesNode(SpBestHypothesesNode toCopy, int indexToMove): value(toCopy->value), mobileIndex(indexToMove), versionIndex(NULL), added(false), mobileIterators(NULL) {

    mobileIterators = new std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator[numVariables];
    variableContribution = new double[numVariables];
    versionIndex = new int[numVariables];

    memcpy(mobileIterators, toCopy->mobileIterators, sizeof(std::set<SpRMMMobileObject, RMMorderedByBestCoherenceOperator>::iterator) * numVariables );
    memcpy(variableContribution, toCopy->variableContribution, sizeof(double) * numVariables );
    memcpy(versionIndex, toCopy->versionIndex, sizeof(int) * numVariables );

    mobileIndex = indexToMove;
    setNextHypothesisValue(indexToMove);

}

bestHypothesesNode::~bestHypothesesNode() {
    delete[] mobileIterators;
    delete[] variableContribution;
    delete[] versionIndex;
}


void bestHypothesesNode::setNextHypothesisValue(int index) {
    if(mobileIterators[index] == newObjectsListEnds[index]) {
        value = -1;
        return;
    }
    mobileIterators[index]++;
    if(mobileIterators[index] == newObjectsListEnds[index]) {
        value = -1;
        return;
    }

    versionIndex[index]++;

    value -= variableContribution[index];
    value += variableContribution[index] = variablesNumFrames[index] * (*mobileIterators[index])->getGlobalProbability() / (double) variablesSum;

}

bool orderedByBestHypothesisProbabilityCooperationOperator::operator()(SpBestHypothesesNode altNode1, SpBestHypothesesNode altNode2) {
    return altNode1->value >= altNode2->value;
}



