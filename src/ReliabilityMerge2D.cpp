#include "ReliabilityMerge2D.h"
#include "Classification.h"
#include "GaussianFunction.h"
#include "blob.h"
#include "Datapool.h"
#include "VideoAnalysis.h"
#include <climits>


ReliabilityMerge2D::AlternativeElement::AlternativeElement():blob_id(-1), mergedBlob(NULL), classified(false) { }

ReliabilityMerge2D::AlternativeElement::AlternativeElement(int i_blob_id) : blob_id(i_blob_id),
                                                    mergedBlob(NULL),
                                                    classified(false) { }
ReliabilityMerge2D::AlternativeElement::~AlternativeElement(){
    mergedElements.clear();
    if(mergedBlob)
        delete mergedBlob;
}

ReliabilityMerge2D::mergeType::mergeType() {}

ReliabilityMerge2D::mergeType::~mergeType() {}

ReliabilityMerge2D::ReliabilityMerge2D() {
    initialized = false;
}

ReliabilityMerge2D::ReliabilityMerge2D(Datapool *i_data) : m_data(i_data) {
    initialized = false;
}

bool ReliabilityMerge2D::prepareRun() {
    if( (m_segmentation = m_data->fgImage) == NULL) {
        AppendToLog("ReliabilityMerge2D: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }
    return true;
}

bool ReliabilityMerge2D::init(){
    initialized = true;
    return true;
}

bool ReliabilityMerge2D::canBeMerged(Blob *i_blob1, Blob *i_blob2) {
    double score1, score2;
    double mergeScore = 0.0, mergeScoreGauss = 0.0;

   //CAMBIAR A OBJETOS MUY PEQUENOS
    if(BLOB_IS_REDUCED_SIZE(i_blob1) && BLOB_IS_REDUCED_SIZE(i_blob2)) {

        //Compute the type and the score of the merge blob
 //       std::map<ObjectType, double>& mergeResultList = m_mapCorrespondences[i_blob1][i_blob2].mergeTypeMap;

/*        std::map<ObjectType, double>::iterator listIt;
        for(listIt = mergeResultList.begin(); listIt!=mergeResultList.end(); listIt++){
            if((*listIt).second > mergeScore){
                mergeScore = (*listIt).second;
                mergeType = (*listIt).first;
            }
        }
    
        if(mergeScore < m_mergeThres)
            mergeType = UNKNOWN;
    
        o_mergeType->type = mergeType;
        o_mergeType->score = mergeScore;
*/
        return true;
    }

    //Find the maximum score
  //  std::map<ObjectType, double>& mergeResultList = m_mapCorrespondences[i_blob1][i_blob2].mergeTypeMap;

 /*   std::map<ObjectType, double>::iterator listIt;
    for(listIt = mergeResultList.begin(); listIt!=mergeResultList.end(); listIt++) {
        if((*listIt).second > mergeScore){
            mergeScore = (*listIt).second;
            mergeType = (*listIt).first;
        }
    }

    if(mergeScore < m_mergeThres) {
        mergeType = UNKNOWN;
        return false;
    }

    //if blobs are overlaping then no need to test if the score is better, only get the best score
    if(m_mapCorrespondences[i_blob1][i_blob2].overlaping) {
        o_mergeType->type = mergeType;
        o_mergeType->score = mergeScore;
        return true;
    }
 
    //  SpFunctionInterface spScore = new CGaussianFunction(std::max(score1,score2), std::max(score1,score2) *  5.0 / 100.0);
//    SpGaussianFunction spScore = QSharedPointer<GaussianFunction>(new GaussianFunction(std::max(score1,score2), std::max(score1,score2) *  0.05));
  
    double
        area1 =  BLOB_WIDTH(i_blob1) * BLOB_HEIGHT(i_blob1),
        area2 =  BLOB_WIDTH(i_blob1) * BLOB_HEIGHT(i_blob1);
    //if the merge blob has a best score or if it is 2 UNKNOWN blobs the merge is OK
    if((mergeScore > std::max(score1,score2) || mergeScoreGauss > 0.9) ||
       (BLOB_TYPE(i_blob1) == UNKNOWN && BLOB_TYPE(i_blob2) == UNKNOWN))   {
        //Assign best score to the merge blob
        (*o_mergeType).type = mergeType;
        (*o_mergeType).score = mergeScore;
        return true;
    } else if( (BLOB_TYPE(i_blob1) == UNKNOWN || BLOB_TYPE(i_blob2) == UNKNOWN) && mergeScore > (score1*area1 + score2*area2) / (area1 + area2) )  {
        (*o_mergeType).type = mergeType;
        (*o_mergeType).score = mergeScore;
        return true;
    } else {
        (*o_mergeType).type = UNKNOWN;
        (*o_mergeType).score = mergeScore;
        return false;
    }*/
    return true;
}


/*!
    \fn ReliabilityMerge2D::getNextCorrespondence(Blob **o_pBlob1, Blob **o_pBlob2)
 */
bool ReliabilityMerge2D::getNextCorrespondence(Blob **o_pBlob1, Blob **o_pBlob2)
{
    // declaration
    double maxScore = -1.0;
    std::map<Blob *, std::map<Blob *, mergeData> >::iterator lineIter;
    std::map<Blob *, mergeData>::iterator colIter;

    (*o_pBlob1) = NULL;
    (*o_pBlob2) = NULL;

    for(lineIter = m_mapCorrespondences.begin(); lineIter != m_mapCorrespondences.end(); lineIter++) {
        for(colIter = (*lineIter).second.begin(); colIter != (*lineIter).second.end(); colIter++) {
            if((*colIter).second.overlaping){
                //No need to find a better correspondence. We want to process theses cases first
                (*o_pBlob1) = (*lineIter).first;
                (*o_pBlob2) = (*colIter).first;
                return true;
            }
     
            // find maximal association degree. If we have two maximal
            // association degrees. We choose this one correspond to two
            // blobs having bigger vertical degree.
            //Parse the list
            /*std::map<ObjectType, double>& mergeResultList = (*colIter).second.mergeTypeMap;

            std::map<ObjectType, double>::iterator listIt;
            for(listIt = mergeResultList.begin(); listIt!=mergeResultList.end(); listIt++) {
                if( (*listIt).second >= maxScore ){
                    maxScore = (*listIt).second;
                    *o_pBlob1 = (*lineIter).first;
                    *o_pBlob2 = (*colIter).first;
                }
            }*/
        }
    }

    return maxScore > -0.5;
}


/*!
    \fn ReliabilityMerge2D::isAValidCorrespondence(Blob *i_blob1, Blob *i_blob2)
 */
bool ReliabilityMerge2D::isAValidCorrespondence(Blob *i_blob1, Blob *i_blob2) {
  return BLOB_TYPE(i_blob1)!=CONTX_OBJECT && BLOB_TYPE(i_blob2)!=CONTX_OBJECT &&
    distanceCriteria(i_blob1, i_blob2);
}


/*!
    \fn ReliabilityMerge2D::distanceCriteria(Blob *i_blob1, Blob *i_blob2))
 */
bool ReliabilityMerge2D::distanceCriteria(Blob *i_blob1, Blob *i_blob2) {
  double dist;
  
  if (widthIntersectionRatio(i_blob1, i_blob2) >= 0.5) {
      dist = (1.5 * m_mergeDistMax);
      
      // One blob is on the top and not on aoi ground, distance is increased
      if(blobOnTopAndNotOnAOIGround(i_blob1, i_blob2))
	dist = (3.5 * m_mergeDistMax);
  } else
    dist = m_mergeDistMax;
  
  
  // Process distance criterion
  
  return Rectangle<int>::rectangleDistance(BLOB_BBOX(i_blob1), BLOB_BBOX(i_blob2)) < dist;
}


/*!
    \fn ReliabilityMerge2D::widthIntersectionRatio(Blob *i_blob1, Blob *i_blob2)
 */
double ReliabilityMerge2D::widthIntersectionRatio(Blob *i_blob1, Blob *i_blob2)
{
  int xleft, xright;
  double common_x;
  
  if((i_blob1 == NULL) || (i_blob2 == NULL))
    return 0.0;
  
  xleft  = std::max(BLOB_XLEFT(i_blob1), BLOB_XLEFT(i_blob2));
  xright = std::min(BLOB_XRIGHT(i_blob1), BLOB_XRIGHT(i_blob2));
  
  // x segment in commom
  if((common_x = xright - xleft + 1) <= 0.0)
		return 0.0;
  
  // return vertical degree
  return (common_x/(double)std::min(BLOB_WIDTH(i_blob1), BLOB_WIDTH(i_blob2)));
}


/*!
  \fn ReliabilityMerge2D::heightIntersectionRatio(Blob *i_blob1, Blob *i_blob2)
*/
double ReliabilityMerge2D::heightIntersectionRatio(Blob *i_blob1, Blob *i_blob2)
{
  int ytop, ybottom;
  double common_y;
  
  if((i_blob1 == NULL) || (i_blob2 == NULL))
    return 0.0;
  ytop    = std::max(BLOB_YTOP(i_blob1), BLOB_YTOP(i_blob2));
  ybottom = std::min(BLOB_YBOTTOM(i_blob1), BLOB_YBOTTOM(i_blob2));
  
  // y segment in commom
  if((common_y = ybottom - ytop + 1) <= 0.0)
    return 0.0;
  
  	// return the horizontal degree
  return (common_y/(double)std::min(BLOB_HEIGHT(i_blob1), BLOB_HEIGHT(i_blob2)));
}


/*!
  \fn ReliabilityMerge2D::BlobOnTopAndNotOnAOIGround(Blob *i_blob1, Blob *i_blob2)
*/
bool ReliabilityMerge2D::blobOnTopAndNotOnAOIGround(Blob *i_blob1, Blob *i_blob2)
{
  // Declaration
  Blob *blob = NULL;

  // Verify
  if(i_blob1 == NULL && i_blob2 == NULL)
    return false;
  
  // Which is on the top ? no overlap possibility
  if ( BLOB_YBOTTOM(i_blob1) < BLOB_YTOP(i_blob2) )
    blob = i_blob1;
  if ( BLOB_YBOTTOM(i_blob2) < BLOB_YTOP(i_blob1) )
    blob = i_blob2;

  // Verify
  if(blob == NULL)
    return true;
  
  // If blob is on aoi ground ?
  if(BLOB_DP_TYPE(blob) == MM_AOI_OUTSIDE)
    return true;
  else
    return false;
}

bool ReliabilityMerge2D::defaultParameters(mergeParameterSet i_mergeParameterSet) {

  switch(i_mergeParameterSet) {
  case MergeInTracking:
    m_mergeThres = 0.4;
    m_overlapingRatio = 0.8;
    m_mergeDistMax = 10;
    m_eliminateOverlapedBlobs = false;
    m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
    m_eliminateOverlapedBlobsProbability = 0.8;
    m_eliminateOverlapedBlobsReliability = 0.5;
    //    m_reducedUnknownBlobsMerge = false;
    break;
  case PreMergeInTracking:
    m_mergeThres = 0.4;
    m_overlapingRatio = 0.8;
    m_mergeDistMax = 10;
    m_eliminateOverlapedBlobs = true;
    m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
    m_eliminateOverlapedBlobsProbability = 0.8;
    m_eliminateOverlapedBlobsReliability = 0.5;

  };

  return true;
}

bool ReliabilityMerge2D::setParameters(QDomNode& config, mergeParameterSet i_mergeParameterSet){
    QDomNode n;

    switch(i_mergeParameterSet) {
    case MergeInTracking:
        m_eliminateOverlapedBlobs = true;
        m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
        m_eliminateOverlapedBlobsProbability = 0.80;
        m_eliminateOverlapedBlobsReliability = 0.50;
        break;
    case PreMergeInTracking:
        if(config.isNull()) { //Parameter set for module not defined
            m_eliminateOverlapedBlobs = true;
            m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
            m_eliminateOverlapedBlobsProbability = 0.80;
            m_eliminateOverlapedBlobsReliability = 0.50;
        } else {
            //Check each parameter
            if( ( n = XmlCommon::getParameterNode("EliminateOverlapedBlobs", config) ).isNull() )  {
                AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs' not found. Taking default.\n");
                m_eliminateOverlapedBlobs = true;
                m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
                m_eliminateOverlapedBlobsProbability = 0.80;
                m_eliminateOverlapedBlobsReliability = 0.50;
            } else {
                if(XmlCommon::getParameterValue(n) == "true") {
                    m_eliminateOverlapedBlobs = true;

                    QDomNode m;

                    if( ( m = XmlCommon::getParameterNode("OverlapingRatio", n) ).isNull() )  {
                        AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::OverlapingRatio' not found. Taking default.\n");
                        m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
                    } else {
                        m_eliminateOverlapedBlobsOverlapingRatio = XmlCommon::getParameterValue(m).toDouble();
                        if(    m_eliminateOverlapedBlobsOverlapingRatio < 0.0
                            || m_eliminateOverlapedBlobsOverlapingRatio > 1.0 ) {
                            AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::OverlapingRatio' out of bounds. Taking default.\n");
                            m_eliminateOverlapedBlobsOverlapingRatio = 0.95;
                        }
                    }

                    if( ( m = XmlCommon::getParameterNode("Probability", n) ).isNull() )  {
                        AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::Probability' not found. Taking default.\n");
                        m_eliminateOverlapedBlobsProbability = 0.8;
                    } else {
                        m_eliminateOverlapedBlobsProbability = XmlCommon::getParameterValue(m).toDouble();
                        if(    m_eliminateOverlapedBlobsProbability < 0.0
                            || m_eliminateOverlapedBlobsProbability > 1.0 ) {
                            AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::Probability' out of bounds. Taking default.\n");
                            m_eliminateOverlapedBlobsProbability = 0.8;
                        }
                    }

                    if( ( m = XmlCommon::getParameterNode("Reliability", n) ).isNull() )  {
                        AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::Reliability' not found. Taking default.\n");
                        m_eliminateOverlapedBlobsReliability = 0.5;
                    } else {
                        m_eliminateOverlapedBlobsReliability = XmlCommon::getParameterValue(m).toDouble();
                        if(    m_eliminateOverlapedBlobsReliability < 0.0
                            || m_eliminateOverlapedBlobsReliability > 1.0 ) {
                            AppendToLog("ReliabilityMerge2D Warning: 'EliminateOverlapedBlobs::Reliability' out of bounds. Taking default.\n");
                            m_eliminateOverlapedBlobsReliability = 0.5;
                        }
                    }
                } else {
                    m_eliminateOverlapedBlobs = false;
                }
            }
        }
        break;
    };

    //General Parameters
    if( ( n = XmlCommon::getParameterNode("MaxDistance", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'MaxDistance' not found. Taking default.\n");
        m_mergeDistMax = 10;
    } else
        m_mergeDistMax = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Threshold", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'Threshold' not found. Taking default.\n");
        m_mergeThres = 0.4;
    } else {
        m_mergeThres = XmlCommon::getParameterValue(n).toDouble();
        if( m_mergeThres < 0.0 || m_mergeThres > 1.0 ) {
            AppendToLog("ReliabilityMerge2D Warning: 'Threshold' out of bounds. Taking default.\n");
            m_mergeThres = 0.4;
        }
    }

    if( ( n = XmlCommon::getParameterNode("OverlapingRatio", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'OverlapingRatio' not found. Taking default.\n");
        m_overlapingRatio = 0.95;
    } else {
        m_overlapingRatio = XmlCommon::getParameterValue(n).toDouble();
        if( m_overlapingRatio < 0.0 || m_overlapingRatio > 1.0 ) {
            AppendToLog("ReliabilityMerge2D Warning: 'OverlapingRatio' out of bounds. Taking default.\n");
            m_overlapingRatio = 0.95;
        }
    }

    return true;
}

bool ReliabilityMerge2D::setParameters(QDomNode& config){
    QDomNode n;

    //General Parameters
    if( ( n = XmlCommon::getParameterNode("MaxDistance", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'MaxDistance' not found. Taking default.\n");
        m_mergeDistMax = 10;
    } else
        m_mergeDistMax = XmlCommon::getParameterValue(n).toDouble();

    if( ( n = XmlCommon::getParameterNode("Threshold", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'Threshold' not found. Taking default.\n");
        m_mergeThres = 0.4;
    } else {
        m_mergeThres = XmlCommon::getParameterValue(n).toDouble();
        if( m_mergeThres < 0.0 || m_mergeThres > 1.0 ) {
            AppendToLog("ReliabilityMerge2D Warning: 'Threshold' out of bounds. Taking default.\n");
            m_mergeThres = 0.4;
        }
    }

    if( ( n = XmlCommon::getParameterNode("OverlapingRatio", config) ).isNull() )  {
        AppendToLog("ReliabilityMerge2D Warning: 'OverlapingRatio' not found. Taking default.\n");
        m_overlapingRatio = 0.95;
    } else {
        m_overlapingRatio = XmlCommon::getParameterValue(n).toDouble();
        if( m_overlapingRatio < 0.0 || m_overlapingRatio > 1.0 ) {
            AppendToLog("ReliabilityMerge2D Warning: 'OverlapingRatio' out of bounds. Taking default.\n");
            m_overlapingRatio = 0.95;
        }
    }

    return true;
}

void ReliabilityMerge2D::updateClassificationTable(Blob *i_blob1, Blob *i_blob2){

    //update entries for blob j in the classification map
    /*std::map<ObjectType, double>& mergeResult = m_mapCorrespondences[i_blob1][i_blob2].mergeTypeMap;
    std::map<ObjectType, double>::iterator listIt;

    for(listIt = mergeResult.begin(); listIt != mergeResult.end(); listIt++)
        m_rc->updateScoreForType(i_blob1, listIt->first, listIt->second);*/
}

void ReliabilityMerge2D::computeModelScoresForMerge(std::map<ObjectType, double>& mergeMap, Blob *i_blob1, Blob *i_blob2) {
    //Parse all models used in merge process
    /*std::map<ObjectType, SpModelInterface>::iterator it;
    std::map<ObjectType, Shape3DData> *typesList;
    std::map<ObjectType, Shape3DData>::iterator types_it, types_it_end;
    mergeType mergeResult;
    std::list<mergeType> mergeResultList;

    m_mergeBlob = Blob::mergeBlob(i_blob1, i_blob2);

    m_rc->setBlob3DFacts(m_mergeBlob, m_segmentation);
    typesList = BLOB_OCCLUSION(m_mergeBlob) == false ? BLOB_NORMAL_3DDATA(m_mergeBlob) : BLOB_OCC_3DDATA(m_mergeBlob);
    types_it_end = typesList->end();

    for(types_it = typesList->begin(); types_it != types_it_end; types_it++) {
        //Extract score for the model
        switch(m_mergingCriteria) {
            case RCReliability:
                mergeMap[types_it->first] = S3D_R(&types_it->second);
                break;
            case RCReliabilityProbability:
                mergeMap[types_it->first] = S3D_PR(&types_it->second);
                break;
            case RCDimensionalProbability:
                mergeMap[types_it->first] = S3D_DP(&types_it->second);
                break;
            case RCProbability:
            default:
                mergeMap[types_it->first] = S3D_P(&types_it->second);
        }
    }*/
}


bool ReliabilityMerge2D::canBelongToBlob(Blob *blob1, Blob *blob2) {

    if( BLOB_DP_TYPE(blob1) == MM_AOI_OUTSIDE && BLOB_DP_TYPE(blob2) == MM_AOI_OUTSIDE )
      return false;

    if( BLOB_DP_TYPE(blob1) != MM_AOI_OUTSIDE && BLOB_DP_TYPE(blob2) != MM_AOI_OUTSIDE )
      return true;

    if(    BLOB_DP_TYPE(blob1) != MM_AOI_OUTSIDE
        && BLOB_XLEFT (blob1) < BLOB_XRIGHT(blob2)
        && BLOB_XRIGHT(blob1) > BLOB_XLEFT (blob2) )
      return true;

    if(    BLOB_DP_TYPE(blob2) != MM_AOI_OUTSIDE
        && BLOB_XLEFT (blob2) < BLOB_XRIGHT(blob1)
        && BLOB_XRIGHT(blob2) > BLOB_XLEFT (blob1) )
      return true;

    return false;

}

void ReliabilityMerge2D::setInitialCorrespondences(bool **initialMergeMap, int blobsNumber, Blob **blobsVector) {
    int i,j; //Only for building a triangular table

    for(i = 0; i < blobsNumber - 1; i++)
      for(j = i + 1; j < blobsNumber; j++)
        if(    canBelongToBlob(blobsVector[i], blobsVector[j])
            && (    Blob::computeIntersectRatio(blobsVector[i], blobsVector[j]) > m_overlapingRatio
                 || Blob::computeIntersectRatio(blobsVector[j], blobsVector[i]) > m_overlapingRatio
                 || isAValidCorrespondence(blobsVector[i], blobsVector[j])
               )
          )
          initialMergeMap[i][j] = initialMergeMap[j][i] = true;
  }


//Merges blobs and calculates 3D info. result is stored in blob1
void ReliabilityMerge2D::mergeBlobsInFirstOne(Blob *i_blob1, Blob *i_blob2) {
    i_blob1->mergeBlob(i_blob2);
    delete BLOB_NORMAL_3DDATA(i_blob1);
    delete BLOB_OCC_3DDATA(i_blob1);
    BLOB_NORMAL_3DDATA(i_blob1) = NULL;
    BLOB_OCC_3DDATA(i_blob1) = NULL;
}

Blob **ReliabilityMerge2D::reduceOverlapedBlobs(Blob **blobsVector, int &blobsNumber, std::vector<Blob>& blobsList) {

    int i,j, newVectorSize;
    double overlap;
    int toEliminateCount = 0;
    bool blobsToEliminate[blobsNumber];
    memset(blobsToEliminate, false, blobsNumber*sizeof(bool));

    for(i = 0; i < blobsNumber; i++) {
        if(blobsToEliminate[i] == false) {
            for(j = 0; j < blobsNumber; j++)
                if(i != j && blobsToEliminate[j] == false) {
                    //Overlaping ratio is 1 when blobsVector[j] is inside blobsVector[i]. Then we search for elimination  blobs that are quite inside blobsVector[i].
                    overlap = Blob::computeIntersectRatio(blobsVector[j], blobsVector[i]);
                    if(    (    BLOB_TYPE(blobsVector[j]) == UNKNOWN
                             && overlap > m_overlapingRatio )
                        || (    m_eliminateOverlapedBlobs
                             && overlap > m_eliminateOverlapedBlobsOverlapingRatio
                             && BLOB_P(blobsVector[j]) < m_eliminateOverlapedBlobsProbability
                             && BLOB_R(blobsVector[j]) < m_eliminateOverlapedBlobsReliability ) ) {
                        toEliminateCount++;
                        blobsToEliminate[j] = true;
                        if(overlap < 0.99999) { //Partial overlap (0.99999 = 1 considering possible floating point error), consider information for blob.
                            mergeBlobsInFirstOne(blobsVector[i], blobsVector[j]);
                            j = -1; //If blob1 is modified, verify again correspondences
                        }
                    }
                }
        }
    }

    if(toEliminateCount) {
        newVectorSize = blobsNumber - toEliminateCount;

        for(i = 0, j = 0; i < blobsNumber; i++) {
            if(blobsToEliminate[i])
                blobsList[i].clean();
            else {
                if(i != j) {
                    blobsList[j] = blobsList[i];
                    blobsVector[j] = &blobsList[j];
                }
                j++;
            }
        }

        blobsNumber = newVectorSize;
        blobsList.resize(blobsNumber);
    }

    return blobsVector;
}


int ReliabilityMerge2D::reduceOverlapedBlobsMarkingElimination(bool *eliminationVector, bool *blobWithMergeResult, Blob **blobsVector, int blobsNumber, bool *pertinentElements) {

    int i,j;
    double overlap;
    int toEliminateCount = 0;

    for(i=0; i<blobsNumber; i++) {
        if(pertinentElements[i]) {
            for(j=0; j<blobsNumber; j++)
                if(i != j && !eliminationVector[j] && !blobWithMergeResult[j] && BLOB_TYPE(blobsVector[j]) == UNKNOWN) {
                    //Overlaping ratio is 1 when blobsVector[j] is inside blobsVector[i]. Then we search for elimination  blobs that are quite inside blobsVector[i].
                    if( (overlap = Blob::isBlob1OverlapBlob2(blobsVector[i], blobsVector[j]) ) > m_overlapingRatio){
                        toEliminateCount++;
                        eliminationVector[j] = true;
                        if(overlap < 0.99999) { //Partial overlap (0.99999 = 1 considering possible floating point error), consider information for blob.
                            mergeBlobsInFirstOne(blobsVector[i], blobsVector[j]);
                            if(BLOB_TYPE(blobsVector[i]) != UNKNOWN)
                                blobWithMergeResult[i] = true;
                            j = -1; //If blob1 is modified, verify again correspondences
                        }
                    }
                }
        }
    }

    return toEliminateCount;
}


void ReliabilityMerge2D::setGroup(int groupsVector[], bool **relations, int elementsNumber, int groupId, int startPoint, int referencePoint, int *elementsToAnalyzeVector) {
    int i;

    groupsVector[referencePoint] = groupId;

    for(i=startPoint; i<elementsNumber; i++)
        if(relations[elementsToAnalyzeVector[referencePoint]][elementsToAnalyzeVector[i]] && groupsVector[i] < 0)
            setGroup(groupsVector, relations, elementsNumber, groupId, startPoint, i, elementsToAnalyzeVector);

}

int ReliabilityMerge2D::setGroups(int groupsVector[], bool **relations, int elementsNumber, int *elementsToAnalyzeVector) {
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


//Quicksort
void ReliabilityMerge2D::orderAscending(int *array, int top, int bottom) {
    int middle;
    if (top < bottom) {
        middle = getMiddle(array, top, bottom);
        orderAscending(array, top, middle);   // sort top partition
        orderAscending(array, middle+1, bottom);    // sort bottom partition
    }
}

//Sub-function of quicksort
int ReliabilityMerge2D::getMiddle(int *array, int top, int bottom) {
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
    } while (i < j);

    return j;           // returns middle index
}


//Function to perform elimination of really overlaping blobs and merge of blobs of reduced size with respect to models of expected objects in the scene.
void ReliabilityMerge2D::preMerge(bool **initialMergeMap, std::vector<Blob>& blobsList) {

    int blobsNumber = blobsList.size();
    int i;

    if(blobsNumber <= 1)
      return;

    clearDefinedMerges();

    Blob **blobsVector = new Blob*[blobsNumber];
    std::vector<Blob>::iterator it = blobsList.begin();
    //Set blobs vector from list of blobs
    for(i = 0; i < blobsNumber; i++, it++)
        blobsVector[i] = &*it;

    tree<SMergedElement> mergeTree;
    tree<SMergedElement>::iterator iter;
    bool someMergingMade = false, someMergingMadeGlobally = false;
    int j, k, groupsNumber;
    int blobGroupVector[blobsNumber];
    int blobsToBeAnalyzed[blobsNumber], blobsToBeAnalyzedNumber;
    int numConcerned;
    bool concernedBlobs[blobsNumber];
    int reallyMergeableBlobs[blobsNumber], initiallyMergeableBlobs[blobsNumber], mergeableNumber, initiallyMergeableNumber;
    bool blobsToBeEliminated[blobsNumber], blobWithMergeResult[blobsNumber];
    int previousBlobNumber,  currentBlobIndex;
    bool ascending;

    someMergingMadeGlobally = false;

    //Initial reduction of very overlaped blobs.
    previousBlobNumber = blobsNumber;

    //Perform this operation until no more pre-merge can be done
    blobsVector = reduceOverlapedBlobs(blobsVector, blobsNumber, blobsList);

    if(blobsNumber == 1) {
        delete[] blobsVector;
        return;
    }

    if(previousBlobNumber > blobsNumber) { //Some reduction made at suppresion by overlaping
        someMergingMadeGlobally = true;
        setInitialCorrespondences(initialMergeMap, blobsNumber, blobsVector);
    }

    //Set the interesting blobs for pre merge (UNKNOWN and of REDUCED SIZE)
    blobsToBeAnalyzedNumber = 0;
    for(i=0; i<blobsNumber; i++)
        if(    BLOB_TYPE(blobsVector[i]) == UNKNOWN   //    blob is of type UNKNOWN
            && BLOB_IS_REDUCED_SIZE(blobsVector[i]) ) //AND it is a blob of REDUCED SIZE
            blobsToBeAnalyzed[blobsToBeAnalyzedNumber++] = i;

    memset(blobsToBeEliminated, false, blobsNumber*sizeof(bool));
    memset(blobWithMergeResult, false, blobsNumber*sizeof(bool));

    //Set groups to be analyzed
    groupsNumber = setGroups(blobGroupVector, initialMergeMap, blobsToBeAnalyzedNumber, blobsToBeAnalyzed);

    for(i=0; i<groupsNumber; i++) { //For each group of blobs in the image

        //Calculate concerned blobs list, according to belongness to the group
        numConcerned = 0;
        for(j=0; j<blobsToBeAnalyzedNumber; j++) {
            concernedBlobs[j] = (blobGroupVector[j] == i) ? true : false;
            numConcerned++;
        }

        if(numConcerned < 2) //Not enough blobs for merge, quit analysis for this group
            continue;

        //Set initial really mergeable blobs
        initiallyMergeableNumber = 0;
        for(j=0; j<blobsToBeAnalyzedNumber; j++) //For each blob
            if(concernedBlobs[j]) //If blob is concerned in terms of group of mergeable blobs
                initiallyMergeableBlobs[initiallyMergeableNumber++] = blobsToBeAnalyzed[j];

        for(j=0; j < initiallyMergeableNumber; j++) { //For each mergeable element of the group

            //If blob is already marked for elimination or it corresponds to a result of pre-merge, analysis is not made starting from this blob
            if(blobsToBeEliminated[initiallyMergeableBlobs[j]] || blobWithMergeResult[initiallyMergeableBlobs[j]])
                continue;

            //Set really mergeable blobs
            mergeableNumber = 0;
            for(k=0; k<initiallyMergeableNumber; k++) { //For each blob
                if(    !blobsToBeEliminated[initiallyMergeableBlobs[k]] // if blob is NOT marked for elimination
                    && !blobWithMergeResult[initiallyMergeableBlobs[k]] //AND blob is NOT storing a new premerge result
                   ) {
                    if(k == j)
                        currentBlobIndex = mergeableNumber;
                    reallyMergeableBlobs[mergeableNumber++] = initiallyMergeableBlobs[k];
                }
            }
            if(mergeableNumber < 2) //Not enough blobs for merge, quit analysis for this group
                break;

            shortestSequence = new int[mergeableNumber];

            someMergingMade = false;

            mergeTree.clear();
            shortestMergePath = mergeableNumber + 1;
            shortestSequenceMerge = NULL;

            //Position at the head
            iter = mergeTree.begin();

            //Creation of root of tree with current blob as first of the sequence of merge
            SMergedElement root(new MergedElement(currentBlobIndex, 0, NULL, mergeableNumber));
            root->initMergeData(initialMergeMap, NULL, mergeableNumber, reallyMergeableBlobs, NULL);
            mergeTree.insert(iter, root);

            //Position at the first node
            iter = mergeTree.begin();
            //Generate the rest of the tree of possible merges
            (*iter)->pathUnknown = setMergesForUsableBlobs(mergeTree, iter, 1, j, initialMergeMap, reallyMergeableBlobs, mergeableNumber, blobsVector, blobsNumber);

            //	if(!(*iter)->pathUnknown) {
            //  std::cout << "Shortest Sequence:" << std::endl;
            //  for(k=0; k <shortestMergePath; k++)
            //    std::cout << shortestSequence[k] << "\t";
            //  std::cout << std::endl;
            //}

            int mergeLimit;

            if ( !(*iter)->pathUnknown ) {        // If there is a merge configuration that gives an existing object type, check if we can perform the merge of the shortest path configuration.
                if(checkValidityOfPathsForSequence(shortestSequence, shortestMergePath, mergeTree, shortestSequenceMerge, mergeableNumber)) { //And if possible merge does not interfere with other
                                                                                                                                        //possible merges
                    //Check if there are merge possibilities, according to subtrees configurations and validate the depth of merge sequence (mergeLimit)
                    if( (mergeLimit = checkValidityOfSubtreesForSequence(shortestSequence, shortestMergePath, initialMergeMap, reallyMergeableBlobs, mergeableNumber, blobsVector, blobsNumber)) ) {
                        int realSequence[shortestMergePath];
                        for(k=0; k<=mergeLimit; k++)
                            realSequence[k] = reallyMergeableBlobs[shortestSequence[k]];

                        //getMergedBlob function requires ascending ordered elements to perform optimally
                        ascending = true;
                        for(k=1; k<=mergeLimit; k++)
                        if(realSequence[k] < realSequence[k-1]) {
                            ascending = false;
                            break;
                        }
                        if(!ascending)
                            orderAscending(realSequence, 0, mergeLimit);

                        if(mergeLimit + 1 < shortestMergePath)
                            shortestSequenceMerge = getMergedBlob(realSequence, mergeLimit + 1, blobsVector, blobsNumber, true);

                        //Modify blob of first position in sequence, copying the data of the merged one to the location
                        Blob::copyBlobData(shortestSequenceMerge, blobsVector[realSequence[0]]);

                        blobWithMergeResult[realSequence[0]] = true;

                        //Mark the rest of sequence for elimination
                        for(k=1; k<=mergeLimit; k++)
                            blobsToBeEliminated[realSequence[k]] = true;
                        someMergingMadeGlobally = someMergingMade = true;
                    }
                }
            }

            //Maybe implemented later, but it can bias the results of tracking. :S
            //else if(m_reducedUnknownBlobsMerge && !someMergingMade && j == mergeableNumber - 1) { // If tree gives just unknown configurations and we are at the
            //                                                                                      //last analysis of the group, merge until the unknown is not reduced,
            //                                                                                      //to group reduced-size unknown blobs if parameter flag activated.
            //}


            //If some merging was made on this step, perform a new reduction of overlaped blobs in pertinent elements
            if(someMergingMade) {

                bool pertinentElements[blobsNumber];
                memset(pertinentElements, false, blobsNumber*sizeof(bool));
                for(k=0; k<blobsToBeAnalyzedNumber; k++) {
                    if( blobGroupVector[k] == i && !blobsToBeEliminated[blobsToBeAnalyzed[k]] )
                        pertinentElements[blobsToBeAnalyzed[k]] = true;
                }
                //Perform new overlaped blobs reduction, focusing on pertinent blobs for group
                reduceOverlapedBlobsMarkingElimination(blobsToBeEliminated, blobWithMergeResult, blobsVector, blobsNumber, pertinentElements);
            }

            delete[] shortestSequence;
        }
    }

    if(someMergingMadeGlobally) {
        int newBlobsNumber = 0;

        for(i = 0, j = 0; i < blobsNumber; i++) {
            if(!blobsToBeEliminated[i])
                newBlobsNumber++;

            if(blobsToBeEliminated[i])
                blobsList[i].clean();
            else {
                if(i != j) {
                    blobsList[j] = blobsList[i];
                    blobsVector[j] = &blobsList[j];
                    BLOB_IDENT(blobsVector[j]) = j;
                }
                j++;
            }
        }

        blobsNumber = newBlobsNumber;
        blobsList.resize(blobsNumber);

        delete[] blobsVector;
        clearDefinedMerges();

        return;
    }

    delete[] blobsVector;
    clearDefinedMerges();

}

bool ReliabilityMerge2D::checkValidityOfPathsForSequence(int *sequence, int length, tree<SMergedElement> &mergeTree, Blob *mergedBlob, int mergeableNumber) {
    tree<SMergedElement>::iterator iter = mergeTree.begin();
    int num_valid = 1;
    bool sequenceElements[mergeableNumber];

    memset(sequenceElements, false, mergeableNumber*sizeof(bool));
    for(int i=0; i<length; i++)
      sequenceElements[sequence[i]] = true;

    tree<SMergedElement>::sibling_iterator child_iter;

    for(child_iter = mergeTree.begin(iter); child_iter != mergeTree.end(iter); child_iter++)
      if(pathValidator(sequenceElements, length, num_valid, mergeTree, child_iter, mergedBlob, mergeableNumber) == false)
        return false;

    return true;

  }

bool ReliabilityMerge2D::pathValidator(bool *sequenceElements, int length, int num_valid, tree<SMergedElement> &mergeTree, tree<SMergedElement>::iterator iter, Blob *mergedBlob, int mergeableNumber) {

    if(sequenceElements[(*iter)->getBlobIndex()]) //If element belongs to sequence
        num_valid++;

    Blob *currentMerged = (*iter)->getMergedBlob();

    //If we have found the sequence before ending the path, it means that the merged object will be inside the merged object produced by this path
    if(num_valid == length)
        return true;

    //If it is not length yet, we have to keep on searching maybe

    if(BLOB_IS_REDUCED_SIZE(currentMerged)) { //If little blob (inherently UNKNOWN), search more

        if(mergeTree.number_of_children(iter) == 0) //No more to search and no evidence of invalidation
            return true;

        //Search in siblings
        tree<SMergedElement>::sibling_iterator child_iter;
        bool validMerge = true;

        for(child_iter = mergeTree.begin(iter); child_iter != mergeTree.end(iter); child_iter++)
            validMerge &= pathValidator(sequenceElements, length, num_valid, mergeTree, child_iter, mergedBlob, mergeableNumber);

        return validMerge;

    }

    //Blob type different from unknown or unknown but not little and sequence end not reached.

    //If current merged blob is the same as the analyzed one or the analyzed blob is inside the current one, sequence is accepted.
    //Overlaping ratio is 1 when mergedBlob is inside currentMerged.
    if(mergedBlob == currentMerged || Blob::isBlob1OverlapBlob2(currentMerged, mergedBlob) > 0.9999)
        return true;

    return false;

}

int ReliabilityMerge2D::checkValidityOfSubtreesForSequence(int *sequence, int length, bool **initialMergeMap, int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber) {
    int i, j;
    tree<SMergedElement> mergeSubTree;
    tree<SMergedElement>::iterator iter;
    bool mergeValidity[length];

    memset(mergeValidity, false, length*sizeof(bool));
    mergeValidity[0] = true;

    for(i=1; i<length; i++) {
        mergeSubTree.clear();
        //Position at the head
        iter = mergeSubTree.begin();

        //Creation of root of tree with first blob of the sequence
        SMergedElement root(new MergedElement(sequence[i], 0, NULL, mergeableNumber));
        bool forbiddenElements[mergeableNumber];
        memset(forbiddenElements, false, mergeableNumber*sizeof(bool));

        for(j=0; j<i; j++)
            forbiddenElements[sequence[j]] = true;

        root->initMergeData(initialMergeMap, NULL, mergeableNumber, reallyMergeableBlobs, forbiddenElements);

        mergeSubTree.insert(iter, root);

        //Position at the first node
        iter = mergeSubTree.begin();

        mergeValidity[i] = subTreesValidator(mergeSubTree, iter, 1, i, initialMergeMap, reallyMergeableBlobs, mergeableNumber, blobsVector, blobsNumber, forbiddenElements);
        if(mergeValidity[i] == false)
            break;

    }

    mergeSubTree.clear();

    for(i=1; i<length; i++)
        if(mergeValidity[i] == false)
            return i - 1;

    return length - 1;

}

bool ReliabilityMerge2D::subTreesValidator(tree<SMergedElement> &mergeSubTree, tree<SMergedElement>::iterator iter, int level, int parent, bool **initialMergeMap,
                                         int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber, bool *forbiddenElements) {
    int i, j;
    bool leavesInserted = false, ascending;
    int mergeSequence[mergeableNumber];

    tree<SMergedElement>::iterator child_iter;

    for(i=0; i<level; i++)
        mergeSequence[i] = reallyMergeableBlobs[(*iter)->mergeSequence[i]];

    for(i=0; i<mergeableNumber; i++) {
        if((*iter)->usableList[i] == true) {

            mergeSequence[level] = reallyMergeableBlobs[i];
            leavesInserted = true;
            //getMergedBlob function requires ascending ordered elements to perform optimally
            ascending = true;
            for(j=1; j<=level; j++)
                if(mergeSequence[j] < mergeSequence[j-1]) {
                    ascending = false;
                    break;
                }
            if(!ascending)
                orderAscending(mergeSequence, 0, level);

            Blob *mergedBlob = getMergedBlob(mergeSequence, level + 1, blobsVector, blobsNumber, true);
            SMergedElement child(new MergedElement(i, level, mergedBlob, mergeableNumber));
            child->initMergeData(initialMergeMap, (*iter)->mergeSequence, mergeableNumber, reallyMergeableBlobs, forbiddenElements);
            child_iter = mergeSubTree.append_child(iter, child);
            if(BLOB_TYPE(mergedBlob) != UNKNOWN) {
                child->pathUnknown = false;
                return false;
            }

            if(!BLOB_IS_REDUCED_SIZE(mergedBlob)) {
                child->pathUnknown = false;
                return false;
            }

            if((child->pathUnknown = subTreesValidator(mergeSubTree, child_iter, level + 1, i, initialMergeMap, reallyMergeableBlobs, mergeableNumber, blobsVector, blobsNumber, forbiddenElements)) == false)
                return false;
        }
    }

    return true;

}


void ReliabilityMerge2D::clearDefinedMerges() {
    definedMerges.clear();
}

void ReliabilityMerge2D::clearDefinedMergesFromStartingKey(int value) {
    std::map<int, SpAlternativeElement>::iterator first = definedMerges.find(value);
    if(first  != definedMerges.end() )
        definedMerges.erase(first, definedMerges.end());
}

Blob *ReliabilityMerge2D::justMerge2D(int *mergeSequence, int sequenceLength, Blob **blobsVector) {
    Blob *mergedBlob = new Blob();
    int i, L = INT_MAX, R = -1, T = INT_MAX, B = -1;
    Blob *blob;
    for(i=0; i<sequenceLength; i++) {
        blob = blobsVector[mergeSequence[i]];
        if(BLOB_XLEFT(blob) < L)
            L = BLOB_XLEFT(blob);
        if(BLOB_XRIGHT(blob) > R)
            R = BLOB_XRIGHT(blob);
        if(BLOB_YTOP(blob) < T)
            T = BLOB_YTOP(blob);
        if(BLOB_YBOTTOM(blob) > B)
            B = BLOB_YBOTTOM(blob);
    }

    BLOB_XLEFT(mergedBlob) = L;
    BLOB_XRIGHT(mergedBlob) = R;
    BLOB_YTOP(mergedBlob) = T;
    BLOB_YBOTTOM(mergedBlob) = B;

    return mergedBlob;
}

//This function manages the possible merging sequences results, avoiding re-calculation of already calculated merged blobs, for optimization.
Blob *ReliabilityMerge2D::getMergedBlob(int *mergeSequence, int sequenceLength, Blob **blobsVector, int blobsNumber, bool classify) {

    //Check if it was already calculated
    int j;
    SpAlternativeElement currentPoint;
    Blob *current_blob, *previous_blob = NULL, *merge_result = NULL;

    if(definedMerges.find(mergeSequence[0]) == definedMerges.end()) { //Create initial element if not created yet
        SpAlternativeElement newPoint(new AlternativeElement(mergeSequence[0]));
        //Check if classification has been already performed
        newPoint->classified = BLOB_NORMAL_3DDATA(blobsVector[mergeSequence[0]]) ? true : false;
        definedMerges[mergeSequence[0]] = newPoint;
        currentPoint = newPoint;
    } else
        currentPoint = definedMerges[mergeSequence[0]];

    previous_blob = blobsVector[mergeSequence[0]];

    for(j=1; j<sequenceLength; j++) {

        if(currentPoint->mergedElements.find(mergeSequence[j]) == currentPoint->mergedElements.end()) { //Create element if not created yet

            SpAlternativeElement newPoint(new AlternativeElement(mergeSequence[j]));

            if(Blob::isBlob1InsideBlob2(blobsVector[mergeSequence[j]], previous_blob)) { //If new blob to merge is inside the previous level one, the result doesn't change
                current_blob = previous_blob->copy();
                if(currentPoint->classified) {
                    //If previous_blob was already classified, copy remaining classification data and set new point as classified
                    if(BLOB_NORMAL_3DDATA(current_blob) != NULL)
                        delete BLOB_NORMAL_3DDATA(current_blob);
                    BLOB_NORMAL_3DDATA(current_blob) = new std::map<ObjectType, Shape3DData>();
                    *BLOB_NORMAL_3DDATA(current_blob) = *BLOB_NORMAL_3DDATA(previous_blob);
                    if(BLOB_OCC_3DDATA(current_blob) != NULL)
                        delete BLOB_OCC_3DDATA(current_blob);
                    BLOB_OCC_3DDATA(current_blob) = new std::map<ObjectType, Shape3DData>();
                    *BLOB_OCC_3DDATA(current_blob) = *BLOB_OCC_3DDATA(previous_blob);
                    newPoint->classified = true;
                } else if(j == sequenceLength - 1) { //We are at the desired point to classify but previous blob wasn't classified
                    newPoint->classified = false;
                }
            } else { //If new blob generates a different blob when merging
                current_blob = Blob::mergeBlob(previous_blob, blobsVector[mergeSequence[j]]);
                if(j == sequenceLength - 1) { //We are at the desired point to classify
                    newPoint->classified = false;
                }
            }

            newPoint->mergedBlob = current_blob;

            currentPoint->mergedElements[mergeSequence[j]] = newPoint;
            currentPoint = newPoint;

        } else { //Element already exists
            currentPoint = currentPoint->mergedElements[mergeSequence[j]];
            current_blob = currentPoint->mergedBlob;
        }

        merge_result = current_blob;
        previous_blob = current_blob;

    }

    return merge_result;

}

bool ReliabilityMerge2D::setMergesForUsableBlobs(tree<SMergedElement> &mergeTree, tree<SMergedElement>::iterator iter, int level, int parent, bool **initialMergeMap, int *reallyMergeableBlobs, int mergeableNumber, Blob **blobsVector, int blobsNumber) {

    int i, j;
    bool leavesInserted = false, typeUnknown = true, ascending;
    int mergeSequence[mergeableNumber];
    tree<SMergedElement>::iterator child_iter;

    for(i=0; i<level; i++)
        mergeSequence[i] = reallyMergeableBlobs[(*iter)->mergeSequence[i]];

    for(i=0; i<mergeableNumber; i++) {
        if((*iter)->usableList[i] == true) {

            mergeSequence[level] = reallyMergeableBlobs[i];
            leavesInserted = true;

            //getMergedBlob function requires ascending ordered elements to perform optimally
            ascending = true;
            for(j=1; j<=level; j++)
                if(mergeSequence[j] < mergeSequence[j-1]) {
                    ascending = false;
                    break;
                }
            if(!ascending)
                orderAscending(mergeSequence, 0, level);

            Blob *mergedBlob = getMergedBlob(mergeSequence, level + 1, blobsVector, blobsNumber, true);
            SMergedElement child(new MergedElement(i, level, mergedBlob, mergeableNumber));
            child->initMergeData(initialMergeMap, (*iter)->mergeSequence, mergeableNumber, reallyMergeableBlobs, NULL);
            child_iter = mergeTree.append_child(iter, child);
            if(BLOB_TYPE(mergedBlob) != UNKNOWN) {
                typeUnknown = false;
                child->pathUnknown = false;
                if(level + 1 < shortestMergePath) {
                    shortestMergePath = level + 1;
                    memcpy(shortestSequence, child->mergeSequence, (level + 1)*sizeof(int));
                    shortestSequenceMerge = mergedBlob;
                }
            } else if(BLOB_IS_REDUCED_SIZE(mergedBlob))
                typeUnknown &= child->pathUnknown = setMergesForUsableBlobs(mergeTree, child_iter, level + 1, i, initialMergeMap, reallyMergeableBlobs, mergeableNumber, blobsVector, blobsNumber);
            else //UNKNOWN AND not little blob
                typeUnknown &= child->pathUnknown = true;
        }
    }

    if(!leavesInserted)
        return true;

    return typeUnknown;
}

void ReliabilityMerge2D::constructCorrespondenceTable() {
    std::vector<Blob>::iterator it1, it2, end = m_data->blobs.end();
    Blob *blob1 = NULL, *blob2 = NULL;
    int i,j; //Only for building a triangular table

    m_mapCorrespondences.clear();

    for(i = 0, it1 = m_data->blobs.begin(); it1 != end; i++, it1++) {
        blob1 = &*it1;
        for(j = 0, it2 = m_data->blobs.begin(); it2 != end && j < i; j++, it2++) {
            blob2 = &*it1;
            if( canBelongToBlob(blob1, blob2)) {
                m_mergeBlob = NULL;
                if(Blob::isBlob1OverlapBlob2(blob1, blob2) > m_overlapingRatio) {
                    m_mapCorrespondences[blob1][blob2].mergedBlob = NULL;
                    m_mapCorrespondences[blob1][blob2].overlaping = true;
                    m_mapCorrespondences[blob1][blob2].mergedBlob = m_mergeBlob;
                } else if( Blob::isBlob1OverlapBlob2(blob2, blob1) > m_overlapingRatio) {
                    m_mapCorrespondences[blob1][blob2].mergedBlob = NULL;
                    m_mapCorrespondences[blob1][blob2].overlaping = true;
                    m_mapCorrespondences[blob1][blob2].mergedBlob = m_mergeBlob;
                } else if(isAValidCorrespondence(blob1, blob2)) {
                    m_mapCorrespondences[blob1][blob2].mergedBlob = NULL;
                    m_mapCorrespondences[blob1][blob2].overlaping = false;
                    m_mapCorrespondences[blob1][blob2].mergedBlob = m_mergeBlob;
                }
            }
        }
    }
    m_mergeBlob = NULL;
}

void ReliabilityMerge2D::mergeBlobs(Blob *i_blob1, Blob *i_blob2) {
    //merge the blobs
    int blob_ident = BLOB_IDENT(i_blob1);
    Blob *mergeBlob = m_mapCorrespondences[i_blob1][i_blob2].mergedBlob != NULL
                    ? m_mapCorrespondences[i_blob1][i_blob2].mergedBlob
                    : m_mapCorrespondences[i_blob2][i_blob1].mergedBlob;

    BLOB_NORMAL_3DDATA(i_blob1) = NULL;
    BLOB_OCC_3DDATA(i_blob1) = NULL;

    *i_blob1 = *mergeBlob;

    delete mergeBlob;

    m_mapCorrespondences[i_blob1][i_blob2].mergedBlob = NULL;

    BLOB_IDENT(i_blob1)=blob_ident;

    std::vector<Blob>::iterator it, end = m_data->blobs.end();
    for(it = m_data->blobs.begin(); it != end; it++)
        if(&*it == i_blob2) {
            m_data->blobs.erase(it);
            break;
        }
}

ReliabilityMerge2D::~ReliabilityMerge2D() {
}

/*!
    \fn ReliabilityMerge2D::updateCorrespondenceTable(Blob *i_blob1, Blob *i_blob2)
 */
void ReliabilityMerge2D::updateCorrespondenceTable(Blob *i_blob) {
    std::vector<Blob>::iterator it, end = m_data->blobs.end();
    Blob *blob1 = NULL;

    for(it = m_data->blobs.begin(); it != end; it++) {
        blob1 = &*it;
        if(blob1 != i_blob && canBelongToBlob(i_blob, blob1)) {
            m_mergeBlob = NULL;

            if(Blob::isBlob1OverlapBlob2(i_blob, blob1) > m_overlapingRatio){
                m_mapCorrespondences[i_blob][blob1].mergedBlob = NULL;
                m_mapCorrespondences[i_blob][blob1].overlaping = true;
                m_mapCorrespondences[i_blob][blob1].mergedBlob = m_mergeBlob;
            } else if(Blob::isBlob1OverlapBlob2(blob1, i_blob) > m_overlapingRatio) {
                m_mapCorrespondences[i_blob][blob1].mergedBlob = NULL;
                m_mapCorrespondences[i_blob][blob1].overlaping = true;
                m_mapCorrespondences[i_blob][blob1].mergedBlob = m_mergeBlob;
            } else if(isAValidCorrespondence(i_blob, blob1)) {
                m_mapCorrespondences[i_blob][blob1].mergedBlob = NULL;
                m_mapCorrespondences[i_blob][blob1].overlaping = false;
                m_mapCorrespondences[i_blob][blob1].mergedBlob = m_mergeBlob;
            }
        }
    }
    m_mergeBlob = NULL;
}


void ReliabilityMerge2D::removeCorrespondencesWithBlob(Blob *i_blob) {
    std::map<Blob *, std::map<Blob *, mergeData> >::iterator lineIter;
    std::map<Blob *, mergeData>::iterator elementIter;

    //Free not useful blob merge results
    for(elementIter = (m_mapCorrespondences[i_blob]).begin(); elementIter != (m_mapCorrespondences[i_blob]).end(); elementIter++) {
        if((*elementIter).second.mergedBlob != NULL) {
            delete (*elementIter).second.mergedBlob;
            (*elementIter).second.mergedBlob = NULL;
        }
    }

    //Delete entries for blob in the correspondence map
    m_mapCorrespondences.erase(i_blob);
    //Delete correspondences between other blobs and blob
    for(lineIter = m_mapCorrespondences.begin(); lineIter != m_mapCorrespondences.end(); lineIter++) {
        for(elementIter = (*lineIter).second.begin(); elementIter != (*lineIter).second.end(); elementIter++) {
            if((*elementIter).first == i_blob) {
                if((*elementIter).second.mergedBlob != NULL) {
                    delete (*elementIter).second.mergedBlob;
                    (*elementIter).second.mergedBlob = NULL;
                }
            }
        }
        (*lineIter).second.erase(i_blob);
    }
}

/*!
  \fn ReliabilityMerge2D::removeCorrespondence(Blob *i_blob1, Blob *i_blob2)
*/
void ReliabilityMerge2D::removeCorrespondence(Blob *i_blob1, Blob *i_blob2) {
    if(m_mapCorrespondences[i_blob1][i_blob2].mergedBlob != NULL) {
        delete m_mapCorrespondences[i_blob1][i_blob2].mergedBlob;
        m_mapCorrespondences[i_blob1][i_blob2].mergedBlob = NULL;
    }
    m_mapCorrespondences[i_blob1].erase(i_blob2);
}

ReliabilityMerge2D::MergedElement::MergedElement(int index, int i_level, Blob *mergedBlob, int mergeableBlobsNumber) {
    blobIndex = index;
    level = i_level;
    mergeResult = mergedBlob;
    usableList = new bool[mergeableBlobsNumber];
    memset(usableList, false, mergeableBlobsNumber*sizeof(bool));
    usedElements = new bool[mergeableBlobsNumber];
    memset(usedElements, false, mergeableBlobsNumber*sizeof(bool));
    mergeSequence = new int[mergeableBlobsNumber];
    pathUnknown = true;
}

ReliabilityMerge2D::MergedElement::~MergedElement() {
    delete[] usableList;
    delete[] usedElements;
    delete[] mergeSequence;
}


Blob *ReliabilityMerge2D::MergedElement::getMergedBlob() {return mergeResult;}
int ReliabilityMerge2D::MergedElement::getBlobIndex() {return blobIndex;}

void ReliabilityMerge2D::MergedElement::initMergeData(bool **initialMergeMap, int *previousMergeSequence, int mergeableNumber, int *reallyMergeableBlobs, bool *forbiddenElements) {
    int i, j, currentUsed;
    if(level && previousMergeSequence)
        memcpy(mergeSequence, previousMergeSequence, sizeof(int)*level);
    mergeSequence[level] = blobIndex;

    for(i=0; i <= level; i++)
        usedElements[mergeSequence[i]] = true;

    for(i=0; i <= level; i++) {
        currentUsed = mergeSequence[i];
        for(j=0; j<mergeableNumber; j++)
            if(currentUsed != j && usedElements[j] == false && initialMergeMap[reallyMergeableBlobs[currentUsed]][reallyMergeableBlobs[j]])
                usableList[j] = true;
    }

    //Set forbidden elements if provided
    if(forbiddenElements)
        for(i=0; i<mergeableNumber; i++)
            if(forbiddenElements[i])
                usableList[i] = false;
}

ReliabilityMerge2D::mergeData::mergeData() {}

ReliabilityMerge2D::mergeData::~mergeData() {}
