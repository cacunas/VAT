//Created by Marcos Zuniga 2005-2007

#include "ReliabilityClassification.h"
#include "MODULES/ModuleInterface.h"
#include "VideoAnalysis.h"
#include "world_def.h"
#include "calibration.h"
#include "geometric.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cassert>

int ReliabilityClassification::RC_MAX_PIXELS = 200;     //Maximal number of pixels to be analyzed
int ReliabilityClassification::RC_MIN_PIXELS = 50;     //Minimal number of pixels to be analyzed
double ReliabilityClassification::m_pixelDensity = 0.50; //Desired rate of pixels to analyze in blob.


ReliabilityClassification::ReliabilityClassification() {
    initialized = false;
    imageWorkingAreaNotComputed = true;

    ALPHA_STEP = M_PI/40.0;
    H_STEP_CM = 5.0;
    m_rcntop = 10;

    const int aux[4][4] = {  {0, 2, 0, 1}   //Given the existence of two types of equations to relate two variables
                            ,{2, 0, 1, 0}   //this matrix stablishes the types of relations between variables.
                            ,{0, 1, 0, 2}   //0: no relation, 1:equation type 1, 2:equation type 2.
                            ,{1, 0, 2, 0} };
    memcpy(eq_relations, aux, sizeof(eq_relations));
}

ReliabilityClassification::ReliabilityClassification(Datapool *i_data): m_data(i_data), m_context(i_data->sceneModel)  {
    initialized = false;
    imageWorkingAreaNotComputed = true;

    ALPHA_STEP = M_PI/40.0;
    H_STEP_CM = 5.0;
    m_rcntop = 10;

    const int aux[4][4] = {  {0, 2, 0, 1}   //Given the existence of two types of equations to relate two variables
                            ,{2, 0, 1, 0}   //this matrix stablishes the types of relations between variables.
                            ,{0, 1, 0, 2}   //0: no relation, 1:equation type 1, 2:equation type 2.
                            ,{1, 0, 2, 0} };
    memcpy(eq_relations, aux, sizeof(eq_relations));
}


void  ReliabilityClassification::checkSubModelsSize( int &reduced, int &excesive, SpModelInterface objectModel, ObjectType modelType, Blob *blob) {

    std::map<ObjectSubtype, SpModelInterface>::iterator modelsIt;
    std::map<QString, Criterion> *criterias;
    std::map<QString, Criterion>::iterator criteriasIt;
    SpModelInterface objectSubModel;
    Criterion *wm, *lm, *hm;
    ObjectSubtype modelSubType;
    int subreduced, subexcesive;
    reduced = 1, excesive = 1;
    //Check limit sizes for all models

    for(modelsIt=(objectModel->m_mapPostures).begin(); modelsIt!=(objectModel->m_mapPostures).end(); modelsIt++) {

        modelSubType = (*modelsIt).first;
        objectSubModel = (*modelsIt).second;

        criterias = &(objectSubModel->m_mapCriteria);

        for(criteriasIt=criterias->begin(); criteriasIt!=criterias->end(); criteriasIt++) {
            if((*criteriasIt).first == "height")
                hm = &(*criteriasIt).second;
            else if((*criteriasIt).first == "width")
                wm = &(*criteriasIt).second;
            else if((*criteriasIt).first == "depth")
                lm = &(*criteriasIt).second;
        }

        model_hmin  = hm->spFunction->getMin();
        model_hmax  = hm->spFunction->getMax();
        model_lmin  = lm->spFunction->getMin();
        model_lmax  = lm->spFunction->getMax();
        model_wmin  = wm->spFunction->getMin();
        model_wmax  = wm->spFunction->getMax();

        subreduced  = checkIfBlobIsReducedSize(blob, model_hmin, model_lmin, model_wmin);
        subexcesive = checkIfBlobIsExcesiveSize(blob, model_hmax, model_lmax, model_wmax);

        reduced  &= subreduced;
        excesive &= subexcesive;

        if( subexcesive || (!BLOB_DP_TYPE(blob) && subreduced) )
            subSizeOkForAnalysis[modelType][modelSubType] = false;
        else
            subSizeOkForAnalysis[modelType][modelSubType] = true;

    }

}

void ReliabilityClassification::checkSubModelsSize(int &reduced, int &excesive, SpModelInterface objectModel, ObjectType modelType, Shape3DData *s3d, int position, DetectionProblemType dptype) {

    std::map<ObjectSubtype, SpModelInterface>::iterator modelsIt;
    SpModelInterface objectSubModel;
    ObjectSubtype modelSubType;
    int subreduced, subexcesive;
    reduced = 1, excesive = 1;
    //Check limit sizes for all models
    int j;

    for(modelsIt=(objectModel->m_mapPostures).begin(); modelsIt!=(objectModel->m_mapPostures).end(); modelsIt++) {
      
        modelSubType = (*modelsIt).first;
        objectSubModel = (*modelsIt).second;

        j = MobileObject::objectSubModelMap[modelType][modelSubType];

        model_wmin = MobileObject::g_postureMinw[j];
        model_wmax = MobileObject::g_postureMaxw[j];
        model_lmin = MobileObject::g_postureMinl[j];
        model_lmax = MobileObject::g_postureMaxl[j];
        model_hmin = MobileObject::g_postureMinh[j];
        model_hmax = MobileObject::g_postureMaxh[j];

        subreduced  = checkIfBlobIsReducedSize(s3d, model_hmin, model_lmin, model_wmin, position);
        subexcesive = checkIfBlobIsExcesiveSize(s3d, model_hmax, model_lmax, model_wmax, position);

        reduced  &= subreduced;
        excesive &= subexcesive;

        if( dptype == MM_DP_NONE && (subreduced || subexcesive) )
            subSizeOkForAnalysis[modelType][modelSubType] = false;
        else
            subSizeOkForAnalysis[modelType][modelSubType] = true;
    }
}


void ReliabilityClassification::checkBlobSize(Blob *blob) {

    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<QString, Criterion> *criterias;
    std::map<QString, Criterion>::iterator criteriasIt;
    SpModelInterface objectModel;
    Criterion *wm, *lm, *hm;
    bool rigid;
    ObjectType modelType;
    int reduced, excesive;
    int totally_reduced = 1, totally_excesive = 1;
    //Check limit sizes for all models
    if(BLOB_DP_TYPE(blob) & (MM_CAM_OCCL_MASK | MM_OBJECT_OCCL_MASK)) { //If occlusion is present, objects
                                                                          //can finally have a good size
        for(modelsIt=m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) {

            modelType = (*modelsIt).first;
            objectModel = (*modelsIt).second;
            rigid = objectModel->IsRigid;

            std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
            SpModelInterface objectSubModel;

            for(subModelsIt=(objectModel->m_mapPostures).begin(); subModelsIt!=(objectModel->m_mapPostures).end(); subModelsIt++)
                subSizeOkForAnalysis[modelType][(*subModelsIt).first] = true;

            sizeOkForAnalysis[modelType] = true;
        }

        BLOB_IS_REDUCED_SIZE(blob) = false;
        BLOB_IS_EXCESIVE_SIZE(blob) = false;
        return;
    }

    for(modelsIt=m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) {

        modelType = (*modelsIt).first;
        objectModel = (*modelsIt).second;
        rigid = objectModel->IsRigid;

        criterias = &(objectModel->m_mapCriteria);

        for(criteriasIt=criterias->begin(); criteriasIt!=criterias->end(); criteriasIt++) {
            if((*criteriasIt).first == "height")
                hm = &(*criteriasIt).second;
            else if((*criteriasIt).first == "width")
                wm = &(*criteriasIt).second;
            else if((*criteriasIt).first == "depth")
                lm = &(*criteriasIt).second;
        }

        model_hmin  = hm->spFunction->getMin();
        model_hmax  = hm->spFunction->getMax();
        model_lmin  = lm->spFunction->getMin();
        model_lmax  = lm->spFunction->getMax();
        model_wmin  = wm->spFunction->getMin();
        model_wmax  = wm->spFunction->getMax();

        reduced  = checkIfBlobIsReducedSize(blob, model_hmin, model_lmin, model_wmin);
        excesive = checkIfBlobIsExcesiveSize(blob, model_hmax, model_lmax, model_wmax);

        if(!rigid && !reduced && !excesive) //Analyze submodels of a postural object
            checkSubModelsSize(reduced, excesive, objectModel, modelType, blob);

        totally_reduced  &= reduced;
        totally_excesive &= excesive;

        if(reduced || excesive)
            sizeOkForAnalysis[modelType] = false;
        else
            sizeOkForAnalysis[modelType] = true;
    }

    BLOB_IS_REDUCED_SIZE(blob) = totally_reduced;
    BLOB_IS_EXCESIVE_SIZE(blob) = totally_excesive;
}

bool ReliabilityClassification::checkBlobSize(int &reduced, int &excesive, Shape3DData *s3d, ObjectType type, int position, DetectionProblemType dptype) {

    std::map<std::string, Criterion>::iterator criteriasIt;
    SpModelInterface objectModel;
    bool rigid;

    objectModel = m_mapModels[type];
    rigid = objectModel->IsRigid;
	 
    int i = MobileObject::objectModelMap[type];

    model_wmin = MobileObject::objectModelMinWidth[i];
    model_wmax = MobileObject::objectModelMaxWidth[i];
    model_wmean = MobileObject::objectModelMeanWidth[i];
    model_lmin = MobileObject::objectModelMinLength[i];
    model_lmax = MobileObject::objectModelMaxLength[i];
    model_lmean = MobileObject::objectModelMeanLength[i];
    model_hmin = MobileObject::objectModelMinHeight[i];
    model_hmax = MobileObject::objectModelMaxHeight[i];
    model_hmean = MobileObject::objectModelMeanHeight[i];

    reduced  = checkIfBlobIsReducedSize(s3d, model_hmin, model_lmin, model_wmin, position);
    excesive = checkIfBlobIsExcesiveSize(s3d, model_hmax, model_lmax, model_wmax, position);

    if(!rigid && !reduced && !excesive) //Analyze submodels of a postural object
        checkSubModelsSize(reduced, excesive, objectModel, type, s3d, position, dptype);

    if( (reduced && dptype == MM_DP_NONE) || excesive ) {
        return sizeOkForAnalysis[type] = false;
    }

    return sizeOkForAnalysis[type] = true;

}


int ReliabilityClassification::checkIfBlobIsReducedSize(Blob *blob, double hmin, double lmin, double wmin) {

    int position = BLOB_POSITION(blob);
    int initial_index;
    double x2d, y2d, x3d, y3d, x3d2, y3d2, alpha, sina, cosa;
    int move_right, move_top;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    double Wb = BLOB_WIDTH(blob), Wm1, Wm2, Hb = BLOB_HEIGHT(blob), Hm1, Hm2;

    switch(position) {
        case 0:
            x2d = BLOB_XRIGHT(blob); y2d = BLOB_YBOTTOM(blob); initial_index = 2; move_right = 0; move_top = 0;
            break;
        case 1:
            x2d = BLOB_XCENTER(blob); y2d = BLOB_YBOTTOM(blob); initial_index = 2; move_right = 1; move_top = 0;
            break;
        case 2:
            x2d = BLOB_XLEFT(blob); y2d = BLOB_YBOTTOM(blob); initial_index = 1; move_right = 0; move_top = 0;
            break;
        case 3:
            x2d = BLOB_XRIGHT(blob); y2d = BLOB_YCENTER(blob); initial_index = 3; move_right = 0; move_top = 1;
            break;
        case 4:
            x2d = BLOB_XCENTER(blob); y2d = BLOB_YCENTER(blob); initial_index = 3; move_right = 1; move_top = 1;
            break;
        case 5:
            x2d = BLOB_XLEFT(blob); y2d = BLOB_YCENTER(blob); initial_index = 0; move_right = 0; move_top = 1;
            break;
        case 6:
            x2d = BLOB_XRIGHT(blob); y2d = BLOB_YTOP(blob); initial_index = 3; move_right = 0; move_top = 0;
            break;
        case 7:
            x2d = BLOB_XCENTER(blob); y2d = BLOB_YTOP(blob); initial_index = 3; move_right = 1; move_top = 0;
            break;
        case 8:
            x2d = BLOB_XLEFT(blob); y2d = BLOB_YTOP(blob); initial_index = 3; move_right = 0; move_top = 0;
            break;
    }
    
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, y2d, 0.0, &x3d, &y3d);
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + 5, y2d, 0.0, &x3d2, &y3d2);
    
    alpha = (x3d2 - x3d) ? atan2(y3d2 - y3d, x3d2 - x3d) : M_PI/2.0;
    sina = sin(alpha);
    cosa = cos(alpha);
    
    //First combination of dimensions (w,l)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*(move_right*lmin*cosa/2.0 - move_top*wmin*sina/2.0),
                               y3d + move_right*lmin*sina/2.0 + move_top*wmin*cosa/2.0,
                               initial_index, alpha, beta_direction, wmin, lmin, hmin);
    Wm1 = RECT_WIDTH(&bb2D);
    Hm1 = RECT_HEIGHT(&bb2D);

    //Second combination of dimensions (l,w)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*(move_right*wmin*cosa/2.0 - move_top*lmin*sina/2.0),
                               y3d + move_right*wmin*sina/2.0 + move_top*lmin*cosa/2.0,
                               initial_index, alpha, beta_direction, lmin, wmin, hmin);
    Wm2 = RECT_WIDTH(&bb2D);
    Hm2 = RECT_HEIGHT(&bb2D);
    
    if((Wb+1 <= Wm1-1 || Hb+1 <= Hm1-1) && (Wb+1 <= Wm2-1  || Hb+1 <= Hm2-1) && Wb*Hb/Wm1*Hm1 < 0.9  && Wb*Hb/Wm2*Hm2 < 0.9)
      return 1;

    return 0;

}

int ReliabilityClassification::checkIfBlobIsReducedSize(Shape3DData *s3d, double hmin, double lmin, double wmin, int position) {

    int initial_index;
    double x2d, y2d, x3d, y3d, x3d2, y3d2, alpha, sina, cosa;
    int move_right, move_top;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    double Wb = S3D_WIDTH(s3d), Wm1, Wm2, Hb = S3D_HEIGHT(s3d), Hm1, Hm2;

    switch(position) {
        case 0:
            x2d = S3D_XRIGHT(s3d); y2d = S3D_YBOTTOM(s3d); initial_index = 2; move_right = 0; move_top = 0;
            break;
        case 1:
            x2d = S3D_XCENTER(s3d); y2d = S3D_YBOTTOM(s3d); initial_index = 2; move_right = 1; move_top = 0;
            break;
        case 2:
            x2d = S3D_XLEFT(s3d); y2d = S3D_YBOTTOM(s3d); initial_index = 1; move_right = 0; move_top = 0;
            break;
        case 3:
            x2d = S3D_XRIGHT(s3d); y2d = S3D_YCENTER(s3d); initial_index = 3; move_right = 0; move_top = 1;
            break;
        case 4:
            x2d = S3D_XCENTER(s3d); y2d = S3D_YCENTER(s3d); initial_index = 3; move_right = 1; move_top = 1;
            break;
        case 5:
            x2d = S3D_XLEFT(s3d); y2d = S3D_YCENTER(s3d); initial_index = 0; move_right = 0; move_top = 1;
            break;
        case 6:
            x2d = S3D_XRIGHT(s3d); y2d = S3D_YTOP(s3d); initial_index = 3; move_right = 0; move_top = 0;
            break;
        case 7:
            x2d = S3D_XCENTER(s3d); y2d = S3D_YTOP(s3d); initial_index = 3; move_right = 1; move_top = 0;
            break;
        case 8:
            x2d = S3D_XLEFT(s3d); y2d = S3D_YTOP(s3d); initial_index = 3; move_right = 0; move_top = 0;
            break;
    }
    
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, y2d, 0.0, &x3d, &y3d);
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + 5, y2d, 0.0, &x3d2, &y3d2);
    
    alpha = (x3d2 - x3d) ? atan2(y3d2 - y3d, x3d2 - x3d) : M_PI/2.0;
    sina = sin(alpha);
    cosa = cos(alpha);
    
    //First combination of dimensions (w,l)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*(move_right*lmin*cosa/2.0 - move_top*wmin*sina/2.0),
                               y3d + move_right*lmin*sina/2.0 + move_top*wmin*cosa/2.0, initial_index, alpha, beta_direction, wmin, lmin, hmin);
    Wm1 = RECT_WIDTH(&bb2D);
    Hm1 = RECT_HEIGHT(&bb2D);

    //Second combination of dimensions (l,w)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*(move_right*wmin*cosa/2.0 - move_top*lmin*sina/2.0),
                               y3d + move_right*wmin*sina/2.0 + move_top*lmin*cosa/2.0, initial_index, alpha, beta_direction, lmin, wmin, hmin);
    Wm2 = RECT_WIDTH(&bb2D);
    Hm2 = RECT_HEIGHT(&bb2D);
    
    //    if((Wb <= Wm1 || Hb <= Hm1) && (Wb <= Wm2  || Hb <= Hm2))
    if((Wb+1 <= Wm1-1 || Hb+1 <= Hm1-1) && (Wb+1 <= Wm2-1  || Hb+1 <= Hm2-1) && Wb*Hb/Wm1*Hm1 < 0.9  && Wb*Hb/Wm2*Hm2 < 0.9)
        return 1;

    return 0;
}

int ReliabilityClassification::checkIfBlobIsExcesiveSize(Blob *blob, double hmax, double lmax, double wmax) {
    int position = BLOB_POSITION(blob);
    int initial_index;
    double x2d, y2d, x3d, y3d, x3d2, y3d2, alpha, sina, cosa;
    int move;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    double Wb = BLOB_WIDTH(blob), Wm1, Wm2, Hb = BLOB_HEIGHT(blob), Hm1, Hm2;

    x2d = BLOB_XCENTER(blob);

    switch(position) {
        case 0: case 1: case 2:
            y2d = BLOB_YBOTTOM(blob); initial_index = 0; move = 0;
            break;
        case 3: case 4: case 5:
            y2d = BLOB_YCENTER(blob); initial_index = 2; move = 1;
            break;
        case 6: case 7: case 8:
            y2d = BLOB_YTOP(blob); initial_index = 2; move = 0;
    }

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, y2d, 0.0, &x3d, &y3d);
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + 5, y2d + 5, 0.0, &x3d2, &y3d2);

    alpha = (x3d2 - x3d) ? atan2(y3d2 - y3d, x3d2 - x3d) : M_PI/2.0;
    sina = sin(alpha);
    cosa = cos(alpha);
    
    //First combination of dimensions (w,l)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*move*(lmax*cosa - wmax*sina)/2.0,
                               y3d + move*(lmax*sina + wmax*cosa)/2.0, initial_index, alpha, beta_direction, wmax, lmax, hmax);
    Wm1 = RECT_WIDTH(&bb2D);
    Hm1 = RECT_HEIGHT(&bb2D);

    //Second combination of dimensions (l,w)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*move*(wmax*cosa - lmax*sina)/2.0,
                               y3d + move*(wmax*sina + lmax*cosa)/2.0, initial_index, alpha, beta_direction, lmax, wmax, hmax);
    Wm2 = RECT_WIDTH(&bb2D);
    Hm2 = RECT_HEIGHT(&bb2D);
    
    if((Wb-1 >= Wm1+1 || Hb-1 >= Hm1+1) && (Wb-1 >= Wm2+1  || Hb-1 >= Hm2+1) && Wm1*Hm1/Wb*Hb < 0.9  && Wm2*Hm2/Wb*Hb < 0.9)
        //if((Wb+1 <= Wm1-1 || Hb+1 <= Hm1-1) && (Wb+1 <= Wm2-1  || Hb+1 <= Hm2-1))
        return 1;

    return 0;

}

int ReliabilityClassification::checkIfBlobIsExcesiveSize(Shape3DData *s3d, double hmax, double lmax, double wmax, int position) {
    int initial_index;
    double x2d, y2d, x3d, y3d, x3d2, y3d2, alpha, sina, cosa;
    int move;
    Parallelpiped bb3D;
    Rectangle<int> bb2D;
    double Wb = S3D_WIDTH(s3d), Wm1, Wm2, Hb = S3D_HEIGHT(s3d), Hm1, Hm2;

    x2d = S3D_XCENTER(s3d);

    switch(position) {
        case 0: case 1: case 2:
            y2d = S3D_YBOTTOM(s3d); initial_index = 0; move = 0;
            break;
        case 3: case 4: case 5:
            y2d = S3D_YCENTER(s3d); initial_index = 2; move = 1;
            break;
        case 6: case 7: case 8:
            y2d = S3D_YTOP(s3d); initial_index = 2; move = 0;
    }

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, y2d, 0.0, &x3d, &y3d);
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + 5, y2d + 5, 0.0, &x3d2, &y3d2);

    alpha = (x3d2 - x3d) ? atan2(y3d2 - y3d, x3d2 - x3d) : M_PI/2.0;
    sina = sin(alpha);
    cosa = cos(alpha);
    
    //First combination of dimensions (w,l)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*move*(lmax*cosa - wmax*sina)/2.0,
                               y3d + move*(lmax*sina + wmax*cosa)/2.0, initial_index, alpha, beta_direction, wmax, lmax, hmax);
    Wm1 = RECT_WIDTH(&bb2D);
    Hm1 = RECT_HEIGHT(&bb2D);

    //Second combination of dimensions (l,w)
    bb3D.getFromInitial3DPoint(m_context, &bb2D,
                               x3d + beta_direction*move*(wmax*cosa - lmax*sina)/2.0,
                               y3d + move*(wmax*sina + lmax*cosa)/2.0, initial_index, alpha, beta_direction, lmax, wmax, hmax);
    Wm2 = RECT_WIDTH(&bb2D);
    Hm2 = RECT_HEIGHT(&bb2D);

    if((Wb-1 >= Wm1+1 || Hb-1 >= Hm1+1) && (Wb-1 >= Wm2+1  || Hb-1 >= Hm2+1) && Wm1*Hm1/Wb*Hb < 0.9  && Wm2*Hm2/Wb*Hb < 0.9)
        //    if((Wb >= Wm1 || Hb >= Hm1) && (Wb >= Wm2  || Hb >= Hm2))
        return 1;

    return 0;
  }

  //Set object type, occlussion type and 3D bounding box for all blobs  
void ReliabilityClassification::constructClassifMap(std::vector<Blob>& i_blobList, QImage *i_segm) {
    //FOR TESTING...
    //Blob *new_blob = blob_new();
    //BLOB_NEXT(new_blob) = NULL;
    //BLOB_IDENT(new_blob) = 1;
    //BLOB_XLEFT(new_blob)  = 220;
    //BLOB_XRIGHT(new_blob) = 235;
    //BLOB_YTOP(new_blob)   = 120;
    //BLOB_YBOTTOM(new_blob)= 140;
    //BLOB_WIDTH(new_blob)  = BLOB_XRIGHT(new_blob) - BLOB_XLEFT(new_blob) + 1;
    //BLOB_HEIGHT(new_blob) = BLOB_YBOTTOM(new_blob)- BLOB_YTOP(new_blob) + 1;
    //BLOB_NEXT(i_blobList) = new_blob;
    //REAL CODE

    //Clear the table
    m_mapClassif.clear();
        
#ifdef RC_OUTPUT_1
    AppendToLog("*****************************************************************");
    AppendToLog("CAM. POSITION:\nX: "+ camx +"\tY: "+ camy +"\tZ: "+ camz +std::endl;
    AppendToLog("*****************************************************************");
    AppendToLog("BLOB LIST:\n");
#endif

    //Order blobs according to proximity to projected focal point
    Blob::orderByProximityToPoint(i_blobList, SM_CAMERA_X2D_FOC_POINT(m_context), SM_CAMERA_Y2D_FOC_POINT(m_context));
    //REAL_CODE
    std::vector<Blob>::iterator it, it_end = i_blobList.begin();

    for(it = i_blobList.begin(); it != it_end; it++) {
        //Set 3D information from 2D blob, segmentation and context information
        setBlob3DFacts(&(*it), i_segm);
        buildClassifTable(&(*it));
    }

#ifdef RC_OUTPUT_1
    std::map<int, std::map<ObjectType, double> >::iterator classifIt;
    std::map<ObjectType, double>::iterator typeIt;
    AppendToLog("CLASSIFICATION MAP");
    for(classifIt=m_mapClassif.begin(); classifIt!=m_mapClassif.end(); classifIt++){
      AppendToLog((*classifIt).first + ":\t";
      for(typeIt=(*classifIt).second.begin(); typeIt!=(*classifIt).second.end(); typeIt++){
        AppendToLog("(" + Blob::getNameFromType((*typeIt).first) + "," + (*typeIt).second + ")\t";
      }
      AppendToLog(std::endl;
    }
#endif  

}

  //Set final object type and occlusion type for a blob
void ReliabilityClassification::buildClassifTable(Blob *blob) {

    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
    std::map<ObjectSubtype, SpModelInterface> *subModels;
    double max, current;
    for(modelsIt=m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) 
        switch(m_rcInterCriteria) {
            case RCReliability:
                if(((*modelsIt).second)->IsRigid)
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first]
                        = BLOB_OCCLUSION(blob) ? ((m_ntopocc[(*modelsIt).first] > 0) ? S3D_R(m_topocc[(*modelsIt).first][0]) : 0.0)
                        : ((m_ntop[(*modelsIt).first] > 0) ? S3D_R(m_top[(*modelsIt).first][0])    : 0.0);
                else {
                    max = 0.0;
                    subModels = &(((*modelsIt).second)->m_mapPostures);
                    for(subModelsIt=subModels->begin(); subModelsIt!=subModels->end(); subModelsIt++) {
                        current = BLOB_OCCLUSION(blob)
                                ? ((m_sub_ntopocc[(*modelsIt).first][(*subModelsIt).first]>0)
                                   ? S3D_R(m_sub_topocc[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0)
                                : ((m_sub_ntop[(*modelsIt).first][(*subModelsIt).first]>0)
                                   ? S3D_R(m_sub_top[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0);
                        if(current > max)
                            max = current;
                    }
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first] = max;
                }
                break;
            case RCReliabilityProbability:
                if(((*modelsIt).second)->IsRigid)
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first]
                        = BLOB_OCCLUSION(blob) ? ((m_ntopocc[(*modelsIt).first]>0) ? S3D_PR(m_topocc[(*modelsIt).first][0]) : 0.0)
                                               : ((m_ntop[(*modelsIt).first]>0)    ? S3D_PR(m_top[(*modelsIt).first][0])    : 0.0);
                else {
                    max = 0.0;
                    subModels = &(((*modelsIt).second)->m_mapPostures);
                    for(subModelsIt=subModels->begin(); subModelsIt!=subModels->end(); subModelsIt++) {
                        current = BLOB_OCCLUSION(blob)
                                ? ((m_sub_ntopocc[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_PR(m_sub_topocc[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0)
                                : ((m_sub_ntop[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_PR(m_sub_top[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0);
                        if(current > max)
                            max = current;
                    }
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first] = max;
                }
                break;
            case RCDimensionalProbability:
                if(((*modelsIt).second)->IsRigid)
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first]
                        = BLOB_OCCLUSION(blob) ? ((m_ntopocc[(*modelsIt).first]>0) ? S3D_DP(m_topocc[(*modelsIt).first][0]) : 0.0)
                                               : ((m_ntop[(*modelsIt).first]>0)    ? S3D_DP(m_top[(*modelsIt).first][0])    : 0.0);
                else {
                    max = 0.0;
                    subModels = &(((*modelsIt).second)->m_mapPostures);
                    for(subModelsIt=subModels->begin(); subModelsIt!=subModels->end(); subModelsIt++) {
                        current = BLOB_OCCLUSION(blob)
                                ? ((m_sub_ntopocc[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_DP(m_sub_topocc[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0)
                                : ((m_sub_ntop[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_DP(m_sub_top[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0);
                        if(current > max)
                            max = current;
                    }
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first] = max;
                }
                break;
            case RCProbability:
            default:
                if(((*modelsIt).second)->IsRigid)
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first]
                        = BLOB_OCCLUSION(blob) ? ((m_ntopocc[(*modelsIt).first]>0) ? S3D_P(m_topocc[(*modelsIt).first][0]) : 0.0)
                                               : ((m_ntop[(*modelsIt).first]>0)    ? S3D_P(m_top[(*modelsIt).first][0])    : 0.0);
                else {
                    max = 0.0;
                    subModels = &(((*modelsIt).second)->m_mapPostures);
                    for(subModelsIt=subModels->begin(); subModelsIt!=subModels->end(); subModelsIt++) {
                        current = BLOB_OCCLUSION(blob)
                                ? ((m_sub_ntopocc[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_P(m_sub_topocc[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0)
                                : ((m_sub_ntop[(*modelsIt).first][(*subModelsIt).first]>0)
                                    ? S3D_P(m_sub_top[(*modelsIt).first][(*subModelsIt).first][0]) : 0.0);
                        if(current > max)
                            max = current;
                    }
                    m_mapClassif[BLOB_IDENT(blob)][(*modelsIt).first] = max;
                }
        }
}

//Set pertinent analysis window of the image
void ReliabilityClassification::computeImageWorkingArea(QImage *i_segm) {
    int XInterval = static_cast<int>(this->m_HToleranceCoeff * i_segm->width());
    int YInterval = static_cast<int>(this->m_VToleranceCoeff * i_segm->height());
    this->m_XMinWA = XInterval;
    this->m_XMaxWA = i_segm->width() - XInterval - 1;
    this->m_YMinWA = YInterval;
    this->m_YMaxWA = i_segm->height() - YInterval - 1;
}
  
//Determine if there is border occlusion and return the border occlusion type, according to analysis window of the image.
DetectionProblemType ReliabilityClassification::isBlobOnImageBorder(Blob *blob) {
  
    int XMinBlob = BLOB_XLEFT(blob);
    int XMaxBlob = BLOB_XRIGHT(blob);
    int YMinBlob = BLOB_YTOP(blob);	
    int YMaxBlob = BLOB_YBOTTOM(blob);
    
    if (YMinBlob <= this->m_YMinWA)
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int)BLOB_DP_TYPE(blob) | MM_CAM_OCCL_TOP);
    else if (YMaxBlob >= this->m_YMaxWA)
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int)BLOB_DP_TYPE(blob) | MM_CAM_OCCL_BOTTOM);

    if (XMinBlob <= this->m_XMinWA)
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int)BLOB_DP_TYPE(blob) | MM_CAM_OCCL_LEFT);
    else if (XMaxBlob >= this->m_XMaxWA)
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int)BLOB_DP_TYPE(blob) | MM_CAM_OCCL_RIGHT);
    
    return (DetectionProblemType)(BLOB_DP_TYPE(blob) & (MM_CAM_OCCL_RIGHT | MM_CAM_OCCL_LEFT | MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP));
}

//Checks if exists vertical or horizontal intersection between two pairs of intervals
bool ReliabilityClassification::thereIsIntersection(interval_t polX, interval_t polY, interval_t blobX, interval_t blobY) {
    int 
        i1 = Interval::intersect(polX, polX, blobX),
        i2 = Interval::intersect(polY, polY, blobY);
    if( i1 || i2 )
        return true;
    return false;
}

//Checks if all possible base points analyzed are inside a context object
//Analysis is made according to certain points of the blob inside or outside some context object, and according to the position of the blob with respect to the camera
bool ReliabilityClassification::all_possible_base_inside(bool p[][3], int position) {
    switch(position) {
        case 0:
            return (!p[1][2] && !p[2][1] && !p[2][2])? true : false;
            break;
        case 1:
            return (!p[1][0] && !p[1][1] && !p[1][2] && !p[2][0] && !p[2][1] && !p[2][2])? true : false;
            break;
        case 2:
            return (!p[1][0] && !p[2][0] && !p[2][1])? true : false;
            break;
        case 3:
            return (!p[0][1] && !p[0][2] && !p[1][1] && !p[1][2] && !p[2][1] && !p[2][2])? true : false;
            break;
        case 4:
            return (!p[1][1])? true : false;
            break;
        case 5:
            return (!p[0][0] && !p[0][1] && !p[1][0] && !p[1][1] && !p[2][0] && !p[2][1])? true : false;
            break;
        case 6:
            return (!p[0][1] && !p[0][2] && !p[1][2])? true : false;
            break;
        case 7:
            return (!p[0][0] && !p[0][1] && !p[0][2] && !p[1][0] && !p[1][1] && !p[1][2])? true : false;
            break;
        case 8:
            return (!p[0][0] && !p[0][1] && !p[1][0])? true : false;
            break;
        default:
            AppendToLog("all_possible_base_inside: Error: This position doesn't exist.");
    }

    return false;
}

//This function returns the number of points inside a context object. Nine points are analyzed, for three coordinates of x2D and three of y2D.
int ReliabilityClassification::determine_points_inside_context_object(world::ContextObject *object, double *point_x2d, double *point_y2d, bool point_outside[][3]) {
    int i, j;
    int inside_count = 0; //Number of test points inside object
    bool in_roof, in_real;

    if(WOBJECT_IS_HOLLOW(object)) {
        for(i=0; i<3; i++)
            for(j=0; j<3; j++) {
                if(WOBJECT_NO_ROOF(object)) //Check if the point is over the roof
                    in_roof = false;
                else {
                    if( WOBJECT_ROOF2D_ON_IMAGE(object)->pointInConvexPolygon(point_x2d[i], point_y2d[j], false))
                        in_roof = true;
                    else
                        in_roof = false;
                }

                if(in_roof) //If point is in the roof, we already know what we want
                    in_real = false;
                else { //if not, we have to analyze if point is over a real wall
                    std::deque<world::Wall2D *>::iterator wall_iter;
                    in_real = false;
                    for(wall_iter=real_walls.begin(); wall_iter!=real_walls.end(); wall_iter++)
                        if(WALL2D_OUTLINE_ON_IMAGE(*wall_iter)->pointInConvexPolygon(point_x2d[i], point_y2d[j], false)) {
                            in_real = true;
                            break;
                        }
                }
	  
                point_outside[j][i] = (in_real || in_roof) ? false : true;
                if(!point_outside[j][i])
                    inside_count++;
            }
    } else { //For a solid object, is just necessary to check if the point is in the convex polygon of the object
        for(i=0; i<3; i++)
            for(j=0; j<3; j++) {
                if(WOBJECT_OUTLINE2D_ON_IMAGE(object)->pointInConvexPolygon(point_x2d[i], point_y2d[j], false)) {
                    point_outside[j][i] = false;
                    inside_count++;
                } else
                    point_outside[j][i] = true;
            }
    }

    return inside_count;

}

//Clear the lists of walls of analyzed blob points which limit the growth of the 2D bounding box in the case of context object occlusion.
void ReliabilityClassification::clearWallsInPoint() {
    
    std::map<world::ContextObject *, std::map<int, std::deque<inWall> > >::iterator iter;

    if(!enteredWallsForPointLeft.empty()) {
        for(iter=enteredWallsForPointLeft.begin(); iter != enteredWallsForPointLeft.end(); iter++) {
            (iter->second)[0].clear(); (iter->second)[1].clear(); (iter->second)[2].clear();
        }
        enteredWallsForPointLeft.clear();
    }

    if(!enteredWallsForPointRight.empty()) {
        for(iter=enteredWallsForPointRight.begin(); iter != enteredWallsForPointRight.end(); iter++) {
            (iter->second)[0].clear(); (iter->second)[1].clear(); (iter->second)[2].clear();
        }
        enteredWallsForPointRight.clear();
    }

    if(!enteredWallsForPointTop.empty()) {
        for(iter=enteredWallsForPointTop.begin(); iter != enteredWallsForPointTop.end(); iter++) {
            (iter->second)[0].clear(); (iter->second)[1].clear(); (iter->second)[2].clear();
        }
        enteredWallsForPointTop.clear();
    }

    if(!enteredWallsForPointBottom.empty()) {
        for(iter=enteredWallsForPointBottom.begin(); iter != enteredWallsForPointBottom.end(); iter++) {
            (iter->second)[0].clear(); (iter->second)[1].clear(); (iter->second)[2].clear();
        }
        enteredWallsForPointBottom.clear();
    }
}

//Function to check if intersection between a 2D image coordinates vertical or horizontal line and a 2D segment exists. It also stores the pertinent intersection point
//of the free coordinate (if line is vertical the free coordinate is y2d, else the free corrdinate is y2d).
bool ReliabilityClassification::getIntersectionWithSegment(double point2d, double lim1, double lim2, world::WallSegment *wsegment, bool vertical, double& intersection) {
    double res1, res2;
    int type;
    
    if( !( type = wsegment->inWallSegmentInterval(point2d, (vertical) ? 1 : 0, res1, res2) ) )
        return false; //If there is no intersection with segment
    if(type == -1) { //Case of multiple intersection, determine nearest segment point to blob point
        if ( (res1 >= lim2) || ((lim1 <= res1) && (lim2 <= res2)) )
            intersection = res1;
        else if ( (res2 <= lim1) || ((lim2 >= res2) && (lim1 >= res1)) )
            intersection = res2;
        else if( fabs(res1 - lim1) >= fabs(res2 - lim2))
            intersection = res2;
        else
            intersection = res1;
    } else // Case of normal intersection
        intersection = res1;

    return true;

  }

//Set the wall segments that can limit the vertical growth of the blob in case of context object occlusion, for each analyzed point of the blob.
void ReliabilityClassification::checkPointsInWallVer(world::ContextObject *object, double point_x2d[], double point_y2d[], DetectionProblemType dptype, world::Wall2D * wall2D) {
    int i, j;
    double min, max, cur;
    int min_type, max_type;
    int num_inter;
    world::WallSegment *wsegment;
    //Check four segments of a wall for each point
    for(j=0; j<3; j++) {
        num_inter = 0;
        for(i=0; i<4; i++) {
            wsegment = WALL2D_SEGMENT(wall2D,i);
            if(getIntersectionWithSegment(point_x2d[j], point_y2d[0], point_y2d[2], wsegment, true, cur)) {
                if(!num_inter) { //if it is the first detected intersection
                    min = cur;
                    min_type = WSEGMENT_IN_HEIGHT(wsegment);
                    num_inter++;;
                } else { //if it is the second detected intersection
                    if(cur <= min) {
                        max = min;
                        max_type = min_type;
                        min = cur;
                        min_type = WSEGMENT_IN_HEIGHT(wsegment);
                    } else {
                        max = cur;
                        max_type = WSEGMENT_IN_HEIGHT(wsegment);
                    }
                    num_inter++;;
                    break;
                }
            }
        }
      
        //Set the wall if there is intersection
        if(num_inter) {
            if(num_inter == 1) { //If just intersect in one point of the wall
                max = min;
                max_type = min_type;
            }

            std::deque<inWall> *inwalls = NULL;
            inWall inwall(wall2D, min, min_type, max, max_type);

            if(dptype & MM_OBJECT_BOTTOM)
                inwalls = &(enteredWallsForPointBottom[object][j]);
            else if(dptype & MM_OBJECT_TOP) { //MM_OBJECT_TOP.
                inwalls = &(enteredWallsForPointTop[object][j]);
                inwall.reverseOrder(); //the growth is in the opposite direction of the y2d image coordinate, then entering value in wall corresponds to max intersection value
            } else { //MM_OBJECT_INSIDE
                if((point_y2d[2] > max) && (point_y2d[0] >= min)) { //Test for TOP occlusion
                    inwalls = &(enteredWallsForPointTop[object][j]);
                    inwall.reverseOrder(); //the growth is in the opposite direction of the y2d image coordinate, then entering value in wall corresponds to max intersection value
                } else
                    inwalls = &(enteredWallsForPointBottom[object][j]);
            }

            if((*inwalls).empty())
                (*inwalls).push_front(inwall);
            else {
                std::deque<inWall>::iterator iter;
                for(iter = (*inwalls).begin(); iter != (*inwalls).end(); iter++) {
                    if((point_y2d[2] > max) && (point_y2d[0] >= min)) { //Test for TOP occlusion, walls ordered by max entering values
                        if(max > (*iter).getEntering()) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    } else { //If BOTTOM occlusion, walls ordered by min entering values
                        if(min < (*iter).getEntering()) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    }
                }
                if(iter == (*inwalls).end())
                    (*inwalls).push_back(inwall);
            }
        }
    }
}

  //Set the wall segments that can limit the horizontal growth of the blob in case of context object occlusion, for each analyzed point of the blob.
void ReliabilityClassification::checkPointsInWallHor(world::ContextObject *object, double point_x2d[], double point_y2d[], DetectionProblemType dptype, world::Wall2D * wall2D) {
    int i, j;
    double min, max, cur;
    int min_type, max_type;
    int num_inter;
    world::WallSegment *wsegment;
    //Check four segments of a wall for each point
    for(j=0; j<3; j++) {
        num_inter = 0;
        for(i=0; i<4; i++) {
            wsegment = WALL2D_SEGMENT(wall2D,i);
            if(getIntersectionWithSegment(point_y2d[j], point_x2d[0], point_x2d[2], wsegment, false, cur)) {
                if(!num_inter) { //if it is the first detected intersection
                    min = cur;
                    min_type = WSEGMENT_IN_HEIGHT(wsegment);
                    num_inter++;;
                } else { //if it is the second detected intersection
                    if(cur <= min) {
                        max = min;
                        max_type = min_type;
                        min = cur;
                        min_type = WSEGMENT_IN_HEIGHT(wsegment);
                    } else {
                        max = cur;
                        max_type = WSEGMENT_IN_HEIGHT(wsegment);
                    }
                    num_inter++;;
                    break;
                }
            }
        }

        //Set the wall if there is intersection
        if(num_inter) {
            if(num_inter == 1) { //If just intersect in one point of the wall
              max = min;
              max_type = min_type;
            }

            std::deque<inWall> *inwalls = NULL;
            inWall inwall(wall2D, min, min_type, max, max_type);

            if(dptype & MM_OBJECT_RIGHT)
                inwalls = &(enteredWallsForPointRight[object][j]);
            else if(dptype & MM_OBJECT_LEFT) {
                inwalls = &(enteredWallsForPointLeft[object][j]);
                inwall.reverseOrder(); //the growth is in the opposite direction of the x2d image coordinate, then entering value in wall corresponds to max intersection value
            } else { //MM_OBJECT_INSIDE
                if((point_x2d[2] > max) && (point_x2d[0] >= min)) { //Test for LEFT occlusion,
                    inwalls = (&enteredWallsForPointLeft[object][j]);
                    inwall.reverseOrder(); //the growth is in the opposite direction of the x2d image coordinate, then entering value in wall corresponds to max intersection value
                } else
                    inwalls = &(enteredWallsForPointRight[object][j]);
            }

            if((*inwalls).empty())
                (*inwalls).push_front(inwall);
            else {
                std::deque<inWall>::iterator iter;
                for(iter = (*inwalls).begin(); iter != (*inwalls).end(); iter++) {
                    if((point_x2d[2] > max) && (point_x2d[0] >= min)) { //Test for LEFT occlusion, walls ordered by max entering values
                        if(max > (*iter).getEntering()) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    } else { //If RIGHT occlusion, walls ordered by min entering values
                        if(min < (*iter).getEntering()) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    }
                }
                if(iter == (*inwalls).end())
                    (*inwalls).push_back(inwall);
            }
        }
    }
}

//Set the segments intersecting the analyzed points in the different directions of possible growth of the blob in case of context object occlusion.
void ReliabilityClassification::checkPointsInRoof(world::ContextObject *object, double point_x2d[], double point_y2d[], QSharedPointer< polygon2D<double> > roof, DetectionProblemType dptype) {
 
    int i;
    double min, max, aux;
    std::deque<inWall> *inwalls = NULL;	
    DetectionProblemType cur_dptype = MM_DP_NONE;

    for(i=0; i<3; i++) {
        //Horizontal
        if(roof->getIntersectionsWithConvexPolygon(point_y2d[i], false, min, max)) {
            if(min > max) {
                aux = min;
                min = max;
                max = aux;
            }

            inWall inwall(NULL, min, 1, max, 1);

            inwalls = NULL;

            if(dptype & MM_OBJECT_RIGHT) {
                inwalls = &(enteredWallsForPointRight[object][i]);
                cur_dptype = MM_OBJECT_RIGHT;
            } else if(dptype & MM_OBJECT_LEFT) {
                inwalls = &(enteredWallsForPointLeft[object][i]);
                inwall.reverseOrder();
                cur_dptype = MM_OBJECT_LEFT;
            } else { //MM_OBJECT_INSIDE
                if((point_x2d[0] < min) && (point_x2d[2] <= max)) { 	   //Test for RIGHT occlusion
                    inwalls = &(enteredWallsForPointRight[object][i]);
                    cur_dptype = MM_OBJECT_RIGHT;
                } else if((point_x2d[2] > max) && (point_x2d[0] >= min)) { //Test for LEFT occlusion
                    inwalls = &(enteredWallsForPointLeft[object][i]);
                    inwall.reverseOrder();
                    cur_dptype = MM_OBJECT_LEFT;
                }
            }

            if(inwalls) {
                if((*inwalls).empty())
                    (*inwalls).push_front(inwall);
                else {
                    std::deque<inWall>::iterator iter;
                    for(iter = (*inwalls).begin(); iter != (*inwalls).end(); iter++) {
                        if( (cur_dptype == MM_OBJECT_RIGHT) ? (min < (*iter).getEntering()) : (max > (*iter).getEntering())) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    }
                    if(iter == (*inwalls).end())
                        (*inwalls).push_back(inwall);
                }
            }
        }

        //Vertical
        if(roof->getIntersectionsWithConvexPolygon(point_x2d[i], true, min, max)) {
            if(min > max) {
                aux = min;
                min = max;
                max = aux;
            }

            inWall inwall(NULL, min, 1, max, 1);

            inwalls = NULL;
            if(dptype & MM_OBJECT_BOTTOM) {
                inwalls = &(enteredWallsForPointBottom[object][i]);
                cur_dptype = MM_OBJECT_BOTTOM;
            } else if(dptype & MM_OBJECT_TOP) { //MM_OBJECT_TOP
                inwalls = &(enteredWallsForPointTop[object][i]);
                inwall.reverseOrder();
                cur_dptype = MM_OBJECT_TOP;
            } else { //MM_OBJECT_INSIDE
                if((point_y2d[0] < min) && (point_y2d[2] <= max)) { 	   //Test for BOTTOM occlusion
                    inwalls = &(enteredWallsForPointBottom[object][i]);
                    cur_dptype = MM_OBJECT_BOTTOM;
                } else if((point_y2d[2] > max) && (point_y2d[0] >= min)) { //Test for TOP occlusion
                    inwalls = &(enteredWallsForPointTop[object][i]);
                    inwall.reverseOrder();
                    cur_dptype = MM_OBJECT_TOP;
                }
            }

            if(inwalls) {
                if((*inwalls).empty())
                    (*inwalls).push_front(inwall);
                else {
                    std::deque<inWall>::iterator iter;
                    for(iter = (*inwalls).begin(); iter != (*inwalls).end(); iter++) {
                        if( (cur_dptype == MM_OBJECT_BOTTOM) ? (min < (*iter).getEntering()) : (max > (*iter).getEntering())) {
                            (*inwalls).insert(iter, inwall);
                            break;
                        }
                    }
                    if(iter == (*inwalls).end())
                        (*inwalls).push_back(inwall);
                }
            }
        }
    }
}

//Use the information obtained from the limiting walls for analyzed points, to determine if the blob has a real possible of being occluded.
DetectionProblemType ReliabilityClassification::correctBlobDetectionProblemType(world::ContextObject *object, double point_x2d[], double point_y2d[], DetectionProblemType cur_dptype) {
    int i;
    DetectionProblemType new_dptype = MM_DP_NONE, new_dptype_h = MM_DP_NONE, new_dptype_v = MM_DP_NONE;
    DetectionProblemType new_dptype_hor[3];
    DetectionProblemType new_dptype_ver[3];
    double min, max;

    int first_in_floor_counter;
    //Horizontal Occlusion Correction
    if(cur_dptype & (MM_OBJECT_INSIDE | MM_OBJECT_RIGHT)) {
        first_in_floor_counter = 0;
        for(i=0; i<3; i++) {
            new_dptype_hor[i] = MM_DP_NONE;
            //Horizontal correction
            if(!enteredWallsForPointRight[object][i].empty()) {
                min = enteredWallsForPointRight[object][i].front().getEntering();
                max = enteredWallsForPointRight[object][i].back().getLeaving();

                //Test for RIGHT occlusion
                if(    (point_x2d[0] < min && point_x2d[2] >= min && point_x2d[2] <= max)
                    || (fabs(point_x2d[2] - min) < RC_PROXIMITY_PIXELS))
                    new_dptype_hor[i] = (DetectionProblemType)(new_dptype_hor[i] | MM_OBJECT_RIGHT);

                if(!enteredWallsForPointRight[object][i].front().getEnteringType())
                    first_in_floor_counter++;
            }
        }
        if(first_in_floor_counter < 3)
            new_dptype_h = (DetectionProblemType)(new_dptype_hor[0] | new_dptype_hor[1] | new_dptype_hor[2]);
  
    }

    if(cur_dptype & (MM_OBJECT_INSIDE | MM_OBJECT_LEFT) ) { //If LEFT occlusion
        first_in_floor_counter = 0;
        for(i=0; i<3; i++) {
            new_dptype_hor[i] = MM_DP_NONE;
            //Horizontal correction
            if(!enteredWallsForPointLeft[object][i].empty()) {
                max = enteredWallsForPointLeft[object][i].front().getEntering();
                min = enteredWallsForPointLeft[object][i].back().getLeaving();

                //Test for LEFT occlusion
                if(    (point_x2d[2] > max && point_x2d[0] <= max && point_x2d[0] >= min)
                    || (fabs(point_x2d[0] - max) < RC_PROXIMITY_PIXELS))
                    new_dptype_hor[i] = (DetectionProblemType)(new_dptype_hor[i] | MM_OBJECT_LEFT);

                if(!enteredWallsForPointLeft[object][i].front().getEnteringType())
                    first_in_floor_counter++;
            }
        }
        if(first_in_floor_counter < 3)
            new_dptype_h = (DetectionProblemType)(new_dptype_h | new_dptype_hor[0] | new_dptype_hor[1] | new_dptype_hor[2]);
    }

    //Vertical Occlusion Correction
    if(cur_dptype & (MM_OBJECT_INSIDE | MM_OBJECT_BOTTOM)) {
        first_in_floor_counter = 0;
        for(i=0; i<3; i++) {
            new_dptype_ver[i] = MM_DP_NONE;
            //Vertical correction
            if(!enteredWallsForPointBottom[object][i].empty()) {
                min = enteredWallsForPointBottom[object][i].front().getEntering();
                max = enteredWallsForPointBottom[object][i].back().getLeaving();

                //Test for BOTTOM occlusion
                if(    ( point_y2d[0] < min && point_y2d[2] >= min && point_y2d[2] <= max)
                    || (fabs(point_y2d[2] - min) < RC_PROXIMITY_PIXELS))
                    new_dptype_ver[i] = (DetectionProblemType)(new_dptype_ver[i] | MM_OBJECT_BOTTOM);

                if(!enteredWallsForPointBottom[object][i].front().getEnteringType())
                    first_in_floor_counter++;
            }
        }

        if(first_in_floor_counter < 3)
            new_dptype_v = (DetectionProblemType)(new_dptype_ver[0] | new_dptype_ver[1] | new_dptype_ver[2]);
    }

    if(cur_dptype & (MM_OBJECT_INSIDE | MM_OBJECT_TOP) ) { //If TOP occlusion
        first_in_floor_counter = 0;
        for(i=0; i<3; i++) {
            new_dptype_ver[i] = MM_DP_NONE;
            //Verizontal correction
            if(!enteredWallsForPointTop[object][i].empty()) {
                max = enteredWallsForPointTop[object][i].front().getEntering();
                min = enteredWallsForPointTop[object][i].back().getLeaving();

                //Test for Top occlusion
                if(    (point_y2d[2] > max && point_y2d[0] <= max && point_y2d[0] >= min)
                    || (fabs(point_y2d[0] - max) < RC_PROXIMITY_PIXELS))
                    new_dptype_ver[i] = (DetectionProblemType)(new_dptype_ver[i] | MM_OBJECT_TOP);
                if(!enteredWallsForPointTop[object][i].front().getEnteringType())
                    first_in_floor_counter++;
            }
        }
        if(first_in_floor_counter < 3)
            new_dptype_v = (DetectionProblemType)(new_dptype_v | new_dptype_ver[0] | new_dptype_ver[1] | new_dptype_ver[2]);
    }

    //Clear new not oculted blob orientations
    if(!(new_dptype_h & MM_OBJECT_RIGHT) && !enteredWallsForPointRight.empty()) {
        enteredWallsForPointRight[object][0].clear(); enteredWallsForPointRight[object][1].clear(); enteredWallsForPointRight[object][2].clear();
        enteredWallsForPointRight.erase(object);
    }

    if(!(new_dptype_h & MM_OBJECT_LEFT) && !enteredWallsForPointLeft.empty()) {
        enteredWallsForPointLeft[object][0].clear(); enteredWallsForPointLeft[object][1].clear(); enteredWallsForPointLeft[object][2].clear();
        enteredWallsForPointLeft.erase(object);
    }

    if(!(new_dptype_v & MM_OBJECT_BOTTOM) && !enteredWallsForPointBottom.empty()) {
        enteredWallsForPointBottom[object][0].clear(); enteredWallsForPointBottom[object][1].clear(); enteredWallsForPointBottom[object][2].clear();
        enteredWallsForPointBottom.erase(object);
    }

    if(!(new_dptype_v & MM_OBJECT_TOP) && !enteredWallsForPointTop.empty()) {
        enteredWallsForPointTop[object][0].clear(); enteredWallsForPointTop[object][1].clear(); enteredWallsForPointTop[object][2].clear();
        enteredWallsForPointTop.erase(object);
    }

    new_dptype = (DetectionProblemType) (new_dptype_h | new_dptype_v);

    return new_dptype;
}

//Perform the analysis of nine points in the blob to determine the correct blob occlusion type and the wall segments which will limit the possible growth
//of blob dimensions in the case of context object occlusion
DetectionProblemType ReliabilityClassification::improveObjectAnalysis(world::ContextObject *object, DetectionProblemType dptype, Blob *blob, interval_t interestX, interval_t interestY) {
    int i;
    int inside_count; //Number of test points inside object
    double point_x2d[3], point_y2d[3];     
    bool point_outside[3][3]; //Analyzing blob points:
                             // (0,0)(top-left)     (0,1)(top-center)     (0,2)(top-right)
                             // (1,0)(middle-left)  (1,1)(middle-center)  (1,2)(middle-right)
                             // (2,0)(bottom-left)  (2,1)(bottom-center)  (2,2)(bottom-right)
                             // Value false: not outside
                             // Value true:  outside object zone

    real_walls.clear();

    //Get virtual walls list if object is hollow
    std::vector< QSharedPointer<world::Wall> > *walls = WOBJECT_WALLS_LIST(object);
    std::vector< QSharedPointer<world::Wall2D> > *walls2d = WOBJECT_WALLS_2D_LIST(object);
    std::vector< QSharedPointer<world::Wall> >::iterator w_it, w_it_end = walls->end();
    std::vector< QSharedPointer<world::Wall2D> >::iterator w2d_it = walls2d->begin();
    if(walls != NULL)
        for(w_it = walls->begin(); w_it != w_it_end; w_it++, w2d_it++) {
            if(WALL_IS_SOLID(&**w_it))
                real_walls.push_back(&**w2d_it);
    }

    //Fill points to analyze for all blob:
    point_x2d[0] = BLOB_XLEFT(blob);
    point_x2d[1] = BLOB_XCENTER(blob);
    point_x2d[2] = BLOB_XRIGHT(blob);
    point_y2d[0] = BLOB_YTOP(blob);
    point_y2d[1] = BLOB_YCENTER(blob);
    point_y2d[2] = BLOB_YBOTTOM(blob);

    //Get points information
    inside_count = determine_points_inside_context_object(object, (double *)point_x2d, (double *)point_y2d, point_outside);

#ifdef RC_DEBUG    
    int j;
    AppendToLog("\nBLOB: %d\n",BLOB_IDENT(blob));
    for(i=0; i<3; i++) {
      for(j=0; j<3; j++)
	if(point_outside[i][j] == true)
          AppendToLog("1 ");
	else
          AppendToLog("0 ");
      AppendToLog(std::endl);
    }
#endif

    //If almost all points are inside the object or all the potencial 3D base points of blob are inside the object, the blob occlusion is apriori classified as inside an object
    if(inside_count >= 8 || all_possible_base_inside(point_outside, BLOB_POSITION(blob))) { // If object is really in object zone
        return MM_OBJECT_INSIDE;
    }

    //Now get the right occlusion limit. Values are ensured to be inside the blob (+-0.9) to avoid missed intersection because of numeric precision problems
    if(dptype != MM_OBJECT_INSIDE) {
        point_x2d[0] = INTERVAL_X1(interestX) + 0.9;
        point_x2d[1] = (INTERVAL_X1(interestX) + INTERVAL_X2(interestX))/2.0;
        point_x2d[2] = INTERVAL_X2(interestX) - 0.9;
        point_y2d[0] = INTERVAL_X1(interestY) + 0.9;
        point_y2d[1] = (INTERVAL_X1(interestY) + INTERVAL_X2(interestY))/2.0;
        point_y2d[2] = INTERVAL_X2(interestY) - 0.9;
    } else {
        point_x2d[0] += 0.9;
        point_x2d[2] -= 0.9;
        point_y2d[0] += 0.9;
        point_y2d[2] -= 0.9;
    }
    //Get points information
    determine_points_inside_context_object(object, (double *)point_x2d, (double *)point_y2d, point_outside);
    
    std::deque<world::Wall2D *>::iterator wall_iter;

#ifdef RC_DEBUG    
    int j;
    AppendToLog("\nBLOB: %d\n",BLOB_IDENT(blob));
    for(i=0; i<3; i++) {
      for(j=0; j<3; j++)
	if(point_outside[i][j] == true)
          AppendToLog("1 ");
	else
          AppendToLog("0 ");
      AppendToLog(std::endl);
    }
#endif

    DetectionProblemType dptype_spec;

    //Analyze walls to check the limits that they will impose to the different analyzed blob points, for the four different possible blob growth directions
    for(wall_iter=real_walls.begin(); wall_iter!=real_walls.end(); wall_iter++) {
        if( (dptype_spec = (DetectionProblemType) (dptype & (MM_OBJECT_BOTTOM | MM_OBJECT_TOP | MM_OBJECT_INSIDE))) )
            checkPointsInWallVer(object, point_x2d, point_y2d, dptype_spec, *wall_iter);

        if( (dptype_spec = (DetectionProblemType) (dptype & (MM_OBJECT_LEFT | MM_OBJECT_RIGHT | MM_OBJECT_INSIDE))) )
            checkPointsInWallHor(object, point_x2d, point_y2d, dptype_spec, *wall_iter);
    }
    
    //Complete analysis with roof 
    if(WOBJECT_IS_HOLLOW(object) && !WOBJECT_NO_ROOF(object))
        checkPointsInRoof(object, point_x2d, point_y2d, WOBJECT_ROOF2D_ON_IMAGE(object), dptype);
    
    //In case of blob inside object, correct detection problem type according to obtained information
    dptype = correctBlobDetectionProblemType(object, point_x2d, point_y2d, dptype);
    
    if(dptype != MM_DP_NONE) {
        object_occlusion_type[object] = dptype;
        for(i = 0; i < 3; i++) {
            analyzedPointsForObjectX[object][i] = point_x2d[i];
            analyzedPointsForObjectY[object][i] = point_y2d[i];
        }
    }
    std::deque<inWall>::iterator iter;

#ifdef RC_DEBUG
    if(enteredWallsForPointRight.count(object)) {
        AppendToLog("\tRight Occlusion for object: %s\n", WOBJECT_NAME(object));
        for(i=0;i<3;i++) {
            AppendToLog("\t\tFor point y: " + point_y2d[i]);
            for(iter = enteredWallsForPointRight[object][i].begin(); iter != enteredWallsForPointRight[object][i].end(); iter++) {
                AppendToLog("\t\tWall " + ((*iter).getWall()!=NULL)?WALL2D_NAME((*iter).getWall()):"ROOF"
                          + ": (" + (*iter).getEntering()
                          + "(" + (*iter).getEnteringType()
                          + ") <-> " + (*iter).getLeaving()
                          + "(" + (*iter).getLeavingType()
                          + "))");
            }
            AppendToLog(std::endl;
        }
    }

    if(enteredWallsForPointLeft.count(object)) {
        AppendToLog("\tLeft Occlusion for object: " + WOBJECT_NAME(object)));
        for(i = 0; i < 3; i++) {
            AppendToLog("\t\tFor point y: %f\n", point_y2d[i]);
            for(iter = enteredWallsForPointLeft[object][i].begin(); iter != enteredWallsForPointLeft[object][i].end(); iter++) {
                AppendToLog("\t\tWall " + ((*iter).getWall()!=NULL)?WALL2D_NAME((*iter).getWall()):"ROOF"
                          + ": (" + (*iter).getEntering()
                          + "(" + (*iter).getEnteringType()
                          + ") <-> " + (*iter).getLeaving()
                          + "(" + (*iter).getLeavingType()
                          + "))");
            }
            AppendToLog(std::endl;
        }
    }

    if(enteredWallsForPointBottom.count(object)) {
        AppendToLog("\tBottom Occlusion for object: " + WOBJECT_NAME(object));
        for(i = 0; i < 3; i++) {
            AppendToLog("\t\tFor point x: " + point_x2d[i]);
            for(iter = enteredWallsForPointBottom[object][i].begin(); iter != enteredWallsForPointBottom[object][i].end(); iter++) {
                AppendToLog("\t\tWall " + ((*iter).getWall()!=NULL)?WALL2D_NAME((*iter).getWall()):"ROOF"
                          + ": (" + (*iter).getEntering()
                          + "(" + (*iter).getEnteringType()
                          + ") <-> " + (*iter).getLeaving()
                          + "(" + (*iter).getLeavingType()
                          + "))");
            }
            AppendToLog(std::endl;
        }
    }

    if(enteredWallsForPointTop.count(object)) {
        AppendToLog("\tTop Occlusion for object: " + WOBJECT_NAME(object));
        for(i=0;i<3;i++) {
            AppendToLog("\t\tFor point x: " + point_x2d[i]);
            for(iter = enteredWallsForPointTop[object][i].begin(); iter != enteredWallsForPointTop[object][i].end(); iter++) {
                AppendToLog("\t\tWall " + ((*iter).getWall()!=NULL)?WALL2D_NAME((*iter).getWall()):"ROOF"
                          + ": (" + (*iter).getEntering()
                          + "(" + (*iter).getEnteringType()
                          + ") <-> " + (*iter).getLeaving()
                          + "(" + (*iter).getLeavingType()
                          + "))");

            }
            AppendToLog(std::endl;
        }
    }
#endif
    return dptype;
}

//Checks if an object is close enough of a context object. If it is the case, it searches to determine the correct context object occlusion type
//It also sets the annoying walls of the object, for coherency between 3D parallelpiped base and these walls
DetectionProblemType ReliabilityClassification::isBlobOccludedByStaticObject(Blob *blob) {
  
    DetectionProblemType dptype, general_dptype = MM_DP_NONE;
    std::vector< QSharedPointer<world::ContextObject> > *objects = &m_context->contextObjects;
    Interval blobX, blobY;

    Interval::newInterval(&blobX, BLOB_XLEFT(blob), BLOB_XRIGHT(blob));
    Interval::newInterval(&blobY, BLOB_YTOP(blob), BLOB_YBOTTOM(blob));

    clearWallsInPoint();

    if(objects != NULL) {
        std::vector< QSharedPointer<world::ContextObject> >::iterator o_it, o_it_end = objects->end();
        Interval polX, polY;
        QSharedPointer< polygon2D<double> > pol;
        double bb_l, bb_r, bb_b, bb_t;
        world::ContextObject *cobject;

        for(o_it = objects->begin(); o_it != o_it_end; o_it++) {

            //TO-DO!!! Verify at pixel level if a blob is occluded by an object or over it.
            //In the mean time certain objects will be suppressed from analysis,

            cobject = &**o_it;
            dptype = MM_DP_NONE;
            pol = WOBJECT_OUTLINE2D_ON_IMAGE(cobject);

            bb_l = POLYGON_2D_BB_XLEFT(pol);
            bb_r = POLYGON_2D_BB_XRIGHT(pol);
            bb_b = POLYGON_2D_BB_YBOTTOM(pol);
            bb_t = POLYGON_2D_BB_YTOP(pol);
      /*
            AppendToLog("\nFor object: " + cobject->name);
            AppendToLog("\t(L,R,T,B)<->(W,H): ("
                      + RECT_XLEFT(&(pol->boundingBox)) + ", "
                      + RECT_XRIGHT(&(pol->boundingBox)) + ", "
                      + RECT_YTOP(&(pol->boundingBox)) + ", "
                      + RECT_YBOTTOM(&(pol->boundingBox)) + ") <-> ("
                      + RECT_WIDTH(&(pol->boundingBox)) + ", "
                      + RECT_HEIGHT(&(pol->boundingBox)) + ") ");
            AppendToLog("\tOutline 2D:");
            for(i=0; i<pol->points.size(); i++)
                AppendToLog("\t\tPoint " + i + ": ("
                          + pol->points[i].x + ", "
                          + pol->points[i].y + ") ");
            AppendToLog(std::endl;
      */

            Interval::newInterval(&polX, bb_l, bb_r);
            Interval::newInterval(&polY, bb_t, bb_b);

            if(thereIsIntersection(&polX, &polY, &blobX, &blobY)) { //If exist some intersection between the blob bounding box and the context object bounding box
                if(    !INTERVAL_IS_NULL(&polX) && !INTERVAL_IS_NULL(&polY)
                    && BLOB_XLEFT(blob)  >= bb_l
                    && BLOB_XRIGHT(blob) <= bb_r
                    && BLOB_YTOP(blob)   >= bb_t
                    && BLOB_YBOTTOM(blob) <= bb_b )//If blob totally inside object bb
                    dptype = MM_OBJECT_INSIDE;       //A priori we will say that blob is inside the context object
                else {
                    if(!INTERVAL_IS_NULL(&polX)) { //Possible Vertical Occlusion
                        if(BLOB_YBOTTOM(blob) < bb_b && bb_t - BLOB_YBOTTOM(blob) <= RC_PROXIMITY_PIXELS)
                            dptype = MM_OBJECT_BOTTOM;
                        else if(BLOB_YTOP(blob) > bb_t && BLOB_YTOP(blob) - bb_b <= RC_PROXIMITY_PIXELS)
                            dptype = MM_OBJECT_TOP;
                    }

                    if(!INTERVAL_IS_NULL(&polY)) { //Possible horizontal occlusion
                        if(BLOB_XRIGHT(blob) < bb_r && bb_l - BLOB_XRIGHT(blob) <= RC_PROXIMITY_PIXELS)
                            dptype = (DetectionProblemType)((int)dptype | MM_OBJECT_RIGHT);
                        else if(BLOB_XLEFT(blob) > bb_l && BLOB_XLEFT(blob) - bb_r <= RC_PROXIMITY_PIXELS)
                            dptype = (DetectionProblemType)((int)dptype | MM_OBJECT_LEFT);
                    }
                }

                if(dptype) { //If some kind of object occlusion has been detected
                    near_objects.push_front(cobject);
                    //Set wall segments to be analyzed (on the floor) when checking coherence of the base of 3D parallelpiped, for not extended blob
                    if(m_treatWallCoherence)
                        setAnnoyingWallsForContextObject(blob, cobject, true);

                    //Analyze blob points to verify the occlusion hypothesis or to get the correct the occlusion type.
                    dptype = improveObjectAnalysis(cobject, dptype, blob, &polX, &polY);
                } else
                    not_near_objects.push_front(cobject);
            }

            //Save the occlusion type according to all context objects
            general_dptype = (DetectionProblemType)((int)general_dptype | (int)dptype);
        }
    }
    return general_dptype;
}

//Checks if an object is close enough of a context object and store it in lists of near or not near objects
void ReliabilityClassification::setNearObjects(Blob *blob) {
    DetectionProblemType dptype;
    std::vector< QSharedPointer<world::ContextObject> > *objects = &m_context->contextObjects;
    Interval blobX, blobY;

    Interval::newInterval(&blobX, BLOB_XLEFT(blob), BLOB_XRIGHT(blob));
    Interval::newInterval(&blobY, BLOB_YTOP(blob), BLOB_YBOTTOM(blob));

    if(objects != NULL) {
        std::vector< QSharedPointer<world::ContextObject> >::iterator o_it, o_it_end = objects->end();
        Interval polX, polY;
        QSharedPointer< polygon2D<double> > pol;
        double bb_l, bb_r, bb_b, bb_t;
        world::ContextObject *cobject;

        for(o_it = objects->begin(); o_it != o_it_end; o_it++) {
            cobject = &**o_it;
            dptype = MM_DP_NONE;
            pol = WOBJECT_OUTLINE2D_ON_IMAGE(cobject);

            bb_l = POLYGON_2D_BB_XLEFT(pol);
            bb_r = POLYGON_2D_BB_XRIGHT(pol);
            bb_b = POLYGON_2D_BB_YBOTTOM(pol);
            bb_t = POLYGON_2D_BB_YTOP(pol);

            Interval::newInterval(&polX, bb_l, bb_r);
            Interval::newInterval(&polY, bb_t, bb_b);

            if(thereIsIntersection(&polX, &polY, &blobX, &blobY)) {

                if( !INTERVAL_IS_NULL(&polX) && !INTERVAL_IS_NULL(&polY)
                    && BLOB_XLEFT(blob)  >= bb_l
                    && BLOB_XRIGHT(blob) <= bb_r
                    && BLOB_YTOP(blob)   >= bb_t
                    && BLOB_YBOTTOM(blob) <= bb_b )//Object inside object bb
                    dptype = MM_OBJECT_INSIDE;
                else {
                    if(!INTERVAL_IS_NULL(&polX)) { //Possible Vertical Occlusion
                        if(BLOB_YBOTTOM(blob) < bb_b && bb_t - BLOB_YBOTTOM(blob) <= RC_PROXIMITY_PIXELS)
                            dptype = MM_OBJECT_BOTTOM;
                        else if(BLOB_YTOP(blob) > bb_t && BLOB_YTOP(blob) - bb_b <= RC_PROXIMITY_PIXELS)
                            dptype = MM_OBJECT_TOP;
                    }

                    if(!INTERVAL_IS_NULL(&polY)) { //Possible horizontal occlusion
                        if(BLOB_XRIGHT(blob) < bb_r && bb_l - BLOB_XRIGHT(blob) <= RC_PROXIMITY_PIXELS)
                            dptype = (DetectionProblemType)((int)dptype | MM_OBJECT_RIGHT);
                        else if(BLOB_XLEFT(blob) > bb_l && BLOB_XLEFT(blob) - bb_r <= RC_PROXIMITY_PIXELS)
                            dptype = (DetectionProblemType)((int)dptype | MM_OBJECT_LEFT);
                    }
                }

                if(dptype) {
                    near_objects.push_front(cobject);
                    //Set wall segments to be analyzed (on the floor) when checking coherence of the base of 3D parallelpiped, for not extended blob
                    setAnnoyingWallsForContextObject(blob, cobject, true);
                } else
                    not_near_objects.push_front(cobject);
            }
        }
    }
}

void ReliabilityClassification::initBlob3DData() {
    OcclusionMinLeft = INT_MIN;
    OcclusionMaxRight = INT_MAX;
    OcclusionMaxBottom = INT_MAX;
    OcclusionMinTop = INT_MIN;
    reset_top();
}

//Set the pertinent maximal vertical blob limits (LEFT and RIGHT), according to limiting context walls
void ReliabilityClassification::check_walls_ver(Blob *blob, DetectionProblemType dptype) {

    world::Wall2D * wall_list;
    double bottom_lim, top_lim, center_lim;
    double bottom_y = BLOB_YBOTTOM(blob), top_y = BLOB_YTOP(blob), center_y  = BLOB_YCENTER(blob);
    double a, b;
    double x;
    int is_function;
    bool exist_pertinent_left  = false;
    bool exist_pertinent_right = false;
    Interval int1, int2, result;
    point2D<double> p1, p2;
    std::multimap<DetectionProblemType, world::Wall2D *>::iterator pos, last;

    //Set pertinence interval for blob
    Interval::newInterval(&int1, top_y, bottom_y);

    //Three analyzed points
    bottom_lim = top_lim = center_lim = RC_INFINITY;

    last = m_pertinent_walls.upper_bound(dptype);

    //Just check walls pertinent according to possible border occlusion type
    for (pos = m_pertinent_walls.lower_bound(dptype); pos != last; pos++) {

        wall_list = pos->second;

        //Set 2D wall pertinent interval
        wall_list->get2DwallLimitPoints(model_hmax, SM_CALIB_MATRIX(m_context), &p1, &p2);
        Interval::newInterval(&int2, POINT_2D_Y(&p1), POINT_2D_Y(&p2));

        if(Interval::intersect(&result, &int1, &int2)) { //If there is some influence of wall in blob occlusion

            is_function = wall_list->get2DWallLine(model_hmax, &a, &b);
            if(!is_function) { //If associated wall 2D function is a line parallel to y image axis

                x = POINT_2D_X(&p1);
                //Taking de value of minimal distance
                if(MM_CAM_OCCL_LEFT & dptype) {
                    exist_pertinent_left = true;
                    bottom_lim = (bottom_lim < x) ? x : bottom_lim;
                    center_lim = (center_lim < x) ? x : center_lim;
                       top_lim = (top_lim < x)    ? x : top_lim;
                }

                if(MM_CAM_OCCL_RIGHT & dptype) { //RIGHT
                    bottom_lim = (bottom_lim > x) ? x : bottom_lim;
                    center_lim = (center_lim > x) ? x : center_lim;
                    top_lim    = (top_lim > x)    ? x : top_lim;
                    exist_pertinent_right = true;
                }
            } else if (a != 0) {
                //Taking the value of minimal distance
                if(MM_CAM_OCCL_LEFT & dptype) {
                    exist_pertinent_left = true;
                    x = (bottom_y - b)/a;
                    bottom_lim = (bottom_lim < x) ? x : bottom_lim;
                    x = (center_y - b)/a;
                    center_lim = (center_lim < x) ? x : center_lim;
                    x = (top_y - b)/a;
                    top_lim    = (top_lim < x)    ? x : top_lim;
                }

                if(MM_CAM_OCCL_RIGHT & dptype) { //RIGHT
                    exist_pertinent_right = true;
                    x = (bottom_y - b)/a;
                    bottom_lim = (bottom_lim > x) ? x : bottom_lim;
                    x = (center_y - b)/a;
                    center_lim = (center_lim > x) ? x : center_lim;
                    x = (top_y - b)/a;
                    top_lim    = (top_lim > x)    ? x : top_lim;
                }
            }
        }
    }

    if(exist_pertinent_left) {

        //Taking the value of maximal distance
        if(MM_CAM_OCCL_LEFT & dptype)
            borderMinLeft = (bottom_lim < center_lim) ?
              ( (bottom_lim < top_lim) ? bottom_lim : top_lim ) : ( (center_lim < top_lim) ? center_lim : top_lim );
    } else
        borderMinLeft = - RC_INFINITY;

    if(exist_pertinent_right) {
      //Taking the value of maximal distance
        if (MM_CAM_OCCL_RIGHT & dptype)//RIGHT
            borderMaxRight = (bottom_lim > center_lim) ?
              ( (bottom_lim > top_lim) ? bottom_lim : top_lim ) : ( (center_lim > top_lim) ? center_lim : top_lim );
    } else
        borderMaxRight= RC_INFINITY;
}

//Set the pertinent maximal horizontal blob limits (TOP and BOTTOM), according to limiting context walls
void ReliabilityClassification::check_walls_hor(Blob *blob, DetectionProblemType dptype) {

    world::Wall2D * wall_list;
    double limit = RC_INFINITY; 
    double left_lim, right_lim, center_lim;
    double left_x = BLOB_XLEFT(blob), right_x = BLOB_XRIGHT(blob), center_x  = BLOB_XCENTER(blob);
    double a, b, y;
    bool is_function, exist_pertinent_bottom = false, exist_pertinent_top = false;
    Interval int1, int2, result;
    point2D<double> p1, p2;
    std::multimap<DetectionProblemType, world::Wall2D *>::iterator pos, last;

    //Set pertinence interval for blob
    Interval::newInterval(&int1, left_x, right_x);

    //Three analyzed points
    left_lim = right_lim = center_lim = RC_INFINITY;

    last = m_pertinent_walls.upper_bound(dptype);

    //Just check walls pertinent according to possible border occlusion type
    for (pos = m_pertinent_walls.lower_bound(dptype); pos != last; pos++) {

        wall_list = pos->second;

        //Set 2D wall pertinent interval
        wall_list->get2DwallLimitPoints(model_hmax, SM_CALIB_MATRIX(m_context), &p1, &p2);
        Interval::newInterval(&int2, POINT_2D_X(&p1), POINT_2D_X(&p2));

        if(Interval::intersect(&result, &int1, &int2)) { //If there is some influence of wall in blob occlusion

            is_function = wall_list->get2DWallLine(model_hmax, &a, &b);
            if(is_function) {
                //Taking the value of minimal distance
                if(MM_CAM_OCCL_BOTTOM & dptype) {
                    exist_pertinent_bottom = true;
                    y = left_x*a + b;
                    left_lim   = (left_lim > y)   ? y : left_lim;
                    y = center_x*a + b;
                    center_lim = (center_lim > y) ? y : center_lim;
                    y = right_x*a + b;
                    right_lim  = (right_lim > y)  ? y : right_lim;
                }

                if(MM_CAM_OCCL_TOP & dptype) { //TOP
                    exist_pertinent_top = true;
                    y = left_x*a + b;
                    left_lim   = (left_lim < y)   ? y : left_lim;
                    y = center_x*a + b;
                    center_lim = (center_lim < y) ? y : center_lim;
                    y = right_x*a + b;
                    right_lim  = (right_lim < y)  ? y : right_lim;
                }
            }
        }
    }

    if(exist_pertinent_bottom) {
        //Taking the value of maximal distance
        if(MM_CAM_OCCL_BOTTOM & dptype)
            limit = (left_lim > center_lim) ?
                  ( (left_lim > right_lim) ? left_lim : right_lim ) : ( (center_lim > right_lim) ? center_lim : right_lim );
    } else
        borderMaxBottom = RC_INFINITY;

    if(exist_pertinent_top) { //TOP
        borderMinTop = (left_lim < center_lim) ?
            ( (left_lim < right_lim) ? left_lim : right_lim ) : ( (center_lim < right_lim) ? center_lim : right_lim );
    } else
        borderMinTop = - RC_INFINITY;

}

//Decide for the right limits for all possible growths of a blob, according to all types of occlusion detected and to physical limits imposed by current
//analyzed expected object model.
void ReliabilityClassification::adjustBlobLimitsByOcclusion(Blob *blob) {
    DetectionProblemType dptype = BLOB_DP_TYPE(blob), obj_dptype;
    double selectedLeft, selectedRight, selectedTop, selectedBottom;
    int filtered_oc_borders, filtered_oc_objects, filtered_oc_max;
    
    //Initial values for growth limits 
    Hmax = BLOB_HEIGHT(blob);
    Wmax = BLOB_WIDTH(blob);    
    OcclusionMinLeft   = borderMinLeft   = objectMinLeft   = BLOB_XLEFT(blob); 
    OcclusionMaxRight  = borderMaxRight  = objectMaxRight  = BLOB_XRIGHT(blob); 
    OcclusionMinTop    = borderMinTop    = objectMinTop    = BLOB_YTOP(blob); 
    OcclusionMaxBottom = borderMaxBottom = objectMaxBottom = BLOB_YBOTTOM(blob); 

    //Set the maximal possible 2D height of an object according to physical limits imposed by current analyzed expected object model,
    //if some type of vertical occlusion has been detected.
    if( (filtered_oc_max = (dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP | MM_OBJECT_BOTTOM | MM_OBJECT_TOP) ) ) ) {
        Hmax = limits_by_max_possible_blob(blob, (DetectionProblemType)filtered_oc_max);
        if(Hmax < BLOB_HEIGHT(blob)) //If maximal height is less than current blob height, then is not possible to enlarge the blob vertically
            dptype = (DetectionProblemType)(dptype & (MM_CAM_OCCL_LEFT | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT | MM_OBJECT_RIGHT) );
    }

    //Set the maximal possible 2D width of an object according to physical limits imposed by current analyzed expected object model,
    //if some type of horizontal occlusion has been detected.
    if( (filtered_oc_max = (dptype & (MM_CAM_OCCL_LEFT | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT | MM_OBJECT_RIGHT) ) ) ) {
        Wmax = limits_by_max_possible_blob(blob, (DetectionProblemType)filtered_oc_max);
        if(Wmax < BLOB_WIDTH(blob)) //If maximal width is less than current blob width, then is not possible to enlarge the blob horizontally
            dptype = (DetectionProblemType)(dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP | MM_OBJECT_BOTTOM | MM_OBJECT_TOP) );
    }

    //Set vertical occlusion on border growth limits.
    if( (filtered_oc_borders = dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP)) )
        check_walls_hor(blob, (DetectionProblemType) filtered_oc_borders);

    //Set horizontal occlusion on border growth limits.
    if( (filtered_oc_borders = dptype & (MM_CAM_OCCL_LEFT | MM_CAM_OCCL_RIGHT)) )
        check_walls_ver(blob, (DetectionProblemType) filtered_oc_borders);

    //Set vertical occlusion with context objects growth limits.
    if( (filtered_oc_objects = dptype & (MM_OBJECT_BOTTOM | MM_OBJECT_TOP)) )
        setObjectOcclusionLimit(blob, (DetectionProblemType) filtered_oc_objects, true);

    //Set horizontal occlusion with context objects growth limits.
    if( (filtered_oc_objects = dptype & (MM_OBJECT_LEFT | MM_OBJECT_RIGHT)) )
        setObjectOcclusionLimit(blob, (DetectionProblemType) filtered_oc_objects, false);

    //Select more constraining limits

    selectedLeft   = 
        ((dptype & MM_CAM_OCCL_LEFT) && (dptype & MM_OBJECT_LEFT)) ?
        ( (borderMinLeft > objectMinLeft) ? borderMinLeft : objectMinLeft ) :
        ( (borderMinLeft < objectMinLeft) ? borderMinLeft : objectMinLeft ) ;

    selectedRight  = 
        ((dptype & MM_CAM_OCCL_RIGHT) && (dptype & MM_OBJECT_RIGHT)) ?
        ( (borderMaxRight < objectMaxRight) ? borderMaxRight : objectMaxRight ) :
        ( (borderMaxRight > objectMaxRight) ? borderMaxRight : objectMaxRight ) ;

    selectedTop    = 
        ((dptype & MM_CAM_OCCL_TOP) && (dptype & MM_OBJECT_TOP)) ?
        ( (borderMinTop > objectMinTop) ? borderMinTop : objectMinTop ) :
        ( (borderMinTop < objectMinTop) ? borderMinTop : objectMinTop ) ;

    selectedBottom = 
        ((dptype & MM_CAM_OCCL_BOTTOM) && (dptype & MM_OBJECT_BOTTOM)) ?
        ( (borderMaxBottom < objectMaxBottom) ? borderMaxBottom : objectMaxBottom ) :
        ( (borderMaxBottom > objectMaxBottom) ? borderMaxBottom : objectMaxBottom ) ;

    //Adjust the limits according to limits imposed by the maximal possible width and height. 

    if( dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT))
        OcclusionMinLeft = BLOB_XRIGHT(blob) - selectedLeft > Wmax ? BLOB_XLEFT(blob) - (Wmax - BLOB_WIDTH(blob)) : selectedLeft ;
    if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT))
        OcclusionMaxRight = selectedRight - BLOB_XLEFT(blob) > Wmax ? BLOB_XRIGHT(blob) + (Wmax - BLOB_WIDTH(blob)) : selectedRight ;
    if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP))
        OcclusionMinTop = BLOB_YBOTTOM(blob) - selectedTop > Hmax ? BLOB_YTOP(blob) - (Hmax - BLOB_HEIGHT(blob)) : selectedTop ;
    if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM))
        OcclusionMaxBottom = selectedBottom - BLOB_YTOP(blob) > Hmax ? BLOB_YBOTTOM(blob) + (Hmax - BLOB_HEIGHT(blob)) : selectedBottom ;

    //If object occlusion exists, limit the bounds to context objects bounding box
    
    if( (obj_dptype = (DetectionProblemType) (dptype & (MM_OBJECT_LEFT | MM_OBJECT_RIGHT | MM_OBJECT_BOTTOM | MM_OBJECT_TOP)) ) ) {
        std::map<world::ContextObject *, DetectionProblemType>::iterator obj_iterator;
        Rectangle<double> *obj_bbox;
        for(obj_iterator = object_occlusion_type.begin(); obj_iterator != object_occlusion_type.end(); obj_iterator++) {
            if((*obj_iterator).second & obj_dptype) {
                obj_bbox = &POLYGON_2D_BB(WOBJECT_OUTLINE2D_ON_IMAGE((*obj_iterator).first));
                if( ((*obj_iterator).second & obj_dptype & MM_OBJECT_LEFT) && (RECT_XLEFT(obj_bbox) > OcclusionMinLeft) )
                    OcclusionMinLeft = RECT_XLEFT(obj_bbox);
                if( ((*obj_iterator).second & obj_dptype & MM_OBJECT_RIGHT) && (RECT_XRIGHT(obj_bbox) < OcclusionMaxRight) )
                    OcclusionMaxRight = RECT_XRIGHT(obj_bbox);
                if( ((*obj_iterator).second & obj_dptype & MM_OBJECT_TOP) && (RECT_YTOP(obj_bbox) > OcclusionMinTop) )
                    OcclusionMinTop = RECT_YTOP(obj_bbox);
                if( ((*obj_iterator).second & obj_dptype & MM_OBJECT_BOTTOM) && (RECT_YBOTTOM(obj_bbox) < OcclusionMaxBottom) )
                    OcclusionMaxBottom = RECT_YBOTTOM(obj_bbox);
            }
        }
    }
}

//Get the projection in a height (the 3D maximal expected object model) for the farest point of the intersection between a segment and a vertical
//segment in y2D image coordinates. This projection imposes the softer limit over an analyzed point x2D in image coordinates. Used in the case of
//horizontal context object occlusion.
double ReliabilityClassification::getX2DInHeight(std::map<int, double> analyzedY, world::Wall2D * wall, double objecth, bool max) {
    double x3d, y3d;
    double x2d, x2d1, x2d2, aux;
    world::WallSegment *ws = WALL2D_SEGMENT(wall,3);
    Interval analyzed, segment;
    Interval::newInterval(&analyzed, analyzedY[0], analyzedY[2]);
    Interval::newInterval(&segment,  WSEGMENT_Y1(ws), WSEGMENT_Y2(ws));
    Interval::intersect(&analyzed, &analyzed, &segment);

    if(WSEGMENT_IS_VERTICAL(ws)) {
        x2d = WSEGMENT_X1(ws); //if segment vertical, x2d is constant

        //Get the projection in height for limiting points of the analyzed interval.
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, INTERVAL_X1(&analyzed), 0, &x3d, &y3d);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &x2d1, &aux);

        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, INTERVAL_X2(&analyzed), 0, &x3d, &y3d);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &x2d2, &aux);
    } else {
        if(WSEGMENT_SLOPE(ws) == 0) { //if segment horizontal, x2d coordinates correspond to the bounds of analyzed interval
            x2d1 = WSEGMENT_X1(ws);
            x2d2 = WSEGMENT_X2(ws);
            x2d = (!max) ? (x2d1 > x2d2 ? x2d1 : x2d2) : (x2d1 < x2d2 ? x2d1 : x2d2); //Use the correct point according to occlusion type

            //Only one projection is needed because the right point have been already chosen
            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, INTERVAL_X1(&analyzed), 0, &x3d, &y3d);
            SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &x2d1, &aux);
            x2d2 = x2d1;
        } else {
            //In normal case, calculate and project the x2d points for both analyzed interval limits

            x2d = (INTERVAL_X1(&analyzed) - WSEGMENT_INTERCEPT(ws))/WSEGMENT_SLOPE(ws);

            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, INTERVAL_X1(&analyzed), 0, &x3d, &y3d);
            SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &x2d1, &aux);

            x2d = (INTERVAL_X1(&analyzed) - WSEGMENT_INTERCEPT(ws))/WSEGMENT_SLOPE(ws);

            SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d, INTERVAL_X2(&analyzed), 0, &x3d, &y3d);
            SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &x2d2, &aux);
        }
    }
    //Return the appropriate value according to the occlusion type
    return (max) ? (x2d1 > x2d2 ? x2d1 : x2d2) : (x2d1 < x2d2 ? x2d1 : x2d2);
}

//Get the projection in a height (the 3D maximal expected object model) for the farest point of the intersection between a segment and a horizontal
//segment in x2D image coordinates. This projection imposes the softer limit over an analyzed point y2D in image coordinates. Used in the case of
//vertical context object occlusion.
double ReliabilityClassification::getY2DInHeight(std::map<int, double> analyzedX, world::Wall2D * wall, double objecth, bool max) {
    double x3d, y3d;
    double y2d, y2d1, y2d2, aux;
    world::WallSegment *ws = WALL2D_SEGMENT(wall,3);
    Interval analyzed, segment;
    Interval::newInterval(&analyzed, analyzedX[0], analyzedX[2]);
    Interval::newInterval(&segment,  WSEGMENT_X1(ws), WSEGMENT_X2(ws));
    Interval::intersect(&analyzed, &analyzed, &segment);

    if(WSEGMENT_IS_VERTICAL(ws)) {
        //if segment vertical, y2d coordinates correspond to the bounds of analyzed interval

	y2d1 = WSEGMENT_Y1(ws);
	y2d2 = WSEGMENT_Y2(ws);
	y2d = (!max) ? (y2d1 > y2d2 ? y2d1 : y2d2) : (y2d1 < y2d2 ? y2d1 : y2d2);
      
	//Both projections are performed because some difference can occur according to the projection in height in different points of the image
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), INTERVAL_X1(&analyzed), y2d, 0, &x3d, &y3d);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &aux, &y2d1);

        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), INTERVAL_X2(&analyzed), y2d, 0, &x3d, &y3d);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &aux, &y2d2);
    } else {
      //In normal case, calculate and project the y2d points for both analyzed interval limits
      y2d = INTERVAL_X1(&analyzed)*WSEGMENT_SLOPE(ws) + WSEGMENT_INTERCEPT(ws);

      SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), INTERVAL_X1(&analyzed), y2d, 0, &x3d, &y3d);
      SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &aux, &y2d1);

      y2d = INTERVAL_X2(&analyzed)*WSEGMENT_SLOPE(ws) + WSEGMENT_INTERCEPT(ws);

      SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), INTERVAL_X1(&analyzed), y2d, 0, &x3d, &y3d);
      SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x3d, y3d, std::min(objecth, model_hmax), &aux, &y2d2);
    }

    //Return the appropriate value according to the occlusion type
    return (max) ? (y2d1 > y2d2 ? y2d1 : y2d2) : (y2d1 < y2d2 ? y2d1 : y2d2);
}

//Calculates the limiting blob growth bounds for all pertinent context object occlusion cases
void ReliabilityClassification::setObjectOcclusionLimit(Blob *blob, DetectionProblemType objects_occlusion, bool vertical) {
    
    int i;
    std::map<world::ContextObject *, std::map<int, std::deque<inWall> > >::iterator objects_iter;
    std::deque<inWall>::iterator walls_iter;
    double current_limit;
    bool first_object, first_wall;
    double limit[3];
    bool valid[3];

    if(vertical) {
        limit[0] = limit[1] = limit[2] = BLOB_YCENTER(blob);
        if(objects_occlusion & MM_OBJECT_TOP) {
            first_object = true;
            for(objects_iter = enteredWallsForPointTop.begin(); objects_iter != enteredWallsForPointTop.end(); objects_iter++) {
                for(i=0; i<3; i++) {
                    if((objects_iter->second)[i].empty()) {
                        valid[i] = false;
                        continue;
                    }
                    valid[i] = true;
                    first_wall = true;
                    for(walls_iter = (objects_iter->second)[i].begin(); walls_iter != (objects_iter->second)[i].end(); walls_iter++) {
                        if(first_wall) {
                            first_wall = false;
                            //For the first wall limiting a point, if first wall is too far or top of the wall is too far, this POINT of the blob is not occluded.
                            if(    (BLOB_YTOP(blob) - (*walls_iter).getEntering() > RC_PROXIMITY_PIXELS)
                                || ( ((*walls_iter).getEnteringType() == 0) && (BLOB_YTOP(blob) - (*walls_iter).getLeaving() > RC_PROXIMITY_PIXELS))) {
                                limit[i] = BLOB_YTOP(blob);
                                break;
                            }
                        } else { //For not first walls
                            // If the current wall is occluded by the one before, just ignore it...
                            if((*walls_iter).getEnteringType() > 0 && (*walls_iter).getLeavingType() > 0 && (*walls_iter).getLeaving() >= limit[i])
                                continue;
                            if(limit[i] - (*walls_iter).getEntering() > RC_PROXIMITY_PIXELS ) // Next wall too far, then we stop.
                                break;
                            if((*walls_iter).getEnteringType() == 0 && limit[i] - (*walls_iter).getLeaving()  > RC_PROXIMITY_PIXELS ) // Next wall in height too far, then we stop also.
                                break;
                        }

                        //In all other cases, we search for a limiting segment on the floor, which immediatelly limits the growth of a dimension. If this is the case, we set the
                        //final limit for the blob point, as the intersecting segment point projected in the maximal height of the currently analyzed expected object.
                        //If the segment is not on the floor, the maximal limit is updated and we continue the search for the maximal.
                        //For the other object occlusion types is the same idea.
                        if((*walls_iter).getEnteringType() == 0) {
                            limit[i] = std::min((*walls_iter).getEntering(),
                                       getY2DInHeight(analyzedPointsForObjectX[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), false));
                            break;
                        } else if((*walls_iter).getLeavingType() == 0) {
                            limit[i] = std::min((*walls_iter).getLeaving(),
                                       getY2DInHeight(analyzedPointsForObjectX[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), false));
                            break;
                        } else {
                            limit[i] = (*walls_iter).getLeaving();
                        }
                    }
                }
                if(!valid[0] && !valid[2]) {
                    limit[0] = limit[2] = limit[1]; //Supposing that central point won't fail to find walls
                } else if(!valid[0])
                    limit[0] = limit[1];
                else if(!valid[2])
                    limit[2] = limit[1];
	    
                if(first_object) {
                    first_object = false;
                    objectMinTop = current_limit = std::min(limit[0], std::min(limit[1], limit[2]));
                } else {
                    current_limit = std::min(limit[0], std::min(limit[1], limit[2]));
                    if(current_limit < objectMinTop)
                        objectMinTop = current_limit;
                }
            }
        }

        if(objects_occlusion & MM_OBJECT_BOTTOM) {
            first_object = true;
            for(objects_iter = enteredWallsForPointBottom.begin(); objects_iter != enteredWallsForPointBottom.end(); objects_iter++) {
                for(i=0; i<3; i++) {
                    if((objects_iter->second)[i].empty()) {
                        valid[i] = false;
                        continue;
                    }
                    valid[i] = true;
                    first_wall = true;
                    for(walls_iter = (objects_iter->second)[i].begin(); walls_iter != (objects_iter->second)[i].end(); walls_iter++) {
                        if(first_wall) {
                            first_wall = false;
                            if(    ((*walls_iter).getEntering() - BLOB_YBOTTOM(blob) > RC_PROXIMITY_PIXELS)
                                || ( ((*walls_iter).getEnteringType() == 0) && ((*walls_iter).getLeaving() - BLOB_YBOTTOM(blob) > RC_PROXIMITY_PIXELS))) {
                                limit[i] = BLOB_YBOTTOM(blob);
                                break;
                            }
                        } else {
                            if((*walls_iter).getEnteringType() > 0 && (*walls_iter).getLeavingType() > 0 && (*walls_iter).getLeaving() <= limit[i]) // The current wall is occluded by the one before
                                continue;
                            if((*walls_iter).getEntering() - limit[i] > RC_PROXIMITY_PIXELS ) // Next wall too far
                                break;
                            if((*walls_iter).getEnteringType() == 0 && (*walls_iter).getLeaving() - limit[i] > RC_PROXIMITY_PIXELS ) // Next wall in height too far
                                break;
                        }

                        if((*walls_iter).getEnteringType() == 0) {
                            limit[i] = std::max((*walls_iter).getEntering(),
                                       getY2DInHeight(analyzedPointsForObjectX[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), true));
                            break;
                        } else if((*walls_iter).getLeavingType() == 0) {
                            limit[i] = std::max((*walls_iter).getLeaving(),
                                       getY2DInHeight(analyzedPointsForObjectX[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), true));
                            break;
                        } else {
                            limit[i] = (*walls_iter).getLeaving();
                        }
                    }
                }

                //If some point didn't intercept a wall.
                if(!valid[0] && !valid[2]) {
                    limit[0] = limit[2] = limit[1]; //Supposing that central point won't fail to find walls
                } else if(!valid[0])
                    limit[0] = limit[1];
                else if(!valid[2])
                    limit[2] = limit[1];
	    
                //Set object limit
                if(first_object) {
                    first_object = false;
                    objectMaxBottom = current_limit = std::max(limit[0], std::max(limit[1], limit[2]));
                } else {
                    current_limit = std::max(limit[0], std::max(limit[1], limit[2]));
                    if(current_limit > objectMaxBottom)
                        objectMaxBottom = current_limit;
                }
            }
        }
    } else { //Horizontal occlusion
        limit[0] = limit[1] = limit[2] = BLOB_XCENTER(blob);

        if(objects_occlusion & MM_OBJECT_LEFT) {
            first_object = true;
            for(objects_iter = enteredWallsForPointLeft.begin(); objects_iter != enteredWallsForPointLeft.end(); objects_iter++) {
                for(i=0; i<3; i++) {
                    if((objects_iter->second)[i].empty()) {
                        valid[i] = false;
                        continue;
                    }
                    valid[i] = true;
                    first_wall = true;
                    for(walls_iter = (objects_iter->second)[i].begin(); walls_iter != (objects_iter->second)[i].end(); walls_iter++) {
                        if(first_wall) {
                            first_wall = false;
                            if(    (BLOB_XLEFT(blob) - (*walls_iter).getEntering() > RC_PROXIMITY_PIXELS)
                                || ( ((*walls_iter).getEnteringType() == 0) && (BLOB_XLEFT(blob) - (*walls_iter).getLeaving() > RC_PROXIMITY_PIXELS))) {
                                limit[i] = BLOB_XLEFT(blob);
                                break;
                            }
                        } else {
                            if((*walls_iter).getEnteringType() > 0 && (*walls_iter).getLeavingType() > 0 && (*walls_iter).getLeaving() >= limit[i]) // The current wall is occluded by the one before
                                continue;
                            if(limit[i] - (*walls_iter).getEntering() > RC_PROXIMITY_PIXELS ) // Next wall too far
                                break;
                            if((*walls_iter).getEnteringType() == 0 && limit[i] - (*walls_iter).getLeaving()  > RC_PROXIMITY_PIXELS ) // Next wall in height too far
                                break;
                        }

                        if((*walls_iter).getEnteringType() == 0) {
                            limit[i] = std::min((*walls_iter).getEntering(),
                                       getX2DInHeight(analyzedPointsForObjectY[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), false));
                            break;
                        } else if((*walls_iter).getLeavingType() == 0) {
                            limit[i] = std::min((*walls_iter).getLeaving(),
                                       getX2DInHeight(analyzedPointsForObjectY[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), false));
                            break;
                        } else {
                            limit[i] = (*walls_iter).getLeaving();
                        }
                    }
                }
                if(!valid[0] && !valid[2]) {
                    limit[0] = limit[2] = limit[1]; //Supposing that central point won't fail to find walls
                } else if(!valid[0])
                    limit[0] = limit[1];
                else if(!valid[2])
                    limit[2] = limit[1];
	    
                if(first_object) {
                    first_object = false;
                    objectMinLeft = current_limit = std::min(limit[0], std::min(limit[1], limit[2]));
                } else {
                    current_limit = std::min(limit[0], std::min(limit[1], limit[2]));
                    if(current_limit < objectMinLeft)
                        objectMinLeft = current_limit;
                }
            }
        }

        if(objects_occlusion & MM_OBJECT_RIGHT) {
            first_object = true;
            for(objects_iter = enteredWallsForPointRight.begin(); objects_iter != enteredWallsForPointRight.end(); objects_iter++) {
                for(i=0; i<3; i++) {
                    if((objects_iter->second)[i].empty()) {
                        valid[i] = false;
                        continue;
                    }
                    valid[i] = true;
                    first_wall = true;
                    for(walls_iter = (objects_iter->second)[i].begin(); walls_iter != (objects_iter->second)[i].end(); walls_iter++) {
                        if(first_wall) {
                            first_wall = false;
                            if(    ((*walls_iter).getEntering() - BLOB_XRIGHT(blob) > RC_PROXIMITY_PIXELS)
                                || ( ((*walls_iter).getEnteringType() == 0) && ((*walls_iter).getLeaving() - BLOB_XRIGHT(blob) > RC_PROXIMITY_PIXELS))) {
                                limit[i] = BLOB_XRIGHT(blob);
                                break;
                            }
                        } else {
                            if((*walls_iter).getEnteringType() > 0 && (*walls_iter).getLeavingType() > 0 && (*walls_iter).getLeaving() <= limit[i]) // The current wall is occluded by the one before
                                continue;
                            if((*walls_iter).getEntering() - limit[i] > RC_PROXIMITY_PIXELS ) // Next wall too far
                                break;
                            if((*walls_iter).getEnteringType() == 0 && (*walls_iter).getLeaving() - limit[i] > RC_PROXIMITY_PIXELS ) // Next wall in height too far
                                break;
                        }

                        if((*walls_iter).getEnteringType() == 0) {
                            limit[i] = std::max((*walls_iter).getEntering(),
                                       getX2DInHeight(analyzedPointsForObjectY[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), true));
                            break;
                        } else if((*walls_iter).getLeavingType() == 0) {
                            limit[i] = std::max((*walls_iter).getLeaving(),
                                       getX2DInHeight(analyzedPointsForObjectY[(objects_iter->first)], (*walls_iter).getWall(), WOBJECT_HEIGHT(objects_iter->first), true));
                            break;
                        } else {
                            limit[i] = (*walls_iter).getLeaving();
                        }
                    }
                }

                //If some point didn't intercept a wall.
                if(!valid[0] && !valid[2]) {
                    limit[0] = limit[2] = limit[1]; //Supposing that central point won't fail to find walls
                } else if(!valid[0])
                    limit[0] = limit[1];
                else if(!valid[2])
                    limit[2] = limit[1];

                //Set object limit
                if(first_object) {
                    first_object = false;
                    objectMaxRight = current_limit = std::max(limit[0], std::max(limit[1], limit[2]));
                } else {
                    current_limit = std::max(limit[0], std::max(limit[1], limit[2]));
                    if(current_limit > objectMaxRight)
                        objectMaxRight = current_limit;
                }
            }
        }
    }
}

//Set object type, occlussion type and 3D bounding box for a blob
void ReliabilityClassification::setBlob3DFacts(Blob *blob, QImage *i_segm) {

    double x, y;
    DetectionProblemType obj_dptype = MM_DP_NONE, border_dptype = MM_DP_NONE;
    bool in_context_wall = false, changed_occl = false;
    possible_occlusion = false;

    //Get foreground pixels for given blob:
    m_foreground = i_segm;

    W = BLOB_WIDTH(blob);
    H = BLOB_HEIGHT(blob);

    //Set default blob 3D position if type is UNKNOWN (to redo with parallelpiped data)
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context),
                                            BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0,
                                            &x, &y);
    BLOB_3D_X(blob) = x;
    BLOB_3D_Y(blob) = y;
//REVISAR aoi_in
    //aoi_in = blobOnAOIGround(blob, m_context);
    aoi_in = true;
#ifdef RC_OUTPUT_1
    AppendToLog("BL-"+ BLOB_IDENT(blob) +" POSITION:\nX: "+ BLOB_3D_X(blob)
              +"\tY: "+ BLOB_3D_Y(blob) +"\tZ: "+ BLOB_3D_Z(blob) +std::endl;
#endif
      
    //Set information to be used in parallelpiped calculation, that can be calculated in blob level:
    RC_set_3D_bbox_blob_level_data(blob);
      
    //Initialize structures and variables used for the determination of object class.
    initBlob3DData();
      
#ifdef RC_OUTPUT_1
    AppendToLog("BLOBID\t'" + BLOB_IDENT(blob) + "'");
#endif	  

    if(m_treatWallCoherence)
        annoyingWallSegments.clear();

    if(m_treatContextObjectOcclusion) {
        near_objects.clear();
        not_near_objects.clear();
        object_occlusion_type.clear();
        //Take into account occlusions related with camera limits and context objects.
        obj_dptype = isBlobOccludedByStaticObject(blob);
    }

    if(m_treatBorderOcclusion)
        border_dptype = isBlobOnImageBorder(blob);

    if(border_dptype || obj_dptype) { //If it can exist some kind of occlusion
        possible_occlusion = true;
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int) BLOB_DP_TYPE(blob) | obj_dptype);
    } 

    //Check if blob is too little or too big according to pre-defined object models
    checkBlobSize(blob);
      
    if(m_treatWallCoherence) {
        if(!m_treatContextObjectOcclusion) {
            near_objects.clear();
            not_near_objects.clear();
            setNearObjects(blob);
        }
        setAnnoyingWallsForContextWalls(blob, true);
        //Check if blob is over a context wall
        in_context_wall = blobOverContextWalls(blob);
    }

    someNormalSolutionInserted = false;
    someOcclusionSolutionInserted = false;

    //If a blob has no possibility of expansion (not occluded).
    if( !(obj_dptype & (MM_OBJECT_OCCL_MASK | MM_CAM_OCCL_MASK)) ) {
        if( ( (obj_dptype & MM_OBJECT_INSIDE) && BLOB_IS_REDUCED_SIZE(blob) ) || in_context_wall ) {
            //If blob is totally inside a wall or an object, must be treated later (merging maybe)
            BLOB_DP_TYPE(blob) = (DetectionProblemType) (BLOB_DP_TYPE(blob) | MM_OBJECT_INSIDE);
        } else if(!aoi_in) { //If is just out of the zone of interest, is just not interesting :P
            BLOB_DP_TYPE(blob) = MM_AOI_OUTSIDE;
            assignTypeToBlob(blob);
            return;
        }
    }

    if( (BLOB_DP_TYPE(blob) & MM_TOP_OCCL) && (BLOB_DP_TYPE(blob) & (MM_BOTTOM_OCCL | MM_LEFT_OCCL | MM_RIGHT_OCCL)) ) {
        BLOB_DP_TYPE(blob) = (DetectionProblemType) (BLOB_DP_TYPE(blob) - (BLOB_DP_TYPE(blob) & (MM_OBJECT_TOP | MM_DYNAMIC_TOP)));
        changed_occl = true;
    }
    if(changed_occl) {
        std::map<world::ContextObject *, DetectionProblemType>::iterator obj_iterator;
        //AppendToLog("\tOcclusions for blob " + BLOB_IDENT(blob) + ":");
        for(obj_iterator = object_occlusion_type.begin(); obj_iterator != object_occlusion_type.end(); obj_iterator++) {
            if((*obj_iterator).second  & (MM_OBJECT_TOP | MM_DYNAMIC_TOP))
                (*obj_iterator).second = (DetectionProblemType) ((*obj_iterator).second - ((*obj_iterator).second & (MM_OBJECT_TOP | MM_DYNAMIC_TOP)));
	  	  
            //if((*obj_iterator).second & MM_OBJECT_INSIDE)
                //AppendToLog("\t\tObject " + WOBJECT_NAME((*obj_iterator).first) + ":" + get_occl_name_from_type((DetectionProblemType)((*obj_iterator).second - MM_OBJECT_INSIDE)) + " and " + get_occl_name_from_type(MM_OBJECT_INSIDE));
            //else
                //AppendToLog("\t\tObject " + WOBJECT_NAME((*obj_iterator).first) + ":" + get_occl_name_from_type((*obj_iterator).second));
        }
    }
    //TO-DO!!! Verify at pixel level if a blob is occluded by an object or over it.
    //In the mean time TOP occlusion treatment is suppressed, unless it is the only occlusion type present
    //for a blob or the TOP occlusion is of camera occlusion type.
      
    //Calculate the highest probabilities with its respective reliability and dimensions values
    if( ( (aoi_in && !BLOB_IS_REDUCED_SIZE(blob)) || possible_occlusion) && !BLOB_IS_EXCESIVE_SIZE(blob))
        top_probabilities_and_reliabilities(blob);
    
    //Decide the blob type and if a solution considering occlusion is better than the normal case solutions. 
    assignTypeToBlob(blob);

#ifdef RC_OUTPUT_1
    AppendToLog("WIDTH 2D\t'" +  BLOB_WIDTH(blob) + "'");
    AppendToLog("HEIGHT_2D\t'" +  BLOB_HEIGHT(blob) + "'");
    AppendToLog("X CENTER\t'" +  BLOB_XCENTER(blob) + "'");
    AppendToLog("Y CENTER\t'" +  BLOB_YCENTER(blob) + "'");
    AppendToLog("Y BOTTOM\t'" +     BLOB_YBOTTOM(blob) + "'");
    AppendToLog("Y TOP\t'" +     BLOB_YTOP(blob) + "'");
    AppendToLog("X LEFT\t'" +     BLOB_XLEFT(blob) + "'");
    AppendToLog("X RIGHT\t'" +     BLOB_XRIGHT(blob) + "'");
    AppendToLog("OCCLUSION TYPE\t'" +  BLOB_DP_TYPE(blob) + "'");
    AppendToLog("MORE PROBABLE TYPE\t'" +  Blob::getNameFromType(BLOB_TYPE(blob)));
    AppendToLog("REAL DISTANCE\t" + D);
    AppendToLog("REAL POINT:\tX: " + BLOB_3D_X(blob) + "\tY: " + BLOB_3D_Y(blob) + "'");
    top_output();
#endif
    
}

void ReliabilityClassification::prepareBlobForRClassification(Blob *blob, QImage *i_segm) {

    double x, y;
    DetectionProblemType obj_dptype = MM_DP_NONE, border_dptype = MM_DP_NONE;
    bool in_context_wall = false, changed_occl = false;
    possible_occlusion = false;

    //Get foreground pixels for given blob:
    m_foreground = i_segm;

    W = BLOB_WIDTH(blob);
    H = BLOB_HEIGHT(blob);

    //Set default blob 3D position if type is UNKNOWN (to redo with parallelpiped data)
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0, &x, &y);
    BLOB_3D_X(blob) = x;
    BLOB_3D_Y(blob) = y;

    //REVISAR
    //aoi_in = blobOnAOIGround(blob, m_context);
    aoi_in = true;

    //Set information to be used in parallelpiped calculation, that can be calculated in blob level:
    RC_set_3D_bbox_blob_level_data(blob);
      
    //Initialize structures and variables used for the determination of object class.
    initBlob3DData();
      
    if(m_treatWallCoherence)
        annoyingWallSegments.clear();

    if(m_treatContextObjectOcclusion) {
        near_objects.clear();
        not_near_objects.clear();
        object_occlusion_type.clear();
        //Take into account occlusions related with camera limits and context objects.
        obj_dptype = isBlobOccludedByStaticObject(blob);
    }

    if(m_treatBorderOcclusion)
        border_dptype = isBlobOnImageBorder(blob);

    if(border_dptype || obj_dptype) { //If it can exist some kind of occlusion
        possible_occlusion = true;
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int) BLOB_DP_TYPE(blob) | obj_dptype);
    } 

    //Check if blob is too little or too big according to pre-defined object models
    checkBlobSize(blob);
      
    if(m_treatWallCoherence) {
        if(!m_treatContextObjectOcclusion) {
            near_objects.clear();
            not_near_objects.clear();
            setNearObjects(blob);
        }
        setAnnoyingWallsForContextWalls(blob, true);
        //Check if blob is over a context wall
        in_context_wall = blobOverContextWalls(blob);
    }
    someNormalSolutionInserted = false;    
    someOcclusionSolutionInserted = false;

    if( (BLOB_DP_TYPE(blob) & MM_TOP_OCCL) && (BLOB_DP_TYPE(blob) & (MM_BOTTOM_OCCL | MM_LEFT_OCCL | MM_RIGHT_OCCL)) ) {
        BLOB_DP_TYPE(blob) = (DetectionProblemType) (BLOB_DP_TYPE(blob) - (BLOB_DP_TYPE(blob) & (MM_OBJECT_TOP | MM_DYNAMIC_TOP)));
        changed_occl = true;
    }
    if(changed_occl) {
        std::map<world::ContextObject *, DetectionProblemType>::iterator obj_iterator;
        //AppendToLog("\tOcclusions for blob " + BLOB_IDENT(blob) + ":");
        for(obj_iterator = object_occlusion_type.begin(); obj_iterator != object_occlusion_type.end(); obj_iterator++) {
            if((*obj_iterator).second  & (MM_OBJECT_TOP | MM_DYNAMIC_TOP))
                (*obj_iterator).second = (DetectionProblemType) ((*obj_iterator).second - ((*obj_iterator).second & (MM_OBJECT_TOP | MM_DYNAMIC_TOP)));
	
        }
    }
}


void ReliabilityClassification::setDPForBlob(Blob *blob, QImage *i_segm) {

    DetectionProblemType obj_dptype = MM_DP_NONE, border_dptype = MM_DP_NONE;
    bool changed_occl = false;

    //Get foreground pixels for given blob:
    m_foreground = i_segm;

    //Set information to be used in parallelpiped calculation, that can be calculated in blob level:
    RC_set_3D_bbox_blob_level_data(blob);
      
    //Initialize structures and variables used for the determination of object class.
    initBlob3DData();
      
    if(m_treatWallCoherence)
        annoyingWallSegments.clear();

    if(m_treatContextObjectOcclusion) {
        near_objects.clear();
        not_near_objects.clear();
        object_occlusion_type.clear();
        //Take into account occlusions related with camera limits and context objects.
        obj_dptype = isBlobOccludedByStaticObject(blob);
    }

    if(m_treatBorderOcclusion)
        border_dptype = isBlobOnImageBorder(blob);

    if(border_dptype || obj_dptype) //If it can exist some kind of occlusion
        BLOB_DP_TYPE(blob) = (DetectionProblemType)((int) BLOB_DP_TYPE(blob) | obj_dptype);

    if( (BLOB_DP_TYPE(blob) & MM_TOP_OCCL) && (BLOB_DP_TYPE(blob) & (MM_BOTTOM_OCCL | MM_LEFT_OCCL | MM_RIGHT_OCCL)) ) {
        BLOB_DP_TYPE(blob) = (DetectionProblemType) (BLOB_DP_TYPE(blob) - (BLOB_DP_TYPE(blob) & (MM_OBJECT_TOP | MM_DYNAMIC_TOP)));
        changed_occl = true;
    }
}

//Checks if a blob is inside a visible context wall.
bool ReliabilityClassification::blobOverContextWalls(Blob *i_blob) {

    int i, j, inside_count = 0;
    double x[3] = {BLOB_XLEFT(i_blob), BLOB_XCENTER(i_blob), BLOB_XRIGHT(i_blob)};
    double y[3] = {BLOB_YTOP(i_blob), 0.0, BLOB_YBOTTOM(i_blob)};
    std::vector< QSharedPointer<world::Wall2D> > *wall2D_list = &m_context->wall2Ds;
    std::vector< QSharedPointer<world::Wall2D> >::iterator it, end_it = wall2D_list->end();
    bool inside[3][3] = { {false, false, false} 
		        , {false, true,  false}
		        , {false, false, false} };
    QSharedPointer< polygon2D<double> > pol;
    for(it = wall2D_list->begin(); it != end_it; it++) {
        pol = WALL2D_OUTLINE_ON_IMAGE(&**it);
        for(i = 0; i < 3; i++)
            for(j = 0; j < 3; j++)
                if(!inside[i][j] && !((i == 1)&&(j == 1)) && pol->pointInConvexPolygon(x[i], y[j], false)) {
                    inside[i][j] = true;
                    inside_count++;
                }
    }

    if(inside_count == 8)
        return true;
    return false;
}
  
//Set in blob the data concerning the last 3D parallelpiped analysis performed.
void ReliabilityClassification::setLastBlobData(double h, double l, double w, double angle, Blob *blob, bool _90_rotated) {
    BLOB_3D_LENGTH(blob) = l;
    BLOB_3D_WIDTH(blob) = w;
    BLOB_3D_HEIGHT(blob) = h;
    BLOB_ALPHA(blob) = angle;

    if(_90_rotated) { //rotate parallellepiped elements also
        int i;
        double x0   = BLOB_3DBBOX_X_i(blob, 0),        y0   = BLOB_3DBBOX_Y_i(blob, 0);
        double X0_0 = BLOB_3DBBOX_X2D_BASE_i(blob, 0), Y0_0 = BLOB_3DBBOX_Y2D_BASE_i(blob, 0);
        double X0_H = BLOB_3DBBOX_X2D_H_i(blob, 0),    Y0_H = BLOB_3DBBOX_Y2D_H_i(blob, 0);
        for(i=0; i<3; i++) {
            BLOB_3DBBOX_X_i(blob, i) = BLOB_3DBBOX_X_i(blob, i + 1);
            BLOB_3DBBOX_Y_i(blob, i) = BLOB_3DBBOX_Y_i(blob, i + 1);
            BLOB_3DBBOX_X2D_BASE_i(blob, i)  = BLOB_3DBBOX_X2D_BASE_i(blob, i + 1);
            BLOB_3DBBOX_Y2D_BASE_i(blob, i)  = BLOB_3DBBOX_Y2D_BASE_i(blob, i + 1);
            BLOB_3DBBOX_X2D_H_i(blob, i)  = BLOB_3DBBOX_X2D_H_i(blob, i + 1);
            BLOB_3DBBOX_Y2D_H_i(blob, i)  = BLOB_3DBBOX_Y2D_H_i(blob, i + 1);
        }
        BLOB_3DBBOX_X_i(blob, 3) = x0;
        BLOB_3DBBOX_Y_i(blob, 3) = y0;
        BLOB_3DBBOX_X2D_BASE_i(blob, 3) = X0_0;
        BLOB_3DBBOX_Y2D_BASE_i(blob, 3) = Y0_0;
        BLOB_3DBBOX_X2D_H_i(blob, 3) =  X0_H;
        BLOB_3DBBOX_Y2D_H_i(blob, 3) = Y0_H;
    }
}

//Returns the obtained measured probability value for a possible solution to the parallelpiped determination problem and stores the solution in the list of top
//solutions if the new solution is good enough in terms of the probabilistic measure.
double ReliabilityClassification::set_possible_solution(double w, double l, double h, Blob *blob, bool rotated, bool normal) {
    with_sol_counter++;
    if(rotated)
        setLastBlobData(h, w, l, alpha + M_PI/2.0, blob, true); //Considering that w is l, and viceversa
    else
        setLastBlobData(h, l, w, alpha, blob, false);

    m_currentModel->computeScore(blob, m_context);
    if(can_be_inserted_top (blob, normal) ) {
        //In occlusion cases this is done before for validation
        if(m_orderByDensity && normal)
            setPixelAnalysis(blob, BLOB_DDATA(blob), BLOB_3DBBOX(blob));
        insert_top(blob, normal);
    }
    return BLOB_P(blob);
}

double ReliabilityClassification::set_possible_solution_for_tracking(double w, double l, double h, Blob *blob, double current_best) {
    double dw, dl, dh, distance, nw, nl, nalpha;

    nw = normalised90 ? l : w;
    nl = normalised90 ? w : l;
    nalpha = NormalizeAngle(alpha  + (normalised180 ? M_PI : 0.0) + (normalised90 ? M_PI/2.0 : 0.0));

    if(normalised90) {
        dw = l - wMobile;
        dl = w - lMobile;
    } else {
        dw = w - wMobile;
        dl = l - lMobile;
    }

    dh = h - hMobile; 
    distance = dw*dw + dl*dl + dh*dh;

    if(distance < current_best) {
        currentDimDistance = current_best = distance;
      
        best_by_h_found = true;

        setLastBlobData(h, nl, nw, nalpha, blob, normalised90);
        setPixelAnalysis(blob, BLOB_DDATA(blob), BLOB_3DBBOX(blob));
        m_currentModel->computeScore(blob, m_context);
      
        S3D_TYPE(best_considering_h) = m_modelId;
        S3D_SUBTYPE(best_considering_h) = m_subModelId;
        S3D_W(best_considering_h) = nw;
        S3D_H(best_considering_h) = h;
        S3D_L(best_considering_h) = nl;
        S3D_ALPHA(best_considering_h) = nalpha;
        S3D_PW(best_considering_h) = BLOB_PW(blob);
        S3D_PL(best_considering_h) = BLOB_PL(blob);
        S3D_PH(best_considering_h) = BLOB_PH(blob);
        S3D_RW(best_considering_h) = BLOB_RW(blob);
        S3D_RL(best_considering_h) = BLOB_RL(blob);
        S3D_RH(best_considering_h) = BLOB_RH(blob);
        S3D_P(best_considering_h) = BLOB_P(blob);
        S3D_R(best_considering_h) = BLOB_R(blob);
        S3D_DP(best_considering_h) = BLOB_DP(blob);
        S3D_PR(best_considering_h) = BLOB_PR(blob);

        memcpy(S3D_BBOX(best_considering_h), BLOB_BBOX(blob), sizeof(Rectangle<int>));
        memcpy(S3D_3DBBOX(best_considering_h), BLOB_3DBBOX(blob), sizeof(Parallelpiped));
        memcpy(S3D_DDATA(best_considering_h), BLOB_DDATA(blob), sizeof(densityData));
        S3D_3DBBOX(best_considering_h)->setReal3DPosition(S3D_3D_POSITION(best_considering_h));
    }
    
    return distance;

}

//Returns the probability value for certain parallelpiped dimensions.
double ReliabilityClassification::getProbability(double h, double l, double w) {
    Blob aux_blob;
    BLOB_3D_WIDTH(&aux_blob) = w;
    BLOB_3D_HEIGHT(&aux_blob) = h;
    BLOB_3D_LENGTH(&aux_blob) = l;
    m_currentModel->computeCriteriaScores(&aux_blob, m_context);
    std::map<QString, Criterion> *results = &(m_currentModel->m_mapCriteria);
    return  (*results)["width"].score * (*results)["depth"].score * (*results)["height"].score;
}

//Stores the best solution regardless if the solution does not accomplishes with dimensional limits imposed by the currently analyzed expected object model.
//This result is used as an starting point by the occlusion treatement part if no real solution have been found.
double ReliabilityClassification::set_best_without_model_limits(Blob *blob, double w, double l, double h, double alpha) {
    bool there_is_current = currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId] ;
    Shape3DData *current_best = currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId] ;

    if(currentRigid)
        there_is_best_valid[m_modelId] = true;
    else
        sub_there_is_best_valid[m_modelId][m_subModelId] = true;

    double newP = getProbability(h, l, w);

    if(!there_is_current || newP >= S3D_P(current_best)) {
        S3D_W(current_best) = w;
        S3D_H(current_best) = h;
        S3D_L(current_best) = l;
        S3D_ALPHA(current_best) = alpha;
        S3D_P(current_best) = newP;
        memcpy(S3D_BBOX(current_best),   BLOB_BBOX(blob), sizeof(Rectangle<int>));
        memcpy(S3D_3DBBOX(current_best), BLOB_3DBBOX(blob), sizeof(Parallelpiped));
    }

    return newP;

}

//Checks if the base of a calculated parallelpiped does not collide with a wall in the scene. Pertinent context walls and walls of context objects are analyzed.
bool ReliabilityClassification::parallelpiped_base_inside_pertinent_walls(Blob *blob, double w, double l) {
    
    if(!m_treatWallCoherence)
        return true;

    if(!annoyingWallSegments.empty()) {
        int i;
        std::deque<world::WallSegment *>::iterator seg_iter;
        Parallelpiped *bb3D = BLOB_3DBBOX(blob);
        polygon2D<double> *base_pol = new polygon2D<double>(4);
        world::WallSegment base_segments[4];
        int int1_index = -1, int2_index = -1;
        double xbb[4], ybb[4], x1bb, x2bb, y1bb, y2bb, xi1 = 0, yi1 = 0, xi2 = 0, yi2= 0, x, y;
        double x1seg, x2seg, y1seg, y2seg;
        bool no_intersection, inside1, inside2;
        double area = w*l, d1, d2, cutted_area;

        //Prepare the treatement of the base of the parallelpiped.
        x1bb = PARALL_X_i(bb3D, 3);
        y1bb = PARALL_Y_i(bb3D, 3);
      
        for(i=0; i<4; i++) {
            x2bb = x1bb;
            y2bb = y1bb;
            x1bb = xbb[i] = PARALL_X_i(bb3D, i);
            y1bb = ybb[i] = PARALL_Y_i(bb3D, i);
            base_pol->points[0].x = x1bb;
            base_pol->points[0].y = y1bb;
            base_segments[i].setWallSegment(x1bb, y1bb, x2bb, y2bb, 0);
        }
      
        //Check all potentially colliding wall segments in the floor.
        for(seg_iter = annoyingWallSegments.begin(); seg_iter != annoyingWallSegments.end(); seg_iter++) {
            x1seg = WSEGMENT_X1(*seg_iter);
            x2seg = WSEGMENT_X2(*seg_iter);
            y1seg = WSEGMENT_X1(*seg_iter);
            y2seg = WSEGMENT_X2(*seg_iter);
            //Check if limits of wall segments are inside the parallelpiped base
            inside1 = (base_pol->pointInConvexPolygon(x1seg, y1seg, false)) ? true : false;
            inside2 = (base_pol->pointInConvexPolygon(x2seg, y2seg, false)) ? true : false;
            no_intersection = true;

            //If at least one is inside, we determine which one is really intersecting with a parallelpiped base segment
            //and which one is not and we store this intersecting points.
            if(inside1 || inside2) {
                for(i=0; i<4; i++) {
                    if(world::WallSegment::twoLinesIntersection(*seg_iter, &base_segments[i], x, y))
                        if(base_segments[i].inSegmentGivenIntersection(x, y) && !(!no_intersection && xi1 == x && yi1 == y)) {
		                                                 //If a point is in a corner it should happen that the two intersections correspond to the same point
		                                                 //and is compulsory to avoid this situation
                            if(no_intersection) {
                                no_intersection = false;
                                xi1 = x;
                                yi1 = y;
                                int1_index = i;
                            } else {
                                xi2 = x;
                                yi2 = y;
                                int2_index = i;
                                break;
                            }
                        }
                }
            } else { // As no point is inside, we don't know yet if a wall crosses the object or if it is completely out
                for(i=0; i<4; i++) {
                    if(world::WallSegment::twoSegmentsIntersection(*seg_iter, &base_segments[i], x, y))
                        if(!(!no_intersection && xi1 == x && yi1 == y)) { //If a point is in a corner it should happen that the two intersections correspond to the same point
		                                                //and is compulsory to avoid this situation
                            if(no_intersection) {
                                no_intersection = false;
                                xi1 = x;
                                yi1 = y;
                                int1_index = i;
                            } else {
                                xi2 = x;
                                yi2 = y;
                                int2_index = i;
                                break;
                            }
                        }
                }
            }

            if(no_intersection || (xi1 == xi2 && yi1 == yi2)) //If there is no intersection or there is some problem with obtained points
                continue;

            //Check the degree of interiorness of partial parts of a wall inside an object
            if(inside1 && !inside2) { //If one inside, calculate the rate of entrance to the base
                d1 = (xi1 - xi2)*(xi1 - xi2) + (yi1 - yi2)*(yi1 - yi2);
                if((*seg_iter)->inSegmentGivenIntersection(xi1, yi1))
                    d2 = (xi1 - x1seg)*(xi1 - x1seg) + (yi1 - y1seg)*(yi1 - y1seg);
                else
                    d2 = (xi2 - x1seg)*(xi2 - x1seg) + (yi2 - y1seg)*(yi2 - y1seg);
                if(sqrt(d2/d1) < m_wallInsideLengthRate) //If line is not so inside of the base
                    continue;
            } else if(!inside1 && inside2) {
                d1 = (xi1 - xi2)*(xi1 - xi2) + (yi1 - yi2)*(yi1 - yi2);
                if((*seg_iter)->inSegmentGivenIntersection(xi1, yi1))
                    d2 = (xi1 - x2seg)*(xi1 - x2seg) + (yi1 - y2seg)*(yi1 - y2seg);
                else
                    d2 = (xi2 - x2seg)*(xi2 - x2seg) + (yi2 - y2seg)*(yi2 - y2seg);
                if(sqrt(d2/d1) < m_wallInsideLengthRate) //If line is not so inside of the base
                    continue;
            }
      
            //Check the area cutted by the wall
            if( (int1_index + 1)%4 == int2_index || (int2_index + 1)%4 == int1_index) { //if walls are contiguous a triangule is formed with intersecting wall
                if((int1_index + 1)%4 == int2_index) {
                    d1 = (xi1 - xbb[int1_index])*(xi1 - xbb[int1_index]) + (yi1 - ybb[int1_index])*(yi1 - ybb[int1_index]);
                    d2 = (xi2 - xbb[int1_index])*(xi2 - xbb[int1_index]) + (yi2 - ybb[int1_index])*(yi2 - ybb[int1_index]);
                } else {
                    d1 = (xi1 - xbb[int2_index])*(xi1 - xbb[int2_index]) + (yi1 - ybb[int2_index])*(yi1 - ybb[int2_index]);
                    d2 = (xi2 - xbb[int2_index])*(xi2 - xbb[int2_index]) + (yi2 - ybb[int2_index])*(yi2 - ybb[int2_index]);
                }

                if(sqrt(d1*d2)/(2.0*area) > m_cuttingAreaRate) {//If covered area is significant
                    delete base_pol;
                    return false;
                }
            } else { //cutted area forms a trapezoid with one square angle
                int ind1 = (!int1_index) ? 3 : int1_index - 1;
                double d3 = sqrt((xbb[int2_index] - xbb[ind1])*(xbb[int2_index] - xbb[ind1]) + (ybb[int2_index] - ybb[ind1])*(ybb[int2_index] - ybb[ind1]));
                d1 = sqrt((xi1 - xbb[ind1])*(xi1 - xbb[ind1]) + (yi1 - ybb[ind1])*(yi1 - ybb[ind1]));
                d2 = sqrt((xi2 - xbb[int2_index])*(xi2 - xbb[int2_index]) + (yi2 - ybb[int2_index])*(yi2 - ybb[int2_index]));
                cutted_area = (d1 + d2)*d3/(2.0*area);
                if(cutted_area > m_cuttingAreaRate || cutted_area < 1.0 - m_cuttingAreaRate) {//If covered area is significant
                    delete base_pol;
                    return false;
                }
            }
        }

        delete base_pol;
    }

    return true;
}


bool ReliabilityClassification::covering_bbox(int blob_position, Blob *blob, ddata_t ddata, Parallelpiped *_3Dbbox) {
    setPixelAnalysis(blob_position, blob, ddata, _3Dbbox);
    double ratio;
    if(DDATA_NPIX(ddata) == 0)
        ratio = 0.0;
    else
        ratio = DDATA_TP(ddata) / DDATA_NPIX(ddata);
    
    if(ratio < m_minPixelCoverageRatio)
        return false;
    
    return true;
}

bool ReliabilityClassification::covering_bbox(Blob *blob) {
    setPixelAnalysis(blob, BLOB_DDATA(blob), BLOB_3DBBOX(blob));
    double ratio;
    if(DDATA_NPIX(BLOB_DDATA(blob)) == 0)
        ratio = 0.0;
    else
        ratio = (double)DDATA_TP(BLOB_DDATA(blob)) / (double)DDATA_NPIX(BLOB_DDATA(blob));

    if(ratio < m_minPixelCoverageRatio)
        return false;

    return true;
}

//TO-DO: It can be improved considering crossing-walls, etc...
//For the moment it just verifies that parallelpiped base is not over
//an object if it is a normal solution, and that the base does not overlap
//an object base too much in occlusion case.
bool ReliabilityClassification::not_over_an_object(Parallelpiped *par, bool not_occluded) {
    polygon2D<double> *base_pol = new polygon2D<double>(4), *r = NULL;
    QSharedPointer< polygon2D<double> > object_pol;
    std::deque<world::ContextObject *>::iterator near_iter;
    double int_area, base_area;
    for(int i=0; i<4; i++) {
        POINT_2D_X(POLYGON_NTH_POINT(base_pol, i)) = PARALL_X2D_BASE_i(par, i);
        POINT_2D_Y(POLYGON_NTH_POINT(base_pol, i)) = PARALL_Y2D_BASE_i(par, i);
    }

    base_pol->computeBoundingRectangle();
    
    base_area = base_pol->polygonArea();

    if(not_occluded) {

        for(near_iter = near_objects.begin(); near_iter != near_objects.end(); near_iter++) {
            if(WOBJECT_IS_HOLLOW(*near_iter)) {//If object is hollow, verify just for the base
                if(!WOBJECT_NO_ROOF(*near_iter)) { //Check if the point is over the roof
                    object_pol = WOBJECT_ROOF2D_ON_IMAGE(*near_iter);
	    
                    //For objects with equal trigonometric orientation
                    if(base_pol->counterclockwisePolygon() == object_pol->counterclockwisePolygon()) {
                        r = polygon2D<double>::intersectionPolygon(base_pol, &*object_pol);
                        if(r == NULL)
                            continue;
                        int_area = r->polygonArea();
                        delete r;
                    } else { //If one goes in the other direction, inverse it.
                        polygon2D<double> *inversed = object_pol->getInversedPolygon();
                        r = polygon2D<double>::intersectionPolygon(base_pol, inversed);
                        if(r == NULL) {
                            delete inversed;
                            continue;
                        }
                        int_area = r->polygonArea();
                        delete r;
                        delete inversed;
                    }

                    if(int_area/base_area > m_maxRoofAndBaseIntersectionRatio) { //The object is on the top of the object
                        delete base_pol;
                        return false;
                    }
                }
            } else {
                object_pol = WOBJECT_ROOF2D_ON_IMAGE(*near_iter);

                if(base_pol->counterclockwisePolygon() == object_pol->counterclockwisePolygon()) {
                    r = polygon2D<double>::intersectionPolygon(base_pol, &*object_pol);
                    if(r != NULL) {
                        int_area = r->polygonArea();
                        delete r;
                    } else
                        int_area = 0;
                } else { //If one goes in the other direction, inverse it.
                    polygon2D<double> *inversed = object_pol->getInversedPolygon();
                    r = polygon2D<double>::intersectionPolygon(base_pol, inversed);
                    if(r != NULL) {
                        int_area = r->polygonArea();
                        delete r;
                    } else
                        int_area = 0;
                    delete inversed;
                }
	  
                if(int_area/base_area > m_maxRoofAndBaseIntersectionRatio) { //The object is on the top of the object
                    delete base_pol;
                    return false;
                }
	
                //If still alive, check the base of object
	    
                object_pol = WOBJECT_BASE2D_ON_IMAGE(*near_iter);
	  
                if(base_pol->counterclockwisePolygon() == object_pol->counterclockwisePolygon()) {
                    r = polygon2D<double>::intersectionPolygon(base_pol, &*object_pol);
                    if(r == NULL)
                        continue;
                    int_area = r->polygonArea();
                    delete r;
                } else { //If one goes in the other direction, inverse it.
                    polygon2D<double> *inversed = object_pol->getInversedPolygon();
                    r = polygon2D<double>::intersectionPolygon(base_pol, inversed);
                    if(r == NULL) {
                        delete inversed;
                        continue;
                    }
                    int_area = r->polygonArea();
                    delete r;
                    delete inversed;
                }
	  
                if(int_area/base_area > m_maxBaseAndBaseIntersectionRatio) { //The object is inside the base of an object
                    delete base_pol;
                    return false;
                }
            }
        }
    } else { //For occlusion solutions, just check base for not hollow objects
        for(near_iter = near_objects.begin(); near_iter != near_objects.end(); near_iter++) {
            if(!WOBJECT_IS_HOLLOW(*near_iter)) {
                object_pol = WOBJECT_BASE2D_ON_IMAGE(*near_iter);

                if(base_pol->counterclockwisePolygon() == object_pol->counterclockwisePolygon()) {
                    r = polygon2D<double>::intersectionPolygon(base_pol, &*object_pol);
                    if(r == NULL)
                        continue;
                    int_area = r->polygonArea();
                    delete r;
                } else { //If one goes in the other direction, inverse it.
                    polygon2D<double> *inversed = object_pol->getInversedPolygon();
                    r = polygon2D<double>::intersectionPolygon(base_pol, inversed);
                    if(r == NULL) {
                        delete inversed;
                        continue;
                    }
                    int_area = r->polygonArea();
                    delete r;
                    delete inversed;
                }

                if(int_area/base_area > m_maxBaseAndBaseIntersectionRatio) { //The object is inside the base of an object
                    delete base_pol;
                    return false;
                }
            }
        }
    }
    delete base_pol;

    return true;
}

bool ReliabilityClassification::allowed_base_dimensions(double w, double l) {
    if(    (w >= model_wmin && w <= model_wmax && l >= model_lmin && l <= model_lmax)
        || (w >= model_lmin && w <= model_lmax && l >= model_wmin && l <= model_wmax))
      //    if(w > 0.0001 && l > 0.0001)
        return true;
    return false;
}


//Considering a fixed angle alpha and fixed blob dimensions, search in the space of possible heights h, the best solution, which
//accomplish with the limits imposed by the expected object model and the objects in the scene.
double ReliabilityClassification::search_solution_by_height(Blob *blob, interval_t interval_h_0, interval_t interval_h_90, bool not_occluded) {

    double h, hl, hr, l, w;
    // R0: right angle 0, L0: left angle 0, R90: right angle 90, L90: left angle 90;
    double max_R0branch, max_L0branch, max_R90branch, max_L90branch, Pmax;
    double P_R0branch, P_L0branch, P_R90branch, P_L90branch;
    bool follow_left = true, follow_right = true;
    bool active_R0=true, active_L0=true, active_R90=true, active_L90=true;
    bool ascended_R0=false, ascended_L0=false, ascended_R90=false, ascended_L90=false;
    bool entered_0=false, entered_90=false;
    bool exited_0=false, exited_90=false;
    bool covered, inzoi;
    Interval intersection;
    double
        hmin_0 = INTERVAL_X1(interval_h_0),
        hmin_90 = INTERVAL_X1(interval_h_90),
        hmax_0 = INTERVAL_X2(interval_h_0),
        hmax_90 = INTERVAL_X2(interval_h_90);

    double hmin;
    double hmax;

    if(INTERVAL_IS_NULL(interval_h_0) && INTERVAL_IS_NULL(interval_h_90))
        return 0.0;

    if(INTERVAL_IS_NULL(interval_h_0)) {
        active_R0=false, active_L0=false;
        entered_0=exited_0=true;

        hmin = hmin_90;
        hmax = hmax_90;

        h = (model_hmean >= hmin && model_hmean <= hmax) ? model_hmean : (hmin + hmax)/2.0;
    } else if(INTERVAL_IS_NULL(interval_h_90)) {
        active_R90=false, active_L90=false;
        entered_90=exited_90=true;

        hmin = hmin_0;
        hmax = hmax_0;
        h = (model_hmean >= hmin && model_hmean <= hmax) ? model_hmean : (hmin + hmax)/2.0;
    } else {
        hmin = (hmin_0 < hmin_90) ? hmin_0 : hmin_90;
        hmax = (hmax_0 > hmax_90) ? hmax_0 : hmax_90;
        if(Interval::intersect(&intersection, interval_h_0, interval_h_90)) {
            entered_0=entered_90=true;
            h = model_hmean >= INTERVAL_X1(&intersection) && model_hmean <= INTERVAL_X2(&intersection) ? model_hmean : (INTERVAL_X1(&intersection) + INTERVAL_X2(&intersection))/2.0;
        } else {
            h = (model_hmean >= hmin_0 && model_hmean <= hmax_0) || (model_hmean >= hmin_90 && model_hmean <= hmax_90) ? model_hmean : (hmin_0 + hmax_0)/2.0;
            entered_0=true;
        }
    }

    models_calculated_counter++;
    if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, h) : RC_compute_blob_3Dbbox_simpler(blob, h)) {
        valid_models_counter++;

        //Set w and l given h
        if(Case != 0 || limits[3][3] == 0) {
            RC_compute_w_from_h(h, &w);
            RC_compute_l_from_h(h, &l);
        } else { //Simpler
            w = MM1*h + MM2;
            l = MM3*h + MM4;
        }

        if(l < 0 || w < 0) {
            max_L0branch = max_R0branch = 0.0;
            max_R90branch = max_L90branch = 0;
        } else {
        /*	AppendToLog("Limits:");
	for(int i=0; i<4; i++) {
	  for(int j=0; j<4; j++) 
            AppendToLog(limits[i][j] + " ";
          AppendToLog(std::endl;
	}
        AppendToLog(std::endl;*/

            covered = not_occluded ? true : covering_bbox(blob);
            inzoi   = not_occluded ? true : parallelpipedBaseOnAOIGround(BLOB_3DBBOX(blob), m_context);

            //Check that base of the parallelpiped is inside pertinent walls:
            if(parallelpiped_base_inside_pertinent_walls(blob, w, l) && inzoi && covered && not_over_an_object(BLOB_3DBBOX(blob), not_occluded) ) {
                models_accepted_after_walls_checking++;

                //Store the most probable solution for each blob type, without considering attribute limits (used for static occlusion)
                if(possible_occlusion) {
                    set_best_without_model_limits(blob, w, l, h, alpha);
                    set_best_without_model_limits(blob, l, w, h, alpha + M_PI/2.0);
                }

                if(h >= hmin_0 && h <= hmax_0 && allowed_base_dimensions(w, l))
                    max_L0branch = max_R0branch = set_possible_solution(w, l, h, blob, false, not_occluded);
                else
                    max_L0branch = max_R0branch = 0.0;
	  
                //rotated in 90 degrees
                if(h >= hmin_90 && h <= hmax_90 && allowed_base_dimensions(l, w))
                    max_R90branch = max_L90branch = set_possible_solution(w, l, h, blob, true, not_occluded);
                else //Solution out of bounds
                    max_R90branch = max_L90branch = 0;
            } else { //No initial valid model found
                max_R0branch = max_L0branch = max_R90branch = max_L90branch = 0;
            }
        }
    } else { //No initial valid model found
      max_R0branch = max_L0branch = max_R90branch = max_L90branch = 0;
    }

    for(hr=h + H_STEP_CM, hl=h - H_STEP_CM; hr<=hmax || hl>=hmin; hr+=H_STEP_CM, hl-=H_STEP_CM) {

        if(entered_0) {
            if(hl < hmin_0)
                active_L0 = false;
            if(hr > hmax_0)
                active_R0 = false;
        }

        if(entered_90) {
            if(hl < hmin_90)
                active_L90 = false;
            if(hr > hmax_90)
                active_R90 = false;
        }

        if(!entered_0) {
            if(hr <= hmax_0 && hl >= hmin_0)
                entered_0 = true;
        }

        if(!exited_0 && entered_0) {
            if(!active_L0 && !active_R0)
                exited_0 = true;
        }

        if(!entered_90) {
            if(hr <= hmax_90 && hl >= hmin_90)
                entered_90 = true;
        }

        if(!exited_90 && entered_90) {
            if(!active_L90 && !active_R90)
                exited_90 = true;
        }

        if(exited_0 && exited_90)
            break;

        //Break conditions
        if(!active_R0 && !active_R90)
            follow_right = false;
        if(!active_L0 && !active_L90)
            follow_left = false;
      
        if(follow_right && hr > hmax)
            follow_right = false;
        if(follow_left && hl < hmin)
            follow_left = false;
      
        if(follow_right) { //Process right branch
            models_calculated_counter++;
            if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, hr) : RC_compute_blob_3Dbbox_simpler(blob, hr)) {
                valid_models_counter++;

                //Set w and l given h
                if(Case != 0 || limits[3][3] == 0) {
                    RC_compute_w_from_h(hr, &w);
                    RC_compute_l_from_h(hr, &l);
                } else { //Simpler
                    w = MM1*hr + MM2;
                    l = MM3*hr + MM4;
                }

                if(l < 0 || w < 0) {
                    max_L0branch = max_R0branch = 0.0;
                    max_R90branch = max_L90branch = 0;
                } else {
                    covered = not_occluded ? true : covering_bbox(blob);
                    inzoi   = not_occluded ? true : parallelpipedBaseOnAOIGround(BLOB_3DBBOX(blob), m_context);

                    //Check that base of the parallelpiped is inside pertinent walls:
                    if(parallelpiped_base_inside_pertinent_walls(blob, w, l) && inzoi && covered && not_over_an_object(BLOB_3DBBOX(blob), not_occluded) ) {
                        models_accepted_after_walls_checking++;

                        //Store the most probable solution for each blob type, without considering attribute limits (used for static occlusion)
                        if(possible_occlusion && l > 0 && w > 0) {
                            set_best_without_model_limits(blob, w, l, hr, alpha);
                            set_best_without_model_limits(blob, l, w, hr, alpha + M_PI/2.0);
                        }

                        if(entered_0 && active_R0) { //Non-rotated solution
                            P_R0branch = (allowed_base_dimensions(w, l)) ? set_possible_solution(w, l, hr, blob, false, not_occluded) : 0.0;

                            if(P_R0branch > max_R0branch) { //Check cross or self-validity of sub-branch of rotated angle in 0
                                ascended_R0 = true;
                                active_L0 = false;
                                max_R0branch = P_R0branch;
                            } else if ( l < model_lmin || w < model_wmin || P_R0branch < max_R0branch )
                                active_R0 = false;
                        }

                        if(entered_90 && active_R90) { //Solution rotated in 90 degrees
                            P_R90branch = (allowed_base_dimensions(l, w)) ? set_possible_solution(w, l, hr, blob, true, not_occluded) : 0.0;
                            if(P_R90branch > max_R90branch) { //Check cross or self-validity of sub-branch of rotated angle in 90
                                ascended_R90 = true;
                                active_L90 = false;
                                max_R90branch = P_R90branch;
                            } else if ( w < model_lmin || l < model_wmin || P_R90branch < max_R90branch )
                                active_R90 = false;
                        }
                    }
                }
            }
        }

        if(follow_left) { //Process left branch
            models_calculated_counter++;
            if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, hl) : RC_compute_blob_3Dbbox_simpler(blob, hl)) {
                valid_models_counter++;
	  
                //Set w and l given h
                if(Case != 0 || limits[3][3] == 0) {
                    RC_compute_w_from_h(hl, &w);
                    RC_compute_l_from_h(hl, &l);
                } else { //Simpler
                    w = MM1*hl + MM2;
                    l = MM3*hl + MM4;
                }

                if(l < 0 || w < 0) {
                    max_L0branch = max_R0branch = 0.0;
                    max_R90branch = max_L90branch = 0;
                } else {
                    covered = not_occluded ? true : covering_bbox(blob);
                    inzoi   = not_occluded ? true : parallelpipedBaseOnAOIGround(BLOB_3DBBOX(blob), m_context);

                    //Check that base of the parallelpiped is inside pertinent walls:
                    if(parallelpiped_base_inside_pertinent_walls(blob, w, l) && inzoi && covered && not_over_an_object(BLOB_3DBBOX(blob), not_occluded) ) {
                        models_accepted_after_walls_checking++;
	  
                        //Store the most probable solution for each blob type, without considering attribute limits (used for static occlusion)
                        if(possible_occlusion && l > 0 && w > 0) {
                            set_best_without_model_limits(blob, w, l, hl, alpha);
                            set_best_without_model_limits(blob, l, w, hl, alpha + M_PI/2.0);
                        }

                        if(entered_0 && active_L0) { //Non-rotated solution
                            P_L0branch = (allowed_base_dimensions(w, l)) ? set_possible_solution(w, l, hl, blob, false, not_occluded) : 0.0;

                            if(P_L0branch > max_L0branch) { //Check cross or self-validity of sub-branch of rotated angle in 0
                                ascended_L0 = true;
                                active_R0 = false;
                                max_L0branch = P_L0branch;
                            } else if ( l > model_lmax || w > model_wmax || P_L0branch < max_L0branch )
                                active_L0 = false;
                        }

                        if(entered_90 && active_L90) { //Solution rotated in 90 degrees
                            P_L90branch = (allowed_base_dimensions(l, w)) ? set_possible_solution(w, l, hl, blob, true, not_occluded) : 0;
                            if(P_L90branch > max_L90branch) { //Check cross or self-validity of sub-branch of rotated angle in 90
                                ascended_L90 = true;
                                active_R90 = false;
                                max_L90branch = P_L90branch;
                            } else if ( w > model_lmax || l > model_wmax || P_L90branch < max_L90branch )
                                active_L90 = false;
                        }
                    }
                }
            }
        }
    }
    
    Pmax = (max_R0branch > max_R90branch) ? max_R0branch : max_R90branch;
    Pmax = (Pmax < max_L0branch) ? max_L0branch : Pmax;
    
    return (Pmax < max_L90branch) ? max_L90branch : Pmax;
}

  
//Search for the best occluding or not occluding parallelpiped solutions, searching the optimal for different parallelpiped orientation angles and heights, and
//different 2D blob dimensions in the case of possible occlusion.
void ReliabilityClassification::top_probabilities_and_reliabilities(Blob *blob) {

    Criterion *wm, *lm, *hm;
    double Pmax, res;
    Interval interval_h_0, interval_h_90;

    std::map<ObjectType, QSharedPointer<ModelInterface> >::iterator modelsIt;
    std::map<ObjectSubtype, QSharedPointer<ModelInterface> >::iterator subModelsIt;
    std::map<ObjectSubtype, QSharedPointer<ModelInterface> > *parentModels;
    
    //Save real blob dimensions
    memcpy(&realBBox, BLOB_BBOX(blob), sizeof(Rectangle<int>));

    global_models_calculated_counter = 0;

    //Check best solutions for all models
    for(modelsIt=m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) {
      
        m_modelId = (*modelsIt).first;
        m_currentModel = (*modelsIt).second;
        currentRigid = m_currentModel->IsRigid;

        if(currentRigid)
            there_is_best_valid[m_modelId]=false;
      
        models_calculated_counter = 0;
        valid_models_counter = 0;
        models_accepted_after_walls_checking = 0;

        if(sizeOkForAnalysis[m_modelId]) {

            BLOB_TYPE(blob) = m_modelId;

            if(!currentRigid) {
                parentModels = &(m_currentModel->m_mapPostures);
                subModelsIt = parentModels->begin();
            }

            do {

                if(!currentRigid) {
                    m_subModelId = (*subModelsIt).first;
                    m_currentModel = (*subModelsIt).second;
                    sub_there_is_best_valid[m_modelId][m_subModelId]=false;
                    if(!subSizeOkForAnalysis[m_modelId][m_subModelId]) { //if blob size is not adequate for subModel, check the next one
                        subModelsIt++;
                        if(subModelsIt == parentModels->end())
                            break;
                        continue;
                    }
                    BLOB_SUBTYPE(blob) = m_subModelId;
                } else
                    BLOB_SUBTYPE(blob) = ST_NO_SUBTYPES;

                std::map<QString, Criterion> *criterias = &(m_currentModel->m_mapCriteria);
                std::map<QString, Criterion>::iterator criteriasIt;

                for(criteriasIt = criterias->begin(); criteriasIt!=criterias->end(); criteriasIt++) {

                if((*criteriasIt).first == "height")
                    hm =&(*criteriasIt).second;
                else if((*criteriasIt).first == "width")
                    wm =&(*criteriasIt).second;
                else if((*criteriasIt).first == "depth")
                    lm =&(*criteriasIt).second;
                }
	  
                model_hmin  = hm->spFunction->getMin();
                model_hmax  = hm->spFunction->getMax();
                model_hmean = hm->spFunction->getMean();
                model_lmin  = lm->spFunction->getMin();
                model_lmax  = lm->spFunction->getMax();
                model_lmean = lm->spFunction->getMean();
                model_wmin  = wm->spFunction->getMin();
                model_wmax  = wm->spFunction->getMax();
                model_wmean = wm->spFunction->getMean();

                Interval::newInterval(&interval_modelh, model_hmin, model_hmax);
                Interval::newInterval(&interval_modelw, model_wmin, model_wmax);
                Interval::newInterval(&interval_modell, model_lmin, model_lmax);

                Pmax = 0.0;

                if(aoi_in) { //Do not classify instances which will remain out of the ZOI
                    //Check for all possible orientations of the parallelpiped
                    for(alpha = beta; beta_direction*alpha < beta_direction*(beta + M_PI/2.0) ; alpha += beta_direction*ALPHA_STEP){
	      
                        //Inititialize the structures for normal case:
                        Parallelpiped::initLimits(limits, nlimits, varlimrel);
                        RC_set_3D_bbox_initial_alpha_level_data(alpha);

                        //Get information about the configuration of parallelpipeds to be generated
                        models_calculated_counter++;
                        if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0 ) // If non treatable degenerate case
                            continue;

                        //The new solution is only useful for a strict configuration for the moment
                        if(Case != 0 || limits[3][3] == 0) {
                            RC_set_3D_bbox_alpha_level_data();
                            //Calculate the valid intervals for 3D height h. If all are null, we check the next orientation alpha.
                            if(!calculate_hintervals( &interval_h_0, &interval_h_90))
                                continue;
                            with_sol_counter = 0;
                            //Search the solution at different valid heights, for two different orientations (parallelpiped have the same shape inverting the dimensions w and l and
                            //changing the orientation angle alpha = alpha + pi/2 )
                            if( (res = search_solution_by_height(blob, &interval_h_0, &interval_h_90, true)) > Pmax )
                                Pmax = res;
                        } else { //Use the new method for normal cases (TO-DO!!!! Get special cases for new method)

                            if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
                                continue;

                            with_sol_counter = 0;
                            if( (res = search_solution_by_height(blob, &interval_h_0, &interval_h_90, true)) > Pmax )
                                Pmax = res;
                        }
                    }  // end alpha for
	    
                    if(m_orderByDensity)
                        putBestByPixels(false);

                    global_models_calculated_counter += models_calculated_counter;
                }
                //If occlusion is possible and there wasn't lots of problems of collisions with walls in the scene, we search for best occluding solutions
                if(possible_occlusion) {// && Pmax < 0.98) {
                    //Set the growth limits for a blob in the four directions in the image (LEFT, RIGHT, TOP, BOTTOM).
                    adjustBlobLimitsByOcclusion(blob);
	
                    if(m_treatWallCoherence) {
                        //Set wall segments to be analyzed (on the floor) when checking coherence of the base of 3D parallelpiped, for not extended blob
                        near_objects_extended.clear();
                        setMaxBlobNearObjects();
                        std::deque<world::ContextObject *>::iterator objects_it;
	  
                        annoyingWallSegments.clear();
	  
                        //Store all possibly colliding walls for the maximal possible blob according to the obtained growth limits.
                        for(objects_it = near_objects_extended.begin(); objects_it != near_objects_extended.end(); objects_it++)
                            setAnnoyingWallsForContextObject(blob, *objects_it, false);

                        setAnnoyingWallsForContextWalls(blob, false);
                    }

                    //search for the occluded solutions
                    fillOcclusionListWithStartingPoints2(blob);
                    //fillOcclusionListWithStartingPoints(blob);
                    //fillOcclusionList(blob);
	    
                    //Order best occlusion solutions by density
                    if(m_orderByDensity)
                        putBestByPixels(true);
                }

                if(!currentRigid)
                    subModelsIt++;

            } while(!currentRigid && subModelsIt != parentModels->end());

        } //OkForAnalysis
#ifdef RC_OUTPUT_2
      AppendToLog("\n\tNUMBER OF CALCULATED parallelpipedS:" + global_models_calculated_counter);
#endif	
    }  // end models for
}
  
//This function orders the top solutions lists in terms of the number of moving pixels corresponding to the generated parallelpiped.
void ReliabilityClassification::putBestByPixels(bool occlusion) {
    int list_length = currentRigid ? 
        (occlusion ? m_ntopocc[m_modelId] : m_ntop[m_modelId]) 
      : (occlusion ? m_sub_ntopocc[m_modelId][m_subModelId] : m_sub_ntop[m_modelId][m_subModelId]);
    if(list_length <= 1)
        return;

    int i;
    std::deque<Shape3DData *> *list = currentRigid ?
        (occlusion ? &m_topocc[m_modelId] : &m_top[m_modelId])
      : (occlusion ? &m_sub_topocc[m_modelId][m_subModelId] : &m_sub_top[m_modelId][m_subModelId]);
    std::deque<Shape3DData *>::iterator it = list->begin(), best_it = it;
    Shape3DData *best = *it;
    double current_value, best_value = densityDistance(S3D_DDATA(best));
    bool changed = false;
#ifdef RC_OUTPUT_1
    AppendToLog("i:%d\tv:%.8f\tP:%.6f\tw:%.2f\tl:%.2f\th:%.2f\ta:%.2f\n",0,best_value,S3D_P(list),S3D_W(list),S3D_L(list),S3D_H(list),S3D_ALPHA(list)*180.0/M_PI);
#endif

    for(i = 1, it++; i < list_length; i++, it++) {
        current_value = densityDistance(S3D_DDATA(*it));
#ifdef RC_OUTPUT_1
      AppendToLog("i:%d\tv:%.8f\tP:%.6f\tw:%.2f\tl:%.2f\th:%.2f\ta:%.2f\n",i,current_value,S3D_P(current),S3D_W(current),S3D_L(current),S3D_H(current),S3D_ALPHA(current)*180.0/M_PI);
#endif
      if(current_value > best_value) {
            best_value = current_value;
            best = *it;
            best_it = it;
            changed = true;
      }
    }

    if(changed) { //The first of the list is not best in terms of pixel density
        if(currentRigid) {
            if(occlusion) {
                m_topocc[m_modelId].erase(best_it);
                m_topocc[m_modelId].push_front(best);
            } else {
                m_top[m_modelId].erase(best_it);
                m_top[m_modelId].push_front(best);
            }
        } else {
            if(occlusion) {
                m_sub_topocc[m_modelId][m_subModelId].erase(best_it);
                m_sub_topocc[m_modelId][m_subModelId].push_front(best);
            } else {
                m_sub_top[m_modelId][m_subModelId].erase(best_it);
                m_sub_top[m_modelId][m_subModelId].push_front(best);
            }
        }
    }  
}
  
//Calculates the criteria of comparison between solutions. It corresponds to True positives (moving points inside the parallelpiped) + True negatives (not moving
//points out of the parallelpiped), divided by the total number of analyzed pixels.
double ReliabilityClassification::densityDistance(ddata_t ddata) {
    int TP = DDATA_TP(ddata);
    //    int FN = DDATA_FN(ddata);
    int TN = DDATA_TN(ddata);
    //double norm = sqrt(TP*TP + FN*FN);
    return (TP+TN)/(double)DDATA_NPIX(ddata);//((TP - FN)/norm + 1.0)/2.0; 
}


  //delete entries in classification map for the given blob (for Split/Merge Step)
void ReliabilityClassification::removeBlob(Blob *i_blob) {
    m_mapClassif.erase(BLOB_IDENT(i_blob));
}
  

SpModelInterface ReliabilityClassification::getModelFromType(ObjectType i_type) {
    return m_mapModels[i_type];
}

void ReliabilityClassification::addModel(ObjectType i_blobType, SpModelInterface i_spModel) {
    //add the model to the model class map
    m_mapModels[i_blobType] = i_spModel;
}

ReliabilityClassification::~ReliabilityClassification() {
    free_top();
}

void ReliabilityClassification::updateScoreForType(Blob *i_blob1, ObjectType i_blobType, double i_score) {
    m_mapClassif[BLOB_IDENT(i_blob1)][i_blobType] = i_score;
}

void ReliabilityClassification::init_data() {
    top_init();
    init_pertinent_occlusion_walls();
}

  //Top classification map management
void ReliabilityClassification::top_init() {
    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
    std::map<ObjectSubtype, SpModelInterface> *parentModels;

    ObjectType type;
    SpModelInterface objectModel; 
    ObjectSubtype subtype;
    bool rigid;

    Shape3DData *news3d, *news3docc, *newbest;
    int i;

    for(modelsIt=m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) {
        type = (*modelsIt).first;
        objectModel = (*modelsIt).second;
        rigid = objectModel->IsRigid;

        if(!rigid) {
            parentModels = &(objectModel->m_mapPostures);
            subModelsIt = parentModels->begin();
        }

        do {
            newbest = new Shape3DData();
            if(rigid) {
                m_ntop[type] = 0;
                m_ntopocc[type] = 0;
                best_valid[type] = newbest;
            } else {
                subtype = (*subModelsIt).first;
                m_sub_ntop[type][subtype] = 0;
                m_sub_ntopocc[type][subtype] = 0;
                sub_best_valid[type][subtype] = newbest;
                subModelsIt++;
            }
            for(i = 0; i < m_rcntop; i++) {
                news3d = new Shape3DData();
                news3docc = new Shape3DData();
                S3D_P(newbest) = - 1.0;
                if(rigid) {
                    m_top[type].push_back(news3d);
                    m_topocc[type].push_back(news3docc);
                } else {
                    m_sub_top[type][subtype].push_back(news3d);
                    m_sub_topocc[type][subtype].push_back(news3docc);
                }
            }
        } while(!rigid && subModelsIt != parentModels->end());
    }
}

  //Classify the context walls according to their position with respect to blob.
void ReliabilityClassification::init_pertinent_occlusion_walls() {
    std::vector< QSharedPointer<world::Wall2D> >::iterator it, it_end = m_context->wall2Ds.end();
    world::Wall2D *current_wall;
    m_pertinent_walls.clear();
    
    for(it = m_context->wall2Ds.begin(); it != it_end; it++) {
        current_wall = &**it;
        if(WALL2D_OCC_PERT(current_wall) & MM_CAM_OCCL_BOTTOM)
            m_pertinent_walls.insert(std::make_pair(MM_CAM_OCCL_BOTTOM, current_wall));
      
        if(WALL2D_OCC_PERT(current_wall) & MM_CAM_OCCL_TOP)
            m_pertinent_walls.insert(std::make_pair(MM_CAM_OCCL_TOP, current_wall));

        if(WALL2D_OCC_PERT(current_wall) & MM_CAM_OCCL_LEFT)
            m_pertinent_walls.insert(std::make_pair(MM_CAM_OCCL_LEFT, current_wall));

        if(WALL2D_OCC_PERT(current_wall) & MM_CAM_OCCL_RIGHT)
            m_pertinent_walls.insert(std::make_pair(MM_CAM_OCCL_RIGHT, current_wall));
    }

    std::multimap<DetectionProblemType, world::Wall2D *>::iterator pos, last;

    last = m_pertinent_walls.upper_bound(MM_CAM_OCCL_BOTTOM);
    for (pos = m_pertinent_walls.lower_bound(MM_CAM_OCCL_BOTTOM); pos != last; pos++)
        AppendToLog("PERTINENT BOTTOM WALL:\t" + (*pos->second).name);

    last = m_pertinent_walls.upper_bound(MM_CAM_OCCL_TOP);
    for (pos = m_pertinent_walls.lower_bound(MM_CAM_OCCL_TOP); pos != last; pos++)
        AppendToLog("PERTINENT TOP WALL:\t" + (*pos->second).name);

    last = m_pertinent_walls.upper_bound(MM_CAM_OCCL_LEFT);
    for (pos = m_pertinent_walls.lower_bound(MM_CAM_OCCL_LEFT); pos != last; pos++)
        AppendToLog("PERTINENT LEFT WALL:\t" + (*pos->second).name);

    last = m_pertinent_walls.upper_bound(MM_CAM_OCCL_RIGHT);
    for (pos = m_pertinent_walls.lower_bound(MM_CAM_OCCL_RIGHT); pos != last; pos++)
        AppendToLog("PERTINENT RIGHT WALL:\t" + (*pos->second).name);
}

void ReliabilityClassification::reset_top() {
    std::map<ObjectType, int>::iterator ntopIt;
    std::map<ObjectType, std::map<ObjectSubtype, int> >::iterator s_ntopIt;
    std::map<ObjectSubtype, int>::iterator sub_ntopIt;
    
    for(ntopIt=m_ntop.begin(); ntopIt!=m_ntop.end(); ntopIt++) {
        S3D_TYPE(m_top[(*ntopIt).first][0])=UNKNOWN;
        (*ntopIt).second=0;
    }

    for(ntopIt=m_ntopocc.begin(); ntopIt!=m_ntopocc.end(); ntopIt++) {
        S3D_TYPE(m_topocc[(*ntopIt).first][0])=UNKNOWN;
        (*ntopIt).second=0;
    }

    for(s_ntopIt=m_sub_ntop.begin(); s_ntopIt!=m_sub_ntop.end(); s_ntopIt++)
        for(sub_ntopIt=((*s_ntopIt).second).begin(); sub_ntopIt!=((*s_ntopIt).second).end(); sub_ntopIt++) {
            S3D_TYPE(m_sub_top[(*s_ntopIt).first][(*sub_ntopIt).first][0])=UNKNOWN;
            (*sub_ntopIt).second=0;
        }

    for(s_ntopIt=m_sub_ntopocc.begin(); s_ntopIt!=m_sub_ntopocc.end(); s_ntopIt++)
        for(sub_ntopIt=((*s_ntopIt).second).begin(); sub_ntopIt!=((*s_ntopIt).second).end(); sub_ntopIt++) {
            S3D_TYPE(m_sub_topocc[(*s_ntopIt).first][(*sub_ntopIt).first][0])=UNKNOWN;
            (*sub_ntopIt).second=0;
        }

}
  
void ReliabilityClassification::free_top() {
    std::map<ObjectType, std::deque<Shape3DData *> >::iterator topIt;
    std::map<ObjectType, std::map<ObjectSubtype, std::deque<Shape3DData *> > >::iterator sTopIt;
    std::map<ObjectSubtype, std::deque<Shape3DData *> >::iterator subTopIt;
    std::deque<Shape3DData *>::iterator it;

    for(topIt = m_top.begin(); topIt != m_top.end(); topIt++)
        for(it = topIt->second.begin(); it != topIt->second.end(); it++)
            delete *it;

    for(topIt = m_topocc.begin(); topIt != m_topocc.end(); topIt++)
        for(it = topIt->second.begin(); it != topIt->second.end(); it++)
            delete *it;

    for(sTopIt=m_sub_top.begin(); sTopIt!=m_sub_top.end(); sTopIt++) {
        for(subTopIt=((*sTopIt).second).begin(); subTopIt!=((*sTopIt).second).end(); subTopIt++)
            for(it = subTopIt->second.begin(); it != subTopIt->second.end(); it++)
                delete *it;
        ((*sTopIt).second).clear();
    }

    for(sTopIt=m_sub_topocc.begin(); sTopIt!=m_sub_topocc.end(); sTopIt++) {
        for(subTopIt=((*sTopIt).second).begin(); subTopIt!=((*sTopIt).second).end(); subTopIt++)
            for(it = subTopIt->second.begin(); it != subTopIt->second.end(); it++)
                delete *it;
            ((*sTopIt).second).clear();
    }

    m_top.clear();
    m_ntop.clear();

    m_topocc.clear();
    m_ntopocc.clear();

    m_sub_top.clear();
    m_sub_ntop.clear();

    m_sub_topocc.clear();
    m_sub_ntopocc.clear();

}

  //Check if the quality of a solution allows it to enter to the top list.
  bool ReliabilityClassification::can_be_inserted_top (Blob *blob, bool normalList) {
    int i;
    std::deque<Shape3DData *> *list;
    int ntop;
    if(currentRigid) {
      if(normalList) {
        list = &m_top[m_modelId];
	ntop = m_ntop[m_modelId];
      } else {
        list = &m_topocc[m_modelId];
	ntop = m_ntopocc[m_modelId];
      }
    } else {
      if(normalList) {
        list = &m_sub_top[m_modelId][m_subModelId];
	ntop = m_sub_ntop[m_modelId][m_subModelId];
      } else {
        list = &m_sub_topocc[m_modelId][m_subModelId];
	ntop = m_sub_ntopocc[m_modelId][m_subModelId];
      }
    }

    if(ntop < m_rcntop)
        return true;
    
    std::deque<Shape3DData *>::iterator it;

    for(i=0, it = list->begin(); i<ntop; i++, it++) {
        switch(m_rcIntraCriteria) {
            case RCReliability:
                if(S3D_R(*it) < BLOB_R(blob))
                    return true;
                break;
            case RCReliabilityProbability:
                if(S3D_PR(*it) < BLOB_PR(blob))
                    return true;
                break;
            case RCDimensionalProbability:
                if(S3D_DP(*it) < BLOB_DP(blob))
                    return true;
                break;
            case RCProbability:
            default:
                if(S3D_P(*it) < BLOB_P(blob))
                    return true;
          }
    }
    
    return false;
    
}

//Insert a solution in a top list.
Shape3DData *ReliabilityClassification::insert_top(Blob *blob, bool normalList) {
    int i;
    bool inserted = false, valid;
    std::deque<Shape3DData *> *list;
    int *ntop;

    Shape3DData *newtop = NULL;

    if(normalList)
        someNormalSolutionInserted = true;
    else
        someOcclusionSolutionInserted = true;

    if(currentRigid) {
        if(normalList) {
            list = &m_top[m_modelId];
            ntop = &m_ntop[m_modelId];
        } else {
            list = &m_topocc[m_modelId];
            ntop = &m_ntopocc[m_modelId];
        }
    } else {
        if(normalList) {
            list = &m_sub_top[m_modelId][m_subModelId];
            ntop = &m_sub_ntop[m_modelId][m_subModelId];
        } else {
            list = &m_sub_topocc[m_modelId][m_subModelId];
            ntop = &m_sub_ntopocc[m_modelId][m_subModelId];
        }
    }
    
    if(*ntop == 0) {
        (*ntop) = 1;
        newtop = new Shape3DData();
        newtop->copyBlobToShape3DData(blob);
        if(!normalList)
            //ACA
            S3D_DP_TYPE(newtop) = BLOB_DP_TYPE(blob);
        list->push_front(newtop);
        return newtop;
    }

    std::deque<Shape3DData *>::iterator it;

    for(i = 0, it = list->begin(); i < *ntop; i++, it++) {
        valid = false;
        switch(m_rcIntraCriteria) {
            case RCReliability:
                if(S3D_R(*it) < BLOB_R(blob))
                    valid = true;
                break;
            case RCReliabilityProbability:
                if(S3D_PR(*it) < BLOB_PR(blob))
                    valid = true;
                break;
            case RCDimensionalProbability:
                if(S3D_DP(*it) < BLOB_DP(blob))
                    valid = true;
                break;
            case RCProbability:
            default:
                if(S3D_P(*it) < BLOB_P(blob))
                    valid = true;
        }

        if(valid) {
            newtop = new Shape3DData();
            newtop->copyBlobToShape3DData(blob);
            if(!normalList)
                S3D_DP_TYPE(newtop) = BLOB_DP_TYPE(blob);
            list->insert(it, newtop);
            inserted = true;
            break;
        }
    }

    if(!inserted) { //If not inserted yet, it should be the last one, if
                    //there are space
        if(*ntop == m_rcntop) {
            return NULL;
        } else {
            (*ntop)++;
            newtop = new Shape3DData();
            newtop->copyBlobToShape3DData(blob);
            if(!normalList)
                S3D_DP_TYPE(newtop) = BLOB_DP_TYPE(blob);
            list->push_back(newtop);

            return newtop;
        }

    }
    
    if(inserted) { //Erase the last one, if there is no more room
        if(*ntop < m_rcntop) {
            delete list->back();
            list->pop_back();
        } else {
            (*ntop)++;
        }
        return newtop;
    }

    return NULL;
}

void ReliabilityClassification::top_output() {
    int i, size;
    std::map<ObjectType, std::deque<Shape3DData *> >::iterator topIt;
    std::map<ObjectType, std::map<ObjectSubtype, std::deque<Shape3DData *> > >::iterator sTopIt;
    std::map<ObjectSubtype, std::deque<Shape3DData *> >::iterator subTopIt;
    
    std::deque<Shape3DData *> *list;
    std::deque<Shape3DData *>::iterator it;

    AppendToLog("BEST NORMAL RESULTS:");
    AppendToLog("INTRA CRITERIA:\t "+ m_rcIntraCriteria);
    AppendToLog("INTER CRITERIA:\t "+ m_rcInterCriteria);

    for(topIt=m_top.begin(); topIt!=m_top.end(); topIt++) {
        list = &(*topIt).second;
        AppendToLog(" TOP "+ QString::number(m_rcntop) +"\t FOR " + QString(Blob::getNameFromType((*topIt).first).c_str()) + " : ");
        size = m_ntop[(*topIt).first];
        for(i = 0, it = list->begin(); i < size; i++, it++) {
            AppendToLog("\t\tP: " + QString::number(S3D_P(*it)) + "\tPR: " + QString::number(S3D_PR(*it)));
            AppendToLog("\t\tR: " + QString::number(S3D_R(*it)) + "\tDP: " + QString::number(S3D_DP(*it)));
            AppendToLog("\t\tw: " + QString::number(S3D_W(*it)) + "\th: " + QString::number(S3D_H(*it)) + "\tl: " + QString::number(S3D_L(*it)));
            AppendToLog("\t\tRw: " + QString::number(S3D_RW(*it)) + "\tRh: " + QString::number(S3D_RH(*it)) + "\tRl: " + QString::number(S3D_RL(*it)));
            AppendToLog("\t\tPw: " + QString::number(S3D_PW(*it)) + "\tPh: " + QString::number(S3D_PH(*it)) + "\tPl: " + QString::number(S3D_PL(*it)));
            AppendToLog("\t\talpha: " + QString::number(180*(S3D_ALPHA(*it)+beta)/M_PI) + "\n");
        }
    }

    for(sTopIt=m_sub_top.begin(); sTopIt!=m_sub_top.end(); sTopIt++) {
        AppendToLog(" TOP "+ QString::number(m_rcntop) +"\t FOR " + QString(Blob::getNameFromType((*sTopIt).first).c_str()) +" : ");

        for(subTopIt=((*sTopIt).second).begin(); subTopIt!=((*sTopIt).second).end(); subTopIt++) {

            list = &(*subTopIt).second;
            AppendToLog("\t\t FOR SUBTYPE " + QString(Blob::getNameFromSubtype((*subTopIt).first).c_str()) +" : ");
            size = m_sub_ntop[(*sTopIt).first][(*subTopIt).first];
            for(i=0, it = list->begin(); i<size; i++) {
                AppendToLog("\t\t\tP: " + QString::number(S3D_P(*it)) + "\tPR: " + QString::number(S3D_PR(*it)));
                AppendToLog("\t\t\tR: " + QString::number(S3D_R(*it)) + "\tDP: " + QString::number(S3D_DP(*it)));
                AppendToLog("\t\t\tw: " + QString::number(S3D_W(*it)) + "\th: " + QString::number(S3D_H(*it)) + "\tl: " + QString::number(S3D_L(*it)));
                AppendToLog("\t\t\tRw: " + QString::number(S3D_RW(*it)) + "\tRh: " + QString::number(S3D_RH(*it)) + "\tRl: " + QString::number(S3D_RL(*it)));
                AppendToLog("\t\t\tPw: " + QString::number(S3D_PW(*it)) + "\tPh: " + QString::number(S3D_PH(*it)) + "\tPl: " + QString::number(S3D_PL(*it)));
                AppendToLog("\t\t\talpha: " + QString::number(180*(S3D_ALPHA(*it)+beta)/M_PI) + "\n");
            }
        }
    }


    if(possible_occlusion) {
        AppendToLog("BEST OCCLUSSION RESULTS:");
        AppendToLog("INTRA CRITERIA:\t "+ m_rcIntraCriteria);
        AppendToLog("INTER CRITERIA:\t "+ m_rcInterCriteria);

        for(topIt=m_topocc.begin(); topIt!=m_topocc.end(); topIt++) {
            list = &(*topIt).second;
            AppendToLog(" TOP "+ QString::number(m_rcntop) +"\t FOR " + QString(Blob::getNameFromType((*topIt).first).c_str()) +" : ");
            size = m_ntopocc[(*topIt).first];

            for(i = 0, it = list->begin(); i < size; i++) {
                AppendToLog("\t\tP: " + QString::number(S3D_P(*it)) + "\tPR: " + QString::number(S3D_PR(*it)));
                AppendToLog("\t\tR: " + QString::number(S3D_R(*it)) + "\tDP: " + QString::number(S3D_DP(*it)));
                AppendToLog("\t\tw: " + QString::number(S3D_W(*it)) + "\th: " + QString::number(S3D_H(*it)) + "\tl: " + QString::number(S3D_L(*it)));
                AppendToLog("\t\tRw: " + QString::number(S3D_RW(*it)) + "\tRh: " + QString::number(S3D_RH(*it)) + "\tRl: " + QString::number(S3D_RL(*it)));
                AppendToLog("\t\tPw: " + QString::number(S3D_PW(*it)) + "\tPh: " + QString::number(S3D_PH(*it)) + "\tPl: " + QString::number(S3D_PL(*it)));
                AppendToLog("\t\talpha: " + QString::number(180*(S3D_ALPHA(*it)+beta)/M_PI) + "\n");
            }
        }

        for(sTopIt=m_sub_topocc.begin(); sTopIt!=m_sub_topocc.end(); sTopIt++) {
            AppendToLog(" TOP "+ QString::number(m_rcntop) +"\t FOR " + QString(Blob::getNameFromType((*sTopIt).first).c_str()) +" : ");

            for(subTopIt=((*sTopIt).second).begin(); subTopIt!=((*sTopIt).second).end(); subTopIt++) {
                list = &(*subTopIt).second;
                AppendToLog("\t\t FOR SUBTYPE " + QString(Blob::getNameFromSubtype((*subTopIt).first).c_str()) +" : ");
                size = m_sub_ntop[(*sTopIt).first][(*subTopIt).first];
                for(i = 0, it = list->begin(); i < size; i++) {
                    AppendToLog("\t\t\tP: " + QString::number(S3D_P(*it)) + "\tPR: " + QString::number(S3D_PR(*it)));
                    AppendToLog("\t\t\tR: " + QString::number(S3D_R(*it)) + "\tDP: " + QString::number(S3D_DP(*it)));
                    AppendToLog("\t\t\tw: " + QString::number(S3D_W(*it)) + "\th: " + QString::number(S3D_H(*it)) + "\tl: " + QString::number(S3D_L(*it)));
                    AppendToLog("\t\t\tRw: " + QString::number(S3D_RW(*it)) + "\tRh: " + QString::number(S3D_RH(*it)) + "\tRl: " + QString::number(S3D_RL(*it)));
                    AppendToLog("\t\t\tPw: " + QString::number(S3D_PW(*it)) + "\tPh: " + QString::number(S3D_PH(*it)) + "\tPl: " + QString::number(S3D_PL(*it)));
                    AppendToLog("\t\t\talpha: " + QString::number(180*(S3D_ALPHA(*it)+beta)/M_PI) + "\n");
                }
            }
        }
    }
}

bool ReliabilityClassification::positiveAngleDirection(double beta, double angle) {

    if(beta < 0 && angle < 0) 
        return angle < beta ? true : false;

    if(beta > 0 && angle < 0) {
        double angle2 = 2*M_PI + angle;
        return angle2 - beta < M_PI ? true : false;
    }

    if(beta < 0 && angle > 0) {
        double beta2 = 2*M_PI + beta;
        return beta2 - angle < M_PI ? false : true;
    }

    return angle >= beta ? true : false;

}

//Set all pertinent calculations that won't change for the given blob 2D dimensions
void ReliabilityClassification::RC_set_3D_bbox_blob_level_data(Blob *blob) {
    int i,j;
    double *pp = (double *)SM_CALIB_MATRIX(m_context);
    double x0, y0, x1, y1;
    double angle;

    //Store ordered blob limits
    BlobLimits[0] = BLOB_XLEFT  (blob);
    BlobLimits[1] = BLOB_YBOTTOM(blob);
    BlobLimits[2] = BLOB_XRIGHT (blob);
    BlobLimits[3] = BLOB_YTOP   (blob);

    //Set blob level constants for equations
    for(i=0;i<4;i++)
        for(j=0;j<4;j++)
            V[i][j] = pp[(i%2)*4 + j] - BlobLimits[i]*pp[8 + j]; //Now V3,V6,V9,V12 will be of type Vn = Vn1*h + Vn2
    Vl0 =  V[0][0];
    Vl1 =  V[0][1];
    Vb0 =  V[1][0];
    Vb1 =  V[1][1];
    Vr0 =  V[2][0];
    Vr1 =  V[2][1];
    Vt0 =  V[3][0];
    Vt1 =  V[3][1];

    K[0]  = (Vr0*Vt0 - Vr1*Vt1);
    K[1]  = Vl1*Vt0*Vr1;
    K[2]  = (Vr0*Vt1 + Vr1*Vt0);
    K[3]  = (Vr0*Vt1 - Vr1*Vt0);
    K[4]  = (Vl0*Vt0 - Vl1*Vt1);
    K[5]  = (Vl0*Vt1 + Vl1*Vt0);
    K[6]  = (Vl0*Vr0 - Vl1*Vr1);
    K[7]  = (Vl0*Vr1 + Vl1*Vr0);
    K[8]  = (Vl0*Vb1*Vr0*Vt1 - Vl1*Vb0*Vr1*Vt0);
    K[9] = (Vb0*Vr1 - Vb1*Vr0);
    K[10] = (Vb0*Vr0 - Vb1*Vr1);
    K[11] = (Vl0*Vt1 - Vl1*Vt0);
    K[12] = (Vl0*Vb0 - Vb1*Vl1);
    K[13] = (Vl0*Vb1 - Vl1*Vb0);

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XLEFT(blob),   BLOB_YCENTER(blob), 0, &x0, &y0);
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XRIGHT(blob),  BLOB_YCENTER(blob), 0, &x1, &y1);
    
    if(x1 - x0 == 0) {
        if(y1 - y0 > 0)
            beta =   M_PI/2;
        else
            beta = - M_PI/2;
    } else
        beta = atan2(y1 - y0,x1 - x0);
    BLOB_BETA(blob) = beta;

    //Calculate 45 degrees in image plane angle, to understand the sense of the growth of beta in the xy world plane
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XLEFT(blob) + 5,  BLOB_YCENTER(blob) - 5, 0, &x1, &y1);
    if(x1 - x0 == 0) {
        if(y1 - y0 > 0)
            angle =   M_PI/2;
        else
            angle = - M_PI/2;
    } else
        angle = atan2(y1 - y0, x1 - x0);

    if(positiveAngleDirection(beta, angle))
        BLOB_BETA_DIRECTION(blob) = beta_direction = +1;
    else
        BLOB_BETA_DIRECTION(blob) = beta_direction = -1;
    //Problems getting the right angle difference
    BLOB_BETA_DIRECTION(blob) = beta_direction = +1;

    /* Positions: (Position 4 is Middle-Center and correspond when blob is absolutely between focal-point and ground)
       
               Left-Front(0)     Middle-Front(1)    Right-Front(2)
                            \           |          /
                               \        |       /
                                  \     |    /
               Left-Center(3)<-----Focal Point----->Right-Center(5) 
                                  /     |    \
                               /        |       \
                            /           |          \
	       Left-Back(6)       Middle-Back(7)    Right-Back(8)
    */
    position = blob->getPositionRelativeToCamera(m_context);
    BLOB_POSITION(blob) = position;

    blob->setInHeightGivenPosition(position, (int *)in_height);

    ml0 = V[0][2]*in_height[0]; ml1 = V[0][3]; 
    mb0 = V[1][2]*in_height[1]; mb1 = V[1][3]; 
    mr0 = V[2][2]*in_height[2]; mr1 = V[2][3]; 
    mt0 = V[3][2]*in_height[3]; mt1 = V[3][3]; 
    
    K[14] = Vr1*mt0 - mr0*Vt1;
    K[15] = Vl1*mb0 - ml0*Vb1;
    K[16] = Vr1*mt1 - mr1*Vt1;
    K[17] = Vl1*mb1 - ml1*Vb1;
    K[18] = Vb1*mr0 - mb0*Vr1;
    K[19] = Vl1*mt0 - ml0*Vt1;
    K[20] = Vb1*mr1 - Vr1*mb1;
    K[21] = Vl1*mt1 - ml1*Vt1;

    LL1  = K[0]*K[13] + K[3]*K[12];
    LL2  = K[13]*(Vr0*mt0 - mr0*Vt0) - K[3]* (Vl0*mb0 - ml0*Vb0);
    LL3  = K[13]*K[14] - K[3]*K[15];
    LL4  = K[13]*(Vr0*mt1 - mr1*Vt0) - K[3]* (Vl0*mb1 - ml1*Vb0);
    LL5  = K[13]*K[16] - K[3]*K[17];
    LL6  = K[9] *(Vl0*mt0 - ml0*Vt0) - K[11]*(Vb0*mr0 - mb0*Vr0);
    LL7  = K[11]*K[18] - K[9]*K[19];
    LL8  = K[9] *(Vl0*mt1 - ml1*Vt0) - K[11]*(Vb0*mr1 - mb1*Vr0);
    LL9  = K[11]*K[20] - K[9]*K[21];
    LL10 = K[18]*Vl0*Vt1 - K[19]*Vb0*Vr1;
    LL11 = K[14]*Vl1*Vb0 + K[15]*Vr0*Vt1;
    LL12 = (K[6]*mb0 - K[10]*ml0 - K[12]*mr0)*Vt1 + (K[10]*Vl1 + K[9]*Vl0)*mt0;
    LL13 = K[20]*Vl0*Vt1 - K[21]*Vb0*Vr1;
    LL14 = K[16]*Vl1*Vb0 + K[17]*Vr0*Vt1;
    LL15 = (K[6]*mb1 - K[10]*ml1 - K[12]*mr1)*Vt1 + (K[10]*Vl1 + K[9]*Vl0)*mt1;

}


//Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants, without knowing which Case of parallelpiped
//solutions can be obtained.
void ReliabilityClassification::RC_set_3D_bbox_initial_alpha_level_data(double alpha) {
    double c1, c2, c3, c4;

    sina = sin(alpha);
    cosa = cos(alpha); 
    
    //Auxiliar values
    c1 = (K[0]*sina - K[2]*cosa)*cosa;
    c2 = (K[4]*sina - K[5]*cosa)*cosa;
    c3 = (K[6]*sina - K[7]*cosa)*cosa;
    c4 =  K[8]*(2*cosa*cosa - 1);

    L[1] = ((2*K[1]*cosa - (K[0]*Vl1 - K[3]*Vl0)*sina)*cosa - K[1]);

    if(eq_relations[varlimrel[1]][varlimrel[2]] == 1) { //FOR TYPE 1 INITIAL EQUATION OF NORMAL CASE  
        L[0] =   Vb1*(c1 + Vr1*Vt0);
        L[2] =   Vb1*(c2 + Vl0*Vt1);
        L[3] = - Vb1*(c3 + Vl0*Vr1);
        L[4] = c4 + (K[4]*K[9] - K[10]*K[11])*sina*cosa;
    } else { //FOR TYPE 2 INITIAL EQUATION OF NORMAL CASE  
        L[0] =   Vb1*(c1 + Vr0*Vt1);
        L[2] =   Vb1*(c2 + Vl1*Vt0);
        L[3] = - Vb1*(c3 + Vl1*Vr0);
        L[4] = c4 - (K[3]*K[12] + K[0]*K[13])*sina*cosa;
    }

    //Initial constants considering h=0
    //Auxiliar values for w and l in function of h;
    C[0] = C[2] = C[4] = C[6] = C[8] = C[10] = C[12] = C[14] = 0;

    C[5] = (L[0]*ml1 + L[1]*mb1 + L[2]*mr1 + L[3]*mt1)/L[4];
    C[7] = - (C[5]*Vb0 + mb1) / Vb1;

    if(eq_relations[varlimrel[1]][varlimrel[2]] == 1) { //FOR TYPE 1 INITIAL EQUATION OF NORMAL CASE  
        C[1] = ( Vl1*C[5]*cosa + (ml1 + Vl1*C[7])*sina ) / (Vl1*cosa - Vl0*sina);
        C[9] = ( Vr1*C[5]*sina - (mr1 + Vr1*C[7])*cosa ) / (Vr0*cosa + Vr1*sina);
    } else {
        C[1] = ( Vl1*C[5]*sina - (ml1 + Vl1*C[7])*cosa ) / (Vl0*cosa + Vl1*sina);
        C[9] = ( Vr1*C[5]*cosa + (mr1 + Vr1*C[7])*sina ) / (Vr1*cosa - Vr0*sina);
    }

    C[3] = - (C[1]*Vl0 + ml1) / Vl1;
    C[11]  = - (C[9]*Vr0 + mr1) / Vr1;

    if(eq_relations[varlimrel[1]][varlimrel[2]] == 1) //FOR TYPE 1 INITIAL EQUATION OF NORMAL CASE  
        C[13] = ( Vt1*C[9]*cosa + (mt1 + Vt1*C[11])*sina ) / (Vt1*cosa - Vt0*sina);
    else 
        C[13] = ( Vt1*C[9]*sina - (mt1 + Vt1*C[11])*cosa ) / (Vt0*cosa + Vt1*sina);

    C[15] = -(C[13]*Vt0 + mt1) / Vt1;

}

//Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants knowing the type (Case) of parallelpiped
//solutions that can be obtained. Normally a parallelpiped has one vertex point on each blob dimension, but there are some degenerate cases
//with some vertexes in more than one blob dimension.
void ReliabilityClassification::RC_set_3D_bbox_alpha_level_data() {
    double aux;
    int aux_ind, aux_ind2, aux_ind3;
    if(Case == 0) { //Normal Case
        //Auxiliar values for w and l in function of h;
        aux_ind = varlimrel[1]*4;
        C[aux_ind    ] = (L[0]*ml0 + L[1]*mb0 + L[2]*mr0 + L[3]*mt0)/L[4];
        C[aux_ind + 1] = (L[0]*ml1 + L[1]*mb1 + L[2]*mr1 + L[3]*mt1)/L[4];
        C[aux_ind + 2] = - (C[aux_ind    ]*Vb0 + mb0) / Vb1;
        C[aux_ind + 3] = - (C[aux_ind + 1]*Vb0 + mb1) / Vb1;

        aux_ind2 = varlimrel[0]*4;
        aux_ind3 = varlimrel[2]*4;

        if(eq_relations[varlimrel[1]][varlimrel[2]] == 1) { //FOR TYPE 1 INITIAL EQUATION OF NORMAL CASE
            aux = Vl1*cosa - Vl0*sina;
            C[aux_ind2    ] = ( Vl1*C[aux_ind    ]*cosa + (ml0 + Vl1*C[aux_ind + 2])*sina ) / aux;
            C[aux_ind2 + 1] = ( Vl1*C[aux_ind + 1]*cosa + (ml1 + Vl1*C[aux_ind + 3])*sina ) / aux;
            aux = Vr0*cosa + Vr1*sina;
            C[aux_ind3    ] = ( Vr1*C[aux_ind    ]*sina - (mr0 + Vr1*C[aux_ind + 2])*cosa ) / aux;
            C[aux_ind3 + 1] = ( Vr1*C[aux_ind + 1]*sina - (mr1 + Vr1*C[aux_ind + 3])*cosa ) / aux;
        } else {
            aux = Vl0*cosa + Vl1*sina;
            C[aux_ind2    ] = ( Vl1*C[aux_ind    ]*sina - (ml0 + Vl1*C[aux_ind + 2])*cosa ) / aux;
            C[aux_ind2 + 1] = ( Vl1*C[aux_ind + 1]*sina - (ml1 + Vl1*C[aux_ind + 3])*cosa ) / aux;
            aux = Vr1*cosa - Vr0*sina;
            C[aux_ind3    ] = ( Vr1*C[aux_ind    ]*cosa + (mr0 + Vr1*C[aux_ind + 2])*sina ) / aux;
            C[aux_ind3 + 1] = ( Vr1*C[aux_ind + 1]*cosa + (mr1 + Vr1*C[aux_ind + 3])*sina ) / aux;
        }

        C[aux_ind2 + 2] = - (C[aux_ind2    ]*Vl0 + ml0) / Vl1;
        C[aux_ind2 + 3] = - (C[aux_ind2 + 1]*Vl0 + ml1) / Vl1;

        C[aux_ind3 + 2] = - (C[aux_ind3    ]*Vr0 + mr0) / Vr1;
        C[aux_ind3 + 3] = - (C[aux_ind3 + 1]*Vr0 + mr1) / Vr1;

        aux_ind = varlimrel[3]*4;
        if(eq_relations[varlimrel[1]][varlimrel[2]] == 1) { //FOR TYPE 1 INITIAL EQUATION OF NORMAL CASE
            aux = Vt1*cosa - Vt0*sina;
            C[aux_ind    ] = ( Vt1*C[aux_ind3    ]*cosa + (mt0 + Vt1*C[aux_ind3 + 2])*sina ) / aux;
            C[aux_ind + 1] = ( Vt1*C[aux_ind3 + 1]*cosa + (mt1 + Vt1*C[aux_ind3 + 3])*sina ) / aux;
        } else {
            aux = Vt0*cosa + Vt1*sina;
            C[aux_ind    ] = ( Vt1*C[aux_ind3    ]*sina - (mt0 + Vt1*C[aux_ind3 + 2])*cosa ) / aux;
            C[aux_ind + 1] = ( Vt1*C[aux_ind3 + 1]*sina - (mt1 + Vt1*C[aux_ind3 + 3])*cosa ) / aux;
        }

        C[aux_ind + 2] = - (C[aux_ind    ]*Vt0 + mt0) / Vt1;
        C[aux_ind + 3] = - (C[aux_ind + 1]*Vt0 + mt1) / Vt1;
 
    } else { //Special cases
        double
            Va0 =  V[a][0],
            Va1 =  V[a][1],
            mA0 =  V[a][2]*in_height[a],
            mA1 =  V[a][3],
            Vb0 =  V[b][0],
            Vb1 =  V[b][1],
            mB0 =  V[b][2]*in_height[b],
            mB1 =  V[b][3],
            Vc0 =  V[c][0],
            Vc1 =  V[c][1],
            mC0 =  V[c][2]*in_height[c],
            mC1 =  V[c][3],
            Vd0 =  V[d][0],
            Vd1 =  V[d][1],
            mD0 =  V[d][2]*in_height[d],
            mD1 =  V[d][3];
	
        aux = Va0*Vb1 - Va1*Vb0;
      
        C[sp_var*4    ] =   (Va1*mB0 - mA0*Vb1)/aux;
        C[sp_var*4 + 1] =   (Va1*mB1 - mA1*Vb1)/aux;

        C[sp_var*4 + 2] = - (Va0*mB0 - mA0*Vb0)/aux;
        C[sp_var*4 + 3] = - (Va0*mB1 - mA1*Vb0)/aux;

        if (Case < 3) {
            if (eq_relations[sp_var][(sp_var+1)%4] == 1) {
                aux = Vc0*cosa + Vc1*sina;
                C[((sp_var+1)%4)*4    ] =   (Vc1*C[sp_var*4    ]*sina - (mC0 + Vc1*C[sp_var*4 + 2])*cosa)/aux;
                C[((sp_var+1)%4)*4 + 1] =   (Vc1*C[sp_var*4 + 1]*sina - (mC1 + Vc1*C[sp_var*4 + 3])*cosa)/aux;
            } else {
                aux = Vc1*cosa - Vc0*sina;
                C[((sp_var+1)%4)*4    ] =   (Vc1*C[sp_var*4    ]*cosa + (mC0 + Vc1*C[sp_var*4 + 2])*sina)/aux;
                C[((sp_var+1)%4)*4 + 1] =   (Vc1*C[sp_var*4 + 1]*cosa + (mC1 + Vc1*C[sp_var*4 + 3])*sina)/aux;
            }

            C[((sp_var+1)%4)*4 + 2] = - (C[((sp_var+1)%4)*4    ]*Vc0 + mC0)/Vc1;
            C[((sp_var+1)%4)*4 + 3] = - (C[((sp_var+1)%4)*4 + 1]*Vc0 + mC1)/Vc1;
      
            if (Case == 2) {
                if (eq_relations[sp_var][(sp_var+3)%4] == 1) {
                    aux = Vd0*cosa + Vd1*sina;
                    C[((sp_var+3)%4)*4    ] =   (Vd1*C[sp_var*4    ]*sina - (mD0 + Vd1*C[sp_var*4 + 2])*cosa)/aux;
                    C[((sp_var+3)%4)*4 + 1] =   (Vd1*C[sp_var*4 + 1]*sina - (mD1 + Vd1*C[sp_var*4 + 3])*cosa)/aux;
                } else {
                    aux = Vd1*cosa - Vd0*sina;
                    C[((sp_var+3)%4)*4    ] =   (Vd1*C[sp_var*4    ]*cosa + (mD0 + Vd1*C[sp_var*4 + 2])*sina)/aux;
                    C[((sp_var+3)%4)*4 + 1] =   (Vd1*C[sp_var*4 + 1]*cosa + (mD1 + Vd1*C[sp_var*4 + 3])*sina)/aux;
                }

                C[((sp_var+3)%4)*4 + 2] = - (C[((sp_var+3)%4)*4    ]*Vd0 + mD0)/Vd1;
                C[((sp_var+3)%4)*4 + 3] = - (C[((sp_var+3)%4)*4 + 1]*Vd0 + mD1)/Vd1;
	
                if (eq_relations[(sp_var+1)%4][(sp_var+2)%4] == 1) {
                    C[((sp_var+2)%4)*4    ] = C[((sp_var+1)%4)*4    ] + ( (C[((sp_var+3)%4)*4    ] - C[((sp_var+1)%4)*4    ])*cosa + (C[((sp_var+3)%4)*4 + 2] - C[((sp_var+1)%4)*4 + 2])*sina )*cosa;
                    C[((sp_var+2)%4)*4 + 1] = C[((sp_var+1)%4)*4 + 1] + ( (C[((sp_var+3)%4)*4 + 1] - C[((sp_var+1)%4)*4 + 1])*cosa + (C[((sp_var+3)%4)*4 + 3] - C[((sp_var+1)%4)*4 + 3])*sina )*cosa;

                    C[((sp_var+2)%4)*4 + 2] = C[((sp_var+3)%4)*4 + 2] - ( (C[((sp_var+3)%4)*4 + 2] - C[((sp_var+1)%4)*4 + 2])*cosa - (C[((sp_var+3)%4)*4    ] - C[((sp_var+1)%4)*4    ])*sina )*cosa;
                    C[((sp_var+2)%4)*4 + 3] = C[((sp_var+3)%4)*4 + 3] - ( (C[((sp_var+3)%4)*4 + 3] - C[((sp_var+1)%4)*4 + 3])*cosa - (C[((sp_var+3)%4)*4 + 1] - C[((sp_var+1)%4)*4 + 1])*sina )*cosa;
                } else {
                    C[((sp_var+2)%4)*4    ] = C[((sp_var+3)%4)*4    ] + ( (C[((sp_var+1)%4)*4    ] - C[((sp_var+3)%4)*4    ])*cosa + (C[((sp_var+1)%4)*4 + 2] - C[((sp_var+3)%4)*4 + 2])*sina )*cosa;
                    C[((sp_var+2)%4)*4 + 1] = C[((sp_var+3)%4)*4 + 1] + ( (C[((sp_var+1)%4)*4 + 1] - C[((sp_var+3)%4)*4 + 1])*cosa + (C[((sp_var+1)%4)*4 + 3] - C[((sp_var+3)%4)*4 + 3])*sina )*cosa;

                    C[((sp_var+2)%4)*4 + 2] = C[((sp_var+1)%4)*4 + 2] - ( (C[((sp_var+1)%4)*4 + 2] - C[((sp_var+3)%4)*4 + 2])*cosa - (C[((sp_var+1)%4)*4    ] - C[((sp_var+3)%4)*4    ])*sina )*cosa;
                    C[((sp_var+2)%4)*4 + 3] = C[((sp_var+1)%4)*4 + 3] - ( (C[((sp_var+1)%4)*4 + 3] - C[((sp_var+3)%4)*4 + 3])*cosa - (C[((sp_var+1)%4)*4 + 1] - C[((sp_var+3)%4)*4 + 1])*sina )*cosa;
                }
	  
            } else {// Case 1: JUST ONE ON THE SIDE
                if (eq_relations[(sp_var+1)%4][(sp_var+2)%4] == 1) {
                    aux = Vd0*cosa + Vd1*sina;
                    C[((sp_var+2)%4)*4    ] =   (Vd1*C[((sp_var+1)%4)*4    ]*sina - (mD0 + Vd1*C[((sp_var+1)%4)*4 + 2])*cosa)/aux;
                    C[((sp_var+2)%4)*4 + 1] =   (Vd1*C[((sp_var+1)%4)*4 + 1]*sina - (mD1 + Vd1*C[((sp_var+1)%4)*4 + 3])*cosa)/aux;
                } else {
                    aux = Vd1*cosa - Vd0*sina;
                    C[((sp_var+2)%4)*4    ] =   (Vd1*C[((sp_var+1)%4)*4    ]*cosa + (mD0 + Vd1*C[((sp_var+1)%4)*4 + 2])*sina)/aux;
                    C[((sp_var+2)%4)*4 + 1] =   (Vd1*C[((sp_var+1)%4)*4 + 1]*cosa + (mD1 + Vd1*C[((sp_var+1)%4)*4 + 3])*sina)/aux;
                }

                C[((sp_var+2)%4)*4 + 2] = - (C[((sp_var+2)%4)*4    ]*Vd0 + mD0)/Vd1;
                C[((sp_var+2)%4)*4 + 3] = - (C[((sp_var+2)%4)*4 + 1]*Vd0 + mD1)/Vd1;
	  
                if (eq_relations[(sp_var+2)%4][(sp_var+3)%4] == 1) {
                    C[((sp_var+3)%4)*4    ] = C[((sp_var+2)%4)*4    ] + ( (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*cosa + (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*sina )*cosa;
                    C[((sp_var+3)%4)*4 + 1] = C[((sp_var+2)%4)*4 + 1] + ( (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*cosa + (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*sina )*cosa;

                    C[((sp_var+3)%4)*4 + 2] = C[sp_var*4 + 2] - ( (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*cosa - (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*sina )*cosa;
                    C[((sp_var+3)%4)*4 + 3] = C[sp_var*4 + 3] - ( (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*cosa - (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*sina )*cosa;
                } else {
                    C[((sp_var+3)%4)*4    ] = C[sp_var*4    ] + ( (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*cosa + (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*sina )*cosa;
                    C[((sp_var+3)%4)*4 + 1] = C[sp_var*4 + 1] + ( (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*cosa + (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*sina )*cosa;

                    C[((sp_var+3)%4)*4 + 2] = C[((sp_var+2)%4)*4 + 2] - ( (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*cosa - (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*sina )*cosa;
                    C[((sp_var+3)%4)*4 + 3] = C[((sp_var+2)%4)*4 + 3] - ( (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*cosa - (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*sina )*cosa;
                }
            }
        } else if (Case == 3) {
            if (eq_relations[sp_var][(sp_var+3)%4] == 1) {
                aux = Vc0*cosa + Vc1*sina;
                C[((sp_var+3)%4)*4    ] =   (Vc1*C[sp_var*4    ]*sina - (mC0 + Vc1*C[sp_var*4 + 2])*cosa)/aux;
                C[((sp_var+3)%4)*4 + 1] =   (Vc1*C[sp_var*4 + 1]*sina - (mC1 + Vc1*C[sp_var*4 + 3])*cosa)/aux;
            } else {
                aux = Vc1*cosa - Vc0*sina;
                C[((sp_var+3)%4)*4    ] =   (Vc1*C[sp_var*4    ]*cosa + (mC0 + Vc1*C[sp_var*4 + 2])*sina)/aux;
                C[((sp_var+3)%4)*4 + 1] =   (Vc1*C[sp_var*4 + 1]*cosa + (mC1 + Vc1*C[sp_var*4 + 3])*sina)/aux;
            }

            C[((sp_var+3)%4)*4 + 2] = - (C[((sp_var+3)%4)*4    ]*Vc0 + mC0)/Vc1;
            C[((sp_var+3)%4)*4 + 3] = - (C[((sp_var+3)%4)*4 + 1]*Vc0 + mC1)/Vc1;
	
            if (eq_relations[(sp_var+3)%4][(sp_var+2)%4] == 1) {
                aux = Vd0*cosa + Vd1*sina;
                C[((sp_var+2)%4)*4    ] =   (Vd1*C[((sp_var+3)%4)*4    ]*sina - (mD0 + Vd1*C[((sp_var+3)%4)*4 + 2])*cosa)/aux;
                C[((sp_var+2)%4)*4 + 1] =   (Vd1*C[((sp_var+3)%4)*4 + 1]*sina - (mD1 + Vd1*C[((sp_var+3)%4)*4 + 3])*cosa)/aux;
            } else {
                aux = Vd1*cosa - Vd0*sina;
                C[((sp_var+2)%4)*4    ] =   (Vd1*C[((sp_var+3)%4)*4    ]*cosa + (mD0 + Vd1*C[((sp_var+3)%4)*4 + 2])*sina)/aux;
                C[((sp_var+2)%4)*4 + 1] =   (Vd1*C[((sp_var+3)%4)*4 + 1]*cosa + (mD1 + Vd1*C[((sp_var+3)%4)*4 + 3])*sina)/aux;
            }

            C[((sp_var+2)%4)*4 + 2] = - (C[((sp_var+2)%4)*4    ]*Vd0 + mD0)/Vd1;
            C[((sp_var+2)%4)*4 + 3] = - (C[((sp_var+2)%4)*4 + 1]*Vd0 + mD1)/Vd1;

            if (eq_relations[sp_var][(sp_var+1)%4] == 1) {
                C[((sp_var+1)%4)*4    ] = C[sp_var*4    ] + ( (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*cosa + (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 1] = C[sp_var*4 + 1] + ( (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*cosa + (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*sina )*cosa;

                C[((sp_var+1)%4)*4 + 2] = C[((sp_var+2)%4)*4 + 2] - ( (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*cosa - (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 3] = C[((sp_var+2)%4)*4 + 3] - ( (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*cosa - (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*sina )*cosa;
            } else {
                C[((sp_var+1)%4)*4    ] = C[((sp_var+2)%4)*4    ] + ( (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*cosa + (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 1] = C[((sp_var+2)%4)*4 + 1] + ( (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*cosa + (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*sina )*cosa;

                C[((sp_var+1)%4)*4 + 2] = C[sp_var*4 + 2] - ( (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*cosa - (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 3] = C[sp_var*4 + 3] - ( (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*cosa - (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*sina )*cosa;
            }
        } else { //Case 4: TWO AGAINST TWO
            aux = Vc0*Vd1 - Vc1*Vd0;

            C[((sp_var+2)%4)*4    ] =   (Vc1*mD0 - mC0*Vd1)/aux;
            C[((sp_var+2)%4)*4 + 1] =   (Vc1*mD1 - mC1*Vd1)/aux;

            C[((sp_var+2)%4)*4 + 2] = - (Vc0*mD0 - mC0*Vd0)/aux;
            C[((sp_var+2)%4)*4 + 3] = - (Vc0*mD1 - mC1*Vd0)/aux;

            if (eq_relations[sp_var][(sp_var+1)%4] == 1) {
                C[((sp_var+1)%4)*4    ] = C[sp_var*4    ] + ( (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*cosa + (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 1] = C[sp_var*4 + 1] + ( (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*cosa + (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*sina )*cosa;

                C[((sp_var+1)%4)*4 + 2] = C[((sp_var+2)%4)*4 + 2] - ( (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*cosa - (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 3] = C[((sp_var+2)%4)*4 + 3] - ( (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*cosa - (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*sina )*cosa;
            } else {
                C[((sp_var+1)%4)*4    ] = C[((sp_var+2)%4)*4    ] + ( (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*cosa + (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 1] = C[((sp_var+2)%4)*4 + 1] + ( (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*cosa + (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*sina )*cosa;

                C[((sp_var+1)%4)*4 + 2] = C[sp_var*4 + 2] - ( (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*cosa - (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*sina )*cosa;
                C[((sp_var+1)%4)*4 + 3] = C[sp_var*4 + 3] - ( (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*cosa - (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*sina )*cosa;
            }

            if (eq_relations[(sp_var+2)%4][(sp_var+3)%4] == 1) {
                C[((sp_var+3)%4)*4    ] = C[((sp_var+2)%4)*4    ] + ( (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*cosa + (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*sina )*cosa;
                C[((sp_var+3)%4)*4 + 1] = C[((sp_var+2)%4)*4 + 1] + ( (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*cosa + (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*sina )*cosa;

                C[((sp_var+3)%4)*4 + 2] = C[sp_var*4 + 2] - ( (C[sp_var*4 + 2] - C[((sp_var+2)%4)*4 + 2])*cosa - (C[sp_var*4    ] - C[((sp_var+2)%4)*4    ])*sina )*cosa;
                C[((sp_var+3)%4)*4 + 3] = C[sp_var*4 + 3] - ( (C[sp_var*4 + 3] - C[((sp_var+2)%4)*4 + 3])*cosa - (C[sp_var*4 + 1] - C[((sp_var+2)%4)*4 + 1])*sina )*cosa;
            } else {
                C[((sp_var+3)%4)*4    ] = C[sp_var*4    ] + ( (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*cosa + (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*sina )*cosa;
                C[((sp_var+3)%4)*4 + 1] = C[sp_var*4 + 1] + ( (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*cosa + (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*sina )*cosa;

                C[((sp_var+3)%4)*4 + 2] = C[((sp_var+2)%4)*4 + 2] - ( (C[((sp_var+2)%4)*4 + 2] - C[sp_var*4 + 2])*cosa - (C[((sp_var+2)%4)*4    ] - C[sp_var*4    ])*sina )*cosa;
                C[((sp_var+3)%4)*4 + 3] = C[((sp_var+2)%4)*4 + 3] - ( (C[((sp_var+2)%4)*4 + 3] - C[sp_var*4 + 3])*cosa - (C[((sp_var+2)%4)*4 + 1] - C[sp_var*4 + 1])*sina )*cosa;
            }
        }
    }
    
    d1 = C[8] - C[4];
    d2 = C[9] - C[5];
    d3 = C[10] - C[6];
    d4 = C[11] - C[7];
    d5 = C[12] - C[8];
    d6 = C[13] - C[9];
    d7 = C[14] - C[10];
    d8 = C[15] - C[11];

    //Constants to linearlly calculate w and l 3D dimensions:
    la = d1*d1 + d3*d3;
    lb = d1*d2 + d3*d4;
    lc = d2*d2 + d4*d4;
    
    wa = d5*d5 + d7*d7;
    wb = d5*d6 + d7*d8;
    wc = d6*d6 + d8*d8;

}

  //Get the 3D width w, given 3D height h and pre-calculated constants, if there is a valid solution. 
void ReliabilityClassification::RC_get_h_from_w(double w, double *h, double hmax_from_area) {
    double h1, h2;
    RC_compute_h_from_w(w, &h1, &h2);
    //Verify the valid solution
    if(h1 > 0)
        *h = h1;
    else if(h2 > 0 && h2 <= hmax_from_area)
        *h = h2;
    else
        *h = h1;

}

//Get 3D height h, given the 3D length l and pre-calculated constants, if there is a valid solution.
void ReliabilityClassification::RC_get_h_from_l(double l, double *h, double hmax_from_area) {
    double h1, h2;
    RC_compute_h_from_l(l, &h1, &h2);
    //Verify the valid solution
    if(h1 > 0)
        *h = h1;
    else if(h2 > 0 && h2 <= hmax_from_area)
        *h = h2;
    else
        *h = h1;
}

//Compute 3D height h  possible values, given the 3D width w and pre-calculated constants.
void ReliabilityClassification::RC_compute_h_from_w(double w, double *h1, double *h2 ) {
    if(h1)
        *h1 = - (   sqrt( wa*(w*w - wc) + wb*wb ) + wb ) / wa ;
    if(h2)
        *h2 = - ( - sqrt( wa*(w*w - wc) + wb*wb ) + wb ) / wa ;
}

//Compute 3D height h possible values, given the 3D length l and pre-calculated constants.
void ReliabilityClassification::RC_compute_h_from_l(double l, double *h1, double *h2 ) {
    if(h1)
        *h1 = - (   sqrt( la*(l*l - lc) + lb*lb ) + lb ) / la ;
    if(h2)
        *h2 = - ( - sqrt( la*(l*l - lc) + lb*lb ) + lb ) / la ;
}

//Compute 3D width w, given the 3D height h and pre-calculated constants.
int ReliabilityClassification::RC_compute_w_from_h(double h, double *w) {
    *w = sqrt( fabs(wa*h*h + 2*wb*h +  wc) );
    return 0;
}

//Compute 3D length l, given the 3D height h and pre-calculated constants.
int ReliabilityClassification::RC_compute_l_from_h(double h, double *l) {
    *l = sqrt( fabs(la*h*h + 2*lb*h +  lc) );
    return 0;
}
  
//Calculate the parallelpiped 3D points according to height and pre-calculated constants.
int ReliabilityClassification::RC_compute_blob_3Dbbox(Blob *blob, double h) {
    double X[4],Y[4];
    int i;

    for(i=0; i<4; i++) {
        X[i] = C[i*4    ] * h + C[i*4 + 1] ;
        Y[i] = C[i*4 + 2] * h + C[i*4 + 3] ;
    }

#ifdef RC_OUTPUT_2000
    std::cout +"\tCASE: " + Case);
    if(Case)
      std::cout +"\tSPECIAL VARIABLE: " + sp_var);
    std::cout +"\tH: " + h);
    std::cout +"\tALPHA: " + 180*alpha/M_PI);
    std::cout +"\tALPHA - BETA: " + 180*(alpha - beta)/M_PI);
    for(i=0; i<4; i++)
      std::cout +"\tAngle "+i+ "-" +(i+1)%4 + ": "  + 180*atan2(Y[(i+1)%4] - Y[i], X[(i+1)%4] - X[i])/M_PI);
    #endif

    return blob->set3DBBox(m_context, (double *)X, (double *)Y, h);

}

//Determine which Case of parallelpiped solution is, according to the position of parallelpiped vertexes in the 2D blob limits.
//Implementation with simpler method
int ReliabilityClassification::RC_set_3Dbbox_case_and_variable_indexes_simpler(Blob *blob) {
    RC_set_3D_bbox_alpha_level_data_get_case();	
    RC_compute_blob_3Dbbox_get_case(blob);
    BLOB_3DBBOX(blob)->getLimits(limits, nlimits, varlimrel);
    return RC_set_3Dbbox_case_and_variable_indexes();
}

int ReliabilityClassification::RC_compute_blob_3Dbbox_simpler(Blob *blob, double h) {
    double X[4], Y[4], w, l;

    w = MM1*h + MM2;
    l = MM3*h + MM4;
    X[3] = MM5*h+ MM6;
    Y[3] = -(Vt0*X[3] + mt0*h + mt1) / Vt1;
    X[2] = X[3] + w*beta_direction*sina;
    Y[2] = Y[3] - w*cosa; 
    X[0] = X[3] - l*beta_direction*cosa;
    Y[0] = Y[3] - l*sina;
    X[1] = X[2] - l*beta_direction*cosa;
    Y[1] = Y[2] - l*sina;

    return blob->set3DBBox(m_context, (double *)X, (double *)Y, h);
}

int ReliabilityClassification::RC_compute_blob_3Dbbox_get_case(Blob *blob) {

    double X[4], Y[4], w, l, h = 0.0;
    bool inversion = false;

    if(MM2 > 0 && MM4 > 0) { //Both w and l have linear intercept > 0, case can be estimated with h = 0 
        w = MM2;
        l = MM4;
        X[3] = MM6;
        Y[3] = -(Vt0*X[3] + mt1) / Vt1;
    } else { //if not, search for a coherent h
      
        Interval interval_h_from_w, interval_h_from_l, new_interval_h;
        if(MM1 > 0) //Positive slope for w, if h grows, w grows
            Interval::newInterval(&interval_h_from_w, (model_wmin - MM2) / MM1, (model_wmax - MM2) / MM1);
        else //Negative slope, invert order
            Interval::newInterval(&interval_h_from_w, (model_wmax - MM2) / MM1, (model_wmin - MM2) / MM1);
      
        if(MM3 > 0) //Positive slope for l, if h grows, l grows
            Interval::newInterval(&interval_h_from_l, (model_lmin - MM4) / MM3, (model_lmax - MM4) / MM3);
        else //Negative slope, invert order
            Interval::newInterval(&interval_h_from_l, (model_lmax - MM4) / MM3, (model_lmin - MM4) / MM3);

        Interval::intersect(&new_interval_h, &interval_h_from_l, &interval_h_from_w);

        if(INTERVAL_IS_NULL(&new_interval_h)) {
            h = 0;
            w = MM1*h + MM2;
            l = MM3*h + MM4;
            if(w == 0.0 || l == 0.0) {  //Two linear equations can be 0 just once each!!
                                        //So, just in case, we test twice :D
                h = 10;
                w = MM1*h + MM2;
                l = MM3*h + MM4;
                if(w == 0.0 || l == 0.0) {
                    h = 20;
                    w = MM1*h + MM2;
                    l = MM3*h + MM4;
                }
            }

            X[3] = MM5*h+ MM6;
            Y[3] = -(Vt0*X[3] + mt0*h + mt1) / Vt1;

            if( (w < 0.0 && l > 0.0) || (w > 0.0 && l < 0.0) )
                inversion = true;

        } else {
            h = (INTERVAL_X1(&new_interval_h) + INTERVAL_X2(&new_interval_h)) / 2.0;
            w = MM1*h + MM2;
            l = MM3*h + MM4;
            X[3] = MM5*h+ MM6;
            Y[3] = -(Vt0*X[3] + mt0*h + mt1) / Vt1;
        }
    }
    
    X[2] = X[3] + w*beta_direction*sina;
    Y[2] = Y[3] - w*cosa; 
    X[0] = X[3] - l*beta_direction*cosa;
    Y[0] = Y[3] - l*sina;
    X[1] = X[2] - l*beta_direction*cosa;
    Y[1] = Y[2] - l*sina;

    if(inversion) {
        double auxX, auxY;
        auxX = X[0];
        auxY = Y[0];
        X[0] = X[3];
        Y[0] = Y[3];
        X[3] = auxX;
        Y[3] = auxY;
        auxX = X[1];
        auxY = Y[1];
        X[1] = X[2];
        Y[1] = Y[2];
        X[2] = auxX;
        Y[2] = auxY;
    }

    return blob->set3DBBox(m_context, (double *)X, (double *)Y, h);

}

//Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants, without knowing which Case of parallelpiped
//solutions can be obtained.
void ReliabilityClassification::RC_set_3D_bbox_alpha_level_data_get_case() {

    //With normal variable<->bound relation (v0<->L, v1<->B, v2<->R, v3<->T).
    DD = K[8]*(1 - 2*cosa*cosa)*beta_direction + LL1*sina*cosa;

    MM1 = (LL2*beta_direction*cosa + LL3*sina) / DD;
    MM2 = (LL4*beta_direction*cosa + LL5*sina) / DD;
    MM3 = (LL6*beta_direction*sina + LL7*cosa) / DD;
    MM4 = (LL8*beta_direction*sina + LL9*cosa) / DD;
    MM5 = (LL10*beta_direction*cosa*cosa + LL11*beta_direction*sina*sina + LL12*sina*cosa ) / DD;
    MM6 = (LL13*beta_direction*cosa*cosa + LL14*beta_direction*sina*sina + LL15*sina*cosa ) / DD;

}

//Set the information considering parallelpiped orientation alpha and blob 2D dimensions as constants knowing the type (Case) of parallelpiped
//solutions that can be obtained. Normally a parallelpiped has one vertex point on each blob dimension, but there are some degenerate cases
//with some vertexes in more than one blob dimension.
void ReliabilityClassification::RC_set_3D_bbox_alpha_level_data_simpler() {
    //ACA TODO!!!
}


//Determine which Case of parallelpiped solution is, according to the position of parallelpiped vertexes in the 2D blob limits.
int ReliabilityClassification::RC_set_3Dbbox_case_and_variable_indexes() {
    int i, k, lim_count = 0;
    sp_var = -1;

    for(i = 0; i < 4; i++) {
        if(sp_var < 0 && nlimits[i] == 2)
            sp_var = i;
        if(nlimits[i] > 0)
            lim_count++;
    }

    if(lim_count == 4) //If each point is on one limit is the Normal case
        return 0;

    if(sp_var < 0) //Degenerate case where point is in three limits, returns normal blob with default values
        return -1;

    //If we have arrived here, this is a special case of a point 'sp_var' with two limits 

    //Search for the two limits
    a = -1;
    for(i = 0; i < 4; i++) {
        if(a < 0 && limits[sp_var][i])
            a = i;
        else if(limits[sp_var][i]) {
            b = i;
            break;
        }
    }

    //Search the indexes for the other limits
    if (nlimits[(sp_var+1)%4] == 1) { //If other points belongs to one limit (following to the right) 
        for(i = 0; i < 4; i++)
            if(limits[(sp_var+1)%4][i]) {
                c = i;
                break;
            }
        if(nlimits[(sp_var+2)%4] == 1) { //The last limit is on sp_var+2
            for(i = 0; i < 4; i++)
                if(limits[(sp_var+2)%4][i]) {
                    d = i;
                    break;
                }

            return 1;
        } else { //point sp_var+3 has 1 limit
            for(i = 0; i < 4; i++)
                if(limits[(sp_var+3)%4][i]) {
                    d = i;
                    break;
                }
            return 2;
        }
    } else if (nlimits[(sp_var+3)%4] == 1) { //If one point has one limit the case is clear: (sp_var+3) and (sp_var+2) are in limits
        for(i = 0; i < 4; i++)
            if(limits[(sp_var+3)%4][i]) {
                c = i;
                break;
            }
        for(i = 0; i < 4; i++)
            if(limits[(sp_var+2)%4][i]) {
                d = i;
                break;
            }
        return 3;
    } else if (nlimits[(sp_var+2)%4] == 2) { //case double 2 limits
        for(i = sp_var + 1; i < 4; i++) //Search the next point with two limits
            if(nlimits[i]==2) {
                k=i;
                break;
            }
        c = -1;
        for(i = 0; i < 4; i++) {
            if(c < 0 && limits[k][i])
                c = i;
            else if(limits[k][i]) {
                d = i;
                break;
            }
        }

        return 4;
    }

    //Degenerate case
    return -1;
}

//Set vertical and horizontal advancement in pixel analysis in term of number of pixels, using parameters pre-defined by the end-user.
void ReliabilityClassification::setPixelRates(int bwidth, int bheight, double *horRate, double *verRate) {

    double t;

    if(bwidth * bheight < RC_MIN_PIXELS)
        *verRate = *horRate = 1.0;
    else if (bwidth * bheight * m_pixelDensity < RC_MIN_PIXELS) {
        t = RC_MIN_PIXELS / (double) (bwidth * bheight);
        *horRate = sqrt(bwidth / (bheight * t));
        *verRate = sqrt(bheight / (bwidth * t));
    } else if (bwidth * bheight * m_pixelDensity > RC_MAX_PIXELS) {
        t = RC_MAX_PIXELS / (double) (bwidth * bheight);
        *horRate = sqrt(bwidth / (bheight * t));
        *verRate = sqrt(bheight / (bwidth * t));
    } else {
        *horRate = sqrt(bwidth / (bheight * m_pixelDensity));
        *verRate = sqrt(bheight / (bwidth * m_pixelDensity));
    }
}


//Used after 3D bounding box construction, to store the number of moving or not moving pixels inside or outside the generated parallelpiped.
void ReliabilityClassification::setPixelAnalysis(int blob_position, Blob *blob, ddata_t ddata, Parallelpiped *_3Dbbox) {
    int TP=0, TN=0, FP=0, FN=0;
    int i,j,counter=0;
    int llim = (RECT_XLEFT(&realBBox) + 1 >= 0) ? RECT_XLEFT(&realBBox) + 1 : 0, 
        rlim = (RECT_XRIGHT(&realBBox)  < m_foreground->width()) ? RECT_XRIGHT(&realBBox)  : m_foreground->width() - 1,
        blim = (RECT_YBOTTOM(&realBBox) < m_foreground->height()) ? RECT_YBOTTOM(&realBBox) : m_foreground->height() - 1,
        tlim = (RECT_YTOP(&realBBox)  + 1 >= 0) ? RECT_YTOP(&realBBox)  + 1 : 0;
    double di, dj, horRate, verRate;
    polygon2D<double> *poly = set_polygon_from_3Dbb(_3Dbbox, blob_position);
    poly->computeBoundingRectangle();

    if(BLOB_MOVING_PIXELS(blob) != NULL) { // Use the information contained in the blob
        char *moving = BLOB_MOVING_PIXELS(blob);
        horRate = DDATA_HRATE(BLOB_DDATA(blob));
        verRate = DDATA_VRATE(BLOB_DDATA(blob));
        for(di = llim, i = (int)round(di); i < rlim; di += horRate, i = (int)round(di)) {
            for(dj = tlim, j = (int)round(dj); j < blim ; dj += verRate, j = (int)round(dj)) {
                if (poly->pointInConvexPolygon(i, j, false)) {
                    if(moving[counter])
                        TP++;
                    else
                        FP++;
                } else {
                    if(moving[counter])
                        FN++;
                    else
                        TN++;
                }
                counter++;
            }
        }

    } else { //Use the information of current frame

        setPixelRates(rlim - llim, blim - tlim, &horRate, &verRate);
        unsigned char *pixels = m_foreground->bits();
        char step = (m_foreground->format() == QImage::Format_ARGB32) ? 4 : 1;
        int width = m_foreground->width();
        llim *= step;
        rlim *= step;
        horRate *= step;
        tlim *= step;
        blim *= step;
        verRate *= step;
        for(di = llim, i = (int)round(di); i < rlim; di += horRate, i=(int)round(di)) {
            for(dj = tlim, j = (int)round(dj); j < blim ; dj += verRate, j=(int)round(dj)) {
                counter++;
                if (poly->pointInConvexPolygon(i, j, false)) {
                    if(pixels[i + j*width])
                        TP++;
                    else
                        FP++;
                } else {
                    if(pixels[i + j*width])
                        FN++;
                    else
                        TN++;
                }
            }
        }
    }

    DDATA_NPIX(ddata) = counter;
    DDATA_HRATE(ddata) = horRate;
    DDATA_VRATE(ddata) = verRate;

    DDATA_TP(ddata) = TP;
    DDATA_TN(ddata) = TN;
    DDATA_FP(ddata) = FP;
    DDATA_FN(ddata) = FN;

    delete poly;
}
  

//Used after 3D bounding box construction, to store the number of moving or not moving pixels inside or outside the generated parallelpiped.
void ReliabilityClassification::setPixelAnalysis(Blob *blob, ddata_t ddata, parallelpiped_t _3Dbbox) {
    
    int i,j,counter = 0;
    int llim = (RECT_XLEFT(&realBBox) + 1 >= 0) ? RECT_XLEFT(&realBBox) + 1 : 0, 
        rlim = (RECT_XRIGHT(&realBBox)  < m_foreground->width()) ? RECT_XRIGHT(&realBBox)  : m_foreground->width() - 1,
        blim = (RECT_YBOTTOM(&realBBox) < m_foreground->height()) ? RECT_YBOTTOM(&realBBox) : m_foreground->height() - 1,
        tlim = (RECT_YTOP(&realBBox)  + 1 >= 0) ? RECT_YTOP(&realBBox)  + 1 : 0;
    double di, dj, horRate, verRate;
    int TP=0, TN=0, FP=0, FN=0;

    polygon2D<double> *poly = set_polygon_from_3Dbb(_3Dbbox, BLOB_POSITION(blob));
    poly->computeBoundingRectangle();

    if(BLOB_MOVING_PIXELS(blob) != NULL) { // Use the information contained in the blob
        char *moving = BLOB_MOVING_PIXELS(blob);
        horRate = DDATA_HRATE(BLOB_DDATA(blob));
        verRate = DDATA_VRATE(BLOB_DDATA(blob));
        for(di = llim, i = (int)round(di); i < rlim; di += horRate, i=(int)round(di)) {
            for(dj = tlim, j = (int)round(dj); j < blim ; dj += verRate, j=(int)round(dj)) {
                if (poly->pointInConvexPolygon(i, j, false)) {
                    if(moving[counter])
                        TP++;
                    else
                        FP++;
                } else {
                    if(moving[counter])
                        FN++;
                    else
                        TN++;
                }
                counter++;
            }
        }

    } else { //Use the information of current frame

        setPixelRates(rlim - llim, blim - tlim, &horRate, &verRate);
        unsigned char *pixels = m_foreground->bits();
        char step = (m_foreground->format() == QImage::Format_ARGB32) ? 4 : 1;
        int width = m_foreground->width();
        llim *= step;
        rlim *= step;
        horRate *= step;
        tlim *= step;
        blim *= step;
        verRate *= step;
        for(di = llim, i = (int)round(di); i < rlim; di += horRate, i=(int)round(di)) {
            for(dj = tlim, j = (int)round(dj); j < blim ; dj += verRate, j=(int)round(dj)) {
                counter++;
                if (poly->pointInConvexPolygon(i, j, false)) {
                    if(pixels[i + j*width])
                        TP++;
                    else
                        FP++;
                } else {
                    if(pixels[i + j*width])
                        FN++;
                    else
                        TN++;
                }
            }
        }
    }

#ifdef RC_OUTPUT_DDATA
    AppendToLog("\nEvaluated pixels: %d\n",counter);
    AppendToLog("TP: %d\t",TP);
    AppendToLog("TN: %d\t",TN);
    AppendToLog("FP: %d\t",FP);
    AppendToLog("FN: %d\n",FN);
#endif

  
    DDATA_TP(ddata) = TP;
    DDATA_TN(ddata) = TN;
    DDATA_FP(ddata) = FP;
    DDATA_FN(ddata) = FN;
    DDATA_NPIX(ddata) = counter;
    DDATA_HRATE(ddata) = horRate;
    DDATA_VRATE(ddata) = verRate;
    
    delete poly;
    
}

//Gets the index of the nearest base point to a blob 2D limit, between the base points lying between the nearest and the farest base points (middle points).
int ReliabilityClassification::get_nearest_from_middle_points(parallelpiped_t _3Dbbox, int limit) {
    int ext1,ext2,mid1=-1,mid2;
    double val, min=DBL_MAX,max = 0.0;
    int i, middle[4] = {1,1,1,1};
     
    //Determine min and max indexes
    for(i=0; i<4 ; i++) {
        val = limit%2 == 1 ? PARALL_X2D_BASE_i(_3Dbbox, i) : PARALL_Y2D_BASE_i(_3Dbbox, i);
        if(val > max){
            max = val;
            ext1 = i;
        }
        if(val < min){
            min = val;
            ext2 = i;
        }
    }

    //Mark the points we don't want.
    middle[ext1] = middle[ext2] = 0;

    //Get the middle points
    for(i=0; i<4 ; i++)
        if(middle[i]) {
            if(mid1>=0) {
                mid2 = i;
                break;
            } else
                mid1 = i;
        }
    
    //Check which one of the two remaining points is nearer to the limit.
    switch(limit) {
        case 0:  //LEFT
            return PARALL_X2D_BASE_i(_3Dbbox, mid1) < PARALL_X2D_BASE_i(_3Dbbox, mid2) ? mid1 : mid2;
            break;
        case 1:  //BOTTOM
            return PARALL_Y2D_BASE_i(_3Dbbox, mid1) > PARALL_Y2D_BASE_i(_3Dbbox, mid2) ? mid1 : mid2;
            break;
        case 2:  //RIGHT
            return PARALL_X2D_BASE_i(_3Dbbox, mid1) > PARALL_X2D_BASE_i(_3Dbbox, mid2) ? mid1 : mid2;
            break;
        case 3:  //TOP
            return PARALL_Y2D_BASE_i(_3Dbbox, mid1) < PARALL_Y2D_BASE_i(_3Dbbox, mid2) ? mid1 : mid2;
            break;
        default:
            assert(0);
    }

}

//Generate the polygon representing the contour of a parallelpiped.
polygon2D<double> *ReliabilityClassification::set_polygon_from_3Dbb(parallelpiped_t _3Dbbox, int position) {
    polygon2D<double> *poly;
    int i;

    if(position == 4) { //If blob in Middle-Center image projected 2d polygon will have just four points corresponding to the 4 points in height h
        poly = new polygon2D<double>(4);
        for(i=0; i<4 ; i++) {
            poly->points[i].x = (int)round(PARALL_X2D_H_i(_3Dbbox, i));
            poly->points[i].y = (int)round(PARALL_Y2D_H_i(_3Dbbox, i));
        }
    } else {
        int j, nearest;
        bool pbase[4]={true,true,true,true}, ph[4]={true,true,true,true}, inh = true;

        switch(position) { //From the two points in the middle, which is the nearest to the closest limit to focal point
            case 0: case 1: case 2:
                nearest = get_nearest_from_middle_points(_3Dbbox, 1);
                break;
            case 3:
                nearest = get_nearest_from_middle_points(_3Dbbox, 2);
                break;
            case 5:
                nearest = get_nearest_from_middle_points(_3Dbbox, 0);
                break;
            case 6: case 7: case 8:
                nearest = get_nearest_from_middle_points(_3Dbbox, 3);
                break;
            default:
                assert(0);
        }
        ph[nearest]=false;
        pbase[(nearest+2) % 4]=false;

        poly = new polygon2D<double>(6);
        i=nearest;

        for(j = 0; j < 6; j++) {
            if(inh && pbase[i % 4]) {
                poly->points[j].x = (int)round(PARALL_X2D_BASE_i(_3Dbbox, i % 4));
                poly->points[j].y = (int)round(PARALL_Y2D_BASE_i(_3Dbbox, i % 4));
                pbase[i % 4] = false;
              inh = false;
            } else if(!inh && ph[i % 4]) {
                poly->points[j].x = (int)round(PARALL_X2D_H_i(_3Dbbox, i % 4));
                poly->points[j].y = (int)round(PARALL_Y2D_H_i(_3Dbbox, i % 4));
                ph[i % 4] = false;
                inh = true;
            } else {
                i++;
                if(inh) {
                    poly->points[j].x = (int)round(PARALL_X2D_H_i(_3Dbbox, i % 4));
                    poly->points[j].y = (int)round(PARALL_Y2D_H_i(_3Dbbox, i % 4));
                    ph[i % 4] = false;
                } else {
                    poly->points[j].x = (int)round(PARALL_X2D_BASE_i(_3Dbbox, i % 4));
                    poly->points[j].y = (int)round(PARALL_Y2D_BASE_i(_3Dbbox, i % 4));
                    pbase[i % 4] = false;
                }
            }
        }
    }

    return poly;
}

 
//Returns the maximal limit (Wmax or Hmax) for the size of a blob, according to the maximal limits
//of dimensions of the expected object in the scen currently processed.
double ReliabilityClassification::limits_by_max_possible_blob(Blob *blob, DetectionProblemType dptype) {
    double x0, x1;
    double y0, y1;
    double x2d, y2d;
    double slope, b;
    int x_dir, pos, in_height, add_height;
    double x2d_modifier, y2d_modifier;
    double hyp = sqrt(model_lmax*model_lmax + model_wmax*model_wmax);
    double new_distance;

    //For testing
    /*    dptype = MM_CAM_OCCL_RIGHT;
    BLOB_XLEFT(blob) = 340;
    BLOB_YBOTTOM(blob) = 270; 
    BLOB_XRIGHT(blob) = 352; 
    BLOB_YTOP(blob) = 255;
    BLOB_WIDTH(blob) = 12;
    BLOB_HEIGHT(blob) = 15;
    BLOB_POSITION(blob) = 2;
    */
    pos = BLOB_POSITION(blob);

    if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) {
        x2d = (double) BLOB_XLEFT(blob); x2d_modifier = 1.0; y2d_modifier = 0.0;
        switch(pos) {
            case 0:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 1; break;
            case 1:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 0; break;
            case 2:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 0; add_height = 1; break;
            case 3:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 1; break;
            case 4:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 0; add_height = 1; break;
            case 6:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 1; break;
            case 7:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 0; break;
            case 8:
                y2d = (double) BLOB_YTOP(blob);     in_height = 0; add_height = 1; break;
        }
    } else if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM)) {
        y2d = (double) BLOB_YTOP(blob); x2d_modifier = 0.0; y2d_modifier = 1.0;
        switch(pos) {
            case 0:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 1; add_height = 1; break;
            case 1:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 1; add_height = 1; break;
            case 2:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 1; add_height = 1; break;
            case 3:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 1; add_height = 0; break;
            case 4:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 1; add_height = 0; break;
            case 6:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 0; add_height = 1; break;
            case 7:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 0; add_height = 1; break;
            case 8:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 0; add_height = 1; break;
        }
    } else if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT)) {
        x2d = (double) BLOB_XRIGHT(blob); x2d_modifier = - 1.0; y2d_modifier = 0.0;
        switch(pos) {
            case 0:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 0; add_height = 1; break;
            case 1:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 0; break;
            case 2:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 1; break;
            case 3:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 0; add_height = 1; break;
            case 4:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 1; break;
            case 6:
                y2d = (double) BLOB_YTOP(blob);     in_height = 0; add_height = 1; break;
            case 7:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 0; break;
            case 8:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 1; break;
        }
    } else if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP)) {
        y2d = (double) BLOB_YBOTTOM(blob); x2d_modifier = 0.0; y2d_modifier = - 1.0;
        switch(pos) {
            case 0:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 0; add_height = 1; break;
            case 1:
                x2d = (double) BLOB_XCENTER(blob); in_height = 0; add_height = 1; break;
            case 2:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 0; add_height = 1; break;
            case 3:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 1; add_height = 0; break;
            case 4:
                x2d = (double) BLOB_XCENTER(blob); in_height = 1; add_height = 0; break;
            case 5:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 1; add_height = 0; break;
            case 6:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 1; add_height = 1; break;
            case 7:
                x2d = (double) BLOB_XCENTER(blob); in_height = 1; add_height = 1; break;
            case 8:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 1; add_height = 1; break;
        }
    }
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d,                y2d,                in_height*model_hmax, &x0, &y0);
    //Get world referential angle for 0 degrees in image referencial
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + x2d_modifier, y2d + y2d_modifier, in_height*model_hmax, &x1, &y1);
    slope = (y1 - y0)/(x1 - x0);
    b = y1 - slope * x1;
    x_dir = x1 > x0 ? 1 : -1;

    //Calculate the point at a distance of the hypotenuse from init point
    get_xy_k_distant_from_init_point(&x1, &y1, x0, y0, slope, b, x_dir, hyp);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x1, y1, in_height*model_hmax, &x2d, &y2d);

    //Check if projection of height adds some distance
    if(add_height) {
        double x2d_with_height, y2d_with_height;
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x1, y1, (1 - in_height)*model_hmax, &x2d_with_height, &y2d_with_height);
        if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT))
            x2d = (x2d < x2d_with_height) ? x2d : x2d_with_height;
        else if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT))
            x2d = (x2d > x2d_with_height) ? x2d : x2d_with_height;
        else if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM))
            y2d = (y2d > y2d_with_height) ? y2d : y2d_with_height;
        else if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP))
            y2d = (y2d < y2d_with_height) ? y2d : y2d_with_height;
    }

    if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT))
        new_distance = BLOB_XRIGHT(blob) - x2d;
    else if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT))
        new_distance = x2d - BLOB_XLEFT(blob);
    else if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM))
        new_distance = y2d - BLOB_YTOP(blob);
    else if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP))
        new_distance = BLOB_YBOTTOM(blob)- y2d;

    return new_distance;  
}

//Returns the blob limits according to different angles and mean model dimensions
void ReliabilityClassification::limits_by_mean_model_values(double *mean_limits, double *starting_angles, Blob *blob, DetectionProblemType dptype) {
    double x[4], y[4];
    double x2d, y2d;
    double slope, b;
    int x_dir, pos, in_height, add_height;
    double x2d_modifier, y2d_modifier;
    double hyp = sqrt(model_lmean*model_lmean + model_wmean*model_wmean);

    pos = BLOB_POSITION(blob);

    if(dptype == MM_RIGHT_OCCL) {
        x2d = (double) BLOB_XLEFT(blob); x2d_modifier = 1.0; y2d_modifier = 0.0;
        switch(pos) {
            case 0:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 1; break;
            case 1:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 0; break;
            case 2:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 0; add_height = 1; break;
            case 3:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 1; break;
            case 4:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 0; add_height = 1; break;
            case 6:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 1; break;
            case 7:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 0; break;
            case 8:
                y2d = (double) BLOB_YTOP(blob);     in_height = 0; add_height = 1; break;
        }
    } else if(dptype == MM_BOTTOM_OCCL) {
        y2d = (double) BLOB_YTOP(blob); x2d_modifier = 0.0; y2d_modifier = 1.0;
        switch(pos) {
            case 0:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 1; add_height = 1; break;
            case 1:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 1; add_height = 1; break;
            case 2:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 1; add_height = 1; break;
            case 3:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 1; add_height = 0; break;
            case 4:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 1; add_height = 0; break;
            case 6:
                x2d = (double) BLOB_XRIGHT(blob);   in_height = 0; add_height = 1; break;
            case 7:
                x2d = (double) BLOB_XCENTER(blob);  in_height = 0; add_height = 1; break;
            case 8:
                x2d = (double) BLOB_XLEFT(blob);    in_height = 0; add_height = 1; break;
        }
    } else if(dptype == MM_LEFT_OCCL) {
        x2d = (double) BLOB_XRIGHT(blob); x2d_modifier = - 1.0; y2d_modifier = 0.0;
        switch(pos) {
            case 0:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 0; add_height = 1; break;
            case 1:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 0; break;
            case 2:
                y2d = (double) BLOB_YTOP(blob);     in_height = 1; add_height = 1; break;
            case 3:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 0; add_height = 1; break;
            case 4:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 0; break;
            case 5:
                y2d = (double) BLOB_YCENTER(blob);  in_height = 1; add_height = 1; break;
            case 6:
                y2d = (double) BLOB_YTOP(blob);     in_height = 0; add_height = 1; break;
            case 7:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 0; break;
            case 8:
                y2d = (double) BLOB_YBOTTOM(blob);  in_height = 1; add_height = 1; break;
        }
    } else if(dptype == MM_TOP_OCCL) {
        y2d = (double) BLOB_YBOTTOM(blob); x2d_modifier = 0.0; y2d_modifier = - 1.0;
        switch(pos) {
            case 0:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 0; add_height = 1; break;
            case 1:
                x2d = (double) BLOB_XCENTER(blob); in_height = 0; add_height = 1; break;
            case 2:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 0; add_height = 1; break;
            case 3:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 1; add_height = 0; break;
            case 4:
                x2d = (double) BLOB_XCENTER(blob); in_height = 1; add_height = 0; break;
            case 5:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 1; add_height = 0; break;
            case 6:
                x2d = (double) BLOB_XLEFT(blob);   in_height = 1; add_height = 1; break;
            case 7:
                x2d = (double) BLOB_XCENTER(blob); in_height = 1; add_height = 1; break;
            case 8:
                x2d = (double) BLOB_XRIGHT(blob);  in_height = 1; add_height = 1; break;
        }
    }

    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d,                y2d,                in_height*model_hmean, &x[0], &y[0]);
    //Get world referential angle for the direction to occluded dimension
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), x2d + x2d_modifier, y2d + y2d_modifier, in_height*model_hmean, &x[1], &y[1]);
    if(x[1] == x[0]) {
        if(y[1] > y[0]) {
            y[1] = y[0] + hyp;
            y[2] = y[0] + model_wmean;
            y[3] = y[0] + model_lmean;
        } else {
            y[1] = y[0] - hyp;
            y[2] = y[0] - model_wmean;
            y[3] = y[0] - model_lmean;
        }
    } else {
        slope = (y[1] - y[0])/(x[1] - x[0]);
        b = y[1] - slope * x[1];
        x_dir = x[1] > x[0] ? 1 : -1;
        //Calculate the point at a distance of the mean model value for w from init point
        get_xy_k_distant_from_init_point(&x[1], &y[1], x[0], y[0], slope, b, x_dir, model_wmean);
        //Calculate the point at a distance of the 2ean model value for l from init point
        get_xy_k_distant_from_init_point(&x[2], &y[2], x[0], y[0], slope, b, x_dir, model_lmean);
        //Calculate the point at a distance of the hypotenuse from init point
        get_xy_k_distant_from_init_point(&x[3], &y[3], x[0], y[0], slope, b, x_dir, hyp);
    }

    int i;
    double x2d_with_height, y2d_with_height;

    for(i=1; i<4; i++) {
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[i], y[i], in_height*model_hmean, &x2d, &y2d);
        //Check if projection of height adds some distance
        if(add_height) {
            SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[i], y[i], (1 - in_height)*model_hmean, &x2d_with_height, &y2d_with_height);
            if(dptype == MM_LEFT_OCCL)
                x2d = (x2d < x2d_with_height) ? x2d : x2d_with_height;
            else if(dptype == MM_RIGHT_OCCL)
                x2d = (x2d > x2d_with_height) ? x2d : x2d_with_height;
            else if(dptype == MM_BOTTOM_OCCL)
                y2d = (y2d > y2d_with_height) ? y2d : y2d_with_height;
            else if(dptype == MM_TOP_OCCL)
                y2d = (y2d < y2d_with_height) ? y2d : y2d_with_height;
        }
        if(dptype == MM_LEFT_OCCL || dptype == MM_RIGHT_OCCL)
            mean_limits[i-1] = x2d;
        else
            mean_limits[i-1] = y2d;
        starting_angles[i-1] = atan2(y[i]-y[0], x[i]-x[0]);
    }

    double aux_limit, aux_angle;
    int mindex = 0; 

    if((dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) || (dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM))) {
        for(i=1; i<3; i++)
            if(mean_limits[i] < mean_limits[0])
                mindex = i;
        if(mindex > 0) {
            aux_angle = starting_angles[0];
            aux_limit = mean_limits[0];
            starting_angles[0] = starting_angles[mindex];
            mean_limits[0] = mean_limits[mindex];
            starting_angles[mindex] = aux_angle;
            mean_limits[mindex] = aux_limit;
        }
        if(mean_limits[1] > mean_limits[2]) {
            aux_angle = starting_angles[1];
            aux_limit = mean_limits[1];
            starting_angles[1] = starting_angles[2];
            mean_limits[1] = mean_limits[2];
            starting_angles[2] = aux_angle;
            mean_limits[2] = aux_limit;
        }
    } else {
        for(i=1; i<3; i++)
            if(mean_limits[i] > mean_limits[0])
                mindex = i;
        if(mindex > 0) {
            aux_angle = starting_angles[0];
            aux_limit = mean_limits[0];
            starting_angles[0] = starting_angles[mindex];
            mean_limits[0] = mean_limits[mindex];
            starting_angles[mindex] = aux_angle;
            mean_limits[mindex] = aux_limit;
        }
        if(mean_limits[1] < mean_limits[2]) {
            aux_angle = starting_angles[1];
            aux_limit = mean_limits[1];
            starting_angles[1] = starting_angles[2];
            mean_limits[1] = mean_limits[2];
            starting_angles[2] = aux_angle;
            mean_limits[2] = aux_limit;
        }
    }

    mean_limits[3] = mean_limits[2];
    starting_angles[3] = MobileObject::NormalizeOrientation(starting_angles[2] + M_PI/2.0);

}

//Returns the blob limits according to different angles and mean model dimensions
//TO-DO!!! Use limits information to have an extra 2D limit if an estimation
//depasses one of those limits, the problem can become a three-2D dimensions
//problem now, or, if estimation depasses limits for vertical and horizontal
//occlusion it becomes a normal classification problem with the 2 new blob
//limits used for the blob, allowing also to have the certainty of the
//impossibility of finding a solution for the analysed type and subtype.
void ReliabilityClassification::limits_by_mean_model_values_bottom_occlusion(
    double *mean_limits, double *mean_left_limits, double *mean_right_limits,
    double *starting_angles, Blob *blob, bool with_left_occlusion,
    Rectangle<int> *mean_bboxes, parallelpiped_t meanparallelpipeds) {
    double x[4], y[4], z, xaux, yaux, zaux, Xaux, Yaux, Yaux2, Yaux3;
    double Ya, Xb, Yb;
    double D;
    Rectangle<int> *current_bbox;
    int pos;
    int
        W_2 = BLOB_WIDTH(blob)/2,
        H_2 = BLOB_HEIGHT(blob)/2,
        L = BLOB_XLEFT(blob),
        R = BLOB_XRIGHT(blob),
        T = BLOB_YTOP(blob),
        B = BLOB_YBOTTOM(blob);

    //Prevent errors for too little blobs
    if(W_2 == 0)
        W_2 = 1;
    if(H_2 == 0)
        H_2 = 1;

    pos = BLOB_POSITION(blob);

    if(with_left_occlusion) {
        Ya = T; Xb = R; Yb = B;
        switch(pos) {
            case 0: case 3:
                //Get starting alpha angles
                k4  = p12 - p22*Yb;
                k8  = p22*Ya - p12;
                k24 = p01 - Xb*p21;
                k25 = p00 - Xb*p20;
                k26 = p03 - Xb*p23;
                k33 = p02 - Xb*p22;

                //1. Get angle parallel to bbox top limit
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L,       T, model_hmean, &x[0], &y[0]);
                //	starting_angles[0] = M_PI*(1 - beta_direction)/2.0 + atan2(y[3] - y[0], beta_direction*(x[3] - x[0]));
                starting_angles[0] = M_PI*(1 - beta_direction)/2.0 + atan2(y[3] - y[0], beta_direction*(x[3] - x[0]));
                starting_angles[1] = starting_angles[0] + M_PI/2.0;
                starting_angles[2] = starting_angles[0] + M_PI/4.0;
                starting_angles[3] = starting_angles[2] + M_PI/2.0;

                sina = sin(starting_angles[0]);
                cosa = cos(starting_angles[0]);

                //1. Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x2,y2 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[2] = x[3] + beta_direction*model_wmean*sina;
                y[2] = y[3] - model_wmean*cosa;

                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[2] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[2] = - (k25*x[2] + k33*z + k26)/k24;
                x[3] = x[2] - beta_direction*model_wmean*sina;
                y[3] = y[2] + model_wmean*cosa;

                //We know that there is an intersection of (x2,y2) at Yb. Now is needed to know where (x3,y3) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x3,y3) is in higher position given this angle, so use x3, y3 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;

                    if(zaux < 0) {  //If solution gives a negative height in intersection, the bbox right limit is correct
                                    //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                    //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }

                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;

                } else { //In this case is (x1,y1) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[2] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[2] = - (k24*y[2] + k26) / k25;
                    }
                    x[3] = x[2] - beta_direction*model_wmean*sina;
                    y[3] = y[2] + model_wmean*cosa;
                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[0], &meanparallelpipeds[0], model_hmean);
                current_bbox = &mean_bboxes[0];
                mean_limits[0]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[0]  = RECT_XLEFT(current_bbox);
                mean_right_limits[0] = RECT_XRIGHT(current_bbox);


                //2. Get bbox for second angle, rotated in 90 degrees
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[2] = x[3] + beta_direction*model_lmean*sina;
                y[2] = y[3] - model_lmean*cosa;

                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[2] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[2] = - (k25*x[2] + k33*z + k26)/k24;
                x[3] = x[2] - beta_direction*model_lmean*sina;
                y[3] = y[2] + model_lmean*cosa;

                //We know that there is an intersection of (x2,y2) at Yb. Now is needed to know where (x3,y3) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x3,y3) is in higher position given this angle, so use x3, y3 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;

                    if(zaux < 0) {  //If solution gives a negative height in intersection, the bbox right limit is correct
                                    //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                    //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }

                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;

                } else { //In this case is (x2,y2) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[2] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[2] = - (k24*y[2] + k26) / k25;
                    }
                    x[3] = x[2] - beta_direction*model_lmean*sina;
                    y[3] = y[2] + model_lmean*cosa;
                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[1], &meanparallelpipeds[1], model_hmean);
                current_bbox = &mean_bboxes[1];
                mean_limits[1]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[1]  = RECT_XLEFT(current_bbox);
                mean_right_limits[1] = RECT_XRIGHT(current_bbox);


                //3. Decide the initial point for 3rd and 4th angles.
                sina = sin(starting_angles[2]);
                cosa = cos(starting_angles[2]);

                //Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x2,y2 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[2] = x[3] + beta_direction*model_wmean*sina;
                y[2] = y[3] - model_wmean*cosa;

                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[2] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[2] = - (k25*x[2] + k33*z + k26)/k24;
                x[3] = x[2] - beta_direction*model_wmean*sina;
                y[3] = y[2] + model_wmean*cosa;

                //We know that there is an intersection of (x2,y2) at Yb. Now is needed to know where (x3,y3) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x3,y3) is in higher position given this angle, so use x3, y3 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;
                    if(zaux < 0) {  //If solution gives a negative height in intersection, the bbox right limit is correct
                                    //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                    //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }

                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;

                } else { //In this case is (x1,y1) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[2] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[2] = - (k24*y[2] + k26) / k25;
                    }
                    x[3] = x[2] - beta_direction*model_wmean*sina;
                    y[3] = y[2] + model_wmean*cosa;
                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[2], &meanparallelpipeds[2], model_hmean);
                current_bbox = &mean_bboxes[2];
                mean_limits[2]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[2]  = RECT_XLEFT(current_bbox);
                mean_right_limits[2] = RECT_XRIGHT(current_bbox);


                //2. Get bbox for second angle, rotated in 90 degrees
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[2] = x[3] + beta_direction*model_lmean*sina;
                y[2] = y[3] - model_lmean*cosa;

                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[2] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[2] = - (k25*x[2] + k33*z + k26)/k24;
                x[3] = x[2] - beta_direction*model_lmean*sina;
                y[3] = y[2] + model_lmean*cosa;

                //We know that there is an intersection of (x2,y2) at Yb. Now is needed to know where (x3,y3) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[2], y[2], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x3,y3) is in higher position given this angle, so use x3, y3 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;
                    if(zaux < 0) {  //If solution gives a negative height in intersection, the bbox right limit is correct
                                    //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                    //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }

                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;

                } else { //In this case is (x2,y2) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[2] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[2] = - (k24*y[2] + k26) / k25;
                    }
                    x[3] = x[2] - beta_direction*model_lmean*sina;
                    y[3] = y[2] + model_lmean*cosa;
                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[3], &meanparallelpipeds[3], model_hmean);
                current_bbox = &mean_bboxes[3];
                mean_limits[3]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[3]  = RECT_XLEFT(current_bbox);
                mean_right_limits[3] = RECT_XRIGHT(current_bbox);

                break;
            case 1: case 4: case 2: case 5:
                //Get starting alpha angles
                k8  = p22*Ya - p12;
                k24 = p01 - Xb*p21;
                k25 = p00 - Xb*p20;
                k27 = p10 - Ya*p20;
                k28 = p11 - Ya*p21;
                k34 = p13 - Ya*p23;

                //1. Get angle parallel to bbox top limit
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                starting_angles[0] = M_PI*(1 - beta_direction)/2.0 + atan2(y[3] - y[0], beta_direction*(x[3] - x[0]));
                starting_angles[1] = starting_angles[0] + M_PI/2.0;
                starting_angles[2] = starting_angles[0] + M_PI/4.0;
                starting_angles[3] = starting_angles[2] + M_PI/2.0;

                //1. For first angle
                sina = sin(starting_angles[0]);
                cosa = cos(starting_angles[0]);
                Parallelpiped::initLimits(limits, nlimits, varlimrel);

                //Check the type of intersection for initial point
                if(    ( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) <= 0)  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 3)                      // or normal case
                    || ( Case == 4 && sp_var != 3 && (sp_var+2)%4 != 3) ) { // or normal case
                    //Determine the line touching the top and right limits
                    if(limits[3][2]) { //variable 3 touches right
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean +(k24*sina + k25*beta_direction*cosa)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_lmean*cosa;
                        y[3] = y[0] + model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    } else {
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k24*cosa - beta_direction*sina*k25)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[0] = x[3] - beta_direction*model_lmean*cosa;
                        y[0] = y[3] - model_lmean*sina;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;

                        //	    int i,j;
                        /*	    AppendToLog("ERROR!!! bottom_occlusion: Still not implemented!!!!");
                                    AppendToLog("Limits:");
                                    for(i=0; i<4; i++) {
                                        for(j=0; j<4; j++)
                                            AppendToLog(limits[i][j] + " ";
                                        AppendToLog(std::endl;
                                    }
                                    AppendToLog(std::endl;*/
                    }
                } else { //The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }
                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[0], &meanparallelpipeds[0], model_hmean);
                current_bbox = &mean_bboxes[0];
                mean_limits[0]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[0]  = RECT_XLEFT(current_bbox);
                mean_right_limits[0] = RECT_XRIGHT(current_bbox);


                //2. Get bbox for second angle, rotated in 90 degrees. Use the same angle and calculations for inverted
                //   wmean and lmean and then swap (Case already calculated).
                if(    ( Case <= 0 )  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 3)               // or normal case
                    || ( Case == 4 && sp_var != 3 && (sp_var+2)%4 != 3) ) { // or special case, but not concerned initial point
                    //Determine the line touching the top and right limits
                    if(limits[3][2]) { //variable 3 touches right
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean +(k24*sina + k25*beta_direction*cosa)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_wmean*cosa;
                        y[3] = y[0] + model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    } else {
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k24*cosa - beta_direction*sina*k25)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[0] = x[3] - beta_direction*model_wmean*cosa;
                        y[0] = y[3] - model_wmean*sina;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    }
                } else { //The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;
	
                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[1], &meanparallelpipeds[1], model_hmean);
                current_bbox = &mean_bboxes[1];
                mean_limits[1]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[1]  = RECT_XLEFT(current_bbox);
                mean_right_limits[1] = RECT_XRIGHT(current_bbox);

                //3. Decide the initial point for 3rd and 4th angles.
                sina = sin(starting_angles[2]);
                cosa = cos(starting_angles[2]);
                Parallelpiped::initLimits(limits, nlimits, varlimrel);

                //Check the type of intersection for initial point
                if(    ( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) <= 0)  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 3)                      // or normal case
                    || ( Case == 4 && sp_var != 3 && (sp_var+2)%4 != 3) ) { // or normal case
                    //Determine the line touching the top and right limits
                    if(limits[3][2]) { //variable 3 touches right
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean +(k24*sina + k25*beta_direction*cosa)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_lmean*cosa;
                        y[3] = y[0] + model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    } else {
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k24*cosa - beta_direction*sina*k25)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[0] = x[3] - beta_direction*model_lmean*cosa;
                        y[0] = y[3] - model_lmean*sina;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    }
                } else { //The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[2], &meanparallelpipeds[2], model_hmean);
                current_bbox = &mean_bboxes[2];
                mean_limits[2]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[2]  = RECT_XLEFT(current_bbox);
                mean_right_limits[2] = RECT_XRIGHT(current_bbox);

                //4. Get bbox for second angle, rotated in 90 degrees. Use the same angle and calculations for inverted
                //   wmean and lmean and then swap (Case already calculated).

                if(    ( Case <= 0 )  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 3)               // or normal case
                    || ( Case == 4 && sp_var != 3 && (sp_var+2)%4 != 3) ) { // or special case, but not concerned initial point
                    //Determine the line touching the top and right limits
                    if(limits[3][2]) { //variable 3 touches right
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean +(k24*sina + k25*beta_direction*cosa)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_wmean*cosa;
                        y[3] = y[0] + model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    } else {
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k24*cosa - beta_direction*sina*k25)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[0] = x[3] - beta_direction*model_wmean*cosa;
                        y[0] = y[3] - model_wmean*sina;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    }
                } else { //The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;
	
                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[3], &meanparallelpipeds[3], model_hmean);
                current_bbox = &mean_bboxes[3];
                mean_limits[3]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[3]  = RECT_XLEFT(current_bbox);
                mean_right_limits[3] = RECT_XRIGHT(current_bbox);
                break;

            default:
                AppendToLog("Case not yet implemented for bottom occlusion.");
        }
    } else { //end of with_left_occlusion
        Ya = T; Xb = L; Yb = B;
        switch(pos) {
            case 2: case 5:
                //Get starting alpha angles
                k4  = p12 - p22*Yb;
                k8  = p22*Ya - p12;
                k24 = p01 - Xb*p21;
                k25 = p00 - Xb*p20;
                k26 = p03 - Xb*p23;
                k33 = p02 - Xb*p22;

                //Get angle parallel to bbox top limit
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R      , T, model_hmean, &x[3], &y[3]);
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R - W_2, T, model_hmean, &x[0], &y[0]);
                starting_angles[0] = M_PI*(1 - beta_direction)/2.0 + atan2(y[3] - y[0], beta_direction*(x[3] - x[0]));
                starting_angles[1] = starting_angles[0] + M_PI/2.0;
                starting_angles[2] = starting_angles[0] + M_PI/4.0;
                starting_angles[3] = starting_angles[2] + M_PI/2.0;

                sina = sin(starting_angles[0]);
                cosa = cos(starting_angles[0]);

                //1. Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x1,y1 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[0], &y[0]);
                x[1] = x[0] + beta_direction*model_wmean*sina;
                y[1] = y[0] - model_wmean*cosa;
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[1], y[1], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[1] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[1] = - (k25*x[1] + k33*z + k26)/k24;
                x[0] = x[1] - beta_direction*model_wmean*sina;
                y[0] = y[1] + model_wmean*cosa;

                //We know that there is an intersection of (x1,y1) at Yb. Now is needed to know where (x0,y0) intersects;
                zaux = - (k25*x[0] + k24*y[0] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[1], y[1], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x0,y0) is in higher position given this angle, so use x0, y0 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[0] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[0] = - (k25*x[0] + k33*zaux + k26)/k24;
                    if(zaux < 0) {  //If solution gives a negative height in intersection, the bbox right limit is correct
                                    //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                    //the right limit of blob.
                        y[0] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[0] = - (k24*y[0] + k26) / k25;
                    }

                    x[3] = x[0] + beta_direction*model_lmean*cosa;
                    y[3] = y[0] + model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                } else { //In this case is (x1,y1) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x1,y1,0) intersects
                                //the right limit of blob.
                        y[1] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[1] = - (k24*y[1] + k26) / k25;
                    }
                    x[0] = x[1] - beta_direction*model_wmean*sina;
                    y[0] = y[1] + model_wmean*cosa;
                    x[3] = x[0] + beta_direction*model_lmean*cosa;
                    y[3] = y[0] + model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[0], &meanparallelpipeds[0], model_hmean);
                current_bbox = &mean_bboxes[0];
                mean_limits[0]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[0]  = RECT_XLEFT(current_bbox);
                mean_right_limits[0] = RECT_XRIGHT(current_bbox);


                //2. The starting point changes as the distance between x0 and x1 changes
                //Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x1,y1 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[0], &y[0]);
                x[1] = x[0] + beta_direction*model_lmean*sina;
                y[1] = y[0] - model_lmean*cosa;
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[1], y[1], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[1] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[1] = - (k25*x[1] + k33*z + k26)/k24;
                x[0] = x[1] - beta_direction*model_lmean*sina;
                y[0] = y[1] + model_lmean*cosa;

                //We know that there is an intersection of (x1,y1) at Yb. Now is needed to know where (x0,y0) intersects;
                zaux = - (k25*x[0] + k24*y[0] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[1], y[1], z,    &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], zaux, &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x0,y0) is in higher position given this angle, so use x0, y0 and recalculate for new angle
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[0] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[0] = - (k25*x[0] + k33*zaux + k26)/k24;
                    if(zaux < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                   //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                   //the right limit of blob.
                        y[0] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[0] = - (k24*y[0] + k26) / k25;
                    }

                    x[3] = x[0] + beta_direction*model_wmean*cosa;
                    y[3] = y[0] + model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                } else { //In this case is (x1,y1) point which intersects
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x1,y1,0) intersects
                                //the right limit of blob.
                        y[1] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[1] = - (k24*y[1] + k26) / k25;
                    }
                    x[0] = x[1] - beta_direction*model_lmean*sina;
                    y[0] = y[1] + model_lmean*cosa;
                    x[3] = x[0] + beta_direction*model_wmean*cosa;
                    y[3] = y[0] + model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[1], &meanparallelpipeds[1], model_hmean);
                current_bbox = &mean_bboxes[1];
                mean_limits[1]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[1]  = RECT_XLEFT(current_bbox);
                mean_right_limits[1] = RECT_XRIGHT(current_bbox);


                //3. Decide the initial point for 3rd and 4th angles.
                sina = sin(starting_angles[2]);
                cosa = cos(starting_angles[2]);

                //Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x0,y0 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[0] = x[3] - beta_direction*model_lmean*cosa;
                y[0] = y[3] - model_lmean*sina;
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[0] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[0] = - (k25*x[0] + k33*z + k26)/k24;
                x[3] = x[0] + beta_direction*model_lmean*cosa;
                y[3] = y[0] + model_lmean*sina;

                //We know that there is an intersection of (x1,y1) at Yb. Now is needed to know where (x0,y0) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], z,    &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x0,y0) is in higher position given this angle, so use x0, y0 and recalculate for new angle
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[0] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[0] = - (k24*y[0] + k26) / k25;
                    }

                    x[3] = x[0] + beta_direction*model_lmean*cosa;
                    y[3] = y[0] + model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                } else { //In this case is (x3,y3) point which intersects
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;
                    if(zaux < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                   //In this case, the intersection point must be recalculated, knowing that (x3,y3,0) intersects
                                   //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }
                    x[0] = x[3] - beta_direction*model_lmean*cosa;
                    y[0] = y[3] - model_lmean*sina;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[2], &meanparallelpipeds[2], model_hmean);
                current_bbox = &mean_bboxes[2];
                mean_limits[2]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[2]  = RECT_XLEFT(current_bbox);
                mean_right_limits[2] = RECT_XRIGHT(current_bbox);


                //4. The starting point changes as the distance between x2 and x3 changes
                //Make test for determining the segment in highest position at right bbox limit
                //Obtain the Y line where x0,y0 should be
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L + W_2, T, model_hmean, &x[3], &y[3]);
                x[0] = x[3] - beta_direction*model_wmean*cosa;
                y[0] = y[3] - model_wmean*sina;
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], model_hmean, &Xaux, &Yaux);

                D = ((p22*Xb - p02)*k1 + k3*k4)*Yaux + (p02*Yb - p12*Xb)*k1 - k2*k4;
                z = ( (k1*Xb - k3*Yb + k2)*(p22*Yaux - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Yaux) ) / D;
                x[0] = (k13 - (k5*Xb + k6*Yb + k7)*z - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                y[0] = - (k25*x[0] + k33*z + k26)/k24;
                x[3] = x[0] + beta_direction*model_wmean*cosa;
                y[3] = y[0] + model_wmean*sina;

                //We know that there is an intersection of (x1,y1) at Yb. Now is needed to know where (x0,y0) intersects;
                zaux = - (k25*x[3] + k24*y[3] + k26) / k33;

                //Get intersections with right limit
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[3], y[3], zaux, &Xaux, &Yaux3);
                SceneModel::worldToImgCoords(SM_CALIB_MATRIX(m_context), x[0], y[0], z,    &Xaux, &Yaux2);
                if(Yaux2 < Yaux3) { //In this case (x0,y0) is in higher position given this angle, so use x0, y0 and recalculate for new angle
                    if(z < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                //In this case, the intersection point must be recalculated, knowing that (x2,y2,0) intersects
                                //the right limit of blob.
                        y[0] = (k18*Yaux + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Yaux + k2);
                        x[0] = - (k24*y[0] + k26) / k25;
                    }
                    x[3] = x[0] + beta_direction*model_wmean*cosa;
                    y[3] = y[0] + model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                } else { //In this case is (x3,y3) point which intersects
                    D = ((p22*Xb - p02)*k1 + k3*k4)*Ya + (p02*Yb - p12*Xb)*k1 - k2*k4;
                    zaux = ( (k1*Xb - k3*Yb + k2)*(p22*Ya - p12)*model_hmean + (k3*p13 - k2*p23 - k1*p03)*(Yb - Ya) ) / D;
                    x[3] = (k13 - (k5*Xb + k6*Yb + k7)*zaux - k9*Xb - k10*Yb) / (k3*Yb - k1*Xb - k2);
                    y[3] = - (k25*x[3] + k33*zaux + k26)/k24;
                    if(zaux < 0) { //If solution gives a negative height in intersection, the bbox right limit is correct
                                   //In this case, the intersection point must be recalculated, knowing that (x3,y3,0) intersects
                                   //the right limit of blob.
                        y[3] = (k18*Ya + k17*Xb + k8*k25*model_hmean - k19) / (k1*Xb - k3*Ya + k2);
                        x[3] = - (k24*y[3] + k26) / k25;
                    }
                    x[0] = x[3] - beta_direction*model_wmean*cosa;
                    y[0] = y[3] - model_wmean*sina;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[3], &meanparallelpipeds[3], model_hmean);
                current_bbox = &mean_bboxes[3];
                mean_limits[3]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[3]  = RECT_XLEFT(current_bbox);
                mean_right_limits[3] = RECT_XRIGHT(current_bbox);

                break;
            case 0: case 1: case 3: case 4:
                //Get starting alpha angles
                k8  = p22*Ya - p12;
                k24 = p01 - Xb*p21;
                k25 = p00 - Xb*p20;
                k26 = p03 - Xb*p23;
                k27 = p10 - Ya*p20;
                k28 = p11 - Ya*p21;
                k33 = p02 - Xb*p22;
                k34 = p13 - Ya*p23;

                //1. Get angle parallel to bbox top limit
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), R, T, model_hmean, &x[3], &y[3]);
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                starting_angles[0] = M_PI*(1 - beta_direction)/2.0 + atan2(y[3] - y[0], beta_direction*(x[3] - x[0]));
                starting_angles[1] = starting_angles[0] + M_PI/2.0;
                starting_angles[2] = starting_angles[0] + M_PI/4.0;
                starting_angles[3] = starting_angles[2] + M_PI/2.0;

                //1. For first angle
                sina = sin(starting_angles[0]);
                cosa = cos(starting_angles[0]);
                Parallelpiped::initLimits(limits, nlimits, varlimrel);

                //Check the type of intersection for initial point
                if(    ( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) <= 0)  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 0)                      // or normal case
                    || ( Case == 4 && sp_var != 0 && (sp_var+2)%4 != 0) ) {        // or special case with interest point not concerned
                    //Determine the line touching the top and left limits
                    if(limits[0][0]) { //variable 0 touches left
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k25*beta_direction*cosa + k24*sina)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[0] = x[3] - beta_direction*model_lmean*cosa;
                        y[0] = y[3] - model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    } else {
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean  - (k24*cosa - k25*beta_direction*sina)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_lmean*cosa;
                        y[3] = y[0] + model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    }
                } else {//The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                    x[3] = x[0] + beta_direction*model_lmean*cosa;
                    y[3] = y[0] + model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[0], &meanparallelpipeds[0], model_hmean);
                current_bbox = &mean_bboxes[0];
                mean_limits[0]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[0]  = RECT_XLEFT(current_bbox);
                mean_right_limits[0] = RECT_XRIGHT(current_bbox);


                //2. Get bbox for second angle, rotated in 90 degrees. Use the same angle and calculations for inverted
                //   wmean and lmean and then swap (Case already calculated).
                if(    ( Case <= 0 )  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 0)               // or normal case
                    || ( Case == 4 && sp_var != 0 && (sp_var+2)%4 != 0) ) { // or special case, but not concerned initial point
                    //Determine the line touching the top and right limits
                    if(limits[0][0]) { //variable 0 touches left
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k25*beta_direction*cosa + k24*sina)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[0] = x[3] - beta_direction*model_wmean*cosa;
                        y[0] = y[3] - model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    } else {
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean  - (k24*cosa - k25*beta_direction*sina)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_wmean*cosa;
                        y[3] = y[0] + model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    }
                } else {//The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                    x[3] = x[0] + beta_direction*model_wmean*cosa;
                    y[3] = y[0] + model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[1], &meanparallelpipeds[1], model_hmean);
                current_bbox = &mean_bboxes[1];
                mean_limits[1]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[1]  = RECT_XLEFT(current_bbox);
                mean_right_limits[1] = RECT_XRIGHT(current_bbox);

                //3. Decide the initial point for 3rd and 4th angles.
                sina = sin(starting_angles[2]);
                cosa = cos(starting_angles[2]);
                Parallelpiped::initLimits(limits, nlimits, varlimrel);

                //Check the type of intersection for initial point
                if(    ( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) <= 0)  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 0)                      // or normal case
                    || ( Case == 4 && sp_var != 0 && (sp_var+2)%4 != 0) ) {        // or special case with interest point not concerned
                    //Determine the line touching the top and left limits
                    if(limits[0][0]) { //variable 0 touches left
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k25*beta_direction*cosa + k24*sina)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[0] = x[3] - beta_direction*model_lmean*cosa;
                        y[0] = y[3] - model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    } else {
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean  - (k24*cosa - k25*beta_direction*sina)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_lmean*cosa;
                        y[3] = y[0] + model_lmean*sina;
                        x[2] = x[3] + beta_direction*model_wmean*sina;
                        y[2] = y[3] - model_wmean*cosa;
                        x[1] = x[0] + beta_direction*model_wmean*sina;
                        y[1] = y[0] - model_wmean*cosa;
                    }
                } else {//The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                    x[3] = x[0] + beta_direction*model_lmean*cosa;
                    y[3] = y[0] + model_lmean*sina;
                    x[2] = x[3] + beta_direction*model_wmean*sina;
                    y[2] = y[3] - model_wmean*cosa;
                    x[1] = x[0] + beta_direction*model_wmean*sina;
                    y[1] = y[0] - model_wmean*cosa;
                }

	
                /*DEBUG
                int i,j;
                AppendToLog("Limits for problem:");
                for(i=0; i<4; i++) {
                  for(j=0; j<4; j++)
                    AppendToLog(limits[i][j] + " ";
                  AppendToLog(std::endl;
                }
                AppendToLog(std::endl;
                */

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[2], &meanparallelpipeds[2], model_hmean);
                current_bbox = &mean_bboxes[2];
                mean_limits[2]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[2]  = RECT_XLEFT(current_bbox);
                mean_right_limits[2] = RECT_XRIGHT(current_bbox);


                //4. Get bbox for fourth angle, rotated in 90 degrees. Use the same angle and calculations for inverted
                //   wmean and lmean and then swap (Case already calculated).
                if(    ( Case <= 0 )  // If non treatable degenerate case
                    || ( Case > 0 && Case < 4 && sp_var != 0)               // or normal case
                    || ( Case == 4 && sp_var != 0 && (sp_var+2)%4 != 0) ) { // or special case, but not concerned initial point
                    //Determine the line touching the top and right limits
                    if(limits[0][0]) { //variable 0 touches left
                        x[3] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean - (k25*beta_direction*cosa + k24*sina)*model_wmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[3] = ( k8*model_hmean - k27*x[3] - k34) / k28;
                        x[0] = x[3] - beta_direction*model_wmean*cosa;
                        y[0] = y[3] - model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    } else {
                        x[0] = (k13 - (k5*Xb + k6*Ya + k7)*model_hmean  - (k24*cosa - k25*beta_direction*sina)*model_lmean*k28 - k9*Xb - k10*Ya) / (k3*Ya - k1*Xb - k2);
                        y[0] = ( k8*model_hmean - k27*x[0] - k34) / k28;
                        x[3] = x[0] + beta_direction*model_wmean*cosa;
                        y[3] = y[0] + model_wmean*sina;
                        x[2] = x[3] + beta_direction*model_lmean*sina;
                        y[2] = y[3] - model_lmean*cosa;
                        x[1] = x[0] + beta_direction*model_lmean*sina;
                        y[1] = y[0] - model_lmean*cosa;
                    }
                } else {//The special case concerns the point of interest
                    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), L, T, model_hmean, &x[0], &y[0]);
                    x[3] = x[0] + beta_direction*model_wmean*cosa;
                    y[3] = y[0] + model_wmean*sina;
                    x[2] = x[3] + beta_direction*model_lmean*sina;
                    y[2] = y[3] - model_lmean*cosa;
                    x[1] = x[0] + beta_direction*model_lmean*sina;
                    y[1] = y[0] - model_lmean*cosa;
                }

                //Swap
                xaux = x[0];
                yaux = y[0];
                memmove(x, x + 1, 3*sizeof(double));
                memmove(y, y + 1, 3*sizeof(double));
                x[3] = xaux;
                y[3] = yaux;

                //Set the limits and  boxes from points
                Blob::set2Dand3Dbboxes(m_context, x, y, &mean_bboxes[3], &meanparallelpipeds[3], model_hmean);
                current_bbox = &mean_bboxes[3];
                mean_limits[3]       = RECT_YBOTTOM(current_bbox);
                mean_left_limits[3]  = RECT_XLEFT(current_bbox);
                mean_right_limits[3] = RECT_XRIGHT(current_bbox);

                break;
            default:
                AppendToLog("Case not yet implemented for bottom occlusion.");
        }
    }
}



//Obtain the k distant 3D point from other point, according to the slope, the intercept of the line where the new point must be and the direction with respect to
//the initial point.
void ReliabilityClassification::get_xy_k_distant_from_init_point(double *xf, double *yf, double xi, double yi, double a, double b, double x_dir, double K) {
    double m1 = a*a + 1, m2 = xi*a - yi + b;
    double Delta = sqrt(K*K*m1 - m2*m2);
    *xf = ( a*(yi - b) + xi + x_dir*Delta )/m1;
    *yf = ( a*(yi*a + xi + x_dir*Delta) + b)/m1;
}

//New version for calculating valid h values for parallelpiped. Erroneous previous version considered that base
//area function was decrecing with respect to h. But this is not the case given the 2D bbox constraint which
//allows parallelpiped base to grow when h increases.
int ReliabilityClassification::calculate_hintervals_simpler(interval_t interval_h_0, interval_t interval_h_90){

    Interval interval_h_from_w, interval_h_from_l;

    //Check limits imposed by w for angle 0

    if(MM1 > 0) //Positive slope for w, if h grows, w grows
        Interval::newInterval(&interval_h_from_w, (model_wmin - MM2) / MM1, (model_wmax - MM2) / MM1);
    else //Negative slope, invert order
        Interval::newInterval(&interval_h_from_w, (model_wmax - MM2) / MM1, (model_wmin - MM2) / MM1);
    
    //First intersect interval from h model limits and the one imposed by w.
    Interval::intersect(interval_h_0, &interval_modelh, &interval_h_from_w);

    if(!INTERVAL_IS_NULL(interval_h_0)) { //If is not yet null, check for l

        if(MM3 > 0) //Positive slope for l, if h grows, l grows
            Interval::newInterval(&interval_h_from_l, (model_lmin - MM4) / MM3, (model_lmax - MM4) / MM3);
        else //Negative slope, invert order
            Interval::newInterval(&interval_h_from_l, (model_lmax - MM4) / MM3, (model_lmin - MM4) / MM3);

        //Second intersect current interval for h with the one imposed by l.
        Interval::intersect(interval_h_0, interval_h_0, &interval_h_from_l);
    }


    //Check limits imposed by w for angle 90. Just invert the w and l limits.

    if(MM1 > 0) //Positive slope for w, if h grows, w grows
        Interval::newInterval(&interval_h_from_w, (model_lmin - MM2) / MM1, (model_lmax - MM2) / MM1);
    else //Negative slope, invert order
        Interval::newInterval(&interval_h_from_w, (model_lmax - MM2) / MM1, (model_lmin - MM2) / MM1);
    
    //First intersect interval from h model limits and the one imposed by w.
    Interval::intersect(interval_h_90, &interval_modelh, &interval_h_from_w);

    if(!INTERVAL_IS_NULL(interval_h_90)) { //If is not yet null, check for l

        if(MM3 > 0) //Positive slope for l, if h grows, l grows
            Interval::newInterval(&interval_h_from_l, (model_wmin - MM4) / MM3, (model_wmax - MM4) / MM3);
        else //Negative slope, invert order
            Interval::newInterval(&interval_h_from_l, (model_wmax - MM4) / MM3, (model_wmin - MM4) / MM3);

      //Second intersect current interval for h with the one imposed by l.
      Interval::intersect(interval_h_90, interval_h_90, &interval_h_from_l);
    }

    if(INTERVAL_IS_NULL(interval_h_0) && INTERVAL_IS_NULL(interval_h_90))
        return 0;
    
    return 1;
}



//Calculate the four intervals where there can be geometrically valid parallelpipeds to be generated.
int ReliabilityClassification::calculate_hintervals(interval_t interval_h_0, interval_t interval_h_90){

    Interval interval_h_from_w, interval_h_from_l;
    double hmin, hmax;

    //Check limits imposed by w for angle 0
    RC_compute_h_from_w(model_wmin, &hmin, NULL);
    RC_compute_h_from_w(model_wmax, &hmax, NULL);

    if(hmax < 0 && hmin < 0)
        Interval::newInterval(&interval_h_from_w, model_hmin, model_hmax);
    else if(hmax > hmin) 
        Interval::newInterval(&interval_h_from_w, hmin, hmax);
    else 
        Interval::newInterval(&interval_h_from_w, hmax, hmin);
    
    //First intersect interval from h model limits and the one imposed by w.
    Interval::intersect(interval_h_0, &interval_modelh, &interval_h_from_w);

    if(!INTERVAL_IS_NULL(interval_h_0)) { //If is not yet null, check for l

        RC_compute_h_from_l(model_lmin, &hmin, NULL);
        RC_compute_h_from_l(model_lmax, &hmax, NULL);
        if(hmax < 0 && hmin < 0)
            Interval::newInterval(&interval_h_from_l, model_hmin, model_hmax);
        else if(hmax > hmin)
            Interval::newInterval(&interval_h_from_l, hmin, hmax);
        else
            Interval::newInterval(&interval_h_from_l, hmax, hmin);

        //Second intersect current interval for h with the one imposed by l.
        Interval::intersect(interval_h_0, interval_h_0, &interval_h_from_l);
    }

    //Check limits imposed by w for angle 90
    RC_compute_h_from_w(model_lmin, &hmin, NULL);
    RC_compute_h_from_w(model_lmax, &hmax, NULL);
    if(hmax < 0 && hmin < 0)
        Interval::newInterval(&interval_h_from_w, model_hmin, model_hmax);
    else if(hmax > hmin) 
        Interval::newInterval(&interval_h_from_w, hmin, hmax);
    else 
        Interval::newInterval(&interval_h_from_w, hmax, hmin);
    
    //First intersect interval from h model limits and the one imposed by w.
    Interval::intersect(interval_h_90, &interval_modelh, &interval_h_from_w);

    if(!INTERVAL_IS_NULL(interval_h_90)) { //If is not yet null, check for l

        RC_compute_h_from_l(model_wmin, &hmin, NULL);
        RC_compute_h_from_l(model_wmax, &hmax, NULL);
        if(hmax < 0 && hmin < 0)
            Interval::newInterval(&interval_h_from_l, model_hmin, model_hmax);
        else if(hmax > hmin)
            Interval::newInterval(&interval_h_from_l, hmin, hmax);
        else
            Interval::newInterval(&interval_h_from_l, hmax, hmin);

        //Second intersect current interval for h with the one imposed by l.
        Interval::intersect(interval_h_90, interval_h_90, &interval_h_from_l);
    }

    if(INTERVAL_IS_NULL(interval_h_0) && INTERVAL_IS_NULL(interval_h_90))
        return 0;
    
    return 1;
}


//Version for tracking with no rotation interval as search is localized. A limit for h is imposed from previous mobile data.
//Calculate the two intervals where there can be geometrically valid parallelpipeds to be generated.
int ReliabilityClassification::calculate_hintervals_for_tracking(interval_t interval_h_0){
    Interval::copyInterval(interval_h_0, &interval_modelh);
    return 1;
}


//OLD VERSIONS
//Calculate the four intervals where there can be geometrically valid parallelpipeds to be generated.
int ReliabilityClassification::calculate_hintervals_old(interval_t interval_h_0, interval_t interval_h_90){

    Interval interval_h, interval_aux;
    double maxh_for_weq, maxh_for_leq, hmin, hmax;
    
    //Set h intervals according to w and l equations
    maxh_for_weq = - wb/wa;
    maxh_for_leq = - lb/la;
    
    if(maxh_for_weq > maxh_for_leq) {
        hmax = maxh_for_weq;
        hmin = maxh_for_leq;
    } else {
        hmax = maxh_for_leq;
        hmin = maxh_for_weq;
    }

    if(hmax < 0)
        interval_h.null();
    else {
        if(hmin < 0)
            hmin = 0.0;
        Interval::newInterval(&interval_h, hmin, hmax);
    }    
        
    if(INTERVAL_IS_NULL(&interval_h))
        return 0;
    
    //Use model limits for h to restrict the existing intervals
    Interval::intersect(&interval_h, &interval_modelh, &interval_h);
    
    if(INTERVAL_IS_NULL(&interval_h))
        return 0;

    //For interval h_0

    //Impose w constraints from model 
    RC_compute_h_from_w(model_wmin, &hmin, NULL);
    RC_compute_h_from_w(model_wmax, &hmax, NULL);
    if(hmin > hmax)
        Interval::newInterval(&interval_aux, hmax, hmin);
    else
        Interval::newInterval(&interval_aux, hmin, hmax);

    Interval::intersect(interval_h_0, &interval_h, &interval_aux);

    if(!INTERVAL_IS_NULL(interval_h_0)) {
        //Impose l constraints from model
        RC_compute_h_from_l(model_lmin, &hmin, NULL);
        RC_compute_h_from_l(model_lmax, &hmax, NULL);
        if(hmin > hmax)
            Interval::newInterval(&interval_aux, hmax, hmin);
        else
            Interval::newInterval(&interval_aux, hmin, hmax);
      
        Interval::intersect(interval_h_0, interval_h_0, &interval_aux);
    }
      
    //For interval h_90

    //Impose w constraints from model, inversed
    RC_compute_h_from_w(model_lmin, &hmin, NULL);
    RC_compute_h_from_w(model_lmax, &hmax, NULL);
    if(hmin > hmax)
        Interval::newInterval(&interval_aux, hmax, hmin);
    else
        Interval::newInterval(&interval_aux, hmin, hmax);

    Interval::intersect(interval_h_90, &interval_h, &interval_aux);

    if(!INTERVAL_IS_NULL(interval_h_90)) {
        //Impose l constraints from model
        RC_compute_h_from_l(model_wmin, &hmin, NULL);
        RC_compute_h_from_l(model_wmax, &hmax, NULL);
        if(hmin > hmax)
            Interval::newInterval(&interval_aux, hmax, hmin);
        else
            Interval::newInterval(&interval_aux, hmin, hmax);
      
        Interval::intersect(interval_h_90, interval_h_90, &interval_aux);
    }
	
    if(INTERVAL_IS_NULL(interval_h_0) && INTERVAL_IS_NULL(interval_h_90))
        return 0;

    return 1;
}

//Version for tracking with no rotation interval as search is localized. A limit for h is imposed from previous mobile data.
//Calculate the two intervals where there can be geometrically valid parallelpipeds to be generated.
int ReliabilityClassification::calculate_hintervals_for_tracking_old(interval_t interval_h_0){

    Interval interval_h, interval_aux;
    double maxh_for_weq, maxh_for_leq, hmin, hmax;

    interval_h_0->null();
    
    //Set h intervals according to w and l equations
    maxh_for_weq = - wb/wa;
    maxh_for_leq = - lb/la;
    
    if(maxh_for_weq > maxh_for_leq) {
        hmax = maxh_for_weq;
        hmin = maxh_for_leq;
    } else {
        hmax = maxh_for_leq;
        hmin = maxh_for_weq;
    }

    if(hmax < 0)
        interval_h.null();
    else {
        if(hmin < 0)
            hmin = 0.0;
        Interval::newInterval(&interval_h, hmin, hmax);
    }    

        
    if(INTERVAL_IS_NULL(&interval_h))
        return 0;
    
    //Use model limits for h to restrict the existing intervals
    Interval::intersect(&interval_h, &interval_modelh, &interval_h);
    
    if(INTERVAL_IS_NULL(&interval_h))
        return 0;

    //For interval h_0

    //Impose w constraints from model 
    RC_compute_h_from_w(model_wmin, &hmin, NULL);
    RC_compute_h_from_w(model_wmax, &hmax, NULL);
    if(hmin > hmax)
        Interval::newInterval(&interval_aux, hmax, hmin);
    else
        Interval::newInterval(&interval_aux, hmin, hmax);

    Interval::intersect(interval_h_0, &interval_h, &interval_aux);

    if(!INTERVAL_IS_NULL(interval_h_0)) {
        //Impose l constraints from model
        RC_compute_h_from_l(model_lmin, &hmin, NULL);
        RC_compute_h_from_l(model_lmax, &hmax, NULL);
        if(hmin > hmax)
            Interval::newInterval(&interval_aux, hmax, hmin);
        else
            Interval::newInterval(&interval_aux, hmin, hmax);
      
        Interval::intersect(interval_h_0, interval_h_0, &interval_aux);
    }
      
    if(INTERVAL_IS_NULL(interval_h_0))
        return 0;

    return 1;

}

//Search for parellelepiped solution for different sizes of a blob, according to the detected types of possible occlusions.
void ReliabilityClassification::fillOcclusionList(Blob *blob) {
    double current_dim, dim_limit, current_result, best_result;
    DetectionProblemType dptype = BLOB_DP_TYPE(blob);
    bool is_vertical   = (dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP   | MM_OBJECT_BOTTOM | MM_OBJECT_TOP  )) ? true : false;
    bool is_horizontal = (dptype & (MM_CAM_OCCL_LEFT   | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT   | MM_OBJECT_RIGHT)) ? true : false;
    bool ascending;
    int num_descending;

    Shape3DData *s3d = currentRigid 
        ? (there_is_best_valid[m_modelId]                  ? best_valid[m_modelId]                   : NULL)
        : (sub_there_is_best_valid[m_modelId][m_subModelId] ? sub_best_valid[m_modelId][m_subModelId] : NULL);
    best_result = (s3d == NULL) ? 0.0 : S3D_P(s3d);
    
    if(is_vertical) {
        double
            top = BLOB_YTOP(blob),
            bottom = BLOB_YBOTTOM(blob);
        bool is_bottom = (dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM)) ? true : false;
        bool is_top    = (dptype & (MM_CAM_OCCL_TOP    | MM_OBJECT_TOP   )) ? true : false;

        if(is_bottom) {
            dim_limit = OcclusionMaxBottom;
            dim_limit = (dim_limit - bottom < m_foreground->height()/2.0 ) ? dim_limit : bottom + m_foreground->height()/2.0;

            ascending = currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId];
            num_descending = 0;

            for(current_dim = bottom + DIM2D_STEP ; current_dim <= dim_limit ; current_dim += DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;

                BLOB_YBOTTOM(blob) = (int) current_dim;
                BLOB_HEIGHT(blob)  = (int) (current_dim - BLOB_YTOP(blob));
	  
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
	  
                //Set blob 3D position (to redo with parallelpiped data)
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0, &BLOB_3D_X(blob), &BLOB_3D_Y(blob));

                RC_set_3D_bbox_blob_level_data(blob);
                current_result = static_occlusion_search_alpha(blob);

                global_models_calculated_counter += models_calculated_counter;

                if(noWallsValidatedSolution())
                    break;

                if(currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId]) {
                    if( current_result > best_result ) {
                        best_result = current_result;
                        ascending = true;
                    } else {
                        if(ascending) {
                            ascending = false;
                            num_descending++;
                            if(num_descending == 2) //When object occluded an bbox starts growing it can happen that the initial search of solution is made in an interval of alpha
                                                    //that does not correspond to the global optima solution. In this case, global result will descend, before start ascending
                                                    //to find the global optimal solution.
                                break;
                        }
                    }
                }
            }

            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }

        if(is_top) { //TOP
            dim_limit = OcclusionMinTop;
            dim_limit = (top - dim_limit < m_foreground->height()/2.0 ) ? dim_limit : top - m_foreground->height()/2.0;

            ascending = currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId];
            num_descending = 0;

            for(current_dim = top - DIM2D_STEP ; current_dim >= dim_limit && BLOB_HEIGHT(blob) <= Hmax  ; current_dim -= DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;

                BLOB_YTOP(blob)   = (int) current_dim;
                BLOB_HEIGHT(blob) = (int) (BLOB_YBOTTOM(blob) - current_dim);

                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
	  
                //Set blob 3D position (to redo with parallelpiped data)
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0, &BLOB_3D_X(blob), &BLOB_3D_Y(blob));

                RC_set_3D_bbox_blob_level_data(blob);
                current_result = static_occlusion_search_alpha(blob);

                global_models_calculated_counter += models_calculated_counter;

                if(noWallsValidatedSolution())
                    break;

                if(currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId]) {
                    if( current_result > best_result ) {
                        best_result = current_result;
                        ascending = true;
                    } else {
                        if(ascending) {
                            ascending = false;
                            num_descending++;
                            if(num_descending == 2) //When object occluded an bbox starts growing it can happen that the initial search of solution is made in an interval of alpha
                                                    //that does not correspond to the global optima solution. In this case, global result will descend, before start ascending
                                                    //to find the global optimal solution.
                                break;
                        }
                    }
                }
            }

            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }
    }

    //Treat separately.
    if(is_horizontal) {
        double
            left = BLOB_XLEFT(blob),
            right = BLOB_XRIGHT(blob);
        bool is_left  = (dptype & (MM_CAM_OCCL_LEFT  | MM_OBJECT_LEFT )) ? true : false;
        bool is_right = (dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) ? true : false;

        if(is_left) {
            dim_limit = OcclusionMinLeft;
            dim_limit = (left - dim_limit < m_foreground->width()/2.0 ) ? dim_limit : left - m_foreground->width()/2.0;

            ascending = currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId];
            num_descending = 0;

            for(current_dim = left - DIM2D_STEP ; current_dim >= dim_limit ; current_dim -= DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;

                BLOB_XLEFT(blob) = (int) current_dim;
                BLOB_WIDTH(blob) = (int) (BLOB_XRIGHT(blob) - current_dim);

                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
	  
                //Set blob 3D position (to redo with parallelpiped data)
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0, &BLOB_3D_X(blob), &BLOB_3D_Y(blob));

                RC_set_3D_bbox_blob_level_data(blob);
                current_result = static_occlusion_search_alpha(blob);

                global_models_calculated_counter += models_calculated_counter;

                if(noWallsValidatedSolution())
                    break;

                if(currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId]) {
                    if( current_result > best_result ) {
                        best_result = current_result;
                        ascending = true;
                    } else {
                        if(ascending) {
                            ascending = false;
                            num_descending++;
                            if(num_descending == 2) //When object occluded an bbox starts growing it can happen that the initial search of solution is made in an interval of alpha
                                                    //that does not correspond to the global optima solution. In this case, global result will descend, before start ascending
                                                    //to find the global optimal solution.
                                break;
                        }
                    }
                }
            }
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }

        if(is_right) { //RIGHT
            dim_limit = OcclusionMaxRight;
            dim_limit = ( dim_limit - right < m_foreground->width()/2.0 ) ? dim_limit : right + m_foreground->width()/2.0;

            ascending = currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId];
            num_descending = 0;

            for(current_dim = right + DIM2D_STEP ; current_dim <= dim_limit && BLOB_WIDTH(blob) <= Wmax ; current_dim += DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;

                BLOB_XRIGHT(blob) = (int) current_dim;
                BLOB_WIDTH(blob)  = (int) (current_dim - BLOB_XLEFT(blob));

                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
	  
                //Set blob 3D position (to redo with parallelpiped data)
                SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(m_context), BLOB_XCENTER(blob), BLOB_YBOTTOM(blob), 0.0, &BLOB_3D_X(blob), &BLOB_3D_Y(blob));

                RC_set_3D_bbox_blob_level_data(blob);
                current_result = static_occlusion_search_alpha(blob);

                global_models_calculated_counter += models_calculated_counter;

                if(noWallsValidatedSolution())
                    break;

                if(currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId]) {
                    if( current_result > best_result ) {
                        best_result = current_result;
                        ascending = true;
                    } else {
                        if(ascending) {
                            ascending = false;
                            num_descending++;
                            if(num_descending == 2) //When object occluded an bbox starts growing it can happen that the initial search of solution is made in an interval of alpha
                                                    //that does not correspond to the global optima solution. In this case, global result will descend, before start ascending
                                                    //to find the global optimal solution.
                                break;
                        }
                    }
                }
            }
        }
    }

    //Restore real blob dimensions
    memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
    W = BLOB_WIDTH(blob);
    H = BLOB_HEIGHT(blob);
    RC_set_3D_bbox_blob_level_data(blob);
}


//This method analyses possible static occlusion solutions taking four different starting points, generated using different starting
//orientations and the most likely object model.
void ReliabilityClassification::static_occlusion_bottom_search_2D_in_corner(double *mean_limits, double *mean_left_limits, double *mean_right_limits,
                                                                            double *starting_angles, Blob *blob, DetectionProblemType ver_dptype,
                                        DetectionProblemType hor_dptype, double ver_dim_limit, double hor_dim_limit) {
    double result, best_result, starting_alpha, current_dim;
    bool active[4] = {true, true, true, true};
    int num_active = 4, i; 
    bool first_solution = true;
    //Save real blob dimensions

    int original_left = BLOB_XLEFT(blob), original_right = BLOB_XRIGHT(blob);

    //Validate limits separetly as order is not warranteed
    
    //First optimize for vertical occlusion
    if(ver_dptype & MM_BOTTOM_OCCL) {
        if( mean_limits[0] > ver_dim_limit ) { active[0] = false; num_active--; }
        if( mean_limits[1] > ver_dim_limit ) { active[1] = false; num_active--; }
        if( mean_limits[2] > ver_dim_limit ) { active[2] = false; num_active--; }
        if( mean_limits[3] > ver_dim_limit ) { active[3] = false; num_active--; }

        if(num_active == 0) {
            double top = BLOB_YTOP(blob), side_value;
            double cur_diff, min_side_diff = DBL_MAX;
            bool selected_side = false;
            //Set dimension for lateral occlusion if possible
            //Select the minimal distance from original blob 2D dimension
            //from all calculated mean parallelpipeds.
            if(hor_dptype & MM_LEFT_OCCL) {
                for(i=0; i<4; i++)
                    if(mean_left_limits[i] < BLOB_XLEFT(blob) && mean_left_limits[i] >= hor_dim_limit) {
                        selected_side = true;
                        if( (cur_diff = BLOB_XLEFT(blob) - mean_left_limits[i]) < min_side_diff) {
                            min_side_diff = cur_diff;
                            side_value = mean_left_limits[i];
                        }
                    }
                if(selected_side) {
                    BLOB_XLEFT(blob) = (int)side_value;
                    BLOB_WIDTH(blob) = (int)(BLOB_XRIGHT(blob) - side_value);
                }
            } else { // MM_RIGHT_OCCL
                for(i=0; i<4; i++)
                    if(mean_right_limits[i] > BLOB_XRIGHT(blob) && mean_right_limits[i] <= hor_dim_limit) {
                        selected_side = true;
                        if( (cur_diff = mean_right_limits[i] - BLOB_XRIGHT(blob)) < min_side_diff) {
                            min_side_diff = cur_diff;
                            side_value = mean_right_limits[i];
                        }
                    }
                if(selected_side) {
                    BLOB_XRIGHT(blob) = (int)side_value;
                    BLOB_WIDTH(blob) = (int)(side_value - BLOB_XLEFT(blob));
                }
            }
	
            for(current_dim = ver_dim_limit; current_dim > BLOB_YBOTTOM(blob); current_dim -= DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                BLOB_YBOTTOM(blob) = (int) current_dim;
                BLOB_HEIGHT(blob) = (int) (current_dim - top);
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
	  
                RC_set_3D_bbox_blob_level_data(blob);
                if(first_solution) {
                    result = static_occlusion_search_alpha(blob);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result > 0.0) {
                        best_result = result;
                        first_solution = false;
                        starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                    } else
                        break;
                } else {
                    result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result < best_result)
                        break;
                    best_result = result;
                }
            }
        } else {
   
            bool not_first_try = false, minactive[4] = {true, true, true, true}, maxactive[4] = {true, true, true, true};
            int current_min[4], current_max[4];
            double minbest[4], maxbest[4], minangle[4], maxangle[4];

            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            BLOB_XLEFT(blob)  = std::min((int) mean_left_limits[i],  original_left);
                            BLOB_XRIGHT(blob) = std::max((int) mean_right_limits[i], original_right);
                            BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);

                            //Top branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= RECT_YBOTTOM(&realBBox))
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YBOTTOM(blob) = current_min[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Bottom branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= ver_dim_limit)
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YBOTTOM(blob) = current_max[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(active[i] == true) {
                            if(mean_limits[i] <= BLOB_YBOTTOM(blob)) {
                                active[i] = false;
                                num_active--;
                                continue;
                            }
                            models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                            BLOB_XLEFT(blob)  = std::min((int) mean_left_limits[i],  original_left);
                            BLOB_XRIGHT(blob) = std::max((int) mean_right_limits[i], original_right);
                            BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                            BLOB_YBOTTOM(blob) = (int) mean_limits[i];
                            BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                            global_models_calculated_counter += models_calculated_counter;
                            if(noWallsValidatedSolution() || result == 0.0) {
                                active[i] = false;
                                num_active--;
                            } else {
                                current_min[i] = current_max[i] = BLOB_YBOTTOM(blob);
                                minbest[i] = maxbest[i] = result;
                                minangle[i] = maxangle[i] = starting_angles[i];
                            }
                        }
                    }
                }
            }
        }
    } else { //For TOP occlusion
        if( mean_limits[0] < ver_dim_limit ) { active[0] = false; num_active--; }
        if( mean_limits[1] < ver_dim_limit ) { active[1] = false; num_active--; }
        if( mean_limits[2] < ver_dim_limit ) { active[2] = false; num_active--; }
        if( mean_limits[3] < ver_dim_limit ) { active[3] = false; num_active--; }

        if(num_active == 0) {
            double bottom = BLOB_YBOTTOM(blob), side_value;
            double cur_diff, min_side_diff = DBL_MAX;
            bool selected_side = false;
            //Set dimension for lateral occlusion if possible
            //Select the minimal distance from original blob 2D dimension
            //from all calculated mean parallelpipeds.
            if(hor_dptype & MM_LEFT_OCCL) {
                for(i=0; i<4; i++)
                    if(mean_left_limits[i] < BLOB_XLEFT(blob) && mean_left_limits[i] >= hor_dim_limit) {
                        selected_side = true;
                        if( (cur_diff = BLOB_XLEFT(blob) - mean_left_limits[i]) < min_side_diff) {
                            min_side_diff = cur_diff;
                            side_value = mean_left_limits[i];
                        }
                    }
                if(selected_side) {
                    BLOB_XLEFT(blob) = (int)side_value;
                    BLOB_WIDTH(blob) = (int)(BLOB_XRIGHT(blob) - side_value);
                }
            } else { // MM_RIGHT_OCCL
                for(i=0; i<4; i++)
                    if(mean_right_limits[i] > BLOB_XRIGHT(blob) && mean_right_limits[i] <= hor_dim_limit) {
                        selected_side = true;
                        if( (cur_diff = mean_right_limits[i] - BLOB_XRIGHT(blob)) < min_side_diff) {
                            min_side_diff = cur_diff;
                            side_value = mean_right_limits[i];
                        }
                    }
                if(selected_side) {
                    BLOB_XRIGHT(blob) = (int)side_value;
                    BLOB_WIDTH(blob) = (int)(side_value - BLOB_XLEFT(blob));
                }
            }

            for(current_dim = ver_dim_limit; current_dim < BLOB_YTOP(blob); current_dim += DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                BLOB_YTOP(blob) = (int) current_dim;
                BLOB_HEIGHT(blob) = (int) (bottom - current_dim);
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);

                RC_set_3D_bbox_blob_level_data(blob);
                if(first_solution) {
                    result = static_occlusion_search_alpha(blob);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result > 0.0) {
                        best_result = result;
                        first_solution = false;
                        starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                    } else
                        break;
                } else {
                    result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result < best_result)
                        break;
                    best_result = result;
                }
            }
        } else {

            bool not_first_try = false, minactive[4] = {true, true, true, true}, maxactive[4] = {true, true, true, true};
            int current_min[4], current_max[4];
            double minbest[4], maxbest[4], minangle[4], maxangle[4];

            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            BLOB_XLEFT(blob)  = std::min((int) mean_left_limits[i],  original_left);
                            BLOB_XRIGHT(blob) = std::max((int) mean_right_limits[i], original_right);
                            BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                            //Top branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= ver_dim_limit)
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YTOP(blob) = current_min[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Bottom branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= RECT_YTOP(&realBBox))
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YTOP(blob) = current_max[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(active[i] == true) {
                            if(mean_limits[i] >= BLOB_YTOP(blob)) {
                                active[i] = false;
                                num_active--;
                                continue;
                            }
                            models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                            BLOB_XLEFT(blob)  = std::min((int) mean_left_limits[i],  original_left);
                            BLOB_XRIGHT(blob) = std::max((int) mean_right_limits[i], original_right);
                            BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                            BLOB_YTOP(blob) = (int) mean_limits[i];
                            BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                            global_models_calculated_counter += models_calculated_counter;
                            if(noWallsValidatedSolution() || result == 0.0) {
                                active[i] = false;
                                num_active--;
                            } else {
                                current_min[i] = current_max[i] = BLOB_YTOP(blob);
                                minbest[i] = maxbest[i] = result;
                                minangle[i] = maxangle[i] = starting_angles[i];
                            }
                        }
                    }
                }
            }
        }
    }
    
    //If until here there is no solutions, we better stop
    if( !(currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId]) )
        return;

    active[0] = true; active[1] = true; active[2] = true; active[3] = true;
    num_active = 4; 

    Shape3DData *best_s3d =  currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId];

    //Now complete analysis with horizontal occlusion, starting from best previous vertical dimensions
    BLOB_YBOTTOM(blob) = S3D_YBOTTOM(best_s3d);
    BLOB_YTOP(blob)    = S3D_YTOP(best_s3d);
    BLOB_HEIGHT(blob)  = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);

    if(hor_dptype == MM_LEFT_OCCL) {
        BLOB_XRIGHT(blob) = S3D_XRIGHT(best_s3d);
            if( mean_left_limits[0] < hor_dim_limit ) { active[0] = false; num_active--; }
            if( mean_left_limits[1] < hor_dim_limit ) { active[1] = false; num_active--; }
            if( mean_left_limits[2] < hor_dim_limit ) { active[2] = false; num_active--; }
            if( mean_left_limits[3] < hor_dim_limit ) { active[3] = false; num_active--; }

            if(num_active == 0) {
                double right = BLOB_XRIGHT(blob);

                for(current_dim = hor_dim_limit; current_dim < BLOB_XLEFT(blob); current_dim += DIM2D_STEP) {

                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                    BLOB_XLEFT(blob) = (int) current_dim;
                    BLOB_WIDTH(blob) = (int) (right - current_dim);
                    W = BLOB_WIDTH(blob);
                    H = BLOB_HEIGHT(blob);
	  
                    RC_set_3D_bbox_blob_level_data(blob);
                    if(first_solution) {
                        result = static_occlusion_search_alpha(blob);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result > 0.0) {
                            best_result = result;
                            first_solution = false;
                            starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                        } else
                        break;
                    } else {
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result < best_result)
                            break;
                        best_result = result;
                    }
                }
                return;
            }

            bool not_first_try = false, minactive[4] = {true, true, true, true}, maxactive[4] = {true, true, true, true};
            int current_min[4], current_max[4];
            double minbest[4], maxbest[4], minangle[4], maxangle[4];

            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            //Left branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                            if(current_min[i] <= hor_dim_limit)
                                minactive[i] = false;
                            else { //Still processing can be made
                                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                BLOB_XLEFT(blob) = current_min[i];
                                BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                W = BLOB_WIDTH(blob);
                                H = BLOB_HEIGHT(blob);
                                RC_set_3D_bbox_blob_level_data(blob);
                                result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                global_models_calculated_counter += models_calculated_counter;
                                if(noWallsValidatedSolution() || result < minbest[i])
                                    minactive[i] = false;
                                else
                                    minbest[i] = result;
                            }
                        }
                        //Right branch
                        if(maxactive[i]) {
                            current_max[i] += DIM2D_STEP;
                            if(current_max[i] >= RECT_XLEFT(&realBBox))
                                maxactive[i] = false;
                            else { //Still processing can be made
                                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                BLOB_XLEFT(blob) = current_max[i];
                                BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                W = BLOB_WIDTH(blob);
                                H = BLOB_HEIGHT(blob);
                                RC_set_3D_bbox_blob_level_data(blob);
                                result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                global_models_calculated_counter += models_calculated_counter;
                                if(noWallsValidatedSolution() || result < maxbest[i])
                                    maxactive[i] = false;
                                else
                                    maxbest[i] = result;
                            }
                        }
                        //Check for branches activity
                        if(!minactive[i] && !maxactive[i]) {
                            active[i] = false;
                            num_active--;
                        }
                    }
                }
            } else { //Initialisation step for each analysis branch
                not_first_try = true;
                for(i=0; i<4; i++) {
                    if(active[i] == true) {
                        if(mean_left_limits[i] >= BLOB_XLEFT(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_XLEFT(blob) = (int) mean_left_limits[i];
                        BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            num_active--;
                            active[i] = false;
                        } else {
                            current_min[i] = current_max[i] = BLOB_XLEFT(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
        }
    } else { //MM_RIGHT_OCCL
        BLOB_XLEFT(blob) = S3D_XLEFT(best_s3d);
        if( mean_right_limits[0] > hor_dim_limit ) { active[0] = false; num_active--; }
        if( mean_right_limits[1] > hor_dim_limit ) { active[1] = false; num_active--; }
        if( mean_right_limits[2] > hor_dim_limit ) { active[2] = false; num_active--; }
        if( mean_right_limits[3] > hor_dim_limit ) { active[3] = false; num_active--; }

        if(num_active == 0) {

            double left = BLOB_XLEFT(blob);

            for(current_dim = hor_dim_limit; current_dim > BLOB_XRIGHT(blob); current_dim -= DIM2D_STEP) {
                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                BLOB_XRIGHT(blob) = (int) current_dim;
                BLOB_WIDTH(blob) = (int) (current_dim - left);
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);

                RC_set_3D_bbox_blob_level_data(blob);
                if(first_solution) {
                    result = static_occlusion_search_alpha(blob);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result > 0.0) {
                        best_result = result;
                        first_solution = false;
                        starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                    } else
                        break;
                } else {
                    result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                    global_models_calculated_counter += models_calculated_counter;
                    if(noWallsValidatedSolution())
                        break;
                    if(result < best_result)
                        break;
                    best_result = result;
                }
            }
            return;
        }

        bool not_first_try = false, minactive[4] = {true, true, true, true}, maxactive[4] = {true, true, true, true};
        int current_min[4], current_max[4];
        double minbest[4], maxbest[4], minangle[4], maxangle[4];
      
        while(num_active > 0) {
            //Initialization step already done
            if(not_first_try) {
                for(i=0; i<4; i++) {
                    if(active[i]) {
                        //Left branch
                        if(minactive[i]) {
                            current_min[i] -= DIM2D_STEP;
                            if(current_min[i] <= RECT_XRIGHT(&realBBox))
                                minactive[i] = false;
                            else { //Still processing can be made
                                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                BLOB_XRIGHT(blob) = current_min[i];
                                BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                W = BLOB_WIDTH(blob);
                                H = BLOB_HEIGHT(blob);
                                RC_set_3D_bbox_blob_level_data(blob);
                                result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                global_models_calculated_counter += models_calculated_counter;
                                if(noWallsValidatedSolution() || result < minbest[i])
                                    minactive[i] = false;
                                else
                                    minbest[i] = result;
                            }
                        }
                        //Right branch
                        if(maxactive[i]) {
                            current_max[i] += DIM2D_STEP;
                            if(current_max[i] >= hor_dim_limit)
                                maxactive[i] = false;
                            else { //Still processing can be made
                                models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                BLOB_XRIGHT(blob) = current_max[i];
                                BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                W = BLOB_WIDTH(blob);
                                H = BLOB_HEIGHT(blob);
                                RC_set_3D_bbox_blob_level_data(blob);
                                result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                global_models_calculated_counter += models_calculated_counter;
                                if(noWallsValidatedSolution() || result < maxbest[i])
                                    maxactive[i] = false;
                                else
                                    maxbest[i] = result;
                            }
                        }
                        //Check for branches activity
                        if(!minactive[i] && !maxactive[i]) {
                            active[i] = false;
                            num_active--;
                        }
                    }
                }
            } else { //Initialisation step for each analysis branch
                not_first_try = true;
                for(i=0; i<4; i++) {
                    if(active[i] == true) {
                        if(mean_right_limits[i] <= BLOB_XRIGHT(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_XRIGHT(blob) = (int) mean_right_limits[i];
                        BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            active[i] = false;
                            num_active--;
                        } else {
                            current_min[i] = current_max[i] = BLOB_XRIGHT(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
        }
    }
}


//This method analyses possible static occlusion solutions taking four different starting points, generated using different starting
//orientations and the most likely object model.
void ReliabilityClassification::static_occlusion_search_2D(double *mean_limits, double *starting_angles, Blob *blob, DetectionProblemType dptype, double dim_limit) {
    double result, best_result, starting_alpha, current_dim;
    bool active[4] = {true, true, true, true};
    int num_active = 4; 
    bool first_solution = true;

    if(dptype == MM_LEFT_OCCL || dptype == MM_TOP_OCCL) {
        //If the first starting point is out of bounds
        //check from the normal case, to see if solution improves

        if( mean_limits[0] < dim_limit ) { active[0] = false; num_active--; }
        if( mean_limits[1] < dim_limit ) { active[1] = false; num_active--; }
        if( mean_limits[2] < dim_limit ) { active[2] = false; num_active--; }
        if( mean_limits[3] < dim_limit ) { active[3] = false; num_active--; }

        if(num_active == 0) {
            if(dptype == MM_LEFT_OCCL) {
	  
                double right = BLOB_XRIGHT(blob);
	  
                for(current_dim = dim_limit ; current_dim < BLOB_XLEFT(blob) ; current_dim += DIM2D_STEP) {
                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                    BLOB_XLEFT(blob) = (int) current_dim;
                    BLOB_WIDTH(blob) = (int) (right - current_dim);
                    W = BLOB_WIDTH(blob);
                    H = BLOB_HEIGHT(blob);

                    RC_set_3D_bbox_blob_level_data(blob);
                    if(first_solution) {
                        result = static_occlusion_search_alpha(blob);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result > 0.0) {
                            best_result = result;
                            first_solution = false;
                            starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                        } else
                            break;
                    } else {
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result < best_result)
                            break;
                        best_result = result;
                    }
                }
            } else { //TOP
                double bottom = BLOB_YBOTTOM(blob);

                for(current_dim = dim_limit ; current_dim < BLOB_YTOP(blob) ; current_dim += DIM2D_STEP) {
                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                    BLOB_YTOP(blob) = (int) current_dim;
                    BLOB_HEIGHT(blob) = (int) (bottom - current_dim);
                    W = BLOB_WIDTH(blob);
                    H = BLOB_HEIGHT(blob);

                    RC_set_3D_bbox_blob_level_data(blob);
                    if(first_solution) {
                        result = static_occlusion_search_alpha(blob);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result > 0.0) {
                            best_result = result;
                            first_solution = false;
                            starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                        } else
                            break;
                    } else {
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result < best_result)
                            break;
                        best_result = result;
                    }
                }
            }
            return;
        }
    } else { //dptype == MM_RIGHT_OCCL || MM_BOTTOM_OCCL

        if( mean_limits[0] > dim_limit ) { active[0] = false; num_active--; }
        if( mean_limits[1] > dim_limit ) { active[1] = false; num_active--; }
        if( mean_limits[2] > dim_limit ) { active[2] = false; num_active--; }
        if( mean_limits[3] > dim_limit ) { active[3] = false; num_active--; }

        if( num_active == 0 ) {
            if(dptype == MM_RIGHT_OCCL) {
                double left = BLOB_XLEFT(blob);

                for(current_dim = dim_limit ; current_dim > BLOB_XRIGHT(blob) ; current_dim -= DIM2D_STEP) {
                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                    BLOB_XRIGHT(blob) = (int) current_dim;
                    BLOB_WIDTH(blob) = (int) (current_dim - left);
                    W = BLOB_WIDTH(blob);
                    H = BLOB_HEIGHT(blob);

                    RC_set_3D_bbox_blob_level_data(blob);
                    if(first_solution) {
                        result = static_occlusion_search_alpha(blob);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result > 0.0) {
                            best_result = result;
                            first_solution = false;
                            starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                        } else
                            break;
                    } else {
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result < best_result)
                            break;
                        best_result = result;
                    }
                }
            } else { //BOTTOM
                double top = BLOB_YTOP(blob);

                for(current_dim = dim_limit ; current_dim > BLOB_YBOTTOM(blob) ; current_dim -= DIM2D_STEP) {
                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
	  
                    BLOB_YBOTTOM(blob) = (int) current_dim;
                    BLOB_HEIGHT(blob) = (int) (current_dim - top);
                    W = BLOB_WIDTH(blob);
                    H = BLOB_HEIGHT(blob);

                    RC_set_3D_bbox_blob_level_data(blob);
                    if(first_solution) {
                        result = static_occlusion_search_alpha(blob);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result > 0.0) {
                            best_result = result;
                            first_solution = false;
                            starting_alpha = S3D_ALPHA(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]);
                        } else
                            break;
                    } else {
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_alpha);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution())
                            break;
                        if(result < best_result)
                            break;
                        best_result = result;
                    }
                }
            }
            return;
        }
    }

    bool not_first_try = false, minactive[4] = {true, true, true, true}, maxactive[4] = {true, true, true, true};
    int i, current_min[4], current_max[4];
    double minbest[4], maxbest[4], minangle[4], maxangle[4]; 

    switch(dptype) {
        case MM_LEFT_OCCL:
            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            //Left branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= dim_limit)
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_XLEFT(blob) = current_min[i];
                                    BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Right branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= RECT_XLEFT(&realBBox))
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_XLEFT(blob) = current_max[i];
                                    BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(mean_limits[i] >= BLOB_XLEFT(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_XLEFT(blob) = (int) mean_limits[i];
                        BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            num_active--;
                            active[i] = false;
                        } else {
                            current_min[i] = current_max[i] = BLOB_XLEFT(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
            break;
        case MM_RIGHT_OCCL:
            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            //Left branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= RECT_XRIGHT(&realBBox))
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_XRIGHT(blob) = current_min[i];
                                    BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Right branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= dim_limit)
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_XRIGHT(blob) = current_max[i];
                                    BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(mean_limits[i] <= BLOB_XRIGHT(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_XRIGHT(blob) = (int) mean_limits[i];
                        BLOB_WIDTH(blob) = BLOB_XRIGHT(blob) - BLOB_XLEFT(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            active[i] = false;
                            num_active--;
                        } else {
                            current_min[i] = current_max[i] = BLOB_XRIGHT(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
            break;
        case MM_TOP_OCCL:
            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            //Top branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= dim_limit)
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YTOP(blob) = current_min[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Bottom branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= RECT_YTOP(&realBBox))
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YTOP(blob) = current_max[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(mean_limits[i] >= BLOB_YTOP(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_YTOP(blob) = (int) mean_limits[i];
                        BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            active[i] = false;
                            num_active--;
                        } else {
                            current_min[i] = current_max[i] = BLOB_YTOP(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
            break;
        case MM_BOTTOM_OCCL:
            while(num_active > 0) {
                //Initialization step already done
                if(not_first_try) {
                    for(i=0; i<4; i++) {
                        if(active[i]) {
                            //Top branch
                            if(minactive[i]) {
                                current_min[i] -= DIM2D_STEP;
                                if(current_min[i] <= RECT_YBOTTOM(&realBBox))
                                    minactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YBOTTOM(blob) = current_min[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, minangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < minbest[i])
                                        minactive[i] = false;
                                    else
                                        minbest[i] = result;
                                }
                            }
                            //Bottom branch
                            if(maxactive[i]) {
                                current_max[i] += DIM2D_STEP;
                                if(current_max[i] >= dim_limit)
                                    maxactive[i] = false;
                                else { //Still processing can be made
                                    models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                                    BLOB_YBOTTOM(blob) = current_max[i];
                                    BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                                    W = BLOB_WIDTH(blob);
                                    H = BLOB_HEIGHT(blob);
                                    RC_set_3D_bbox_blob_level_data(blob);
                                    result = static_occlusion_search_alpha_with_starting_point(blob, maxangle[i]);
                                    global_models_calculated_counter += models_calculated_counter;
                                    if(noWallsValidatedSolution() || result < maxbest[i])
                                        maxactive[i] = false;
                                    else
                                        maxbest[i] = result;
                                }
                            }
                            //Check for branches activity
                            if(!minactive[i] && !maxactive[i]) {
                                active[i] = false;
                                num_active--;
                            }
                        }
                    }
                } else { //Initialisation step for each analysis branch
                    not_first_try = true;
                    for(i=0; i<4; i++) {
                        if(mean_limits[i] <= BLOB_YBOTTOM(blob)) {
                            active[i] = false;
                            num_active--;
                            continue;
                        }
                        models_calculated_counter = valid_models_counter = models_accepted_after_walls_checking = 0;
                        BLOB_YBOTTOM(blob) = (int) mean_limits[i];
                        BLOB_HEIGHT(blob) = BLOB_YBOTTOM(blob) - BLOB_YTOP(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        result = static_occlusion_search_alpha_with_starting_point(blob, starting_angles[i]);
                        global_models_calculated_counter += models_calculated_counter;
                        if(noWallsValidatedSolution() || result == 0.0) {
                            active[i] = false;
                            num_active--;
                        } else {
                            current_min[i] = current_max[i] = BLOB_YBOTTOM(blob);
                            minbest[i] = maxbest[i] = result;
                            minangle[i] = maxangle[i] = starting_angles[i];
                        }
                    }
                }
            }
            break;
        default:
            AppendToLog("ReliabilityClassification::static_occlusion_search_2D: Not supported occlusion type in switch.");
    }    
}


//New implementation for static occlusion solutions search, taking into account different possible
//starting points based on the pre-defined models. The idea is to boost the computation time
//Search for parellelepiped solution for different sizes of a blob, according to the detected types of possible occlusions.
void ReliabilityClassification::fillOcclusionListWithStartingPoints(Blob *blob) {
    DetectionProblemType dptype = BLOB_DP_TYPE(blob);
    double dim_limit, starting_angles[4], mean_limits[4];

    memcpy(&realBBox, BLOB_BBOX(blob), sizeof(Rectangle<int>));

    //Treat vertical occlusion.
    if(dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP | MM_OBJECT_BOTTOM | MM_OBJECT_TOP)) {
        if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM)) {
            dim_limit = OcclusionMaxBottom;
            dim_limit = (dim_limit - BLOB_YBOTTOM(blob) < m_foreground->height()/2.0 ) ? dim_limit : BLOB_YBOTTOM(blob) + m_foreground->height()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_BOTTOM_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_BOTTOM_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }
        if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP)){ //TOP
            dim_limit = OcclusionMinTop;
            dim_limit = (BLOB_YTOP(blob) - dim_limit < m_foreground->height()/2.0 ) ? dim_limit : BLOB_YTOP(blob) - m_foreground->height()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_TOP_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_TOP_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }
    }

    //Treat horizontal occlusion.
    if(dptype & (MM_CAM_OCCL_LEFT   | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT   | MM_OBJECT_RIGHT)) {
        if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT)) {
            dim_limit = OcclusionMinLeft;
            dim_limit = (BLOB_XLEFT(blob) - dim_limit < m_foreground->width()/2.0 ) ? dim_limit : BLOB_XLEFT(blob) - m_foreground->width()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_LEFT_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_LEFT_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        }
        if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) { //RIGHT
            dim_limit = OcclusionMaxRight;
            dim_limit = ( dim_limit - BLOB_XRIGHT(blob) < m_foreground->width()/2.0 ) ? dim_limit : BLOB_XRIGHT(blob) + m_foreground->width()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_RIGHT_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_RIGHT_OCCL, dim_limit);
        }
    }

    //Restore real blob dimensions
    memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
    W = BLOB_WIDTH(blob);
    H = BLOB_HEIGHT(blob);
    RC_set_3D_bbox_blob_level_data(blob);
}


//New implementation for static occlusion solutions search, taking into account different possible
//starting points based on the pre-defined models. The idea is to boost the computation time
//Search for parellelepiped solution for different sizes of a blob, according to the detected types of possible occlusions.
void ReliabilityClassification::fillOcclusionListWithStartingPoints2(Blob *blob) {
    DetectionProblemType dptype = BLOB_DP_TYPE(blob);
    double dim_limit, starting_angles[4], mean_limits[4];

    memcpy(&realBBox, BLOB_BBOX(blob), sizeof(Rectangle<int>));
    
    //Treat corner occlusions first
    if( (dptype & (MM_OBJECT_BOTTOM | MM_CAM_OCCL_BOTTOM)) &&  (dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT | MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT) ) ) {     //BOTTOM-SIDE Occlusion
        //Treat bottom first but considering left and right extensions
        double mean_left_limits[4], mean_right_limits[4];
        Rectangle<int> bb2Ds[4];
        Parallelpiped bb3Ds[4];
        int i, found_index = -1;
        double minimal_change = DBL_MAX, current_change;
        double side_dim_limit;

        dim_limit = OcclusionMaxBottom;
        dim_limit = (dim_limit - BLOB_YBOTTOM(blob) < m_foreground->height()/2.0 ) ? dim_limit : BLOB_YBOTTOM(blob) + m_foreground->height()/2.0;
        if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT)) {
            side_dim_limit = OcclusionMinLeft;
            limits_by_mean_model_values_bottom_occlusion(mean_limits, mean_left_limits, mean_right_limits, starting_angles,
                                                         blob, true, bb2Ds, bb3Ds);

            densityData ddatas[4];                             // Data associated to fitness of 3D bounding box according to moving pixels.

            //Check if solutions already define a good occlusion configuration.
            for(i=0; i<4; i++) {
                if(   RECT_XLEFT(&bb2Ds[i]) <= BLOB_XLEFT(blob)
                   && RECT_YBOTTOM(&bb2Ds[i]) >= BLOB_YBOTTOM(blob)
                   && RECT_XLEFT(&bb2Ds[i]) > OcclusionMinLeft
                   && RECT_YBOTTOM(&bb2Ds[i]) < OcclusionMaxBottom
                   && covering_bbox(BLOB_POSITION(blob), blob, &ddatas[i], &bb3Ds[i])
                   && not_over_an_object(&bb3Ds[i], false)
                   && parallelpipedBaseOnAOIGround(&bb3Ds[i], m_context)
                ) {
                    if( (current_change = BLOB_XLEFT(blob) - RECT_XLEFT(&bb2Ds[i])) < minimal_change ) {
                        minimal_change = current_change;
                        found_index = i;
                    }
                }
            }
            //Optimal occlusion solution found
            if(found_index >= 0) {
                double
                    x1 = PARALL_X_i(&bb3Ds[found_index], 1),
                    y1 = PARALL_Y_i(&bb3Ds[found_index], 1),
                    x2 = PARALL_X_i(&bb3Ds[found_index], 2),
                    y2 = PARALL_Y_i(&bb3Ds[found_index], 2),
                    dx = x1 - x2, dy = y1 - y2,
                    l = sqrt(dx*dx + dy*dy),
                    w;
                if(fabs(l - model_lmean) < fabs(l - model_wmean))
                    w = model_wmean;
                else
                    w = model_lmean;

                memcpy(BLOB_BBOX(blob),   &bb2Ds[found_index], sizeof(Rectangle<int>));
                memcpy(BLOB_3DBBOX(blob), &bb3Ds[found_index], sizeof(Parallelpiped));
                memcpy(BLOB_DDATA(blob), &ddatas[found_index], sizeof(densityData));

                alpha = starting_angles[found_index];
                set_best_without_model_limits(blob, w, l, model_hmean, alpha);
                if(allowed_base_dimensions(w, l))
                    set_possible_solution(w, l, model_hmean, blob, false, false);

                //Restore real blob dimensions
                memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
                RC_set_3D_bbox_blob_level_data(blob);
                return;
            }
            static_occlusion_bottom_search_2D_in_corner(mean_limits, mean_left_limits, mean_right_limits, starting_angles, blob, MM_BOTTOM_OCCL, MM_LEFT_OCCL, dim_limit, side_dim_limit);

            //Eliminate left occlusion flags from global occlusion flag, as this occlusion is already treated
            //It is done if any solution has been found.
            if( (currentRigid ? m_ntopocc[BLOB_TYPE(blob)] : m_sub_ntop[BLOB_TYPE(blob)][BLOB_SUBTYPE(blob)]) > 0) {
                if(dptype & MM_CAM_OCCL_LEFT)
                    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_LEFT);
                if(dptype & MM_OBJECT_LEFT)
                    dptype = (DetectionProblemType)(dptype - MM_OBJECT_LEFT);
            }
        } else {
            side_dim_limit = OcclusionMaxRight;
            limits_by_mean_model_values_bottom_occlusion(mean_limits, mean_left_limits, mean_right_limits, starting_angles,
                                                         blob, false, bb2Ds, bb3Ds);

            densityData ddatas[4];                             // Data associated to fitness of 3D bounding box according to moving pixels.

            //Check if solutions already define a good occlusion configuration.
            for(i=0; i<4; i++)
                if(    RECT_XRIGHT(&bb2Ds[i]) >= BLOB_XRIGHT(blob)
                    && RECT_YBOTTOM(&bb2Ds[i]) >= BLOB_YBOTTOM(blob)
                    && RECT_XRIGHT(&bb2Ds[i]) < OcclusionMaxRight
                    && RECT_YBOTTOM(&bb2Ds[i]) < OcclusionMaxBottom
                    && covering_bbox(BLOB_POSITION(blob), blob, &ddatas[i], &bb3Ds[i])
                    && not_over_an_object(&bb3Ds[i], false)
                ) {
                    if( (current_change = RECT_XRIGHT(&bb2Ds[i]) - BLOB_XRIGHT(blob)) < minimal_change ) {
                        minimal_change = current_change;
                        found_index = i;
                    }
                }
	
            //Optimal occlusion solution found
            if(found_index >= 0) {
                double
                    x1 = PARALL_X_i(&bb3Ds[found_index], 1),
                    y1 = PARALL_Y_i(&bb3Ds[found_index], 1),
                    x2 = PARALL_X_i(&bb3Ds[found_index], 2),
                    y2 = PARALL_Y_i(&bb3Ds[found_index], 2),
                    dx = x1 - x2, dy = y1 - y2,
                    w, l = sqrt(dx*dx + dy*dy);
	    
                if(fabs(l - model_lmean) < fabs(l - model_wmean))
                    w = model_wmean;
                else
                    w = model_lmean;

                memcpy(BLOB_BBOX(blob),   &bb2Ds[found_index], sizeof(Rectangle<int>));
                memcpy(BLOB_3DBBOX(blob), &bb3Ds[found_index], sizeof(Parallelpiped));
                memcpy(BLOB_DDATA(blob), &ddatas[found_index], sizeof(densityData));

                alpha = starting_angles[found_index];
                set_best_without_model_limits(blob, w, l, model_hmean, alpha);
                if(allowed_base_dimensions(w, l))
                    set_possible_solution(w, l, model_hmean, blob, false, false);

                //Restore real blob dimensions
                memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
                W = BLOB_WIDTH(blob);
                H = BLOB_HEIGHT(blob);
                RC_set_3D_bbox_blob_level_data(blob);
                return;
            }

            static_occlusion_bottom_search_2D_in_corner(mean_limits, mean_left_limits, mean_right_limits, starting_angles, blob, MM_BOTTOM_OCCL, MM_RIGHT_OCCL, dim_limit, side_dim_limit);
	
            //Eliminate right occlusion flags from global occlusion flag, as this occlusion is already treated
            //It is done if any solution has been found.
            if( (currentRigid ? m_ntopocc[BLOB_TYPE(blob)] : m_sub_ntop[BLOB_TYPE(blob)][BLOB_SUBTYPE(blob)]) > 0) {
                if(dptype & MM_CAM_OCCL_RIGHT)
                    dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_RIGHT);
                if(dptype & MM_OBJECT_RIGHT)
                    dptype = (DetectionProblemType)(dptype - MM_OBJECT_RIGHT);
            }
        }

        //If a best solution have been found, use it as initial step for other occlusions
        if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
            memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
        else
            memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));

        //Eliminate bottom occlusion flags from global occlusion flag, as this occlusion is already treated
        if(dptype & MM_CAM_OCCL_BOTTOM)
            dptype = (DetectionProblemType)(dptype - MM_CAM_OCCL_BOTTOM);
        if(dptype & MM_OBJECT_BOTTOM)
            dptype = (DetectionProblemType)(dptype - MM_OBJECT_BOTTOM);

    } else if( (dptype & (MM_OBJECT_TOP | MM_CAM_OCCL_TOP)) &&  (dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT | MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT) ) ) {     //TOP-SIDE Occlusion
        //TO-DO: Implement (less frequent) TOP-LEFT and TOP-RIGHT occlusions.
    }

    //Treat vertical occlusion.
    if(dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP | MM_OBJECT_BOTTOM | MM_OBJECT_TOP)) {
        if(dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM)) {
            dim_limit = OcclusionMaxBottom;
            dim_limit = (dim_limit - BLOB_YBOTTOM(blob) < m_foreground->height()/2.0 ) ? dim_limit : BLOB_YBOTTOM(blob) + m_foreground->height()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_BOTTOM_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_BOTTOM_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
            else
                memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
        }
        if(dptype & (MM_CAM_OCCL_TOP | MM_OBJECT_TOP)) { //TOP
            dim_limit = OcclusionMinTop;
            dim_limit = (BLOB_YTOP(blob) - dim_limit < m_foreground->height()/2.0 ) ? dim_limit : BLOB_YTOP(blob) - m_foreground->height()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_TOP_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_TOP_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
            else
                memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
        }
    }

    //Treat horizontal occlusion.
    if(dptype & (MM_CAM_OCCL_LEFT   | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT   | MM_OBJECT_RIGHT)) {
        if(dptype & (MM_CAM_OCCL_LEFT | MM_OBJECT_LEFT)) {
            dim_limit = OcclusionMinLeft;
            dim_limit = (BLOB_XLEFT(blob) - dim_limit < m_foreground->width()/2.0 ) ? dim_limit : BLOB_XLEFT(blob) - m_foreground->width()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_LEFT_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_LEFT_OCCL, dim_limit);
            //If a best solution have been found, use it as initial step for other occlusions
            if (currentRigid ? there_is_best_valid[m_modelId] : sub_there_is_best_valid[m_modelId][m_subModelId])
                memcpy(BLOB_BBOX(blob), S3D_BBOX(currentRigid ? best_valid[m_modelId] : sub_best_valid[m_modelId][m_subModelId]), sizeof(Rectangle<int>));
            else
                memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
        }
        if(dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) { //RIGHT
            dim_limit = OcclusionMaxRight;
            dim_limit = ( dim_limit - BLOB_XRIGHT(blob) < m_foreground->width()/2.0 ) ? dim_limit : BLOB_XRIGHT(blob) + m_foreground->width()/2.0;
            limits_by_mean_model_values(mean_limits, starting_angles, blob, MM_RIGHT_OCCL);
            static_occlusion_search_2D(mean_limits, starting_angles, blob, MM_RIGHT_OCCL, dim_limit);
        }
    }

    //Restore real blob dimensions
    memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
    W = BLOB_WIDTH(blob);
    H = BLOB_HEIGHT(blob);
    RC_set_3D_bbox_blob_level_data(blob);
}


//Search for parallelpiped solutions, changing orientation alpha, for the occlusion case.
double ReliabilityClassification::static_occlusion_search_alpha(Blob *blob) {
    double current_result, best_result;

    Interval interval_h_0, interval_h_90;

    best_result = 0.0;
    
    for(alpha = beta; beta_direction*alpha < beta_direction*beta + M_PI/2.0 ; alpha += beta_direction*ALPHA_STEP) {

        //Inititialize the structures for normal case:
        Parallelpiped::initLimits(limits, nlimits, varlimrel);
	
        //Set initial values for boosting constants
        RC_set_3D_bbox_initial_alpha_level_data(alpha);
	
        //Test 3D bounding box generation
        models_calculated_counter++;
        if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0) // If non treatable degenerate case
            continue;
	
        if(Case != 0 || limits[3][3] == 0) {
            RC_set_3D_bbox_alpha_level_data();
            if(!calculate_hintervals( &interval_h_0, &interval_h_90))
                continue;

        } else if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
            continue;

        with_sol_counter = 0;
        if( (current_result = search_solution_by_height(blob, &interval_h_0, &interval_h_90, false)) > best_result )
            best_result = current_result;
    }

    return best_result;
}

//Search for parallelpiped solutions, changing orientation alpha, for the occlusion case.
double ReliabilityClassification::static_occlusion_search_alpha_with_starting_point(Blob *blob, double &starting_alpha) {
    double current_result;
    Interval interval_h_0, interval_h_90;

    alpha = MobileObject::NormalizeOrientation(starting_alpha);

    //Inititialize the structures for normal case:
    Parallelpiped::initLimits(limits, nlimits, varlimrel);
	
    //Set initial values for boosting constants
    RC_set_3D_bbox_initial_alpha_level_data(alpha);	
	
    //Test 3D bounding box generation for height 0.0
    models_calculated_counter++;
    if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0 ) // If non treatable degenerate case
        return 0.0;

    if(Case != 0 || limits[3][3] == 0) {
        RC_set_3D_bbox_alpha_level_data();
      
        if(!calculate_hintervals( &interval_h_0, &interval_h_90))
            return 0.0;

    } else if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
	return 0.0;

    with_sol_counter = 0;
    current_result  = search_solution_by_height(blob, &interval_h_0, &interval_h_90, false);

    if( current_result == 0.0 )
        return 0.0;

    double lalpha = alpha, ralpha = alpha, best_result = current_result;
    bool lactive = true, ractive = true, active = true;

    while(active) {
        //Left branch
        if(lactive) {
            lalpha -= ALPHA_STEP;
            alpha = lalpha;
            //Inititialize the structures for normal case:
            Parallelpiped::initLimits(limits, nlimits, varlimrel);
            //Set initial values for boosting constants
            RC_set_3D_bbox_initial_alpha_level_data(alpha);
            //Test 3D bounding box generation for height 0.0
            models_calculated_counter++;
            if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0 ) // If non treatable degenerate case
                lactive = false;

            if(lactive) {
                if(Case != 0 || limits[3][3] == 0) {
                    RC_set_3D_bbox_alpha_level_data();
                    if(!calculate_hintervals(&interval_h_0, &interval_h_90))
                        lactive = false;
                } else if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
                    lactive = false;
	  
                if(lactive) {
                    with_sol_counter = 0;
                    current_result  = search_solution_by_height(blob, &interval_h_0, &interval_h_90, false);
                }
	
                if( current_result > best_result ) {
                    best_result = current_result;
                    starting_alpha = alpha; //Current best alpha is updated
                    ractive = false; //Assuming that function is locally monotonic
                } else
                    lactive = false;
            }
        }
        //Right branch
        if(ractive) {
            ralpha += ALPHA_STEP;
            alpha = ralpha;
            //Inititialize the structures for normal case:
            Parallelpiped::initLimits(limits, nlimits, varlimrel);
            //Set initial values for boosting constants
            RC_set_3D_bbox_initial_alpha_level_data(alpha);
            //Test 3D bounding box generation for height 0.0
            models_calculated_counter++;
            if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0 ) // If non treatable degenerate case
                ractive = false;

            if(ractive) {
                if(Case != 0 || limits[3][3] == 0) {
                    RC_set_3D_bbox_alpha_level_data();
                    if(!calculate_hintervals(&interval_h_0, &interval_h_90))
                        ractive = false;
                } else if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
                    ractive = false;
	  
                if(ractive) {
                    with_sol_counter = 0;
                    current_result  = search_solution_by_height(blob, &interval_h_0, &interval_h_90, false);
                }
	  
                if( current_result > best_result ) {
                    best_result = current_result;
                    starting_alpha = alpha; //Current best alpha is updated
                    lactive = false; //Assuming that function is locally monotonic
                } else
                    ractive = false;
            }
        }
      
        if(!lactive && !ractive)
            active = false;
    }

    return best_result;
}

//Set the potencially colliding context object walls for certain blob 2D dimension, for checking when parallelpiped base is obtained.
void ReliabilityClassification::setAnnoyingWallsForContextObject(Blob *blob, world::ContextObject *object, bool normal) {
    
    std::vector< QSharedPointer<world::Wall> > *wallList = &object->wallList;
    std::vector< QSharedPointer<world::Wall2D> > *wall2DList = &object->wall2DList;

    std::vector< QSharedPointer<world::Wall> >::iterator wall_it, wall_it_end = wallList->end();
    std::vector< QSharedPointer<world::Wall2D> >::iterator wall2D_it = wall2DList->begin();

    world::WallSegment *base_segment;
    double y1, y2;
    Interval blobX, blobY, segX, segY, segYgivenX;
    if(normal) {
        Interval::newInterval(&blobX, BLOB_XLEFT(blob), BLOB_XRIGHT(blob));
        Interval::newInterval(&blobY, BLOB_YTOP(blob), BLOB_YBOTTOM(blob));
    } else {
        Interval::newInterval(&blobX, OcclusionMinLeft, OcclusionMaxRight);
        Interval::newInterval(&blobY, OcclusionMinTop, OcclusionMaxBottom);
    }

    for(wall_it = wallList->begin(); wall_it != wall_it_end; wall_it++, wall2D_it++) {
        if(!WALL_IS_SOLID(&**wall_it))
            continue;

        base_segment = WALL2D_SEGMENT(*wall2D_it, 3);
      
        Interval::newInterval(&segX, WSEGMENT_X1(base_segment), WSEGMENT_X2(base_segment));
        Interval::newInterval(&segY, WSEGMENT_Y1(base_segment), WSEGMENT_Y2(base_segment));
      
        if(!Interval::intersect(&segX, &segX, &blobX) || !Interval::intersect(&segY, &segY, &blobY))
            continue;

        y1 = INTERVAL_X1(&segX)*WSEGMENT_SLOPE(base_segment) + WSEGMENT_INTERCEPT(base_segment);
        y2 = INTERVAL_X1(&segX)*WSEGMENT_SLOPE(base_segment) + WSEGMENT_INTERCEPT(base_segment);
        Interval::newInterval(&segYgivenX, y1, y2);
      
        if(Interval::intersect(&segY, &segY, &segYgivenX))
            annoyingWallSegments.push_front(WALL2D_SEGMENT_BASE3D(&**wall2D_it));
    }
}

//Set the potencially colliding context walls for certain blob 2D dimension, for checking when parallelpiped base is obtained.
void ReliabilityClassification::setAnnoyingWallsForContextWalls(Blob *blob, bool normal) {
    
    std::vector< QSharedPointer<world::Wall2D> > *wall2DList = &m_context->wall2Ds;
    std::vector< QSharedPointer<world::Wall2D> >::iterator wall2D_it, wall2D_it_end = wall2DList->end();;

    world::WallSegment *base_segment;
    double y1, y2;
    Interval blobX, blobY, segX, segY, segYgivenX;

    if(normal) {
        Interval::newInterval(&blobX, BLOB_XLEFT(blob), BLOB_XRIGHT(blob));
        Interval::newInterval(&blobY, BLOB_YTOP(blob), BLOB_YBOTTOM(blob));
    } else {
        Interval::newInterval(&blobX, OcclusionMinLeft, OcclusionMaxRight);
        Interval::newInterval(&blobY, OcclusionMinTop, OcclusionMaxBottom);
    }
    
    for(wall2D_it = wall2DList->begin(); wall2D_it != wall2D_it_end; wall2D_it++) {

        base_segment = WALL2D_SEGMENT(&**wall2D_it, 3);
      
        Interval::newInterval(&segX, WSEGMENT_X1(base_segment), WSEGMENT_X2(base_segment));
        Interval::newInterval(&segY, WSEGMENT_Y1(base_segment), WSEGMENT_Y2(base_segment));
      
        if(!Interval::intersect(&segX, &segX, &blobX) || !Interval::intersect(&segY, &segY, &blobY))
            continue;

        y1 = INTERVAL_X1(&segX)*WSEGMENT_SLOPE(base_segment) + WSEGMENT_INTERCEPT(base_segment);
        y2 = INTERVAL_X1(&segX)*WSEGMENT_SLOPE(base_segment) + WSEGMENT_INTERCEPT(base_segment);
        Interval::newInterval(&segYgivenX, y1, y2);
      
        if(Interval::intersect(&segY, &segY, &segYgivenX))
            annoyingWallSegments.push_front(WALL2D_SEGMENT_BASE3D(&**wall2D_it));
    }
}

//Add the information of context objects near to the maximized version of the blob, according to calculated dimension growth limits,
//when occlusion situations occur.
void ReliabilityClassification::setMaxBlobNearObjects() {
    near_objects_extended = near_objects;
    std::deque<world::ContextObject *>::iterator objects;
    QSharedPointer < polygon2D<double> > pol;
    Interval blobX, blobY, polX, polY;
    double bb_l, bb_r, bb_b, bb_t;
    double x1, x2, y1, y2;

    Interval::newInterval(&blobX, OcclusionMinLeft, OcclusionMaxRight);
    Interval::newInterval(&blobY, OcclusionMinTop, OcclusionMaxBottom);
    x1 = OcclusionMinLeft;
    x2 = OcclusionMaxRight;
    y1 = OcclusionMinTop;
    y2 = OcclusionMaxBottom;
    
    for(objects = not_near_objects.begin(); objects != not_near_objects.end(); objects++) {

        pol = WOBJECT_OUTLINE2D_ON_IMAGE(*objects);

        bb_l = POLYGON_2D_BB_XLEFT(pol);
        bb_r = POLYGON_2D_BB_XRIGHT(pol);
        bb_b = POLYGON_2D_BB_YBOTTOM(pol);
        bb_t = POLYGON_2D_BB_YTOP(pol);

        Interval::newInterval(&polX, bb_l, bb_r);
        Interval::newInterval(&polY, bb_t, bb_b);

        if(thereIsIntersection(&polX, &polY, &blobX, &blobY)) {

            if( !INTERVAL_IS_NULL(&polX) && !INTERVAL_IS_NULL(&polY) && x1 >= bb_l && x2 <= bb_r && y1 >= bb_t && y2 <= bb_b ) {//Object inside object bb
                near_objects_extended.push_front(*objects);
                continue;
            } else {
                if(!INTERVAL_IS_NULL(&polX)) { //Possible Vertical Occlusion
                    if(y2 < bb_b && bb_t - y2 <= RC_PROXIMITY_PIXELS) {
                        near_objects_extended.push_front(*objects);
                        continue;
                    } else if(y1 > bb_t && y1 - bb_b <= RC_PROXIMITY_PIXELS) {
                        near_objects_extended.push_front(*objects);
                        continue;
                    }
                }

                if(!INTERVAL_IS_NULL(&polY)) { //Possible horizontal occlusion
                    if(x2 < bb_r && bb_l - x2 <= RC_PROXIMITY_PIXELS) {
                        near_objects_extended.push_front(*objects);
                        continue;
                    } else if(x1 > bb_l && x1 - bb_r <= RC_PROXIMITY_PIXELS) {
                        near_objects_extended.push_front(*objects);
                        continue;
                    }
                }
            }
        }
    }
}

//Returns true if we have found solutions but all of them have collided with a wall.
bool ReliabilityClassification::noWallsValidatedSolution() {
    if(valid_models_counter > 0 && models_accepted_after_walls_checking == 0)
        return true;
    
    return false;
}

Shape3DData *ReliabilityClassification::getMostLikelyDataForType(Blob *blob) {

    double Pmax, res;
    Interval interval_h_0, interval_h_90;
    int i, j;

    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
    std::map<ObjectSubtype, SpModelInterface> *parentModels;
    
    //Save real blob dimensions
    memcpy(&realBBox, BLOB_BBOX(blob), sizeof(Rectangle<int>));

    global_models_calculated_counter = 0;

    //Check best solutions for a specific model
    m_modelId = BLOB_TYPE(blob);
    m_currentModel = m_mapModels[m_modelId];
    currentRigid = m_currentModel->IsRigid;

    if(currentRigid)
        there_is_best_valid[m_modelId]=false;
    else
        j = 0;

    models_calculated_counter = 0;
    valid_models_counter = 0;
    models_accepted_after_walls_checking = 0;

    if(sizeOkForAnalysis[m_modelId]) {

        if(!currentRigid) {
            parentModels = &(m_currentModel->m_mapPostures);
            subModelsIt = parentModels->begin();
        }

        do {
            if(!currentRigid) {
                m_subModelId = (*subModelsIt).first;
                m_currentModel = (*subModelsIt).second;
                sub_there_is_best_valid[m_modelId][m_subModelId] = false;
                if(!subSizeOkForAnalysis[m_modelId][m_subModelId]) { //if blob size is not adequate for subModel, check the next one
                    subModelsIt++;
                    j++;
                    if(subModelsIt == parentModels->end())
                        break;
                    continue;
                }
                model_wmin = MobileObject::g_postureMinw[j];
                model_wmax = MobileObject::g_postureMaxw[j];
                model_wmean = MobileObject::g_postureMeanw[j];
                model_lmin = MobileObject::g_postureMinl[j];
                model_lmax = MobileObject::g_postureMaxl[j];
                model_lmean = MobileObject::g_postureMeanl[j];
                model_hmin = MobileObject::g_postureMinh[j];
                model_hmax = MobileObject::g_postureMaxh[j];
                model_hmean = MobileObject::g_postureMeanh[j];
                BLOB_SUBTYPE(blob) = m_subModelId;
            } else {
                i = MobileObject::objectModelMap[m_modelId];
                model_wmin = MobileObject::objectModelMinWidth[i];
                model_wmax = MobileObject::objectModelMaxWidth[i];
                model_wmean = MobileObject::objectModelMeanWidth[i];
                model_lmin = MobileObject::objectModelMinLength[i];
                model_lmax = MobileObject::objectModelMaxLength[i];
                model_lmean = MobileObject::objectModelMeanLength[i];
                model_hmin = MobileObject::objectModelMinHeight[i];
                model_hmax = MobileObject::objectModelMaxHeight[i];
                model_hmean = MobileObject::objectModelMeanHeight[i];
                m_subModelId = BLOB_SUBTYPE(blob) = ST_NO_SUBTYPES;
            }
	
            Interval::newInterval(&interval_modelh, model_hmin, model_hmax);
            Interval::newInterval(&interval_modelw, model_wmin, model_wmax);
            Interval::newInterval(&interval_modell, model_lmin, model_lmax);
      
            Pmax = 0.0;

            if(aoi_in) { //Do not classify instances which will remain out of the ZOI
                //Check for all possible orientations of the parallelpiped
                for(alpha = beta; beta_direction*alpha < beta_direction*(beta + M_PI/2.0) ; alpha += beta_direction*ALPHA_STEP){
	      
                    //Inititialize the structures for normal case:
                    Parallelpiped::initLimits(limits, nlimits, varlimrel);
                    RC_set_3D_bbox_initial_alpha_level_data(alpha);
	      
                    //Get information about the configuration of parallelpipeds to be generated
                    models_calculated_counter++;
                    if( (Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob)) < 0 ) // If non treatable degenerate case
                        continue;
	      
                    //The new solution is only useful for a strict configuration for the moment
                    if(Case != 0 || limits[3][3] == 0) {
                        RC_set_3D_bbox_alpha_level_data();
                        //Calculate the valid intervals for 3D height h. If all are null, we check the next orientation alpha.
                        if(!calculate_hintervals( &interval_h_0, &interval_h_90))
                            continue;
                        with_sol_counter = 0;
                        //Search the solution at different valid heights, for two different orientations (parallelpiped have the same shape inverting the dimensions w and l and
                        //changing the orientation angle alpha = alpha + pi/2 )
                        if( (res = search_solution_by_height(blob, &interval_h_0, &interval_h_90, true)) > Pmax )
                            Pmax = res;
                    } else { //Use the new method for normal cases (TO-DO!!!! Get special cases for new method)
                        if(!calculate_hintervals_simpler(&interval_h_0, &interval_h_90))
                            continue;
	      
                        with_sol_counter = 0;
                        if( (res = search_solution_by_height(blob, &interval_h_0, &interval_h_90, true)) > Pmax )
                            Pmax = res;
                    }
                }  // end alpha for
	    
                if(m_orderByDensity)
                    putBestByPixels(false);

                global_models_calculated_counter += models_calculated_counter;
            }
            //If occlusion is possible and there wasn't lots of problems of collisions with walls in the scene, we search for best occluding solutions
            if(possible_occlusion && Pmax < 0.98) {
                //Set the growth limits for a blob in the four directions in the image (LEFT, RIGHT, TOP, BOTTOM).
                adjustBlobLimitsByOcclusion(blob);
	  
                if(m_treatWallCoherence) {
                    //Set wall segments to be analyzed (on the floor) when checking coherence of the base of 3D parallelpiped, for not extended blob
                    near_objects_extended.clear();
                    setMaxBlobNearObjects();
                    std::deque<world::ContextObject *>::iterator objects_it;
	    
                    annoyingWallSegments.clear();
	    
                    //Store all possibly colliding walls for the maximal possible blob according to the obtained growth limits.
                    for(objects_it = near_objects_extended.begin(); objects_it != near_objects_extended.end(); objects_it++)
                        setAnnoyingWallsForContextObject(blob, *objects_it, false);
	    
                    setAnnoyingWallsForContextWalls(blob, false);
                }
	  
                //search for the occluded solutions
                fillOcclusionListWithStartingPoints2(blob);
                //fillOcclusionListWithStartingPoints(blob);
                //fillOcclusionList(blob);
	  
                //Order best occlusion solutions by density
                if(m_orderByDensity)
                    putBestByPixels(true);
            }
	
            if(!currentRigid) {
                subModelsIt++;
                j++;
            }
        } while(!currentRigid && subModelsIt != parentModels->end());

    } //OkForAnalysis
#ifdef RC_OUTPUT_2
      AppendToLog("\n\tNUMBER OF CALCULATED PARALLELPIPEDS:" + global_models_calculated_counter);
#endif	

    //Set blob's best type information
    return generateS3DListsForBlob(blob, m_modelId);
}

Shape3DData *ReliabilityClassification::addBestDataForType(bool normal, Blob *blob, ObjectType analyzed_type, std::map<ObjectType, Shape3DData> *S3D_list, Shape3DData *data) {
    ObjectType current_type;
    Shape3DData *current;
    std::map<ObjectType, Shape3DData>::iterator it, it_end = S3D_list->end();
    for(it = S3D_list->begin(); it != it_end; it++) {
        current_type = it->first;
        current = &(it->second);
        //Set information for no solution found
        if(current_type == analyzed_type) {
            data->copyShape3DDataOnlyInfo(current);
            if(normal)
                memcpy(S3D_BBOX(current), BLOB_BBOX(blob), sizeof(Rectangle<int>));
            if(!m_mapModels[current_type]->IsRigid) {
                std::map<ObjectSubtype, Shape3DData>::iterator sit, sit_end = S3D_SUBTYPES_LIST(current)->end();
                for(sit = S3D_SUBTYPES_LIST(current)->begin(); sit != sit_end; sit++) {
                    if(sit->first == S3D_SUBTYPE(data)) {
                        data->copyShape3DDataOnlyInfo(&it->second);
                        break;
                    }
                }
            }
            return current;
        }
    }

    return NULL;
}
 
Shape3DData *ReliabilityClassification::generateS3DListsForBlob(Blob *blob, ObjectType analyzed_type) {
    Shape3DData *best_s3d = NULL, *best_s3d_occ = NULL;
    double best = 0.0, best_occ = 0.0;
    bool exists3d = false, exists3docc = false;

    if(BLOB_NORMAL_3DDATA(blob) == NULL) {
        BLOB_NORMAL_3DDATA(blob) = getBestOnesList(true, &best_s3d, &best, &exists3d, blob);
        best_s3d = &((*BLOB_NORMAL_3DDATA(blob))[analyzed_type]);
    } else if (someNormalSolutionInserted) { //Some solution has been inserted
        if(currentRigid)
            best_s3d = m_top[analyzed_type][0];
        else { //get the best one from postures
            std::map<ObjectSubtype, std::deque<Shape3DData *> > *subModels;
            std::map<ObjectSubtype, std::deque<Shape3DData *> >::iterator subModelsIt;
            std::map<ObjectSubtype, int> *n_subModels;
            std::map<ObjectSubtype, int>::iterator n_subModelsIt;
            best = - 1.0;
            n_subModels = &(m_sub_ntop[analyzed_type]);
            subModels = &(m_sub_top[analyzed_type]);
            for(n_subModelsIt=n_subModels->begin(), subModelsIt=subModels->begin(); n_subModelsIt!=n_subModels->end(); n_subModelsIt++, subModelsIt++) {
                if((*n_subModelsIt).second > 0) {
                    if(S3D_P((*subModelsIt).second[0]) > best) {
                        best_s3d = (*subModelsIt).second[0];
                        best = S3D_P((*subModelsIt).second[0]);
                    }
                }
            }
        }
        best_s3d = addBestDataForType(true, blob, analyzed_type, BLOB_NORMAL_3DDATA(blob), best_s3d);
    }

    if(someOcclusionSolutionInserted) {
        if(BLOB_OCC_3DDATA(blob) == NULL) {
            BLOB_OCC_3DDATA(blob) = getBestOnesList(false, &best_s3d_occ, &best_occ, &exists3docc, blob);
            best_s3d_occ = &((*BLOB_OCC_3DDATA(blob))[analyzed_type]);
        } else {
            if(currentRigid)
                best_s3d_occ = m_topocc[analyzed_type][0];
            else { //get the best one from postures
                best_occ = - 1.0;
                std::map<ObjectSubtype, std::deque<Shape3DData *> > *subModels;
                std::map<ObjectSubtype, std::deque<Shape3DData *> >::iterator subModelsIt;
                std::map<ObjectSubtype, int> *n_subModels;
                std::map<ObjectSubtype, int>::iterator n_subModelsIt;
                n_subModels = &(m_sub_ntopocc[analyzed_type]);
                subModels = &(m_sub_topocc[analyzed_type]);
                for(n_subModelsIt=n_subModels->begin(), subModelsIt=subModels->begin(); n_subModelsIt!=n_subModels->end(); n_subModelsIt++, subModelsIt++) {
                    if((*n_subModelsIt).second > 0) {
                        if(S3D_P((*subModelsIt).second[0]) > best_occ) {
                            best_s3d_occ = (*subModelsIt).second[0];
                            best_occ = S3D_P((*subModelsIt).second[0]);
                        }
                    }
                }
            }
            best_s3d_occ = addBestDataForType(false, blob, analyzed_type, BLOB_OCC_3DDATA(blob), best_s3d_occ);
        }
    
        if(best_s3d_occ == NULL)
            return best_s3d;
        else if(best_s3d == NULL)
            return best_s3d_occ;
      
        return S3D_P(best_s3d) >= S3D_P(best_s3d_occ) ? best_s3d : best_s3d_occ;
    }

    //Return just for normal
    return best_s3d;
}

void ReliabilityClassification::getMostCoherentDataFromMobile(Shape3DData *bests3d, MobileObject *mobile, DetectionProblemType dptype, Blob *blob) {
    ObjectType type = BLOB_TYPE(blob);
    ObjectSubtype current_subtype;
    bool rigid = MobileObject::rigidModel[type];
    int i, j;
    int curW = BLOB_WIDTH(blob), curH = BLOB_HEIGHT(blob);
    bool is_vertical   = (dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP   | MM_OBJECT_BOTTOM | MM_OBJECT_TOP  )) ? true : false;
    bool is_horizontal = (dptype & (MM_CAM_OCCL_LEFT   | MM_CAM_OCCL_RIGHT | MM_OBJECT_LEFT   | MM_OBJECT_RIGHT)) ? true : false;
    int current_variation;
    bool first_found = false, ascending;

    m_modelId = type;
    currentRigid = rigid;

    //Initialize best found solution structures
    best_considering_alpha = new Shape3DData();
    best_considering_h = new Shape3DData();
    best_by_alpha_found = false; 
    bestAlphaDistance = DBL_MAX;

    //Initialize limiting data for search
    Winitial = (int)mobile->t2DDimData.W;
    Hinitial = (int)mobile->t2DDimData.H;
    alphaMobile = alphaInit = mobile->t3DDimData.alpha;
    if(mobile->t3DDimData.RCalpha < MobileObject::m_DimensionalCoherenceReliabilityThreshold ) {
        alphaVar = mobile->t3DDimData.SDalpha;
    } else {
        alphaChange = MobileObject::m_maximalAlphaRotationSpeed*MobileObject::secDiffSequence[0];
        alphaVar = (mobile->t3DDimData.SDalpha < alphaChange) ? 2*mobile->t3DDimData.SDalpha : 2*alphaChange;
    }
    hMobile = hInit = mobile->t3DDimData.h;
    wMobile = mobile->t3DDimData.w;
    lMobile = mobile->t3DDimData.l;
    dChange = MobileObject::m_Maximal3DDimensionChangeSpeed*MobileObject::secDiffSequence[0];
    
    hVar = (mobile->t3DDimData.SDh < dChange) ? mobile->t3DDimData.SDh : dChange;
    if(rigid) {
        hMin = hInit - hVar <= 0 ? 1 : hInit - hVar;
        hMax = hInit + hVar;
    } else {
        //If the standard deviation is higher to mean velocity absolute, direction of dimension velocity cannot be trusted
        if(mobile->t3DDimData.SDVh > fabs(mobile->t3DDimData.Vh)) { //Then do not consider it
            hMin = hInit - hVar <= 0 ? 1 : hInit - hVar;
            hMax = hInit + hVar;
        } else {
            double
                speedChange = mobile->t3DDimData.Vh*MobileObject::secDiffSequence[0],
                maxChange = MobileObject::m_Maximal3DDimensionChangeSpeed*MobileObject::secDiffSequence[0];
                speedChange = speedChange > maxChange ? (speedChange >= 0 ? maxChange : -maxChange) : speedChange;
                hInit += speedChange;
                hMin = hInit - hVar <= 0 ? 0 : hInit - hVar;
                hMax = hInit + hVar <= 0 ? 0 : hInit + hVar;
                if(hMin == 0 && hMax == 0)
                    hMin = hMax = hInit = mobile->t3DDimData.h;
        }
    }
    Interval::newInterval(&interval_hmobile, hMin, hMax);

    //If no occlusion, just analyze on the current frame
    Wmax = curW;
    Hmax = curH;   

    double Wvar = 0, Hvar = 0;
    bool second_found;

    if (dptype != MM_DP_NONE) {
        Rectangle<int> bb2D;
        Parallelpiped bb3D;
        double sina = sin(alphaMobile), cosa = cos(alphaMobile);
        bb3D.getFromInitial3DPoint(m_context, &bb2D,
                                   mobile->t3DSpatialData.x + beta_direction*(mobile->t3DDimData.w*sina - mobile->t3DDimData.l*cosa)/2.0,
                                   mobile->t3DSpatialData.y - (mobile->t3DDimData.w*cosa + mobile->t3DDimData.l*sina)/2.0,
                                   1, alphaMobile, beta_direction, mobile->t3DDimData.w, mobile->t3DDimData.l, mobile->t3DDimData.h);

        //New version starting exploration from last computed 2D size for mobile
        if (dptype & MM_HORIZONTAL_OCCL_MASK) {
            Winitial = RECT_WIDTH(&bb2D);
            Wvar = abs(Winitial - (int)mobile->t2DDimData.W) + 2*(int)mobile->t2DDimData.SDW;
        }
        if(dptype & MM_VERTICAL_OCCL_MASK) {
            Hinitial = RECT_HEIGHT(&bb2D);
            Hvar = abs(Hinitial - (int)mobile->t2DDimData.H) + 2*(int)mobile->t2DDimData.SDH;
        }
    } else {
        Winitial = curW;
        Hinitial = curH;
    }

    if(rigid) {
        i = MobileObject::objectModelMap[type];
        model_wmin = MobileObject::objectModelMinWidth[i];
        model_wmax = MobileObject::objectModelMaxWidth[i];
        model_wmean = MobileObject::objectModelMeanWidth[i];
        model_lmin = MobileObject::objectModelMinLength[i];
        model_lmax = MobileObject::objectModelMaxLength[i];
        model_lmean = MobileObject::objectModelMeanLength[i];
        model_hmin = MobileObject::objectModelMinHeight[i];
        model_hmax = MobileObject::objectModelMaxHeight[i];
        model_hmean = MobileObject::objectModelMeanHeight[i];

        m_currentModel = m_mapModels[type];
        m_subModelId = BLOB_SUBTYPE(blob) = ST_NO_SUBTYPES;
    }

    //Store a copy of the original
    memcpy(&realBBox, BLOB_BBOX(blob), sizeof(Rectangle<int>));

    if(sizeOkForAnalysis[type]) {
        S3D_TYPE(bests3d) = type;
        BLOB_TYPE(blob) = type;
	
        if(!rigid)
            j = 0;
        do {
            if(!rigid) {
                current_subtype = MobileObject::g_posturesList[j];
                m_subModelId = current_subtype;
                m_currentModel = MobileObject::objectSubModelsList[MobileObject::objectModelMap[type]][MobileObject::objectSubModelMap[type][current_subtype]];

                if(!subSizeOkForAnalysis[type][current_subtype]) { //if blob size is not adequate for subModel, check the next one
                    j++;
                    if(j == MobileObject::m_numberOfPostures)
                        break;
                    continue;
                }
                BLOB_SUBTYPE(blob) = S3D_SUBTYPE(bests3d) = current_subtype;

                model_wmin = MobileObject::g_postureMinw[j];
                model_wmax = MobileObject::g_postureMaxw[j];
                model_lmin = MobileObject::g_postureMinl[j];
                model_lmax = MobileObject::g_postureMaxl[j];
                model_hmin = MobileObject::g_postureMinh[j];
                model_hmax = MobileObject::g_postureMaxh[j];
            } else
                BLOB_SUBTYPE(blob) = S3D_SUBTYPE(bests3d) = ST_NO_SUBTYPES;
	
            Interval::newInterval(&interval_modelh, model_hmin, model_hmax);
            Interval::newInterval(&interval_modelw, model_wmin, model_wmax);
            Interval::newInterval(&interval_modell, model_lmin, model_lmax);
	
            //Check different sizes of bounding box
            bool no_occlusion = is_vertical || is_horizontal ? false : true;
            bool best_by_alpha_found_before = false;
            int initialVariation;

            //initial classification
            W = BLOB_WIDTH(blob);
            H = BLOB_HEIGHT(blob);
            RC_set_3D_bbox_blob_level_data(blob);

            reliability_tracking_search_alpha(blob);

            //no occlusion flag allows to enter to one processing loop
            if(is_vertical || no_occlusion) {
                int
                    top = BLOB_YTOP(blob),
                    bottom = BLOB_YBOTTOM(blob);
                bool is_bottom = (dptype & (MM_CAM_OCCL_BOTTOM | MM_OBJECT_BOTTOM)) ? true : false;
                bool is_top    = (dptype & (MM_CAM_OCCL_TOP    | MM_OBJECT_TOP   )) ? true : false;

                if(is_bottom) {
                    first_found = false;
                    second_found = false;
                    if(Hinitial < RECT_YBOTTOM(&realBBox) - top + DIM2D_STEP)
                        initialVariation = RECT_YBOTTOM(&realBBox) - top + DIM2D_STEP;
                    else
                        initialVariation = Hinitial;
                    //Compute initial size and explore bottom
                    for(current_variation = initialVariation; current_variation <= Hvar; current_variation += DIM2D_STEP) {
                        BLOB_YBOTTOM(blob) = top + current_variation;
                        BLOB_HEIGHT(blob)  = BLOB_YBOTTOM(blob) - top;
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        ascending = reliability_tracking_search_alpha(blob);
                        if(first_found) {
                            if(!ascending)
                                break;
                            second_found = true;
                        } else if(ascending)
                            first_found = true;
                    }
                    //Explore bottom
                    if(!second_found) //If quality of solution ascends in one sense, it will descend in the other
                        for(current_variation = DIM2D_STEP; current_variation <= Hvar ; current_variation += DIM2D_STEP) {
                            BLOB_YBOTTOM(blob) = top + Hinitial - current_variation;
                            if(RECT_YBOTTOM(&realBBox) >= BLOB_YBOTTOM(blob))
                                break;
                            BLOB_HEIGHT(blob)  = BLOB_YBOTTOM(blob) - top;
                            if(BLOB_HEIGHT(blob) <= 0)
                                break;
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            ascending = reliability_tracking_search_alpha(blob);
                            if(first_found) {
                                if(!ascending)
                                    break;
                            } else if(ascending)
                                first_found = true;
                        }

                    //If a best solution have been found, use it as initial step for other occlusions
                    if (best_by_alpha_found)
                        memcpy(BLOB_BBOX(blob), S3D_BBOX(best_considering_alpha), sizeof(Rectangle<int>));
                }

                best_by_alpha_found_before = best_by_alpha_found;

                if(is_top) { //TOP

                    first_found = false;
                    second_found = false;

                    if(Hinitial < bottom - RECT_YTOP(&realBBox) + DIM2D_STEP)
                        initialVariation = bottom - RECT_YTOP(&realBBox) + DIM2D_STEP;
                    else
                        initialVariation = Hinitial;
                    //Compute initial size and explore top
                    for(current_variation = initialVariation ; current_variation <= Hvar; current_variation += DIM2D_STEP) {
                        BLOB_YTOP(blob) = bottom - current_variation;
                        BLOB_HEIGHT(blob)  = bottom - BLOB_YTOP(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        ascending = reliability_tracking_search_alpha(blob);
                        if(best_by_alpha_found_before) {
                            if(!ascending)
                                break;
                        } else {
                            if(first_found) {
                                if(!ascending)
                                    break;
                                second_found = true;
                            } else if(ascending)
                                first_found = true;
                        }
                    }
                    //Explore bottom
                    if(!second_found) //If quality of solution ascends in one sense, it will descend in the other
                        for(current_variation = DIM2D_STEP; current_variation <= Hvar; current_variation += DIM2D_STEP) {
                            BLOB_YTOP(blob) = bottom - Hinitial + current_variation;
                            if(BLOB_YTOP(blob) > RECT_YTOP(&realBBox))
                                break;
                            BLOB_HEIGHT(blob)  = bottom - BLOB_YTOP(blob);
                            if(BLOB_HEIGHT(blob) <= 0)
                                break;
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            ascending = reliability_tracking_search_alpha(blob);

                            if(best_by_alpha_found_before) {
                                if(!ascending)
                                break;
                            } else {
                                if(first_found) {
                                    if(!ascending)
                                        break;
                                } else if(ascending)
                                    first_found = true;
                            }
                        }

                    //If a best solution have been found, use it as initial step for other occlusions
                    if (best_by_alpha_found)
                        memcpy(BLOB_BBOX(blob), S3D_BBOX(best_considering_alpha), sizeof(Rectangle<int>));
                }
            }

            best_by_alpha_found_before = best_by_alpha_found;

            if(is_horizontal) {
                int
                    left = BLOB_XLEFT(blob),
                    right = BLOB_XRIGHT(blob);
                bool is_left  = (dptype & (MM_CAM_OCCL_LEFT  | MM_OBJECT_LEFT )) ? true : false;
                bool is_right = (dptype & (MM_CAM_OCCL_RIGHT | MM_OBJECT_RIGHT)) ? true : false;

                if(is_left) {
                    if(Winitial < right - RECT_XLEFT(&realBBox) + DIM2D_STEP)
                        initialVariation = right - RECT_XLEFT(&realBBox) + DIM2D_STEP;
                    else
                        initialVariation = Winitial;

                    first_found = false;
                    second_found = false;

                    //Compute initial size and explore left
                    for(current_variation = initialVariation ; current_variation <= Wvar; current_variation += DIM2D_STEP) {
                        BLOB_XLEFT(blob) = right - current_variation;
                        BLOB_WIDTH(blob)  = right - BLOB_XLEFT(blob);
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        ascending = reliability_tracking_search_alpha(blob);
                        if(best_by_alpha_found_before) {
                            if(!ascending)
                                break;
                        } else {
                            if(first_found) {
                                if(!ascending)
                                    break;
                                second_found = true;
                            } else if(ascending)
                                first_found = true;
                        }
                    }
                    //Explore right
                    if(!second_found) //If quality of solution ascends in one sense, it will descend in the other
                        for(current_variation = DIM2D_STEP; current_variation <= Wvar; current_variation += DIM2D_STEP) {
                            BLOB_XLEFT(blob) = right - Winitial + current_variation;
                            if(BLOB_XLEFT(blob) >= RECT_XLEFT(&realBBox))
                                break;
                            BLOB_WIDTH(blob)  = right - BLOB_XLEFT(blob);
                            if(BLOB_WIDTH(blob) <= 0)
                                break;
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            ascending = reliability_tracking_search_alpha(blob);
                            if(best_by_alpha_found_before) {
                                if(!ascending)
                                    break;
                            } else {
                                if(first_found) {
                                    if(!ascending)
                                        break;
                                } else if(ascending)
                                    first_found = true;
                            }
                        }

                    //If a best solution have been found, use it as initial step for other occlusions
                    if (best_by_alpha_found)
                        memcpy(BLOB_BBOX(blob), S3D_BBOX(best_considering_alpha), sizeof(Rectangle<int>));
                }

                best_by_alpha_found_before = best_by_alpha_found;

                if(is_right) { //RIGHT

                    if(Winitial < RECT_XRIGHT(&realBBox) - left + DIM2D_STEP)
                        initialVariation = RECT_XRIGHT(&realBBox) - left + DIM2D_STEP;
                    else
                        initialVariation = Winitial;

                    first_found = false;
                    second_found = false;

                    //Compute initial size and explore right
                    for(current_variation = initialVariation; current_variation <= Wvar; current_variation += DIM2D_STEP) {
                        BLOB_XRIGHT(blob) = left + current_variation;
                        BLOB_WIDTH(blob)  = BLOB_XRIGHT(blob) - left;
                        W = BLOB_WIDTH(blob);
                        H = BLOB_HEIGHT(blob);
                        RC_set_3D_bbox_blob_level_data(blob);
                        ascending = reliability_tracking_search_alpha(blob);
                        if(best_by_alpha_found_before) {
                            if(!ascending)
                                break;
                        } else {
                            if(first_found) {
                                if(!ascending)
                                    break;
                                second_found = true;
                            } else if(ascending)
                                first_found = true;
                        }
                    }
                    //Explore left
                    if(!second_found) //If quality of solution ascends in one sense, it will descend in the other
                        for(current_variation =  DIM2D_STEP ; current_variation <= Wvar; current_variation += DIM2D_STEP) {
                            BLOB_XRIGHT(blob) = left + Winitial - current_variation;
                            if(BLOB_XRIGHT(blob) < RECT_XRIGHT(&realBBox))
                                break;
                            BLOB_WIDTH(blob)  = BLOB_XRIGHT(blob) - left;
                            if(BLOB_WIDTH(blob) <= 0)
                                break;
                            W = BLOB_WIDTH(blob);
                            H = BLOB_HEIGHT(blob);
                            RC_set_3D_bbox_blob_level_data(blob);
                            ascending = reliability_tracking_search_alpha(blob);
                            if(best_by_alpha_found_before) {
                                if(!ascending)
                                    break;
                            } else {
                                if(first_found) {
                                    if(!ascending)
                                        break;
                                } else if(ascending)
                                    first_found = true;
                            }
                        }
                }
            }

            //Restore real blob dimensions
            memcpy(BLOB_BBOX(blob), &realBBox, sizeof(Rectangle<int>));
            W = BLOB_WIDTH(blob);
            H = BLOB_HEIGHT(blob);
            RC_set_3D_bbox_blob_level_data(blob);

            j++;

        } while(!rigid && j < MobileObject::m_numberOfPostures);

    } //OkForAnalysis

    //Store the best result in terms of alpha distance to mobile  
    if(best_by_alpha_found) 
        best_considering_alpha->copyNoLists(bests3d);

    delete best_considering_alpha;
    delete best_considering_h;

  }


bool ReliabilityClassification::set_best_valid_alpha_for_tracking() {

    //double
    //    dw = fabs(S3D_W(best_considering_h) - wMobile),
    //    dl = fabs(S3D_L(best_considering_h) - lMobile),
    //    dh = fabs(S3D_H(best_considering_h) - hMobile);

    //    if(dw < dChange && dl < dChange && dh < dChange) {
    double distance = fabs(MobileObject::NormalizeOrientation(S3D_ALPHA(best_considering_h)) - MobileObject::NormalizeOrientation(alphaMobile));
    if(best_by_alpha_found == false) {
        best_by_alpha_found = true;
        bestDimDistance = currentDimDistance;
        bestAlphaDistance = distance;
        best_considering_h->copyNoLists(best_considering_alpha);
        return true;
    } else {
        if(distance < bestAlphaDistance) {
            best_considering_h->copyNoLists(best_considering_alpha);
            bestDimDistance = currentDimDistance;
            bestAlphaDistance = distance;
            return true;
        } else if (distance == bestAlphaDistance) {
            if(currentDimDistance < bestDimDistance) {
                best_considering_h->copyNoLists(best_considering_alpha);
                bestDimDistance = currentDimDistance;
                bestAlphaDistance = distance;
                return true;
            }
        }
    }

    return false;

}

double ReliabilityClassification::NormalizeAngle(double alpha) {
    return alpha < 0 ? fmod(alpha, 2.0*M_PI) + 2.0*M_PI : fmod(alpha, 2.0*M_PI);
}


//Lead an angle to the interval [beta, beta+pi/2], to ensure good processing
double ReliabilityClassification::convertAlpha(double original_alpha) {
    double 
        beta90 = NormalizeAngle(beta + M_PI/2.0),
        beta180 = NormalizeAngle(beta + M_PI),
        normalisedAlpha = NormalizeAngle(original_alpha);

    normalised90 = normalised180 = false;

    //Test for normalisation in PI
    if(beta180 < beta) { //The angle made a loop
        if(normalisedAlpha < beta && normalisedAlpha > beta180) { //Invert test
            normalisedAlpha = NormalizeAngle(normalisedAlpha + M_PI);
            normalised180 = true;
        }
    } else {
        if(normalisedAlpha < beta || normalisedAlpha > beta180) {
            normalisedAlpha = NormalizeAngle(normalisedAlpha + M_PI);
            normalised180 = true;
        }
    }

    //Test for normalisation in PI/2.0
    if(beta90 < beta) { //The angle made a loop
        if(normalisedAlpha < beta && normalisedAlpha > beta90) { //Invert test
            normalisedAlpha = NormalizeAngle(normalisedAlpha - M_PI/2.0);
            normalised90 = true;
        }
    } else {
        if(normalisedAlpha < beta || normalisedAlpha > beta90) {
            normalisedAlpha = NormalizeAngle(normalisedAlpha - M_PI/2.0);
            normalised90 = true;
        }
    }

    return normalisedAlpha;
}


bool ReliabilityClassification::reliability_tracking_search_alpha(Blob *blob) {
    
    double current_result;
    Interval interval_h_0;
    double alpha_distance;
    double original_alpha = alphaInit; 

    alpha = convertAlpha(original_alpha);
    //alpha = original_alpha;
    //Initial solution for alpha equal to initial
    //1. Inititialize the structures for normal case:
    Parallelpiped::initLimits(limits, nlimits, varlimrel);
    //2. Set initial values for boosting constants
    RC_set_3D_bbox_initial_alpha_level_data(alpha);	
    //3. Test 3D bounding box generation and determine Case
    Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob);
    if(Case >= 0) { //If treatable Case but not the most normal
        //4. Set alpha level information
        RC_set_3D_bbox_alpha_level_data();

        //5. Calculate best solutions in the intervals for h
        if( calculate_hintervals_for_tracking(&interval_h_0) ) {
            best_by_h_found = false;
            current_result = search_solution_by_height_for_tracking(blob, &interval_h_0, DBL_MAX);
            //If solution is found in h and validated in alpha, nothing more to be done
            if(best_by_h_found && set_best_valid_alpha_for_tracking())
                return true;
            else if(best_by_alpha_found)
                return false;
        }
    } 

    bool up_active = true, down_active = true; 
  
    //Now check next solutions and stop
    for(alpha_distance = ALPHA_STEP; alpha_distance <= alphaVar ; alpha_distance += ALPHA_STEP) {
        //Check alpha adding distance
        if(up_active) {
            alpha = convertAlpha(alphaInit + alpha_distance);
            //alpha = alphaInit + alpha_distance;
            Parallelpiped::initLimits(limits, nlimits, varlimrel);
            RC_set_3D_bbox_initial_alpha_level_data(alpha);	
	
            Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob);
	
            if(Case >= 0) { //If treatable Case
                RC_set_3D_bbox_alpha_level_data();
                if( calculate_hintervals_for_tracking(&interval_h_0) ) {
                    best_by_h_found = false;
                    current_result = search_solution_by_height_for_tracking(blob, &interval_h_0, DBL_MAX);
                    //If solution is found in h and validated in alpha, nothing more to be done
                    if(best_by_h_found && set_best_valid_alpha_for_tracking())
                        return true;
                    else if(best_by_alpha_found)
                        up_active = false;
                }// else
                 // up_active = false;
            }
        }

        if(down_active) {
            //Check alpha substracting distance
            alpha = convertAlpha(alphaInit - alpha_distance);
            //alpha = alphaInit - alpha_distance;
            Parallelpiped::initLimits(limits, nlimits, varlimrel);
            RC_set_3D_bbox_initial_alpha_level_data(alpha);

            Case = RC_set_3Dbbox_case_and_variable_indexes_simpler(blob);

            if(Case >= 0) { //If treatable Case
                RC_set_3D_bbox_alpha_level_data();
                if( calculate_hintervals_for_tracking(&interval_h_0) ) {
                    best_by_h_found = false;
                    current_result = search_solution_by_height_for_tracking(blob, &interval_h_0, DBL_MAX);
                    //If solution is found in h and validated in alpha, nothing more to be done
                    if(best_by_h_found && set_best_valid_alpha_for_tracking())
                        return true;
                    else
                        down_active = false;
                } //else
                  //down_active = false;
            }
        }

        if(!up_active && !down_active)
            return false;
    }
    
    return false;
}

double ReliabilityClassification::search_solution_by_height_for_tracking(Blob *blob, interval_t interval_h, double previous_best) {

    double h, hl, hr, l, w;
    double best_Rbranch, best_Lbranch;
    double P_Rbranch, P_Lbranch;
    bool follow_left = true, follow_right = true;
    bool active_R=true, active_L=true;
    bool ascended_R=false, ascended_L=false;
    double
        hmin = INTERVAL_X1(interval_h),
        hmax = INTERVAL_X2(interval_h);
      
    if(hInit < hmin) {
        h = hmin;
        follow_left = false;
    } else if(hInit > hmax) {
        h = hmax;
        follow_left = false;
    } else
        h = hInit;


    if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, h) : RC_compute_blob_3Dbbox_simpler(blob, h)) {
        valid_models_counter++;

        //Set w and l given h
        if(Case != 0 || limits[3][3] == 0) {
            RC_compute_w_from_h(h, &w);
            RC_compute_l_from_h(h, &l);
        } else { //Simpler
            w = MM1*h + MM2;
            l = MM3*h + MM4;
        }

        if(allowed_base_dimensions(w, l))
            best_Lbranch = best_Rbranch = set_possible_solution_for_tracking(w, l, h, blob, previous_best);
        else
            best_Lbranch = best_Rbranch = previous_best;
    } else //No initial valid model found
      best_Rbranch = best_Lbranch = previous_best;

    for(hr=h + H_STEP_CM, hl=h - H_STEP_CM; hr<=hmax || hl>=hmin; hr+=H_STEP_CM, hl-=H_STEP_CM) {

        if(hl < hmin)
            active_L = false;
        if(hr > hmax)
            active_R = false;

        if(!active_L && !active_R)
            break;
      
        //Break conditions
        if(!active_R)
            follow_right = false;
        if(!active_L)
            follow_left = false;
      
        if(follow_right && hr > hmax)
            follow_right = false;
        if(follow_left && hl < hmin)
            follow_left = false;
      
        if(follow_right) { //Process right branch

            if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, hr) : RC_compute_blob_3Dbbox_simpler(blob, hr)) {
                valid_models_counter++;
	  
                //Set w and l given h
                if(Case != 0 || limits[3][3] == 0) {
                    RC_compute_w_from_h(hr, &w);
                    RC_compute_l_from_h(hr, &l);
                } else { //Simpler
                    w = MM1*hr + MM2;
                    l = MM3*hr + MM4;
                }

                if(l < 0 || w < 0)
                    P_Rbranch = DBL_MAX;
                else {
                    if(active_R) {
                        P_Rbranch = (allowed_base_dimensions(w, l)) ? set_possible_solution_for_tracking(w, l, hr, blob, best_Rbranch < best_Lbranch ? best_Rbranch : best_Lbranch) : DBL_MAX;

                        if(P_Rbranch < best_Rbranch) { //Check cross or self-validity of sub-branch of rotated angle in 0
                            ascended_R = true;
                            active_L = false;
                            best_Rbranch = P_Rbranch;
                        } else if ( l < normalised90 ? model_wmin : model_lmin || w < normalised90 ? model_lmin : model_wmin || P_Rbranch > best_Rbranch )
                            active_R = false;
                    }
                }
            }
        }
    
        if(follow_left) { //Process left branch

            if((Case != 0 || limits[3][3] == 0) ? RC_compute_blob_3Dbbox(blob, hl) : RC_compute_blob_3Dbbox_simpler(blob, hl)) {
                valid_models_counter++;
	  
                //Set w and l given h
                if(Case != 0 || limits[3][3] == 0) {
                    RC_compute_w_from_h(hl, &w);
                    RC_compute_l_from_h(hl, &l);
                } else { //Simpler
                    w = MM1*hl + MM2;
                    l = MM3*hl + MM4;
                }

                if(l < 0 || w < 0)
                    P_Lbranch = DBL_MAX;
                else {
                    if(active_L) { //Non-rotated solution
                        P_Lbranch = (allowed_base_dimensions(w, l)) ? set_possible_solution_for_tracking(w, l, hl, blob, best_Rbranch < best_Lbranch ? best_Rbranch : best_Lbranch) : DBL_MAX;

                        if(P_Lbranch < best_Lbranch) { //Check cross or self-validity of sub-branch of rotated angle in 0
                            ascended_L = true;
                            active_R = false;
                            best_Lbranch = P_Lbranch;
                        } else if ( l > normalised90 ? model_wmax : model_lmax || w > normalised90 ? model_lmax : model_wmax || P_Lbranch > best_Lbranch )
                            active_L = false;
                    }
                }
            }
        }
    }
    
    return (best_Rbranch < best_Lbranch) ? best_Rbranch : best_Lbranch;

}


//Classify for specific type
Shape3DData *ReliabilityClassification::getMostLikelyDataFromMobile(Blob *blob, ObjectType type) {

    Blob *blobCopy = blob->copyWithLists();
    
    BLOB_TYPE(blob) = BLOB_TYPE(blobCopy) = type;
    //REVISAR aoi_in
    aoi_in = true;
    //Check if blob is too little or too big according to pre-defined object models
    if(    ( (aoi_in && !BLOB_IS_REDUCED_SIZE(blob)) || possible_occlusion) 
	&& !BLOB_IS_EXCESIVE_SIZE(blob)
	&& sizeOkForAnalysis[type] ) {

        someOcclusionSolutionInserted = false;
        someNormalSolutionInserted = false;

        Shape3DData *best_s3d = getMostLikelyDataForType(blobCopy);
//        Shape3DData *best_s3d = NULL;
        Shape3DData *return_s3d = NULL;
        if(best_s3d != NULL)
            return_s3d = best_s3d->copy();

        delete blobCopy;
        return return_s3d;
    }
      
    return NULL;
}


Shape3DData *ReliabilityClassification::getMostCoherentDataFromMobileAndBBoxLimit(MobileObject *mobile,
                                                                                  std::map<ObjectType, Shape3DData> *&normal,
                                                                                  std::map<ObjectType, Shape3DData> *&occ,
                                                                                  Blob *blob) {
    Shape3DData *bests3d = NULL;
    Shape3DData auxs3d, *curs3d;
    DetectionProblemType dptype = BLOB_DP_TYPE(blob);
    ObjectType type = BLOB_TYPE(blob);
    Blob *blobCopy = blob->copyWithLists();
    
    memset(&auxs3d, 0, sizeof(Shape3DData));

    //Check if blob is too little or too big according to pre-defined object models
    if( ( (aoi_in && !BLOB_IS_REDUCED_SIZE(blob)) || possible_occlusion) && !BLOB_IS_EXCESIVE_SIZE(blob)) {
        getMostCoherentDataFromMobile(&auxs3d, mobile, dptype, blobCopy);
      
        delete blobCopy;

        //Set the best solution if there is one, in the right s3d
        if(S3D_P(&auxs3d) > 0.0) {
            //Check if it is an occlusion solution
            bool occlusion = BLOB_WIDTH(blob) == S3D_WIDTH(&auxs3d) && BLOB_HEIGHT(blob) == S3D_HEIGHT(&auxs3d) ? false : true;
            S3D_DP_TYPE(&auxs3d) = occlusion ? dptype : MM_DP_NONE;
            std::map<ObjectSubtype, Shape3DData> *subtypes;
            ObjectType aux_type;
            S3D_3DBBOX(&auxs3d)->setReal3DPosition(S3D_3D_POSITION(&auxs3d));

            if(occlusion) {
                occ = BLOB_OCC_3DDATA(blob) = Shape3DData::copyList(normal);
                std::map<ObjectType, Shape3DData>::iterator normal_it,
                                                            normal_it_end = normal->end(),
                                                            occ_it = occ->begin();
                for(normal_it = normal->begin(); normal_it != normal_it_end; normal_it++, occ_it++) {
                    aux_type = normal_it->first;
                    curs3d = &normal_it->second;

                    subtypes = S3D_SUBTYPES_LIST(curs3d);
                    curs3d->setNull(aux_type, ST_UNKNOWN, MM_DP_NONE, BLOB_BBOX(blob));
                    S3D_SUBTYPES_LIST(curs3d) = subtypes;

                    curs3d = &occ_it->second;
                    if(type == aux_type) {
                        subtypes = S3D_SUBTYPES_LIST(curs3d);
                        memcpy(curs3d, &auxs3d, sizeof(Shape3DData));
                        S3D_SUBTYPES_LIST(curs3d) = subtypes;
                        bests3d = curs3d;
                    } else {
                        subtypes = S3D_SUBTYPES_LIST(curs3d);
                        curs3d->setNull(aux_type, ST_UNKNOWN, MM_DP_NONE, BLOB_BBOX(blob));
                        S3D_SUBTYPES_LIST(curs3d) = subtypes;
                    }
                }
            } else { //If solution is normal
                std::map<ObjectType, Shape3DData>::iterator normal_it,
                                                            normal_it_end = normal->end(),
                                                            occ_it;
                if(occ)
                    occ_it = occ->begin();
                for(normal_it = normal->begin(); normal_it != normal_it_end; normal_it++) {
                    aux_type = normal_it->first;

                    if(occ) {
                        curs3d = &occ_it->second;
                        subtypes = S3D_SUBTYPES_LIST(curs3d);
                        curs3d->setNull(aux_type, ST_UNKNOWN, MM_DP_NONE, BLOB_BBOX(blob));
                        S3D_SUBTYPES_LIST(curs3d) = subtypes;
                    }
                    curs3d = &normal_it->second;
                    if(type == aux_type) {
                        subtypes = S3D_SUBTYPES_LIST(curs3d);
                        memcpy(curs3d, &auxs3d, sizeof(Shape3DData));
                        S3D_SUBTYPES_LIST(curs3d) = subtypes;
                        bests3d = curs3d;
                    } else {
                        subtypes = S3D_SUBTYPES_LIST(curs3d);
                        curs3d->setNull(aux_type, ST_UNKNOWN, MM_DP_NONE, BLOB_BBOX(blob));
                        S3D_SUBTYPES_LIST(curs3d) = subtypes;
                    }
                    if(occ)
                        occ_it++;
                }
            }
        } else //No solution has been found
            bests3d = NULL;
      
        if(bests3d != NULL)
            bests3d->copyShape3DDataToBlob(blob);
    }

    return bests3d;
}

void ReliabilityClassification::setExtraPixelInfo(Blob *blob, ddata_t ddata, QImage *i_segm) {
    int i, j, counter = 0, width = i_segm->width(), height = i_segm->height();

    char moving_pixels[width*height];
    unsigned char *pixels = i_segm->bits();
    memset(moving_pixels, 0, width*height*sizeof(char));
    int llim = (BLOB_XLEFT(blob) + 1 >= 0) ? BLOB_XLEFT(blob) + 1 : 0, 
        rlim = (BLOB_XRIGHT(blob)  < width) ? BLOB_XRIGHT(blob)  : width - 1,
        blim = (BLOB_YBOTTOM(blob) < height) ? BLOB_YBOTTOM(blob) : height - 1,
        tlim = (BLOB_YTOP(blob)  + 1 >= 0) ? BLOB_YTOP(blob)  + 1 : 0;
    double di, dj, horRate, verRate;
    int cols_num = 0;

    setPixelRates(rlim - llim, blim - tlim, &horRate, &verRate);

    for(di = llim, i = (int)round(di); i < rlim; di += horRate, i=(int)round(di))
        cols_num++;

    for(di = llim, i = (int)round(di); i < rlim; di += horRate, i=(int)round(di)) {
        for(dj = tlim, j = (int)round(dj); j < blim ; dj += verRate, j=(int)round(dj)) {
            if(pixels[j*width + i])
                moving_pixels[counter] = 1;
            counter++;
        }
    }

    BLOB_MOVING_PIXELS(blob) = new char[counter];
    memcpy(BLOB_MOVING_PIXELS(blob), moving_pixels, counter*sizeof(char));

    DDATA_NPIX(ddata) = counter;
    DDATA_HRATE(ddata) = horRate;
    DDATA_VRATE(ddata) = verRate;
    DDATA_COLS_NUM(ddata) = cols_num;
}

//launch the classifyBlob function for each blob of the list, which will find the model class which corresponds to each blob
void ReliabilityClassification::classify(std::vector<Blob>& i_blobs, QImage *i_segm) {
    return constructClassifMap(i_blobs, i_segm);
}

bool ReliabilityClassification::parallelpipedBaseOnAOIGround(Parallelpiped *par, SceneModel *context) {
    int i;
    std::vector<QSharedPointer<world::AOI> > *aois = &context->AOIs;
    std::vector<QSharedPointer<world::AOI> >::iterator it, end_it = aois->end();
    //Check eight limit position of object to determine if it's inside an AOI.
    for(it = aois->begin(); it != end_it; it++) {
        for(i=0; i<4; i++)
            if( (*it)->pointInside(PARALL_X_i(par, i), PARALL_Y_i(par, i)) )
                return true;
    }
    return false;
}

bool ReliabilityClassification::blobOnAOIGround(Blob *i_blob, SceneModel *context) {
    int xCenter = BLOB_XCENTER(i_blob);
    int xRight = BLOB_XRIGHT(i_blob);
    int xLeft = BLOB_XLEFT(i_blob);
    int yCenter = BLOB_YCENTER(i_blob);
    int yBottom = BLOB_YBOTTOM(i_blob);
    int yTop = BLOB_YTOP(i_blob);
    double x3d, y3d;

    std::vector<QSharedPointer<world::AOI> > *aois = &context->AOIs;
    std::vector<QSharedPointer<world::AOI> >::iterator it, end_it = aois->end();
    //Check eight limit position of object to determine if it's inside an AOI.
    for(it = aois->begin(); it != end_it; it++) {

        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xLeft, yTop, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xCenter, yTop, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xRight, yTop, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;

        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xLeft, yCenter, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xCenter, yCenter, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xRight, yCenter, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;

        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xLeft, yBottom, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xCenter, yBottom, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(context), xRight, yBottom, 0.0, &x3d, &y3d);
        if( (*it)->pointInside(x3d, y3d) )
            return true;
    }

    return false;
}

bool ReliabilityClassification::prepareRun() {
    if(!initialized) {
        AppendToLog("ReliabilityClassification: error: Module not properly initialized. Aborting execution.");
        return false;
    }

    if( (m_foreground = m_data->fgImage) == NULL) {
        AppendToLog("ReliabilityClassification: error: fgImage is NULL in Datapool (A segmentation algorithm, as segmentationModule, sets it).");
        return false;
    }

    if(imageWorkingAreaNotComputed) {
        imageWorkingAreaNotComputed = false;
        computeImageWorkingArea(m_foreground);
    }

    return true;
}

bool ReliabilityClassification::init(){

    if(m_data->objectModels.size() == 0) {
        AppendToLog("ReliabilityClassification: error: At least one ObjectModel definition is required (ContextModule does it). See file 'config/parallelpiped-models.xml' as an example.");
        initialized = false;
        return false;
    }

    if( (m_context = m_data->sceneModel) == NULL) {
        AppendToLog("ReliabilityClassification: error: SceneModel is NULL (ContextModule does it).");
        initialized = false;
        return false;
    }

    std::map<ObjectType, SpModelInterface>::iterator modelsIt, EndIt = m_data->objectModels.end();
    m_mapModels.clear();
    for(modelsIt = m_data->objectModels.begin(); modelsIt != EndIt; modelsIt++)
        addModel((*modelsIt).first, (*modelsIt).second);

    double *pp = (double *)SM_CALIB_MATRIX(m_context);
    p00 = pp[0];    p01 = pp[1];    p02 = pp[2];    p03 = pp[3];
    p10 = pp[4];    p11 = pp[5];    p12 = pp[6];    p13 = pp[7];
    p20 = pp[8];    p21 = pp[9];    p22 = pp[10];   p23 = pp[11];

    //Pre-calculated constants for estimation of intial solution for static occlusion problem
    k1  = p10*p21 - p20*p11;
    k2  = p11*p00 - p01*p10;
    k3  = p00*p21 - p20*p01;
    //k4 depends on blob
    k5  = p22*p11 - p21*p12;
    k6  = p21*p02 - p22*p01;
    k7  = p01*p12 - p11*p02;
    //k8 depends on blob
    k9  = p11*p23 - p21*p13;
    k10 = p21*p03 - p01*p23;
    k11 = p23*p02 - p22*p03;
    k12 = p13*p02 - p12*p03;
    k13 = p11*p03 - p13*p01;
    k14 = p20*p12 - p22*p10;
    k15 = p00*p22 - p20*p02;
    k16 = p10*p02 - p00*p12;
    k17 = p20*p13 - p10*p23;
    k18 = p00*p23 - p20*p03;
    k19 = p00*p13 - p10*p03;
    //k20 to k34 depend on blob.
//ACA
    init_data();

    initialized = true;
    return true;
}

bool ReliabilityClassification::defaultParameters() {
    m_rcIntraCriteria = RCProbability;
    m_rcntop = 5;
    m_classifThres = 0.5;
    H_STEP_CM = 5.0;
    ALPHA_STEP = M_PI/40;
    DIM2D_STEP = 2;
    m_orderByDensity = true;
    m_pixelDensity = 0.50;
    RC_MIN_PIXELS = 50;
    RC_MAX_PIXELS = 200;
    m_HToleranceCoeff = 0.01;
    m_VToleranceCoeff = 0.01;
    m_treatBorderOcclusion = true;
    m_treatContextObjectOcclusion = true;
    RC_PROXIMITY_PIXELS = 5;
    m_maxRoofAndBaseIntersectionRatio = 0.9999;
    m_maxBaseAndBaseIntersectionRatio = 0.85;
    m_treatWallCoherence = false;
    m_cuttingAreaRate = 0.2;
    m_wallInsideLengthRate = 0.2;
    m_minPixelCoverageRatio = 0.01;

    return true;
}


bool ReliabilityClassification::setParameters(QDomNode& config) {
    QDomNode n;
    double value;

    if(config.isNull()) { //Parameter set for module not defined
        AppendToLog("ReliabilityClassification: Warning: No paramter definition. Taking defaults...");
        m_rcIntraCriteria = RCProbability;
        m_rcntop = 5;
        m_classifThres = 0.5;
        H_STEP_CM = 5.0;
        ALPHA_STEP = M_PI/40;
        DIM2D_STEP = 2;
        m_orderByDensity = true;
        m_pixelDensity = 0.50;
        RC_MIN_PIXELS = 50;
        RC_MAX_PIXELS = 200;
        m_HToleranceCoeff = 0.01;
        m_VToleranceCoeff = 0.01;
        m_treatBorderOcclusion = true;
        m_treatContextObjectOcclusion = true;
        RC_PROXIMITY_PIXELS = 5;
        m_maxRoofAndBaseIntersectionRatio = 0.9999;
        m_maxBaseAndBaseIntersectionRatio = 0.85;
        m_treatWallCoherence = false;
        m_cuttingAreaRate = 0.2;
        m_wallInsideLengthRate = 0.2;
        m_minPixelCoverageRatio = 0.01;
    } else {

        if( !( n = XmlCommon::getParameterNode("IntraCriteria", config) ).isNull() ) {
            QString s = XmlCommon::getParameterValue(n);
            AppendToLog("IntraCriteria:\t" + s);
            if(s == "RCProbability")
                m_rcIntraCriteria = RCProbability;
            else if(s == "RCReliabilityProbability")
                m_rcIntraCriteria = RCReliabilityProbability;
            else if(s == "RCReliability")
                m_rcIntraCriteria = RCReliability;
            else if(s == "RCDimensionalProbability")
                m_rcIntraCriteria = RCDimensionalProbability;
            else { //default
                m_rcIntraCriteria = RCProbability;
                AppendToLog("ReliabilityClassification: Warning: 'reliabilityClassificationIntraCriteria' not well defined. Taking default: RCProbability.");
            }

            QDomNode m;
            if( ( m = XmlCommon::getParameterNode("Threshold", n) ).isNull() ){
                m_classifThres = 0.5;
                AppendToLog("ReliabilityClassification: Warning: 'reliabilityClassificationIntraCriteria.classificationThreshold' not defined. Taking default: " + QString::number(m_classifThres));
            } else
                m_classifThres = XmlCommon::getParameterValue(m).toDouble();

            if( ( m = XmlCommon::getParameterNode("TopListSize", n) ).isNull() ) {
                m_rcntop = 5; //default
                AppendToLog("ReliabilityClassification: Warning: 'IntraCriteria.TopListSize' not defined. Taking default: " + QString::number(m_rcntop));
            } else
                m_rcntop = XmlCommon::getParameterValue(m).toInt();
        } else { //default
            m_rcIntraCriteria = RCProbability;
            m_rcntop = 5;
            m_classifThres = 0.5;
            AppendToLog("ReliabilityClassification: Warning: 'reliabilityClassificationIntraCriteria' not defined. Taking defaults: \n\t\tIntraCriteria = RCProbability\n\t\tIntraCriteria.classificationThreshold = " + QString::number(m_classifThres) + "\\t\tIntraCriteria.topListSize = " + QString::number(m_rcntop));
        }

        if( !( n = XmlCommon::getParameterNode("InterCriteria", config) ).isNull() ) {
            QString s = XmlCommon::getParameterValue(n);
            AppendToLog("InterCriteria:\t" + s);
            if(s == "RCProbability")
                m_rcInterCriteria = RCProbability;
            else if(s == "RCReliabilityProbability")
                m_rcInterCriteria = RCReliabilityProbability;
            else if(s == "RCReliability")
                m_rcInterCriteria = RCReliability;
            else if(s == "RCDimensionalProbability")
                m_rcInterCriteria = RCDimensionalProbability;
            else { //default
                AppendToLog("ReliabilityClassification: Warning: 'reliabilityClassificationInterCriteria' not well defined. Taking default: RCProbability.");
                m_rcInterCriteria = RCProbability;
            }

            QDomNode m;
            if( ( m = XmlCommon::getParameterNode("Threshold", n) ).isNull() ){
                m_classifThresInter = 0.5;
                AppendToLog("ReliabilityClassification: Warning: 'InterCriteria.Threshold' not defined. Taking default:" + QString::number(m_classifThresInter) + ".");
            } else
              m_classifThresInter = XmlCommon::getParameterValue(m).toDouble();

        } else { //default
            m_rcInterCriteria = RCProbability;
            m_classifThresInter = 0.50;
            AppendToLog("ReliabilityClassification: Warning: 'InterCriteria' not defined. Taking defaults: \n\t\tInterCriteria = RCProbability\n\t\tInterCriteria.Threshold = " + QString::number(m_classifThresInter));
        }

        if( ( n = XmlCommon::getParameterNode("HStepCM", config) ).isNull() ) {
            AppendToLog("ReliabilityClassification: Warning: 'HStepCM' not defined. Taking default.");
            H_STEP_CM = 5.0;            
        } else {
            value = XmlCommon::getParameterValue(n).toDouble();
            if(value > 0)
                H_STEP_CM = value;
            else {
                AppendToLog("ReliabilityClassification: Warning: 'HStepCM' defined as minor or equal to zero. Taking default.");
                H_STEP_CM = 5.0;
            }
        }

        if( ( n = XmlCommon::getParameterNode("AlphaStepRadians", config) ).isNull() ) {
            AppendToLog("ReliabilityClassification: Warning: 'AlphaStepRadians' not defined. Taking default: pi/40");
            ALPHA_STEP = M_PI/40;
        } else {
            value = XmlCommon::getParameterValue(n).toDouble();
            if(value > 0 && value < M_PI/2.0)
                ALPHA_STEP = value;
            else {
                AppendToLog("ReliabilityClassification: Warning: 'AlphaStepRadians' not taken from interval ]0,PI/2[ . Taking default: pi/40");
                ALPHA_STEP = M_PI/40;
            }
        }

        if( ( n = XmlCommon::getParameterNode("Dim2DStep", config) ).isNull() ) {
            AppendToLog("ReliabilityClassification: Warning: 'Dim2DStep' not defined. Taking default (2 pixels).");
            DIM2D_STEP = 2;
        } else {
            DIM2D_STEP = XmlCommon::getParameterValue(n).toInt();
            if(DIM2D_STEP <= 0) {
                AppendToLog("ReliabilityClassification: Warning: 'Dim2DStep' defined as minor or equal to zero. Taking default (2 pixels).");
                DIM2D_STEP = 2;
            }
        }

        if( ( n = XmlCommon::getParameterNode("OrderedByDensity", config) ).isNull() ) {
            AppendToLog("ReliabilityClassification: Warning: 'OrderedByDensity' not well defined ('true' or 'false') . Taking defaults.");
            m_orderByDensity = true;
            m_pixelDensity = 0.50;
            RC_MIN_PIXELS = 50;
            RC_MAX_PIXELS = 200;
        } else if (XmlCommon::getParameterValue(n) == "true") {
            m_orderByDensity = true;
            QDomNode m;
            if( ( m = XmlCommon::getParameterNode("DensityMaxPixels", n) ).isNull() ) {
                RC_MAX_PIXELS = 200;
                AppendToLog("ReliabilityClassification: Warning: 'DensityMaxPixels' not defined. Taking default: "
                          + RC_MAX_PIXELS);
            } else {
                RC_MAX_PIXELS = XmlCommon::getParameterValue(m).toInt();
                if(RC_MAX_PIXELS <= 0) {
                    RC_MAX_PIXELS = 200;
                    AppendToLog("ReliabilityClassification: Warning: 'DensityMaxPixels' not well defined. It must be a positive integer. Taking default: "
                              + RC_MAX_PIXELS);
                }
            }

            if( ( m = XmlCommon::getParameterNode("DensityMinPixels", n) ).isNull() ) {
                RC_MIN_PIXELS = 50;
                AppendToLog("ReliabilityClassification: Warning: 'DensityMinPixels' not defined. Taking default: " + QString::number(RC_MIN_PIXELS) + ".");
            } else {
                RC_MIN_PIXELS = XmlCommon::getParameterValue(m).toInt();
                if(RC_MIN_PIXELS >= RC_MAX_PIXELS) {
                    AppendToLog("ReliabilityClassification: Warning: 'DensityMinPixels' not well defined. It must be lower than 'DensityMaxPixels' and higher than 0. Taking default (DensityMaxPixels - 100) if (DensityMaxPixels - 100) > 0, 1 else.");
                    RC_MIN_PIXELS = (RC_MAX_PIXELS - 100 > 0) ? RC_MAX_PIXELS - 100 : 1;
                } else if(RC_MIN_PIXELS <= 0) {
                    RC_MIN_PIXELS = 50;
                    AppendToLog("ReliabilityClassification: Warning: 'DensityMinPixels' not well defined. It must be a positive integer. Taking default:" + QString::number(RC_MIN_PIXELS) + ".");
                }
            }

            if( ( m = XmlCommon::getParameterNode("PixelDensity", n) ).isNull() ) {
                m_pixelDensity = 0.50;
                AppendToLog("ReliabilityClassification: Warning: 'PixelDensity' not defined. Taking default: " + QString::number(m_pixelDensity) + ".");
            } else {
                m_pixelDensity = XmlCommon::getParameterValue(m).toDouble();
                if(m_pixelDensity <= 0) {
                    AppendToLog("ReliabilityClassification: Warning: 'PixelDensity' not well defined. It must be a positive real number. Taking default: 0.5.");
                    m_pixelDensity = 0.50;
                }
            }
        } else
          m_orderByDensity = false;

        if( ( n = XmlCommon::getParameterNode("HToleranceCoeff", config) ).isNull() ) {
            m_HToleranceCoeff = 0.01;
            AppendToLog("ReliabilityClassification: Warning: 'HToleranceCoeff' not defined. Taking default:" + QString::number(m_HToleranceCoeff));
        } else
            m_HToleranceCoeff = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("VToleranceCoeff", config) ).isNull() ) {
            m_VToleranceCoeff = 0.01;
            AppendToLog("ReliabilityClassification: Warning: 'VToleranceCoeff' not defined. Taking default:" + QString::number(m_VToleranceCoeff));
        } else
            m_VToleranceCoeff = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("BorderOcclusionTreatment", config) ).isNull() ) {
            AppendToLog("ReliabilityClassification: Warning: 'BorderOcclusionTreatment' not well defined ('true' or 'false') . Taking default: true");
            m_treatBorderOcclusion = true;
        } else {
            if(XmlCommon::getParameterValue(n) == "false")
                m_treatBorderOcclusion = false;
            else if (XmlCommon::getParameterValue(n) == "true")
                m_treatBorderOcclusion = true;
            else {
                AppendToLog("ReliabilityClassification: Warning: 'BorderOcclusionTreatment' not defined. Taking default: true");
                m_treatBorderOcclusion = true;
            }
        }

        if( !( n = XmlCommon::getParameterNode("ContextObjectOcclusionTreatment", config) ).isNull() ) {
            if(XmlCommon::getParameterValue(n) == "false")
                m_treatContextObjectOcclusion = false;
            else if (XmlCommon::getParameterValue(n) == "true")
                m_treatContextObjectOcclusion = true;
            else {
                AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment' not well defined ('true' or 'false').\n Taking default: true");
                m_treatContextObjectOcclusion = true;
            }

            if(m_treatContextObjectOcclusion) {

                QDomNode m;
                if( ( m = XmlCommon::getParameterNode("ProximityPixels", n) ).isNull() ) {
                    RC_PROXIMITY_PIXELS = 5;
                    AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.ProximityPixels' not defined.\n Taking default:" + QString::number(RC_PROXIMITY_PIXELS) + ".");
                } else {
                    RC_PROXIMITY_PIXELS = XmlCommon::getParameterValue(m).toInt();
                    if(RC_PROXIMITY_PIXELS <= 0) {
                        RC_PROXIMITY_PIXELS = 5;
                        AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.ProximityPixels' not well defined.\n It must be a positive integer. Taking default:" + QString::number(RC_PROXIMITY_PIXELS) + ".");
                    }
                }

                if( ( m = XmlCommon::getParameterNode("MaxRoofAndBaseIntersectionRatio", n) ).isNull() ) {
                    m_maxRoofAndBaseIntersectionRatio = 0.9999;
                    AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.MaxRoofAndBaseIntersectionRatio' not defined.\n Taking default:" + QString::number(m_maxRoofAndBaseIntersectionRatio) + ".");
                } else {
                    m_maxRoofAndBaseIntersectionRatio = XmlCommon::getParameterValue(m).toDouble();
                    if(m_maxRoofAndBaseIntersectionRatio < 0 || m_maxRoofAndBaseIntersectionRatio > 1) {
                        m_maxRoofAndBaseIntersectionRatio = 0.9999;
                        AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.MaxRoofAndBaseIntersectionRatio' not well defined.\n It must be real in [0 ; 1]. Taking default:" + QString::number(m_maxRoofAndBaseIntersectionRatio) + ".");
                    }
                }

                if( ( m = XmlCommon::getParameterNode("MaxBaseAndBaseIntersectionRatio", n) ).isNull() ) {
                    m_maxBaseAndBaseIntersectionRatio = 0.85;
                    AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.MaxBaseAndBaseIntersectionRatio' not defined.\n Taking default:" + QString::number(m_maxBaseAndBaseIntersectionRatio) + ".");
                } else {
                    m_maxBaseAndBaseIntersectionRatio = XmlCommon::getParameterValue(m).toDouble();
                    if(m_maxBaseAndBaseIntersectionRatio < 0 || m_maxBaseAndBaseIntersectionRatio > 1) {
                        m_maxBaseAndBaseIntersectionRatio = 0.85;
                        AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment.MaxBaseAndBaseIntersectionRatio' not well defined.\n It must be real in [0 ; 1]. Taking default:" + QString::number(m_maxBaseAndBaseIntersectionRatio) + ".");
                    }
                }
            }
        } else {
            m_treatContextObjectOcclusion = true;
            RC_PROXIMITY_PIXELS = 5;
            m_maxBaseAndBaseIntersectionRatio = 0.85;
            m_maxRoofAndBaseIntersectionRatio = 0.9999;
            AppendToLog("ReliabilityClassification: Warning: 'ContextObjectOcclusionTreatment' not defined. Taking defaults:\n\t\tContextObjectOcclusionTreatment = true\n\t\tContextObjectOcclusionTreatment.ProximityPixels = " + QString::number(RC_PROXIMITY_PIXELS) + "\t\tContextObjectOcclusionTreatment.MaxBaseAndBaseIntersectionRatio = " + QString::number(m_maxBaseAndBaseIntersectionRatio) + "\t\tContextObjectOcclusionTreatment.MaxRoofAndBaseIntersectionRatio = " + QString::number(m_maxRoofAndBaseIntersectionRatio) );
        }

        if( !( n = XmlCommon::getParameterNode("WallCoherenceTreatment", config) ).isNull() ) {
            if(XmlCommon::getParameterValue(n) == "false")
                m_treatWallCoherence = false;
            else if (XmlCommon::getParameterValue(n) == "true")
                m_treatWallCoherence = true;
            else {
                AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment' not well defined ('true' or 'false'). Taking default: false");
                m_treatWallCoherence = false;
            }

            if(m_treatWallCoherence) {
                QDomNode m;
                if( ( m = XmlCommon::getParameterNode("MaxCuttingAreaRateOfWall", n) ).isNull() ) {
                    m_cuttingAreaRate = 0.2;
                    AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment.MaxCuttingAreaRateOfWall' not defined. Taking default: " + QString::number(m_cuttingAreaRate));
                } else {
                    m_cuttingAreaRate = XmlCommon::getParameterValue(m).toDouble();
                    if(m_cuttingAreaRate < 0 || m_cuttingAreaRate > 0.5) {
                        m_cuttingAreaRate = 0.2;
                        AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment.MaxCuttingAreaRateOfWall' not well defined. It must correspond to a value between 0 and 0.5. Taking default: " + QString::number(m_cuttingAreaRate));
                    }
                }

                if( ( m = XmlCommon::getParameterNode("MaxLengthRateOfWallInBase", n) ).isNull() ) {
                    m_wallInsideLengthRate = 0.2;
                    AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment.MaxLengthRateOfWallInBase' not defined. Taking default: " + QString::number(m_wallInsideLengthRate));
                } else {
                    m_wallInsideLengthRate = XmlCommon::getParameterValue(m).toDouble();

                    if(m_wallInsideLengthRate < 0 || m_wallInsideLengthRate > 1.0) {
                        m_wallInsideLengthRate = 0.2;
                        AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment.MaxLengthRateOfWallInBase' not well defined. It must correspond to a value between 0 and 1. Taking default: " + QString::number(m_wallInsideLengthRate));
                    }
                }
            }
        } else {
            m_treatWallCoherence = true;
            m_cuttingAreaRate = 0.2;
            m_wallInsideLengthRate = 0.2;
            AppendToLog("ReliabilityClassification: Warning: 'WallCoherenceTreatment' not defined. Taking defaults:\n\t\tContextWallCoherenceTreatment = true\n\t\tContextWallCoherenceTreatment.MaxCuttingAreaRateOfWall = "  + QString::number(m_cuttingAreaRate) + "\n\t\tContextWallCoherenceTreatment.MaxLengthRateOfWallInBase = " + QString::number(m_wallInsideLengthRate));
        }

        if( ( n = XmlCommon::getParameterNode("MinPixelCoverageRatio", config) ).isNull() ) {
            m_minPixelCoverageRatio = 0.01;
            AppendToLog("ReliabilityClassification: Warning: 'MinPixelCoverageRatio' not defined. Taking default: " + QString::number(m_minPixelCoverageRatio));
        } else
            m_minPixelCoverageRatio = XmlCommon::getParameterValue(n).toDouble();
    }

    m_parametersSet = true;

    return true;
}


std::map<ObjectType, Shape3DData> *ReliabilityClassification::getBestOnesList(bool normal, Shape3DData **best_s3d, double *best, bool *exists3d, Blob *i_blob) {

    Shape3DData *current_s3d = NULL;
    std::map<ObjectType, Shape3DData> *s3d_list = NULL;
    bool better, subbetter;
    double subbest;
    bool rigid;
    SpModelInterface objectModel;
    ObjectType type;
    ObjectSubtype subtype;    
    std::map<ObjectType, SpModelInterface>::iterator modelsIt;
    std::map<ObjectSubtype, SpModelInterface>::iterator subModelsIt;
    std::map<ObjectSubtype, SpModelInterface> *parentModels;

    if(m_mapModels.size() > 0)
        s3d_list = new std::map<ObjectType, Shape3DData>();

    for(modelsIt = m_mapModels.begin(); modelsIt!=m_mapModels.end(); modelsIt++) {
        type = (*modelsIt).first;
        objectModel = (*modelsIt).second;
        rigid = objectModel->IsRigid;

        if(rigid) {
            (*s3d_list)[type] = *(normal ? m_top[type][0] : m_topocc[type][0]);
            current_s3d = &(*s3d_list)[type];
            S3D_SUBTYPE(current_s3d) = ST_NO_SUBTYPES;
            if(S3D_TYPE(current_s3d) == UNKNOWN) {
                memcpy(S3D_BBOX(current_s3d), BLOB_BBOX(i_blob), sizeof(Rectangle<int>));
                S3D_P(current_s3d)=0.0;
                S3D_PR(current_s3d)=0.0;
                S3D_DP(current_s3d)=0.0;
                S3D_R(current_s3d)=0.0;
                S3D_TYPE(current_s3d)=type;
                S3D_PW(current_s3d) = S3D_PL(current_s3d) = S3D_PH(current_s3d) = 0.0;
                S3D_RW(current_s3d) = S3D_RL(current_s3d) = S3D_RH(current_s3d) = 0.0;
                S3D_BBOX(current_s3d)->setPositionAtCenterBottom(m_context, *S3D_3D_POSITION(current_s3d));
            } else {
                if(normal)
                    memcpy(S3D_BBOX(current_s3d), BLOB_BBOX(i_blob), sizeof(Rectangle<int>));
                S3D_3DBBOX(current_s3d)->setReal3DPosition(S3D_3D_POSITION(current_s3d));
            }

            if(normal ? m_ntop[type]>0 : m_ntopocc[type]>0) {
                *exists3d = true;
                if((better = checkIfBetter(best, current_s3d)))
                    *best_s3d = current_s3d;
            }

        } else { //postural
            std::map<ObjectSubtype, Shape3DData> *s3d_sublist = NULL;
            Shape3DData *current_subs3d, *best_subs3d;

            parentModels = &(objectModel->m_mapPostures);

            s3d_sublist = new std::map<ObjectSubtype, Shape3DData>();

            for(subModelsIt = parentModels->begin(); subModelsIt != parentModels->end(); subModelsIt++) {

                subtype = (*subModelsIt).first;
                (*s3d_sublist)[subtype] = *(normal ? m_sub_top[type][subtype][0] : m_sub_topocc[type][subtype][0]);
                best_subs3d = current_subs3d = &(*s3d_sublist)[subtype];
                subbest = 0;
                if(S3D_TYPE(current_subs3d) == UNKNOWN) {
                    memcpy(S3D_BBOX(current_subs3d), BLOB_BBOX(i_blob), sizeof(Rectangle<int>));
                    S3D_P(current_subs3d)=0.0;
                    S3D_PR(current_subs3d)=0.0;
                    S3D_DP(current_subs3d)=0.0;
                    S3D_R(current_subs3d)=0.0;
                    S3D_TYPE(current_subs3d)=type;
                    S3D_SUBTYPE(current_subs3d)=subtype;
                    S3D_PW(current_subs3d) = S3D_PL(current_subs3d) = S3D_PH(current_subs3d) = 0.0;
                    S3D_RW(current_subs3d) = S3D_RL(current_subs3d) = S3D_RH(current_subs3d) = 0.0;
                    S3D_BBOX(current_subs3d)->setPositionAtCenterBottom(m_context, *S3D_3D_POSITION(current_subs3d));
                } else {
                    if(normal)
                        memcpy(S3D_BBOX(current_subs3d), BLOB_BBOX(i_blob), sizeof(Rectangle<int>));
                    S3D_3DBBOX(current_subs3d)->setReal3DPosition(S3D_3D_POSITION(current_subs3d));
                }

                if(normal ? m_sub_ntop[type][subtype]>0 : m_sub_ntopocc[type][subtype]>0) {
                    *exists3d = true;
                    if((subbetter = checkIfBetter(&subbest, current_subs3d)))
                        best_subs3d = current_subs3d;
                }
            }

            //Copy the best found for submodels
            (*s3d_list)[type] = *best_subs3d;

            if((*exists3d) && ((better=checkIfBetter(best, best_subs3d))))
                *best_s3d = best_subs3d;

            delete s3d_sublist;
        }
    }


    return s3d_list;
}

void ReliabilityClassification::assignTypeToBlob(Blob *i_blob) {
    Shape3DData *best_s3d = NULL;
    std::map<ObjectType, Shape3DData> *normalList;
    double best = 0;
    bool exists3d = false;
    Shape3DData *best_s3d_occ = NULL;
    double best_occ = 0;
    bool exists3docc = false;

    BLOB_P(i_blob) = 0.0;

    if(BLOB_NORMAL_3DDATA(i_blob) != NULL) {
        delete BLOB_NORMAL_3DDATA(i_blob);
        BLOB_NORMAL_3DDATA(i_blob) = NULL;
    }

    if(BLOB_OCC_3DDATA(i_blob) != NULL) {
        delete BLOB_OCC_3DDATA(i_blob);
        BLOB_OCC_3DDATA(i_blob) = NULL;
    }

    //Set results per type list for normal list
    normalList = BLOB_NORMAL_3DDATA(i_blob) = getBestOnesList(true, &best_s3d, &best, &exists3d, i_blob);

    //Set results per type list for occlusion list
    if(someOcclusionSolutionInserted)
        BLOB_OCC_3DDATA(i_blob) = getBestOnesList(false, &best_s3d_occ, &best_occ, &exists3docc, i_blob);

    if( (!exists3d || best_s3d == NULL) && exists3docc && best_s3d_occ != NULL ) { //If normal solution doesn't exist and occlusion solution exists
        if(S3D_P(best_s3d_occ) > m_classifThresInter) { //Occlusion solution is good enough
            best_s3d_occ->copyShape3DDataToBlob(i_blob);
            BLOB_BEST_3DDATA(i_blob)=best_s3d_occ;
            BLOB_OCCLUSION(i_blob) = true;
        } else { //Is also not a good solution
            best_s3d_occ->copyShape3DDataToBlob(i_blob);
            BLOB_OCCLUSION(i_blob) = false;
            BLOB_TYPE(i_blob) = UNKNOWN;
            BLOB_BEST_3DDATA(i_blob) = NULL;
            BLOB_BBOX(i_blob)->setPositionAtCenterBottom(m_context, *BLOB_3D_POSITION(i_blob));
        }
    } else if(exists3d && best_s3d != NULL) { //If normal solution exist ...
        if (exists3docc && best_s3d_occ != NULL) { //and also exist a occlusion solution...
            if(    (S3D_P(best_s3d) < m_classifThresInter/2.0 && S3D_P(best_s3d_occ) > m_classifThresInter)
                || (S3D_P(best_s3d) < m_classifThresInter && S3D_P(best_s3d_occ) >  m_classifThresInter && S3D_P(best_s3d_occ) >  0.95 ) ) {
                //If normal solution is not good enough and occlusion solution is good enough.
                best_s3d_occ->copyShape3DDataToBlob(i_blob);
                BLOB_BEST_3DDATA(i_blob) = best_s3d_occ;
                BLOB_OCCLUSION(i_blob) = true;
            } else if( S3D_P(best_s3d) < m_classifThresInter) { //If normal solution is not good enough.
                best_s3d->copyShape3DDataToBlob(i_blob);
                BLOB_OCCLUSION(i_blob) = false;
                BLOB_BEST_3DDATA(i_blob) = NULL;
                BLOB_TYPE(i_blob) = UNKNOWN;
                BLOB_BBOX(i_blob)->setPositionAtCenterBottom(m_context, *BLOB_3D_POSITION(i_blob));
            } else { //If normal solution is good enough
                best_s3d->copyShape3DDataToBlob(i_blob);
                BLOB_BEST_3DDATA(i_blob) = best_s3d;
                BLOB_OCCLUSION(i_blob) = false;
            }
        } else { //and if there is no occlusion solution.
            if( S3D_P(best_s3d) < m_classifThresInter) { //If normal solution is not good enough.
                best_s3d->copyShape3DDataToBlob(i_blob);
                BLOB_OCCLUSION(i_blob) = false;
                BLOB_BEST_3DDATA(i_blob) = NULL;
                BLOB_TYPE(i_blob) = UNKNOWN;
                BLOB_BBOX(i_blob)->setPositionAtCenterBottom(m_context, *BLOB_3D_POSITION(i_blob));
            } else { //If normal solution is good enough
                best_s3d->copyShape3DDataToBlob(i_blob);
                BLOB_BEST_3DDATA(i_blob) = best_s3d;
                BLOB_OCCLUSION(i_blob) = false;
            }
        }
    } else { //No solution at all
        BLOB_OCCLUSION(i_blob) = false;
        BLOB_BEST_3DDATA(i_blob) = NULL;
        //BLOB_DP_TYPE(i_blob) = MM_DP_NONE;
        BLOB_TYPE(i_blob) = UNKNOWN;
        BLOB_BBOX(i_blob)->setPositionAtCenterBottom(m_context, *BLOB_3D_POSITION(i_blob));
    }

    //Set blob bounding box to normal size
    memcpy(BLOB_BBOX(i_blob), S3D_BBOX(&BLOB_NORMAL_3DDATA(i_blob)->begin()->second), sizeof(Rectangle<int>));
    //Set blob occlusion type to the possible occlusion, even if there is no real occlusion
    if(BLOB_OCC_3DDATA(i_blob))
        BLOB_DP_TYPE(i_blob) = S3D_DP_TYPE(&BLOB_OCC_3DDATA(i_blob)->begin()->second);
}

// Change blob type to UNKNOWN if blob is not included on aoi in 3d
bool ReliabilityClassification::changeBlobTypeToUnknown(Blob *i_blob) {
    // Verify
    if(i_blob == NULL){
        AppendToLog("ReliabilityClassification: Error: blob type unknown - no valid blob");
        return false;
    }

    if(BLOB_DP_TYPE(i_blob) == MM_AOI_OUTSIDE) {
        if(BLOB_TYPE(i_blob) != UNKNOWN)
            BLOB_TYPE(i_blob) = UNKNOWN;
        else
            BLOB_TYPE(i_blob) = NOISE;
    }

    return true;
}

bool ReliabilityClassification::checkIfBetter(double *best, Shape3DData *s3d){

    bool better=false;

    switch(m_rcInterCriteria) {
        case RCReliability:
            if(S3D_R(s3d) >= *best) {
                *best = S3D_R(s3d);
                better=true;
            }
            break;
        case RCReliabilityProbability:
            if(S3D_PR(s3d) >= *best) {
                *best = S3D_PR(s3d);
                better=true;
            }
            break;
        case RCDimensionalProbability:
            if(S3D_DP(s3d) >= *best) {
                *best = S3D_DP(s3d);
                better=true;
            }
          break;
        case RCProbability:
        default:
            if(S3D_P(s3d) >= *best) {
                *best = S3D_P(s3d);
                better=true;
            }
    }

    return better;
}


inWall::inWall() {
    wall = NULL;
    leaves = enters = 0.0;
    entering_type = leaving_type = 0;
}

inWall::inWall(world::Wall2D * i_wall, double i_enters, int i_enter_type, double i_leaves, int i_leave_type) {
    wall = i_wall;
    enters = i_enters;
    leaves = i_leaves;
    entering_type = i_enter_type; 
    leaving_type = i_leave_type; 
}

inWall::~inWall() { }

world::Wall2D *inWall::getWall() {
    return wall;
}

double inWall::getEntering() {
    return enters;
}

double inWall::getLeaving() {
    return leaves;
}

int inWall::getEnteringType() {
    return entering_type;
}

int inWall::getLeavingType() {
    return leaving_type;
}

inWall& inWall::operator=(const inWall& inwall) {
    wall = inwall.wall;
    enters = inwall.enters;
    leaves = inwall.leaves;
    entering_type = inwall.entering_type;
    leaving_type = inwall.leaving_type;

    return *this;
}
  
void inWall::reverseOrder() {
    double enters_aux = enters;
    int entering_type_aux = entering_type;
    enters = leaves;
    entering_type = leaving_type;
    leaves = enters_aux;
    leaving_type = entering_type_aux;
}

