#include "ModelInterface.h"

Criterion::Criterion() :    spFunction(NULL),
                            score(0.0), reliability(0.0) {}

Criterion::~Criterion() {}

ModelInterface::ModelInterface(): velocity_defined(false), IsRigid(false) {}

ModelInterface::ModelInterface(Datapool *i_data): velocity_defined(false),
                                                  IsRigid(false),
                                                  m_data(i_data) {}
ModelInterface::~ModelInterface() {}

void ModelInterface::addSubModel(ObjectSubtype i_subType, SpModelInterface i_spModel) {
          //add the model to the model class map
          m_mapPostures[i_subType] = i_spModel;
}

double ModelInterface::computeScore(Blob *i_pBlob, SceneModel *i_pSceneModel) {
    computeCriteriaScores(i_pBlob,i_pSceneModel);
    computeReliabilities(i_pBlob,i_pSceneModel);

    double error = 0.000001;
    double mulScores = 1.0;
    double sumScores = 0.0;
    double sumScores2 = 0.0;
    double sumRelScores = 0.0;
    double sumWeights = 0.0;
    double dim, dimSum = 0.0;
    std::map<QString, Criterion>::iterator itMap;
    for (itMap = m_mapCriteria.begin(); itMap != m_mapCriteria.end(); itMap++) {
        if (itMap->first.compare("velocity")) {
            dim=itMap->second.spFunction->getMean();
            dimSum += dim;
            sumRelScores += itMap->second.reliability * itMap->second.score;
            sumScores += itMap->second.reliability;
            sumScores2 += itMap->second.score * dim;
            mulScores *= itMap->second.score;
            sumWeights += itMap->second.reliability;
        }
    }

    BLOB_P(i_pBlob) = mulScores;
    if(sumWeights > error) {
        BLOB_PR(i_pBlob) = sumRelScores/sumWeights;
        BLOB_R(i_pBlob) = sumScores/3.0;
    } else
        BLOB_R(i_pBlob) = BLOB_PR(i_pBlob) = 0.0;

    BLOB_DP(i_pBlob) = sumScores2 / dimSum;

    return mulScores;
}

void ModelInterface::computeCriteriaScores(Blob *i_pBlob, SceneModel *i_pSceneModel) {
    double D3D=0.0, H3D=0.0, W3D=0.0;

    D3D = BLOB_3D_LENGTH(i_pBlob);
    W3D = BLOB_3D_WIDTH(i_pBlob);
    H3D = BLOB_3D_HEIGHT(i_pBlob);

    //blob criters computing
    std::map<QString, Criterion>::iterator itMap;
    for (itMap = m_mapCriteria.begin(); itMap != m_mapCriteria.end(); itMap++) {
        if (itMap->first == "width") {
            itMap->second.score = itMap->second.spFunction->getValue(W3D);
            BLOB_PW(i_pBlob) = itMap->second.score;
        } else if (itMap->first == "depth") {
            itMap->second.score = itMap->second.spFunction->getValue(D3D);
            BLOB_PL(i_pBlob) = itMap->second.score;
        } else if (itMap->first == "height") {
            itMap->second.score = itMap->second.spFunction->getValue(H3D);
            BLOB_PH(i_pBlob) = itMap->second.score;
        }
    }
}

void ModelInterface::computeReliabilities(Blob *i_pBlob, SceneModel *i_pSceneModel) {
    int h_near_ind = -1;
    double minD = DBL_MAX;
    double xf2d = i_pSceneModel->camera_focal_point2D->x;
    double yf2d = i_pSceneModel->camera_focal_point2D->y;
    double dhX, dwX, dlX, dhY, dwY, dlY, dXaux, dYaux;
    double dx, dy, H, W;
    DetectionProblemType dptype = BLOB_DP_TYPE(i_pBlob);
    int occH, occW;
    int i;


    //Search the nearest point.
    for(i = 0; i < 4; i++) {
        dx = xf2d - BLOB_3DBBOX_X2D_BASE_i(i_pBlob, i);
        dy = yf2d - BLOB_3DBBOX_Y2D_BASE_i(i_pBlob, i);
        if(dx*dx + dy*dy < minD) {
            minD = dx*dx + dy*dy;
            h_near_ind = i;
        }
    }

    //Calculate length of 2D projections
    dhX = fabs(BLOB_3DBBOX_X2D_BASE_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_X2D_H_i(i_pBlob, h_near_ind));
    dhY = fabs(BLOB_3DBBOX_Y2D_BASE_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_Y2D_H_i(i_pBlob, h_near_ind));
    dlX = fabs(BLOB_3DBBOX_X2D_H_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_X2D_H_i(i_pBlob, (h_near_ind + 1 == 4)? 0 : h_near_ind + 1 ) );
    dlY = fabs(BLOB_3DBBOX_Y2D_H_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_Y2D_H_i(i_pBlob, (h_near_ind + 1 == 4)? 0 : h_near_ind + 1 ) );
    dwX = fabs(BLOB_3DBBOX_X2D_H_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_X2D_H_i(i_pBlob, (h_near_ind - 1  < 0)? 3 : h_near_ind - 1 ) );
    dwY = fabs(BLOB_3DBBOX_Y2D_H_i(i_pBlob, h_near_ind) - BLOB_3DBBOX_Y2D_H_i(i_pBlob, (h_near_ind - 1  < 0)? 3 : h_near_ind - 1 ) );

    if(h_near_ind % 2 == 0) { //Invert values if w is l and viceversa
        dXaux = dlX;
        dYaux = dlY;
        dlX = dwX;
        dlY = dwY;
        dwX = dXaux;
        dwY = dYaux;
    }

    H = BLOB_HEIGHT(i_pBlob);
    W = BLOB_WIDTH(i_pBlob);

    occH = (dptype & (MM_CAM_OCCL_BOTTOM | MM_CAM_OCCL_TOP  )) ? 0 : 1;
    occW = (dptype & (MM_CAM_OCCL_LEFT   | MM_CAM_OCCL_RIGHT)) ? 0 : 1;

    std::map<QString, Criterion>::iterator itMap;
    for (itMap = m_mapCriteria.begin(); itMap != m_mapCriteria.end(); itMap++) {
        if (!(itMap->first.compare("width"))) {
            itMap->second.reliability = std::min( occH*dwY/H + occW*dwX/W, 1.0 );
            BLOB_RW(i_pBlob) = itMap->second.reliability;
        } else if (!(itMap->first.compare("depth"))) {
            itMap->second.reliability = std::min( occH*dlY/H + occW*dlX/W, 1.0 );
            BLOB_RL(i_pBlob) = itMap->second.reliability;
        } else if (!(itMap->first.compare("height"))) {
            itMap->second.reliability = (h_near_ind>=0)?( std::min( occH*dhY/H + occW*dhX/W, 1.0 ) ):(0.0);
            BLOB_RH(i_pBlob) = itMap->second.reliability;
        }
    }
}

