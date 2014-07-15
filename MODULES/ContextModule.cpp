#include "ContextModule.h"
#include "src/calibration.h"
#include "VideoAnalysis.h"

#include <errno.h>
#include <iostream>

ContextModule::ContextModule(Datapool *i_data):
  ModuleInterface(i_data), m_fileName(""), m_ObjectsFileName("") {
    oreader = new ObjectModelReader(i_data);
    focalNotChecked = true;
    m_current = NULL;
}

ContextModule::~ContextModule() {
    delete oreader;
}

//Set module configuration parameters
bool ContextModule::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        m_fileName = "config/default_scene.xml";
        m_ObjectsFileName = "config/parallelpiped-models.xml";
        m_OneMeterRepresentation = 100; //defaults to centimeters.
    } else {
        if( ( n = XmlCommon::getParameterNode("SceneModelFileName", config) ).isNull() ) {
            AppendToLog("ContextModule: warning: Parameter 'SceneModelFileName' not defined. Taking default 'config/default_scene.xml'.");
            m_fileName = "config/default_scene.xml";
        } else
            m_fileName = XmlCommon::getParameterValue(n);

        if( ( n = XmlCommon::getParameterNode("ObjectModelsFileName", config) ).isNull() ) {
            AppendToLog("ContextModule: warning: Parameter 'ObjectModelsFileName' not defined. Taking default 'config/parallelpiped-models.xml'.");
            m_ObjectsFileName = "config/parallelpiped-models.xml";
        } else
            m_ObjectsFileName = XmlCommon::getParameterValue(n);

        if( ( n = XmlCommon::getParameterNode("OneMeterRepresentation", config) ).isNull() ) {
            AppendToLog("ContextModule: warning: Parameter 'OneMeterRepresentation' not defined. Taking default: 100 (centimeters).");
            m_OneMeterRepresentation = 100;
        } else {
            if( (m_OneMeterRepresentation = XmlCommon::getParameterValue(n).toDouble()) <= 0) {
                AppendToLog("ContextModule: warning: Parameter 'OneMeterRepresentation' not well defined. It must be higher than zero. Taking default: 100 (centimeters).");
                m_OneMeterRepresentation = 100.0;
            }
        }
        if( ( n = XmlCommon::getParameterNode("MaximalObjectSpeed", config) ).isNull() ) {
            AppendToLog("ContextModule: warning: Parameter 'MaximalObjectSpeed' not defined. Taking default: 5000 [cm/s].");
            m_MaximalObjectSpeed = 5000.0;
        } else {
            if( (m_MaximalObjectSpeed = XmlCommon::getParameterValue(n).toDouble()) <= 0) {
                AppendToLog("ContextModule: warning: Parameter 'MaximalObjectSpeed' not well defined. It must be higher than zero. Taking default: 5000 [cm/s].");
                m_MaximalObjectSpeed = 5000.0;
            }
        }
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("SceneModelFileName", m_fileName, "QString");
    addParameter("ObjectModelsFileName", m_ObjectsFileName, "QString");
    addParameter("OneMeterRepresentation", QString::number(m_OneMeterRepresentation), "double");
    addParameter("MaximalObjectSpeed", QString::number(m_MaximalObjectSpeed), "double");
    return true;
}

bool ContextModule::updateParameters(){
    parameter *smfn, *omfn, *omr, *mos;
    smfn = getParameter("SceneModelFileName");
    omfn = getParameter("ObjectModelsFileName");
    omr = getParameter("OneMeterRepresentation");
    mos = getParameter("MaximalObjectSpeed");

    if( smfn == 0 || omfn == 0 || omr == 0 || mos == 0 ||
            smfn->value.isEmpty() || smfn->value.isNull() ||
            omfn->value.isEmpty() || omfn->value.isNull() ||
            omr->value.toDouble() < 0.0 || mos->value.toDouble() < 0.0)
        return false;

    m_fileName = smfn->value;
    m_ObjectsFileName = omfn->value;
    m_OneMeterRepresentation = omr->value.toDouble();
    m_MaximalObjectSpeed = mos->value.toDouble();
    return true;
}

bool ContextModule::init() {    
    if(m_data->sceneModel == NULL)
        m_data->sceneModel = new SceneModel();
    m_data->sceneModel->m_MaximalObjectSpeed = m_MaximalObjectSpeed;
    m_data->sceneModel->m_OneMeterRepresentation = m_OneMeterRepresentation;
    return    m_data->sceneModel->readScene(m_fileName)
           && oreader->readObjectModels(m_ObjectsFileName);
}

//Function executed at each frame
bool ContextModule::run() {
    if(focalNotChecked && m_data->sceneModel != NULL && (m_data->sceneModel->pmatrix_filled || m_data->sceneModel->hmatrix_filled) ) {
        focalNotChecked = false;

        if( (m_current = m_data->currentImage) == NULL) {
            AppendToLog("ContextModule: error: currentImage is NULL in Datapool (An acquisition function, as AcquisitionModule, sets it).");
            return false;
        }

        double camx, camy, camz;
        if(m_data->sceneModel->pmatrix_filled) {
            if(!m_data->sceneModel->computeFocalPoint(m_data->sceneModel->p_matrix, m_current->width(),
                                                      m_current->height(), &camx, &camy, &camz) ) {
                std::cout << "ContextModule: Warning: Focal point calculation failed. Microscope images?" << std::endl;
                m_data->sceneModel->badFocal = true;
            } else {
                m_data->sceneModel->badFocal = false;
                m_data->sceneModel->camera_focal_point->x = camx;
                m_data->sceneModel->camera_focal_point->y = camy;
                m_data->sceneModel->camera_focal_point->z = camz;
    //            SceneModel::worldToImgCoords(m_data->sceneModel->p_matrix, camx, camy, camz,
                SceneModel::worldToImgCoords(m_data->sceneModel->p_matrix, camx, camy, 0,
                                               &m_data->sceneModel->camera_focal_point2D->x,
                                             &m_data->sceneModel->camera_focal_point2D->y);
            }
        } else { //hmatrix_filled
            //if(!m_data->sceneModel->computeFocalPointFromHomography(m_data->sceneModel->p_matrix, m_current->width(),
            //                                          m_current->height(), &camx, &camy, &camz) ) {
            //    std::cout << "ContextModule: Warning: Focal point calculation failed. Parallel proyection?" << std::endl;
            //    m_data->sceneModel->badFocal = true;
            //} else {
            m_data->sceneModel->badFocal = false;
            m_data->sceneModel->camera_focal_point2D->x = m_current->width()/2;
            m_data->sceneModel->camera_focal_point2D->y = m_current->height();

            m_data->sceneModel->camera_focal_point->z = 0.0;
            SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix, m_data->sceneModel->camera_focal_point2D->x, m_data->sceneModel->camera_focal_point2D->y,
                                                  &m_data->sceneModel->camera_focal_point->x,
                                                  &m_data->sceneModel->camera_focal_point->y);
        }
    }

    if(m_data->sceneModel == NULL)
        m_data->sceneModel->badFocal = true;

    return true;
}

