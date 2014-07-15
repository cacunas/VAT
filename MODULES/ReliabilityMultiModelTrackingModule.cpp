#include "ReliabilityMultiModelTrackingModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include "src/object.h"
#include <QPointer>

ReliabilityMultiModelTrackingModule::ReliabilityMultiModelTrackingModule(Datapool *i_data): ModuleInterface(i_data) {
    m_RMMTracker = new RMMTracker(i_data);
}

ReliabilityMultiModelTrackingModule::~ReliabilityMultiModelTrackingModule() {
    delete m_RMMTracker;
    //QSharedPointer
}

//Set module configuration parameters
bool ReliabilityMultiModelTrackingModule::setParameters(QDomNode& config) {
  return m_RMMTracker->setParameters(config);
}

bool ReliabilityMultiModelTrackingModule::updateParameters(){
    return true;
}

//Function executed at each frame
bool ReliabilityMultiModelTrackingModule::run() {
    if(m_RMMTracker->initialPreparation)
        if(!m_RMMTracker->initialPrepareRun())
            return false;

    if(!m_RMMTracker->prepareRun())
        return false;
    m_RMMTracker->run(m_data->blobs);

    //Set most likely mobiles
    m_RMMTracker->getMostLikelyMobileObjects(m_data->RMMobjects);

    if(m_RMMTracker->m_setObjectLog)
       m_RMMTracker->writeObjectLog();


  //  Blob b;
  //  Object o;
  //  int X1,Y1,X2,Y2,id;
  //  double X,Y,x,y,Vx,Vy;
  //  ObjectType type;
  //  std::map<int, Object>::iterator it, end_it;

  //                      b.Xu = Y1 < Y2 ? Y1 : Y2;
                        //Y upper is leftmost horizontal value
  //                      b.Yl = X1 < X2 ? X1 : X2;
                        //Height (in number of pixels):
  //                      b.H = (Y1 > Y2 ? Y1 : Y2) - b.Xu + 1;
                        //Height (in number of pixels):
  //                      b.W = (X1 > X2 ? X1 : X2) - b.Yl + 1;
  //                      Scene::getXY(m_pos,X,Y,b);
  //                      Scene::img_to_world_coords_given_height(m_data->sceneModel->p_matrix, X, Y, 0.0, &x, &y);

    return true;
}

bool ReliabilityMultiModelTrackingModule::init() {
    return m_RMMTracker->init();
}




