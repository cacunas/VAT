#ifndef __FAKETRACKING_MODULE_H__
#define __FAKETRACKING_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "src/calibration.h"
#include "ModuleInterface.h"
#include <QDir>

class FakeTrackingModule: public ModuleInterface {
 public:
  FakeTrackingModule(Datapool *i_data);
  ~FakeTrackingModule();

  //Set module configuration parameters
  bool setParameters(QDomNode& config);
  
  //Initialization after reading parameters
  bool init();

  //Function executed at each frame
  bool run();

  //update parameters at runtime.
  bool updateParameters();

  private:
  //Internal data:
  QString m_fileName;
  position m_pos;
  QDomDocument *xmlGT;
  QDomElement root;
  QDomNode currentNode;
  int currentFrame;
  int line;
};


#endif
