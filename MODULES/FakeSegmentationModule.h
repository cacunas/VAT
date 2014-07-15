#ifndef __FAKESEGMENTATION_MODULE_H__
#define __FAKESEGMENTATION_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"

#include <QDir>

class FakeSegmentationModule: public ModuleInterface {
 public:
  FakeSegmentationModule(Datapool *i_data);
  ~FakeSegmentationModule();

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
  QDomDocument *xmlGT;
  QDomElement root;
  QDomNode currentNode;
  int currentFrame;
  int line;
};


#endif
