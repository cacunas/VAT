#ifndef __CONTEXT_MODULE_H__
#define __CONTEXT_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include "src/ObjectModelReader.h"
#include <QDir>

class ContextModule: public ModuleInterface {
 public:
  ContextModule(Datapool *i_data);
  ~ContextModule();

  //Set module configuration parameters
  bool setParameters(QDomNode &config);

  //Initialization after reading parameters
  bool init();

  //Function executed at each frame
  bool run();

  //update parameters at runtime.
  bool updateParameters();

  private:
  //Internal
  QString m_fileName;
  QString m_ObjectsFileName;
  ObjectModelReader *oreader;
  double m_MaximalObjectSpeed;
  double m_OneMeterRepresentation;
  bool focalNotChecked;
  QImage *m_current;

};


#endif
