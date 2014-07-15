#ifndef __ETISEOOUTPUT_MODULE_H__
#define __ETISEOOUTPUT_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"


class ETISEOOutputModule: public ModuleInterface {
 public:
  ETISEOOutputModule(Datapool *i_data);
  ~ETISEOOutputModule();

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
  QString m_outputFileName;
  int m_endFrame;
  QFile m_file;
  std::map<unsigned long, int> startingFrame;
  int currentFrame;
};


#endif
