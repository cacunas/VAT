#ifndef __ETISEO_ACQUISITION_MODULE_H__
#define __ETISEO_ACQUISITION_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include <QDir>
#include "string.h"
#include "ModuleInterface.h"

class ETISEOAcquisitionModule: public ModuleInterface {
 public:
  ETISEOAcquisitionModule(Datapool *i_data);
  ~ETISEOAcquisitionModule();

  //Set module configuration parameters
  bool setParameters(QDomNode& config);
  
  //Initialization after reading parameters
  bool init();

  //Function executed at each frame
  bool run();

  //update parameters at runtime.
  bool updateParameters();

  static int m_defaultMillisecs;
  QImage *getNextFrame();
  QImage *getFrame(int i_numFrame);
  int getNumFrame();
  bool goToFirstFrame();
  void openDir();
  void openFiles();
  bool readImageOnDisk(QImage **);
  void readTimeStamp(QString&);
  void setDirectory(QString dir);
  void setDirectory();
  void initialFrame();
  void setCurrentFiles(int num);
  void readETISEOHeader(ImageHeader &H, QString &time);

  private:
  //Internal data:
  int m_startFrame;
  QString m_seqDir;
  bool m_parseDirectory;
  bool firstTime;
  int m_currentImageIndex;
  int m_currentDirIndex;
  QDir directory;
  QDir subdirectory;
  QStringList directories;
  QStringList currentFilesFromDir;
  std::vector<QString> currentFiles;
  int m_numberOfDirs;
  int m_numberOfFiles;
  int m_imageType;
  int m_NumFrame;
  QString m_videoDirectory;
  QString m_fileName;
};

#endif
