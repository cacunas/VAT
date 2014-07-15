#ifndef __ACQUISITION_MODULE_H__
#define __ACQUISITION_MODULE_H__

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include <QDir>
#include "string.h"
#include "ModuleInterface.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>

class AcquisitionModule: public ModuleInterface {
 public:
  AcquisitionModule(Datapool *i_data); 
  ~AcquisitionModule();

  //Set module configuration parameters
  bool setParameters(QDomNode& config);
  
  //Initialization after reading parameters
  bool init();

  //Function executed at each frame
  bool run();

  //update parameters at runtime.
  bool updateParameters();

cv::Mat opencvImage;
  //opencv2::Mat opencvImage;
  //Mat opencvImage;

  static int m_defaultMillisecs;
  bool m_saveCurrent;
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
  QStringList currentFiles;
  int m_numberOfDirs;
  int m_numberOfFiles;
  int m_imageType;
  int m_NumFrame;
  QString m_videoDirectory;
  QString m_fileName;

};


#endif
