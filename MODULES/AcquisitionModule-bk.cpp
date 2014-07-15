#include <AcquisitionModule.h>

#include <dirent.h>
#include <errno.h>
#include <iostream>
#include <QDir>

AcquisitionModule::AcquisitionModule(Datapool *i_data): 
  m_data(i_data),
  m_startFrame(0),
  m_parseDirectory(false),
  firstTime(true),
  m_currentImageIndex(0),
  m_currentDirIndex(0),
  m_ppDirectoryList(NULL),
  m_ppImageList(NULL),
  m_numberOfDirs(0),
  m_numberOfFiles(0) { /*data = &i_data;*/ }

AcquisitionModule::~AcquisitionModule() {
  if(m_ppDirectoryList) {
    int i;
    for(i=0; i<m_numberOfDirs; i++)
      delete[] m_ppDirectoryList[i];
    delete[] m_ppDirectoryList;
  }

  if(m_ppImageList) {
    int i;
    for(i=0; i<m_numberOfFiles; i++)
      delete[] m_ppImageList[i];
    delete[] m_ppImageList;
  }

}

//Set module configuration parameters
bool AcquisitionModule::setParameters(std::map<std::string, std::map<std::string, std::string> >* list) {
  std::map<std::string, std::map<std::string, std::string> >::iterator lend = list->end(), found;
  std::map<std::string, std::string>::iterator lend2, found2;

  if( (found=list->find("AcquisitionModule")) != lend) {
    lend2 = (*found).second.end();

    if( (found2=(*found).second.find("startFrame")) != lend2)
        m_startFrame = atoi((*found2).second.c_str());
    else {
        m_startFrame = 0;
    }

    if( (found2=(*found).second.find("subDirectories")) != lend2)
        m_parseDirectory = ((*found2).second == "yes") ? true : false;
    else {
        m_parseDirectory = false;
    }
  } else {
     m_startFrame = 0;
     m_parseDirectory = false;
  }

  return true;
}

//Function executed at each frame
bool AcquisitionModule::run() {
    if(firstTime) {
        firstTime = false;
        m_NumFrame = m_startFrame;

        if (m_parseDirectory)
            openDir();

        if (!goToFirstFrame())
            std::cout << "Start frame does not exist in any given directory, or file corrupted" << std::endl;
    }

    QImage *anImage = getNextFrame();

    if (anImage == NULL)
        return false;
  
    if(m_data->currentImage != NULL)
        delete m_data->currentImage;
    m_data->currentImage = anImage;

    return true;
}

QImage *AcquisitionModule::getNextFrame() {

  if (m_currentImageIndex >= m_numberOfFiles) {
    if (m_parseDirectory) {
      // We have to change directory
      m_currentDirIndex++;
      openFiles();
    } else {
      // We reached the end of the sequences!
      std::cout << "CDiskInput::getNextFrame() : End of the sequence reached.\n";
      return NULL;
    }
  }
        
  if (m_parseDirectory && m_currentDirIndex >= m_numberOfDirs) {
    // We reached the end of the sequences!
    std::cout << "CDiskInput::getNextFrame() : End of the sequence reached.\n";
    return NULL;
  }
  
  int retValue;
  QImage *tmpImage = NULL;
  retValue = readImageOnDisk(&tmpImage);

  if (retValue != 1) {
    // The image is corrupted, we attempt to read the following one...
    std::cout << "CDiskInput::getNextFrame() : ERROR : error reading an image, skipping...\n";
    return getNextFrame();
  } else 
    readTimeStamp(tmpImage);

  m_currentImageIndex++;
  m_NumFrame++;
  return tmpImage;
}

bool AcquisitionModule::init() {
  return true;
}


QImage *AcquisitionModule::getFrame(int i_numFrame) {

  m_startFrame = i_numFrame;
    
  if (m_parseDirectory)
    openDir();
    
  if (!goToFirstFrame())
    return NULL;
					
  return getNextFrame();
}


int AcquisitionModule::getNumFrame() { 
  return m_NumFrame-1; 
}

bool AcquisitionModule::goToFirstFrame() {
  int i;
  std::string fileName;
  int currentFrameNum = 0;
  do {
    openFiles();
    
    // Let search the start image in the list of files
    for (i = 0; i < m_numberOfFiles; i++) {
      fileName = m_ppImageList[i]->d_name;
      std::string::size_type pos = fileName.find_first_of(".");
      std::string fileNameCopy = fileName;
      currentFrameNum = atoi(fileName.erase(pos).c_str());
           
      if (currentFrameNum < m_startFrame)
	continue;
      else if (currentFrameNum > m_startFrame)
	return false;
      else {
	// We found it
	m_currentImageIndex = i;
	return true;
      }
    }           
            
    // No more files in this directory, change it or fail
    if (m_parseDirectory) {
      // Have we got still some directories ?
      if (++m_currentDirIndex >= m_numberOfDirs)
	return false;
    } else
      // We parsed all the files of the unique dir... 
      return false;
  }
  while (1);
}

void AcquisitionModule::openDir() {
  int i;
  if (m_ppDirectoryList) {
    for (i = 0; i < m_numberOfDirs; i++) {
      delete[] m_ppDirectoryList[i];
      m_ppDirectoryList[i] = NULL;
    }
    delete[] m_ppDirectoryList;
    m_ppDirectoryList = NULL;
  }
  
  m_numberOfDirs = scandir(m_seqDir.c_str(), &m_ppDirectoryList, AcquisitionModule::chooseDirs, alphasort);
  m_currentDirIndex = 0;
}

void AcquisitionModule::openFiles() {
  int i;
  if(m_ppImageList) {
    for(i = 0; i < m_numberOfFiles; i++) {
      delete[] m_ppImageList[i];
      m_ppImageList[i] = NULL;
    }
    delete[] m_ppImageList;
    m_ppImageList = NULL;
  }

  if(m_parseDirectory && m_currentDirIndex < m_numberOfDirs) {
    do { 
      std::string directoryName = m_seqDir + "/" + m_ppDirectoryList[m_currentDirIndex]->d_name;
      m_numberOfFiles = scandir(directoryName.c_str(), &m_ppImageList, AcquisitionModule::chooseJpgFiles, alphasort);
      if (m_numberOfFiles <= 0)
	m_currentDirIndex++;
    } while (m_numberOfFiles <= 0);
  } else m_numberOfFiles = scandir(m_seqDir.c_str(), &m_ppImageList, AcquisitionModule::chooseJpgFiles, alphasort);
    
    m_currentImageIndex = 0;

     QDir dir(m_seqDir.c_str());
     dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
     dir.setSorting(QDir::Size | QDir::Reversed);

     QFileInfoList list = dir.entryInfoList();
     std::cout << "     Bytes Filename" << std::endl;
     for (int i = 0; i < list.size(); ++i) {
         QFileInfo fileInfo = list.at(i);
         std::cout << qPrintable(QString("%1 %2").arg(fileInfo.size(), 10)
                                                 .arg(fileInfo.fileName()));
         std::cout << std::endl;
     }

}


int AcquisitionModule::chooseDirs(const struct dirent *pDir) {

  std::string name(pDir->d_name);

  if(name == "." || name == ".." )
    return 0;
  else 
    return 1;
}

int AcquisitionModule::chooseJpgFiles(const struct dirent *pDir) {

    int nameLength = strlen(pDir->d_name);

    return ( strcmp(&(pDir->d_name[nameLength-strlen(".jpg")]), ".jpg")==0 ||
             strcmp(&(pDir->d_name[nameLength-strlen(".JPG")]), ".JPG")==0 ||
             strcmp(&(pDir->d_name[nameLength-strlen(".jpeg")]), ".jpeg")==0 ||
             strcmp(&(pDir->d_name[nameLength-strlen(".JPEG")]), ".JPEG")==0 );
}

int AcquisitionModule::readImageOnDisk(QImage **o_rpOutputImage) {
  std::string fileName;
  int retValue;

  if(m_parseDirectory)
    fileName = m_seqDir 
      + "/" + m_ppDirectoryList[m_currentDirIndex]->d_name + "/" 
      + m_ppImageList[m_currentImageIndex]->d_name;
  else
    fileName = m_seqDir + "/" + m_ppImageList[m_currentImageIndex]->d_name;
  
  QString filename = fileName.c_str();
  //  std::cout << "ACA:'" << filename.toStdString() << "'" << std::endl;
  *o_rpOutputImage = new QImage(filename);

  retValue = 1;

  if(retValue != 1)
    std::cout << "ReadImageOnDisk - impossible to load image (load_image returned error " 
	      << retValue << " for file " << fileName << std::endl;

  return retValue;
}

void AcquisitionModule::readTimeStamp(QImage* i_pImage) {
  
  //char *buffer_time_stamp  = NULL;
  //  buffer_time_stamp = IMAGE_JPEG_APP0(i_pImage);

  //  if(buffer_time_stamp != NULL) {
  //    if(m_pTimeStamp)
  //      free(m_pTimeStamp);
  //    m_pTimeStamp = jpeg_header_convert(buffer_time_stamp, NULL);
  //  }
}

void AcquisitionModule::setDirectory(std::string dir) {
   m_seqDir = dir;
}



