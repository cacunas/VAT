#include "AcquisitionModule.h"

#include <errno.h>
#include <iostream>
#include "VideoAnalysis.h"
#include <QTime>
#include <QtGui>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>

int AcquisitionModule::m_defaultMillisecs = 80;

AcquisitionModule::AcquisitionModule(Datapool *i_data):ModuleInterface(i_data),
  m_startFrame(0),
  m_parseDirectory(false),
  firstTime(true),
  m_currentImageIndex(0),
  m_currentDirIndex(0),
  m_numberOfDirs(0),
  m_numberOfFiles(0) {
}

AcquisitionModule::~AcquisitionModule() {}

//Set module configuration parameters
bool AcquisitionModule::setParameters(QDomNode& config) {
    QDomNode n;
    if(config.isNull()) { //Parameter set for module not defined
        m_startFrame = 0;
        m_parseDirectory = false;
        m_videoDirectory = "./YO";
        m_saveCurrent = true;
    } else {
        if( ( n = XmlCommon::getParameterNode("startFrame", config) ).isNull() )
            m_startFrame = 0;
        else
            m_startFrame = XmlCommon::getParameterValue(n).toInt();


        if( ( n = XmlCommon::getParameterNode("subDirectories", config) ).isNull() )
            m_parseDirectory = false;
        else
            m_parseDirectory = (XmlCommon::getParameterValue(n) == "yes") ? true : false;

        if( ( n = XmlCommon::getParameterNode("videoDirectory", config) ).isNull() )
            m_videoDirectory = "./YO";
        else
            m_videoDirectory = XmlCommon::getParameterValue(n);

        if( ( n = XmlCommon::getParameterNode("defaultMillisecs", config) ).isNull() ) {
            m_defaultMillisecs = 80;
            AppendToLog("ReliabilityTracker Warning: 'defaultMillisecs' not defined. Taking Default (80).\n");
        } else {
            if( (m_defaultMillisecs = XmlCommon::getParameterValue(n).toInt()) <= 0) {
                m_defaultMillisecs = 80;
                AppendToLog("ReliabilityTracker Warning: 'defaultMillisecs' must be an integer higher than 0. Taking Default (80).\n");
            }
        }

        if( ( n = XmlCommon::getParameterNode("saveCurrent", config) ).isNull() ) {
            m_saveCurrent = true;
            AppendToLog("ReliabilityTracker Warning: 'saveCurrent' not defined. Taking Default (yes).\n");
        } else {
            m_saveCurrent = XmlCommon::getParameterValue(n) != "yes" ? false : true;
        }
    }

    setDirectory(m_videoDirectory);

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("startFrame", QString::number(m_startFrame), "int");
    addParameter("subDirectories", (m_parseDirectory == true)? "yes" : "no", "bool");
    addParameter("videoDirectory", m_videoDirectory, "QString");
    addParameter("defaultMillisecs", QString::number(m_defaultMillisecs), "int");
    addParameter("saveCurrent", (m_saveCurrent == true)? "yes" : "no", "bool");
    return true;
}

bool AcquisitionModule::updateParameters(){
    parameter *startframe, *subdirs, *videodir, *defmillisecs;
    startframe = getParameter("startFrame");
    subdirs = getParameter("subDirectories");
    videodir = getParameter("videoDirectory");
    defmillisecs = getParameter("defaultMillisecs");

    if( startframe == 0 || subdirs == 0 || videodir == 0 || defmillisecs == 0 ||
           startframe->value.toInt() < 0 || defmillisecs->value.toInt() < 0 ||
           subdirs->value.isEmpty() || subdirs->value.isNull() ||
           videodir->value.isEmpty() || videodir->value.isNull())
       return false;
    m_startFrame = startframe->value.toInt();
    m_parseDirectory = (subdirs->value == "yes")? true : false;
    m_videoDirectory = videodir->value;
    m_defaultMillisecs = defmillisecs->value.toInt();

    return true;
}

//Function executed at each frame
bool AcquisitionModule::run() {
    if(firstTime) {
        firstTime = false;
        m_NumFrame = m_startFrame;

        if (m_parseDirectory)
            openDir();

        if (!goToFirstFrame()) {
            AppendToLog("AcquisitionModule: Start frame does not exist in any given directory, or file corrupted. Aborting computation.");
            return false;
        }
    }

    m_data->frameNumber = this->m_NumFrame;
    QImage *anImage = getNextFrame();

    if (anImage == NULL)
        return false;
    if(m_saveCurrent) {
        if(m_data->currentImage != NULL) {
            if(m_data->previousImage != NULL)
                delete m_data->previousImage;
            m_data->previousImage = m_data->currentImage;
            m_data->currentImage = anImage;
        } else {
            m_data->currentImage = anImage;
            m_data->previousImage = NULL;
        }
    } else { //Just mantain current
        if(m_data->currentImage != NULL)
            delete m_data->currentImage;
        m_data->currentImage = anImage;
    }
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
            AppendToLog("AcquisitionModule: getNextFrame() : End of the sequence reached.");
            return NULL;
        }
    }
        
    if (m_parseDirectory && m_currentDirIndex >= m_numberOfDirs) {
        // We reached the end of the sequences!
        AppendToLog("AcquisitionModule: getNextFrame() : End of the sequence reached.");
        return NULL;
    }

    QImage *tmpImage = NULL;
    bool retValue = readImageOnDisk(&tmpImage);

    if (retValue == false) {
        // The image is corrupted, we attempt to read the following one...
        AppendToLog("AcquisitionModule: getNextFrame() : Error reading an image, Trying with next frame...");
        return getNextFrame();
    } else
        readTimeStamp(m_fileName);

    m_currentImageIndex++;
    m_NumFrame++;
    return tmpImage;
}

bool AcquisitionModule::init() {
    QStringList filters;
    filters << "*.jpg" << "*.jpeg" << "*.ppm" << "*.gif" << "*.bmp";
    if(m_parseDirectory) {
        directory.setFilter(QDir::AllDirs);
        subdirectory.setNameFilters(filters);
    } else {
        directory.setNameFilters(filters);
    }
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
    return m_NumFrame - 1;
}

bool AcquisitionModule::goToFirstFrame() {
    int i;
    QString fileName;
    int currentFrameNum = 0;
    do {
        openFiles();
    
        // Let search the start image in the list of files
        for (i = 0; i < m_numberOfFiles; i++) {
            fileName = currentFiles[i];
            currentFrameNum = fileName.section("/",-1,-1).section(".", -2, -2).toInt();
           
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
    } while(1);
}

void AcquisitionModule::openDir() {
    directories = directory.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    m_numberOfDirs = directories.size();
    m_currentDirIndex = 0;
}

void AcquisitionModule::openFiles() {

    if(m_parseDirectory && m_currentDirIndex < m_numberOfDirs) {
        do {
            QString directoryName = m_seqDir + "/" + directories[m_currentDirIndex];

            subdirectory.setPath(directoryName);
            currentFiles = subdirectory.entryList();
            m_numberOfFiles = currentFiles.size();
            if (m_numberOfFiles <= 0)
                m_currentDirIndex++;
        } while (m_numberOfFiles <= 0);
    } else {
        currentFiles.clear();
        currentFiles = directory.entryList();

        m_numberOfFiles = currentFiles.size();
        AppendToLog("1");
    }
    
    m_currentImageIndex = 0;

}

bool AcquisitionModule::readImageOnDisk(QImage **o_rpOutputImage) {

    if(m_parseDirectory)
        m_fileName = subdirectory.path() + "/"
        + currentFiles[m_currentImageIndex];
    else
        m_fileName = directory.path() + "/" + currentFiles[m_currentImageIndex];
    //std::cout << "ACA:'" << fileName.toStdString() << "'" << std::endl;



    opencvImage= cv::imread(m_fileName.toStdString());

    cv::cvtColor(opencvImage, opencvImage, CV_BGR2RGB);
    QImage *img= new QImage(opencvImage.data, opencvImage.cols, opencvImage.rows, QImage::Format_RGB888);

    *o_rpOutputImage =new QImage(opencvImage.cols, opencvImage.rows, QImage::Format_ARGB32);

    **o_rpOutputImage = img->convertToFormat(QImage::Format_ARGB32);


    delete(img);

    if((*o_rpOutputImage)->isNull()) {
        AppendToLog("ReadImageOnDisk: Impossible to load image (load_image returned error for file " + m_fileName );
        return false;
    }
    return true;
}

void AcquisitionModule::readTimeStamp(QString& fileName) {
    ImageHeader *h = m_data->currentHeader, H;
    if(h == NULL)
        h = m_data->currentHeader = new ImageHeader();

    H.readJpegHeader(fileName);
    /*
    Se cambio el if en donde se hacia comprobacion del header, debido a que en algunas secuencias
    de video el header tiene contenido, pero no tienen el formato de DDMMAAHHMMSS... Es por ello que
    se anadio la condicion de que al menos tuviese un largo de 26 caracteres (siguiendo el formato 
    DDMMAAHHMMSS..). Lo que si, seria mejor anadir una funcion en ImageHeader.cpp que haga esto 
    teniendo un formato unico de header a utilizar.
    */
    if(H.jpeg_header_string != "" && H.jpeg_header_string.length()==26 ) {
    //if(H.jpeg_header_string != "") {
        H.jpegHeaderConvert(h->jpeg_header_string, h->m_camId);
        H.ts_present = true;
        memcpy(h, &H, sizeof(ImageHeader));
    } else {
        h->ts_present = false;
        h->ts_diff = m_defaultMillisecs;
    }

    if(!h->ts_present) { //If timestamp not found, add milliseconds
        if(h->ts.millisecond < 0) //First time
            memset(&(h->ts), 0, sizeof(TimeStamp));
        else
            h->ts += m_defaultMillisecs;
    }
    h->ts.frame_id = fileName.section("/",-1,-1).section(".", -2, -2).toInt();

}

void AcquisitionModule::setDirectory(QString dir) {
   m_seqDir = dir;   
   directory.setPath(dir);
}

void AcquisitionModule::setDirectory() {
    directory.setPath(this->m_videoDirectory);
}


void AcquisitionModule::initialFrame(){
    this->m_NumFrame = this->m_startFrame;
}

