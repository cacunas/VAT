#include "ETISEOAcquisitionModule.h"

#include <errno.h>
#include <iostream>
#include "VideoAnalysis.h"

int ETISEOAcquisitionModule::m_defaultMillisecs = 80;

ETISEOAcquisitionModule::ETISEOAcquisitionModule(Datapool *i_data):ModuleInterface(i_data),
  m_startFrame(0),
  m_parseDirectory(false),
  firstTime(true),
  m_currentImageIndex(0),
  m_currentDirIndex(0),
  m_numberOfDirs(0),
  m_numberOfFiles(0) {}

ETISEOAcquisitionModule::~ETISEOAcquisitionModule() {}

//Set module configuration parameters
bool ETISEOAcquisitionModule::setParameters(QDomNode& config) {
    QDomNode n;
    if(config.isNull()) { //Parameter set for module not defined
        m_startFrame = 0;
        m_parseDirectory = false;
        m_videoDirectory = "./YO";
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
            AppendToLog("ETISEOAcquisitionModule Warning: 'defaultMillisecs' not defined. Taking Default (80).\n");
        } else {
            if( (m_defaultMillisecs = XmlCommon::getParameterValue(n).toInt()) <= 0) {
                m_defaultMillisecs = 80;
                AppendToLog("ETISEOAcquisitionModule Warning: 'defaultMillisecs' must be an integer higher than 0. Taking Default (80).\n");
            }
        }

    }

    setDirectory(m_videoDirectory);

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("startFrame", QString::number(m_startFrame), "int");
    addParameter("subDirectories", (m_parseDirectory)?"true":"false", "bool");
    addParameter("videoDirectory", m_videoDirectory, "QString");
    addParameter("defaultMillisecs", QString::number(m_defaultMillisecs), "int");
    return true;
}

bool ETISEOAcquisitionModule::updateParameters(){
    parameter *sf, *sd, *vd, *dm;
    sf = getParameter("startFrame");
    sd = getParameter("subDirectories");
    vd = getParameter("videoDirectory");
    dm = getParameter("defaultMillisecs");

    if( sf == 0 || sd == 0 || vd == 0 || dm == 0 ||
            sf->value.toInt() < 0 || dm->value.toInt() < 0 ||
            sd->value.isEmpty() || sd->value.isNull() ||
            vd->value.isEmpty() || vd->value.isNull())
        return false;
    m_startFrame = sf->value.toInt();
    m_parseDirectory = (sd->value == "true")? true : false;
    m_videoDirectory = vd->value;
    m_defaultMillisecs = dm->value.toInt();
    return true;
}

//Function executed at each frame
bool ETISEOAcquisitionModule::run() {
    if(firstTime) {
        firstTime = false;
        m_NumFrame = m_startFrame;

        if (m_parseDirectory)
            openDir();

        if (!goToFirstFrame()) {
            AppendToLog("ETISEOAcquisitionModule: Start frame does not exist in any given directory, or file corrupted. Aborting computation.");
            return false;
        }
    }

    m_data->frameNumber = this->m_NumFrame;
    QImage *anImage = getNextFrame();

    if (anImage == NULL)
        return false;
  
    if(m_data->currentImage != NULL)
        delete m_data->currentImage;
    m_data->currentImage = anImage;

    return true;
}

QImage *ETISEOAcquisitionModule::getNextFrame() {

    if (m_currentImageIndex >= m_numberOfFiles) {
        if (m_parseDirectory) {
            // We have to change directory
            m_currentDirIndex++;
            openFiles();
        } else {
            // We reached the end of the sequences!
            AppendToLog("ETISEOAcquisitionModule: getNextFrame() : End of the sequence reached.");
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

bool ETISEOAcquisitionModule::init() {
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


QImage *ETISEOAcquisitionModule::getFrame(int i_numFrame) {

    m_startFrame = i_numFrame;
    
    if (m_parseDirectory)
        openDir();
    
    if (!goToFirstFrame())
        return NULL;
					
    return getNextFrame();
}


int ETISEOAcquisitionModule::getNumFrame() {
    return m_NumFrame - 1;
}

bool ETISEOAcquisitionModule::goToFirstFrame() {
    int i;
    QString fileName, stamp_frame;
    int currentFrameNum = 0;
    do {
        openFiles();
    
        // Let search the start image in the list of files
        for (i = 0; i < m_numberOfFiles; i++) {
            fileName = currentFiles[i];
            currentFrameNum = i;
           
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

void ETISEOAcquisitionModule::openDir() {
    directories = directory.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    m_numberOfDirs = directories.size();
    m_currentDirIndex = 0;
}

void ETISEOAcquisitionModule::setCurrentFiles(int num) {
    currentFiles.resize(num);

    QStringList::const_iterator it, end_it = currentFilesFromDir.constEnd();
    int frame;

    for (it = currentFilesFromDir.constBegin(); it != end_it; ++it) {
        frame = (*it).section("/",-1,-1).section(".", -2, -2).section("-",-1,-1).toInt();
        currentFiles[frame] = *it;
    }
}

void ETISEOAcquisitionModule::openFiles() {
    if(m_parseDirectory && m_currentDirIndex < m_numberOfDirs) {
        do {
            QString directoryName = m_seqDir + "/" + directories[m_currentDirIndex];
            subdirectory.setPath(directoryName);
            currentFilesFromDir = subdirectory.entryList();
            m_numberOfFiles = currentFilesFromDir.size();
            setCurrentFiles(m_numberOfFiles);
            if (m_numberOfFiles <= 0)
                m_currentDirIndex++;
        } while (m_numberOfFiles <= 0);
    } else {
        currentFilesFromDir = directory.entryList();
        m_numberOfFiles = currentFilesFromDir.size();
        setCurrentFiles(m_numberOfFiles);
    }
    
    m_currentImageIndex = 0;

}

bool ETISEOAcquisitionModule::readImageOnDisk(QImage **o_rpOutputImage) {

    if(m_parseDirectory)
        m_fileName = subdirectory.path() + "/"
        + currentFiles[m_currentImageIndex];
    else
        m_fileName = directory.path() + "/" + currentFiles[m_currentImageIndex];
    //std::cout << "ACA:'" << fileName.toStdString() << "'" << std::endl;
    *o_rpOutputImage = new QImage(m_fileName);

    if((*o_rpOutputImage)->isNull()) {
        AppendToLog("ReadImageOnDisk: Impossible to load image (load_image returned error for file " + m_fileName );
        return false;
    }
    return true;
}

void ETISEOAcquisitionModule::readETISEOHeader(ImageHeader &H, QString &time) {
    QString buffer;
    int pos = 0;
    // Year
    H.ts.year = 0;
    // Month
    H.ts.month = 1;
    // Day
    H.ts.day = 1;
    // Hour
    buffer = time.mid(pos, 2);
    pos += 3;
    H.ts.hour = buffer.toInt();
    // Minute
    buffer = time.mid(pos, 2);
    pos += 3;
    H.ts.minute = buffer.toInt();
    // Second
    buffer = time.mid(pos, 2);
    pos += 3;
    H.ts.second = buffer.toInt();
    // Millisecond
    buffer = time.mid(pos, 3);
    H.ts.millisecond = buffer.toInt();
}


void ETISEOAcquisitionModule::readTimeStamp(QString& fileName) {

    QString aux = fileName.section("/",-1,-1).section(".", -2, -2);
    int frame = aux.section("-",-1,-1).toInt();
    QString time = aux.section("-",-2,-2);

    ImageHeader *h = m_data->currentHeader, H;
    if(h == NULL)
        h = m_data->currentHeader = new ImageHeader();

    //Function for ETISEO time in filename
    H.jpeg_header_string = time;
    if(H.jpeg_header_string != "") {
        readETISEOHeader(H, time);
        H.ts.frame_id = frame;
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

}

void ETISEOAcquisitionModule::setDirectory(QString dir) {
   m_seqDir = dir;   
   directory.setPath(dir);
}

void ETISEOAcquisitionModule::setDirectory() {
    directory.setPath(this->m_videoDirectory);
}

void ETISEOAcquisitionModule::initialFrame(){
    this->m_NumFrame = this->m_startFrame;
}

