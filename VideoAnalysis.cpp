#include "VideoAnalysis.h"
#include "src/ReliabilityTracker.h"
#include "src/IncrementalEventLearning.h"
#include "src/LearningConcept.h"
#include "src/LearningContextualisedObject.h"
#include <fstream>
#include "src/MathFunctions.h"
#include "image_display.h"
#include <iostream>
#include <string>
#include <QPainter>
#include <QString>
#include <QMutex>
#include <QTime>
#include <QTextStream>
#include <QRgb>
#include <map>

#include "src/LearningContext.h"
#include "src/HierarchyTree.h"

QMutex outmutex;
QMutex paramMutex;

//Get module headers and handlers
#include "modules.h"

Datapool *VideoAnalysis::m_data = NULL;
std::set<std::string> VideoAnalysis::availableModules;
std::map<std::string, ModuleType> VideoAnalysis::moduleType;
std::deque<ModuleInterface *> VideoAnalysis::moduleSequence;
std::map<std::string, ModuleInterface *(*)(Datapool *)> VideoAnalysis::moduleConstructor;
std::multimap<ModuleType, std::string, std::less<ModuleType> > VideoAnalysis::modulesByType;

std::set<QString> VideoAnalysis::availableModels;
std::map<QString, ReliabilitySingleModelInterface *(*)()> VideoAnalysis::modelConstructor;


//Get draw headers and handlers
//#include "draw.h"
//std::deque<drawInterface *> VideoAnalysis::drawSequence;
//std::map<std::string, drawInterface *(*)(Datapool *)> VideoAnalysis::drawConstructor;


std::deque<paintView *> VideoAnalysis::paintDeque;

VideoAnalysis::VideoAnalysis(char *i_config_file, Datapool *d) {
    m_config_file = i_config_file;
    m_default_module_seq = strdup("config/default_module_sequence.txt");
    m_app_config_file = strdup("config/app_config.xml");
    m_display_config_file = strdup("config/display_config.xml");
    m_data = d;
    list = new std::map<std::string, std::map<std::string, std::string> >();

    fakeTrackingFlag = false;
    fakeSegmentationFlag = false;

    //Process app configuration file, first
    xmlConfig = NULL;
    xmlAppConfig = NULL;
    xmlDisplayConfig = NULL;

    setApplicationParameters();

    setAppParametersOverride();

    moduleConstructor.clear();

    setAvailableModules();

    setAvailableModels();

    setInitialModuleSequence();


    //test
    int i;
    std::set<std::string>::iterator iter, it_end = availableModules.end();
    for(i=0, iter = availableModules.begin();iter != it_end;iter++, i++)
        std::cout << i << ":\t" << *iter << std::endl;

    //GLOBAL DATAPOOL INFORMATION
    m_data->m_config_file = QString(m_config_file);

}

VideoAnalysis::~VideoAnalysis() {
    if(xmlConfig != NULL)
        delete xmlConfig;
    if(xmlAppConfig != NULL)
        delete xmlAppConfig;
    if(moduleSequence.size() > 0) {
        std::deque<ModuleInterface *>::iterator it, it_end = moduleSequence.end();
        for(it = moduleSequence.begin(); it != it_end; it++)
            delete *it;
        moduleSequence.clear();
    }

    if(paintDeque.size() > 0) {
        std::deque<paintView *>::iterator it, it_end = paintDeque.end();
        for(it = paintDeque.begin(); it != it_end; it++)
            delete *it;
        paintDeque.clear();
    }

}

bool VideoAnalysis::isValidModelName(QString name) {
    return availableModels.count(name) > 0;
}


void VideoAnalysis::setInitialModuleSequence() {

    if(moduleSequence.size() > 0) {
        std::deque<ModuleInterface *>::iterator it, it_end = moduleSequence.end();
        for(it = moduleSequence.begin(); it != it_end; it++)
            delete *it;
        moduleSequence.clear();
    }

    std::ifstream ifs(m_default_module_seq);
    std::string s;
    int i;
    size_t pos;
    ModuleInterface *m;
    while(getline(ifs, s)) {
    //std::cout << "'" << s << "'" << std::endl;
        if( (pos=s.find_first_not_of(" \t\r\n")) != std::string::npos ) {
            s = s.substr(pos);
            //std::cout << "Dentro: " << s << std::endl;
            i = s.size()-1;
            if(s[i] == '\n' || s[i] == '\r')
                s = s.substr(0, i);
            if(availableModules.count(s) > 0) {
                m = moduleConstructor[s](m_data);
                moduleSequence.push_back(m);
            }
        }
    }

    seq_it_end = moduleSequence.end();
}


//Reinicializa variables de los modulo
void VideoAnalysis::resetModules(std::deque<std::string>& sequence) {

    m_data->clear();

    std::deque<ModuleInterface *>::iterator mit, mit_end = moduleSequence.end();
    for(mit = moduleSequence.begin(); mit != mit_end; mit++)
        delete *mit;

    moduleSequence.clear();

    m_data->init();

    std::deque<std::string>::iterator it, it_end = sequence.end();
    for(it = sequence.begin(); it != it_end; it++)
        moduleSequence.push_back(moduleConstructor[*it](m_data));
    seq_it_end = moduleSequence.end();

}

//Reinicializa variables de los modulo
void VideoAnalysis::resetModules() {

    m_data->clear();

    std::deque<std::string> strModuleSequence;
    std::deque<ModuleInterface *>::iterator mit, mit_end = moduleSequence.end();
    for(mit = moduleSequence.begin(); mit != mit_end; mit++) {
        strModuleSequence.push_back((*mit)->name);
        delete *mit;
    }

    moduleSequence.clear();

    m_data->init();

    std::deque<std::string>::iterator it, it_end = strModuleSequence.end();
    for(it = strModuleSequence.begin(); it != it_end; it++)
        moduleSequence.push_back(moduleConstructor[*it](m_data));
    seq_it_end = moduleSequence.end();
}

bool VideoAnalysis::setApplicationParameters() {

    //1. Init xml reading process
    if(xmlAppConfig != NULL)
        delete xmlAppConfig;
    xmlAppConfig = new QDomDocument( "APP_PARAMETERS" );
    QFile file(m_app_config_file);
    QString q_config_file(m_app_config_file);
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("Error opening application configuration file '" + q_config_file + "' for reading. Execution will be aborted.");
        std::cerr << "Error opening application configuration file '" <<
                     q_config_file.toStdString() + "' for reading. Execution will be aborted." << std::endl;
        return false;
    }

    QString error;
    int line, column;
    if( !xmlAppConfig->setContent( &file, &error, &line, &column ) ) {

        AppendToLog(   "Error opening application configuration file '" + q_config_file + "'. XML content could not be extracted. Execution will be aborted.\n"
                                    + "Error on line " + QString::number(line) + " and column " + QString::number(column) + ": " + error);
        std::cerr << "Error opening application configuration file '"
                  << q_config_file.toStdString() << "'. XML content could not be extracted. Execution will be aborted.\n"
                  << "Error in line " << line << " and column " << column << + ": " + error.toStdString() << std::endl;

        file.close();
        return false;
    }
    file.close();

    rootAppConfig = xmlAppConfig->documentElement();
    if( rootAppConfig.tagName() != "APP_PARAMETERS" ) {
        AppendToLog("Error reading XML information. XML content in the configuration file '" + q_config_file + "' does not correspond to application parameters description. Execution will be aborted.");
        return false;
    }

    //2. Process parameters
    QDomNode n, m, o;

    if( ( n = XmlCommon::getParameterNode("ViewConfigFile", rootAppConfig) ).isNull() ) {
        m_viewFile = "config/view_config.xml";
    } else {
        m_viewFile = XmlCommon::getParameterValue(n);
    }

    if( ( n = XmlCommon::getParameterNode("ActivateTimeStats", rootAppConfig) ).isNull() )
        m_timeStatsActive = false;
    else
        m_timeStatsActive = (XmlCommon::getParameterValue(n) == "yes") ? true : false;

    if(m_timeStatsActive) {
        AppendToLog("Time Stats activated...");
        if( ( m = XmlCommon::getParameterNode("OutputToFile", n) ).isNull() ) {
            m_timeOutputToFile = false;
            AppendToLog("Time output to standard output");
        } else {
            m_timeOutputToFile = (XmlCommon::getParameterValue(m) == "yes") ? true : false;
            if(m_timeOutputToFile == false) {
                AppendToLog("Time output to standard output");
            } else {
                QDate d;
                QTime t;
                if( ( o = XmlCommon::getParameterNode("OutputFileNamePrefix", m) ).isNull() ) {
                    m_timeOutputFile = "VAT-Time-" + d.currentDate().toString("dd.MM.yyyy")
                                     + "-" + t.currentTime().toString("hh.mm.ss") + ".txt";
                } else {
                    QString prefix = XmlCommon::getParameterValue(o);
                    if(prefix == "") prefix = "VAT-Time-";
                    m_timeOutputFile = prefix + d.currentDate().toString("dd.MM.yyyy")
                                     + "-" + t.currentTime().toString("hh.mm.ss") + ".txt";
                }
                AppendToLog("Time output to file: " + m_timeOutputFile);
            }
        }

        if( ( m = XmlCommon::getParameterNode("AtFrame", n) ).isNull() ) {
            m_timeAtFrame = -1;
        } else {
            m_timeAtFrame = XmlCommon::getParameterValue(m).toLong();
        }

        if( ( m = XmlCommon::getParameterNode("AtStep", n) ).isNull() ) {
            m_timeAtStep = 1;
        } else {
            m_timeAtStep = XmlCommon::getParameterValue(m).toInt();
            if(m_timeAtStep <= 0) m_timeAtStep = 1;
        }

        AppendToLog(QString("Time registered each ") + m_timeAtStep + QString(" frames. "));
        AppendToLog(QString("Also registered at frame ") + m_timeAtFrame + QString(". "));
    }


    //3. Clean xml variable
    delete xmlAppConfig;
    xmlAppConfig = NULL;

    return true;
}


//Llama al metodo especifico de lectura de parametros de cada modulo:
bool VideoAnalysis::setAppParametersOverride(){

    //Process module parameters file
    if(xmlConfig != NULL)
        delete xmlConfig;

    xmlConfig = new QDomDocument( "MODULE_PARAMETERS" );

    QFile file( m_config_file );
    QString q_config_file(m_config_file);
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("Error opening module configuration file '" + q_config_file + "' for reading. Execution will be aborted.");
        return false;
    }

    QString error;
    int line, column;
    if( !xmlConfig->setContent( &file, &error, &line, &column ) ) {

        AppendToLog(   "Error opening module configuration file '" + q_config_file + "'. XML content could not be extracted. Execution will be aborted.\n"
                                    + "Error on line " + QString::number(line) + " and column " + QString::number(column) + ": " + error);
        file.close();
        return false;
    }
    file.close();

    rootConfig = xmlConfig->documentElement();
    if( rootConfig.tagName() != "MODULE_PARAMETERS" ) {
        AppendToLog("Error reading XML information. XML content in the configuration file '" + q_config_file + "' does not correspond to a module parameters description. Execution will be aborted.");
        return false;
    }

    QDomNode ovAppConfig = XmlCommon::getParameterNode("GeneralApplicationParameters", rootConfig);

    if(ovAppConfig.isNull())
        return true;

    //2. Process parameters
    QDomNode n, m, o;


    if( ! ( n = XmlCommon::getParameterNode("ModuleSequenceFile", ovAppConfig) ).isNull() ) {
        if (m_default_module_seq != NULL) delete m_default_module_seq;
        m_default_module_seq = strdup(XmlCommon::getParameterValue(n).toStdString().c_str());
    }

    if( ! ( n = XmlCommon::getParameterNode("ViewConfigFile", ovAppConfig) ).isNull() )
        m_viewFile = XmlCommon::getParameterValue(n);

    if( ! ( n = XmlCommon::getParameterNode("ActivateTimeStats", ovAppConfig) ).isNull() )
        m_timeStatsActive = (XmlCommon::getParameterValue(n) == "yes") ? true : false;

    if(m_timeStatsActive) {
        AppendToLog("Time Stats activated...");
        if( ! ( m = XmlCommon::getParameterNode("OutputToFile", n) ).isNull() ) {
            m_timeOutputToFile = (XmlCommon::getParameterValue(m) == "yes") ? true : false;
            if(m_timeOutputToFile == false) {
                AppendToLog("Time output to standard output");
            } else {
                QDate d;
                QTime t;
                if( ( o = XmlCommon::getParameterNode("OutputFileNamePrefix", m) ).isNull() ) {
                    m_timeOutputFile = "VAT-Time-" + d.currentDate().toString("dd.MM.yyyy")
                                     + "-" + t.currentTime().toString("hh.mm.ss") + ".txt";
                } else {
                    QString prefix = XmlCommon::getParameterValue(o);
                    if(prefix == "") prefix = "VAT-Time-";
                    m_timeOutputFile = prefix + d.currentDate().toString("dd.MM.yyyy")
                                     + "-" + t.currentTime().toString("hh.mm.ss") + ".txt";
                }
                AppendToLog("Time output to file: " + m_timeOutputFile);
            }
        }

        if( ! ( m = XmlCommon::getParameterNode("AtFrame", n) ).isNull() )
            m_timeAtFrame = XmlCommon::getParameterValue(m).toLong();

        if( ! ( m = XmlCommon::getParameterNode("AtStep", n) ).isNull() ) {
            m_timeAtStep = XmlCommon::getParameterValue(m).toInt();
            if(m_timeAtStep <= 0) m_timeAtStep = 1;
        }

        AppendToLog(QString("Time registered each ") + m_timeAtStep + QString(" frames. "));
        AppendToLog(QString("Also registered at frame ") + m_timeAtFrame + QString(". "));
    }


    return true;
}


//Llama al metodo especifico de lectura de parametros de cada modulo:
bool VideoAnalysis::setParameters(){

    //Process module parameters file
    if(xmlConfig != NULL)
        delete xmlConfig;

    xmlConfig = new QDomDocument( "MODULE_PARAMETERS" );

    QFile file( m_config_file );
    QString q_config_file(m_config_file);
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("Error opening module configuration file '" + q_config_file + "' for reading. Execution will be aborted.");
        return false;
    }

    QString error;
    int line, column;
    if( !xmlConfig->setContent( &file, &error, &line, &column ) ) {

        AppendToLog(   "Error opening module configuration file '" + q_config_file + "'. XML content could not be extracted. Execution will be aborted.\n"
                                    + "Error on line " + QString::number(line) + " and column " + QString::number(column) + ": " + error);
        file.close();
        return false;
    }
    file.close();

    rootConfig = xmlConfig->documentElement();
    if( rootConfig.tagName() != "MODULE_PARAMETERS" ) {
        AppendToLog("Error reading XML information. XML content in the configuration file '" + q_config_file + "' does not correspond to a module parameters description. Execution will be aborted.");
        return false;
    }

    //std::string moduleName;
    QDomNode currentModule;

    for(seq_it = moduleSequence.begin(); seq_it != seq_it_end; seq_it++) {
        currentModule = rootConfig.elementsByTagName((*seq_it)->name.c_str()).item(0);
        if( ! (*seq_it)->setParameters(currentModule) ) {
            delete xmlConfig;
            xmlConfig = NULL;
            return false;
        }
    }

    delete xmlConfig;
    xmlConfig = NULL;
    return true;
}

//Llama al metodo especifico de inicializacion de cada modulo:
bool VideoAnalysis::init() {
    //Clear lists:
    int i;
    if(m_timeStatsActive)
        initTimeStats();

    for(i=0, seq_it = moduleSequence.begin(); seq_it != seq_it_end; seq_it++, i++) {
        if( ! (*seq_it)->init() )
            return false;
        //Initialize program modules m_data
        if(m_timeStatsActive)
            initTimeStatsForModule(i,QString((*seq_it)->name.c_str()));
    }



    std::deque<paintView *>::iterator it, it_end = paintDeque.end();
    for(it = paintDeque.begin(); it != it_end; it++)
        if(!(*it)->init())
            return false;

  return true;

  //double *pm = m_data->sceneModel->p_matrix;
    //std::cout << pm[0] << "\t" << pm[1] << "\t" << pm[2] << "\t"  << pm[3]  << std::endl;
    //std::cout << pm[4] << "\t" << pm[5] << "\t" << pm[6] << "\t"  << pm[7]  << std::endl;
    //std::cout << pm[8] << "\t" << pm[9] << "\t" << pm[10] << "\t" << pm[11] << std::endl;

}

void VideoAnalysis::updateTimeStats(int index, int elapsed_ms) {
    int Ncurrent = m_data->processedFrames;
    double mean = m_data->moduleSequenceMeanTime[index];
    double var = m_data->moduleSequenceVarTime[index];
    m_data->moduleSequenceMSTime[index] += elapsed_ms;
    m_data->totalTime += elapsed_ms;
    m_data->moduleSequenceMeanTime[index] = mean = MathFunctions::incrementalMean(mean, elapsed_ms, Ncurrent);
    m_data->moduleSequenceVarTime[index] = MathFunctions::incrementalVariance(mean, var, elapsed_ms, Ncurrent);
    if(elapsed_ms < m_data->moduleSequenceMinTime[index])
        m_data->moduleSequenceMinTime[index] = elapsed_ms;
    if(elapsed_ms > m_data->moduleSequenceMaxTime[index])
        m_data->moduleSequenceMaxTime[index] = elapsed_ms;
}

void VideoAnalysis::printTimeStats() {
    if(m_data->processedFrames % m_timeAtStep == 0 || m_data->frameNumber == m_timeAtFrame) {
        if(m_timeOutputToFile) {
            if (!m_timeFile.open(QIODevice::Append | QIODevice::Text)) {
                AppendToLog("VideoAnalysis Error: Unable to open file '"+ m_timeOutputFile +"' for append.");
                return;
            }
        } else
            m_timeFile.open(stdout,QIODevice::WriteOnly);
        QTextStream timeStream(&m_timeFile);
        int i, size = m_data->moduleSequenceNames.size();
        timeStream << "At frame:" << m_data->frameNumber
                   << "(" << m_data->processedFrames << "):\t\t"
                   << m_data->totalTime / 1000.0 << "[secs]" << endl;
        timeStream << "Module\t\tmean\tsigma\tmin\tmax\ttotal [secs]" << endl;
        for(i=0; i<size; i++) {
            QString s = m_data->moduleSequenceNames[i];
            s.truncate(12);
            timeStream << s << "\t";
            timeStream << QString::number(m_data->moduleSequenceMeanTime[i]/1000.0,'f',4) << "\t";
            timeStream << QString::number(sqrt(m_data->moduleSequenceVarTime[i])/1000.0,'f',4) << "\t";
            timeStream << QString::number(m_data->moduleSequenceMinTime[i]/1000.0,'f',4) << "\t";
            timeStream << QString::number(m_data->moduleSequenceMaxTime[i]/1000.0,'f',4) << "\t";
            timeStream << QString::number(m_data->moduleSequenceMSTime[i]/1000.0,'f',4) << endl;
        }
    }

    if(m_timeOutputToFile)
        m_timeFile.close();
}

void VideoAnalysis::initTimeStats() {
    unsigned int size = moduleSequence.size();
    m_data->moduleSequenceNames.clear();
    m_data->moduleSequenceNames.resize(size);
    m_data->moduleSequenceMSTime.clear();
    m_data->moduleSequenceMSTime.resize(size);
    m_data->moduleSequenceMeanTime.clear();
    m_data->moduleSequenceMeanTime.resize(size);
    m_data->moduleSequenceVarTime.clear();
    m_data->moduleSequenceVarTime.resize(size);
    m_data->moduleSequenceMinTime.clear();
    m_data->moduleSequenceMinTime.resize(size);
    m_data->moduleSequenceMaxTime.clear();
    m_data->moduleSequenceMaxTime.resize(size);
    m_data->totalTime = 0.0;
    m_data->processedFrames = 0;
    if(m_timeOutputToFile) {
        m_timeFile.setFileName(m_timeOutputFile);
        //Erasing file and checking write permissions
        if (!m_timeFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            AppendToLog("VideoAnalysis Error: Unable to open file '" + m_timeOutputFile +"' for writing.");
        } else
            m_timeFile.close();
    }

}

void VideoAnalysis::initTimeStatsForModule(int index, QString name) {
    m_data->moduleSequenceNames[index] = name;
    m_data->moduleSequenceMSTime[index] = 0.0;
    m_data->moduleSequenceMeanTime[index] = 0.0;
    m_data->moduleSequenceVarTime[index] = 0.0;
    m_data->moduleSequenceMinTime[index] = INT_MAX;
    m_data->moduleSequenceMaxTime[index] = 0;
}


//Llama al metodo especifico de procesamiento de cada modulo.
//Este metodo se ejecutara una vez por cada image frame recibido
//de la secuencia de imagenes.
bool VideoAnalysis::execute() {
    int i, elapsed;
    bool response;
    //Count of processed frames for Time Stats
    m_data->processedFrames++;

	//Should run modules unless one of them tells it to stop
	m_data->runModules = true;

	for(i=0, seq_it = moduleSequence.begin(); seq_it != seq_it_end && m_data->runModules; i++, seq_it++) {
        QTime t;
        t.start();
        //Bloquea para que no se modifiquen los parametros de los modulos mientras se esta ejecutando.
        paramMutex.lock();
            response = (*seq_it)->run();
        paramMutex.unlock();
        elapsed = t.elapsed();
        if(m_timeStatsActive)
            updateTimeStats(i, elapsed);

        if(!response)
            return false;
        std::deque<paintView *>::iterator it, it_end = paintDeque.end();

        for(it = paintDeque.begin(); it != it_end; it++)
            if((*it)->getDisplayPaint())
                //se verifica si el momento de dibujado es el indicado para pintar cada capa.
                if((*seq_it)->name == ((*it)->getAfterModule()).toStdString()) {
                    if(!(*it)->draw())
                        return false;
                    else if((*it)->saveImage)
                        saveImageToDir((*it)->getImage(), (*it)->fnumber++, (*it)->saveDir);
                }
    }


    if(m_timeStatsActive)
        printTimeStats();

    return true;

}

void VideoAnalysis::copyBytes(QImage *im1, QImage *im2) {
    memcpy(im1->bits(),im2->bits(), im1->height()*im1->bytesPerLine());
}


void VideoAnalysis::setFakeSegmentationFlag(bool option){
    this->fakeSegmentationFlag = option;
}

void VideoAnalysis::setFakeTrackingFlag(bool option){
    this->fakeTrackingFlag = option;
}

void VideoAnalysis::saveImageToDir(QImage *im, int frame, QString &dir){
    QDir d("./");
    if(!d.exists(dir)) {
        d.mkdir(dir);
    }
    QString fileName, num = QString::number(frame);
    int diff = 6-num.size();
    if(diff > 0)
        fileName = "./" + dir + "/" + QString("0").repeated(diff) + QString::number(frame) + ".jpg";
    else
        fileName = "./" + dir + "/" + QString::number(frame) + ".jpg";
    im->save(fileName);
}


