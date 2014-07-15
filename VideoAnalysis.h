#ifndef VIDEOANALYSIS_H
#define VIDEOANALYSIS_H

#ifdef __COMPILE_QT5__
    #include <QtWidgets/QTextEdit>
    #include <QtWidgets/QCheckBox>
#else
    #include<QTextEdit>
    #include<QCheckBox>
#endif
#include<deque>
#include<set>
#include<map>

#include "Datapool.h"
#include "MODULES/ModuleInterface.h"
#include "DRAW/drawInterface.h"
#include "paintView.h"

#define AppendToLog(s) VideoAnalysis::m_data->appendToLog(s)
#define pauseApp() VideoAnalysis::m_data->pauseApp()


class VideoAnalysis: public QObject {
public:
  VideoAnalysis(char *i_config_file, Datapool *d);
  ~VideoAnalysis();

  //Reinicializa variables de los modulo
  void resetModules();
  void resetModules(std::deque<std::string>& sequence);

  //Llama al metodo especifico de lectura de parametros de cada modulo:
  bool setParameters();

  //Llama al metodo especifico de lectura de parametros de la aplicacion:
  bool setApplicationParameters();

  //Opens configuration file and searches for <GeneralApplicationParameters> tag
  //and overrides the default applications parameters defined in config/app_config.xml
  bool setAppParametersOverride();

  //Llama al metodo especifico de inicializacion de cada modulo:
  bool init();

  //Llama al metodo especifico de procesamiento de cada modulo.
  //Este metodo se ejecutara una vez por cada image frame recibido
  //de la secuencia de imagenes.
  bool execute();

  void copyBytes(QImage *im1, QImage *im2);

  //Module management functions

  //Sets initial module sequence
  void setAvailableModules();
  void setInitialModuleSequence();
  void deleteModuleFromSequence(std::deque<ModuleInterface *>::iterator *);
  void insertModuleInSequence(std::deque<ModuleInterface *>::iterator *, std::string name);
  void clearSequence();
  static void appendToLog(const QString& toLog);

  //Available Object Models
  void setAvailableModels();
  static bool isValidModelName(QString name);

  //Pool de datos, que contiene toda la informacion del sistema,
  //utilizada y actualizada por los distintos modulos:
  static Datapool *m_data;

  //Nombre del archivo de configuracion
  char *m_config_file;
  char *m_app_config_file;
  char *m_display_config_file;

  //Nombre del archivo de secuencia de modulos por defecto
  char *m_default_module_seq;

  //Application Parameters
  //TimeStats
  bool m_timeStatsActive;
  bool m_timeOutputToFile;
  QString m_timeOutputFile;
  long m_timeAtFrame;
  int m_timeAtStep;
  bool m_firstTime;
  QFile m_timeFile;

  //Display Parameters

  //Lista construida desde el archivo de configuracion
  std::map<std::string, std::map<std::string, std::string > >* list;

  //paint
  static std::deque<paintView *> paintDeque;

  //Modulos:
  static std::set<std::string> availableModules;
  static std::map<std::string, ModuleType> moduleType;
  static std::map<std::string, ModuleInterface *(*)(Datapool *)> moduleConstructor;
  static std::deque<ModuleInterface *> moduleSequence;
  static std::multimap<ModuleType, std::string, std::less<ModuleType> > modulesByType;

  std::deque<ModuleInterface *>::iterator seq_it;
  std::deque<ModuleInterface *>::iterator seq_it_end;

  //Modelos
  static std::set<QString> availableModels;
  static std::map<QString, ReliabilitySingleModelInterface *(*)()> modelConstructor;

  //Time Stat Functions
  void updateTimeStats(int index, int elapsed_ms);
  void initTimeStats();
  void initTimeStatsForModule(int index, QString name);
  void printTimeStats();

  //Flags de los fake modules
  void setFakeSegmentationFlag(bool option);
  void setFakeTrackingFlag(bool option);
  bool fakeSegmentationFlag;
  bool fakeTrackingFlag;


  static void saveImageToDir(QImage *im, int frame, QString &dir);

  //App Configuration XML variables
  QDomDocument *xmlAppConfig;
  QDomElement rootAppConfig;

  //Display Configuration XML variables
  QDomDocument *xmlDisplayConfig;
  QDomElement rootDisplayConfig;

  //Module Configuration XML variables
  QDomDocument *xmlConfig;
  QDomElement rootConfig;

  QString m_viewFile;
};


#endif // VIDEOANALYSIS_H
