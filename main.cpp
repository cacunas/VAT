// Header for the QApplication class
#ifdef __COMPILE_QT5__
    #include <QtWidgets/QApplication>
#else
    #include <QtGui/QApplication>
#endif

// Header for our dialog
#include "image_display.h"
#include "VideoAnalysis.h"
#include "Datapool.h"



// Remember the ( int, char** ) part as the QApplication needs them
int main( int argc, char **argv ) {

  char *config;

  if(argc >= 2)
     config = argv[1];
  else
     config = strdup("config/default_config.xml");

  // We must always have an application
  QApplication a( argc, argv );

  Datapool *d = new Datapool();

  VideoAnalysis *m_videoAnalysis = new VideoAnalysis(config, d);

  MainWindow *m = new MainWindow(m_videoAnalysis, config);   // We create our dialog

  QObject::connect(d,SIGNAL(logString(QString)),m,SLOT(appendToLog(QString)));
  QObject::connect(d,SIGNAL(pause()),m,SLOT(on_pauseButton_clicked()));


  m->show();                    // Show it...

  return a.exec();              // And run!
} 
