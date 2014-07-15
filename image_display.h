#ifndef IMAGE_DISPLAY_H
#define IMAGE_DISPLAY_H

#include "ui_image_display.h"
#include <QImage>
#include "ModulesDialog.h"
#include "parameterDialog.h"
#include "viewform.h"
#include <QVector>
#include <QScrollArea>
#include "productionthread.h"

class MainWindow;

enum ViewTypes {
    _4Views,
    _4ViewsMILES,
    _2ViewsH,
    _2ViewsV
};

typedef enum ViewTypes ViewTypes;

typedef QImage *(MainWindow::*OPERATOR)();

class MainWindow : public QMainWindow
 {
   Q_OBJECT

 public:
   MainWindow(VideoAnalysis *i_va, char *config_file, QMainWindow *parent = 0);
   ~MainWindow();
   void closeEvent(QCloseEvent *event);

   void saveImage(QImage *image, QString toPrint);

   void resizeEvent(QResizeEvent *event);



   static void setGrayScaleTable();
   static QVector<QRgb> *grayScaleTable;
   static void setThermalScaleTable();
   static QVector<QRgb> *thermalScaleTable;

private slots:
   void on_initialButton_clicked();
   void on_nextButton_clicked();

   // slots para hebra
   void on_playButton_clicked();
   void on_pauseButton_clicked();
   void showImages();   // inicia la reproduccion de las imagenes
   void undonePlay();

   void appendToLog(QString);


public:

   ProductionThread *producer;
   //View elements
   QScrollArea *m_viewArea;
   QVector<ViewForm *> m_views;
   ViewTypes m_vtype;
   int m_vwidth;
   int m_vheight;
   int m_virow;
   int m_vicol;
   int m_nactive_views;

   int m_vhorpix;
   int m_vverpix;
   //Nombre del archivo de configuracion
   char *m_view_config_file;
   //View Configuration XML variables
   QDomDocument *xmlViewConfig;
   QDomElement rootViewConfig;

   //Customized View Functions:
   void disableSaveViews();
   void initViews(QFrame *);
   void setupUiViews();


private:
   Ui::MainWindow ui;
   int counter;
   int dialog_id;
   Datapool *m_data;
   VideoAnalysis *m_videoAnalysis;
   ModulesDialog *m_dialog;
   ParameterDialog *m_parameterDialog;
   QString fNumber;

   std::map<int, QImage *(*)()> operators;
   std::map<std::string, std::string> list;

   //todo y cambiarlo en el while...

   // Flag para saber si ya se construyeron todos los modulos
   //del programa por primera vez

    // Flags for controlling proper initialization
    bool parametrized;
    bool initialized;
    bool from_next;

 };

#endif
