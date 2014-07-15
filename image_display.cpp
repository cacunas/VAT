#include <iostream>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QMutex>
#include <QTime>
#include <QWaitCondition>
#include <QThread>
#include <QCloseEvent>
#include <QVector>

#include "image_display.h"
#include "paintView.h"

extern QMutex datapoolmutex;

QVector<QRgb> *MainWindow::grayScaleTable = NULL;
QVector<QRgb> *MainWindow::thermalScaleTable = NULL;

MainWindow::MainWindow(VideoAnalysis *i_va, char *config_file, QMainWindow *parent):QMainWindow(parent),
dialog_id(0) {
    m_view_config_file = strdup(i_va->m_viewFile.toStdString().c_str());

    m_videoAnalysis = i_va;
    m_data = i_va->m_data;

    parametrized = false;
    initialized = false;

    qRegisterMetaType<QTextCursor>("QTextCursor");

    xmlViewConfig = NULL;

    ui.setupUi(this);

    //Process view configuration file, and set view
    //m_views = NULL;
    m_viewArea = NULL;
    m_vwidth = m_vheight = 2;
    m_virow = m_vicol = 0;
    m_nactive_views = 4;

    setupUiViews();

    setGrayScaleTable();
    setThermalScaleTable();

    counter = 0;

    m_dialog = new ModulesDialog(m_videoAnalysis, this);
    m_parameterDialog = new ParameterDialog(m_videoAnalysis, this);
    from_next = false;

    //Init Processing Thread
    producer = new ProductionThread(m_videoAnalysis);
    QObject::connect(producer,SIGNAL(load_data()),this,SLOT(showImages()));
    QObject::connect(producer,SIGNAL(bad_init()),this,SLOT(undonePlay()));
    producer->start();

    QObject::connect(ui.actionModule_Sequence_Editor,SIGNAL(triggered()),m_dialog,SLOT(init()));
    QObject::connect(ui.actionModule_Sequence_Editor,SIGNAL(triggered()),m_dialog,SLOT(show()));

    QObject::connect(ui.actionModule_Parameters_Editor,SIGNAL(triggered()),m_parameterDialog,SLOT(init()));
    QObject::connect(ui.actionModule_Parameters_Editor,SIGNAL(triggered()),m_parameterDialog,SLOT(show()));

}

void MainWindow::setupUiViews() {
    if(m_viewArea != NULL)
        delete m_viewArea;
    m_viewArea = new QScrollArea(this);
    QFrame *viewsFrame = new QFrame();

    //1. Init xml reading process
    if(xmlViewConfig != NULL)
        delete xmlViewConfig;
    xmlViewConfig = new QDomDocument( "VIEW_PARAMETERS" );
    QFile file(m_view_config_file);
    QString q_config_file(m_view_config_file);
    if( !file.open(QIODevice::ReadOnly) ) {
        AppendToLog("Error opening view configuration file '" + q_config_file + "' for reading. Execution will be aborted.");
        std::cerr << "Error opening view configuration file '"
                  << q_config_file.toStdString() << "' for reading. Execution will be aborted." << std::endl;
        return;
    }

    QString error;
    int line, column;
    if( !xmlViewConfig->setContent( &file, &error, &line, &column ) ) {

        AppendToLog(   "Error reading view configuration file '" + q_config_file + "'. XML content could not be extracted. Execution will be aborted.\n"
                                    + "Error on line " + QString::number(line) + " and column " + QString::number(column) + ": " + error);
        std::cerr << "Error reading view configuration file '"
                  << q_config_file.toStdString() << "'. XML content could not be extracted. Execution will be aborted.\n"
                  << "Error in line " << line << " and column " << column << + ": " + error.toStdString() << std::endl;
        file.close();
        return;
    }
    file.close();


    rootViewConfig = xmlViewConfig->documentElement();
    if( rootViewConfig.tagName() != "VIEW_PARAMETERS" ) {
        AppendToLog("Error reading XML information. XML content in the configuration file '" + q_config_file + "' does not correspond to application parameters description. Execution will be aborted.");
        return;
    }

    //2. Process parameters
    QDomNode n, m, o, p;
    if( ! ( n = XmlCommon::getParameterNode("ViewDistribution", rootViewConfig) ).isNull() ) {
        QString value = XmlCommon::getParameterValue(n);
        if(value == "4Views")
            m_vtype = _4Views;
        else if(value == "4ViewsMILES")
            m_vtype = _4ViewsMILES;
        else if(value == "2ViewsH")
            m_vtype = _2ViewsH;
        else if(value == "2ViewsV")
            m_vtype = _2ViewsV;
        else {
            m_vtype = _4Views;
            AppendToLog("Error processing ViewDistribution entry: Undefined value: '" + value + "'. Taking default: '4Views'.");
        }

        initViews(viewsFrame);

        m = n.firstChild();
        QDomElement e;
        bool activated;
        int row, col;
        while(!m.isNull()) {
            e = m.toElement();
            if( e.tagName() == "View" ) {
                activated = e.attribute( "value", "" ) == "true" ? true : false;
                if(activated) {
                    if( ! ( o = XmlCommon::getParameterNode("Row", m) ).isNull() ) {
                        row = XmlCommon::getParameterValue(o).toInt();
                        if(row >= 0 && row < m_vheight) {
                            if( ! ( o = XmlCommon::getParameterNode("Column", m) ).isNull() ) {
                                col = XmlCommon::getParameterValue(o).toInt();
                                if(col >= 0 && col < m_vwidth) {
                                    //Valid coordinate according to defined distribution
                                    if( ! ( o = XmlCommon::getParameterNode("Label", m) ).isNull() ) {
                                        m_views[row*m_vwidth + col]->setLabel(XmlCommon::getParameterValue(o));
                                        //create a paintView with the new View
                                        if(!(o = XmlCommon::getParameterNode("afterModule", m)).isNull()){
                                            QString value = XmlCommon::getParameterValue(o);
                                            paintView *paint = new paintView(m_data, value, col, row, m_vwidth);
                                            if(!(o = XmlCommon::getParameterNode("sequence", m)).isNull()){
                                                bool flag = (XmlCommon::getParameterValue(o) == "true")? true: false;
                                                //Se actualiza el flag y si es necesario se setean los parametros del paintView.
                                                paint->setDisplayPaint(flag);
                                                if(flag)
                                                    paint->setParameters(o);

                                                if((o = XmlCommon::getParameterNode("SaveToDir", m)).isNull()){
                                                    paint->saveImage = false;
                                                } else {
                                                    bool flag = (XmlCommon::getParameterValue(o) == "true")? true: false;
                                                    paint->saveImage = flag;
                                                    if(flag) {
                                                        if((p = XmlCommon::getParameterNode("Directory", o)).isNull()) {
                                                            paint->saveImage = false;
                                                        } else {
                                                            if( (paint->saveDir = XmlCommon::getParameterValue(p) ) == "" )
                                                                paint->saveImage = false;
                                                        }
                                                    }
                                                }
                                            }
                                            m_videoAnalysis->paintDeque.push_back(paint);

                                        }
                                    }
                                }
                            }
                        }
                    }

                }
            } else if( e.tagName() != "" ) {
                AppendToLog("Error reading views config file: Children of ViewDistribution canonly be View. Element '" + e.tagName() + "' ignored.");
            }
            m = m.nextSibling();
        }

        if( ! ( n = XmlCommon::getParameterNode("ViewSize", rootViewConfig) ).isNull() ) {
            activated = (XmlCommon::getParameterValue(n) == "true") ? true : false;
            if(activated) {
                if( ! ( m = XmlCommon::getParameterNode("Width", n) ).isNull() ) {
                     m_vhorpix = XmlCommon::getParameterValue(m).toInt();
                    if(m_vhorpix < 100)
                        m_vhorpix = 100;
                } else
                    m_vhorpix = 352;

                if( ! ( m = XmlCommon::getParameterNode("Height", n) ).isNull() ) {
                     m_vverpix = XmlCommon::getParameterValue(m).toInt();
                    if(m_vverpix < 100)
                        m_vverpix = 100;
                } else
                    m_vverpix = 330;
            } else {
                m_vhorpix = 352;
                m_vverpix = 330;
            }
        } else {
            m_vhorpix = 352;
            m_vverpix = 330;
        }
    } else { //No environment defined
       m_nactive_views = 0;
    }

    int border = 10;
    int interspace = 10;

    int width = 2*border + m_vwidth*m_vhorpix + (m_vwidth - 1)*interspace;
    int height = 2*border + m_vheight*m_vverpix + (m_vheight - 1)*interspace;

    //Set name of non set views
    int i, j;
    for(i=0; i<m_vheight; i++)
        for(j=0; j<m_vwidth; j++) {
            if(m_views[i*m_vwidth + j]->getLabel() == "") {
                m_views[i*m_vwidth + j]->setLabel("View (" + QString::number(i) + "," + QString::number(j) + ")");
            }
            //Set position
            //m_views[i*m_vwidth + j]->move(border + j*(m_vhorpix + interspace), border + i*(m_vverpix + interspace));
            //m_views[i*m_vwidth + j]->setGeometry(m_views[i*m_vwidth + j]->pos().x(),m_views[i*m_vwidth + j]->pos().y(),m_vhorpix, m_vverpix);
            m_views[i*m_vwidth + j]->setGeometry(border + j*(m_vhorpix + interspace), border + i*(m_vverpix + interspace),
                                                 m_vhorpix, m_vverpix);

        }
    //3. Clean xml variable
    delete xmlViewConfig;
    xmlViewConfig = NULL;

    m_viewArea->move(5,35);
    m_viewArea->resize(this->width()-226, this->height()-60);
    //ui.layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
//    QRect r(10,10,400,400);
//    ui.layout->setGeometry(r);
    viewsFrame->resize(width,height);
//    m_viewArea->setWidgetResizable(true);
    m_viewArea->setMaximumSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX);
    m_viewArea->setMinimumSize(300,300);
    m_viewArea->setWidget(viewsFrame);
    m_viewArea->setAlignment(Qt::AlignLeft);
    //m_viewArea->setBackgroundRole(QPalette::Light);


}

//Formato ARGB

void MainWindow::setGrayScaleTable() {
  int i;
  grayScaleTable = new QVector<QRgb>(256);

  for(i=0;i<256;i++)
    (*grayScaleTable)[i] = qRgb(i,i,i);

}

void MainWindow::setThermalScaleTable() {
  int i;
  thermalScaleTable = new QVector<QRgb>(256);

  for(i=0;i<256;i++)
    (*thermalScaleTable)[i] = qRgb(i,0,255-i);

}

MainWindow::~MainWindow() {
    producer->do_end();
    delete grayScaleTable;
    delete m_dialog;
    if(xmlViewConfig != NULL)
        delete xmlViewConfig;
    if(! m_views.empty()) {
        if(m_nactive_views > 0)
            for(int i=0; i<m_nactive_views; i++)
                if(m_views[i] != NULL)
                    delete m_views[i];
        m_views.clear();

    }
    std::cout << "Ahi si?" << std::endl;
    delete m_videoAnalysis;
}

void MainWindow::initViews(QFrame *parentFrame) {
    if(m_vtype == _4Views) {
        m_nactive_views = 4;
        m_vwidth = 2;
        m_vheight = 2;
        m_virow = m_vicol = 0;
    } else if(m_vtype == _4ViewsMILES) {
        m_nactive_views = 4;
        m_vwidth = 2;
        m_vheight = 2;
        m_virow = m_vicol = 0;
    } else if(m_vtype == _2ViewsH) {
        m_nactive_views = 2;
        m_vwidth = 2;
        m_vheight = 1;
        m_virow = m_vicol = 0;
    } else {
        m_nactive_views = 2;
        m_vwidth = 1;
        m_vheight = 2;
        m_virow = m_vicol = 0;
    }

    int i;
    if(!m_views.empty()) {
        for(i=0; i<m_nactive_views; i++)
            delete m_views[i];
        m_views.clear();
    }

    for(i=0; i<m_nactive_views; i++)
        m_views.insert(i, new ViewForm(parentFrame));
}

void MainWindow::appendToLog(QString s) {
    if(ui.logCheck->isChecked())
        ui.log->append(s);
}


void MainWindow::disableSaveViews() {
    int i, j;
    for(i=0; i<m_vheight; i++) {
        for(j=0; j<m_vwidth; j++) {
            m_views[i*m_vwidth+j]->setEnableSave(false);
        }
    }
}

void MainWindow::on_playButton_clicked() {
    //Se deshabilitan los botones
    ui.playButton->setEnabled(false);
    ui.nextButton->setEnabled(false);
    ui.pauseButton->setEnabled(true);
    ui.initialButton->setEnabled(false);
    disableSaveViews();
    from_next = false;
    producer->do_play();
}

void MainWindow::undonePlay() {
    parametrized = false;
    initialized = false;
    AppendToLog("VideoAnalysis: Execution aborted due to error in parametrization and/or initialization steps.");
    //Se deshabilitan los botones
    ui.playButton->setEnabled(true);
    ui.nextButton->setEnabled(true);
    ui.pauseButton->setEnabled(false);
    ui.initialButton->setEnabled(true);
    if(!m_views.empty()) disableSaveViews();
    from_next = true;
}

void MainWindow::showImages() {
//    if(m_vtype == _4Views) {
//        m_views[0]->useImage(m_videoAnalysis->m_data->currentImage);
//        m_views[1]->useImage(m_videoAnalysis->segmentationDisplay());
//        m_views[2]->useImage(m_videoAnalysis->bgDisplay());
//        m_views[3]->useImage(m_videoAnalysis->trackingDisplay());
//    } else if (m_vtype == _4ViewsMILES) {
//        m_views[0]->useImage(m_videoAnalysis->m_data->currentImage);
//        m_views[1]->useImage(m_videoAnalysis->trackingDisplay());
//        m_views[2]->useImage(m_videoAnalysis->segmentationDisplay());
//        m_views[3]->useImage(m_videoAnalysis->learningDisplay());

//    } else {
//        m_views[0]->useImage(m_videoAnalysis->segmentationDisplay());
//        m_views[1]->useImage(m_videoAnalysis->trackingDisplay());
//    }
    int elapsed;
    QTime t;
    t.start();

    std::deque<paintView *>::iterator it, it_end = m_videoAnalysis->paintDeque.end();
    //Se revisa la lista de imagenes que se mostraran en la interfaz
    for(it = m_videoAnalysis->paintDeque.begin(); it != it_end; it++)
        //Se verifica quiere ser mostrada
        if((*it)->getDisplayPaint())
            //Se mustra la imagen deseada en la posicion que se le indico.
            m_views[(*it)->getIndexView()]->useImage((*it)->getImage());

/*    if(m_videoAnalysis->m_activeVideoOutputSeg == false)
        std::cout << "SEG es falsa!!" << std::endl;
    if(m_videoAnalysis->m_activeVideoOutputTra == false)
        std::cout << "TRA es falsa!!" << std::endl;
    if(m_videoAnalysis->m_activeVideoOutputLea == false)
        std::cout << "LEA es falsa!!" << std::endl;
*/
    elapsed = t.elapsed();
    //std::cout << "Next elapsed time: " << elapsed << std::cout;

    fNumber.setNum(m_videoAnalysis->m_data->frameNumber);
    datapoolmutex.unlock();

    AppendToLog("frame:" + fNumber);

    producer->load_done();
}

void MainWindow::on_pauseButton_clicked() {
    from_next = true;
    //Se habilitan los botones
    ui.playButton->setEnabled(true);
    ui.nextButton->setEnabled(true);
    ui.initialButton->setEnabled(true);
    producer->do_pause();
}

void MainWindow::on_nextButton_clicked(){
    from_next = true;
    producer->do_next();
}

void MainWindow::on_initialButton_clicked(){
    producer->do_init();
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    m_viewArea->resize(this->width()-226, this->height()-60);
    ui.controlFrame->move(m_viewArea->width() + 10, ui.controlFrame->y());
    ui.logFrame->move(m_viewArea->width() + 10, ui.logFrame->y());
    ui.logFrame->resize(ui.logFrame->width(), this->height() - 270 - 50);
    ui.log->resize(ui.logFrame->width(), this->height() - 300 - 60);

}

void MainWindow::closeEvent(QCloseEvent *event) {
    producer->do_end();

    int i, s = m_views.size();
    for(i=0;i<s;i++) {
        m_views[i]->getViewport()->free_window();
    }
    event->accept();

    // copiado desde destructor:
    delete grayScaleTable;
    delete m_dialog;
    if(xmlViewConfig != NULL)
        delete xmlViewConfig;
    if(! m_views.empty()) {
        if(m_nactive_views > 0)
            for(int i=0; i<m_nactive_views; i++)
                if(m_views[i] != NULL)
                    delete m_views[i];
        m_views.clear();

    }
    //std::cout << "Ahi si?" << std::endl;
    delete m_videoAnalysis;

}
