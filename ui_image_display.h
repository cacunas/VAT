/********************************************************************************
** Form generated from reading UI file 'image_display.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGE_DISPLAY_H
#define UI_IMAGE_DISPLAY_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionModule_Sequence_Editor;
    QAction *actionView_Editor;
    QAction *actionQuit;
    QAction *actionModule_Parameters_Editor;
    QWidget *centralwidget;
    QFrame *controlFrame;
    QPushButton *playButton;
    QPushButton *pauseButton;
    QPushButton *nextButton;
    QPushButton *initialButton;
    QLabel *label_3;
    QFrame *logFrame;
    QCheckBox *logCheck;
    QPushButton *clearLog;
    QTextEdit *log;
    QStatusBar *statusbar;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuConfiguration;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(1145, 705);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(600, 400));
        MainWindow->setMaximumSize(QSize(10000, 10000));
        MainWindow->setDockOptions(QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks);
        actionModule_Sequence_Editor = new QAction(MainWindow);
        actionModule_Sequence_Editor->setObjectName(QString::fromUtf8("actionModule_Sequence_Editor"));
        actionView_Editor = new QAction(MainWindow);
        actionView_Editor->setObjectName(QString::fromUtf8("actionView_Editor"));
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionModule_Parameters_Editor = new QAction(MainWindow);
        actionModule_Parameters_Editor->setObjectName(QString::fromUtf8("actionModule_Parameters_Editor"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        controlFrame = new QFrame(centralwidget);
        controlFrame->setObjectName(QString::fromUtf8("controlFrame"));
        controlFrame->setGeometry(QRect(930, 10, 211, 256));
        controlFrame->setFrameShape(QFrame::StyledPanel);
        controlFrame->setFrameShadow(QFrame::Raised);
        playButton = new QPushButton(controlFrame);
        playButton->setObjectName(QString::fromUtf8("playButton"));
        playButton->setEnabled(true);
        playButton->setGeometry(QRect(10, 15, 191, 51));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        playButton->setFont(font);
        pauseButton = new QPushButton(controlFrame);
        pauseButton->setObjectName(QString::fromUtf8("pauseButton"));
        pauseButton->setGeometry(QRect(10, 75, 191, 51));
        pauseButton->setFont(font);
        nextButton = new QPushButton(controlFrame);
        nextButton->setObjectName(QString::fromUtf8("nextButton"));
        nextButton->setGeometry(QRect(10, 135, 191, 51));
        nextButton->setFont(font);
        initialButton = new QPushButton(controlFrame);
        initialButton->setObjectName(QString::fromUtf8("initialButton"));
        initialButton->setGeometry(QRect(10, 195, 191, 51));
        initialButton->setFont(font);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 325, 111, 24));
        QFont font1;
        font1.setPointSize(8);
        font1.setBold(true);
        font1.setWeight(75);
        label_3->setFont(font1);
        logFrame = new QFrame(centralwidget);
        logFrame->setObjectName(QString::fromUtf8("logFrame"));
        logFrame->setGeometry(QRect(930, 270, 211, 381));
        logFrame->setFrameShape(QFrame::StyledPanel);
        logFrame->setFrameShadow(QFrame::Raised);
        logCheck = new QCheckBox(logFrame);
        logCheck->setObjectName(QString::fromUtf8("logCheck"));
        logCheck->setEnabled(true);
        logCheck->setGeometry(QRect(15, 0, 101, 31));
        QFont font2;
        font2.setPointSize(9);
        font2.setBold(true);
        font2.setWeight(75);
        logCheck->setFont(font2);
        logCheck->setChecked(true);
        logCheck->setTristate(false);
        clearLog = new QPushButton(logFrame);
        clearLog->setObjectName(QString::fromUtf8("clearLog"));
        clearLog->setGeometry(QRect(130, 5, 76, 21));
        clearLog->setFont(font);
        log = new QTextEdit(logFrame);
        log->setObjectName(QString::fromUtf8("log"));
        log->setGeometry(QRect(5, 30, 200, 346));
        log->setMinimumSize(QSize(200, 0));
        log->setMaximumSize(QSize(200, 16777215));
        QFont font3;
        font3.setPointSize(7);
        font3.setBold(true);
        font3.setWeight(75);
        log->setFont(font3);
        log->setLineWidth(165);
        log->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        log->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        log->setReadOnly(true);
        log->setOverwriteMode(false);
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1145, 27));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuConfiguration = new QMenu(menuBar);
        menuConfiguration->setObjectName(QString::fromUtf8("menuConfiguration"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuConfiguration->menuAction());
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuConfiguration->addAction(actionModule_Sequence_Editor);
        menuConfiguration->addAction(actionModule_Parameters_Editor);
        menuConfiguration->addAction(actionView_Editor);

        retranslateUi(MainWindow);
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(clearLog, SIGNAL(clicked()), log, SLOT(clear()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Video Analysis Tool", 0, QApplication::UnicodeUTF8));
        actionModule_Sequence_Editor->setText(QApplication::translate("MainWindow", "Module Sequence Editor", 0, QApplication::UnicodeUTF8));
        actionModule_Sequence_Editor->setShortcut(QApplication::translate("MainWindow", "Ctrl+M", 0, QApplication::UnicodeUTF8));
        actionView_Editor->setText(QApplication::translate("MainWindow", "View Editor", 0, QApplication::UnicodeUTF8));
        actionView_Editor->setShortcut(QApplication::translate("MainWindow", "Ctrl+V", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        actionQuit->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        actionModule_Parameters_Editor->setText(QApplication::translate("MainWindow", "Module Parameters Editor", 0, QApplication::UnicodeUTF8));
        playButton->setText(QApplication::translate("MainWindow", "Play", 0, QApplication::UnicodeUTF8));
        pauseButton->setText(QApplication::translate("MainWindow", "Pause", 0, QApplication::UnicodeUTF8));
        nextButton->setText(QApplication::translate("MainWindow", "Next\n"
"Frame", 0, QApplication::UnicodeUTF8));
        initialButton->setText(QApplication::translate("MainWindow", "Initial\n"
"Frame", 0, QApplication::UnicodeUTF8));
        label_3->setText(QString());
        logCheck->setText(QApplication::translate("MainWindow", "Output Log", 0, QApplication::UnicodeUTF8));
        clearLog->setText(QApplication::translate("MainWindow", "clear", 0, QApplication::UnicodeUTF8));
        log->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Cantarell'; font-size:7pt; font-weight:600; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'DejaVu Sans'; font-size:9pt; font-weight:400;\"><br /></p></body></html>", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuConfiguration->setTitle(QApplication::translate("MainWindow", "Configuration", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGE_DISPLAY_H
