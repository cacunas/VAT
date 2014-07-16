/********************************************************************************
** Form generated from reading UI file 'ImageWindowDisplay.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGEWINDOWDISPLAY_H
#define UI_IMAGEWINDOWDISPLAY_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ImageWindowDisplay
{
public:
    QWidget *centralwidget;
    QPushButton *saveImage;
    QPushButton *exit;
    QScrollArea *scrollArea;
    QLabel *imageDisplay;

    void setupUi(QMainWindow *ImageWindowDisplay)
    {
        if (ImageWindowDisplay->objectName().isEmpty())
            ImageWindowDisplay->setObjectName(QString::fromUtf8("ImageWindowDisplay"));
        ImageWindowDisplay->setWindowModality(Qt::NonModal);
        ImageWindowDisplay->resize(655, 600);
        ImageWindowDisplay->setMinimumSize(QSize(300, 200));
        ImageWindowDisplay->setMaximumSize(QSize(10000, 10000));
        ImageWindowDisplay->setDockOptions(QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks);
        centralwidget = new QWidget(ImageWindowDisplay);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        saveImage = new QPushButton(centralwidget);
        saveImage->setObjectName(QString::fromUtf8("saveImage"));
        saveImage->setEnabled(false);
        saveImage->setGeometry(QRect(10, 0, 131, 24));
        exit = new QPushButton(centralwidget);
        exit->setObjectName(QString::fromUtf8("exit"));
        exit->setGeometry(QRect(150, 0, 131, 24));
        scrollArea = new QScrollArea(centralwidget);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setGeometry(QRect(0, 30, 650, 550));
        scrollArea->setMinimumSize(QSize(0, 0));
        scrollArea->setMaximumSize(QSize(10000, 10000));
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setWidgetResizable(false);
        scrollArea->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        imageDisplay = new QLabel();
        imageDisplay->setObjectName(QString::fromUtf8("imageDisplay"));
        imageDisplay->setGeometry(QRect(0, 0, 548, 350));
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(imageDisplay->sizePolicy().hasHeightForWidth());
        imageDisplay->setSizePolicy(sizePolicy);
        imageDisplay->setMinimumSize(QSize(0, 0));
        imageDisplay->setSizeIncrement(QSize(0, 0));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        palette.setBrush(QPalette::Active, QPalette::Window, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush);
        imageDisplay->setPalette(palette);
        imageDisplay->setAutoFillBackground(true);
        imageDisplay->setFrameShape(QFrame::StyledPanel);
        imageDisplay->setFrameShadow(QFrame::Sunken);
        imageDisplay->setLineWidth(8);
        imageDisplay->setMidLineWidth(8);
        scrollArea->setWidget(imageDisplay);
        ImageWindowDisplay->setCentralWidget(centralwidget);

        retranslateUi(ImageWindowDisplay);
        QObject::connect(exit, SIGNAL(clicked()), ImageWindowDisplay, SLOT(close()));

        QMetaObject::connectSlotsByName(ImageWindowDisplay);
    } // setupUi

    void retranslateUi(QMainWindow *ImageWindowDisplay)
    {
        ImageWindowDisplay->setWindowTitle(QString());
        saveImage->setText(QApplication::translate("ImageWindowDisplay", "Save Image", 0, QApplication::UnicodeUTF8));
        exit->setText(QApplication::translate("ImageWindowDisplay", "Exit", 0, QApplication::UnicodeUTF8));
        imageDisplay->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ImageWindowDisplay: public Ui_ImageWindowDisplay {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGEWINDOWDISPLAY_H
