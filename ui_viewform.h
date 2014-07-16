/********************************************************************************
** Form generated from reading UI file 'viewform.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VIEWFORM_H
#define UI_VIEWFORM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QWidget>
#include <viewportqlabel.h>

QT_BEGIN_NAMESPACE

class Ui_ViewForm
{
public:
    QLabel *label;
    QFrame *line;
    QScrollArea *ScrollArea;
    ViewportQLabel *viewport;
    QPushButton *saveView;

    void setupUi(QWidget *ViewForm)
    {
        if (ViewForm->objectName().isEmpty())
            ViewForm->setObjectName(QString::fromUtf8("ViewForm"));
        ViewForm->resize(600, 400);
        ViewForm->setSizeIncrement(QSize(1, 1));
        label = new QLabel(ViewForm);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(12, 6, 291, 20));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        line = new QFrame(ViewForm);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(120, 7, 221, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        ScrollArea = new QScrollArea(ViewForm);
        ScrollArea->setObjectName(QString::fromUtf8("ScrollArea"));
        ScrollArea->setGeometry(QRect(2, 28, 451, 300));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ScrollArea->sizePolicy().hasHeightForWidth());
        ScrollArea->setSizePolicy(sizePolicy);
        ScrollArea->setMinimumSize(QSize(451, 300));
        ScrollArea->setMaximumSize(QSize(451, 311));
        QFont font1;
        font1.setPointSize(8);
        ScrollArea->setFont(font1);
        ScrollArea->viewport()->setProperty("cursor", QVariant(QCursor(Qt::ArrowCursor)));
        ScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        ScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        ScrollArea->setWidgetResizable(false);
        ScrollArea->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        viewport = new ViewportQLabel();
        viewport->setObjectName(QString::fromUtf8("viewport"));
        viewport->setGeometry(QRect(0, 0, 548, 350));
        QSizePolicy sizePolicy1(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(viewport->sizePolicy().hasHeightForWidth());
        viewport->setSizePolicy(sizePolicy1);
        viewport->setMinimumSize(QSize(0, 0));
        viewport->setSizeIncrement(QSize(0, 0));
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        palette.setBrush(QPalette::Active, QPalette::Window, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush);
        viewport->setPalette(palette);
        viewport->setCursor(QCursor(Qt::PointingHandCursor));
        viewport->setAutoFillBackground(true);
        viewport->setFrameShape(QFrame::StyledPanel);
        viewport->setFrameShadow(QFrame::Sunken);
        viewport->setLineWidth(8);
        viewport->setMidLineWidth(8);
        ScrollArea->setWidget(viewport);
        saveView = new QPushButton(ViewForm);
        saveView->setObjectName(QString::fromUtf8("saveView"));
        saveView->setGeometry(QRect(350, 4, 90, 21));

        retranslateUi(ViewForm);

        QMetaObject::connectSlotsByName(ViewForm);
    } // setupUi

    void retranslateUi(QWidget *ViewForm)
    {
        ViewForm->setWindowTitle(QApplication::translate("ViewForm", "Form", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        viewport->setText(QString());
        saveView->setText(QApplication::translate("ViewForm", "Save", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ViewForm: public Ui_ViewForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIEWFORM_H
