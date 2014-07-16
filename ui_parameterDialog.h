/********************************************************************************
** Form generated from reading UI file 'parameterDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PARAMETERDIALOG_H
#define UI_PARAMETERDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ParameterDialog
{
public:
    QDialogButtonBox *buttonBox;
    QListWidget *currentList;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QLabel *label;
    QPushButton *guardarButton;
    QPushButton *resetButton;
    QPushButton *createButton;

    void setupUi(QDialog *ParameterDialog)
    {
        if (ParameterDialog->objectName().isEmpty())
            ParameterDialog->setObjectName(QString::fromUtf8("ParameterDialog"));
        ParameterDialog->resize(589, 442);
        buttonBox = new QDialogButtonBox(ParameterDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(20, 400, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        currentList = new QListWidget(ParameterDialog);
        currentList->setObjectName(QString::fromUtf8("currentList"));
        currentList->setGeometry(QRect(10, 20, 241, 331));
        scrollArea = new QScrollArea(ParameterDialog);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setGeometry(QRect(270, 40, 311, 311));
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setWidgetResizable(false);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 292, 309));
        scrollAreaWidgetContents->setAutoFillBackground(true);
        scrollArea->setWidget(scrollAreaWidgetContents);
        label = new QLabel(ParameterDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(270, 20, 311, 16));
        guardarButton = new QPushButton(ParameterDialog);
        guardarButton->setObjectName(QString::fromUtf8("guardarButton"));
        guardarButton->setGeometry(QRect(510, 360, 75, 23));
        resetButton = new QPushButton(ParameterDialog);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));
        resetButton->setGeometry(QRect(430, 360, 75, 23));
        createButton = new QPushButton(ParameterDialog);
        createButton->setObjectName(QString::fromUtf8("createButton"));
        createButton->setGeometry(QRect(10, 360, 131, 23));

        retranslateUi(ParameterDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ParameterDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ParameterDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(ParameterDialog);
    } // setupUi

    void retranslateUi(QDialog *ParameterDialog)
    {
        ParameterDialog->setWindowTitle(QApplication::translate("ParameterDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ParameterDialog", "Parameters of --:", 0, QApplication::UnicodeUTF8));
        guardarButton->setText(QApplication::translate("ParameterDialog", "Save Values", 0, QApplication::UnicodeUTF8));
        resetButton->setText(QApplication::translate("ParameterDialog", "Reset Values", 0, QApplication::UnicodeUTF8));
        createButton->setText(QApplication::translate("ParameterDialog", "Create a New Config File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ParameterDialog: public Ui_ParameterDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARAMETERDIALOG_H
