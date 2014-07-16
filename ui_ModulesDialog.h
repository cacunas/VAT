/********************************************************************************
** Form generated from reading UI file 'ModulesDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MODULESDIALOG_H
#define UI_MODULESDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_ModulesDialog
{
public:
    QPushButton *addButton;
    QPushButton *upButton;
    QPushButton *downButton;
    QPushButton *supButton;
    QPushButton *cancelButton;
    QPushButton *okButton;
    QPushButton *saveButton;
    QPushButton *loadButton;
    QLabel *label;
    QLabel *label_2;
    QListWidget *sequenceView;
    QListWidget *availableView;

    void setupUi(QDialog *ModulesDialog)
    {
        if (ModulesDialog->objectName().isEmpty())
            ModulesDialog->setObjectName(QString::fromUtf8("ModulesDialog"));
        ModulesDialog->resize(522, 390);
        ModulesDialog->setModal(true);
        addButton = new QPushButton(ModulesDialog);
        addButton->setObjectName(QString::fromUtf8("addButton"));
        addButton->setGeometry(QRect(240, 130, 41, 41));
        QIcon icon;
        icon.addFile(QString::fromUtf8("icons/1leftarrow.png"), QSize(), QIcon::Normal, QIcon::Off);
        addButton->setIcon(icon);
        addButton->setIconSize(QSize(32, 32));
        upButton = new QPushButton(ModulesDialog);
        upButton->setObjectName(QString::fromUtf8("upButton"));
        upButton->setGeometry(QRect(20, 280, 41, 41));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8("icons/1uparrow.png"), QSize(), QIcon::Normal, QIcon::Off);
        upButton->setIcon(icon1);
        upButton->setIconSize(QSize(32, 32));
        downButton = new QPushButton(ModulesDialog);
        downButton->setObjectName(QString::fromUtf8("downButton"));
        downButton->setGeometry(QRect(70, 280, 41, 41));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8("icons/1downarrow.png"), QSize(), QIcon::Normal, QIcon::Off);
        downButton->setIcon(icon2);
        downButton->setIconSize(QSize(32, 32));
        supButton = new QPushButton(ModulesDialog);
        supButton->setObjectName(QString::fromUtf8("supButton"));
        supButton->setGeometry(QRect(190, 280, 41, 41));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8("icons/no.png"), QSize(), QIcon::Normal, QIcon::Off);
        supButton->setIcon(icon3);
        supButton->setIconSize(QSize(32, 32));
        cancelButton = new QPushButton(ModulesDialog);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));
        cancelButton->setGeometry(QRect(400, 340, 101, 41));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8("icons/cancel.png"), QSize(), QIcon::Normal, QIcon::Off);
        cancelButton->setIcon(icon4);
        cancelButton->setIconSize(QSize(32, 32));
        okButton = new QPushButton(ModulesDialog);
        okButton->setObjectName(QString::fromUtf8("okButton"));
        okButton->setGeometry(QRect(290, 340, 101, 41));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8("icons/ok.png"), QSize(), QIcon::Normal, QIcon::Off);
        okButton->setIcon(icon5);
        okButton->setIconSize(QSize(32, 32));
        saveButton = new QPushButton(ModulesDialog);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setGeometry(QRect(130, 340, 101, 41));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8("icons/filesaveas.png"), QSize(), QIcon::Normal, QIcon::Off);
        saveButton->setIcon(icon6);
        saveButton->setIconSize(QSize(32, 32));
        loadButton = new QPushButton(ModulesDialog);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));
        loadButton->setGeometry(QRect(20, 340, 101, 41));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8("icons/fileopen.png"), QSize(), QIcon::Normal, QIcon::Off);
        loadButton->setIcon(icon7);
        loadButton->setIconSize(QSize(32, 32));
        label = new QLabel(ModulesDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 10, 111, 16));
        label_2 = new QLabel(ModulesDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(300, 10, 161, 16));
        sequenceView = new QListWidget(ModulesDialog);
        sequenceView->setObjectName(QString::fromUtf8("sequenceView"));
        sequenceView->setGeometry(QRect(20, 30, 211, 251));
        availableView = new QListWidget(ModulesDialog);
        availableView->setObjectName(QString::fromUtf8("availableView"));
        availableView->setGeometry(QRect(290, 30, 211, 251));

        retranslateUi(ModulesDialog);
        QObject::connect(cancelButton, SIGNAL(clicked()), ModulesDialog, SLOT(hide()));

        QMetaObject::connectSlotsByName(ModulesDialog);
    } // setupUi

    void retranslateUi(QDialog *ModulesDialog)
    {
        ModulesDialog->setWindowTitle(QApplication::translate("ModulesDialog", "Module Sequence Editor", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        addButton->setToolTip(QApplication::translate("ModulesDialog", "Insert selected available module into the module sequence.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        addButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        upButton->setToolTip(QApplication::translate("ModulesDialog", "Swap selected module with module before it.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        upButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        downButton->setToolTip(QApplication::translate("ModulesDialog", "Swap selected module with module after it.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        downButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        supButton->setToolTip(QApplication::translate("ModulesDialog", "Suppress module from execution sequence.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        supButton->setText(QString());
        cancelButton->setText(QApplication::translate("ModulesDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("ModulesDialog", "OK", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        saveButton->setToolTip(QApplication::translate("ModulesDialog", "Save current module sequence.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        saveButton->setText(QApplication::translate("ModulesDialog", "Save", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        loadButton->setToolTip(QApplication::translate("ModulesDialog", "Load module sequence.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        loadButton->setText(QApplication::translate("ModulesDialog", "Load", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ModulesDialog", "Module Sequence", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ModulesDialog", "Available Modules", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ModulesDialog: public Ui_ModulesDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MODULESDIALOG_H
