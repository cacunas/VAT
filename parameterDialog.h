#ifndef PARAMETERDIALOG_H
#define PARAMETERDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "VideoAnalysis.h"
#include <iostream>

namespace Ui {
    class ParameterDialog;
}

class ParameterDialog : public QDialog
{
    Q_OBJECT

public:
    ParameterDialog(VideoAnalysis *i_va, QWidget *parent = 0);
    ~ParameterDialog();
    void setCurrentSequence();
    void clearLists();
    std::deque<parameter> children(parameter &parent);
    std::deque<parameter> getParameters(int index);
    QString writeXML(parameter &parent);

    //listas de etiquetas y campos editables de cada modulo.
    std::deque<QSpinBox *>valueSB;
    std::deque<QDoubleSpinBox *>valueDSB;
    std::deque<QLineEdit *>valueL;
    std::deque<QLabel *>nameParam;


    VideoAnalysis *va;
    std::deque<std::string> currentSequence;


public slots:
    void init();
    void updateField(QString);
    void modifyParameters(QString);

private slots:
    void on_resetButton_clicked();

    void on_guardarButton_clicked();

    void on_buttonBox_accepted();

    void on_createButton_clicked();

private:
    Ui::ParameterDialog *ui;

};

#endif // PARAMETERDIALOG_H
