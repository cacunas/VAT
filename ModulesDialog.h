#ifndef MODULESDIALOG_H
#define MODULESDIALOG_H

#ifdef __COMPILE_QT5__
    #include <QtWidgets/QDialog>
#else
    #include <QtGui/QDialog>
#endif
#include "VideoAnalysis.h"

namespace Ui {
    class ModulesDialog;
}

class ModulesDialog : public QDialog {
    Q_OBJECT
public:
    ModulesDialog(VideoAnalysis *i_va, QWidget *parent = 0);
    ~ModulesDialog();

    void insertAvailableModules();
    void setCurrentSequence();
    void initButtons();
    void setButtonsBySequence(int currentRow);

    std::deque<std::string> currentSequence;
    int current_row;
    VideoAnalysis *va;
private slots:
   void on_addButton_clicked();
   void on_okButton_clicked();
   void on_supButton_clicked();
   void on_upButton_clicked();
   void on_downButton_clicked();
   void on_loadButton_clicked();
   void on_saveButton_clicked();
   void on_availableView_currentRowChanged (int currentRow);
   void on_sequenceView_currentRowChanged (int currentRow);
   void init();

  signals:
   void changes_ok();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::ModulesDialog *m_ui;
};

#endif // MODULESDIALOG_H
