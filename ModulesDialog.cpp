#ifdef __COMPILE_QT5__
    #include <QtWidgets/QMessageBox>
    #include <QtWidgets/QFileDialog>
#else
    #include <QMessageBox>
    #include <QFileDialog>
#endif

#include "ModulesDialog.h"
#include "ui_ModulesDialog.h"
#include <iostream>
#include <fstream>
ModulesDialog::ModulesDialog(VideoAnalysis *i_va, QWidget *parent) :
    QDialog(parent),
    va(i_va),
    m_ui(new Ui::ModulesDialog) {
    m_ui->setupUi(this);
    insertAvailableModules();
}

ModulesDialog::~ModulesDialog() {
    delete m_ui;
}

void ModulesDialog::insertAvailableModules() {
    if(va->availableModules.empty())
        return;
    m_ui->availableView->clear();
    std::set<std::string>::iterator it, it_end = va->availableModules.end();
    QString cur_module;
    for(it = va->availableModules.begin(); it != it_end; it++) {
        cur_module = it->c_str();
        new QListWidgetItem(cur_module, m_ui->availableView);
    }
}

void ModulesDialog::setCurrentSequence() {
    currentSequence.clear();
    m_ui->sequenceView->clear();
    std::deque<ModuleInterface *>::iterator it, it_end = va->moduleSequence.end();
    QString cur_module;
    for(it = va->moduleSequence.begin(); it != it_end; it++) {
        currentSequence.push_back((*it)->name);
        cur_module = (*it)->name.c_str();
        new QListWidgetItem(cur_module, m_ui->sequenceView);
    }
}

void ModulesDialog::initButtons() {
    m_ui->sequenceView->setCurrentRow(-1);
    m_ui->availableView->setCurrentRow(0);
    m_ui->addButton->setEnabled(true);
    m_ui->upButton->setEnabled(false);
    m_ui->downButton->setEnabled(false);
    m_ui->supButton->setEnabled(false);
    current_row = 0;
}


void ModulesDialog::on_addButton_clicked() {
    QString current_string = m_ui->availableView->item(current_row)->text();
    m_ui->sequenceView->insertItem(m_ui->sequenceView->count(), current_string);
    currentSequence.push_back(current_string.toStdString());
}

void ModulesDialog::on_availableView_currentRowChanged (int currentRow) {
    m_ui->sequenceView->setCurrentRow(-1);
    m_ui->addButton->setEnabled(true);
    m_ui->upButton->setEnabled(false);
    m_ui->downButton->setEnabled(false);
    m_ui->supButton->setEnabled(false);
    current_row = currentRow;
}

void ModulesDialog::setButtonsBySequence(int currentRow) {
    int count = m_ui->sequenceView->count();
    if(count == 0) {
        m_ui->upButton->setEnabled(false);
        m_ui->downButton->setEnabled(false);
        m_ui->supButton->setEnabled(false);
        m_ui->saveButton->setEnabled(false);
    } else if(count == 1) {
        m_ui->upButton->setEnabled(false);
        m_ui->downButton->setEnabled(false);
        m_ui->supButton->setEnabled(true);
        m_ui->saveButton->setEnabled(true);
    } else {
        m_ui->supButton->setEnabled(true);
        m_ui->upButton->setEnabled(currentRow == 0 ? false : true);
        m_ui->downButton->setEnabled(currentRow == count-1 ? false : true);
        m_ui->saveButton->setEnabled(true);
    }
}

void ModulesDialog::on_sequenceView_currentRowChanged (int currentRow) {
    m_ui->availableView->setCurrentRow(-1);
    m_ui->addButton->setEnabled(false);
    setButtonsBySequence(currentRow);
    current_row = currentRow;
}

void ModulesDialog::on_okButton_clicked() {
    if(currentSequence.empty()) {
        QMessageBox::warning(this, tr("Empty Module Sequence"),
            tr("The sequence is empty. Previous configuration will be considered."),
            QMessageBox::Ok, QMessageBox::Ok);
        this->hide();
        return;
    }

    std::deque<std::string> strModuleSequence;
    std::deque<ModuleInterface *>::iterator mod_it, mod_it_end = va->moduleSequence.end();
    for(mod_it = va->moduleSequence.begin(); mod_it != mod_it_end; mod_it++)
        strModuleSequence.push_back( (*mod_it)->name);

    if(currentSequence != strModuleSequence) {
        int ret = QMessageBox::question(this, tr("About to Change Sequence Order"),
                   tr("You are about to permanently modify the module sequence for this session. This will reinitialize all modules. \n"
                      "Do you want continue?"),
                   QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok);
        if(ret == QMessageBox::Ok) {


            //Clear modules and datapool:
            va->resetModules(currentSequence);
            va->setParameters();
            va->init();

            this->hide();
        }
    } else {
        std::cout << "Equal" << std::endl;
        this->hide();
    }
}

void ModulesDialog::on_supButton_clicked() {
    std::deque<std::string>::iterator it, it_end = currentSequence.end();
    int i = 0;
    for(it = currentSequence.begin();it != it_end; it++, i++)
        if(i == current_row) {
            currentSequence.erase(it);
            delete m_ui->sequenceView->takeItem(current_row);
            current_row = 0;
            m_ui->sequenceView->setCurrentRow(current_row);
            setButtonsBySequence(current_row);
            break;
        }
}

void ModulesDialog::on_upButton_clicked() {
    int c_row = current_row;
    std::string aux = currentSequence[c_row];
    currentSequence[c_row] = currentSequence[c_row - 1];
    currentSequence[c_row - 1] = aux;

    QListWidgetItem
            *item1 = m_ui->sequenceView->takeItem(c_row - 1),
            *item2 = m_ui->sequenceView->takeItem(c_row - 1);
    m_ui->sequenceView->insertItem(c_row - 1, item1);
    m_ui->sequenceView->insertItem(c_row - 1, item2);

    c_row--;
    current_row = c_row;
    m_ui->sequenceView->setCurrentRow(current_row);
    setButtonsBySequence(current_row);
}

void ModulesDialog::on_downButton_clicked() {
    int c_row = current_row;
    std::string aux = currentSequence[c_row];
    currentSequence[c_row] = currentSequence[c_row + 1];
    currentSequence[c_row + 1] = aux;

    QListWidgetItem
            *item1 = m_ui->sequenceView->takeItem(c_row),
            *item2 = m_ui->sequenceView->takeItem(c_row);
    m_ui->sequenceView->insertItem(c_row, item1);
    m_ui->sequenceView->insertItem(c_row, item2);

    c_row++;
    current_row = c_row;
    m_ui->sequenceView->setCurrentRow(current_row);
    setButtonsBySequence(current_row);

}

void ModulesDialog::on_loadButton_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Module Sequence File"),
                                                 "./",
                                                 tr("All files (*)"));
    if(fileName != "") {
        std::ifstream ifs(fileName.toLatin1());
        if(!ifs.is_open() || !ifs.good()) {
            QString message = "Unable to load file '" + fileName + "' for reading.";
            QMessageBox::warning(this, tr("Unable To Load File"),
                tr(message.toLatin1()),
                QMessageBox::Ok, QMessageBox::Ok);
            return;
        }

        std::string s;
        int i;
        size_t pos;
        std::deque<std::string> tmpSequence, unavailableSequence;

        while(getline(ifs, s)) {
            if( (pos=s.find_first_not_of(" \t\r\n")) != std::string::npos ) {
                s = s.substr(pos);
                i = s.size()-1;
                if(s[i] == '\n' || s[i] == '\r')
                    s = s.substr(0, i);
                if(va->availableModules.count(s) > 0)
                    tmpSequence.push_back(s);
                else
                    unavailableSequence.push_back(s);
            }
        }

        if(unavailableSequence.size() > 0) {
            QString message = "The following were not added, as they are unavailable:\n";
            std::deque<std::string>::iterator it, it_end = unavailableSequence.end();
            for(it = unavailableSequence.begin(); it != it_end; it++)
                message += QString(it->c_str()) + "\n";
            QMessageBox::warning(this, tr("Uvavailable Modules Detected"),
                tr(message.toLatin1()),
                QMessageBox::Ok, QMessageBox::Ok);
        }
        if(tmpSequence.size() == 0) {
            QString message = "None of the modules read from file '" + fileName + "' are available or file is empty. Keeping previous configuration.";
            QMessageBox::warning(this, tr("Unable To Load Any Module"),
                tr(message.toLatin1()),
                QMessageBox::Ok, QMessageBox::Ok);
            return;
        }
        //add to real data:
        currentSequence.clear();
        m_ui->sequenceView->clear();
        std::deque<std::string>::iterator it, it_end = tmpSequence.end();
        for(it = tmpSequence.begin(); it != it_end; it++) {
            currentSequence.push_back(*it);
            m_ui->sequenceView->addItem(QString(it->c_str()));
        }
    }
}

void ModulesDialog::on_saveButton_clicked() {
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Module Sequence File"),
                                                 "./",
                                                 tr("All files (*)"));
    if(fileName != "") {
        std::ifstream ifs(fileName.toLatin1());
        if(ifs.is_open()) {
            ifs.close();
            QString message = "Are you sure you want to overwrite file '" + fileName + "?";
            int answer = QMessageBox::question(this, tr("Overwrite Existing File"),
                tr(message.toLatin1()),
                QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok);
            if(answer == QMessageBox::Cancel)
                return;
        }
        std::ofstream ofs(fileName.toLatin1());

        std::deque<std::string>::iterator it, it_end = currentSequence.end();
        for(it = currentSequence.begin(); it != it_end; it++)
            ofs << *it << std::endl;
    }

}

void ModulesDialog::init() {
    setCurrentSequence();
    initButtons();
}

void ModulesDialog::changeEvent(QEvent *e) {
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
