#include "parameterDialog.h"
#include "ui_parameterDialog.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QSpinBox>
#include <QMutex>

extern QMutex paramMutex;

ParameterDialog::ParameterDialog(VideoAnalysis *i_va, QWidget *parent) : QDialog(parent), va(i_va), ui(new Ui::ParameterDialog){
    ui->setupUi(this);
    QObject::connect(ui->currentList,SIGNAL(currentTextChanged(QString)),this,SLOT(updateField(QString)));
}

ParameterDialog::~ParameterDialog(){
    delete ui;
}

void ParameterDialog::setCurrentSequence() {
    currentSequence.clear();
    ui->currentList->clear();
    std::deque<ModuleInterface *>::iterator it, it_end = va->moduleSequence.end();
    QString cur_module;
    for(it = va->moduleSequence.begin(); it != it_end; it++) {
        currentSequence.push_back((*it)->name);
        cur_module = (*it)->name.c_str();
        new QListWidgetItem(cur_module, ui->currentList);
    }
}

//al inicializar se actualiza la lista de modulos que se esta ejecutando.
void ParameterDialog::init(){
    setCurrentSequence();
}

std::deque<parameter> ParameterDialog::children(parameter &parent){
    std::deque<parameter> list;
    if(parent.subParam.empty()){
        list.push_back(parent);
        return list;
    }

    std::multimap<QString, parameter>::iterator it, it_end = parent.subParam.end();
    for(it = parent.subParam.begin(); it != it_end; it++){
        std::deque<parameter> aux;
        parameter &temp = it->second;
        aux = children(temp);
        std::deque<parameter>::iterator it, it_end = aux.end();
        for(it = aux.begin(); it != it_end; it++)
            list.push_back(*it);
    }

    list.push_back(parent);
    return list;
}

std::deque<parameter> ParameterDialog::getParameters(int index){
    std::deque<parameter>::iterator it, it_end = va->moduleSequence[index]->listParameters.end();
    std::deque<parameter> list;
    list.clear();
    for(it = va->moduleSequence[index]->listParameters.begin(); it != it_end; it++){
        std::deque<parameter> aux = children(*it);
        std::deque<parameter>::iterator it, it_end = aux.end();
        for(it = aux.begin(); it != it_end; it++)
            list.push_back(*it);
    }
    return list;
}

//al seleccionar un modulo, se crean las etiquetas y los campos editables de los parametros de dicho modulo.
void ParameterDialog::updateField(QString module){
    ui->label->setText("Parameters of "+module+":");
    int index = ui->currentList->currentRow();
    clearLists();
    if(index < 0 )
        return;
    std::deque<parameter> parametros;
    parametros = getParameters(index);
    int j=0;
    //Se revisa la lista de parametros del module seleccionado.
    for(int i = 0; i < parametros.size(); i++){
        //por cada parametro se crea una etiqueta y su campo editor de dato dependiendo de la tipo del parametro.
        QLabel *labelTemp = new QLabel(ui->scrollAreaWidgetContents);
        parameter &p = parametros[i];
        labelTemp->setText(p.name);
        if( p.type == "int"){
            QSpinBox *lineEditTemp = new QSpinBox(ui->scrollAreaWidgetContents);
            lineEditTemp->setMaximum((va->moduleSequence[index]->listParameters[i].value.toInt())+1000);
            lineEditTemp->setValue(va->moduleSequence[index]->listParameters[i].value.toInt());
            lineEditTemp->setObjectName(va->moduleSequence[index]->listParameters[i].name);
            lineEditTemp->show();
            lineEditTemp->setGeometry(QRect(160,30+j,113,20));
            valueSB.push_back(lineEditTemp);
        }
        else if( p.type == "QString"){
            QLineEdit *lineEditTemp = new QLineEdit(ui->scrollAreaWidgetContents);
            lineEditTemp->setText(p.value);
            lineEditTemp->setObjectName(p.name);
            lineEditTemp->show();
            lineEditTemp->setGeometry(QRect(160,30+j,113,20));
            valueL.push_back(lineEditTemp);
        }
        else if( p.type == "double"){
            QDoubleSpinBox * lineEditTemp = new QDoubleSpinBox(ui->scrollAreaWidgetContents);
            lineEditTemp->setMaximum(p.value.toInt()+1000);
            lineEditTemp->setValue(p.value.toDouble());
            lineEditTemp->setObjectName(p.name);
            lineEditTemp->show();
            lineEditTemp->setGeometry(QRect(160,30+j,113,20));
            valueDSB.push_back(lineEditTemp);
        }
        else {
            QLineEdit *lineEditTemp = new QLineEdit(ui->scrollAreaWidgetContents);
            lineEditTemp->setText(p.value);
            lineEditTemp->setObjectName(p.name);
            lineEditTemp->show();
            lineEditTemp->setGeometry(QRect(160,30+j,113,20));
            valueL.push_back(lineEditTemp);
        }
        labelTemp->show();
        labelTemp->setGeometry(QRect(10,30+j,(labelTemp->text().size())*7,13));
        nameParam.push_back(labelTemp);
        j+=30;
    }
    ui->scrollAreaWidgetContents->setGeometry(ui->scrollAreaWidgetContents->x(), ui->scrollAreaWidgetContents->y(), ui->scrollAreaWidgetContents->width(), j+40);
}


//Se limpian las listas de etiquetas y campos editables.
void ParameterDialog::clearLists(){
    std::deque<QSpinBox *>::iterator it_SB, it_SB_end = valueSB.end();
    std::deque<QDoubleSpinBox*>::iterator it_DSB, it_DSB_end = valueDSB.end();
    std::deque<QLineEdit*>::iterator it_L, it_L_end = valueL.end();
    std::deque<QLabel *>::iterator it_label, it_label_end = nameParam.end();

    if(valueSB.size()>0){
        for(it_SB = valueSB.begin(); it_SB != it_SB_end; it_SB++)
            delete (*it_SB);
        valueSB.clear();
    }
    if(valueDSB.size()>0){
        for(it_DSB = valueDSB.begin(); it_DSB != it_DSB_end; it_DSB++)
            delete (*it_DSB);
        valueDSB.clear();
    }
    if(valueL.size()>0){
        for(it_L = valueL.begin(); it_L != it_L_end; it_L++)
            delete (*it_L);
        valueL.clear();
    }
    if(nameParam.size()>0){
        for(it_label = nameParam.begin(); it_label != it_label_end; it_label++)
            delete (*it_label);
        nameParam.clear();
    }
}

void ParameterDialog::modifyParameters(QString param){

}

//al apretar el boton reset, vuelve a cargar los parametros del modulo seleccionado.
void ParameterDialog::on_resetButton_clicked(){
    updateField(ui->currentList->currentItem()->text());
}

//al apretar el boton guardar, la lista de parametros del modulo seleccionado se actualiza.
void ParameterDialog::on_guardarButton_clicked(){
    QString module = ui->currentList->currentItem()->text();
    int index = ui->currentList->currentRow();
    std::deque<QLabel *>::iterator it_line, it_line_end = nameParam.end();
    std::deque<QSpinBox *>::iterator it_SB = valueSB.begin();
    std::deque<QDoubleSpinBox*>::iterator it_DSB = valueDSB.begin();
    std::deque<QLineEdit*>::iterator it_L = valueL.begin();
    std::deque<parameter> parametros = getParameters(index);
    int i = 0;
    parameter &p = parametros[i];
    for(it_line = nameParam.begin(); it_line != it_line_end; it_line++){
        if( p.type == "int"){
            p.value = QString::number((*it_SB++)->value());
        }
        else if(p.type == "QString"){
            p.value = (*it_L++)->text();
        }
        else if( p.type == "double"){
            p.value = QString::number((*it_DSB++)->value());
        }
        else {
            p.value = (*it_L++)->text();
        }
        i++;
    }
    QMessageBox::information(this, tr("Save"),
                             tr("%1\'s parameters were successfully stored.").arg(module));
}

void ParameterDialog::on_buttonBox_accepted(){
    std::deque<ModuleInterface *>::iterator it, it_end = va->moduleSequence.end();
    for(it = va->moduleSequence.begin(); it != it_end; it++) {
        //Se actualizan los parametros cuando no se esten leyendo.
        paramMutex.lock();
            (*it)->updateParameters();
        paramMutex.unlock();
    }
}

QString ParameterDialog::writeXML(parameter &parent){
    QString text;

    if( parent.subParam.empty()){
        text = "<"+parent.name + " value=\""+parent.value+"\"/>\n";
        return text;
    }
    std::multimap<QString, parameter>::iterator it, it_end = parent.subParam.end();
    text += "<"+parent.name + " value=\""+parent.value+"\" >\n";
    for(it = parent.subParam.begin(); it != it_end; it++){
        text += writeXML((*it).second);
    }
    text += "</"+parent.name + " >\n";
    return text;

}

void ParameterDialog::on_createButton_clicked(){
    QString fileName = QFileDialog::getSaveFileName(this, "Save Config file", "./", ("xml (*.xml);; All Files (*.*)"));
    QFile file(fileName);
    QString text;
    if(file.open(QIODevice::WriteOnly|QIODevice::Text)){
        text = "<!DOCTYPE MODULE_PARAMETERS>\n<MODULE_PARAMETERS>\n";
        std::deque<ModuleInterface *>::iterator it, it_end = va->moduleSequence.end();

        for(it = va->moduleSequence.begin(); it != it_end; it++) {
            QString moduleName = (*it)->name.c_str();
            text += "<"+moduleName+">\n";
            std::deque<parameter>::iterator it_m, it_m_end = (*it)->listParameters.end();
            for(it_m = (*it)->listParameters.begin(); it_m != it_m_end ; it_m++)
                text += writeXML(*it_m);
            text += "</"+moduleName+">\n";
        }
        text += "</MODULE_PARAMETERS>";
#ifdef __COMPILE_QT5__
        file.write(text.toLatin1());
#else
        file.write(text.toAscii());
#endif
        file.close();
    }
}
