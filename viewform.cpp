#include "viewform.h"
#include "ui_viewform.h"
#include <QFileDialog>
#include <QMessageBox>

ViewForm::ViewForm(QWidget *parent) : QWidget(parent){
    ui = new Ui::ViewForm;
    ui->setupUi(this);
    m_image = NULL;
    ui->saveView->setEnabled(false);
    ui->label->setText(QString());
}

ViewForm::~ViewForm()
{
    if(ui != NULL)
        delete ui;
    if(m_image != NULL)
        delete m_image;
}

ViewportQLabel *ViewForm::getViewport() {
    return ui->viewport;
}

void ViewForm::setEnableSave(bool value) {
    ui->saveView->setEnabled(value);
}

void ViewForm::setLabel(QString label) {
//    if(ui != NULL && ui->label != NULL)
    ui->label->setText(label);
    ui->viewport->label = label;
}

QString ViewForm::getLabel() {
    return ui->label->text();
}

void ViewForm::on_saveView_clicked() {
    saveImage();
}

void ViewForm::useImage(QImage *result){
    if(result == NULL)
        return;
    if(m_image == NULL)
        m_image = new QPixmap(result->width(),result->height());
    else if(    m_image->width() != result->width()
             || m_image->height() != result->height()) {
        delete m_image;
        m_image = new QPixmap(result->width(),result->height());
    }

    *m_image = QPixmap::fromImage(*result, Qt::AutoColor);
    ui->viewport->setPixmap(*m_image);
    ui->viewport->resize(m_image->size());
    ui->viewport->repaint(0,0,m_image->width(),m_image->height());
    if(ui->viewport->display_window != NULL)
        ui->viewport->display_window->setPixmap(m_image);
}


void ViewForm::saveImage() {

  QString fileName = QFileDialog::getSaveFileName(this,
          "Save Image from " + ui->label->text(), "./", tr("Image Files (*.png *.jpg *.bmp *.ppm);; All files (*.*)"));

  if (fileName == "")
    return;

  QImage im = ui->viewport->pixmap()->toImage();

  if(im.save(fileName))
    QMessageBox::information(this, tr("Image Successfully Saved!"),
                             tr("Image saved in file %1.").arg(fileName));
  else
    QMessageBox::information(this, tr("Error Saving Image!"),
                             tr("Image could not be saved. Check if selected format is valid (extension)."));
}

