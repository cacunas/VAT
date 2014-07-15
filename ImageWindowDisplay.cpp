#include <iostream>
#include <map>

#include <QFileDialog>
#include <QMessageBox>

#include "ImageWindowDisplay.h"

ImageWindowDisplay::ImageWindowDisplay(QString name, QMainWindow *parent):QMainWindow(parent),
                                                        m_image(NULL) {
  ui.setupUi(this);
  this->setObjectName(name);
#ifdef __COMPILE_QT5__
  this->setWindowTitle(QApplication::translate("MainWindow", name.toStdString().c_str(), 0));
#else
  this->setWindowTitle(QApplication::translate("MainWindow", name.toStdString().c_str(), 0, QApplication::UnicodeUTF8));
#endif
}

ImageWindowDisplay::~ImageWindowDisplay() {
    if(m_image != NULL)
        delete m_image;
}

void ImageWindowDisplay::setPixmap(const QPixmap *p) {
    ui.imageDisplay->setPixmap(*p);
    ui.imageDisplay->resize(p->size());
    ui.imageDisplay->repaint(0,0,p->width(),p->height());
}

void ImageWindowDisplay::closeEvent(QCloseEvent *event) {
  emit dialogClosed();
}

void ImageWindowDisplay::useImage(QImage *result) {
  m_image = new QImage(*result);
  ui.imageDisplay->setPixmap(QPixmap::fromImage(*m_image, Qt::AutoColor));
  ui.imageDisplay->resize(ui.imageDisplay->pixmap()->size());
}

void ImageWindowDisplay::on_saveImage_clicked() {

  if(m_image == NULL || m_image->isNull()) {
    QMessageBox::information(this, tr("Save Operator Result Failure"),
			     tr("Cannot save null image."));
    return;
  }

  QString fileName = QFileDialog::getSaveFileName(this,
	  tr("Save Image"), "./", tr("Image Files (*.png *.jpg *.bmp *.ppm);; All files (*.*)"));

  if (fileName == "")
    return;
  if(m_image->save(fileName))
    QMessageBox::information(this, tr("Image Successfully Saved!"),
			     tr("Image saved in file %1.").arg(fileName));
  else
    QMessageBox::information(this, tr("Error Saving Image!"),
			     tr("Image could not be saved. Check if selected format is valid (extension)."));


}

void ImageWindowDisplay::resizeEvent(QResizeEvent *event) {
  ui.scrollArea->resize(this->width() - 10, this->height() - 40);
}


