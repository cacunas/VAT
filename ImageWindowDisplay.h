#ifndef IMAGE_WINDOW_DISPLAY_H
#define IMAGE_WINDOW_DISPLAY_H

#include "ui_ImageWindowDisplay.h"

#ifdef __COMPILE_QT5__
    #include <QtWidgets/QMainWindow>
#else
    #include <QtGui/QMainWindow>
#endif

class ImageWindowDisplay: public QMainWindow
 {
     Q_OBJECT

 public:
     ImageWindowDisplay(QString name, QMainWindow *parent = 0);
     ~ImageWindowDisplay();
     void resizeEvent(QResizeEvent *event);
     void useImage(QImage *result);
     void closeEvent(QCloseEvent *event);
     QImage *getImage();
     void setPixmap(const QPixmap *p);
 private slots:
     void on_saveImage_clicked();

 signals:
     void dialogClosed();

 private:
     QImage *m_image;
     Ui::ImageWindowDisplay ui;
     int id;
 };

#endif
