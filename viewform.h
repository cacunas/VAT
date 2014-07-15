#ifndef VIEWFORM_H
#define VIEWFORM_H

#include <QWidget>
#include <QMainWindow>
#include "viewportqlabel.h"

namespace Ui {
    class ViewForm;
}

class ViewForm : public QWidget
{
    Q_OBJECT

public:
    explicit ViewForm(QWidget *parent);
    ~ViewForm();
    ViewportQLabel *getViewport();
    void setLabel(QString label);
    QString getLabel();
    void setEnableSave(bool);
    void saveImage();
    void useImage(QImage *);

private slots:
  void on_saveView_clicked();

private:
    Ui::ViewForm *ui;
    QPixmap *m_image;
};

#endif // VIEWFORM_H
