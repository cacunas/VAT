#ifndef VIEWPORTQLABEL_H
#define VIEWPORTQLABEL_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include "ImageWindowDisplay.h"

class ViewportQLabel : public QLabel
{
    Q_OBJECT

public:
    ViewportQLabel(QWidget *parent = 0);
    ~ViewportQLabel();

    ImageWindowDisplay *display_window;
    QString label;

public slots:
    void free_window();

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);
//    void paintEvent(QPaintEvent *event);

};

#endif // VIEWPORTQLABEL_H
