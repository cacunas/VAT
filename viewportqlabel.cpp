#include "viewportqlabel.h"
#include "VideoAnalysis.h"
#include <QScrollArea>
#include <QtAlgorithms>

ViewportQLabel::ViewportQLabel(QWidget *parent):QLabel(parent) {
    setBackgroundRole(QPalette::Dark);
    display_window = NULL;
    label = QString();
}

ViewportQLabel::~ViewportQLabel() {
    if(display_window != NULL)
        delete display_window;
}

void ViewportQLabel::free_window() {
    if(display_window != NULL) {
        QObject::disconnect(display_window,SIGNAL(dialogClosed()),this,SLOT(free_window()));
        delete display_window;
        display_window = NULL;
    }
}


//void ViewportQLabel::paintEvent(QPaintEvent *event) {
//      QLabel::paintEvent(event);
//      QPainter painter(this);
//      painter.setPen(Qt::red);
//      painter.setBrush(Qt::CrossPattern);
//          Dibujar la area en la instancia Painter
//      painter.drawConvexPolygon(polygon);
//}

void ViewportQLabel::mouseDoubleClickEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton){
        const QPixmap *p = this->pixmap();
        if(p != NULL) {
            if(display_window == NULL) {
                display_window = new ImageWindowDisplay(label,0);
                QObject::connect(display_window,SIGNAL(dialogClosed()),this,SLOT(free_window()));
                display_window->show();
            } else {
                display_window->raise();
            }
            display_window->setPixmap(p);
        }
    }
}
