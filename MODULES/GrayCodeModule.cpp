#include "GrayCodeModule.h"
#include "image_display.h"
#include "src/MathFunctions.h"
#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include <src/blob.h>
#include <map>
#include <set>
#include <QFile>
#include <QTextStream>

GrayCodeModule::GrayCodeModule(Datapool *i_data): ModuleInterface(i_data) {
    first = false;
}

GrayCodeModule::~GrayCodeModule() {}

bool GrayCodeModule::setParameters(QDomNode& config){
    return true;
}


bool GrayCodeModule::updateParameters(){
    return true;
}

bool GrayCodeModule::init(){
    if(first == false && m_data->currentImage != NULL) {
        if(m_data->grayImage != NULL)
            delete m_data->grayImage;

        m_data->grayImage = new QImage(m_data->currentImage->width(),
                                       m_data->currentImage->height(),
                                       QImage::Format_ARGB32);
        if(m_data->grayImage->isNull()) {
            AppendToLog("GrayCodeModule: Warning: Cannot create gray image from current image.\n");
            delete m_data->grayImage;
            m_data->grayImage = NULL;
        } else
            first = true;
    }

    return true;
}




bool GrayCodeModule::run(){

    if(m_data->currentImage == NULL || m_data->currentImage->isNull()) {
        AppendToLog("GrayCodeModule: Warning: No current image. Aborting execution...\n");
        return false;
    }

    if(first == false) {
        if(m_data->grayImage != NULL)
            delete m_data->grayImage;

        m_data->grayImage = new QImage(m_data->currentImage->width(),
                                       m_data->currentImage->height(),
                                       QImage::Format_ARGB32);
        if(m_data->grayImage->isNull()) {
            AppendToLog("GrayCodeModule: Warning: Cannot create gray image from current image.\n");
            delete m_data->grayImage;
            m_data->grayImage = NULL;
            return false;
        } else
            first = true;
    }



    if(    m_data->currentImage->width() != m_data->grayImage->width()
        || m_data->currentImage->height() != m_data->grayImage->height()) {
        delete m_data->grayImage;
        m_data->grayImage = new QImage(m_data->currentImage->width(),
                                       m_data->currentImage->height(),
                                       QImage::Format_ARGB32);
        if(m_data->grayImage->isNull()) {
            AppendToLog("GrayCodeModule: Warning: Cannot create gray image from current image.\n");
            delete m_data->grayImage;
            m_data->grayImage = NULL;
            return false;
        }
    }



    getGray(m_data->grayImage);

    return true;
}


uchar GrayCodeModule::getGrayCode(uchar &input) {
    uchar result = 0;

    result = input & BIT_8;
    if(( (input & BIT_8) >> 1) != (input & BIT_7))
        result |= BIT_7;
    if(( (input & BIT_7) >> 1) != (input & BIT_6))
        result |= BIT_6;
    if(( (input & BIT_6) >> 1) != (input & BIT_5))
        result |= BIT_5;
    if(( (input & BIT_5) >> 1) != (input & BIT_4))
        result |= BIT_4;
    if(( (input & BIT_4) >> 1) != (input & BIT_3))
        result |= BIT_3;
    if(( (input & BIT_3) >> 1) != (input & BIT_2))
        result |= BIT_2;
    if(( (input & BIT_2) >> 1) != (input & BIT_1))
        result |= BIT_1;
    return result;
}


void GrayCodeModule::getGray(QImage *result) {
    if(result == NULL || result->isNull())
        return;
    int w4 = 4*m_data->currentImage->width(),
        h = m_data->currentImage->height(),
        b = m_data->currentImage->bytesPerLine();

    uchar *rbits = result->bits(),
          *bits = m_data->currentImage->bits();
    int i, j, aux;

    memset(rbits, 255, h*b);

    for(i=0; i<h; i++) {
        for(j=0; j<w4; j+=4) {
            aux = i*b + j;
            //Blue
            rbits[aux  ] = getGrayCode(bits[aux  ]);
            //Green
            rbits[aux+1] = getGrayCode(bits[aux+1]);
            //Red
            rbits[aux+2] = getGrayCode(bits[aux+2]);
        }
    }
}

