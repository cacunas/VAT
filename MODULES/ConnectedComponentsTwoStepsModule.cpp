#include "ConnectedComponentsTwoStepsModule.h"
#include "image_display.h"
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

ConnectedComponentsTwoStepsModule::ConnectedComponentsTwoStepsModule(Datapool *i_data): ModuleInterface(i_data) {}

ConnectedComponentsTwoStepsModule::~ConnectedComponentsTwoStepsModule() {}

bool ConnectedComponentsTwoStepsModule::setParameters(QDomNode& config){
    return true;
}

bool ConnectedComponentsTwoStepsModule::init(){
    return true;
}

bool ConnectedComponentsTwoStepsModule::updateParameters(){
    return true;
}

bool ConnectedComponentsTwoStepsModule::run(){


    if(m_data->fgImage != NULL) {
        m_data->blobs.clear();
        setLabel();
    }
    return true;
}

void ConnectedComponentsTwoStepsModule::updateConnections(int new_v, int old_v, std::map<int, int> &con) {

    std::map<int, int>::iterator it, last = con.end();
    for(it = con.begin(); it!=last; it++) {
        if((*it).first != old_v) {
           if((*it).second == old_v)
             updateConnections(new_v, (*it).first, con);
        }
    }

    con[old_v] = new_v;
}

void ConnectedComponentsTwoStepsModule::setLabel(){

    int w = m_data->fgImage->width();
    int h = m_data->fgImage->height();
    int i;
    int limit = w*h;
    int labelTemp;

    //minimo label que se puede ocupar
    int labelMin = 0;
    //matriz con la información de labeling
    int labelmatrix[limit];

    //conexiones entre píxeles
    std::map<int, int> conecPixels;
    std::map<int, int>::iterator it;

    memset(labelmatrix,0,w*h*sizeof(int));

    //cout << (0%10) << endl;
    //se obtiene un arreglo que tiene la información de cada píxel de la imegen front
    uchar *bits = m_data->fgImage->bits();

    //TWO-STEPS
    //FIRST STEP
    int j, k, l, x, y, label;

    for(i=0; i<limit; i++){
        if(bits[i] > 0){
           x = i%w;    //ancho
           y = i/w;    //alto

           //1. Find minimal neighbour connection
           labelTemp = 0;
           //Analysis of previous row neighbours
           if(y > 0) {
              j = y - 1;
              l = (x == w - 1) ? w - 1 : x + 1;
              for(k = x > 0 ? x - 1 : 0 ; k <= l; k++) {
                 label = labelmatrix[j*w + k];
                 if(label > 0) { //Pixel is mouvement
                    if((it = conecPixels.find(label)) != conecPixels.end()) //Label tiene conexion
                       label = (*it).second;
                    if(labelTemp == 0 || labelTemp > label)
                       labelTemp = label;
                 }
              }
           }
           //Analysis of previous columns
           if(x > 0) {
              label = labelmatrix[i - 1];
              if(label > 0) {
                 if((it = conecPixels.find(label)) != conecPixels.end()) //Label tiene conexion
                    label = (*it).second;
                 if(labelTemp == 0 || labelTemp > label)
                    labelTemp = label;
              }
           }

           //Update label and connections
           if(labelTemp == 0){
              labelmatrix[i] = labelMin++;
           } else {
              labelmatrix[i] = labelTemp;
              //Update lesser connections
              if(y > 0) {
                 j = y - 1;
                 l = (x == w - 1) ? w - 1 : x + 1;
                 for(k = x > 0 ? x - 1 : 0 ; k <= l; k++) {
                    label = labelmatrix[j*w + k];
                    if(label > 0 && labelTemp < label) //Pixel is mouvement && there is lesser
                       updateConnections(labelTemp, label, conecPixels); //actualizamos con temporal
                 }
              }
              //Analysis of previous columns
              if(x > 0) {
                 label = labelmatrix[i - 1];
                 if(label > 0 && labelTemp < label) //Pixel is mouvement && there is lesser
                    updateConnections(labelTemp, label, conecPixels); //actualizamos con temporal
              }
           }
       }
    }

    //SECOND STEP
    std::set<int> values;
    int validLabel;
    for(i=0; i<limit; i++){
        if(bits[i] > 0){
            label = labelmatrix[i];
            if(conecPixels.find(label) != conecPixels.end()) {
                labelmatrix[i] = validLabel = conecPixels[label];
                if(values.count(validLabel) == 0)
                    values.insert(validLabel);
            } else if(values.count(label) == 0)
                    values.insert(label);

        }
    }

    //Set datapool matrix
    if(m_data->labelMatrix == NULL)
        m_data->labelMatrix = new int[limit];
    memcpy(m_data->labelMatrix, &labelmatrix, limit*sizeof(int));

    //Blobs generation
    std::vector<Blob>& p_blobs = m_data->blobs;
    //Resize to new blobs
    p_blobs.clear();
    if(values.size() > 0) {
        p_blobs.reserve(values.size());
        int numBlobs = 0;
        //Label correspondences to vector positions
        std::map<int, Blob *>::iterator liter;
        std::map<int, Blob *> labelCor;

        Rectangle<int> *b;
        Blob B;

        for(y=0, i=0; y < h; y++)
            for(x=0; x < w; x++, i++){
                if( ( (label = labelmatrix[i]) > 0) ){
                    if( (liter = labelCor.find(label) ) == labelCor.end() ) {
                        B.setRectangle(x, x, y, y);
                        B.component_label = label;
                        p_blobs.push_back(B);
                        labelCor[label] = &p_blobs[numBlobs];
                        numBlobs++;

                    } else {
                        b = &(liter->second->bbox);
                        if( b->xleft > x)
                            b->xleft = x;
                        if( b->xright < x)
                            b->xright = x;
                        if( b->ytop > y)
                            b->ytop = y;
                        if( b->ybottom < y)
                            b->ybottom = y;
                    }
                }
            }

        //Set width and height
        std::vector<Blob>::iterator iter, last = p_blobs.end();
        for(i=0, iter = p_blobs.begin();iter != last; i++, iter++){
            B.id = i;
            b = &iter->bbox;
            RECT_WIDTH(b) = RECT_XRIGHT(b) - RECT_XLEFT(b) + 1;
            RECT_HEIGHT(b) = RECT_YBOTTOM(b) - RECT_YTOP(b) + 1;
        }
        labelCor.clear();
    }
    conecPixels.clear();
}

