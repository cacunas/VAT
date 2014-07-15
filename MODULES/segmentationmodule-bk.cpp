#include "segmentationmodule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include <blob.h>
#include <map>


using namespace std;

segmentationModule::segmentationModule(Datapool *i_data): m_data(i_data) {}
bool segmentationModule::setParameters(std::map<std::string, std::map<std::string, std::string> >* list){
    // Esto hay que obternerlo desde default_config.txt
    alpha = 0.1;
    T = 50;
    return true;
}

bool segmentationModule::init(){
    return true;
}

bool segmentationModule::run(){

    //Set background image:
    if(m_data->bgImage == NULL)
        m_data->bgImage = new QImage(*m_data->currentImage);

    //Set forward image
    if(m_data->fgImage == NULL)
        m_data->fgImage = new QImage(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_ARGB32);

    m_data->fgImage = m_data->currentImage;
    setGrayScaleTable();
    m_data->fgImage = this->getDiff();

    m_data->blobs.clear();

    setLabel();

    return true;
}

void segmentationModule::setLabel(){

    int w = m_data->fgImage->width();
    int h = m_data->fgImage->height();
    int i,j;
    int limit = w*h;
    int labelTemp;

    //minimo label que se puede ocupar
    int labelMin = 0;
    //matriz con la información de labeling
    int labelmatrix[h][w];

    //conexiones entre píxeles
    std::map<int, int> conecPixels;
    std::map<int, int>::iterator it;

    for(i=0; i<h; i++)
        for(j=0; j<w; j++)
            labelmatrix[i][j]=0;

    //cout << (0%10) << endl;
    //se obtiene un arreglo que tiene la información de cada píxel de la imegen front
    uchar *bits = m_data->fgImage->bits();

    //TWO-PASS
    //PRIMERO PASO

    for(i=0; i<limit; i++){
        //labelmatrix[i/w][i%w] = 0;
        //cout << (i/w) << endl;
        if(bits[i] > 0){
            int x = i/w;    //alto
            int y = i%w;    //ancho

            labelTemp = 0;

            if( (x-1>=0) && (y-1>=0) && (labelmatrix[x-1][y-1]>0) ){
                if(labelTemp > labelmatrix[x-1][y-1]){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x-1][y-1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   //it->second = labelmatrix[x-1][y-1];
                                    conecPixels[it->first] = labelmatrix[x-1][y-1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x-1][y-1];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x-1][y-1];
                }
                else if(labelTemp==0)
                     labelTemp = labelmatrix[x-1][y-1];
            }

            if( (x-1>=0) && (labelmatrix[x-1][y]>0) ){
                if((labelTemp > labelmatrix[x-1][y])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x-1][y]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x-1][y];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x-1][y];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x-1][y];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x-1][y];
            }

            if( (x-1>=0) && (y+1<w) && (labelmatrix[x-1][y+1]>0) ){
                if((labelTemp > labelmatrix[x-1][y+1])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x-1][y+1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x-1][y+1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x-1][y+1];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x-1][y+1];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x-1][y+1];
            }

            if( (y-1>=0) && (labelmatrix[x][y-1]>0) ){
                if((labelTemp > labelmatrix[x][y-1])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x][y-1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x][y-1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x][y-1];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x][y-1];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x][y-1];
            }

            if( (y+1<w) && (labelmatrix[x][y+1]>0) ){
                if((labelTemp > labelmatrix[x][y+1])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x][y+1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x][y+1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x][y+1];
                    }
                    // Actualiza el valor de labelTemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x][y+1];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x][y+1];
            }

            if( (x+1<h) && (y-1>=0) && (labelmatrix[x+1][y-1]>0) ){
                if((labelTemp > labelmatrix[x+1][y-1])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x+1][y-1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x+1][y-1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x+1][y-1];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x+1][y-1];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x+1][y-1];
            }

            if( (x+1<h) && (labelmatrix[x+1][y]>0) ){
                if((labelTemp > labelmatrix[x+1][y])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x+1][y]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x+1][y];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x+1][y];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x+1][y];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x+1][y];
            }

            if( (x+1<h) && (y+1<w) && (labelmatrix[x+1][y+1]>0) ){
                if((labelTemp > labelmatrix[x+1][y+1])){
                    // Aca se busca la conexion ...
                    if(conecPixels.find(labelTemp) != conecPixels.end()){
                        if(conecPixels[labelTemp]>labelmatrix[x+1][y+1]){
                            it = conecPixels.begin();
                            while(it != conecPixels.end()){
                                if(it->second == conecPixels[labelTemp])
                                   conecPixels[it->first] = labelmatrix[x+1][y+1];
                                it++;
                            }
                        }
                    }else{
                        // Entra a este else, ya que no existia la conexion...
                        // Se crea la conexion entre los pixeles...
                        conecPixels[labelTemp] = labelmatrix[x+1][y+1];
                    }
                    // Actualiza el valor de labeltemp al del vecino, ya que
                    // su label es menor
                    labelTemp = labelmatrix[x+1][y+1];
                }
                else if(labelTemp==0)
                    labelTemp = labelmatrix[x+1][y+1];
            }

            if(labelTemp == 0){
                labelmatrix[x][y] = labelMin++;
            } else
                labelmatrix[x][y] = labelTemp;


            //std::cout << labelmatrix[x][y] << std::endl;
        }

    }

    //SEGUNDO PASO
    for(i=0; i<limit; i++){
        if(bits[i] > 0){
            int x = i/w;
            int y = i%w;
            if( (labelmatrix[x][y] > 0) )
                if(conecPixels.find(labelmatrix[x][y]) != conecPixels.end())
                    labelmatrix[x][y] = conecPixels[labelmatrix[x][y]];
        }
    }


    // Generacion de Blobs
    map<int,Bleb>blebs;
    map<int,Bleb>::iterator iter;

    for(int x=0; x<h; x++)
        for(int y=0; y<w; y++){
            if( (labelmatrix[x][y] > 0) ){
                if( (iter=blebs.find(labelmatrix[x][y])) == blebs.end() ){
                    Bleb b;
                    b.xmin = x;
                    b.ymin = y;
                    b.xmax = x;
                    b.ymax = y;
                    blebs.insert(pair<int,Bleb>(labelmatrix[x][y],b));
                }else{
                    Bleb b;
                    b = iter->second;
                    if( b.xmax < x )
                        b.xmax = x;
                    if( b.xmin > x )
                        b.xmin = x;
                    if( b.ymax < y )
                        b.ymax = y;
                    if( b.ymin > y )
                        b.ymin = y;
                    blebs[iter->first] = b;
                }
            }
        }

    iter = blebs.begin();
    while(iter != blebs.end()){
        Bleb b;
        b = iter->second;
        iter++;
        Blob bl;
        bl.Xu = b.xmin;
        bl.Yl = b.ymin;
        bl.H = b.xmax - b.xmin;
        bl.W = b.ymax - b.ymin;
        //if( (bl.H > 1) && (bl.W > 1))
            m_data->blobs.push_back(bl);
    }
    blebs.clear();
    conecPixels.clear();
}

QImage * segmentationModule::getDiff(){
    setGrayScaleTable();
    int w,h;
    w = m_data->currentImage->width();
    h = m_data->currentImage->height();


    QImage *m_result = new QImage(w, h, QImage::Format_Indexed8);
    m_result->setColorTable(*grayScaleTable);
    uchar d1, d2, d3;
    uchar
    *bits = m_result->bits(),
    *new_bits = m_data->currentImage->bits(),
    *bg_bits = m_data->bgImage->bits();

    int i,j,limit = w*h*4;
    for(i=0, j=0; i<limit; i+=4, j++) {
        d1 = abs(new_bits[i+2] - (uchar)(alpha * new_bits[i+2] + (1 - alpha) * bg_bits[i+2]));
        d2 = abs(new_bits[i+1] - (uchar)(alpha * new_bits[i+1] + (1 - alpha) * bg_bits[i+1]));
        d3 = abs(new_bits[i] - (uchar)(alpha * new_bits[i] + (1 - alpha) * bg_bits[i]));
        bits[j] = (d1>T || d2>T || d3>T) ? 255 : 0;
    }
    return m_result;
}


void segmentationModule::setGrayScaleTable() {
  int i;
  grayScaleTable = new QVector<QRgb>(256);

  for(i=0;i<255;i++)
    (*grayScaleTable)[i] = qRgb(i,i,i);

}
