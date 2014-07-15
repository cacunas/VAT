#include "LightSourceModel.h"


LightSourceModel::LightSourceModel():
Blob2DFromBGSubstractionModel() { }

LightSourceModel::~LightSourceModel() { }


/*
void LightSourceModel::lightProb(){

//m_data->fgImage;m_data->rFgImage;


        int //w = result->width(),
            //h = result->height(),
            b1 = m_data->currentImage->bytesPerLine(),
            b2 = m_data->fgImage->bytesPerLine(),
            b3 = m_data->rFgImage->bytesPerLine();

        int w = m_data->fgImage->width(),
            h = m_data->fgImage->height();

        uchar *currentImageBits;
        //uchar *fgImageBits;

        currentImageBits = m_data->currentImage->bits();
        uchar *fgImageBits=m_data->fgImage->bits();
        uchar *rfgImageBits=m_data->rFgImage->bits();


        int i, j, k, aux;

        int width= attributes["W"].att.value;
        int height= attributes["H"].att.value;

        int iInit= attributes["Y"].att.value-height/2;
        int jInit= attributes["X"].att.value-width/2;

        int iEnd= attributes["Y"].att.value+height/2;
        int jEnd= attributes["X"].att.value+width/2;

        int hue;

        if(jEnd>=w)
            jEnd=w-1;
        if(iEnd>=h)
            iEnd=h-1;
        if(jInit<0)
            jInit=0;
        if(iInit<0)
            iInit=0;

        int auxInitasdf=width;
        auxInitasdf++;
        for (i = iInit; i <= iEnd; i++){
            for (k = jInit; k <= jEnd; k++){
        //for (i = 0; i < h; i++){
          //  for(j=0, k=0; k<w; j+=4, k++){
                j=k*4;
                aux = i*b2 + k;

                if(fgImageBits[aux]==255){
                    int analisis=aux;
                    int analisisi=i;
                    int analisisj=k;

                    int analisisValorBack=fgImageBits[aux];

                    int aux2 = i*b1 + j;
                    uchar pixelR=currentImageBits[aux2+2];
                    uchar pixelG=currentImageBits[aux2+1];
                    uchar pixelB=currentImageBits[aux2];

                    uchar max=pixelR;
                    uchar min=pixelR;
                    uchar minIndice=0;
                    uchar maxIndice=0;

                    if(pixelG>max){
                        max=pixelG;
                        maxIndice=1;
                    }
                    if(pixelG<min){
                        min=pixelG;
                        minIndice=1;
                    }
                    if(pixelB>max){
                        max=pixelB;
                        maxIndice=2;
                    }
                    if(pixelB<min){
                        min=pixelB;
                        minIndice=2;
                    }
                    if(max - min >15 && ((max+min)/2>00)){

                        if(maxIndice==0){
                            if(pixelG>pixelB){
                                hue=(60*(pixelG-pixelB)/(max-min));
                            }else{
                                hue=(60*(pixelG-pixelB)/(max-min)+360);
                        }



                        }else if(maxIndice==1){
                            hue=(60*(pixelB-pixelR)/(max-min)+120);
                        }else if(maxIndice==2){
                            hue=(60*(pixelR-pixelG)/(max-min)+240);
                        }


                        hue=hue/10;
                        Hues[hue]++;
                        pSize++;
                        //pixels.insert();
                        //pixels.insert(pixelR);
                        //pixels.insert(pixelG);
                        //pixels.insert(pixelB);
                    }else{
                        pDiscard++;

                    }
                }
            }
        }





}*/
