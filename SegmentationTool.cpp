#include "SegmentationTool.h"

void SegmentationTool::chromaticSegmentation(QImage * currentImage,QImage *result, QImage *result_c, QImage *maskImage,bool m_mask,int minGrade,int maxGrade, int minSaturation, int maxLight) {
    if(result == NULL)
        return;
    int w = result->width(),
        h = result->height(),
        b1 = currentImage->bytesPerLine(),
        b2 = result->bytesPerLine(),
        b3 = result_c->bytesPerLine();

    uchar *bits = result->bits(),
    *bits_c = result_c->bits(),
    *new_bits = currentImage->bits();
    int i, j, k, aux;

    uchar r,g,b;
    int maxIndice, minIndice;
    int max, min;
    float l;
    int hue;

    memset(bits, 0, h*b2);
    memset(bits_c, 0, h*b3);


    if(m_mask) {
        uchar *mask = maskImage->bits();
        for(i=0; i<h; i++) {
            for(j=0, k=0; k<w; j+=4, k++) {
                if(mask[i*b1 + k]) {


                    aux = i*b1 + j;
                    r = new_bits[aux + 2];
                    g = new_bits[aux + 1];
                    b = new_bits[aux];

                    maxIndice=0;
                    minIndice=0;

                    max=r;
                    min=r;

                    if(r<g){
                        maxIndice=1;
                        max=g;
                    }
                    if(r>g){
                        minIndice=1;
                        min=g;
                    }

                    if(max<b){
                        maxIndice=2;
                        max=b;
                    }
                    if(min>b){
                        minIndice=2;
                        min=b;
                    }

                    if(maxIndice==minIndice){

                    }else{
                        l=(float)((max+min)/(2));

                        if(max-min>minSaturation){
                            if(maxIndice==0){
                                if(g>b){
                                    hue=60*(g-b)/(max-min);
                                }else{
                                    hue=60*(g-b)/(max-min)+360;
                            }



                            }else if(maxIndice==1){
                                hue=60*(b-r)/(max-min)+120;
                            }else if(maxIndice==2){
                                hue=60*(r-g)/(max-min)+240;
                            }

                            if(minGrade>maxGrade){//se puede optimizar creando 4 casos, posibles, con o sin mascara y con los 2 casos posibles con los angulos
                                if((hue<maxGrade||hue>minGrade)&&l<maxLight){
                                    bits[i*b2 + k] = 255;
                                    memset(bits_c + i*b3 + 3*k, 255, 3);
                                }
                            }else{
                                if((hue<maxGrade && hue>minGrade)&&l<maxLight){
                                        bits[i*b2 + k] = 255;
                                        memset(bits_c + i*b3 + 3*k, 255, 3);

                                 }
                            }
                        }
                    }


                }
            }
        }
    } else {
        for(i=0; i<h; i++) {
            for(j=0, k=0; k<w; j+=4, k++) {

                aux = i*b1 + j;
                r = new_bits[aux + 2];
                g = new_bits[aux + 1];
                b = new_bits[aux];

                maxIndice=0;
                minIndice=0;

                max=r;
                min=r;

                if(r<g){
                    maxIndice=1;
                    max=g;
                }
                if(r>g){
                    minIndice=1;
                    min=g;
                }

                if(max<b){
                    maxIndice=2;
                    max=b;
                }
                if(min>b){
                    minIndice=2;
                    min=b;
                }


                if(maxIndice==minIndice){

                }else{
                    l=(float)((max+min)/(2));

                    if(max-min>minSaturation){
                        if(maxIndice==0){
                            if(g>b){
                                hue=60*(g-b)/(max-min);
                            }else{
                                hue=60*(g-b)/(max-min)+360;
                            }



                        }else if(maxIndice==1){
                            hue=60*(b-r)/(max-min)+120;
                        }else if(maxIndice==2){
                            hue=60*(r-g)/(max-min)+240;
                        }
                        if(minGrade>maxGrade){
                            if((hue<maxGrade || hue>minGrade) && l<maxLight){
                                bits[i*b2 + k] = 255;
                                memset(bits_c + i*b3 + 3*k, 255, 3);
                            }
                        }
                           else{
                            if((hue<maxGrade && hue>minGrade)&& l<maxLight){
                                bits[i*b2 + k] = 255;
                                memset(bits_c + i*b3 + 3*k, 255, 3);

                            }
                        }
                    }

                }
            }
        }
    }




}
