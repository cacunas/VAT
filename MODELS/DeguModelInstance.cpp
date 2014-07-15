#include "DeguModelInstance.h"
#include "Datapool.h"

DeguModelInstance::DeguModelInstance(Datapool *i_data):
Blob2DFromBGSubstractionModelInstance(i_data) { }

DeguModelInstance::~DeguModelInstance() { }

void DeguModelInstance::initAttributes() {
    attributes["W"] = AttributeDuplet();
    attributes["H"] = AttributeDuplet();
    attributes["X"] = AttributeDuplet();
    attributes["Y"] = AttributeDuplet();
}

void DeguModelInstance::printAttributes() {}

void DeguModelInstance::printGeneral() {}

//sets activation criteria for each model: reliability on input (distance, bad data),
//   needs (occlusion, priority),
//sets priority of models (hierarchy).
void DeguModelInstance::setParameters(QDomNode &i_parameters) {
}

//sets activation flags for each model, according to the current context
void DeguModelInstance::activate(Blob *roi) {
active = true; }

//updates every activated model
void DeguModelInstance::update(Blob *roi) {
    BoundingBox &r = roi->bbox;
    attributes["W"] = AttributeDuplet(r.width, 1.0);
    attributes["H"] = AttributeDuplet(r.height, 1.0);
    //Central point
    attributes["X"] = AttributeDuplet(RECT_XCENTER(&r), 1.0);
    attributes["Y"] = AttributeDuplet(RECT_YCENTER(&r), 1.0);




}


//sets global probability for the model (how good is the model)
void DeguModelInstance::setProbability() {
    P = 1.0;
}

//sets global reliability for the model (how good is the data used in the model)
void DeguModelInstance::setReliability() {
    R = 1.0;
}

//sets blob enclosing the model.
void DeguModelInstance::setBlobInterface() {
    double w = attributes["W"].value, h = attributes["H"].value;
    BoundingBox &r = binterface.bbox;
    r.initRectangle(attributes["X"].value-w/2, attributes["Y"].value-h/2, w, h);
}

bool DeguModelInstance::draw(QImage *image) {}

Blob DeguModelInstance::getBlobInterface() { return Blob(); }



void DeguModelInstance::makeHistoryGraph(){



        int //w = result->width(),
            //h = result->height(),
            b1 = m_data->currentImage->bytesPerLine(),
            b2 = m_data->fgImage->bytesPerLine(),
            b3 = m_data->rFgImage->bytesPerLine();

        uchar *currentImageBits = m_data->currentImage->bits();
        uchar *fgImageBits=m_data->fgImage->bits();
        uchar *rfgImageBits=m_data->rFgImage->bits();


        int i, j, k, aux;

        int width= attributes["W"].value;
        int height= attributes["H"].value;

        int iInit= attributes["Y"].value-height/2;
        int jInit= attributes["X"].value-width/2;

        int iEnd= attributes["Y"].value+height/2;
        int jEnd= attributes["X"].value+width/2;

        int hue;



        for (i = iInit; i <= iEnd; i++){
            for (j = jInit; j <= jEnd; j++){
                aux = i*b1 + j;
                if(fgImageBits[aux]==255){
                    uchar pixelR=currentImageBits[aux+2];
                    uchar pixelG=currentImageBits[aux+1];
                    uchar pixelB=currentImageBits[aux];

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
                    if(max - min >15){

                        if(maxIndice==0){
                            if(pixelG>pixelB){
                                hue=60*(pixelG-pixelB)/(max-min);
                            }else{
                                hue=60*(pixelG-pixelB)/(max-min)+360;
                        }



                        }else if(maxIndice==1){
                            hue=60*(pixelB-pixelR)/(max-min)+120;
                        }else if(maxIndice==2){
                            hue=60*(pixelR-pixelG)/(max-min)+240;
                        }

                        Hues[hue]++;

                        //pixels.insert();
                        //pixels.insert(pixelR);
                        //pixels.insert(pixelG);
                        //pixels.insert(pixelB);
                    }
                }
            }
        }





}


void DeguModelInstance::checkHistory(){

    int pSize=pixels.size();

    if(pSize>1500){
        //buscar maximo
        int max=0;
        int maxIndex=0;
        for (int i =0;i < 360; i++){

            int value=0;

            int k = i +10;
            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){
                    value += Hues[t];

                }

            }

            for(int j=i; j<k; j++){

                value += Hues[j];
            }

            if(value > max){
                max=value;
                maxIndex=i;
            }
        }
        //buscar lado derecho
        int indice = maxIndex;

        indice--;
        if(indice<0){
            indice=359;
        }
        while(indice!=maxIndex){

            int value=0;

            int k = indice +10;
            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){
                    value += Hues[t];

                }

            }

            for(int j=indice; j<k; j++){

                value += Hues[j];
            }

            if(value<max*0.2 ){

                break;
            }

            indice--;
            if(indice<0){
                indice=359;
            }
        }
        //buscar lado izquierdo
        int iColor= (indice+5)%360;


        indice = maxIndex;

        indice++;
        if(indice>359){
            indice=0;
        }

        while(indice!=maxIndex){

            int value=0;

            int k = indice +10;

            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){

                    value += Hues[t];

                }

            }

            for(int j=indice; j<k; j++){

                value += Hues[j];
            }

            if(value<max*0.2 ){

                break;
            }

            indice++;

            if(indice>359){
                indice=0;
            }
        }

        int dColor= (indice+5)%360;

        printf("maximo %d, derecha %d, izquierda %d ", maxIndex, iColor, dColor);
    }





}
