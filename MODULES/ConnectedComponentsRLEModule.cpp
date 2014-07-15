#include "ConnectedComponentsRLEModule.h"
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

ConnectedComponentsRLEModule::ConnectedComponentsRLEModule(Datapool *i_data): ModuleInterface(i_data) {}

ConnectedComponentsRLEModule::~ConnectedComponentsRLEModule() {}

bool ConnectedComponentsRLEModule::setParameters(QDomNode& config){
    QDomNode n;
    if(config.isNull()) { //Parameter set for module not defined
        step = 2;
        tolerance = 1;
    } else {
        if( !( n = XmlCommon::getParameterNode("Step", config) ).isNull() ) {
            if( (step = XmlCommon::getParameterValue(n).toInt()) <= 0) {
                step = 2;
                AppendToLog("ConnectedComponentsRLEModule: Warning: 'Step' must be higher than 0. Taking default: Step = " + QString::number(step) );
            }
        } else { //default
            step = 2;
            AppendToLog("ConnectedComponentsRLEModule: Warning: 'Step' not defined. Taking default: Step = " + QString::number(step) );
        }

        if( !( n = XmlCommon::getParameterNode("Tolerance", config) ).isNull() ) {
            if( (tolerance = XmlCommon::getParameterValue(n).toInt()) < 0) {
                tolerance = step-1;
                AppendToLog("ConnectedComponentsRLEModule: Warning: 'Tolerance' must be higher or equal to 0. Taking default: Tolerance = Step - 1 = " + QString::number(tolerance) );
            } else if(tolerance >= step) {
                tolerance = step-1;
                AppendToLog("ConnectedComponentsRLEModule: Warning: 'Tolerance' must be lower than 'Step'. Taking default: Tolerance = Step - 1 = " + QString::number(tolerance) );
            }
        } else { //default
            tolerance = step-1;
            AppendToLog("ConnectedComponentsRLEModule: Warning: 'Step' not defined. Taking default: Step = " + QString::number(step) );
        }

    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("Step", QString::number(step), "int");
    addParameter("Tolerance", QString::number(tolerance), "int");
    return true;
}

bool ConnectedComponentsRLEModule::updateParameters(){
    parameter *st, *to;
    st = getParameter("Step");
    to = getParameter("Tolerance");
    if( st == 0 || to == 0 || st->value.toInt() < 0 || to->value.toInt() < 0)
        return false;
    step = st->value.toInt();
    tolerance = to->value.toInt();
    return true;
}

bool ConnectedComponentsRLEModule::init(){
    return true;
}

bool ConnectedComponentsRLEModule::run(){


    if(m_data->fgImage != NULL) {
        m_data->blobs.clear();
        setLabel();
    }
    return true;
}

void ConnectedComponentsRLEModule::setLabel(){

    //QImage *prueba = new QImage("./test.png");
    //uchar *bits = prueba->bits();
    //int w = prueba->width(),
    //    h = prueba->height(),
    //    wc = prueba->bytesPerLine(),
    int w = m_data->fgImage->width(),
        h = m_data->fgImage->height(),
        wc = m_data->fgImage->bytesPerLine(),
        x, y, i, j;

    //minimo label que se puede ocupar
    int labelMin = 0;

    std::map<int, std::deque<RLESegment> > &RLEImage = m_data->RLEFGImage;
    std::map<int, RLERegion> &RLERegions = m_data->RLERegions;

    //se obtiene un arreglo que tiene la información de cada píxel de la imegen front
    uchar *bits = m_data->fgImage->bits();

    RLEImage.clear();
    RLERegions.clear();
    /*for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            std::cout << (bits[y*wc + x] ? 1 : 0) << " ";
        }
        std::cout << std::endl;
    }*/

    //FIRST STEP: Convert to RLE
    int start, tolcount;
    for(y=0; y<h; y++){
        for(x=step-1; x<w; ){
            if(bits[y*wc + x] > 0) {
                //First determine the starting point
                start = x;
                if(tolerance == 0) {
                    for(i=start-1; i>=0 && bits[y*wc + i]; i--);
                    start = i != 0 ? i + 1 : (bits[y*wc] ? 0 : 1);
                } else {
                    tolcount = 0;
                    j = start;
                    for(i=start-1; i>=0; i--)
                        if(bits[y*wc + i] != 0) {
                            tolcount = 0;
                            j=i;
                        } else {
                            tolcount++;
                            if(tolcount>tolerance)
                                break;
                        }
                    start = j;
                }
                RLESegment s;
                s.x = start;
                //Then, the length of the segment...

                if(tolerance == 0) {
                    for(i=1; x+i < w && bits[y*wc + x + i]; i++);
                    s.length = x - start + i;
                    x += i + 1;
                } else {
                    tolcount = 0;
                    j = 0;
                    for(i=1; i+x < w; i++)
                        if(bits[y*wc + x + i] != 0) {
                            tolcount = 0;
                            j = i;
                        } else {
                            tolcount++;
                            if(tolcount>tolerance)
                                break;
                        }
                    s.length = x - start + 1 + j;
                    x += j + 1 + tolerance;
                }
                RLEImage[y].push_back(s);
            } else //No movement
                x+=step;
        }
    }

    //SECOND STEP
    //Regions generation
    std::map<int, std::deque<RLESegment> >::iterator prev_it, row_it, row_end;
    std::deque<RLESegment>::iterator prev_col_it, col_it, col_end;
    int aux_label;

    if(!RLEImage.empty()) {
        //INIT WITH FIRST NON-EMPTY ROW AS THE BEGINNING OF EACH REGION:
        row_it = RLEImage.begin();
        row_end = RLEImage.end();
        col_end = (*row_it).second.end();
        for(col_it = (*row_it).second.begin(); col_it != col_end; col_it++) {
            RLERegion r(labelMin);
            RLERegions[labelMin] = r;
            RLERegions[labelMin++].insert((*row_it).first, *col_it);
        }

        //NEXT SEGMENTS COMPLETE PREVIOUS AND INIT NEW ONES
        for(prev_it=row_it, row_it++; row_it != row_end; prev_it=row_it, row_it++) {
            if(row_it->first - prev_it->first > tolerance + 1) { //All separated for sure, so just new ones
                col_end = (*row_it).second.end();
                for(col_it = (*row_it).second.begin(); col_it != col_end; col_it++) {
                    RLERegion r(labelMin);
                    RLERegions[labelMin] = r;
                    RLERegions[labelMin++].insert((*row_it).first, *col_it);
                }
            } else { //We have to check if some regions continue
                col_end = (*row_it).second.end();
                for(col_it = (*row_it).second.begin(), prev_col_it = (*prev_it).second.begin();
                    col_it != col_end; col_it++) {
                    //Advance position of previous segments
                    if(prev_col_it != (*prev_it).second.end())
                        while( (*col_it).x - ((*prev_col_it).x + (*prev_col_it).length - 1) > tolerance) {
                            prev_col_it++;
                            if(prev_col_it == (*prev_it).second.end())
                                break;
                        }
                    //Check coincident columns
                    if(prev_col_it != (*prev_it).second.end())
                        while( (*prev_col_it).x - ((*col_it).x + (*col_it).length - 1) <= tolerance + 1) {
                            if((*col_it).label < 0 || (*prev_col_it).label == (*col_it).label) { //First asignment
                                //plabel = (*prev_col_it).label;
                                //yy = (*row_it).first;
                                //RLESegment &s = *col_it;
                                //std::cerr << "y:" << yy << "; plabel: " << plabel << std::endl;
                                RLERegions[(*prev_col_it).label].insert((*row_it).first, *col_it);

                            } else { //Already assigned, update the previous one and merge regions
                                aux_label = (*prev_col_it).label;
                                //Update previous segments with current label
                                RLERegions[(*col_it).label].merge(RLERegions[aux_label]);
                                RLERegions.erase(aux_label);
                            }

                            //Si segmento previo sobrepasa el actual, avanza (HABIA ERROR prev_col_it por col_it)
                            if(((*prev_col_it).x + (*prev_col_it).length - 1) - ((*col_it).x + (*col_it).length - 1) <= tolerance)
                                prev_col_it++;
                            else
                                break;
                            if(prev_col_it == (*prev_it).second.end())
                                break;
                        }

                    if((*col_it).label < 0) { //No assignment yet: new region
                        RLERegion r(labelMin);
                        RLERegions[labelMin] = r;
                        RLERegions[labelMin++].insert((*row_it).first, *col_it);
                    }
                }
            }
        }
    }

    //Set datapool labels matrix
    if(m_data->labelMatrix == NULL)
        m_data->labelMatrix = new int[w*h];


    std::vector<Blob>& p_blobs = m_data->blobs;
    //Resize to new blobs
    p_blobs.clear();
    if(RLERegions.size() > 0) {
        p_blobs.reserve(RLERegions.size());
        //Label correspondences to vector positions
        std::map<int, RLERegion>::iterator riter, rend = RLERegions.end();
        Blob B;
        Rectangle<int> *b;
        std::map<int, std::deque<RLESegment *> >::iterator sit, sit_end;
        std::deque<RLESegment *>::iterator dit, dit_end;
        int rlen, start, startx, k, l;

        for(i=0, riter = RLERegions.begin(); riter != rend; i++, riter++) {
            b = & (riter->second.r);
            B.setRectangle(b->xleft, b->xright, b->ytop, b->ybottom);
            B.id = i;
            B.component_label = riter->first;
            p_blobs.push_back(B);

            //Build index matrix
            sit_end = riter->second.s.end();
            l = riter->second.label;

            for(sit = riter->second.s.begin(); sit != sit_end; sit++) {
                dit_end = sit->second.end();
                start = w*sit->first;
                for(dit = sit->second.begin(); dit != dit_end; dit++) {
                    startx = start + (*dit)->x;
                    rlen =(*dit)->length;
                    for(k=0; k<rlen; k++)
                        m_data->labelMatrix[startx+k] = l;
                }
            }
        }
    }
}

