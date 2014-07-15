#include "DeguObjectModule.h"
#include "image_display.h"
#include "src/hullmodel.h"
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

//cv::RNG rng(12345);

#define RSEG_DEBUG

DeguInstance::DeguInstance() {}

DeguInstance::~DeguInstance() {}


DeguObjectModule::DeguObjectModule(Datapool *i_data): ModuleInterface(i_data) {
}

DeguObjectModule::~DeguObjectModule() {}

bool DeguObjectModule::setParameters(QDomNode& config){
    QDomNode n, m;

    if(config.isNull()) { //Parameter set for module not defined
        DTolerance = 100;
    } else {
        if( !( n = XmlCommon::getParameterNode("FilterById", config) ).isNull() ) {
            bool value = XmlCommon::getParameterValue(n) == "true" ? true : false;
            if(value) {
                QDomNode m = n.firstChild();
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "Id" )
                            m_ids.push_back(XmlCommon::getParameterValue(e).toInt());
                    }
                    m = m.nextSibling();
                }
            }
        }


        //ARREGLAR con parametros!!!
        if( !( n = XmlCommon::getParameterNode("ToleranceInPixels", config) ).isNull() ) {
            DTolerance = XmlCommon::getParameterValue(n).toInt();
            if(DTolerance <= 0) {
                DTolerance = 100;
                AppendToLog("DeguObjectModule: Warning: 'ToleranceInPixels' not well defined (<0). Taking default: 100 [pixels]" );
            }
        } else {
            DTolerance = 100;
            AppendToLog("DeguObjectModule: Warning: 'ToleranceInPixels' not defined. Taking default: 100 [pixels]" );
        }

        addParameter("ToleranceInPixels", QString::number(DTolerance), "int");
    }

    return true;
}

bool DeguObjectModule::updateParameters(){
    return true;
}

bool DeguObjectModule::init(){

    m_data->degu = new DeguModel(5);
    foundBefore = false;

    return true;
}

//Formato ARGB
//b == bits[i]    g == bits[i+1]    r == bits[i+2]    alpha == bits[i+3]
bool DeguObjectModule::run(){
    int size = m_data->RMMobjects.size();
    if(size > 0) { //Requires foreground and current images
        int i, the_one = -1;
        long oldest = -1;
        SpRMMMobileObject object;
        bool found = false;

        //First, if defined, use list and wait for a first appearance:
        if(m_ids.size() > 0) {
            SpRMMMobileObject obj;
            std::deque<SpRMMMobileObject>::iterator iter, it_end = m_data->RMMobjects.end();
            std::deque<int>::iterator iiter, iit_end = m_ids.end();
            for(i=0, iter = m_data->RMMobjects.begin(); iter != it_end; i++, iter++) {
                obj = *iter;
                for(iiter = m_ids.begin(); iiter != iit_end; iiter++) {
                    if(obj->mobile_id == *iiter) {
                        object = obj;
                        found = true;
                        the_one = i;
                        break;
                    }
                }
                if(found)
                    break;
            }


            if(!found && foundBefore) {
                double d, min_d = DBL_MAX;
                int xx, yy, x = RECT_XCENTER(&lastReliableRect), y = RECT_YCENTER(&lastReliableRect);
                for(i=0, iter = m_data->RMMobjects.begin(); iter != it_end; i++, iter++) {
                    if((*iter)->numberOfFramesNotSeen == 0) {
                        Rectangle<int> &r = (*iter)->iestimator;
                        xx = RECT_XCENTER(&r);
                        yy = RECT_YCENTER(&r);
                        d = (x-xx)*(x-xx) + (y-yy)*(y-yy);
                        if(d < min_d && d < DTolerance) {
                            object = (*iter);
                            min_d = d;
                            the_one = i;
                            found = true;
                        }
                    }
                }
            }

            //Update last reliable rectangle
            if(found) {
                if(object->numberOfFramesNotSeen == 0) {
                    lastReliableRect = object->iestimator;
                    foundBefore = true;
                }
            }
        } else {
            //Assuming just one degu present, search for the oldest,
            //not lost for long time, object:
            if(size == 1) {
                found = true;
                object = m_data->RMMobjects[0];
                the_one = 0;
            } else {
                for(i=0; i<size; i++) {
                    object = m_data->RMMobjects[i];
                    if(object->numberOfFramesNotSeen <= object->m_blobsBufferSize)
                        if(object->numberOfFramesSinceFirstTimeSeen > oldest) {
                            oldest = object->numberOfFramesSinceFirstTimeSeen;
                            the_one = i;
                        }
                }
                if(oldest == -1) //Every object is lost, take just the oldest
                    for(i=0; i<size; i++) {
                        object = m_data->RMMobjects[i];
                        if(object->numberOfFramesSinceFirstTimeSeen > oldest) {
                            oldest = object->numberOfFramesSinceFirstTimeSeen;
                            the_one = i;
                        }
                    }
                object = m_data->RMMobjects[the_one];
            }
        }

        if(found) {
            object->best_type = DEGU;

            if(m_data->hulls.size() > 0) { //Ideal case
                SpHullModel hull = m_data->hulls[the_one];
                if(hull->hull.size() > 0)
                    m_data->degu->insert(m_data, hull, object);
                else {
                    object->best_type = DEGU;
                    m_data->degu->move(m_data, object);
                }
            } else { //Object found but not the hull (move head as body)
                object->best_type = DEGU;
                m_data->degu->move(m_data, object);
            }
        } else { //Currently lost or not yet present (Do something?)


        }
    }/* else { //There is no object
        SpMobileObject2D object(new MobileObject2D());
        m_data->objects2D.push_back(object);
        object->best_type = DEGU;

        if(m_data->hulls.size() > 0) { //Initialize if coherent
        } else { //There is no hull
            //Keep data coherent if there is some
            m_data->degu->keep(m_data->frameNumber);
        }
    }*/
    return true;
}

