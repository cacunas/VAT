#include "ObjectFilteringModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <vector>
#include <src/blob.h>

ObjectFilteringModule::ObjectFilteringModule(Datapool *i_data): ModuleInterface(i_data) {}

ObjectFilteringModule::~ObjectFilteringModule() {}

bool ObjectFilteringModule::setParameters(QDomNode& config){
    QDomNode n;
    //int intT;

    if(config.isNull()) { //Parameter set for module not defined
        m_minArea = 900;
        m_minHeight = 30;
        m_minWidth = 30;
        m_minNumFrames = 3;
        m_minNumFramesReliability = 8;
        m_minReliability = 0.3;
        m_minObjects = 1;
        m_maxObjects = 10;
        AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry ObjectFilteringModule. Taking defaults...");
    } else {
        if( ( n = XmlCommon::getParameterNode("MinArea", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinArea' for ObjectFilteringModule. Taking default value (900).");
            m_minArea = 900;
        } else
            m_minArea = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinHeight", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinHeight' for ObjectFilteringModule. Taking default value (30).");
            m_minHeight = 30;
        } else
            m_minHeight = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinWidth", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinWidth' for ObjectFilteringModule. Taking default value (30).");
            m_minWidth = 30;
        } else
            m_minWidth = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinNumFrames", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinNumFrames' for ObjectFilteringModule. Taking default value (3).");
            m_minNumFrames = 3;
        } else
            m_minNumFrames = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinNumFramesReliability", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinNumFramesReliability' for ObjectFilteringModule. Taking default value (8).");
            m_minNumFramesReliability = 8;
        } else
            m_minNumFramesReliability = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MinReliability", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinReliability' for ObjectFilteringModule. Taking default value (0.3).");
            m_minReliability = 0.3;
        } else
            m_minReliability = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("MinObjects", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MinObjects' for ObjectFilteringModule. Taking default value (1).");
            m_minObjects = 1;
        } else
            m_minObjects = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("MaxObjects", config) ).isNull() ) {
            AppendToLog("ObjectFilteringModule: Warning: Configuration '" + m_data->m_config_file + "' file does not define an entry 'MaxObjects' for ObjectFilteringModule. Taking default value (10).");
            m_maxObjects = 10;
        } else
            m_maxObjects = XmlCommon::getParameterValue(n).toInt();

        if( !( n = XmlCommon::getParameterNode("FilterById", config) ).isNull() ) {
            bool value = XmlCommon::getParameterValue(n) == "true" ? true : false;
            if(value) {
                QDomNode m = n.firstChild();
                while( !m.isNull() ) {
                    QDomElement e = m.toElement();
                    if( !e.isNull() ) {
                        if( e.tagName() == "Id" )
                            m_ids.insert(XmlCommon::getParameterValue(e).toInt());
                    }
                    m = m.nextSibling();
                }
            }
        }
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("MinArea", QString::number(m_minArea), "int");
    addParameter("MinHeight", QString::number(m_minHeight), "int");
    addParameter("MinWidth", QString::number(m_minWidth), "int");
    addParameter("MinNumFrames", QString::number(m_minNumFrames), "int");
    addParameter("MinNumFramesReliability", QString::number(m_minNumFramesReliability), "int");
    addParameter("MinReliability", QString::number(m_minReliability), "double");
    addParameter("MinObjects", QString::number(m_minObjects), "int");
    addParameter("MaxObjects", QString::number(m_maxObjects), "int");
    return true;
}

bool ObjectFilteringModule::updateParameters(){
    parameter *mA, *mH, *mW, *mNF, *mNFR, *mR, *mO, *MO;
    mA = getParameter("MinArea");
    mH = getParameter("MinHeight");
    mW = getParameter("MinWidth");
    mNF = getParameter("MinNumFrames");
    mNFR = getParameter("MinNumFramesReliability");
    mR = getParameter("MinReliability");
    mO = getParameter("MinObjects");
    MO = getParameter("MaxObjects");

    if( mA == 0 || mH == 0 || mW == 0 || mNF == 0 || mNFR == 0 || mR == 0 || mO == 0 ||
            mA->value.toInt() < 0 || mH->value.toInt() < 0 || mW->value.toInt() < 0 ||
            mNF->value.toInt() < 0 || mNFR->value.toInt() < 0 ||
            mR->value.toDouble() < 0.0 || MO->value.toInt() <= 0 || mO->value.toInt() <= 0)
        return false;
    m_minArea = mA->value.toInt();
    m_minHeight = mH->value.toInt();
    m_minWidth = mW->value.toInt();
    m_minNumFrames = mNF->value.toInt();
    m_minNumFramesReliability = mNFR->value.toInt();
    m_minReliability = mR->value.toDouble();
    m_minObjects = mO->value.toInt();
    m_maxObjects = MO->value.toInt();
    return true;
}

bool ObjectFilteringModule::init() {
    return true;
}

bool ObjectFilteringModule::run(){

    AppendToLog("ObjectFilteringModule: Objects number before: " +  QString::number(m_data->RMMobjects.size()));

    //Check id constraints
    if(!m_data->RMMobjects.empty() && !m_ids.empty()) {
        std::deque<SpRMMMobileObject> new_objects1;
        SpRMMMobileObject obj;
        std::deque<SpRMMMobileObject>::iterator iter, it_end = m_data->RMMobjects.end();
        for(iter = m_data->RMMobjects.begin(); iter != it_end; iter++) {
            obj = *iter;
            if( m_ids.count(obj->mobile_id) > 0 )
                new_objects1.push_back(obj);
        }

        if(new_objects1.empty())
            m_data->RMMobjects.clear();
        else
            m_data->RMMobjects = new_objects1;
    }

    //Check bounding box size constraints
    if(m_data->RMMobjects.size() > m_minObjects && !m_data->RMMobjects.empty()) {
        std::deque<SpRMMMobileObject> new_objects1;
        SpRMMMobileObject obj;
        int i, numFrames[m_data->RMMobjects.size()];
        std::deque<SpRMMMobileObject>::iterator iter, it_end = m_data->RMMobjects.end();
        for(i=0, iter = m_data->RMMobjects.begin(); iter != it_end; iter++) {
            obj = *iter;
            if( objectOK(obj) ) {
                new_objects1.push_back(obj);
                numFrames[i++] = obj->numberOfFramesSinceFirstTimeSeen;
            }
        }

        //If not enough objects after filtering, get oldest
        if(new_objects1.size() < m_minObjects) {
            int index, inserted = 0, max;
            SpRMMMobileObject maxObj;
            std::deque<SpRMMMobileObject> new_objects2;
            while(inserted < m_minObjects) {
                max = 0;
                for(i=0, iter = m_data->RMMobjects.begin(); iter != it_end; i++, iter++) {
                    obj = *iter;
                    numFrames[i] = obj->numberOfFramesSinceFirstTimeSeen;
                    if(numFrames[i] > max) {
                        maxObj = obj;
                        index = i;
                    }
                }
                new_objects2.push_back(maxObj);
                numFrames[i] = 0; //not max now
                inserted++;
            }
            m_data->RMMobjects = new_objects2;
        } else
            m_data->RMMobjects = new_objects1;

        if(m_maxObjects > 0) {
            int n = m_data->RMMobjects.size();
            if(m_maxObjects < n) {
                int i, j, i_max;
                int frames[n];
                for(i=0,iter = m_data->RMMobjects.begin(); iter != m_data->RMMobjects.end(); iter++, i++)
                    frames[i] = (*iter)->numberOfFramesSinceFirstTimeSeen;

                std::deque<SpRMMMobileObject> new_objects;
                for(j=0;j<m_maxObjects;j++) {
                    int max = -1;
                    for(i=0; i<n; i++)
                        if(frames[i] > max) {
                            max = frames[i];
                            i_max = i;
                        }
                    new_objects.push_back(m_data->RMMobjects[i_max]);
                    frames[i_max] = -2;
                }
                m_data->RMMobjects = new_objects;
            }
        }
    }
    AppendToLog("ObjectFilteringModule: Objects number after: " +  QString::number(m_data->RMMobjects.size()));

    return true;
}

bool ObjectFilteringModule::objectOK(SpRMMMobileObject obj) {
    int w = (int)obj->multiModel.binterface.bbox.width,
        h = (int)obj->multiModel.binterface.bbox.height;
    //Dimensional conditions
    if(w < m_minWidth)
        return false;
    if(h < m_minHeight)
        return false;
    if(w*h < m_minArea)
        return false;

    if(obj->numberOfFramesSinceFirstTimeSeen < m_minNumFrames)
        return false;
    else if(obj->numberOfFramesSinceFirstTimeSeen < m_minNumFramesReliability) {
        if(obj->R < m_minReliability)
            return false;
    }

    return true;
}


