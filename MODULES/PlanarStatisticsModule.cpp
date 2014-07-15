#include "PlanarStatisticsModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <vector>
#include <src/blob.h>

PlanarStatisticsModule::PlanarStatisticsModule(Datapool *i_data): ModuleInterface(i_data) {
    p = new PlanarStatistics(m_data);
}

PlanarStatisticsModule::~PlanarStatisticsModule() {
    delete p;
}

bool PlanarStatisticsModule::setParameters(QDomNode& config) {
    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    /*addParameter("MinArea", QString::number(m_minArea), "int");
    addParameter("MinHeight", QString::number(m_minHeight), "int");
    addParameter("MinWidth", QString::number(m_minWidth), "int");
    addParameter("MinNumFrames", QString::number(m_minNumFrames), "int");
    addParameter("MinNumFramesReliability", QString::number(m_minNumFramesReliability), "int");
    addParameter("MinReliability", QString::number(m_minReliability), "double");
    addParameter("MaxObjects", QString::number(m_maxObjects), "int");*/
    return p->setParameters(config);
}

bool PlanarStatisticsModule::updateParameters(){
/*    parameter *mA, *mH, *mW, *mNF, *mNFR, *mR, *mO;
    mA = getParameter("MinArea");
    mH = getParameter("MinHeight");
    mW = getParameter("MinWidth");
    mNF = getParameter("MinNumFrames");
    mNFR = getParameter("MinNumFramesReliability");
    mR = getParameter("MinReliability");
    mO = getParameter("MaxObjects");

    if( mA == 0 || mH == 0 || mW == 0 || mNF == 0 || mNFR == 0 || mR == 0 || mO == 0 ||
            mA->value.toInt() < 0 || mH->value.toInt() < 0 || mW->value.toInt() < 0 ||
            mNF->value.toInt() < 0 || mNFR->value.toInt() < 0 ||
            mR->value.toDouble() < 0.0 || mO->value.toInt() < 0)
        return false;
    m_minArea = mA->value.toInt();
    m_minHeight = mH->value.toInt();
    m_minWidth = mW->value.toInt();
    m_minNumFrames = mNF->value.toInt();
    m_minNumFramesReliability = mNFR->value.toInt();
    m_minReliability = mR->value.toDouble();
    m_maxObjects = mO->value.toInt();*/
    return true;
}

bool PlanarStatisticsModule::init() {
    p->init();

    return true;
}

bool PlanarStatisticsModule::run(){

    p->run();

    return true;
}

