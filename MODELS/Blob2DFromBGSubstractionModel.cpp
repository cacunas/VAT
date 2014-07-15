#include "Blob2DFromBGSubstractionModel.h"
#include "Datapool.h"
#include "MODELS/reliabilityattribute.h"
#include "xmlcommon.h"

#include <QDir>


Blob2DFromBGSubstractionModel::Blob2DFromBGSubstractionModel() {
    m_minimalAttributeVelocityReliability = 0.1;
    m_minimalAttributeAccelerationReliability = 0.1;
    m_minimalWidth = 5.0;
    m_minimalHeight = 5.0;
    m_pixelAcuity = 3;
}

Blob2DFromBGSubstractionModel::~Blob2DFromBGSubstractionModel() { }

void Blob2DFromBGSubstractionModel::initInstanceAttributes() {
    instanceTemplate.attributes["W"] = AttributeDupletValue();
    instanceTemplate.attributes["H"] = AttributeDupletValue();
    instanceTemplate.attributes["X"] = AttributeDupletValue();
    instanceTemplate.attributes["Y"] = AttributeDupletValue();
}

void Blob2DFromBGSubstractionModel::initDynamicsAttributes() {
    //Dynamics for W and H of blob will not consider acceleration
    dynamics.dynamics["W"] = ReliabilityDynamicsAttribute(true,true,false);
    dynamics.dynamics["H"] = ReliabilityDynamicsAttribute(true,true,false);
    dynamics.dynamics["X"] = ReliabilityDynamicsAttribute();
    dynamics.dynamics["Y"] = ReliabilityDynamicsAttribute();
}

void Blob2DFromBGSubstractionModel::init() { }

//sets activation flags for each model, according to the current context
void Blob2DFromBGSubstractionModel::activate(Blob *roi) {
active = true; }

//updates every activated model
void Blob2DFromBGSubstractionModel::updateInstance(Blob *roi) {
    BoundingBox &r = roi->bbox;
    double R = BLOB_R(roi);

    ModelInstanceDataPack m = instanceTemplate;

    m.attributes["W"] = AttributeDupletValue(r.width, R);
    m.attributes["H"] = AttributeDupletValue(r.height, R);
    //Central point
    m.attributes["X"] = AttributeDupletValue(RECT_XCENTER(&r), R);
    m.attributes["Y"] = AttributeDupletValue(RECT_YCENTER(&r), R);

    m.dp_type = roi->dp_type;

    instances.insert(m);
}

/*
//sets global probability for the model (how good is the model)
void Blob2DFromBGSubstractionModelInstance::setProbability() {
    P = 0.5;
}

//sets global reliability for the model (how good is the data used in the model)
void Blob2DFromBGSubstractionModelInstance::setReliability() {
    R = 0.5;
}*/

//sets blob enclosing the model.
/*void Blob2DFromBGSubstractionModelInstance::setBlobInterface() {
    double w = attributes["W"].v.value, h = attributes["H"].v.value;
    BoundingBox &r = binterface.bbox;
    r.initRectangle(attributes["X"].v.value-w/2, attributes["Y"].v.value-h/2, w, h);
}*/

bool Blob2DFromBGSubstractionModel::draw(QPainter &painter) {
    return draw(painter, 255, 255, 255);
}

bool Blob2DFromBGSubstractionModel::draw(QPainter &painter, int r, int g, int b) {

    std::map <QString, ReliabilityDynamicsAttribute> &attributes = dynamics.dynamics;

    int
        X   = (int) attributes["X"].att.value,
        Y   = (int) attributes["Y"].att.value,
        W_2 = (int) (attributes["W"].att.value / 2.0),
        H_2 = (int) (attributes["H"].att.value / 2.0);

    painter.setPen(QColor(r, g, b, 255));//white

    painter.drawLine(X - W_2, Y - H_2, X + W_2, Y - H_2);
    painter.drawLine(X + W_2, Y - H_2, X + W_2, Y + H_2);
    painter.drawLine(X + W_2, Y + H_2, X - W_2, Y + H_2);
    painter.drawLine(X - W_2, Y + H_2, X - W_2, Y - H_2);

    return true;
}




//sets activation criteria for each model: reliability on input (distance, bad data),
//   needs (occlusion, priority),
//sets priority of models (hierarchy).
void Blob2DFromBGSubstractionModel::setParameters(QDomNode &i_parameters) {

    if(i_parameters.isNull()) { //Parameter set for module not defined
        m_minimalAttributeVelocityReliability = 0.1;
        m_minimalAttributeAccelerationReliability = 0.1;
        m_minimalWidth = 5.0;
        m_minimalHeight = 5.0;
        m_pixelAcuity = 3;
    } else {
        QDomNode n;
        if( ( n = XmlCommon::getParameterNode("MinimalAttributeVelocityReliability", i_parameters) ).isNull() )
            m_minimalAttributeVelocityReliability = 0.1;
        else
            m_minimalAttributeVelocityReliability = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("MinimalAttributeAccelerationReliability", i_parameters) ).isNull() )
            m_minimalAttributeAccelerationReliability = 0.1;
        else
            m_minimalAttributeAccelerationReliability = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("MinimalWidth", i_parameters) ).isNull() )
            m_minimalWidth = 5;
        else
            m_minimalWidth = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("MinimalHeight", i_parameters) ).isNull() )
            m_minimalHeight = 5;
        else
            m_minimalHeight = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("PixelAcuity", i_parameters) ).isNull() )
            m_pixelAcuity = 3;
        else
            m_pixelAcuity = XmlCommon::getParameterValue(n).toInt();

    }

}



void Blob2DFromBGSubstractionModel::initAttribute(ReliabilityDynamicsAttribute &d, AttributeDupletValue &i) {
    d.att.value = i.value;
    d.att.VR.SD = 0.0;
    d.att.VR.RC = 1.0;
    d.att.VR.accR = d.att.VR.RD = i.RD;
    memset(&d.V, 0, sizeof(AttributeTripletValue));
    memset(&d.A, 0, sizeof(AttributeTripletValue));
    d.V.VR.RC = 0.0;
}

void Blob2DFromBGSubstractionModel::updateMinConstrainedAttribute(const QString &name, ReliabilityDynamicsAttribute &d, double min) {
    double tdiff = RMMMobileObject::secDiffSequence[0];
    bool *foundSupport = m_mobile->foundSupport;
    double currentCooling = m_mobile->coolingValue[1];
    double attcur, data, RData;
    AttributeDupletValue &acurrent = instances[0].attributes[name];
    double pdata, VData;
    int i;


    //First for ATTRIBUTE VALUE:

    //Get current estimation, according to model.
    if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability)
        attcur = d.att.value + d.V.value*tdiff + 0.5*d.A.value*tdiff*tdiff;
    else if(d.updateV  && d.V.VR.RC >= m_minimalAttributeVelocityReliability)
        attcur = d.att.value + d.V.value*tdiff;
    else
        attcur = d.att.value;

    if(foundSupport[0]) { //Use data if visual evidence found
        //Get current data
        data = acurrent.value;

        //RELIABILITY
        //Set data reliability:
#ifdef __RMM_NO_RELIABILITY_CODE__
        RData = 1.0;
#else
        RData = acurrent.RD;
#endif
        updateAttributeDynamics(data, RData, m_pixelAcuity, attcur, currentCooling, d.att);

    } else //The current element has not been found
        updateAttributeDynamicsNoData(attcur, m_pixelAcuity, currentCooling, d.att);

    if(d.att.value < min)
        d.att.value = min;

    //Update Attribute Velocity
    if(d.updateV) {
        //Get current estimation, according to model.
        if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability)
            attcur = d.V.value + d.A.value*tdiff;
        else
            attcur = d.V.value;

        if(foundSupport[0] && m_mobile->numberOfFound >=2) { //Use data if visual evidence found
            //Get current data pair
            for(i=1; i<RMMMobileObject::m_blobsBufferSize; i++)
                if(foundSupport[i])
                    break;
            AttributeDupletValue &aprevious = instances[i].attributes[name];
            pdata = aprevious.value;

            VData = (data - pdata) / m_mobile->secDiffToCurrent[i];

            //RELIABILITY
            //Set data reliability:
    #ifdef __RMM_NO_RELIABILITY_CODE__
            RData = 1.0;
    #else
            RData = (aprevious.RD + acurrent.RD)/2.0;
    #endif
            //Acuity: pixel_acuity/frame = pixel_acuity/tdiff pixels/sec
            updateAttributeDynamics(VData, RData, m_pixelAcuity/tdiff, attcur, currentCooling, d.V);

        } else { //The current element has not been found
            updateAttributeDynamicsNoData(attcur, m_pixelAcuity/tdiff, currentCooling, d.V);
            VData = 0.0;
        }

        if(d.updateA) {
            //Get current estimation, according to model.
            attcur = d.A.value;

            if(foundSupport[0] && m_mobile->numberOfFound >= 3) { //Use data if visual evidence found
                double ppdata, VData2, AData, RAData, RData2;
                //Get current data pair
                int j;
                for(j=i+1; j<RMMMobileObject::m_blobsBufferSize; j++)
                    if(foundSupport[j])
                        break;

                AttributeDupletValue &aprevious = instances[i].attributes[name];
                pdata = aprevious.value;
                AttributeDupletValue &aprevious2 = instances[j].attributes[name];
                ppdata = aprevious2.value;

                VData2 = (pdata - ppdata) / (m_mobile->secDiffToCurrent[j] - m_mobile->secDiffToCurrent[i]);

                AData = (VData - VData2) / m_mobile->secDiffToCurrent[i];

                //RELIABILITY
                //Set data reliability:
    #ifdef __RMM_NO_RELIABILITY_CODE__
                RAData = 1.0;
    #else
                RData2 = (aprevious.RD + aprevious2.RD)/2.0;
                RAData = (RData + RData2) / 2.0;
    #endif
                //Acuity: 3 pixels/frame2 = 3/tdiff2 pixels/sec
                updateAttributeDynamics(AData, RAData, m_pixelAcuity/(tdiff*tdiff), attcur, currentCooling, d.A);

            } else //The current element has not been found
                updateAttributeDynamicsNoData(attcur, m_pixelAcuity/(tdiff*tdiff), currentCooling, d.A);
        } //A inside V
    }


}



void Blob2DFromBGSubstractionModel::updateAttribute(const QString &name, ReliabilityDynamicsAttribute &d) {
    double tdiff = RMMMobileObject::secDiffSequence[0];
    bool *foundSupport = m_mobile->foundSupport;
    double currentCooling = m_mobile->coolingValue[1];
    double attcur, data, RData;
    AttributeDupletValue &acurrent = instances[0].attributes[name];
    double pdata, VData;
    int i;


    //First for ATTRIBUTE VALUE:

    if(foundSupport[0]) { //If object found for this frame
        //Get current estimation, according to model.
        if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability)
            attcur = d.att.value + d.V.value*tdiff + 0.5*d.A.value*tdiff*tdiff;
        else if(d.updateV  && d.V.VR.RC >= m_minimalAttributeVelocityReliability)
            attcur = d.att.value + d.V.value*tdiff;
        else
            attcur = d.att.value;
    } else { //if not found:
        double auxA = d.A.value*tdiff*tdiff;
        //Get current estimation, according to model.
        if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability && d.att.VR.SD < auxA && m_mobile->numberOfFound >= 3)
            attcur = d.att.value + d.V.value*tdiff + 0.5*auxA;
        else if(d.updateV  && d.V.VR.RC >= m_minimalAttributeVelocityReliability && d.att.VR.SD < d.V.value*tdiff && m_mobile->numberOfFound >= 2)
            attcur = d.att.value + d.V.value*tdiff;
        else
            attcur = d.att.value;
    }

    if(foundSupport[0]) { //Use data if visual evidence found
        //Get current data
        data = acurrent.value;

        //RELIABILITY
        //Set data reliability:
#ifdef __RMM_NO_RELIABILITY_CODE__
        RData = 1.0;
#else
        RData = acurrent.RD;
#endif
        updateAttributeDynamics(data, RData, m_pixelAcuity, attcur, currentCooling, d.att);

    } else //The current element has not been found
        updateAttributeDynamicsNoData(attcur, m_pixelAcuity, currentCooling, d.att);


    //Update Attribute Velocity
    if(d.updateV) {

        if(foundSupport[0]) { //If object found for this frame
            //Get current estimation, according to model.
            if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability)
                attcur = d.V.value + d.A.value*tdiff;
            else
                attcur = d.V.value;
        } else { //if not found:
            //Get current estimation, according to model.
            if(d.updateA && d.A.VR.RC >= m_minimalAttributeAccelerationReliability && d.V.VR.SD < d.A.value*tdiff && m_mobile->numberOfFound >= 3)
                attcur = d.V.value + d.A.value*tdiff;
            else if(m_mobile->numberOfFound >= 2)
                attcur = d.V.value;
            else
                attcur = 0.0;
        }



        if(foundSupport[0] && m_mobile->numberOfFound >=2) { //Use data if visual evidence found
            //Get current data pair
            for(i=1; i<RMMMobileObject::m_blobsBufferSize; i++)
                if(foundSupport[i])
                    break;
            AttributeDupletValue &aprevious = instances[i].attributes[name];
            pdata = aprevious.value;

            VData = (data - pdata) / m_mobile->secDiffToCurrent[i];

            //RELIABILITY
            //Set data reliability:
    #ifdef __RMM_NO_RELIABILITY_CODE__
            RData = 1.0;
    #else
            RData = (aprevious.RD + acurrent.RD)/2.0;
    #endif
            //Acuity: pixel_acuity/frame = pixel_acuity/tdiff pixels/sec
            updateAttributeDynamics(VData, RData, m_pixelAcuity/tdiff, attcur, currentCooling, d.V);
        } else //The current element has not been found
            updateAttributeDynamicsNoData(attcur, m_pixelAcuity/tdiff, currentCooling, d.V);

        if(d.updateA) {
            //Get current estimation, according to model.
            attcur = d.A.value;

            if(foundSupport[0] && m_mobile->numberOfFound >= 3) { //Use data if visual evidence found
                double ppdata, VData2, AData, RAData, RData2;
                //Get current data pair
                int j;
                for(j=i+1; j<RMMMobileObject::m_blobsBufferSize; j++)
                    if(foundSupport[j])
                        break;

                AttributeDupletValue &aprevious = instances[i].attributes[name];
                pdata = aprevious.value;
                AttributeDupletValue &aprevious2 = instances[j].attributes[name];
                ppdata = aprevious2.value;

                VData2 = (pdata - ppdata) / (m_mobile->secDiffToCurrent[j] - m_mobile->secDiffToCurrent[i]);

                AData = (VData - VData2) / m_mobile->secDiffToCurrent[i];

                //RELIABILITY
                //Set data reliability:
    #ifdef __RMM_NO_RELIABILITY_CODE__
                RAData = 1.0;
    #else
                RData2 = (aprevious.RD + aprevious2.RD)/2.0;
                RAData = (RData + RData2) / 2.0;
    #endif
                //Acuity: 3 pixels/frame2 = 3/tdiff2 pixels/sec
                updateAttributeDynamics(AData, RAData, m_pixelAcuity/(tdiff*tdiff), attcur, currentCooling, d.A);

            } else //The current element has not been found
                updateAttributeDynamicsNoData(attcur, m_pixelAcuity/(tdiff*tdiff), currentCooling, d.A);
        }
    } //A inside V
}

void Blob2DFromBGSubstractionModel::updateAttributeDynamics(double data, double RData, double acuity,
                             double estimate, double currentCooling, AttributeTripletValue &att) {
    double curSD, auxSum;

    //Cool old reliability X
    auxSum = att.VR.accR;
    att.VR.accR = RData + currentCooling*auxSum;
#ifdef __RMM_NO_RELIABILITY_CODE__
    att.VR.RD = 1.0;
#else
    att.VR.RD = att.VR.accR / m_mobile->sumCooling;
    if(att.VR.RD > 1.0) att.VR.RD = 1.0;
    if(att.VR.RD < 0.0) att.VR.RD = 0.0;
#endif
    //Incremental SD Formula uses Xcur value, storing failures in estimation on real data, representing COHERENCE
    double curDiff = (data - estimate);
    curSD = att.VR.SD;
    att.VR.SD = sqrt( (currentCooling*auxSum/att.VR.accR) * (curSD*curSD + RData*(curDiff*curDiff/att.VR.RD)) );
#ifdef __RMM_NO_RELIABILITY_CODE__
    att.VR.RC = 1.0;
#else
    if(auxSum == 0.0) //For first element there is no coherence yet
        att.VR.RC = 0.0;
    else  //for next elements this reliability makes sense
        att.VR.RC = normalisedSigmaCoherenceReliability(att.VR.SD, acuity);
#endif
    att.value = (data*RData + currentCooling*auxSum*estimate) / att.VR.accR;
}

void Blob2DFromBGSubstractionModel::updateAttributeDynamicsNoData(double estimate, double acuity, double currentCooling, AttributeTripletValue &att) {
    double curSD, auxSum;


    if(att.VR.RC < 0.0000001) { //If no elements has been already added in later frames
        //In this case the last estimation is keeped to avoid loosing information in case of sucesive noisy
        //data obtention
        att.VR.accR = att.VR.RD = att.VR.RC = 0.0;
        att.VR.SD = att.VR.SD < acuity ? acuity : att.VR.SD;
    } else { //If not found and we have previous information
        //Incremental step
        auxSum = att.VR.accR;
        att.VR.accR = currentCooling * att.VR.accR; //Reliability in data is diminished in time
#ifdef __NO_RELIABILITY_CODE__
        att.VR.RD = 1.0;
#else
        att.VR.RD = att.VR.accR / m_mobile->sumCooling;
        if(att.VR.RD > 1.0) att.VR.RD = 1.0;
        if(att.VR.RD < 0.0) att.VR.RD = 0.0;
#endif
        //Incremental SD Formula uses cur value, storing failures in estimation on real data, representing COHERENCE
        curSD = att.VR.SD;
        //In abscense of data SD must be incremented. We take the acuity as step value
        att.VR.SD = sqrt( (currentCooling*auxSum/att.VR.accR) * (curSD*curSD + 1.0*(acuity*acuity/att.VR.RD)) );
        //att.VR.SD = sqrt( (currentCooling*auxSum/att.VR.accR) * (curSD*curSD) );

#ifdef __NO_RELIABILITY_CODE__
        att.VR.RC = 1.0;
#else
        att.VR.RC = normalisedSigmaCoherenceReliability(att.VR.SD, acuity);
#endif
        att.value = estimate; //Use estimation
    }

}


//updates every activated model
void Blob2DFromBGSubstractionModel::updateDynamics() {


    if(instances.size() == 1) {
        std::map <QString, AttributeDupletValue> &inst_att = instances.back().attributes;
        std::map <QString, AttributeDupletValue>::iterator it, it_end = inst_att.end();
        for(it = inst_att.begin(); it != it_end; it ++)
            initAttribute(dynamics.dynamics[it->first], it->second);
        return;
    }

    std::map <QString, ReliabilityDynamicsAttribute>::iterator it, it_end = dynamics.dynamics.end();
    for(it = dynamics.dynamics.begin(); it != it_end; it ++) {
        const QString &name = it->first;
/*        if(name == "W")
            updateMinConstrainedAttribute(it->first, it->second, buffer, m_minimalWidth);
        else if (name == "H")
            updateMinConstrainedAttribute(it->first, it->second, buffer, m_minimalHeight);
        else*/
        updateAttribute(it->first, it->second);
    }
}

//sets global probability P for the model (how good is the model)
//sets global reliability R for the model (how good is the data used in the model)
//It is part of the global feedback process: it considers dependant models response, and
//
void Blob2DFromBGSubstractionModel::setProbabilityAndReliability() {
    double cr, cp, r, psum = 0.0, rsum = 0.0;
    std::map <QString, ReliabilityDynamicsAttribute>::iterator it, it_end = dynamics.dynamics.end();
    for(it = dynamics.dynamics.begin(); it != it_end; it ++) {
        r = it->second.att.VR.RD;
        psum += r*it->second.att.VR.RC;
        rsum += r;
    }
    P = rsum > 0.0 ? psum / rsum : 0.0;
    R_backward = rsum / dynamics.dynamics.size();

    if(!dependants.empty()) { //If mean response of dependant models is higher, keep it.
        std::deque <SpReliabilitySingleModelInterface>::iterator dit, dit_end = dependants.end();
        psum = 0.0; rsum = 0.0;
        int j = 0;
        for(dit=dependants.begin(); dit !=dit_end; dit++) {
            cr = (*dit)->R_backward;
            cp = (*dit)->P;
            if(R_backward <= cr && P < cp) { //Consider only improvements in probability and reliability measure
                psum += cp*cr;
                rsum += cr;
                j++;
            }
        }
        if(rsum > 0.0) { //At least one was better
            double p = psum / rsum;
            P = p;
            R_backward = rsum / j;
        }
    }

}

//feedback process for depurating information based on dependant models.
//It should be programmed checking the presence of different models that can
//improve the estimation of model attributes.
void Blob2DFromBGSubstractionModel::feedback() { }

//sets blob enclosing the model.
void Blob2DFromBGSubstractionModel::setForwardBlobInterface() {
    double w = dynamics.dynamics["W"].att.value, h = dynamics.dynamics["H"].att.value;
    BoundingBox &r = binterface_forward.bbox;
    r.initRectangle(dynamics.dynamics["X"].att.value-w/2, dynamics.dynamics["Y"].att.value-h/2, w, h);

}

//sets blob enclosing the model.
void Blob2DFromBGSubstractionModel::setBackwardBlobInterface() {
    double w = dynamics.dynamics["W"].att.value, h = dynamics.dynamics["H"].att.value;
    BoundingBox &r = binterface_backward.bbox;
    r.initRectangle(dynamics.dynamics["X"].att.value-w/2, dynamics.dynamics["Y"].att.value-h/2, w, h);

}



double Blob2DFromBGSubstractionModel::projectAttributeValue(ReliabilityDynamicsAttribute &attr, double &R) {
    double t = RMMMobileObject::secDiffSequence[0];

    if(!attr.updateA) //Accepted that a model do not consider acceleration.
        R = (attr.att.VR.RC + attr.V.VR.RC)/2.0;
    else
        R = (attr.att.VR.RC + attr.V.VR.RC + attr.A.VR.RC)/3.0;

    if(!attr.updateV || attr.V.VR.RC < m_minimalAttributeVelocityReliability) {
        return attr.att.value;
    } else if(!attr.updateA || attr.A.VR.RC < m_minimalAttributeAccelerationReliability) {
        return attr.att.value + t * attr.V.value;
    } else {
        return attr.att.value + t * attr.V.value + 0.5 * t * t * attr.A.value;
    }
}

Rectangle<int> Blob2DFromBGSubstractionModel::getEstimator(double &R) {
    Rectangle<int> rec;
    double rx, ry, rw, rh;
    double x = projectAttributeValue(dynamics.dynamics["X"], rx),
           y = projectAttributeValue(dynamics.dynamics["Y"], ry),
           w = projectAttributeValue(dynamics.dynamics["W"], rw),
           h = projectAttributeValue(dynamics.dynamics["H"], rh);

    int w_2 = (int)w / 2, h_2 = (int)h / 2;

    R = (rx + ry + rw + rh) / 4.0;

    rec.xleft   = x - w_2;
    rec.xright  = x + w_2;
    rec.ytop    = y - h_2;
    rec.ybottom = y + h_2;
    rec.width = rec.xright - rec.xleft + 1;
    rec.height = rec.ybottom - rec.ytop + 1;

    return rec;

}

void Blob2DFromBGSubstractionModel::copy(SpReliabilitySingleModelInterface model) {
    ReliabilitySingleModelInterface *i = &*model;
    Blob2DFromBGSubstractionModel*ii = (Blob2DFromBGSubstractionModel *)i;
    m_minimalAttributeVelocityReliability = ii->m_minimalAttributeVelocityReliability;
    m_minimalAttributeAccelerationReliability = ii->m_minimalAttributeAccelerationReliability;
    m_pixelAcuity = ii->m_pixelAcuity;
}

void Blob2DFromBGSubstractionModel::copy_structure(SpReliabilitySingleModelInterface model) {
}

