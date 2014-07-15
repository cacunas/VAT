#include "segmentationWithColorFilterModule.h"
#include "image_display.h"
#include "src/MathFunctions.h"
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

segmentationWithColorFilterModule::segmentationWithColorFilterModule(Datapool *i_data): ModuleInterface(i_data) {
    globalMask = NULL;
    globalValidPoints = 0;
    global_validated = false;
    first = true;
    globalNotSet = true;
    validate_once = true;
    different_bg_current = false;

    //Rectangular structuring element
    element = cv::getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 3, 3 ),
                                         cv::Point( 1, 1 ) );
}

segmentationWithColorFilterModule::~segmentationWithColorFilterModule() {}

bool segmentationWithColorFilterModule::setParameters(QDomNode& config){
    QDomNode n, m;
    int intT;

    if(config.isNull()) { //Parameter set for module not defined
        alpha = 0.01;
        BGup = true;
        T = 50;
        first = true;
        m_mask = false;
        m_global = false;
        m_N = 200;
        m_K = 3;
        m_InZone = "";
        m_OutZone = "";
        m_inDefined = false;
        m_outDefined = false;
        zInType = Z_NONE;
        zOutType = Z_NONE;
        globalMask = NULL;
        globalValidPoints = 0;
        m_aperture = true;
        minGrade=300;
        maxGrade=80;

    } else {
        if( !( n = XmlCommon::XmlCommon::getParameterNode("UpdateBG", config) ).isNull() ) {
            BGup = XmlCommon::getParameterValue(n) == "true" ? true : false;
            alpha = 0;
            if(BGup) {
                QDomNode m;
                if( ( m = XmlCommon::XmlCommon::getParameterNode("Alpha", n) ).isNull() ) {
                    alpha = 0.01; //default
                    AppendToLog("segmentationModule: Warning: 'UpdateBG.Alpha' not defined. Taking default: " + QString::number(alpha));
                } else
                    alpha = XmlCommon::getParameterValue(m).toDouble();
            }
        } else { //default
            BGup = true;
            alpha = 0.01;
            AppendToLog("segmentationModule: Warning: 'UpdateBG' not defined. Taking defaults: \n\t\tUpdateBG = true\n\tUpdateBG.Alpha = " + QString::number(alpha) );
        }

        if( !( n = XmlCommon::getParameterNode("minColor", config) ).isNull() ) {
            minGrade = XmlCommon::getParameterValue(n).toInt();

        } else { //default
            minGrade = 300;

            AppendToLog("ChromaticSegmentationModule: Warning: non min grade color value" );
        }

        if( !( n = XmlCommon::getParameterNode("maxColor", config) ).isNull() ) {
            maxGrade = XmlCommon::getParameterValue(n).toInt();

        } else { //default
            maxGrade = 80;

            AppendToLog("ChromaticSegmentationModule: Warning: non max grade color value" );
        }



        if( ( n = XmlCommon::getParameterNode("Threshold", config) ).isNull() ) {
            T = 50;
//            parameters["Threshold"] = "50";
//            addParameter("Threshold", "50", "uchar", 0);
            addParameter("Threshold", "50", "uchar");
            AppendToLog("segmentationModule: Warning: 'Threshold' not defined. Taking default: " + QString::number(T));
        } else {
            intT = XmlCommon::getParameterValue(n).toInt();
//            parameters["Threshold"] = getParameterValue(n);
//            addParameter("Threshold", getParameterValue(n), "uchar", 0);
            addParameter("Threshold", XmlCommon::getParameterValue(n), "uchar");
            T = (unsigned char) (intT <= 0 || intT >= 255 ? 50 : intT);
        }

        if( ( n = XmlCommon::getParameterNode("InitialBG", config) ).isNull() )
            first = true;
        else {
            if (XmlCommon::getParameterValue(n) == "first")
                first = true;
            else if(XmlCommon::getParameterValue(n) == "defined") {
                first = false;
                if( ( m = XmlCommon::getParameterNode("BGImage", n) ).isNull() ) {
                    first = true;
                    AppendToLog("SegmentationModule: Warning: 'BGImage' parameter of 'InitialBG' parameter for segmentationModule not found. Taking default for 'InitialBG': 'first'.\n");
                } else
                    bgImName = XmlCommon::getParameterValue(m);
            } else {
                first = true;
                AppendToLog("SegmentationModule: Warning: Value '" + XmlCommon::getParameterValue(n) + "' for 'InitialBG' parameter (segmentationModule) for  not found. Taking default: 'first'.\n");
            }
        }

        if( ( n = XmlCommon::getParameterNode("Mask", config) ).isNull() )
            m_mask = false;
        else {
            if (XmlCommon::getParameterValue(n) == "yes") {
                m_mask = true;
                if( ( m = XmlCommon::getParameterNode("MaskImage", n) ).isNull() ) {
                    m_mask = false;
                    AppendToLog("SegmentationModule: Warning: Expecting 'MaskImage' parameter. Then, no mask image is considered.\n");
                } else
                    m_maskImName = XmlCommon::getParameterValue(m);
            } else
                m_mask = false;
        }

    }



    if( !( n = XmlCommon::XmlCommon::getParameterNode("GlobalUpdateBG", config) ).isNull() ) {
        m_global = XmlCommon::getParameterValue(n) == "true" ? true : false;
        if(m_global) {
            QDomNode m, o;
            if( ( m = XmlCommon::XmlCommon::getParameterNode("N", n) ).isNull() ) {
                m_N = 200; //default
                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.N' not defined. Taking default: " + QString::number(m_N));
            } else {
                m_N = XmlCommon::getParameterValue(m).toInt();
                if(m_N <= 0) {
                    m_N = 200; //default
                    AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.N' must be a positive integer. Taking default: " + QString::number(m_N));
                }
            }

            if( ( m = XmlCommon::XmlCommon::getParameterNode("K", n) ).isNull() ) {
                m_K = 3; //default
                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.K' not defined. Taking default: " + QString::number(m_K));
            } else {
                m_K = XmlCommon::getParameterValue(m).toInt();
                if(m_K <= 0) {
                    m_K = 3; //default
                    AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.K' must be a positive integer. Taking default: " + QString::number(m_K));
                }
            }

            //InZone
            if( ( m = XmlCommon::XmlCommon::getParameterNode("InZone", n) ).isNull() ) {
                m_inDefined = false; //default
                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.InZone' not defined. Taking default: false");
            } else {
                m_inDefined = XmlCommon::getParameterValue(m) == "true" ? true : false;

                if(m_inDefined) {
                    if( ( o = XmlCommon::XmlCommon::getParameterNode("ZoneList", m) ).isNull() ) {
                        m_inDefined = false; //default
                        AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.InZone.ZoneList' not defined. Taking default: GlobalUpdateBG.InZone = false");
                    } else {
                        zInType = Z_NONE;
                        QString lname = XmlCommon::getParameterValue(o);
                        if(lname == "ZoneHomographyList") zInType = Z_H;
                        else if(lname == "ZoneList")      zInType = Z_3D;
                        else if(lname == "Zone2DList")    zInType = Z_2D;
                        else {
                            zInType = Z_NONE;
                            m_inDefined = false; //default
                            AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.InZone.ZoneList' not well defined. Taking default: GlobalUpdateBG.InZone = false");
                        }
                        if(zInType != Z_NONE) {
                            if( ( o = XmlCommon::XmlCommon::getParameterNode("ZoneName", m) ).isNull() ) {
                                m_inDefined = false; //default
                                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.InZone.ZoneName' not defined. Taking default: GlobalUpdateBG.InZone = false");
                            } else {
                                m_InZone = XmlCommon::getParameterValue(o);
                            }
                        }
                    }
                }
            }

            //OutZone
            if( ( m = XmlCommon::XmlCommon::getParameterNode("OutZone", n) ).isNull() ) {
                m_outDefined = false; //default
                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.OutZone' not defined. Taking default: false");
            } else {
                m_outDefined = XmlCommon::getParameterValue(m) == "true" ? true : false;

                if(m_outDefined) {
                    if( ( o = XmlCommon::XmlCommon::getParameterNode("ZoneList", m) ).isNull() ) {
                        m_outDefined = false; //default
                        AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.OutZone.ZoneList' not defined. Taking default: GlobalUpdateBG.OutZone = false");
                    } else {
                        zOutType = Z_NONE;
                        QString lname = XmlCommon::getParameterValue(o);
                        if(lname == "ZoneHomographyList") zOutType = Z_H;
                        else if(lname == "ZoneList")      zOutType = Z_3D;
                        else if(lname == "Zone2DList")    zOutType = Z_2D;
                        else {
                            zOutType = Z_NONE;
                            m_outDefined = false; //default
                            AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.OutZone.ZoneList' not well defined. Taking default: GlobalUpdateBG.OutZone = false");
                        }
                        if(zOutType != Z_NONE) {
                            if( ( o = XmlCommon::XmlCommon::getParameterNode("ZoneName", m) ).isNull() ) {
                                m_outDefined = false; //default
                                AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG.InZone.ZoneName' not defined. Taking default: GlobalUpdateBG.OutZone = false");
                            } else {
                                m_OutZone = XmlCommon::getParameterValue(o);
                            }
                        }
                    }
                }
            } //end OutZone

        }
    } else { //default
        m_global = false;
        m_N = 200;
        m_K = 3;
        m_inDefined = false;
        m_outDefined = false;
        zInType = Z_NONE;
        zOutType = Z_NONE;
        m_InZone = "";
        m_OutZone = "";
        globalMask = NULL;
        globalValidPoints = 0;
        AppendToLog("segmentationModule: Warning: 'GlobalUpdateBG' not defined. Taking defaults: \n\t\tGlobalUpdateBG = false");
    }

    if( !( n = XmlCommon::XmlCommon::getParameterNode("Aperture", config) ).isNull() )
        m_aperture = XmlCommon::getParameterValue(n) == "true" ? true : false;
    else {
        m_aperture = true;
        AppendToLog("segmentationModule: Warning: 'Aperture' not defined. Taking defaults: \n\t\tAperture = true");
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("UpdateBG", (BGup)?"true":"false", "bool");
    //addParameterChild("Alpha", QString::number(alpha), "double", getParameter("Alpha"));
    addParameter("InitialBG", (first ? "first" : "defined"), "bool");
    addParameterChild("BGImage", bgImName, "QString", getParameter("InitialBG"));
    addParameter("Mask", (m_mask)?"yes":"no", "bool");
    addParameterChild("MaskImage", m_maskImName, "QString", getParameter("Mask"));
    return true;
}

bool segmentationWithColorFilterModule::updateParameters(){
    parameter *threshold, *updatebg, *alpha, *initialbg, *bgimage, *mask, *maskimage;
    threshold = getParameter("Threshold");
    updatebg = getParameter("UpdateBG");
    alpha = getParameter("Alpha", updatebg);
    initialbg = getParameter("InitialBG");
    bgimage = getParameter("BGImage", initialbg);
    mask = getParameter("Mask");

    maskimage = getParameter("MaskImage", mask);
    if( threshold == 0 || updatebg == 0 || alpha == 0 || initialbg == 0 || bgimage == 0 || mask == 0 || maskimage == 0 ||
            threshold->value.toInt() < 0 || alpha->value.toDouble() < 0.0 || updatebg->value.isEmpty() ||
            updatebg->value.isNull() || initialbg->value.isEmpty() || initialbg->value.isNull() ||
            bgimage->value.isEmpty() || bgimage->value.isNull() || mask->value.isEmpty() ||
            mask->value.isNull()|| maskimage->value.isEmpty() || maskimage->value.isNull())
        return false;

    T = (unsigned char) (threshold->value.toInt() <= 0 || threshold->value.toInt() >= 255 ? 50 : threshold->value.toInt());
    BGup = (updatebg->value == "true") ? true : false;
    this->alpha  = alpha->value.toDouble();
    first = (initialbg->value == "first") ? true : false;
    bgImName = bgimage->value;
    m_mask = (mask->value == "yes") ? true : false;
    m_maskImName = maskimage->value;
    return true;
}

bool segmentationWithColorFilterModule::init(){
    if(first == false) {
        if(m_data->bgImage != NULL)
            delete m_data->bgImage;

        m_data->bgImage = new QImage(bgImName);
        if(m_data->bgImage->isNull()) {
            AppendToLog("SegmentationWithColorFilterModule: Warning: Cannot open background image '" + bgImName + "'. Considering first image in sequence as background.\n");

            delete m_data->bgImage;
            m_data->bgImage = NULL;
            first = true;
        }
    }

    return true;
}


bool segmentationWithColorFilterModule::setGlobalZones() {
    if(m_data->sceneModel == NULL)
        return false;
    if(m_inDefined) {
        if(zInType == Z_H) {
            if(m_data->sceneModel->ZonesH.empty()) {
                m_inDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::ZoneH> >::iterator it, it_end = m_data->sceneModel->ZonesH.end();
                for(it = m_data->sceneModel->ZonesH.begin(); it != it_end; it++)
                    if((*it)->name == m_InZone) {
                        found = true;
                        zinH = (*it);
                        break;
                    }
                if(found == false) {
                    m_inDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
                }
           }
        } else if(zInType == Z_3D) {
            if(m_data->sceneModel->Zones.empty()) {
                m_inDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::Zone> >::iterator it, it_end = m_data->sceneModel->Zones.end();
                for(it = m_data->sceneModel->Zones.begin(); it != it_end; it++)
                    if((*it)->name == m_InZone) {
                        found = true;
                        zin3D = (*it);
                        break;
                    }
                if(found == false) {
                    m_inDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
                }
           }
        } else { //    zInType == Z_2D;
            if(m_data->sceneModel->Zones2D.empty()) {
                m_inDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::Zone2D> >::iterator it, it_end = m_data->sceneModel->Zones2D.end();
                for(it = m_data->sceneModel->Zones2D.begin(); it != it_end; it++)
                    if((*it)->name == m_InZone) {
                        found = true;
                        zin2D = (*it);
                        break;
                    }
                if(found == false) {
                    m_inDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_InZone + " for 'GlobalUpdateBG.InZone.ZoneName' not found. Taking default: GlobalUpdateBG.InZone = false");
                }
            }
        }
    }

    if(m_outDefined) {
        if(zOutType == Z_H) {
            if(m_data->sceneModel->ZonesH.empty()) {
                m_outDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::ZoneH> >::iterator it, it_end = m_data->sceneModel->ZonesH.end();
                for(it = m_data->sceneModel->ZonesH.begin(); it != it_end; it++)
                    if((*it)->name == m_OutZone) {
                        found = true;
                        zoutH = (*it);
                        break;
                    }
                if(found == false) {
                    m_outDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
                }
            }
        } else if(zOutType == Z_3D) {
            if(m_data->sceneModel->Zones.empty()) {
                m_outDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::Zone> >::iterator it, it_end = m_data->sceneModel->Zones.end();
                for(it = m_data->sceneModel->Zones.begin(); it != it_end; it++)
                    if((*it)->name == m_OutZone) {
                        found = true;
                        zout3D = (*it);
                        break;
                    }
                if(found == false) {
                    m_outDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
                }
            }
        } else { //    zOutType == Z_2D;
            if(m_data->sceneModel->Zones2D.empty()) {
                m_outDefined = false; //default
                AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
            } else {
                bool found = false;
                std::vector< QSharedPointer<world::Zone2D> >::iterator it, it_end = m_data->sceneModel->Zones2D.end();
                for(it = m_data->sceneModel->Zones2D.begin(); it != it_end; it++)
                    if((*it)->name == m_OutZone) {
                        found = true;
                        zout2D = (*it);
                        break;
                    }
                if(found == false) {
                    m_outDefined = false; //default
                    AppendToLog("segmentationModule: Warning: Value:" + m_OutZone + " for 'GlobalUpdateBG.OutZone.ZoneName' not found. Taking default: GlobalUpdateBG.OutZone = false");
                }
            }
        }
    }

    if(m_inDefined || m_outDefined)
        return true;

    return false;

}


//i :y img coord, j: x img coord
bool segmentationWithColorFilterModule::insideInZone(int i, int j) {
    if(!m_inDefined || m_data->sceneModel == NULL)
        return true;
    double x, y;
    if(zInType == Z_H) {
        SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix, j, i, &x, &y);
        return zinH->pointInZone(x, y);
    } else if(zInType == Z_3D) {
        SceneModel::imgToWorldCoordsGivenHeight(m_data->sceneModel->p_matrix, j, i, 0, &x, &y);
        return zin3D->pointInZone(x, y);
    }

    return zin2D->pointInZone(j, i);
}

//i :y img coord, j: x img coord
bool segmentationWithColorFilterModule::outsideOutZone(int i, int j) {
    if(!m_outDefined || m_data->sceneModel == NULL)
        return true;
    double x, y;
    if(zOutType == Z_H) {
        SceneModel::imgToHomographyCoords(m_data->sceneModel->h_matrix, j, i, &x, &y);
        return !zoutH->pointInZone(x, y);
    } else if(zOutType == Z_3D) {
        SceneModel::imgToWorldCoordsGivenHeight(m_data->sceneModel->p_matrix, j, i, 0, &x, &y);
        return !zout3D->pointInZone(x, y);
    }

    return !zout2D->pointInZone(j, i);
}

bool segmentationWithColorFilterModule::setGlobalMask() {

    //First build image
    int w = m_data->bgImage->width(), h = m_data->bgImage->height();
    if(globalMask == NULL) {
        globalMask = new QImage(w, h, QImage::Format_Indexed8);
        globalMask->setColorTable(*m_data->grayScaleTable);
    }

    uchar *pixels = globalMask->bits();
    int i, j, step = globalMask->bytesPerLine();
    bool in_run;
    int current_start;

    memset(pixels, 0, h*step*sizeof(uchar));
    globalValidPoints = 0;

    for(i = 0; i < h; i++) {
        in_run = false;
        for(j = 0; j < w; j++)
            if(insideInZone(i, j) && outsideOutZone(i, j)) {
                pixels[i*step + j] = 255;
                globalValidPoints++;
                if(in_run == false) {
                    in_run = true;
                    current_start = j;
                }
            } else {
                if(in_run == true) {
                    in_run = false;
                    RLEglobal[i].push_back(RLESegment(current_start, j - current_start + 1));
                }
            }
        if(in_run == true) { //run ended without closing
            in_run = false;
            RLEglobal[i].push_back(RLESegment(current_start, j - current_start + 1));
        }
    }

    if(globalValidPoints == 0)
        return false;
    else if(globalValidPoints / (int)(m_N + 2) > 0) { //Add two to discard first and last
        int point_step = globalValidPoints / (int)(m_N + 2),
            yindex, xindex, next_index = point_step, curr_index = 0, num_inserted = 0, length;

        std::map<int, std::deque<RLESegment> >::iterator rle_it, rle_end = RLEglobal.end();
        std::deque<RLESegment>::iterator seg_it, seg_end;

        for(rle_it = RLEglobal.begin(); rle_it != rle_end; rle_it++) {
            yindex = rle_it->first;
            seg_end = rle_it->second.end();
            for(seg_it = rle_it->second.begin(); seg_it != seg_end; seg_it++) {
                xindex = seg_it->x;
                length = seg_it->length;
                //Put points until out of segment
                while(next_index < curr_index + length) {
                    //Add point to process
                    globalPoints[yindex].push_back(xindex + next_index - curr_index);
                    num_inserted++;
                    if(num_inserted == m_N)
                        break;
                    next_index += point_step;
                }

                curr_index += length;
                if(num_inserted == m_N)
                    break;
            }
            if(num_inserted == m_N)
                break;

        }
    }


    return true;
}

void segmentationWithColorFilterModule::updateBGGlobally(QImage *current) {
    uchar *cpixels = current->bits(), *bgpixels = m_data->bgImage->bits();
    int x, y, i, j, k, cstep = current->bytesPerLine(), bstep = m_data->bgImage->bytesPerLine();
    std::map<int, std::deque<int> >::iterator it, it_end = globalPoints.end();
    std::deque<int>::iterator pit, pit_end;
    int w = current->width(), h = current->height(), K2 = m_K/2, count, ind1, ind2;

    double means[3] = {0, 0, 0}, sigmas[3] = {0, 0, 0}, sum[3];

    if(global_validated) {
        for(k=1, it = globalPoints.begin(); it != it_end; it++){
            y = it->first;
            pit_end = it->second.end();
            for(pit = it->second.begin(); pit != pit_end; k++, pit++) {
                x = *pit;
                memset(sum, 0, 3*sizeof(double)); count = 0;
                for(i=y-K2; i <= y+K2; i++)
                    if(i>=0 && i<h)
                        for(j=x-K2; j <= x+K2; j++)
                            if(j>=0 && j<w) {
                                ind1 = i*cstep + j*4;
                                ind2 = i*bstep + j*4;
                                //Blue
                                sum[0] += cpixels[ind1] - bgpixels[ind2];
                                //Green
                                sum[1] += cpixels[ind1+1] - bgpixels[ind2+1];
                                //Red
                                sum[2] += cpixels[ind1+2] - bgpixels[ind2+2];
                                count++;
                            }
                sum[0] /= count;
                sum[1] /= count;
                sum[2] /= count;

                means[0] = MathFunctions::incrementalMean(means[0], sum[0], k);
                sigmas[0] = MathFunctions::incrementalSigma(means[0], sigmas[0], sum[0], k);

                means[1] = MathFunctions::incrementalMean(means[1], sum[1], k);
                sigmas[1] = MathFunctions::incrementalSigma(means[1], sigmas[1], sum[1], k);

                means[2] = MathFunctions::incrementalMean(means[2], sum[2], k);
                sigmas[2] = MathFunctions::incrementalSigma(means[2], sigmas[2], sum[2], k);

            }
        }
    } else {
        int l = w*h, m, step = l/m_N;
        if (step == 0) step = 1;

        std::cerr << "w:"       << w     << ", h:"     << h
                  << ", cstep:" << cstep << ", bstep:" << bstep
                  << ", step:"  << step  << ", l:"     << l << std::endl;

        for(k=1, m=step/2; m<l; k++, m+=step){
            y = m/w;
            x = m%w;
            memset(sum, 0, 3*sizeof(double)); count = 0;
            for(i=y-K2; i <= y+K2; i++)
                if(i>=0 && i<h)
                    for(j=x-K2; j <= x+K2; j++)
                        if(j>=0 && j<w) {
                            ind1 = i*cstep + j*4;
                            ind2 = i*bstep + j*4;
                            //Blue
                            std::cerr << "y:"      << y    << ", x:" << x
                                      << ", i:"    << i    << ", j:" << j
                                      << ", ind1:" << ind1 << ", ind2:" << ind2 << std::endl;
                            sum[0] += cpixels[ind1] - bgpixels[ind2];
                            //Green
                            sum[1] += cpixels[ind1+1] - bgpixels[ind2+1];
                            //Red
                            sum[2] += cpixels[ind1+2] - bgpixels[ind2+2];
                            count++;
                        }
            sum[0] /= count;
            sum[1] /= count;
            sum[2] /= count;

            means[0] = MathFunctions::incrementalMean(means[0], sum[0], k);
            sigmas[0] = MathFunctions::incrementalSigma(means[0], sigmas[0], sum[0], k);

            means[1] = MathFunctions::incrementalMean(means[1], sum[1], k);
            sigmas[1] = MathFunctions::incrementalSigma(means[1], sigmas[1], sum[1], k);

            means[2] = MathFunctions::incrementalMean(means[2], sum[2], k);
            sigmas[2] = MathFunctions::incrementalSigma(means[2], sigmas[2], sum[2], k);

        }
    }


    std::cout << "Global BG:" << std::endl;
    std::cout << "B: mean = " << means[0] << "sigma = " << sigmas[0] << std::endl;
    std::cout << "G: mean = " << means[1] << "sigma = " << sigmas[1] << std::endl;
    std::cout << "R: mean = " << means[2] << "sigma = " << sigmas[2] << std::endl;

    //Criteria for significant difference
    if(abs(means[0]) + abs(means[1]) + abs(means[2]) > T/2) {
        int aux, diffB = means[0], diffG = means[1], diffR = means[2];
        if(m_mask) {
            uchar *mask = m_data->maskImage->bits();
            int mstep = m_data->maskImage->bytesPerLine();
            for(i=0; i<h; i++) {
                for(j=0, k=0; k<w; j+=4, k++) {
                    if(mask[i*mstep + k]) {
                        ind1 = i*bstep + j;
                        aux = bgpixels[ind1] + diffB;
                        bgpixels[ind1] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                        aux = bgpixels[ind1+1] + diffG;
                        bgpixels[ind1+1] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                        aux = bgpixels[ind1+2] + diffR;
                        bgpixels[ind1+2] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                    }
                }
            }
        } else {
            for(i=0; i<h; i++) {
                for(j=0, k=0; k<w; j+=4, k++) {
                    ind1 = i*bstep + j;
                    aux = bgpixels[ind1] + diffB;
                    bgpixels[ind1] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                    aux = bgpixels[ind1+1] + diffG;
                    bgpixels[ind1+1] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                    aux = bgpixels[ind1+2] + diffR;
                    bgpixels[ind1+2] = aux > 255 ? 255 : (aux < 0 ? 0 : aux);
                }
            }
        }
    }
}


bool segmentationWithColorFilterModule::run(){

    if(m_data->currentImage == NULL || m_data->currentImage->isNull()) {
        AppendToLog("SegmentationModule: Warning: No current image. Aborting execution...\n");
        return false;
    }

    if(validate_once) {
        validate_once = false;

        //Set background image:
        if(first && m_data->bgImage == NULL)
            m_data->bgImage = new QImage(*m_data->currentImage);

        if(    m_data->currentImage->width() != m_data->bgImage->width()
            || m_data->currentImage->height() != m_data->bgImage->height()) {
            AppendToLog("SegmentationModule: Warning: Background image size different from current image. Aborting execution...\n");
            different_bg_current = true;
        }
    }

    if(different_bg_current)
        return false;

    if(globalNotSet && m_global) { //If global background updating is activated
        globalNotSet = false;
        global_validated = false;
        if(setGlobalZones()) {
            if(setGlobalMask())
                global_validated = true;
        }
    }

    //Set foreground image
    if(m_data->fgImage == NULL) {
        m_data->fgImage = new QImage(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_Indexed8);
        m_data->fgImage->setColorTable(*(m_data->grayScaleTable));
        m_data->rFgImage = new QImage(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_RGB888);
    }

    //Set mask or set it to NULL
    if(m_mask == true) {
        if(m_data->maskImage == NULL) {
            m_data->maskImage = new QImage(m_data->currentImage->width(),
                                           m_data->currentImage->height(),
                                           QImage::Format_Indexed8);
            if(!m_data->maskImage->load(m_maskImName) || m_data->maskImage->isNull()) {
                m_mask = false;
                delete m_data->maskImage;
                m_data->maskImage = NULL;
                AppendToLog("SegmentationModule: Warning: Cannot open mask image '" + m_maskImName + "'. Then, no mask image is considered.\n");
            } else if(   m_data->maskImage->width() != m_data->currentImage->width()
                      || m_data->maskImage->height() != m_data->currentImage->height()) {
                m_mask = false;
                delete m_data->maskImage;
                m_data->maskImage = NULL;
                AppendToLog("SegmentationModule: Warning: Mask image '" + m_maskImName + "' has different dimensions than image sequence. Then, no mask image is considered.\n");
            }
        }
    }

    if(m_global)
        updateBGGlobally(m_data->currentImage);

    getChromaticSegmentation(m_data->fgImage, m_data->rFgImage, minGrade, maxGrade);

    if(m_aperture)
        applyAperture(m_data->fgImage, m_data->rFgImage);

    if(BGup)
        updateBG(m_data->currentImage);

    return true;
}

void segmentationWithColorFilterModule::applyAperture(QImage *fg, QImage *fg_c) {
    int w = fg->width(), h = fg->height();

    cv::Mat f(h, w, CV_8UC1);

    int bl = fg->bytesPerLine();
    uchar *fg_p = fg->bits();

    memcpy(f.data, fg_p, h*bl);

    //Reduce bad detections
    cv::erode(f, f, element, cv::Point(-1,-1), 1);
    cv::dilate(f, f, element, cv::Point(-1,-1), 1);

    memcpy(fg_p, f.data, h*bl);


}

void segmentationWithColorFilterModule::getChromaticSegmentation(QImage *result, QImage *result_c, int minGrade,int maxGrade) {
    if(result == NULL)
        return;
    int w = result->width(),
        h = result->height(),
        b1 = m_data->currentImage->bytesPerLine(),
        b2 = result->bytesPerLine(),
        b3 = result_c->bytesPerLine();
    printf("dimensiones %d,%d\n", w,h);
    uchar *bits = result->bits(),
    *bits_c = result_c->bits(),
    *new_bits = m_data->currentImage->bits();
    int i, j, k, aux, T2 = T*T;

    uchar r,g,b;
    int maxIndice, minIndice;
    int max, min;
    float l;
    int hue;

    memset(bits, 0, h*b2);
    memset(bits_c, 0, h*b3);


    if(m_mask) {
        uchar *mask = m_data->maskImage->bits();
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

                        if(max-min>15){
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
                                if((hue<maxGrade||hue>minGrade)&&l<100){/*
                                    imageToDisplay.at<Vec3b>(y,x)[0] =b;
                                    imageToDisplay.at<Vec3b>(y,x)[1] =g;
                                    imageToDisplay.at<Vec3b>(y,x)[2] =r;*/
                                    bits[i*b2 + k] = 255;
                                    memset(bits_c + i*b3 + 3*k, 255, 3);
                                }
                            }else{
                                if((hue<maxGrade && hue>minGrade)&&l<115){/*
                                        imageToDisplay.at<Vec3b>(y,x)[0] =b;
                                        imageToDisplay.at<Vec3b>(y,x)[1] =g;
                                        imageToDisplay.at<Vec3b>(y,x)[2] =r;*/
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

                    if(max-min>25){
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
                            if((hue<maxGrade || hue>minGrade) && l<160){/*
                                imageToDisplay.at<Vec3b>(y,x)[0] =b;
                                imageToDisplay.at<Vec3b>(y,x)[1] =g;
                                imageToDisplay.at<Vec3b>(y,x)[2] =r;*/
                                //bits[aux+1] = 255;
                                //bits[aux+2] = 255;
                                bits[i*b2 + k] = 255;
                                memset(bits_c + i*b3 + 3*k, 255, 3);
                            }
                        }
                           else{
                            if((hue<maxGrade && hue>minGrade)&& l<160){/*
                                imageToDisplay.at<Vec3b>(y,x)[0] =b;
                                imageToDisplay.at<Vec3b>(y,x)[1] =g;
                                imageToDisplay.at<Vec3b>(y,x)[2] =r;*/
                                //bits[aux+1] = 255;
                                //bits[aux+2] = 255;
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

//The simplest: update by alpha with FG
void segmentationWithColorFilterModule::updateBG(QImage *current) {
    if(current == NULL)
        return;
    int w,h;
    w = current->width();
    h = current->height();

    uchar
    *bits = current->bits(),
    *bg_bits = m_data->bgImage->bits();

    int i, limit = w*h*4;
    double _alpha = 1.0 - alpha;

    for(i=0; i<limit; i+=4) {
        bg_bits[i+2] = _alpha*bg_bits[i+2] + alpha*bits[i+2];
        bg_bits[i+1] = _alpha*bg_bits[i+1] + alpha*bits[i+1];
        bg_bits[i]   = _alpha*bg_bits[i]   + alpha*bits[i]  ;
    }
}



