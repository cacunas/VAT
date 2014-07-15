#include "ETISEOOutputModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QTextStream>
#include <QPainter>
#include <deque>

ETISEOOutputModule::ETISEOOutputModule(Datapool *i_data): ModuleInterface(i_data) {}

ETISEOOutputModule::~ETISEOOutputModule() {}

//Set module configuration parameters
bool ETISEOOutputModule::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        QMessageBox::warning(0, "ETISEOOutputModule Error.", "If ETISEOOutputModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
        return false;
    } else {
        if( ( n = XmlCommon::getParameterNode("OutputFileName", config) ).isNull() ) {
            QMessageBox::warning(0, "ETISEOOutputModule Error.", "If ETISEOOutputModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
            return false;
        } else
            m_outputFileName = XmlCommon::getParameterValue(n);
        if( ( n = XmlCommon::getParameterNode("EndFrameNumber", config) ).isNull() ) {
            QMessageBox::warning(0, "ETISEOOutputModule Error.", "If ETISEOOutputModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
            return false;
        } else
            m_endFrame = XmlCommon::getParameterValue(n).toInt();
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("OutputFileName", m_outputFileName, "QString");
    addParameter("EndFrameNumber", QString::number(m_endFrame), "int");
    return true;
}

bool ETISEOOutputModule::updateParameters(){
    parameter *ofn, *efn;
    ofn = getParameter("OutputFileName");
    efn = getParameter("EndFrameNumber");

    if( ofn == 0 || efn == 0 ||
            ofn->value.isNull() || ofn->value.isEmpty() ||
            efn->value.toInt() < 0)
        return false;
    m_outputFileName = ofn->value;
    m_endFrame = efn->value.toInt();
    return true;
}

//Function executed at each frame
bool ETISEOOutputModule::run() {

    if (!m_file.open(QIODevice::Append | QIODevice::Text)) {
        return false;
    }
    QTextStream etiseo_stream(&m_file);

    etiseo_stream << "\t\t<frame frame_id=\"" << m_data->frameNumber << "\">"  << endl;
    std::deque<SpMobileObject2D>::iterator
            mobile_it = m_data->objects2D.begin(),
            end_it = m_data->objects2D.end();
    SpMobileObject2D currentMobile;
    int sframe, X, Y, W, H;

    for(;mobile_it != end_it; mobile_it++) {
        currentMobile = (*mobile_it);
        X = (int)currentMobile->t2DSpatialData.X;
        Y = (int)currentMobile->t2DSpatialData.Y;
        W = (int)currentMobile->t2DDimData.W;
        H = (int)currentMobile->t2DDimData.H;

        if(startingFrame.count(currentMobile->getMobileId()) == 0)
            startingFrame[currentMobile->getMobileId()] = sframe = m_data->frameNumber;
        else
            sframe = startingFrame[currentMobile->getMobileId()];

        etiseo_stream << "\t\t\t<tracked_target target_id=\""
                      << currentMobile->getMobileId()
                      << "\" start_frame_track=\"" << sframe
                      << "\" world_classification=\"0\">" << endl;
        etiseo_stream << "\t\t\t\t<info_3d x=\"0\" y=\"0\" z=\"0\" width=\"0\" length=\"0\" height=\"0\"/>"
                      << endl;
        etiseo_stream << "\t\t\t\t<object object_id=\""
                      << currentMobile->getMobileId()
                      << "\" classification=\"0\">" << endl;
        etiseo_stream << "\t\t\t\t\t<info_2d xmin=\"" << X - W/2
                      << "\" xmax=\"" << X + W/2
                      << "\" ymin=\"" << Y - H/2
                      << "\" ymax=\"" << Y + H/2
                      << "\" x_center=\"" << X
                      << "\" y_center=\"" << Y
                      << "\" viewed_in_camera_id=\"1\">" << endl;
        etiseo_stream << "\t\t\t\t\t</info_2d>" << endl;
        etiseo_stream << "\t\t\t\t</object>" << endl;
        etiseo_stream << "\t\t\t\t<parent_target frame_id=\""
                      << m_data->frameNumber - 1 << "\" target_id=\""
                      << currentMobile->getMobileId() << "\"/>" << endl;
        etiseo_stream << "\t\t\t\t<speed>0</speed>" << endl;
        etiseo_stream << "\t\t\t</tracked_target>" << endl;
    }

    etiseo_stream << "\t\t</frame>"  << endl;

    if(m_data->frameNumber == m_endFrame) {
        etiseo_stream << "\t</results>"  << endl;
        etiseo_stream << "</ETISEO_RESULTS>"  << endl;
    }


    m_file.close();
    return true;
}

bool ETISEOOutputModule::init() {
    m_file.setFileName(m_outputFileName);
    //Erasing file and checking write permissions
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(0, "ETISEOOutputModule Error.", "Unable to open file '"+ m_outputFileName +"' for writing.");
       return false;
    }
    //Write header
    QTextStream etiseo_stream(&m_file);
    etiseo_stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    etiseo_stream << "<ETISEO_RESULTS xmlns=\"http://www.silogic.fr/etiseo/\" ";
    etiseo_stream << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" ";
    etiseo_stream << "xsi:schemaLocation=\"http://www.silogic.fr/etiseo/ETISEO_RESULTS.xsd\">" << endl;
    etiseo_stream << "\t<developer_details author=\"\" email=\"\" institution=\"\"/>" << endl;
    etiseo_stream << "\t<algorithm_details dataset_name=\"VS1\" is_testing_results=\"true\"";
    etiseo_stream << " dataset_capture_number=\"11\" task_developed_for=\"\" version=\"\">" << endl;
    etiseo_stream << "\t\t<run_date day=\"0\" month=\"0\" year=\"0\"/>" << endl;
    etiseo_stream << "\t\t<cameras_used camera_id=\"1\"/>" << endl;
    etiseo_stream << "\t</algorithm_details>" << endl;
    etiseo_stream << "\t<results>" << endl;

    m_file.close();
    return true;
}





