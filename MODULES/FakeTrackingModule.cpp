#include "FakeTrackingModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include "src/object.h"

FakeTrackingModule::FakeTrackingModule(Datapool *i_data): ModuleInterface(i_data) {}

FakeTrackingModule::~FakeTrackingModule() {}

//Set module configuration parameters
bool FakeTrackingModule::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        QMessageBox::warning(0, "FakeTrackingModule Error.", "If FakeTrackingModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
        return false;
    } else {
        if( ( n = XmlCommon::getParameterNode("fileName", config) ).isNull() ) {
            QMessageBox::warning(0, "FakeTrackingModule Error.", "If FakeSegmentationModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
            return false;
        } else
            m_fileName = XmlCommon::getParameterValue(n);

        if( ( n = XmlCommon::getParameterNode("objectPosition", config) ).isNull() ){
            m_pos = Bottom;
            addParameter("objectPosition", "Bottom", "QString");
        }
        else{
            m_pos = SceneModel::setPosition(XmlCommon::getParameterValue(n).toStdString());
            addParameter("objectPosition", XmlCommon::getParameterValue(n), "QString");
        }
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("fileName", m_fileName, "QString");
    return true;
}

bool FakeTrackingModule::updateParameters(){
    parameter *op, *fn;
    op = getParameter("objectPosition");
    fn = getParameter("fileName");

    if( op == 0 || fn == 0 ||
            fn->value.isNull() || fn->value.isEmpty() || (op->value != "Bottom" &&
            op->value != "MiddleCenter" && op->value != "Left" && op->value != "Right" &&
            op->value != "Top" && op->value != "TopLeft" && op->value != "BottomLeft" &&
            op->value != "TopRight" && op->value != "BottomRight"))
        return false;
    m_fileName = fn->value;
    m_pos = SceneModel::setPosition(op->value.toStdString());
    return true;
}

//Function executed at each frame
bool FakeTrackingModule::run() {

    QDomElement e;
    Blob b;
    Object o;
    int X1,Y1,X2,Y2,id;
    double X,Y,x,y,Vx,Vy;
    ObjectType type;
    std::map<int,Object>::iterator it, end_it;

    //VideoAnalysis::appendToLog("Tracking: Se puede!! :D");
    if(!currentNode.isNull()) {
        do {
            e = currentNode.toElement();
            if( !e.isNull() ) {
                if( e.tagName() == "Image" ) {
                    currentFrame = e.attribute( "IDimage", "" ).toInt();
                } else {
                    QMessageBox::warning(0, "Error reading ground-truth.", "The ground-truth file '" + m_fileName + "', at line" + e.lineNumber() + ", contains an element different than Image, inside VideoAnalysis. Execution will be aborted.");
                    return false;
                }
            } else {
                    QMessageBox::warning(0, "Error reading ground-truth.", "Error in the ground-truth file '" + m_fileName + "' at line" + e.lineNumber() + ". Execution will be aborted.");
                    return false;
            }
            if(currentFrame < m_data->frameNumber)
                currentNode = currentNode.nextSibling();
            //std::cout << "frameNumber: " << m_data->frameNumber << std::endl;
            //std::cout << "currentFrame: " << currentFrame << std::endl;
        } while(currentFrame < m_data->frameNumber && !currentNode.isNull());

        //If frame is found, extract blobs:
        if(!currentNode.isNull() && currentFrame == m_data->frameNumber) {
            QDomNode n = currentNode.firstChild();
            end_it = m_data->fake_objects.end();
            while(!n.isNull()) {
                e = n.toElement();
                if( !e.isNull() ) {
                    if( e.tagName() == "Object" ) {
                        //These coordinates are X horizontal, Y vertical...
                        X1 = e.attribute( "beginX", "" ).toInt() - 4;
                        Y1 = e.attribute( "beginY", "" ).toInt() - 2;
                        X2 = e.attribute( "endX", "" ).toInt() - 4;
                        Y2 = e.attribute( "endY", "" ).toInt() - 2;
                        id = e.attribute( "IDobject", "" ).toInt();
                        type = Object::extractType( e.attribute( "Nameobject", ""));
                        //Vertical
                        if(Y1 < Y2) {
                            BLOB_YTOP(&b) = Y1;
                            BLOB_YBOTTOM(&b) = Y2;
                            BLOB_HEIGHT(&b) = Y2 - Y1 + 1;
                        } else {
                            BLOB_YTOP(&b) = Y2;
                            BLOB_YBOTTOM(&b) = Y1;
                            BLOB_HEIGHT(&b) = Y1 - Y2 + 1;
                        }
                        //Horizontal
                        if(X1 < X2) {
                            BLOB_XLEFT(&b) = X1;
                            BLOB_XRIGHT(&b) = X2;
                            BLOB_WIDTH(&b) = X2 - X1 + 1;
                        } else {
                            BLOB_XLEFT(&b) = X2;
                            BLOB_XRIGHT(&b) = X1;
                            BLOB_WIDTH(&b) = X1 - X2 + 1;
                        }

                        SceneModel::getXY(m_pos,X,Y,b);
                        SceneModel::imgToWorldCoordsGivenHeight(m_data->sceneModel->p_matrix, X, Y, 0.0, &x, &y);
                        if( (it = m_data->fake_objects.find(id)) != end_it) { //Existing object
                            (*it).second.info2D = b;
                            Vx = x - ((*it).second.trajectory.back()).x;
                            Vy = y - ((*it).second.trajectory.back()).y;
                            Info3D i;
                            i.x = x; i.y = y; i.Vx = Vx; i.Vy = Vy;
                            (*it).second.trajectory.push_back(i);
                        } else { //new object
                            o.info2D = b;
                            o.label = id;
                            o.type = type;
                            Info3D i;
                            i.x = x; i.y = y; i.Vx = 0.0; i.Vy = 0.0;
                            o.trajectory.push_back(i);
                            m_data->fake_objects[id] = o;
                        }
                    } else {
                        QMessageBox::warning(0, "Error reading ground-truth.", "The ground-truth file '" + m_fileName + "', at line" + e.lineNumber() + ", contains an element different than Object, inside Image. Execution will be aborted.");
                        return false;
                    }
                } else {
                    QMessageBox::warning(0, "Error reading ground-truth.", "Error in the ground-truth file '" + m_fileName + "' at line" + e.lineNumber() + ". Execution will be aborted.");
                    return false;
                }
                n = n.nextSibling();
            }
            currentNode = currentNode.nextSibling();
        }
    }

    return true;
}

bool FakeTrackingModule::init() {
    line = 1;
    xmlGT = new QDomDocument( "VideoAnalysis" );
    QFile file( m_fileName );
    if( !file.open(QIODevice::ReadOnly) ) {
        QMessageBox::warning(0, "Error opening file.", "The ground-truth file '" + m_fileName + "' could not be opened. Execution will be aborted.");
        return false;
    }

    if( !xmlGT->setContent( &file ) ) {
        QMessageBox::warning(0, "Error opening file.", "XML content in the ground-truth file '" + m_fileName + "' could not be extracted. Execution will be aborted.");
        file.close();
        return false;
    }
    file.close();

    root = xmlGT->documentElement();
    if( root.tagName() != "VideoAnalysis" ) {
        QMessageBox::warning(0, "Error reading XML information.", "XML content in the ground-truth file '" + m_fileName + "' does not correspond to a ground-truth description. Execution will be aborted.");
        return false;
    }

    currentNode = root.firstChild();

    return true;
}





