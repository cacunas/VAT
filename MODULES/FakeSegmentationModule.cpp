#include "FakeSegmentationModule.h"
#include "image_display.h"

#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>

FakeSegmentationModule::FakeSegmentationModule(Datapool *i_data): ModuleInterface(i_data) {}

FakeSegmentationModule::~FakeSegmentationModule() {}

//Set module configuration parameters
bool FakeSegmentationModule::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        QMessageBox::warning(0, "FakeSegmentationModule Error.", "If FakeSegmentationModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
        return false;
    } else {
        if( ( n = XmlCommon::getParameterNode("fileName", config) ).isNull() ) {
            QMessageBox::warning(0, "FakeSegmentationModule Error.", "If FakeSegmentationModule is called the parameter 'fileName' must be compulsory defined to find the XML file containing the information related to groundtruth.");
            return false;
        } else
            m_fileName = XmlCommon::getParameterValue(n);
    }

    //Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
    addParameter("fileName", m_fileName, "QString");
    return true;
}

bool FakeSegmentationModule::updateParameters(){
    parameter *fn;
    fn = getParameter("fileName");

    if( fn == 0 || fn->value.isNull() || fn->value.isEmpty())
        return false;
    m_fileName = fn->value;
    return true;
}

//Function executed at each frame
bool FakeSegmentationModule::run() {
    QDomElement e;
    Blob b;
    int X1,Y1,X2,Y2;

    //VideoAnalysis::appendToLog("Segmentation: Se puede!! :D");
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

            m_data->blobs.clear();
            QDomNode n = currentNode.firstChild();
            while(!n.isNull()) {
                e = n.toElement();
                if( !e.isNull() ) {
                    if( e.tagName() == "Object" ) {
                        //These coordinates are X horizontal, Y vertical...
                        X1 = e.attribute( "beginX", "" ).toInt() - 4;
                        Y1 = e.attribute( "beginY", "" ).toInt() - 4;
                        X2 = e.attribute( "endX", "" ).toInt() - 4;
                        Y2 = e.attribute( "endY", "" ).toInt() - 4;

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

                        m_data->blobs.push_back(b);
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


    //Set background image:
    if(m_data->bgImage == NULL)
        m_data->bgImage = new QImage(*m_data->currentImage);

    //QImage temp(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_ARGB32);
    if(m_data->fgImage == NULL)
        m_data->fgImage = new QImage(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_ARGB32);


    QPainter painter;
    //painter.begin(&temp);
    //while(!painter.begin(m_data->fgImage)) {painter.end();}
    painter.begin(m_data->fgImage);
    //QColor( int r, int g, int b, int a = 255 )
    painter.setBrush(QColor(0, 0, 0, 255));//negro
    painter.setPen(QColor(0, 0, 0, 255)); //negro
    painter.drawRect(0, 0, m_data->currentImage->width(),m_data->currentImage->height());

    if(!m_data->blobs.empty()) {
        std::vector<Blob>::iterator end_it = m_data->blobs.end(), iter;

        painter.setBrush(QColor(255, 255, 255, 255));//blanco
        painter.setPen(QColor(255, 255, 255, 255));//blanco
        for(iter=m_data->blobs.begin();iter != end_it; iter++) {
            //std::cout << "Rect: " << (*iter).Yl << ", " << (*iter).Xu << ", "
            //          << (*iter).W - 1 << ", " << (*iter).H - 1 << std::endl;
            painter.drawRect(BLOB_XLEFT(&*iter), BLOB_YTOP(&*iter),
                             BLOB_WIDTH(&*iter) - 1, BLOB_HEIGHT(&*iter) - 1);
        }
    }
    painter.end();

    m_data->fgImage->convertToFormat(QImage::Format_Indexed8, *MainWindow::grayScaleTable, Qt::MonoOnly | Qt::OrderedDither);

    //temp.convertToFormat(QImage::Format_Indexed8, *MainWindow::grayScaleTable, Qt::MonoOnly | Qt::OrderedDither);
    //m_data->fgImage = new QImage(temp);


    return true;
}

bool FakeSegmentationModule::init() {
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





