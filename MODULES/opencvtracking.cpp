#include "ModuleInterface.h"
#include "opencvtracking.h"
#include "VideoAnalysis.h"
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/legacy/compat.hpp>
#include "src/blob.h"
#include "src/object.h"
#include "src/blobtrackingautoc.h"


using namespace cv;
using namespace std;

OpencvTracking::OpencvTracking(Datapool *i_data): ModuleInterface(i_data) {}

OpencvTracking::~OpencvTracking() {}

bool OpencvTracking::setParameters(QDomNode& config){
    //m_pos = Bottom;
    QDomNode n;

    CvBlobTrackerAutoParam1     param;
    param.pBTGen = NULL;
    param.pBTPP = NULL;
    param.pBTA = NULL;
    param.FGTrainFrames = 0;
    param.UsePPData = 0;
    debug = false;

    if(config.isNull()) { //Parameter set for module not defined
        param.pFG = NULL;
        param.pBD = cvCreateBlobDetectorCC();
        param.pBT = cvCreateBlobTrackerCCMSPF();

        pTracker = new CvBlobTrackerAutoc(&param);
    } else {
        if( ( n = XmlCommon::getParameterNode("FGDetector", config) ).isNull() )
            param.pFG = NULL;
        else{
             if((XmlCommon::getParameterValue(n)) == "none" ) param.pFG = NULL;
             //else if((getParameterValue(n)) == "FDG" ) param.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_FGD, NULL);
             //else if((getParameterValue(n)) == "FDGS" ) param.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_FGD_SIMPLE, NULL);
             //else if((getParameterValue(n)) == "MOG" ) param.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_MOG, NULL);
             else param.pFG = NULL;
        }

        if( ( n = XmlCommon::getParameterNode("BlobDetector", config) ).isNull() )
            param.pBD = cvCreateBlobDetectorCC();
        else{
            if((XmlCommon::getParameterValue(n)) == "Simple" ) param.pBD = cvCreateBlobDetectorSimple();
            else if((XmlCommon::getParameterValue(n)) == "CC" ) param.pBD = cvCreateBlobDetectorCC();
            else param.pBD = cvCreateBlobDetectorCC();
        }

        if( ( n = XmlCommon::getParameterNode("BlobTracker", config) ).isNull() )
            param.pBT = cvCreateBlobTrackerCCMSPF();
        else{
            if((XmlCommon::getParameterValue(n)) == "CCMSPF" ) param.pBT = cvCreateBlobTrackerCCMSPF();
            else if((XmlCommon::getParameterValue(n)) == "CC" ) param.pBT = cvCreateBlobTrackerCC();
            else if((XmlCommon::getParameterValue(n)) == "MS" ) param.pBT = cvCreateBlobTrackerMS();
            else if((XmlCommon::getParameterValue(n)) == "MSFG" ) param.pBT = cvCreateBlobTrackerMSFG();
            else if((XmlCommon::getParameterValue(n)) == "MSPF" ) param.pBT = cvCreateBlobTrackerMSPF();
            else param.pBT = cvCreateBlobTrackerCCMSPF();
        }

        pTracker = new CvBlobTrackerAutoc(&param);

    }

    // Valores necesarios para crear objetos (arbitrios)
    int m_BlobBufferSize = 4;
    MobileObject2D::secDiffSequence = new double[m_BlobBufferSize];
    MobileObject2D::coolingValue    = new double[m_BlobBufferSize];
    MobileObject2D::secDiffToCurrent = new double[m_BlobBufferSize];
    memset(MobileObject2D::secDiffSequence,  1, sizeof(double)*m_BlobBufferSize);
    memset(MobileObject2D::coolingValue,     1, sizeof(double)*m_BlobBufferSize);
    memset(MobileObject2D::secDiffToCurrent, 1, sizeof(double)*m_BlobBufferSize);
    MobileObject2D::coolingValue[0] = 1.0;


    return true;
}

bool OpencvTracking::updateParameters(){
    return true;
}


bool OpencvTracking::init(){

    if(!pTracker){
        qWarning() << "Error while creating the tracker";
        return false;
    }

    return true;
}


bool OpencvTracking::run(){

/* Se eliminan los blobs de la iteracion pasada */
    m_data->blobs.clear();

/* Transformacion de imagenes de QT a OpenCV */
    Mat c = qImage2Mat(*m_data->fgImage);
    Mat current_fg;
    cvtColor(c,current_fg,CV_RGB2GRAY);
    Mat current_img = qImage2Mat(*m_data->currentImage);
    IplImage pMask = current_fg;
    IplImage pImg = current_img;


/* Proceso de deteccion y tracking */
    pTracker->Process(&pImg,&pMask);

/* Transformacion de blobs y tracks de opencv a los del VAT */
    for(int i=pTracker->GetBlobNum();i>0;i--){
        CvBlob* blob = pTracker->GetBlob(i-1);

        // Llena blobs
        Blob B(blob->x - blob->w/2, blob->x + blob->w/2, blob->y - blob->h/2, blob->y + blob->h/2);
        m_data->blobs.push_back(B);


        // Si existe, agrega ruta al objeto
        deque<SpMobileObject2D>::iterator it;
        bool updated = false;

        for(it = m_data->objects2D.begin(); it != m_data->objects2D.end();it++ ){
            SpMobileObject2D obj = *it;
            if(blob->ID == (int)obj->getMobileId()){
                obj->updateMobilePath(&B);
                updated = true;
            }
        }
        // Si no, agrega un nuevo objeto
        if(!updated){
            SpMobileObject2D newMobile(new MobileObject2D());
            newMobile->setNewMobileFromBlob(&B,blob->ID,blob->ID);
            newMobile->setNumberOfFramesNotSeen(0);
            m_data->objects2D.push_back(newMobile);

        }
    }


    // Revisa objetos inactivos.
    deque<SpMobileObject2D>::iterator it;
    for(it = m_data->objects2D.begin(); it != m_data->objects2D.end();++it ){
        SpMobileObject2D obj = *it;

        bool active = false;
        for(int i=pTracker->GetBlobNum();i>0;i--){
            CvBlob* blob = pTracker->GetBlob(i-1);
            if(blob->ID == (int)obj->getMobileId()){
                active = true;
                break;
            }
        }

        if(!active){
            obj->incrementNumberOfFramesNotSeen();
        }

    }

    //Se eliminan los objetos que no hallan sido visto despues de x frames
    for(it = m_data->objects2D.begin(); it != m_data->objects2D.end();++it ){
        SpMobileObject2D obj = *it;

        if(obj->getNumberOfFramesNotSeen() > 0){
            if(it != m_data->objects2D.end()){
                m_data->objects2D.erase(it);
                break;
            }
            m_data->objects2D.erase(it);
        }
    }

    if(debug) OpencvDebug(pImg);

    return true;
}


/* Pasa de qImage a Mat, devuelve imagen de 3 canales de 8 bits. */
/*   Utiliza memoria compartida */
Mat OpencvTracking::qImage2Mat(const QImage& qimage) {

    QImage img;
    if( (qimage.format() != QImage::Format_RGB32))
        img = qimage.convertToFormat(QImage::Format_RGB32);
    else
        img = qimage;

    Mat mat = Mat(img.height(),
                  img.width(),
                  CV_8UC4,
                  (uchar*)img.bits(),
                  img.bytesPerLine()
                  );

    Mat mat2 = Mat(mat.rows, mat.cols, CV_8UC3 );
    int from_to[] = { 0,0,  1,1,  2,2 };
    mixChannels( &mat, 1, &mat2, 1, from_to, 3 );

    return mat2;
};

/* Pasa de Mat a qImage, devuelve imagen en formato RGB888. */
/*   Utiliza memoria compartida */
QImage OpencvTracking::Mat2QImage(const Mat& mat) {
    Mat rgb;

    cvtColor(mat, rgb, CV_BGR2RGB);
    return QImage(
            (const unsigned char*)(rgb.data),
            rgb.cols,
            rgb.rows,
            QImage::Format_RGB888
            );
};


/* Muestra el tracking en una ventana independiente de OpenCV */
void OpencvTracking::OpencvDebug(IplImage & pImg){
    char        str[1024];
    int         line_type = CV_AA;   // Change it to 8 to see non-antialiased graphics.
    CvFont      font;
    int         i;
    IplImage*   pI = cvCloneImage(&pImg);

    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 0.7, 0.7, 0, 1, line_type );

    for(i=pTracker->GetBlobNum(); i>0; i--)
    {
        CvSize  TextSize;
        CvBlob* pB = pTracker->GetBlob(i-1);
        CvPoint p = cvPoint(cvRound(pB->x*256),cvRound(pB->y*256));
        CvSize  s = cvSize(MAX(1,cvRound(CV_BLOB_RX(pB)*256)), MAX(1,cvRound(CV_BLOB_RY(pB)*256)));
        int c = cvRound(255*pTracker->GetState(CV_BLOB_ID(pB)));

        cvEllipse( pI,
                  p,
                  s,
                  0, 0, 360,
                  CV_RGB(c,255-c,0), cvRound(1+(3*0)/255), CV_AA, 8 );

        p.x >>= 8;
        p.y >>= 8;
        s.width >>= 8;
        s.height >>= 8;
        sprintf(str,"%03d",CV_BLOB_ID(pB));
        cvGetTextSize( str, &font, &TextSize, NULL );
        p.y -= s.height;
        cvPutText( pI, str, p, &font, CV_RGB(0,255,255));
        {
            const char* pS = pTracker->GetStateDesc(CV_BLOB_ID(pB));

            if(pS)
            {
                char* pStr = strdup(pS);
                char* pStrFree = pStr;

                while (pStr && strlen(pStr) > 0)
                {
                    char* str_next = strchr(pStr,'\n');

                    if(str_next)
                    {
                        str_next[0] = 0;
                        str_next++;
                    }

                    p.y += TextSize.height+1;
                    cvPutText( pI, pStr, p, &font, CV_RGB(0,255,255));
                    pStr = str_next;
                }
                free(pStrFree);
            }
        }

    }   /* Next blob. */;

    cvNamedWindow( "Tracking", 0);
    cvShowImage( "Tracking",pI );

    cvReleaseImage(&pI);
   /* Draw all information about test sequence. */

    //cvReleaseImage(&pImg);


    waitKey(10);
}
