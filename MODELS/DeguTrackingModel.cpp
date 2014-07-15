#include "DeguTrackingModel.h"
#include "Datapool.h"
#include "xmlcommon.h"

DeguTrackingModel::DeguTrackingModel():
Blob2DFromBGSubstractionModel() { }

DeguTrackingModel::~DeguTrackingModel() { }

void DeguTrackingModel::initDynamicsAttributes() {
    //Dynamics for W and H of blob will not consider acceleration
    dynamics.dynamics["W"] = ReliabilityDynamicsAttribute(true,true,false);
    dynamics.dynamics["H"] = ReliabilityDynamicsAttribute(true,true,false);
    dynamics.dynamics["X"] = ReliabilityDynamicsAttribute();
    dynamics.dynamics["Y"] = ReliabilityDynamicsAttribute();

    std::fill_n(Hues, 36, 0);
    pSize=0;
    pDiscard=0;
    nFrames=0;
}

//sets activation criteria for each model: reliability on input (distance, bad data),
//   needs (occlusion, priority),
//sets priority of models (hierarchy).
void DeguTrackingModel::setParameters(QDomNode &i_parameters) {

    if(i_parameters.isNull()) { //Parameter set for module not defined
        m_minimalAttributeVelocityReliability = 0.1;
        m_minimalAttributeAccelerationReliability = 0.1;
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

        if( ( n = XmlCommon::getParameterNode("PixelAcuity", i_parameters) ).isNull() )
            m_pixelAcuity = 3;
        else
            m_pixelAcuity = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("FirstColorRange", i_parameters) ).isNull() )
            m_first_color_range = 3;
        else
            m_first_color_range = XmlCommon::getParameterValue(n).toInt();

        if( ( n = XmlCommon::getParameterNode("SecondColorRange", i_parameters) ).isNull() )
            m_second_color_range = 10;
        else
            m_second_color_range = XmlCommon::getParameterValue(n).toInt();

    }

}




void DeguTrackingModel::makeHistoryGraph(){

        int //w = result->width(),
            //h = result->height(),
            b1 = m_data->currentImage->bytesPerLine(),
            b2 = m_data->fgImage->bytesPerLine(),
            b3 = m_data->rFgImage->bytesPerLine();

        uchar *currentImageBits = m_data->currentImage->bits();
        uchar *fgImageBits=m_data->fgImage->bits();
        uchar *rfgImageBits=m_data->rFgImage->bits();


        int i, j, k, aux;

        int width= dynamics.dynamics["W"].att.value;
        int height= dynamics.dynamics["H"].att.value;

        int iInit= dynamics.dynamics["Y"].att.value-height/2;
        int jInit= dynamics.dynamics["X"].att.value-width/2;

        int iEnd= dynamics.dynamics["Y"].att.value+height/2;
        int jEnd= dynamics.dynamics["X"].att.value+width/2;

        int hue;



        for (i = iInit; i <= iEnd; i++){
            for (j = jInit; j <= jEnd; j++){
                aux = i*b1 + j;
                if(fgImageBits[aux]==255){
                    uchar pixelR=currentImageBits[aux+2];
                    uchar pixelG=currentImageBits[aux+1];
                    uchar pixelB=currentImageBits[aux];

                    uchar max=pixelR;
                    uchar min=pixelR;
                    uchar minIndice=0;
                    uchar maxIndice=0;

                    if(pixelG>max){
                        max=pixelG;
                        maxIndice=1;
                    }
                    if(pixelG<min){
                        min=pixelG;
                        minIndice=1;
                    }
                    if(pixelB>max){
                        max=pixelB;
                        maxIndice=2;
                    }
                    if(pixelB<min){
                        min=pixelB;
                        minIndice=2;
                    }
                    if(max - min >15){

                        if(maxIndice==0){
                            if(pixelG>pixelB){
                                hue=60*(pixelG-pixelB)/(max-min);
                            }else{
                                hue=60*(pixelG-pixelB)/(max-min)+360;
                        }



                        }else if(maxIndice==1){
                            hue=60*(pixelB-pixelR)/(max-min)+120;
                        }else if(maxIndice==2){
                            hue=60*(pixelR-pixelG)/(max-min)+240;
                        }

                        Hues[hue]++;

                        //pixels.insert();
                        //pixels.insert(pixelR);
                        //pixels.insert(pixelG);
                        //pixels.insert(pixelB);
                    }
                }
            }
        }





}




void DeguTrackingModel::checkHistory(){

    int pSize=pixels.size();

    if(pSize>1500){
        //buscar maximo
        int max=0;
        int maxIndex=0;
        for (int i =0;i < 360; i++){

            int value=0;

            int k = i +10;
            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){
                    value += Hues[t];

                }

            }

            for(int j=i; j<k; j++){

                value += Hues[j];
            }

            if(value > max){
                max=value;
                maxIndex=i;
            }
        }
        //buscar lado derecho
        int indice = maxIndex;

        indice--;
        if(indice<0){
            indice=359;
        }
        while(indice!=maxIndex){

            int value=0;

            int k = indice +10;
            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){
                    value += Hues[t];

                }

            }

            for(int j=indice; j<k; j++){

                value += Hues[j];
            }

            if(value<max*0.2 ){

                break;
            }

            indice--;
            if(indice<0){
                indice=359;
            }
        }
        //buscar lado izquierdo
        int iColor= (indice+5)%360;


        indice = maxIndex;

        indice++;
        if(indice>359){
            indice=0;
        }

        while(indice!=maxIndex){

            int value=0;

            int k = indice +10;

            if (k>=360){
                int h= k-360;
                k=360;

                for (int t=0; t<=h; t++){

                    value += Hues[t];

                }

            }

            for(int j=indice; j<k; j++){

                value += Hues[j];
            }

            if(value<max*0.2 ){

                break;
            }

            indice++;

            if(indice>359){
                indice=0;
            }
        }

        int dColor= (indice+5)%360;

        printf("maximo %d, derecha %d, izquierda %d ", maxIndex, iColor, dColor);
    }

}

//updates every activated model
void DeguTrackingModel::updateDynamics() {
    Blob2DFromBGSubstractionModel::updateDynamics();

    makeHistoryGraph();
    checkHistory();
    //shapeAnalysis();

    std::map<QString, DeguTrackingModel *>::iterator modeloMain;
    modeloMain= m_data->modelsToFollow.find("main");
    if(modeloMain!=m_data->modelsToFollow.end()){
        if(modeloMain->second->nFrames<nFrames)
            m_data->modelsToFollow["main"]=this;
    }else{
        m_data->modelsToFollow["main"]=this;

    }

}


void DeguTrackingModel::copy(SpReliabilitySingleModelInterface model) {
    ReliabilitySingleModelInterface *i = &*model;
    DeguTrackingModel *ii = (DeguTrackingModel *)i;
    m_minimalAttributeVelocityReliability = ii->m_minimalAttributeVelocityReliability;
    m_minimalAttributeAccelerationReliability = ii->m_minimalAttributeAccelerationReliability;
    m_pixelAcuity = ii->m_pixelAcuity;
    for (int s=0; s<36; s++){
        Hues[s]=ii->Hues[s];
    }
    pSize = ii->pSize;
    pDiscard = ii->pDiscard;
    nFrames = ii->nFrames;
    m_first_color_range=ii->m_first_color_range;
    m_second_color_range=ii->m_second_color_range;
    deguShape=ii->deguShape;
}

//Extracted from dynamics
void DeguTrackingModel::makeHistoryGraph_Dynamics(){

//m_data->fgImage;m_data->rFgImage;

        nFrames++;

        int //w = result->width(),
            //h = result->height(),
            b1 = m_data->currentImage->bytesPerLine(),
            b2 = m_data->fgImage->bytesPerLine(),
            b3 = m_data->rFgImage->bytesPerLine();

        int w = m_data->fgImage->width(),
            h = m_data->fgImage->height();

        uchar *currentImageBits;
        //uchar *fgImageBits;

        currentImageBits = m_data->currentImage->bits();
        uchar *fgImageBits=m_data->fgImage->bits();
        uchar *rfgImageBits=m_data->rFgImage->bits();


        int i, j, k, aux;

        int width= dynamics.dynamics["W"].att.value;
        int height= dynamics.dynamics["H"].att.value;

        int iInit= dynamics.dynamics["Y"].att.value-height/2;
        int jInit= dynamics.dynamics["X"].att.value-width/2;

        int iEnd= dynamics.dynamics["Y"].att.value+height/2;
        int jEnd= dynamics.dynamics["X"].att.value+width/2;

        int hue;

        if(jEnd>=w)
            jEnd=w-1;
        if(iEnd>=h)
            iEnd=h-1;
        if(jInit<0)
            jInit=0;
        if(iInit<0)
            iInit=0;

        int auxInitasdf=width;
        auxInitasdf++;
        for (i = iInit; i <= iEnd; i++){
            for (k = jInit; k <= jEnd; k++){
        //for (i = 0; i < h; i++){
          //  for(j=0, k=0; k<w; j+=4, k++){
                j=k*4;
                aux = i*b2 + k;

                if(fgImageBits[aux]==255){
                    int analisis=aux;
                    int analisisi=i;
                    int analisisj=k;

                    int analisisValorBack=fgImageBits[aux];

                    int aux2 = i*b1 + j;
                    uchar pixelR=currentImageBits[aux2+2];
                    uchar pixelG=currentImageBits[aux2+1];
                    uchar pixelB=currentImageBits[aux2];

                    uchar max=pixelR;
                    uchar min=pixelR;
                    uchar minIndice=0;
                    uchar maxIndice=0;

                    if(pixelG>max){
                        max=pixelG;
                        maxIndice=1;
                    }
                    if(pixelG<min){
                        min=pixelG;
                        minIndice=1;
                    }
                    if(pixelB>max){
                        max=pixelB;
                        maxIndice=2;
                    }
                    if(pixelB<min){
                        min=pixelB;
                        minIndice=2;
                    }
                    if(max - min >15 && ((max+min)/2>00)){

                        if(maxIndice==0){
                            if(pixelG>pixelB){
                                hue=(60*(pixelG-pixelB)/(max-min));
                            }else{
                                hue=(60*(pixelG-pixelB)/(max-min)+360);
                        }



                        }else if(maxIndice==1){
                            hue=(60*(pixelB-pixelR)/(max-min)+120);
                        }else if(maxIndice==2){
                            hue=(60*(pixelR-pixelG)/(max-min)+240);
                        }


                        hue=hue/10;
                        Hues[hue]++;
                        pSize++;
                        //pixels.insert();
                        //pixels.insert(pixelR);
                        //pixels.insert(pixelG);
                        //pixels.insert(pixelB);
                    }else{
                        pDiscard++;

                    }
                }
            }
        }


}


void DeguTrackingModel::checkHistory_Dynamics(){

    if(pSize>1500){
        //buscar maximo
        int max=0;
        int maxIndex=0;
        for (int i =0;i < 36; i++){

            int value=Hues[i];

            if(value > max){
                max=value;
                maxIndex=i;
            }
        }
        //buscar lado derecho
        int indice = maxIndex;

        indice--;
        if(indice<0){
            indice=35;
        }
        while(indice!=maxIndex){

            int value=Hues[indice];

            if(value<max*0.2){
                break;
            }

            indice--;

            if(indice<0){
                indice=35;
            }

        }
        //buscar lado izquierdo
        int iColor= (indice)%36;


        indice = maxIndex;

        indice++;
        if(indice>35){
            indice=0;
        }

        while(indice!=maxIndex){

            int value=Hues[indice];

            if(value<max*0.2){
                break;
            }

            indice++;

            if(indice>35){
                indice=0;
            }
        }

        int dColor= (indice)%36;

        //printf("maximo %d, derecha %d, izquierda %d ", maxIndex, iColor, dColor);
    }


    if(pSize!=0){

        if(m_first_color_range<=m_second_color_range){

            int sumaCorrecto=0;
            for (int i =0;i < 36; i++){

                int value=Hues[i];

                if(i >= m_first_color_range && i <= m_second_color_range){
                    sumaCorrecto+=value;
                }
            }

            colorAccuracy=(float)sumaCorrecto/(float)pSize*100;
        }

        if(m_first_color_range>m_second_color_range){

            int sumaCorrecto=0;
            for (int i =0;i < 36; i++){

                int value=Hues[i];

                if(i >= m_first_color_range || i <= m_second_color_range){
                    sumaCorrecto+=value;
                }
            }

            colorAccuracy=(float)sumaCorrecto/(float)pSize*100;
        }

    }


}


bool DeguTrackingModel::shapeAnalysis_Dynamics(){

    int w = m_data->fgImage->width(), h = m_data->fgImage->height();

    cv::Rect roi;

    roi.x=dynamics.dynamics["X"].att.value-dynamics.dynamics["W"].att.value/2;
    roi.y=dynamics.dynamics["Y"].att.value-dynamics.dynamics["H"].att.value/2;
    roi.width=dynamics.dynamics["W"].att.value;
    roi.height=dynamics.dynamics["H"].att.value;

    if(roi.x>=w)
            roi.x=w-1;
    if(roi.y>=h)
            roi.y=h-1;
    if(roi.x<0)
            roi.x=0;
    if(roi.y<0)
            roi.y=0;
    if(roi.width<0)
            roi.width=0;
    if(roi.height<0)
            roi.height=0;
    if(roi.height+roi.y>=h)
        roi.height=h-roi.y;

    cv::Mat totalImage(h, w, CV_8UC1);

    uchar  *fg_p = m_data->fgImage->bits();
    int bl = m_data->fgImage->bytesPerLine();

    memcpy(totalImage.data, fg_p, h*bl);

    cv::Mat f(totalImage, roi);

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                           cv::Size( 3, 3 ),
                                           cv::Point( 2, 2 ) );
/*
    cv::Mat f1(h, w, CV_8UC1);

    //int bl = fg->bytesPerLine();
    //uchar *fg_p = fg->bits();

    memcpy(f1.data, fg_p, h*bl);

    //Reduce bad detections
    //cv::erode(f, f, element, cv::Point(-1,-1), 1);
    //cv::dilate(f1, f1, element);
    //cv::dilate(f1, f1, element);

    memcpy(fg_p, f1.data, h*bl);*/

    cv::dilate( f, f, element );



    cv::Mat border_aux(f.size(), CV_8UC1);
    cv::Canny(f,border_aux, 50,100, 3);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat border_copy(border_aux.size(), CV_8UC1);
    border_aux.copyTo(border_copy);
    cv::findContours(border_copy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );


    std::vector<cv::Point> big_contour;
    std::vector<cv::Point> hull;
    if(contours.size() > 0) {
        //Group found contours in one big contour
        for(int i=0;i<contours.size(); i++) {
            if(hierarchy[i][2] < 0) { // No parent, so it's parent
                if(big_contour.empty())
                    big_contour = contours[i];
                else
                    big_contour.insert( big_contour.end(), contours[i].begin(), contours[i].end());
            }
        }
        //Get initial convex hull
        cv::convexHull( big_contour, hull, false );


    } else {
        return true;
    }
    if(hull.size() == 0) {
        return true;
    }


    //cv::erode(aux, aux, element);
    //cv::dilate(aux, aux, element,cv::Point(-1,-1),2);
    //cv::erode(aux, aux, element);

    SpHullModel newHull(new HullModel());

    newHull->local_hull = hull;
    //newHull->off_x = 0;
    //newHull->off_y = 0;
    newHull->off_x = roi.x;
    newHull->off_y = roi.y;
    //newHull->off_x = j0;
    //newHull->off_y = i0;
    //newHull->id = (*it)->mobile_id;
    newHull->id = NULL;

    int w0 = m_data->fgImage->width(),
        h0 = m_data->fgImage->height();

    //Get principal/minor axis
    std::vector<cv::Point2f> data_aux(h0*w0);
    float mean_x = 0, mean_y = 0;
    int count = 0;



    for(int i=0; i<h0; i++)
        for(int j=0; j<w0; j++)
            if(cv::pointPolygonTest(hull, cv::Point2f(j, i), true) > - 1) {
                data_aux[count++] = cv::Point2f(j, i);
                mean_x += j;
                mean_y += i;
            }
    //data_aux.resize(count);
    //cv::Mat data(2, count, CV_32FC1, &data_aux.front());
    cv::Mat data(2, count, CV_32FC1);
    cv::Point2f x;
    for(int i=0; i<count; i++) {
        data.at<float>(0,i) = data_aux[i].x;
        data.at<float>(1,i) = data_aux[i].y;
    }

    //cv::Mat data();
    mean_x /= count;
    mean_y /= count;
    cv::Mat mean(2, 1, CV_32FC1);
    mean.at<float>(0) = mean_x;
    mean.at<float>(1) = mean_y;

    //2. perform PCA
    cv::PCA pca(data, mean, CV_PCA_DATA_AS_COL, 1);
    //result is contained in pca.eigenvectors (as row vectors)
    //std::cout << pca.eigenvectors << std::endl;

    //3. get angle of principal axis
    float dx = pca.eigenvectors.at<float>(0, 0),
          dy = pca.eigenvectors.at<float>(0, 1),
          scale = 40.0;
    cv::Point3f rline;
    cv::Point2f r1, r2;

    //Get line general form from principal component
    getGeneralLineForm(cv::Point2f(mean_x, mean_y),
                       cv::Point2f(mean_x + dx*scale, mean_y + dy*scale),
                       rline);
    //Get segment from line
    int n1, n2;
    getContourToLineIntersection(hull, rline, r1, r2, &n1, &n2);

    //Get pixel intersections for normals
    std::vector< segment2D<float> > &segs = newHull->segs;
    std::vector< segment2D<float> > &hull_segs = newHull->hull_segs;

/*
    cv::Rect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = w0;
    roi.height = h0;*/

    cv::Mat aux(f);

    //Get segments of movement normal to principal axis. Also reorders r1 and r2 in
    //coherence with segments order
    getNormalIntersections(aux, roi, hull, r1, r2, n1, n2, dx, dy, segs, hull_segs);

    newHull->axis1 = r1;
    newHull->axis2 = r2;


    //Set new representation
    //m_data->hulls.push_back(newHull);

    deguShape=newHull;

    findHead(newHull);

    return true;




}

void DeguTrackingModel::getNormalIntersections(cv::Mat &f,
                                   cv::Rect &roi,
                                   std::vector<cv::Point> &hull,
                                   cv::Point2f &r1, cv::Point2f &r2,
                                   int n1, int n2,
                                   float dx, float dy,
                                   std::vector< segment2D<float> > &segs,
                                   std::vector< segment2D<float> > &hull_segs) {
//                                   std::vector< cv::Point2f > &) {
    cv::Point2f R1, R2;
    int I1, I2, sy;
    //Normalize search: from leftmost point, if equal, higher one.
    if(r1.x < r2.x) {
        R1 = r1;
        R2 = r2;
        I1 = n1;
        sy = (R1.y <= R2.y) ? 1 : -1;
    } else if(r1.x > r2.x) {
        R1 = r2;
        R2 = r1;
        I1 = n2;
        sy = (R1.y <= R2.y) ? 1 : -1;
    } else {
        sy = 1;
        if(r1.y <= r2.y) {
            R1 = r1;
            R2 = r2;
            I1 = n1;
        } else {
            R1 = r2;
            R2 = r1;
            I1 = n2;
        }
    }
    I2 = I1;

    //Set scale and advances in X and Y to move K pixels, given dx and dy.
    float m_K=2;
    float lx = R2.x - R1.x, ly = R2.y - R1.y,
          D_axis = sqrt(lx*lx + ly*ly),
          a = m_K/sqrt(dx*dx + dy*dy),
          Dx = fabs(a*dx),
          Dy = sy*fabs(a*dy);
    int i, n = (int)D_axis/m_K;
    float x, y;

    segs.resize(n);
    hull_segs.resize(n);


    for(i=0, x=R1.x + Dx, y=R1.y + Dy; i<n; i++, x+=Dx, y+=Dy)
        setForegroundSegment(f, roi, hull, segs[i], hull_segs[i], x, y, dx, dy, I1, I2);

    //Reorder principal axis points, w/r to normal segments
    r1 = R1;
    r2 = R2;
}


void DeguTrackingModel::setForegroundSegment(cv::Mat &f, cv::Rect &roi,
                                                      std::vector<cv::Point> &hull,
                                                      segment2D<float> &seg,
                                                      segment2D<float> &hseg,
                                                      float x, float y,
                                                      float dx, float dy,
                                                      int &I1, int &I2) {
    cv::Point3f rline;
    //Get line form from perpendicular line in x,y
    getGeneralLineForm(x, y, dy, -dx, rline);

    cv::Point2f r1, r2;

//    if(getContourToLineIntersectionIndexed(hull, rline, r1, r2, I1, I2) == 2) {
    if(getContourToLineIntersection(hull, rline, r1, r2) == 2) {
        float xt, yt, Dx = r2.x - r1.x, Dy = r2.y - r1.y, D = sqrt(Dx*Dx + Dy*Dy);
        Dx /= D; Dy /= D; //Unitary increment
        int d, xx, yy, w_mean = 9/2;

        hseg.first.x = r1.x;
        hseg.first.y = r1.y;
        hseg.last.x = r2.x;
        hseg.last.y = r2.y;

        for(xt=r1.x, yt=r1.y, d=0; d<D; xt+=Dx, yt+=Dy, d++) {
            xx = rint(xt); yy = rint(yt);
            if(movementCount(f, 3, yy, xx, roi) > w_mean) {
                seg.first.x = xx;
                seg.first.y = yy;
                break;
            }
        }
        for(xt=r2.x, yt=r2.y, d=0; d<D; xt-=Dx, yt-=Dy, d++) {
            xx = rint(xt); yy = rint(yt);
            if(movementCount(f, 3, yy, xx, roi) > w_mean) {
                seg.last.x = xx;
                seg.last.y = yy;
                break;
            }
        }
    } else
        std::cout << "setForegroundSegment: Error: 2 intersections not found" << std::cout;

}

int DeguTrackingModel::getContourToLineIntersectionIndexed(std::vector<cv::Point> &polygon,
                                         cv::Point3f &rline,
                                         cv::Point2f &r1, cv::Point2f &r2,
                                         int &n1, int &n2) {

    int i, k1 = 0, k2 = 0, n = polygon.size();
    cv::Point2f p1_1, p2_1, p1_2, p2_2, r;
    bool ready1 = false, ready2 = false, got_same = false;

    p1_1 = polygon[n1];
    p2_1 = polygon[(n1+1 == n ? 0 : n1+1)];
    if(lineSegmentIntersection(rline, p1_1, p2_1, r))
        got_same = true;
    n1 = (n1+1 == n ? 0 : n1+1);
    n2 = (n2-1 == -1) ? n-1 : n2-1;

    do {
        if(!ready1) {
            p1_1 = polygon[n1];
            p2_1 = polygon[(n1+1 == n ? 0 : n1+1)];
            if(lineSegmentIntersection(rline, p1_1, p2_1, r1)) {
                ready1 = true;
                if(got_same) {
                    r2 = r;
                    break;
                }
            } else
                n1 = (n1+1 == n ? 0 : n1+1);
            k1++;
        }
        if(!ready2) {
            p1_2 = polygon[n2];
            p2_2 = polygon[(n2+1 == n ? 0 : n2+1)];
            if(lineSegmentIntersection(rline, p1_2, p2_2, r2)) {
                ready2 = true;
                if(got_same) {
                    r1 = r;
                    break;
                }
            } else
                n2 = (n2-1 == -1) ? n-1 : n2-1;
            k2++;
        }
        if(k1 > n && k2 > n) //If true, there is a problem
            break;

    } while (!ready1 || !ready2);

    if(k1 > n && k2 > n) //If true, there is a problem
        return 0;
    else if(k1 > n || k2 > n) //If true, there is a problem
        return 1;
    return 2;
}

int DeguTrackingModel::getContourToLineIntersection(std::vector<cv::Point> &polygon,
                                         cv::Point3f &rline,
                                         cv::Point2f &r1, cv::Point2f &r2,
                                         int *n1, int *n2) {
    int i, ind[3], n = polygon.size();
    int num_found = 0;
    cv::Point2f p1, p2, pts[3]; //Max 4 for a convex polygon (2 in 2 corners)
    p2 = polygon[n-1];
    for(i=0; i<n; i++) {
        p1 = p2;
        p2 = polygon[i];
        if(lineSegmentIntersection(rline, p1, p2, pts[num_found])) {
            ind[num_found] = i == 0 ? n-1 : i-1;
            if(++num_found == 3)
                break;
        }
    }

    if (num_found == 1) {
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[0];
        r1 = pts[0];
        return 1;
    } else if(num_found == 2) {
        r1 = pts[0];
        r2 = pts[1];
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[1];
        return 2;
    }

    //If more than two, the intersection repeated a polygon point
    if(   fabs(pts[0].y - pts[1].y) < /*EPS*/1e-8
       && fabs(pts[0].x - pts[1].x) < /*EPS*/1e-8) { //Take 1 and 3
        r1 = pts[0];
        r2 = pts[2];
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[2];

        return 3;
    }

    r1 = pts[0];
    r2 = pts[1];
    if(n1 != NULL)
        *n1 = ind[0];
    if(n2 != NULL)
        *n2 = ind[1];

    return 3;

}

// Finds the intersection of two lines, or returns false.
// The line is defined in general form at cv::Point3f(a,b,c) with 'a y + b x + c = 0'
// and the segment by p1 and p2.
bool DeguTrackingModel::lineSegmentIntersection(cv::Point3f &line,
                             cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &r) {

    // if b == 0 Horizontal line
    if(line.y == 0) {
        if(line.x == 0)
            return false;
        float y = -line.z/line.x;
        if(p1.y == p2.y) { //Parallel?
            if(fabs(p1.y - y) < /*EPS*/1e-8) {
                r.x = (p1.x + p2.x)/2.0;
                r.y = p1.y;
                return true;
            } else //Parallel
                return false;
        }
        //Not parallel
        //Check if intercept is in interval
        if(p1.y > p2.y) {
            if(p2.y > y || y > p1.y)
                return false;
        } else {
            if(p1.y > y || y > p2.y)
                return false;
        }
        if(p2.x - p1.x == 0) {
            r.x = p1.x;
            r.y = y;
        } else {
            float m = (p2.y - p1.y)/(p2.x - p1.x), n = p1.y - m*p1.x;
            r.x = (y - n)/m;
            r.y = y;
            return true;
        }
    } else if(line.x == 0) { //a == 0 Vertical line
        float x = -line.z/line.y;
        if(p1.x == p2.x) { //Parallel?
            if(fabs(p1.x - x) < /*EPS*/1e-8) {
                r.x = p1.x;
                r.y = (p1.y + p2.y)/2.0;
                return true;
            } else //Parallel
                return false;
        }
        //Not parallel
        //Check if intercept is in interval
        if(p1.x > p2.x) {
            if(p2.x > x || x > p1.x)
                return false;
        } else {
            if(p1.x > x || x > p2.x)
                return false;
        }
        float m = (p2.y - p1.y)/(p2.x - p1.x), n = p1.y - m*p1.x;
        r.x = x;
        r.y = m*x + n;
        return true;
    }
    //General case

    float m1 = -line.y/line.x, n1 = -line.z/line.x;

    if(p2.x - p1.x == 0) {
        r.x = p1.x;
        r.y = r.x*m1 + n1;
    } else {
        /*if(fabs(p2.y - p1.y) == 0.0) {
            r.y = p1.y;
            r.x = (r.y - n1)/m1;
            return true;
        }*/
        float m2 = (p2.y - p1.y)/(p2.x - p1.x),
              n2 = p1.y - m2*p1.x;

        if (fabs(m1 - m2) < /*EPS*/1e-8)
            return false;

        r.x = (n1 - n2)/(m2 - m1);
        r.y = m1*r.x + n1;
    }

    //Check if point is in interval
    if(p1.y != p2.y) {
        if(p1.y > p2.y) {
            if(p2.y > r.y || r.y > p1.y)
                return false;
        } else {
            if(p1.y > r.y || r.y > p2.y)
                return false;
        }
    }

    if(p1.x > p2.x) {
        if(p2.x > r.x || r.x > p1.x)
            return false;
    } else {
        if(p1.x > r.x || r.x > p2.x)
            return false;
    }

    return true;
}

//a y + b x + c = 0
bool DeguTrackingModel::getGeneralLineForm(cv::Point2f p1, cv::Point2f p2,
                                                    cv::Point3f &rline) {

    // Horizontal line
    if(fabs(p1.y - p2.y) < /*EPS*/1e-8) {
        if(fabs(p1.x - p2.x) < /*EPS*/1e-8)
            return false;
        rline.x = 1.0;   //a
        rline.y = 0.0;   //b
        rline.z = -p1.y; //c
        return true;
    } else if(fabs(p1.x - p2.x) < /*EPS*/1e-8) { //b == 0 Vertical line
        rline.x = 0.0;   //a
        rline.y = 1.0;   //b
        rline.z = -p1.x; //c
        return true;
    }

    rline.x = p2.x - p1.x;
    rline.y = p1.y - p2.y;
    rline.z = p2.y*p1.x - p1.y*p2.x;
    return true;
}


bool DeguTrackingModel::getGeneralLineForm(float x, float y,
                                                    float dx, float dy,
                                                    cv::Point3f &rline) {

    // Horizontal line
    if(fabs(dy) < /*EPS*/1e-8) {
        if(fabs(dx) < /*EPS*/1e-8)
            return false;
        rline.x = 1.0;   //a
        rline.y = 0.0;   //b
        rline.z = -y; //c
    } else if(dx < /*EPS*/1e-8) { //b == 0 Vertical line
        rline.x = 0.0;   //a
        rline.y = 1.0;   //b
        rline.z = -x; //c
    }

    rline.x = dx;
    rline.y = -dy;
    rline.z = dy*x - dx*y;
    return true;
}


double DeguTrackingModel::distanceToSegment(int i1, int i2, std::vector<cv::Point> &contour) {
    int i, x, y, dx, dy, x1, y1, x2, y2, n = contour.size();
    double sum = 0;

    x1 = contour[i1%n].x;
    y1 = contour[i1%n].y;
    x2 = contour[i2%n].x;
    y2 = contour[i2%n].y;
    dx = x2 - x1;
    dy = y2 - y1;
    if(dx == 0) {
        for(i=i1+1; i<i2; i++)
            sum += fabs(contour[i].x - x1);
    } else if(dy == 0) {
        for(i=i1+1; i<i2; i++)
            sum += fabs(contour[i].y - y1);
    } else {
        double m = dy/(double)dx, n = y1 - m*x1, f = sqrt(m*m + 1);
        for(i=i1+1; i<i2; i++)
            sum += fabs(m*contour[i].x + n - contour[i].y)/f;
    }

    return sum;
}



bool DeguTrackingModel::movementFound(cv::Mat f, int wsize, int i0, int j0) {
    int i, j, diff = wsize/2;
    int w = f.cols, h = f.rows;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<h)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < w)
                    if(f.data[i*f.step + j] != 0)
                        return true;
    return false;
}

bool DeguTrackingModel::movementFound(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
    int i, j, diff = wsize/2,
        y1 = f.rows, x1 = f.cols;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<y1)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < x1)
                    if(f.data[i*f.step + j] != 0)
                        return true;
    return false;
}

int DeguTrackingModel::movementCount(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
    int i, j, diff = wsize/2, count = 0,
        y1 = f.rows, x1 = f.cols;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<y1)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < x1)
                    if(f.data[i*f.step + j] != 0)
                        count++;
    return count;
}


double DeguTrackingModel::histogramDistance(cv::MatND h1, cv::MatND h2) {
    cv::MatND d(h1.size(), CV_32FC1);
    cv::absdiff(h1, h2, d);
    cv::Scalar s = cv::sum(d);
    return s.val[0];
}

void DeguTrackingModel::findHead(SpHullModel hull){

    std::vector< segment2D<float> > &segs = hull->segs;
    int  n = segs.size();

    int sumaIzq=0;
    int sumaDer=0;

    int indice=0;

    if(n==0){

    }else{
        while(indice<n){
            segment2D<float> &s1 = segs[indice];
            segment2D<float> &s2 = segs[n-1];

            sumaIzq=sumaIzq+(s1.first.x-s1.last.x)*(s1.first.x-s1.last.x)+(s1.first.y-s1.last.y)*(s1.first.y-s1.last.y);
            sumaDer=sumaDer+(s2.first.x-s2.last.x)*(s2.first.x-s2.last.x)+(s2.first.y-s2.last.y)*(s2.first.y-s2.last.y);

            indice++;
            n--;
        }
        n = segs.size();
        indice=0;

        if(sumaDer<sumaIzq){
            segment2D<float> &se = segs[n-1];
            headLocation.x=(se.first.x+se.last.x)/2+hull->off_x;
            headLocation.y=(se.first.y+se.last.y)/2+hull->off_y;
        }else{
            segment2D<float> &se = segs[indice];
            headLocation.x=(se.first.x+se.last.x)/2+hull->off_x;
            headLocation.y=(se.first.y+se.last.y)/2+hull->off_y;
        }
    }





}



