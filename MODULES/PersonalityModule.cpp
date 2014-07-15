#include "PersonalityModule.h"
#include "image_display.h"
#include "src/hullmodel.h"
#include <errno.h>
#include <iostream>
#include <fstream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include <src/blob.h>
#include <map>
#include <set>
#include <QFile>
#include <QTextStream>
#include <cmath>

//cv::RNG rng(12345);

//#define RSEG_DEBUG

const int maxComponents = 1;

PersonalityModule::PersonalityModule(Datapool *i_data): ModuleInterface(i_data) {
    m_min_images_to_process = 10;
    m_bins = 9;
    w_size = 11;
    m_hullOffset = 1;
    m_K = 2;
    processed_images = 0;
    m_num_ev = 2;
    m_clusters = 3;
    m_use_mean = false;
    m_use_reliability = true;
    m_scale = 8;
    m_ostep = 40;
    m_tuning_mode = true;
    m_abs_angular_diff = true;

    gaussianConstant[1]  =          M_1_DIV_2PI;
    gaussianConstant[2]  = 1/4.0   *M_1_DIV_2PI;
    gaussianConstant[4]  = 1/16.0  *M_1_DIV_2PI;
    gaussianConstant[8]  = 1/64.0  *M_1_DIV_2PI;
    gaussianConstant[16] = 1/256.0 *M_1_DIV_2PI;
    gaussianConstant[32] = 1/1024.0*M_1_DIV_2PI;
    gaussianConstant[64] = 1/4096.0*M_1_DIV_2PI;
    gaussianFactor[1]  = 1/   2.0;
    gaussianFactor[2]  = 1/   8.0;
    gaussianFactor[4]  = 1/  32.0;
    gaussianFactor[8]  = 1/ 128.0;
    gaussianFactor[16] = 1/ 512.0 ;
    gaussianFactor[32] = 1/2048.0;
    gaussianFactor[64] = 1/8192.0;

    names.resize(40);
    elegant.resize(40);

    names[0] = "1A";
    names[1] = "1B";
    names[2] = "1C";
    names[3] = "1D";
    names[4] = "2A";
    names[5] = "2B";
    names[6] = "2C";
    names[7] = "2D";
    names[8] = "3A";
    names[9] = "3B";
    names[10] = "3C";
    names[11] = "3D";
    names[12] = "4A";
    names[13] = "4B";
    names[14] = "4C";
    names[15] = "4D";
    names[16] = "5A";
    names[17] = "5B";
    names[18] = "5C";
    names[19] = "5D";
    names[20] = "6A";
    names[21] = "6B";
    names[22] = "6C";
    names[23] = "6D";
    names[24] = "7A";
    names[25] = "7B";
    names[26] = "7C";
    names[27] = "7D";
    names[28] = "8A";
    names[29] = "8B";
    names[30] = "8C";
    names[31] = "8D";
    names[32] = "9A";
    names[33] = "9B";
    names[34] = "9C";
    names[35] = "9D";
    names[36] = "10A";
    names[37] = "10B";
    names[38] = "10C";
    names[39] = "10D";

    elegant[0] = false;
    elegant[1] = true;
    elegant[2] = false;
    elegant[3] = false;
    elegant[4] = true;
    elegant[5] = false;
    elegant[6] = true;
    elegant[7] = false;
    elegant[8] = true;
    elegant[9] = true;
    elegant[10] = false;
    elegant[11] = false;
    elegant[12] = false;
    elegant[13] = false;
    elegant[14] = false;
    elegant[15] = true;
    elegant[16] = false;
    elegant[17] = false;
    elegant[18] = false;
    elegant[19] = true;
    elegant[20] = true;
    elegant[21] = true;
    elegant[22] = false;
    elegant[23] = false;
    elegant[24] = false;
    elegant[25] = true;
    elegant[26] = false;
    elegant[27] = false;
    elegant[28] = false;
    elegant[29] = false;
    elegant[30] = true;
    elegant[31] = false;
    elegant[32] = false;
    elegant[33] = false;
    elegant[34] = false;
    elegant[35] = true;
    elegant[36] = false;
    elegant[37] = false;
    elegant[38] = false;
    elegant[39] = false;

//    elegant[0] = ;
//    elegant[1] = ;
//    elegant[2] = ;
//    elegant[3] = ;
//    elegant[4] = ;
//    elegant[5] = ;
//    elegant[6] = ;
//    elegant[7] = ;
//    elegant[8] = ;
//    elegant[9] = ;
//    elegant[10] = ;
//    elegant[11] = ;
//    elegant[12] = ;
//    elegant[13] = ;
//    elegant[14] = ;
//    elegant[15] = ;
//    elegant[16] = ;
//    elegant[17] = ;
//    elegant[18] = ;
//    elegant[19] = ;
//    elegant[20] = ;
//    elegant[21] = ;
//    elegant[22] = ;
//    elegant[23] = ;
//    elegant[24] = ;
//    elegant[25] = ;
//    elegant[26] = ;
//    elegant[27] = ;
//    elegant[28] = ;
//    elegant[29] = ;
//    elegant[30] = ;
//    elegant[31] = ;
//    elegant[32] = ;
//    elegant[33] = ;
//    elegant[34] = ;
//    elegant[35] = ;
//    elegant[36] = ;
//    elegant[37] = ;
//    elegant[38] = ;
//    elegant[39] = ;
    first = true;
    m_supervisedFile = "";

}

PersonalityModule::~PersonalityModule() {}

bool PersonalityModule::setParameters(QDomNode& config){
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        m_min_images_to_process = 10;
        m_bins = 9;
        w_size = 11;
        m_hullOffset = 1;
        m_K = 2;
        m_supervisedFile = "config/personality-copas.txt";
        m_min_contour_length = 20;
        m_num_ev = 2;
        m_clusters = 3;
        m_use_mean = false;
        m_use_reliability = true;
        m_scale = 8;
        m_ostep = 40;
        m_abs_angular_diff = true;
        m_tuning_mode = true;

        //Tuned parameters
        //Improvement 12: 0
        //m_bins= 5; m_scale= 6; m_ostep= 20; m_abs_angular_diff= false; m_use_reliability= true; m_use_mean= false;

        m_bins= 4; m_scale= 20; m_ostep= 60; m_abs_angular_diff= true; m_use_reliability= true; m_use_mean= true;
    } else {
        if( !( n = XmlCommon::getParameterNode("ColorHistogramBinSize", config) ).isNull() ) {
            m_bins = XmlCommon::getParameterValue(n).toInt();
            if(m_bins <= 0 || m_bins > 128) {
                m_bins = 32;
                AppendToLog("PersonalityModule: Warning: 'ColorHistogramBinSize' must be in ]0,128]. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(m_bins) );
            }
        } else { //default
            m_bins = 32;
            AppendToLog("PersonalityModule: Warning: 'ColorHistogramBinSize' not defined. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(m_bins) );
        }
        addParameter("ColorHistogramBinSize", QString::number(m_bins), "int");

        if( !( n = XmlCommon::getParameterNode("SearchWindowSize", config) ).isNull() ) {
            w_size = XmlCommon::getParameterValue(n).toInt();
            if(w_size <= 0) {
                w_size = 11;
                AppendToLog("PersonalityModule: Warning: 'SearchWindowSize' must be higher than 0. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(w_size) );
            }
        } else { //default
            w_size = 11;
            AppendToLog("PersonalityModule: Warning: 'SearchWindowSize' not defined. Taking default: \n\t\tSearchWindowSize = " + QString::number(w_size) );
        }
        addParameter("SearchWindowSize", QString::number(w_size), "int");

        if( !( n = XmlCommon::getParameterNode("ConvexHullOffset", config) ).isNull() ) {
            m_hullOffset = XmlCommon::getParameterValue(n).toInt();
        } else { //default
            m_hullOffset = 1;
            AppendToLog("PersonalityModule: Warning: 'ConvexHullOffset' not defined. Taking default: \n\t\tConvexHullOffset = " + QString::number(m_hullOffset) );
        }
        addParameter("ConvexHullOffset", QString::number(m_hullOffset), "int");

        if( !( n = XmlCommon::getParameterNode("PrincipalAxisStep", config) ).isNull() ) {
            m_K = XmlCommon::getParameterValue(n).toInt();
        } else { //default
            m_K = 2;
            AppendToLog("PersonalityModule: Warning: 'PrincipalAxisStep' not defined. Taking default: \n\t\tPrincipalAxisStep = " + QString::number(m_K) );
        }
        addParameter("PrincipalAxisStep", QString::number(m_K), "int");

        if( !( n = XmlCommon::getParameterNode("SupervisedFile", config) ).isNull() ) {
            m_supervisedFile = XmlCommon::getParameterValue(n).toStdString();
        } else { //default
            m_supervisedFile = "config/personality-copas.txt";
            AppendToLog("PersonalityModule: Warning: 'SupervisedFile' not defined. Taking default: \n\t\t = config/personality-copas.txt" );
        }
        addParameter("SupervisedFile", m_supervisedFile.c_str(), "std::string");

        if( !( n = XmlCommon::getParameterNode("MinContourLength", config) ).isNull() ) {
            m_min_contour_length = XmlCommon::getParameterValue(n).toInt();
        } else { //default
            m_min_contour_length = 20;
            AppendToLog("PersonalityModule: Warning: 'MinContourLength' not defined. Taking default: \n\t\t = " + QString::number(m_min_contour_length) );
        }
        addParameter("MinContourLength", QString::number(m_min_contour_length), "int");

    }

    return true;
}

bool PersonalityModule::updateParameters(){
    parameter *bins, *size, *hull;
    bins = getParameter("ColorHistogramBinSize");
    size = getParameter("SearchWindowSize");
    hull = getParameter("ConvexHullOffset");

    if( bins == 0 || size == 0 || hull == 0 ||
            bins->value.toInt() <= 0 || bins->value.toInt() > 128 ||
            size->value.toInt() <= 0 )
            return false;
    return true;
}

bool PersonalityModule::init(){
    return true;
}


int PersonalityModule::readSupervisedData() {
    std::ifstream ifs(m_supervisedFile.c_str());
    std::string s, saux;
    int i;
    size_t pos;
    std::deque<SupervisedStatisticalInfo> sup;
    int num;
    double m, M, sd, r;

    while(getline(ifs, s)) {
        if( (pos=s.find_first_not_of(" \t\r\n")) != std::string::npos ) {
            s = s.substr(pos);
            if(s[0] != '#') {
                //Separate in numbers
                if( (pos=s.find_first_of(" \t\r\n")) != std::string::npos ) { //image number
                    saux = s.substr(0, pos+1);
                    //std::cout << "num: " << saux << std::endl;
                    QString qs(saux.c_str());
                    num = qs.toInt();
                    s = s.substr(pos+1);
                    if( (pos=s.find_first_of(" \t\r\n")) != std::string::npos ) { //mean
                        saux = s.substr(0, pos+1);
                        //std::cout << "m: " << saux << std::endl;
                        QString qs(saux.c_str());
                        m = qs.toDouble();
                        s = s.substr(pos+1);
                        if( (pos=s.find_first_of(" \t\r\n")) != std::string::npos ) { //mean
                            saux = s.substr(0, pos+1);
                            //std::cout << "sd: " << saux << std::endl;
                            QString qs(saux.c_str());
                            sd = qs.toDouble();
                            s = s.substr(pos+1);
                            if( (pos=s.find_first_of(" \t\r\n")) != std::string::npos ) { //median and reliability
                                saux = s.substr(0, pos+1);
                                //std::cout << "M: " << saux << std::endl;
                                QString qs(saux.c_str());
                                M = qs.toDouble();
                                s = s.substr(pos+1);
                                //std::cout << "r: " << s << std::endl;
                                QString qs2(s.c_str());
                                r = qs2.toDouble();
                                SupervisedStatisticalInfo ss(m, sd, M, r);
                                sup.push_back(ss);
                            }
                        }
                    }
                }
            }
        }
    }

    int size = sup.size();
    if(size == 0)
        return 0;
    supervised.resize(size);
    for(int j=0; j<size; j++) {
        std::cout << j << ": " << sup[j].mean << "; " << sup[j].sd << "; " << sup[j].median << "; " << sup[j].R << std::endl;
        supervised[j] = sup[j];
    }
    return size;
}

float PersonalityModule::getPrediction(CvANN_MLP &ann,
                                       SupervisedStatisticalInfo &sup) {
    int j, m = sup.descriptor.size();
    cv::Mat trainData(1, m, CV_32FC1);
    for(j=0; j<m; j++)
        trainData.at<float>(0,j) = sup.descriptor[j];

    //std::cout << "\tInput descriptor:" << trainData;

    //Output data samples. Matrix of order (n x 1)
    cv::Mat prediction(1, 1, CV_32FC1);
    prediction.at<float>(0,0) = - 10.0;

    ann.predict(trainData, prediction);

    return prediction.at<float>(0,0);
}

void PersonalityModule::getModel(std::vector<SupervisedStatisticalInfo> &sup,
                                 CvANN_MLP &ann,
                                 bool mean, bool reliability) {
    //CvANN_MLP_TrainParams ann(); //Will use default values
    int i, j, n = sup.size(), m = sup[0].descriptor.size();

    //Input data samples. Matrix of order (n x m)
    cv::Mat trainData(n, m, CV_32FC1);
    for(i=0; i<n; i++)
        for(j=0; j<m; j++)
            trainData.at<float>(i,j) = sup[i].descriptor[j];

    //Output data samples. Matrix of order (n x 1)
    cv::Mat trainClasses(n, 1, CV_32FC1);
    if(mean)
        for(i=0; i<n; i++)
            trainClasses.at<float>(i,0) = sup[i].mean;
    else
        for(i=0; i<n; i++)
            trainClasses.at<float>(i,0) = sup[i].median;

    //The weight of each training data sample.
    cv::Mat sampleWts(n, 1, CV_32FC1);
    if(reliability)
        for(i=0; i<n; i++)
            sampleWts.at<float>(i,0) = sup[i].R;
    else
        for(i=0; i<n; i++)
            sampleWts.at<float>(i,0) = 1.0;

    //The matrix representation of our ANN. We'll have four layers.
    //Setting the number of neurons on each of 3 layers of the ANN
    /*
        We have in Layer 1: m neurons (m inputs)
                   Layer 2: m/2 neurons (hidden layer)
                   Layer 3: 1 neurons (1 output)
    */
    cv::Mat neuralLayers(3, 1, CV_32SC1);
    neuralLayers.at<int>(0,0) = m;
    neuralLayers.at<int>(1,0) = m/2;
    //neuralLayers.at<int>(1,0) = 2;
    neuralLayers.at<int>(2,0) = 1;

    //ann.create(neuralLayers, CvANN_MLP::IDENTITY, 0, 0);
    ann.create(neuralLayers, CvANN_MLP::SIGMOID_SYM, 0, 0);

    CvANN_MLP_TrainParams params;
    CvTermCriteria criteria;
    criteria.max_iter = 1000000;
    criteria.epsilon  = 0.00000001;
    criteria.type     = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    params.train_method = CvANN_MLP_TrainParams::RPROP;
    params.bp_dw_scale  = 0.0001;
    params.bp_moment_scale = 0.0001;
    params.term_crit  = criteria;

    ann.train(trainData, trainClasses, sampleWts, cv::Mat(), params, 0);

//    std::cout << ann;

}



std::ostream& operator<<(std::ostream& out, CvANN_MLP &ann) {

    int layers = ann.get_layer_count();
    cv::Mat lsizes = ann.get_layer_sizes();
    out << "  Layers: " << layers << std::endl;
    int i, j, prev_ls;
    for(i=0; i<layers; i++){
        int ls = lsizes.at<int>(0,i);
        out << "    Layer " << i << ":" << std::endl;
        out << "\tLayer size:" << ls << std::endl;
        if(i>0) {
            out << "\tLayers " << i-1 <<"->"<< i << " weights: \n [ ";
            int nweights = (prev_ls + 1)*ls;
            double ww, *w = ann.get_weights(i);
            for(j=0; j<nweights; j++) {
                ww = w[j];
                if(j < nweights - 1)
                    out << w[j] << "; ";
                else
                    out << w[j];
            }
            out << " ]" << std::endl;
        }
        prev_ls = ls;
    }


    return out;
}


//Implements LOOCV (Leave-one-out cross-validation)
float PersonalityModule::crossValidation(std::vector<SupervisedStatisticalInfo> &sup, std::vector<CvANN_MLP> &anns, float &rel_error) {
    std::vector<SupervisedStatisticalInfo> train_sup;
    SupervisedStatisticalInfo test_sup;
    int i, j, k, n = sup.size();
    train_sup.resize(n-1);
    anns.resize(n);
    float error = 0.0, rel = 0.0;
    rel_error = 0.0;

    std::cout << "  Start: " << std::endl;
    for(i=0; i<n; i++) {
        for(j=0, k=0; j<n; j++)
            if(i != j)
                train_sup[k++] = sup[j];
            else
                test_sup = sup[j];

        getModel(train_sup, anns[i], m_use_mean, m_use_reliability);

        //if(!m_tuning_mode) {
            //std::cout << "Test Object " << i << ":" << std::endl;
            //std::cout << "  Neural Network " << i << ":"
            //          << std::endl << anns[i] << std::endl;
            //std::cout << "  Test Object Data:"
            //          << std::endl << test_sup << std::endl;
        //}

        float diff, prediction = getPrediction(anns[i], test_sup);
        //if(!m_tuning_mode)
        //    std::cout << "  Prediction: " << prediction << std::endl;
        float v = (m_use_mean? test_sup.mean : test_sup.median);
        diff = fabs(prediction - v);
        std::cout << prediction << ";" << v << ";" << diff << std::endl;
        error += diff;
        rel_error += test_sup.R * diff;
        rel += test_sup.R;
    }
    error /= (float)n;
    rel_error /= rel;
    return error;
}

//Formato ARGB
//b == bits[i]    g == bits[i+1]    r == bits[i+2]    alpha == bits[i+3]
bool PersonalityModule::run() {

    //Set forward image
    if(m_data->persoImage == NULL && m_data->currentImage != NULL) {
        m_data->persoImage = new QImage(m_data->currentImage->width(),m_data->currentImage->height(),QImage::Format_RGB888);
    }

    if(first) {
        first = false;
        int n = readSupervisedData();
        if(m_tuning_mode)
            image_contours.resize(n);
    }

    float rel_error;

    if(m_tuning_mode) {
        if(m_data->currentImage != NULL) { //Requires current image
            getPersonalityPreAngular(m_data->currentImage, image_contours[processed_images]);
            processed_images++;
            if(processed_images >= m_min_images_to_process) {
                float min_diff = FLT_MAX;
                int i,j,k,l,m,n,o, best_count = 0;
                /*for(i=4; i<=36; i++){ //m_bins
                    m_bins = i;
                    for(j=1; j<=90; j+=1){ //m_scale
                        m_scale = j;
                        for(k=5; k<=70; k+=5){ //m_ostep
                            m_ostep = k;
                            for(l=0; l<=1; l++){ //m_abs_angular_diff
                                m_abs_angular_diff = (l==0 ? true : false);*/
                                m_bins= 9; m_scale= 8; m_ostep= 15; m_abs_angular_diff= false; m_use_reliability= true; m_use_mean= true;
                                for(m=0; m<processed_images; m++)
                                    getPersonalityFromAngular(image_contours[m], supervised[m]);
                                /*for(n=0; n<=1; n++){
                                    m_use_reliability = (n==0 ? true : false);
                                    for(o=0; o<=1; o++){
                                        m_use_mean = (o==0 ? false : true);*/
                                        //m_bins= 4; m_scale= 20; m_ostep= 60; m_abs_angular_diff= true; m_use_reliability= true; m_use_mean= true;
                                    for(o=0; o<=1000; o++){
                                        std::vector<CvANN_MLP> anns2;
                                        float abs_diff = crossValidation(supervised, anns2, rel_error);
                                        if(rel_error <= min_diff) {
                                            min_diff = rel_error;
                                            std::cout <<"Improvement " << best_count << ": " << min_diff << std::endl;
                                            std::cout << "  m_bins: " << m_bins
                                                      << "; m_scale: " << m_scale
                                                      << "; m_ostep: " << m_ostep
                                                      << "; m_abs_angular_diff: " << (m_abs_angular_diff ? "true" : "false")
                                                      << "; m_use_reliability: " << (m_use_reliability ? "true" : "false")
                                                      << "; m_use_mean: " << (m_use_mean ? "true" : "false") << std::endl;
                                            best_count++;
                                        }
                                    }
                                    /*}
                                }
                            }
                        }
                    }
                }*/
            }
        }
        std::cout << "End of tuning process." << std::endl;
    } else {
        if(m_data->currentImage != NULL) { //Requires current image
            getPersonality(m_data->currentImage, supervised[processed_images]);
            processed_images++;
            if(processed_images >= m_min_images_to_process) {
                crossValidation(supervised, anns, rel_error);

                //PCA
                //cv::Mat pcadata(m_bins, histograms.size(), CV_32FC1);
                //cv::Mat pcamean = cv::Mat::zeros(m_bins, 1, CV_32FC1);
                //sup.pca = getPCAfromHistograms(histograms, pcadata, pcamean);

                //K-means
                //if(sup.pca->eigenvectors.rows == m_bins)
                //    kcenters = getKMeansFromPCA(sup.pca, pcadata, pcamean);
            }
        }
    }
    return true;
}

void PersonalityModule::safeCopyQImageToMat(QImage &q, cv::Mat &m) {
    int bl_mat = m.step, bl = q.bytesPerLine();
    uchar *c_p = q.bits();
    if(bl == bl_mat)
        memcpy(m.data, c_p, q.height()*bl);
    else{
        int h = q.height();
        for(int i=0; i<h; i++)
            memcpy(m.data + i*bl_mat, c_p + i*bl, 3*q.width());
    }

}

void PersonalityModule::drawContourToMat(std::vector<cv::Point> &contour,
                                         cv::Mat &m) {
    int i, ll = contour.size();
    cv::Point p1, p2;
    cv::Scalar color;
    //std::cout << "Longest:" << std::endl;
    for(i = 0; i< ll-1; i++ ) {
        p1 = contour[i];
        p2 = contour[i+1];
        //std::cout << "Longest:" << std::endl;
        color = cv::Scalar((255*i)/ll,0,((ll-i)*255)/ll);
        cv::line(m,p1,p2,color);
    }
    p1 = contour[ll-1];
    p2 = contour[0];
    color = cv::Scalar(255,0,0);
    cv::line(m,p1,p2,color);
}

void PersonalityModule::blurContour(std::vector<cv::Point> &contour,
                                    int scale, int step) {
    int i, j, k, ll = contour.size();
    double f = 1.0/(scale*scale);
    double x, y, xx, yy, dist, sum, ex;
    cv::Point pc, pl;
    std::vector<cv::Point> out = contour;
    step = step > ll/2 ? ll/2 : step;
    //std::cout << "Longest:" << std::endl;
    for(i = 0; i< ll; i++ ) {
        pc = contour[i];
        x = pc.x; y = pc.y;
        sum = 1.0;
        //Left of gaussian
        for(j = -step; j < 0; j++ ) {
            for(k=i + j; k < 0; k += ll);
            pl = contour[k];
            xx = pc.x - pl.x;
            yy = pc.y - pl.y;
            dist = xx*xx + yy*yy;
            ex = exp(-dist*f);
            x += pl.x * ex;
            y += pl.y * ex;
            sum += ex;
        }
        //Right of gaussian
        for(j = 1; j <= step; j++ ) {
            pl = contour[(i + j)%ll];
            xx = pc.x - pl.x;
            yy = pc.y - pl.y;
            dist = xx*xx + yy*yy;
            ex = exp(-dist*f);
            x += pl.x * ex;
            y += pl.y * ex;
            sum += ex;
        }
        //Mean values
        x /= sum;
        y /= sum;
        //Assign to output vector
        out[i].x = (int)round(x);
        out[i].y = (int)round(y);
    }
    for(i = 0; i< ll; i++ )
        contour[i] = out[i];
}


void PersonalityModule::drawContourToMat(std::vector<cv::Point> &contour,
                                         cv::Mat &m, cv::Scalar color) {
    int i, ll = contour.size();
    cv::Point p1, p2;

    for(i = 0; i< ll-1; i++ ) {
        p1 = contour[i];
        p2 = contour[i+1];
        cv::line(m,p1,p2,color);
    }
    p1 = contour[ll-1];
    p2 = contour[0];
    cv::line(m,p1,p2,color);
}

void PersonalityModule::filterContours(std::vector< std::vector<cv::Point> > &contours,
                                       std::vector< std::vector<cv::Point> > &fcontours,
                                       int min_length) {
    int ll = contours.size();

    if(ll == 1)
        fcontours = contours;
    else {
        int i, max = 0, s, smax = 0;
        for(i=0; i<ll; i++) {
            s = contours[i].size();
            if(s >= min_length)
                fcontours.push_back(contours[i]);
            if(s > smax) {
                smax = s;
                max = i;
            }
        }
        if(fcontours.size() == 0)
            fcontours.push_back(contours[max]);
    }
}


void PersonalityModule::connectContours(std::vector< std::vector<cv::Point> > &contours,
                                        std::vector< std::vector<cv::Point> > &ccontours) {
    int i, j, s, ll = contours.size(), xx, yy, d, dmin, min, k = 0;
    cv::Point p1, p2, p;
    bool reverse = false;

    if(ll == 1)
        ccontours = contours;
    else {
        bool usedContours[ll];
        memset(usedContours, 0, ll);

        //int connected_to[contours.size()];
        for(i=0; i<ll; i++) {
            if(!usedContours[i]) {

                ccontours.push_back(contours[i]); //Add non used contour as base
                std::vector<cv::Point> &c1 = ccontours[k++];

                do {
                    p1 = c1.front();
                    p2 = c1.back();
                    xx = p1.x - p2.x;
                    yy = p1.y - p2.y;
                    dmin = xx*xx + yy*yy;
                    min = i;
                    reverse = false;

                    for(j=i+1; j<ll;j++) {
                        if(!usedContours[j]) {
                            std::vector<cv::Point> &c2 = contours[j];
                            p = c2.front();
                            xx = p2.x - p.x;
                            yy = p2.y - p.y;
                            d = xx*xx + yy*yy;
                            if(d < dmin) {
                                min = j;
                                reverse = false;
                                dmin = d;
                            }
                            p = c2.back();
                            xx = p2.x - p.x;
                            yy = p2.y - p.y;
                            d = xx*xx + yy*yy;
                            if(d < dmin) {
                                min = j;
                                reverse = true;
                                dmin = d;
                            }
                        }
                    }
                    if(min != i){ //Reconnecting
                        usedContours[min] = true;
                        std::vector<cv::Point> in_c = contours[min];
                        if(reverse) {
                            int l, cs = in_c.size();
                            cv::Point paux;
                            for(j=0,l=cs;j<cs/2;j++,l--) {
                                paux = in_c[j];
                                in_c[j] = in_c[l];
                                in_c[l] = paux;
                            }
                        }
                        c1.insert(c1.end(),in_c.begin(), in_c.end());
                    }
                } while(min != i);
            }
        }
    }

}


void PersonalityModule::prepareImage(cv::Mat &current) {

    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 3, 3 ),
                                       cv::Point( 1, 1 ) );
    cv::threshold(current, current, 128, 255, cv::THRESH_BINARY_INV);
    cv::erode(current, current, element);
    cv::dilate(current, current,element);
}


void PersonalityModule::getShapeBorder(cv::Mat &current, cv::Mat &border) {
    //Rectangular structuring element
    cv::Canny(current, border, 50,100, 3);
}

int PersonalityModule::getCentroid(cv::Mat &image, float x, float y) {

    uchar *data = image.data;
    x = 0.0, y = 0.0;
    int i, j, counter = 0, w = image.cols,
        h = image.rows, bl = image.step;
    for(i=0; i<h; i++) {
        for(j=0; j<w; j++) {
            if(data[i*bl + j] > 128) {
                x += j;
                y += i;
                counter++;
            }
        }
    }
    x /= (double)counter;
    y /= (double)counter;

    return counter;
}


void PersonalityModule::getMainAxis(cv::Mat &image, float x, float y, int n,
                                    cv::Point2f &r1, cv::Point2f &r2) {

    std::vector<cv::Point2f> data_aux(n);
    int i, j, count = 0, w = image.cols,
        h = image.rows, bl = image.step;
    uchar *dat = image.data;

    for(i=0; i<h; i++)
        for(j=0; j<w; j++)
            if(dat[i*bl + j] > 128) {
                data_aux[count++] = cv::Point2f(j, i);
            }

    cv::Mat data(2, count, CV_32FC1);
    for(i=0; i<count; i++) {
        data.at<float>(0,i) = data_aux[i].x;
        data.at<float>(1,i) = data_aux[i].y;
    }

    cv::Mat mean(2, 1, CV_32FC1);
    mean.at<float>(0) = x;
    mean.at<float>(1) = y;

    //Perform PCA
    cv::PCA pca(data, mean, CV_PCA_DATA_AS_COL, maxComponents);

    //result is contained in pca.eigenvectors (as row vectors)
    //std::cout << pca.eigenvectors << std::endl;

    //Get angle of principal axis
    float dx = pca.eigenvectors.at<float>(0, 0),
          dy = pca.eigenvectors.at<float>(0, 1),
          scale = 40.0;
    cv::Point3f rline;

    //Get line general form from principal component
    getGeneralLineForm(cv::Point2f(x, y),
                       cv::Point2f(x + dx*scale, y + dy*scale),
                       rline);

    getFarPoints(image, dx, dy, r1, r2, x, y);
}

static char randstate[2048];


void PersonalityModule::getAngularFunction(std::vector<float> &angular,
                        std::vector< std::vector<cv::Point> > &contours) {
    //Process every "long" contour. Not sure. They where already filtered...
    //int min_to_process = 100;
    int WStep = 3*m_scale; //Empirical result
    int i, j, l, s;
    double diff;

    for(i = 0; i< contours.size(); i++ ) {
        //if(contours[i].size() > min_to_process) {
            std::vector<cv::Point> &lcontour = contours[i];
            blurContour(lcontour, m_scale, WStep);

            //Get directions
            s = lcontour.size() - 1;
            std::vector<float> ldirection;
            int ss = (s-1)/m_ostep;
            ldirection.resize(ss);
            for(j = 0, l = 0; j < ss; j++, l += m_ostep)
                ldirection[j] = atan2(lcontour[l+m_ostep].y - lcontour[l].y, lcontour[l+m_ostep].x - lcontour[l].x);

            //Get angular function
            std::vector<float> langular_function;
            s = ldirection.size() - 1;
            langular_function.resize(s);
            for(j = 0; j < s; j++) {
                diff = m_abs_angular_diff ?
                            abs(ldirection[j+1] - ldirection[j])
                          : ldirection[j+1] - ldirection[j];
                langular_function[j] = diff;
            }

            //Join for different sub-contours
            if(angular.empty())
                angular = langular_function;
            else
                angular.insert(angular.end(),langular_function.begin(),langular_function.end());
        //}
    }
}


void PersonalityModule::getHistogram(std::vector<float> &input, cv::MatND &hist) {

    //Histogram parameters
    int channels[] = {0};
    int histSize[] = {m_bins};
    float pranges[2];
    if(m_abs_angular_diff) {
        pranges[0] = 0.0; pranges[1] = 2*M_PI;
    } else {
        pranges[0] = -2*M_PI; pranges[1] = 2*M_PI;
    }

    const float* ranges[] = {pranges};

    int i, s = input.size();
    cv::Mat afunc(s,1,CV_32FC1);
    for(i=0; i<s; i++)
        afunc.at<float>(i,0) = input[i];

    cv::calcHist( &afunc, 1, channels, cv::Mat(), // do not use mask
                hist, 1, histSize, ranges,
                true, // the histogram is uniform
                false );
    float Sum = cv::sum(hist)[0];
    hist /= Sum;
}

cv::PCA *PersonalityModule::getPCAfromHistograms(std::vector<cv::MatND> h) {
    int i, j;
    cv::Mat pcadata(m_bins, h.size(), CV_32FC1);
    cv::Mat pcamean = cv::Mat::zeros(m_bins, 1, CV_32FC1);
    for(i=0; i<h.size(); i++) {
        for(j=0; j<m_bins; j++)
            pcamean.at<float>(j) += pcadata.at<float>(j,i) = h[i].at<float>(j,0);
    }
    pcamean /= h.size();

    return getPCA(pcadata, pcamean);
}

cv::PCA *PersonalityModule::getPCAfromHistograms(std::vector<cv::MatND> h, cv::Mat &pcadata, cv::Mat &pcamean) {
    int i, j;
    pcadata.create(m_bins, h.size(), CV_32FC1);
    pcamean = cv::Mat::zeros(m_bins, 1, CV_32FC1);
    for(i=0; i<h.size(); i++) {
        for(j=0; j<m_bins; j++)
            pcamean.at<float>(j) += pcadata.at<float>(j,i) = h[i].at<float>(j,0);
    }
    pcamean /= h.size();

    return getPCA(pcadata, pcamean);
}


cv::PCA *PersonalityModule::getPCA(cv::Mat &pcadata, cv::Mat &pcamean) {

    int i;
    cv::PCA *pca_result = new cv::PCA(pcadata, pcamean, CV_PCA_DATA_AS_COL, m_bins);

    //result is contained in pca.eigenvectors (as row vectors)
    std::cout << "data: " << pcadata << std::endl;
    std::cout << "means: " << pcamean << std::endl;
    std::cout << "evalues: " << pca_result->eigenvalues << std::endl;
    std::cout << "evectors: " << pca_result->eigenvectors << std::endl;

    cv::Mat ev = pca_result->eigenvalues;

    //std::cout << "ev rows: " << ev.rows << std::endl;

    //std::cout << "ev cols: " << ev.cols << std::endl;

    float esum = 0.0, pesum = 0.0;
    for(i=0; i<ev.rows; i++)
        esum += ev.at<float>(i,0);

    ev /= esum;

    //Sum Square Error

    for(i=0; i<m_num_ev; i++)
        pesum += ev.at<float>(i,0);

    std::cout << "ev: " << ev << std::endl;
    std::cout << "Info rate: " << pesum*100 << "%" << std::endl;

    return pca_result;

}

cv::Mat PersonalityModule::getKMeansFromPCA(cv::PCA *pca, cv::Mat &pcadata, cv::Mat &pcamean) {

    int i, j;
    cv::Mat A(m_num_ev, m_bins, CV_32FC1);
    cv::vector<cv::Point2f> pca_points;

    for(i=0; i<m_num_ev; i++)
        for(j=0; j<m_bins; j++)
            A.at<float>(i,j) = pca->eigenvectors.at<float>(i,j);

    cv::Mat pdata, pmean;
    cv::transpose(pcadata, pdata);
    cv::transpose(pcamean, pmean);
    std::cout << "pdata rows: " << pdata.rows << std::endl;
    std::cout << "pdata cols: " << pdata.cols << std::endl;
    std::cout << "pcamean rows: " << pmean.rows << std::endl;
    std::cout << "pcamean cols: " << pmean.cols << std::endl;
    std::cout << "A rows: " << A.rows << std::endl;
    std::cout << "A cols: " << A.cols << std::endl;
    std::cout << "A: " << A << std::endl;


    pca_points.resize(pdata.rows);
    cv::Point2f p;

    cv::Mat ksamples(pdata.rows,2, CV_32FC1);
    cv::Mat klabels(pdata.rows,1, CV_16UC1);
    kcenters.create(m_clusters,2, CV_32FC1);

    for(i=0; i<pdata.rows; i++) {
        cv::Mat tnor, nor = pdata.row(i)-pmean;
        std::cout << "nor: " << nor << std::endl;
        cv::transpose(nor, tnor);
        cv::Mat R = A*tnor;
        std::cout << "R: " << R << std::endl;
        p.x = R.at<float>(0,0);
        p.y = R.at<float>(1,0);
        ksamples.at<float>(i,0) = p.x;
        ksamples.at<float>(i,1) = p.y;
        pca_points[i] = p;
    }

    //KMEANS
    cv::kmeans(ksamples, m_clusters, klabels,
               cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 0),
               10, cv::KMEANS_RANDOM_CENTERS, kcenters);

    std::cout << "klabels: " << klabels << std::endl;
    std::cout << "kcenters: " << kcenters << std::endl;

    cv::Mat rpca(m_data->personalityPCAImage->height(), m_data->personalityPCAImage->width(), CV_8UC3);
    rpca = cv::Scalar(255,255,255);

    drawKMeansPointsToMat(pca_points, rpca, klabels, kcenters, m_clusters);

    //Personality PCA
    int bl_mat = rpca.step, bl = m_data->personalityPCAImage->bytesPerLine();
    uchar *c_p = m_data->personalityPCAImage->bits();

    int hh = m_data->personalityPCAImage->height(),
        ww = m_data->personalityPCAImage->width();
    if(bl == bl_mat)
        memcpy(c_p, rpca.data, hh*bl);
    else{
        for(i=0; i<hh; i++)
            memcpy(c_p + i*bl, rpca.data + i*bl_mat, 3*ww);
    }

    return kcenters;
}


void PersonalityModule::getPersonality(QImage *current,
                                       SupervisedStatisticalInfo &sup) {

    int i, j, w = current->width(), h = current->height();
    QImage curr888 = current->convertToFormat(QImage::Format_RGB888);

    cv::Mat c(h, w, CV_8UC3),
            f(h, w, CV_8UC1), f0(h, w, CV_8UC1), r(h, w, CV_8UC3);

    int bl = curr888.bytesPerLine();
    uchar *c_p = curr888.bits();

    safeCopyQImageToMat(curr888, c);
    int bl_mat = r.step;
    memset(r.data, 0, h*bl_mat);

    cv::cvtColor(c, f, CV_RGB2GRAY);

    //1. Prepare current image
    prepareImage(f);

    //2. Get shape border
    getShapeBorder(f, f0);

    //3. Centroid
    float x, y;
    int n = getCentroid(f, x, y);

    //Draw resulting centroid
    cv::cvtColor(f0, r, CV_GRAY2RGB);
    cv::circle(r,cv::Point((int)x,(int)y),5, cv::Scalar(255,0,0));

    //4. Get main axis
    cv::Point2f r1, r2; //Points representing the main axis segment
    getMainAxis(f0, x, y, n, r1, r2);

    //Draw line
    cv::Scalar color = cv::Scalar( 255, 0, 0 );
    cv::line(r, r1, r2, color);

    //5. Get separated contours
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(f, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );


    //Draw contours
    initstate(233, randstate, 256);
    for(i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        cv::drawContours(r, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    cv::Mat rr2 = cv::Mat::zeros( r.size(), CV_8UC3);
    for(i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(5+250*(random()/(double)RAND_MAX), 50+250*(random()/(double)RAND_MAX), 50+250*(random()/(double)RAND_MAX));
        cv::drawContours(rr2, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    cv::namedWindow( "Contours", 1 );
    cv::imshow( "Contours", rr2 );

    //6. Filter contours
    std::vector< std::vector<cv::Point> > fcontours;
    filterContours(contours, fcontours, m_min_contour_length);

    //Draw contours
    cv::Mat r4 = cv::Mat::zeros( r.size(), CV_8UC3);
    for(i = 0; i< fcontours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        drawContourToMat(fcontours[i], r4, color);
    }
    cv::namedWindow( "FContours", 1 );
    cv::imshow( "FContours", r4 );


    //7. Connect contours
    std::vector< std::vector<cv::Point> > ccontours;
    connectContours(fcontours, ccontours);

    //Draw contours
    cv::Mat r3 = cv::Mat::zeros( r.size(), CV_8UC3);
    int longest = -1, len = 0;
    for(i = 0; i< ccontours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        drawContourToMat(ccontours[i], r3, color);
        if(ccontours[i].size() > len) {
            len = ccontours[i].size();
            longest = i;
        }
    }

    cv::namedWindow( "CContours", 1 );
    cv::imshow( "CContours", r3 );

    //8. Get DESCRIPTORS
    // Idea:
    // Each boundary point has a descriptor of mean Angular function, and its s.d.
    // Mean obtained by convolving with a Gaussian centred in the point.
    // Then, a 2D histogram (mean, s.d.) can be obtained. Features can be weighted by scale.
    // Bigger scale has bigger visual weight? Can highly repeated feature overcome the visual
    // weight if a bigger scale feature?

    //8.1 Angular function (change in shape boundary direction)
    std::vector<float> angular_function;
    getAngularFunction(angular_function, ccontours);

    //Draw contours
    color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
    cv::Mat raux = cv::Mat::zeros( r.size(), CV_8UC3);
    r = cv::Scalar(255,255,255);
    for(i = 0; i< ccontours.size(); i++ ) {
        std::vector<cv::Point> &lcontour = ccontours[i];
        color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        drawContourToMat(lcontour, r, color);
        drawContourToMat(lcontour, raux, color);
    }
    cv::namedWindow( "Scaled contours", 1 );
    cv::imshow( "Scaled contours", raux );

    //8.2 Get Histogram
    cv::MatND hist;
    getHistogram(angular_function, hist);

    std::cout << m_data->frameNumber << ": " << hist << std::endl;
    histograms.push_back(hist);

    //8.3 Set Initial Descriptor
    sup.descriptor.resize(m_bins);
    for(j=0; j<m_bins; j++)
        sup.descriptor[j] = hist.at<float>(j,0);

    //Set datapool images
    //Personality
    if(r.rows != m_data->persoImage->height() || r.cols != m_data->persoImage->width()) {
        delete m_data->persoImage;
        m_data->persoImage = new QImage(r.cols, r.rows, QImage::Format_RGB888);
    }

    bl_mat = r.step;
    bl = m_data->persoImage->bytesPerLine();
    c_p = m_data->persoImage->bits();

    if(bl == bl_mat)
        memcpy(c_p, r.data, h*bl);
    else{
        for(i=0; i<h; i++)
            memcpy(c_p + i*bl, r.data + i*bl_mat, 3*w);
    }


}

void PersonalityModule::getPersonalityPreAngular(QImage *current,
                                       std::vector< std::vector<cv::Point> > &ccontours) {

    int i, j, w = current->width(), h = current->height();
    QImage curr888 = current->convertToFormat(QImage::Format_RGB888);
    cv::Mat c(h, w, CV_8UC3),
            f(h, w, CV_8UC1), f0(h, w, CV_8UC1), r(h, w, CV_8UC3);
    int bl = curr888.bytesPerLine(), bl2 = f.step;
    uchar *c_p = curr888.bits();

    safeCopyQImageToMat(curr888, c);
    int bl_mat = r.step;
    memset(r.data, 0, h*bl_mat);

    cv::cvtColor(c, f, CV_RGB2GRAY);

    //1. Prepare current image
    prepareImage(f);

    //2. Get shape border
    getShapeBorder(f, f0);

    //3. Centroid
    float x, y;
    int n = getCentroid(f, x, y);

    //Draw resulting centroid
    cv::cvtColor(f0, r, CV_GRAY2RGB);
    cv::circle(r,cv::Point((int)x,(int)y),5, cv::Scalar(255,0,0));

    //4. Get main axis
    cv::Point2f r1, r2; //Points representing the main axis segment
    getMainAxis(f0, x, y, n, r1, r2);

    //Draw line
    cv::Scalar color = cv::Scalar( 255, 0, 0 );
    cv::line(r, r1, r2, color);

    //5. Get separated contours
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(f, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );


    //Draw contours
    initstate(233, randstate, 256);
    for(i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        cv::drawContours(r, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    cv::Mat rr2 = cv::Mat::zeros( r.size(), CV_8UC3);
    for(i = 0; i< contours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(5+250*(random()/(double)RAND_MAX), 50+250*(random()/(double)RAND_MAX), 50+250*(random()/(double)RAND_MAX));
        cv::drawContours(rr2, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }
    cv::namedWindow( "Contours", 1 );
    cv::imshow( "Contours", rr2 );

    //6. Filter contours
    std::vector< std::vector<cv::Point> > fcontours;
    filterContours(contours, fcontours, m_min_contour_length);

    //Draw contours
    cv::Mat r4 = cv::Mat::zeros( r.size(), CV_8UC3);
    for(i = 0; i< fcontours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        drawContourToMat(fcontours[i], r4, color);
    }
    cv::namedWindow( "FContours", 1 );
    cv::imshow( "FContours", r4 );

    //7. Connect contours
    connectContours(fcontours, ccontours);

    //Draw contours
    cv::Mat r3 = cv::Mat::zeros( r.size(), CV_8UC3);
    int longest = -1, len = 0;
    for(i = 0; i< ccontours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX), 255*(random()/(double)RAND_MAX));
        drawContourToMat(ccontours[i], r3, color);
        if(ccontours[i].size() > len) {
            len = ccontours[i].size();
            longest = i;
        }
    }

    cv::namedWindow( "CContours", 1 );
    cv::imshow( "CContours", r3 );

    //Set datapool images
    //Personality
    if(r.rows != m_data->persoImage->height() || r.cols != m_data->persoImage->width()) {
        delete m_data->persoImage;
        m_data->persoImage = new QImage(r.cols, r.rows, QImage::Format_RGB888);
    }

    bl_mat = r.step;
    bl = m_data->persoImage->bytesPerLine();
    c_p = m_data->persoImage->bits();

    if(bl == bl_mat)
        memcpy(c_p, r.data, h*bl);
    else{
        for(i=0; i<h; i++)
            memcpy(c_p + i*bl, r.data + i*bl_mat, 3*w);
    }


}

void PersonalityModule::getPersonalityFromAngular(std::vector< std::vector<cv::Point> > &contours,
                                       SupervisedStatisticalInfo &sup) {
    //8. Get DESCRIPTORS
    // Idea:
    // Each boundary point has a descriptor of mean Angular function, and its s.d.
    // Mean obtained by convolving with a Gaussian centred in the point.
    // Then, a 2D histogram (mean, s.d.) can be obtained. Features can be weighted by scale.
    // Bigger scale has bigger visual weight? Can highly repeated feature overcome the visual
    // weight if a bigger scale feature?

    //8.1 Angular function (change in shape boundary direction)
    std::vector<float> angular_function;
    getAngularFunction(angular_function, contours);

    //8.2 Get Histogram
    cv::MatND hist;
    getHistogram(angular_function, hist);

    //std::cout << m_data->frameNumber << ": " << hist << std::endl;
    histograms.push_back(hist);

    //8.3 Set Initial Descriptor
    sup.descriptor.resize(m_bins);
    for(int j=0; j<m_bins; j++)
        sup.descriptor[j] = hist.at<float>(j,0);

}

void PersonalityModule::drawKMeansPointsToMat(cv::vector<cv::Point2f> &pca_points, cv::Mat &m,
                                        cv::Mat &klabels, cv::Mat &kcenters, int K) {
    int W = m.cols, H = m.rows;
    float dx, dy, max_x = 0.0, max_y = 0.0;
    float dist, kdist[K];
    int label, i, l = pca_points.size();
    cv::Point2f p;
    float cx, cy;
    cv::Point p2D;

    memset(kdist, 0, K*sizeof(float));

    cv::line(m, cv::Point(W/2,0), cv::Point(W/2,H-1),cv::Scalar(255,0,0));
    cv::line(m, cv::Point(0,H/2), cv::Point(W-1,H/2),cv::Scalar(255,0,0));

    for(i=0; i<l; i++) {
        p = pca_points[i];
        if(fabs(p.x) > max_x)
            max_x = fabs(p.x);
        if(fabs(p.y) > max_y)
            max_y = fabs(p.y);
        label = klabels.at<int>(i,0);
        cx = kcenters.at<float>(label,0);
        cy = kcenters.at<float>(label,1);
        dx = cx - p.x;
        dy = cy - p.y;
        dist = sqrt(dx*dx + dy*dy);
        if(kdist[label] < dist)
            kdist[label] = dist;
    }

    max_x *= 1.1;
    max_y *= 1.1;

    int radius;
    for(i=0; i<K; i++) {
        radius = (int)(kdist[i]*H/(2*max_y));
        radius += 20;
        p2D.x = (int)((kcenters.at<float>(i,0) + max_x)*W/(2*max_x));
        p2D.y = (int)((kcenters.at<float>(i,1) + max_y)*H/(2*max_y));
        cv::circle(m, p2D, radius, cv::Scalar(200,200,200));
        cv::circle(m, p2D, 2, cv::Scalar(255,255,0));
    }

    for(i=0; i<l; i++) {
        p = pca_points[i];
        p2D.x = (int)((p.x + max_x)*W/(2*max_x));
        p2D.y = (int)((p.y + max_y)*H/(2*max_y));
        if(elegant[i])
            cv::rectangle(m, cv::Rect(p2D.x-3,p2D.y-3,6,6), cv::Scalar(0,255,0));
        else
            cv::circle(m, p2D, 3, cv::Scalar(0,0,255));
//        cv::putText(m, names[i], p2D, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
        cv::putText(m, QString::number(i+1).toStdString(), p2D, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
    }


}


void PersonalityModule::drawPointsToMat(cv::vector<cv::Point2f> &pca_points, cv::Mat &m) {
    int W = m.cols, H = m.rows;
    float max_x = 0.0, max_y = 0.0;
    int i, l = pca_points.size();
    cv::Point2f p;
    cv::Point p2D;

    cv::line(m, cv::Point(W/2,0), cv::Point(W/2,H-1),cv::Scalar(255,0,0));
    cv::line(m, cv::Point(0,H/2), cv::Point(W-1,H/2),cv::Scalar(255,0,0));

    for(i=0; i<l; i++) {
        p = pca_points[i];
        if(fabs(p.x) > max_x)
            max_x = fabs(p.x);
        if(fabs(p.y) > max_y)
            max_y = fabs(p.y);
    }

    max_x *= 1.1;
    max_y *= 1.1;

    for(i=0; i<l; i++) {
        p = pca_points[i];
        p2D.x = (int)((p.x + max_x)*W/(2*max_x));
        p2D.y = (int)((p.y + max_y)*H/(2*max_y));
        if(elegant[i])
            cv::rectangle(m, cv::Rect(p2D.x-3,p2D.y-3,6,6), cv::Scalar(0,255,0));
        else
            cv::circle(m, p2D, 3, cv::Scalar(0,0,255));
        cv::putText(m, names[i], p2D, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
    }
}

void PersonalityModule::getNormalIntersections(cv::Mat &f,
                                   cv::Rect &roi,
                                   std::vector<cv::Point> &hull,
                                   cv::Point2f &r1, cv::Point2f &r2,
                                   int n1, int n2,
                                   float dx, float dy,
                                   std::vector< segment2D<float> > &segs) {
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
    float lx = R2.x - R1.x, ly = R2.y - R1.y,
          D_axis = sqrt(lx*lx + ly*ly),
          a = m_K/sqrt(dx*dx + dy*dy),
          Dx = fabs(a*dx),
          Dy = sy*fabs(a*dy);
    int i, n = (int)D_axis/m_K;
    float x, y;
    segs.resize(n);

    for(i=0, x=R1.x + Dx, y=R1.y + Dy; i<n; i++, x+=Dx, y+=Dy)
        setForegroundSegment(f, roi, hull, segs[i], x, y, dx, dy, I1, I2);
}


void PersonalityModule::setForegroundSegment(cv::Mat &f, cv::Rect &roi,
                                                      std::vector<cv::Point> &hull,
                                                      segment2D<float> &seg,
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

int PersonalityModule::getContourToLineIntersectionIndexed(std::vector<cv::Point> &polygon,
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

bool PersonalityModule::pointInNeighborhood(cv::Mat im, float pos_x, float pos_y,
                         int &x, int &y) {

    uchar *data = im.data;
    int b = im.step;

    //3x3 neighborhood if no point on the center
    if(im.data[b*((int)pos_y) + (int)pos_x] > 128) {
        x = (int)pos_x;
        y = (int)pos_y;
        return true;
    }

    int h = im.rows, w = im.cols, count = 0;
    float sx, sy;
    x = (int)pos_x;
    y = (int)pos_y;
    sx = 0.0, sy = 0.0;
    if(x-1 >= 0 && y-1 >= 0)
        if(im.data[b*(y-1) + x-1] > 128) {sx += x-1; sy += y-1; count++;}
    if(            y-1 >= 0)
        if(im.data[b*(y-1) + x  ] > 128) {sx += x  ; sy += y-1; count++;}
    if(x+1 <  w && y-1 >= 0)
        if(im.data[b*(y-1) + x+1] > 128) {sx += x+1; sy += y-1; count++;}

    if(x-1 >= 0)
        if(im.data[b*y + x-1] > 128) {sx += x-1; sy += y; count++;}
    if(x+1 <  w)
        if(im.data[b*y + x+1] > 128) {sx += x+1; sy += y; count++;}

    if(x-1 >= 0 && y+1 < h)
        if(im.data[b*(y+1) + x-1] > 128) {sx += x-1; sy += y+1; count++;}
    if(            y+1 < h)
        if(im.data[b*(y+1) + x  ] > 128) {sx += x  ; sy += y+1; count++;}
    if(x+1 <  w && y+1 < h)
        if(im.data[b*(y+1) + x+1] > 128) {sx += x+1; sy += y+1; count++;}

    if(count > 0) {
        x = (int)rint(sx/count);
        y = (int)rint(sy/count);
        return true;
    }

    return false;
}


void PersonalityModule::getFarPoints(cv::Mat im, float dx, float dy,
                                     cv::Point2f &r1, cv::Point2f &r2,
                                     float mean_x, float mean_y) {
    int h = im.rows, w = im.cols, x, y;
    float pos_x = mean_x, pos_y = mean_y, dist, max_dist;
    bool is_set = false;
    float D = sqrt(dx*dx + dy*dy);
    float Dx = dx/D, Dy = dy/D;

    max_dist = 0.0;
    pos_x = mean_x + Dx;
    pos_y = mean_y + Dy;
    //Normal orientation
    while(pos_x < w && pos_x >= 0 && pos_y < h && pos_y >= 0 ) {

        if(pointInNeighborhood(im, pos_x, pos_y, x, y)) {
            dist = sqrt( (x-mean_x)*(x-mean_x) + (y-mean_y)*(y-mean_y) );
            if(dist > max_dist) {
                is_set = true;
                r1.x = x;
                r1.y = y;
                max_dist = dist;
            }
        }
        pos_x += Dx;
        pos_y += Dy;
    }
    if(is_set == false) {
        r1.x = mean_x;
        r1.y = mean_y;
    }

    //The same for second point in opposite direction
    is_set = false;
    max_dist = 0.0;
    pos_x = mean_x - Dx;
    pos_y = mean_y - Dy;
    //Normal orientation
    while(pos_x < w && pos_x >= 0 && pos_y < h && pos_y >= 0 ) {

        if(pointInNeighborhood(im, pos_x, pos_y, x, y)) {
            dist = sqrt( (x-mean_x)*(x-mean_x) + (y-mean_y)*(y-mean_y) );
            if(dist > max_dist) {
                is_set = true;
                r2.x = x;
                r2.y = y;
                max_dist = dist;
            }
        }
        pos_x -= Dx;
        pos_y -= Dy;
    }
    if(is_set == false) {
        r2.x = mean_x;
        r2.y = mean_y;
    }

}


int PersonalityModule::getContourToLineIntersection(std::vector<cv::Point> &polygon,
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
bool PersonalityModule::lineSegmentIntersection(cv::Point3f &line,
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
bool PersonalityModule::getGeneralLineForm(cv::Point2f p1, cv::Point2f p2,
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


bool PersonalityModule::getGeneralLineForm(float x, float y,
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


double PersonalityModule::distanceToSegment(int i1, int i2, std::vector<cv::Point> &contour) {
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



bool PersonalityModule::movementFound(cv::Mat f, int wsize, int i0, int j0) {
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

bool PersonalityModule::movementFound(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
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

int PersonalityModule::movementCount(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
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


double PersonalityModule::histogramDistance(cv::MatND h1, cv::MatND h2) {
    cv::MatND d(h1.size(), CV_32FC1);
    cv::absdiff(h1, h2, d);
    cv::Scalar s = cv::sum(d);
    return s.val[0];
}

