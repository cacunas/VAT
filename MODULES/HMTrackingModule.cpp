#include "HMTrackingModule.h"

using namespace std;
using namespace cv;

HMTrackingModule::HMTrackingModule(Datapool* i_data) :
    ModuleInterface(i_data) {
    this->firstTime = true;
    this->alpha = 0.1;
    this->beta = 0.75;
}

HMTrackingModule::~HMTrackingModule()
{

}

bool HMTrackingModule::setParameters(QDomNode& config)
{
    if(config.isNull()) {
        return true;
    }

    return true;
}

bool HMTrackingModule::init()
{
    return true;
}

bool HMTrackingModule::run()
{
    if(this->firstTime)
    {

        this->firstTime = false;
        if(m_data->bgImage == NULL)
        {
            m_data->bgImage=
                    new QImage(m_data->currentImage->width(),
                               m_data->currentImage->height(),
                               QImage::Format_RGB888); //Set Background
<<<<<<< HEAD
=======
        }
        if(m_data->grayImage == NULL)
        {
            m_data->grayImage=
                    new QImage(m_data->currentImage->width(),
                               m_data->currentImage->height(),
                               QImage::Format_Indexed8); //Set Background
>>>>>>> 241c16516b6e3ffb1bd54505e2197a9747e8f2d9
        }

    }

    this->GrassClassifier();
<<<<<<< HEAD
    this->HFilter(m_data->currentImage);
=======
    this->Line_detect();
>>>>>>> 241c16516b6e3ffb1bd54505e2197a9747e8f2d9
    return true;
}

bool HMTrackingModule::updateParameters()
{
    return true;
}

Mat HMTrackingModule::qimage_to_mat_cpy(QImage* img, int format)
{
    return cv::Mat(
                img->height(),
                img->width(),
                format,
                img->bits(),
                img->bytesPerLine()
                ).clone();
}

<<<<<<< HEAD
QImage *HMTrackingModule::Mat2QImage(cv::Mat &mat) {
    QImage *temp = NULL;

    if(true/*mat.type() == CV_8UC3*/) {
        temp = new QImage(mat.cols, mat.rows, QImage::Format_RGB32);

        uchar *a = temp->bits();
        uchar *b = mat.data;

        // I had to use this (2 for) to transform 3 channels to 4, since
        // the program is using RGB32 format for QImage:
        for(int j=0; j<mat.rows; j++)
            for(int i=0; i<mat.cols; i++) {
                a[j*temp->bytesPerLine() + i*4 + 0] =
                        b[j*mat.step + i*3 + 0];
                a[j*temp->bytesPerLine() + i*4 + 1] =
                        b[j*mat.step + i*3 + 1];
                a[j*temp->bytesPerLine() + i*4 + 2] =
                        b[j*mat.step + i*3 + 2];
                a[j*temp->bytesPerLine() + i*4 + 3] = 255;
            }
    } else {
        //AppendToLog("Mat format not supported to QImage");
    }
    return temp;
}

=======
>>>>>>> 241c16516b6e3ffb1bd54505e2197a9747e8f2d9
vector<hist> HMTrackingModule::calculateHistograms(QImage *img)
{
    vector<hist> channels(4);

    for (int i=0; i<4; i++)
        channels[i] = hist(256,0);

    QRgb pixel;

    for (int x=0; x<img->width(); x++) {
        for (int y=0; y<img->height(); y++) {
            pixel = img->pixel(x,y);
            channels[0][qRed(pixel)]++;
            channels[1][qGreen(pixel)]++;
            channels[2][qBlue(pixel)]++;
            channels[3][qGray(pixel)]++;
<<<<<<< HEAD
        }
    }

    return channels;
}

vector<float> HMTrackingModule::calculateMoments(vector<hist> channels) {
    vector<float> fMoments(4,0.);
    vector<float> sMoments(4,0.);

    float num_u=0, num_v=0, den=0;

    for (int ch=0; ch<4; ch++) {
        for (int i=0; i<256; i++) {
            if ( channels[ch][i] >= (alpha * channels[ch][cvRound(A_p[ch])]) )
            {
                num_u += i*channels[ch][i];
                num_v += i*i*channels[ch][i];
                den   += channels[ch][i];
=======
        }
    }

    return channels;
}

vector<float> HMTrackingModule::calculateMoments(vector<hist> channels) {
    vector<float> fMoments(4,0.);
    vector<float> sMoments(4,0.);

    float num_u=0, num_v=0, den=0;

    for (int ch=0; ch<4; ch++) {
        for (int i=0; i<256; i++) {
            if ( channels[ch][i] >= (alpha * channels[ch][cvRound(A_p[ch])]) )
            {
                num_u += i*channels[ch][i];
                num_v += i*i*channels[ch][i];
                den += channels[ch][i];
            }
        }

        fMoments[ch] = num_u/den;
        sMoments[ch] = num_v/den;
        num_u = num_v = den = 0;
    }

    vector<float> moments = fMoments;
    moments.insert(moments.end(),sMoments.begin(),sMoments.end());

    return moments;
}

void HMTrackingModule::calculatePeaks(vector<hist> channels)
{
    vector<float> peaks(4,0);

    if (firstTime)
    {
        // Compute intensity peaks
        int max[4] = {
            channels[0][0],
            channels[1][0],
            channels[2][0],
            channels[3][0]};

        for (int ch=0; ch<4; ch++) {
            for (int i=0; i<256; i++) {
                if (channels[ch][i] > max[ch]) {
                    max[ch] = channels[ch][i];
                    peaks[ch] = (float) i;
                }
            }
        }
    }

    else
    {
        vector<float> aux = this->calculateMoments(channels);
        for (int ch=0; ch<4; ch++)
            peaks[ch] = aux[ch];
    }

    for (int ch=0; ch<4; ch++)
        A_p[ch] = peaks[ch];
}

void HMTrackingModule::calculateThresholds(vector<hist> ch) {
    vector<float> moments = this->calculateMoments(ch);

    for (int ch=0; ch<3; ch++) {
        A_t[ch] = sqrt(moments[ch+4]-moments[ch]*moments[ch]);
    }

    A_t[3] = A_p[3] + beta * sqrt(moments[7]-moments[3]*moments[3]);
}

void HMTrackingModule::GrassClassifier()
{
    QImage *image = m_data->currentImage;

    /* Histograms for each channel:
* 0 - red
* 1 - green
* 2 - blue
* 3 - gray
*/
    vector<hist> channels = this->calculateHistograms(image);

    // Compute channels' peaks
    // red:0, blue:1, green:2, gray:3
    this->calculatePeaks(channels);

    // Compute thresholds
    this->calculateThresholds(channels);

    /*
* Debug: print peaks and thresholds
*/

// for (int i=0; i<4; i++) {
// cout <<"Peak ch " << i << ":\t" << A_p[i] << endl;
// cout <<"Thre ch " << i << ":\t" << A_t[i] << endl;
// }

    QRgb pixel;

    for (int x=0; x < image->width(); x++) {
        for (int y=0; y < image->height(); y++) {
            pixel = image->pixel(x,y);
            if (
                    qGreen(pixel) > qRed(pixel) &&
                    qGreen(pixel) > qBlue(pixel) &&
                    abs(qRed(pixel) - A_p[0]) < A_t[0] &&
                    abs(qGreen(pixel) - A_p[1]) < A_t[1] &&
                    abs(qBlue(pixel) - A_p[2]) < A_t[2] &&
                    qGray(pixel) < A_t[3]
                    )
            {
                //cout << "Debug: Yay!\n" ;
                int gray = qGray(m_data->currentImage->pixel(x,y));
                m_data->bgImage->setPixel(x,y,qRgb(gray,gray,gray));
>>>>>>> 241c16516b6e3ffb1bd54505e2197a9747e8f2d9
            }
        }

        fMoments[ch] = num_u/den;
        sMoments[ch] = num_v/den;
        num_u = num_v = den = 0;
    }

    vector<float> moments = fMoments;
    moments.insert(moments.end(),sMoments.begin(),sMoments.end());

    return moments;
}

<<<<<<< HEAD
void HMTrackingModule::calculatePeaks(vector<hist> channels)
{
    vector<float> peaks(4,0);

    if (firstTime)
    {
        // Compute intensity peaks
        int max[4] = {
            channels[0][0],
            channels[1][0],
            channels[2][0],
            channels[3][0]};

        for (int ch=0; ch<4; ch++) {
            for (int i=0; i<256; i++) {
                if (channels[ch][i] > max[ch]) {
                    max[ch] = channels[ch][i];
                    peaks[ch] = (float) i;
                }
            }
        }
    }

    else
    {
        vector<float> aux = this->calculateMoments(channels);
        for (int ch=0; ch<4; ch++)
            peaks[ch] = aux[ch];
    }

    for (int ch=0; ch<4; ch++)
        A_p[ch] = peaks[ch];
}

void HMTrackingModule::calculateThresholds(vector<hist> ch) {
    vector<float> moments = this->calculateMoments(ch);

    for (int ch=0; ch<3; ch++) {
        A_t[ch] = sqrt(moments[ch+4]-moments[ch]*moments[ch]);
    }

    A_t[3] = A_p[3] + beta * sqrt(moments[7]-moments[3]*moments[3]);
}

void HMTrackingModule::GrassClassifier()
{
    QImage *image = m_data->currentImage;

    /* Histograms for each channel:
     * 0 - red
     * 1 - green
     * 2 - blue
     * 3 - gray
     */
    vector<hist> channels = this->calculateHistograms(image);

    // Compute channels' peaks
    // red:0, blue:1, green:2, gray:3
    this->calculatePeaks(channels);

    // Compute thresholds
    this->calculateThresholds(channels);

    /*
     * Debug: print peaks and thresholds
     */

//    for (int i=0; i<4; i++) {
//        cout <<"Peak ch " << i << ":\t" << A_p[i] << endl;
//        cout <<"Thre ch " << i << ":\t" << A_t[i] << endl;
//    }

    QRgb pixel;

    for (int x=0; x < image->width(); x++) {
        for (int y=0; y < image->height(); y++) {
            pixel = image->pixel(x,y);
            if (
                    qGreen(pixel) > qRed(pixel) &&
                    qGreen(pixel) > qBlue(pixel) &&
                    abs(qRed(pixel) - A_p[0]) < A_t[0] &&
                    abs(qGreen(pixel) - A_p[1]) < A_t[1] &&
                    abs(qBlue(pixel) - A_p[2]) < A_t[2] &&
                    qGray(pixel) < A_t[3]
                    )
            {
                //cout << "Debug: Yay!\n" ;
                int pix = qGray(m_data->currentImage->pixel(x,y));
                QRgb grayPix = QColor(pix, pix, pix).rgb();
                m_data->bgImage->setPixel ( x, y, grayPix);
            }
            else
                m_data->bgImage->setPixel(x,y,0);
        }
    }
}

void HMTrackingModule::HFilter(QImage* img)
{
    cout << "DEBUG:\t entering HFilter()" << endl;
    //QImage* img = m_data->bgImage;
    //Mat src = this->qimage_to_mat_cpy(img, CV_8UC3), dst;
    Mat src = qImage2Mat(img);
    Mat dst;

    //cout << "DEBUG:\t src= " << src << endl;

    Mat kernel = (Mat_<double>(3,3) << -2, 1, -2, 1, 4, 1, -2, 1, -2);

    cout << "DEBUG:\t kernel= " << kernel << endl;

    //Mat kernel = ( Mat_<float>(3,3) << -2.,1.,-2., 1.,4.,1., -2.,1.,-2.);
    Point anchor = Point(-1,-1);
    double delta = 0;
    int ddepth = -1;
    //int kernel_size;

    anchor = Point(-1,-1);
    delta = 0.;

    cout << "DEBUG:\t Before filter2D(...)" << endl;
    filter2D(src, dst, ddepth , kernel, anchor, delta, BORDER_DEFAULT );

    cout << "DEBUG:\t dst.type()= " << dst.type() << endl;

    m_data->bgImage = this->Mat2QImage(dst);

    //cout << "DEBUG:\t dst= " << dst << endl;

    //m_data->bgImage = this->Mat2QImage(dst);
=======
void HMTrackingModule::Line_detect()
{
    if(m_data->currentImage == NULL)
    {
        AppendToLog("SegemntationModule: Warning: No current image. Aborting execution...\n");
    }
    if(m_data->grayImage == NULL)
    {
        AppendToLog("SegemntationModule: Warning: No Background image. Aborting execution...\n");
    }

    AppendToLog("asd1");

    QImage *current_img = this->m_data->currentImage;
    QImage *gray_img = this->m_data->grayImage;
    QImage *bg_img = this->m_data->bgImage;
    int current_bpl = current_img->bytesPerLine();
    int gray_bpl = gray_img->bytesPerLine();
    int bg_bpl = bg_img->bytesPerLine();
    int w = gray_img->width();
    int h = gray_img->height();
    uchar *current_bits = current_img->bits();
    uchar *gray_bits = gray_img->bits();

    for(int y=0; y<h; y++)
    {
        for(int x=0; x<w; x++)
        {
            int current_pos = y*current_bpl + 4*x;
            int gray_pos = y*gray_bpl + x;
            float promedio = float(current_bits[current_pos])
                    + float(current_bits[current_pos+1])
                    + float(current_bits[current_pos+2]);
            promedio = promedio/3;
            uchar gray  = (char) promedio;
            gray_bits[gray_pos] = gray;
        }
    }

    this->ApplyFilter(gray_img,gray_img);

}

void HMTrackingModule::ApplyFilter(QImage *f_in, QImage *f_out)
{
    int w = f_in->width(), h = f_in->height();

    cv::Mat kernel = (Mat_<double>(3,3) << -2, 1, -2, 1, 4, 1, -2, 1, -2);
    cv::Mat f(h, w, CV_8UC1);

    int bl = f_in->bytesPerLine();
    uchar *fin_p = f_in->bits();
    uchar *fout_p = f_out->bits();

    memcpy(f.data, fin_p, h*bl);

    filter2D(f, f, -1 , kernel, Point( -1, -1 ), 0, BORDER_DEFAULT );

    memcpy(fout_p, f.data, h*bl);
>>>>>>> 241c16516b6e3ffb1bd54505e2197a9747e8f2d9
}
