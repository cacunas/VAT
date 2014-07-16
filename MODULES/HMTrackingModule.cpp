#include "HMTrackingModule.h"

using namespace std;
using namespace cv;

HMTrackingModule::HMTrackingModule(Datapool *i_data) :
    ModuleInterface(i_data) {
    this->firstTime = true;
    this->alpha = 0.1;
    this->beta = 0.75;
}

HMTrackingModule::~HMTrackingModule()
{

}

bool HMTrackingModule::setParameters(QDomNode &config)
{
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
        }

    }

    this->GrassClassifier();
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


void HMTrackingModule::GrassClassifier()
{
    QImage *image = m_data->currentImage;

    hist b_hist(256,0), g_hist(256,0), r_hist(256,0), gray_hist(256,0);

    QRgb pixel;

    // Compute the histograms:
    for (int x=0; x<image->width(); x++) {
        for (int y=0; y<image->height(); y++) {
            pixel = image->pixel(x,y);
            b_hist[qBlue(pixel)]++;
            g_hist[qGreen(pixel)]++;
            r_hist[qRed(pixel)]++;
            gray_hist[qGray(pixel)]++;
        }
    }

    // Compute channels' peaks
    // blue:0, green:1, red:2, grey:3
    this->calculatePeaks(r_hist, g_hist, b_hist, gray_hist);

    // Compute thresholds

    for (int x=0; x < image->width(); x++) {
        for (int y=0; y < image->height(); y++) {
            pixel = image->pixel(x,y);
            if (
                    //image.pixel(x,.y).
                    qGreen(pixel) > qRed(pixel) &&
                    qGreen(pixel) > qBlue(pixel) &&
                    abs(qRed(pixel) - R_p) < R_t &&
                    abs(qGreen(pixel) - G_p) < G_t &&
                    abs(qBlue(pixel) - B_p) < B_t &&
                    qGray(pixel) < GL_t
                    )
            {
                m_data->bgImage->setPixel(x,y,m_data->currentImage->pixel(x,y)); //image->pixel(x,y);
            }
            else
                m_data->bgImage->setPixel(x,y,0);
        }
    }
}


void HMTrackingModule::calculatePeaks(hist r, hist g, hist b, hist gray)
{
    vector<float> peaks(4,0);//Peaks: blue, green, red, gray
    peaks[0] = B_p;
    peaks[1] = G_p;
    peaks[2] = R_p;
    peaks[3] = GL_p;

    if (firstTime)
    {
        // Compute intensity peaks
        int max[4] = {b[0],g[0],r[0], gray[0]};
        for (int i=0; i<256; i++)
        {
            //cout << b[i] << "\t" << g[i] << "\t" << r[i] << endl;
            if (b[i]>max[0])
            {
                max[0] = b[i];
                peaks[0] = (float) i;
            }
            if (g[i]>max[1])
            {
                max[1] = g[i];
                peaks[1] = (float) i;
            }
            if (r[i]>max[2])
            {
                max[2] = r[i];
                peaks[2] = (float) i;
            }
            if (gray[i]>max[3])
            {
                max[3] = gray[i];
                peaks[3] = (float) i;
            }
        }
    }
    else
    {
        hist numerator_u(4,0), numerator_v(4,0), denominator(4,0);
        hist n(4,0); //number of elements per channel
        vector<float> var(4,0);

        for (int i=0; i<256; i++)
        {
            cout << b[i] << "\t" << g[i] << "\t" << r[i] << endl;
            if (b[i] >= (alpha*b[cvRound(B_p)]))
            {
                numerator_u[0] += i*b[i];
                numerator_v[0] += i*i*b[i];
                denominator[0] += b[i];
                n[0]++;
            }
            if (g[i] >= (alpha*g[cvRound(G_p)]))
            {
                numerator_u[1] += i*g[i];
                numerator_v[1] += i*i*g[i];
                denominator[1] += g[i];
                n[i]++;
            }
            if (r[i] >= (alpha*r[cvRound(R_p)]))
            {
                numerator_u[2] += i*r[i];
                numerator_v[2] += i*i*r[i];
                denominator[2] += r[i];
                n[2]++;
            }
            if (gray[i] >= (alpha*gray[cvRound(GL_p)]))
            {
                numerator_u[3] += i*gray[i];
                numerator_u[3] += i*i*gray[i];
                denominator[3] += gray[i];
                n[3]++;
            }
        }

        for (int p=0; p<4; p++)
        {
            if (denominator[p] != 0)
            {
                peaks[p] = (float) (numerator_u[p]/denominator[p]);
                var[p] = (float) (numerator_v[p]/denominator[p]);
            }
        }

        B_u = peaks[0];
        G_u = peaks[1];
        R_u = peaks[2];
        GL_u = peaks[3];

        B_std = sqrt(var[0]-peaks[0]*peaks[0]);
        G_std = sqrt(var[1]-peaks[1]*peaks[1]);
        R_std = sqrt(var[2]-peaks[2]*peaks[2]);
        GL_std = sqrt(var[3]-peaks[3]*peaks[3]);
    }

    this->B_p = peaks[0];
    this->G_p = peaks[1];
    this->R_p = peaks[2];
    this->GL_p = peaks[3];

    R_t = R_std;
    G_t = G_std;
    B_t = B_std;
    GL_t = GL_p + beta*GL_std;
}
