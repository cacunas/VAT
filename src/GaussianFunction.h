#ifndef __GAUSSIAN_FUNCTION__
#define __GAUSSIAN_FUNCTION__

#include <QDomNode>
#include <QSharedPointer>

class GaussianFunction {
    public:
        double mean;
        double sigma;
        double min;
        double max;
        GaussianFunction();
        GaussianFunction(double i_mean, double i_sigma);
        ~GaussianFunction();
        double getValue(double i_val);
        double getNormalisedValue(double i_val);
        bool setParameters(QDomNode& config);

        inline void setMean(const double i_mean) { mean = i_mean; }
        inline void setSigma(const double i_sigma) { sigma = i_sigma; }
        inline void setMin(const double i_min) { min = i_min; }
        inline void setMax(const double i_max) { max = i_max; }
        inline double getMean()	{ return mean; }
        inline double getSigma() { return sigma; }
        inline double getMin() { return min; }
        inline double getMax() { return max; }
};

typedef QSharedPointer<GaussianFunction> SpGaussianFunction;

#endif


