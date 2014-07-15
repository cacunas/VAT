#include "GaussianFunction.h"
#include "MathFunctions.h"
#include "VideoAnalysis.h"

GaussianFunction::GaussianFunction() : mean(0.0), sigma(0.0), min(0.0), max(0.0) {}

GaussianFunction::GaussianFunction(double i_mean, double i_sigma) :
                                mean(i_mean), sigma(i_sigma), min(0.0), max(0.0) {}

GaussianFunction::~GaussianFunction() {}

double GaussianFunction::getValue(double i_val) {
    return MathFunctions::gaussian(mean, sigma, i_val);
}

double GaussianFunction::getNormalisedValue(double i_val) {
    return MathFunctions::normalised_gaussian(mean, sigma, i_val);
}

bool GaussianFunction::setParameters(QDomNode& config) {
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        AppendToLog("GaussianFunction Error: In definition of Gaussian function model (mean, sigma, min, max). Aborting Context loading.");
        return false;
    } else {
        if( ( n = XmlCommon::getParameterNode("mean", config) ).isNull() ) {
            AppendToLog("GaussianFunction Error: In definition of Gaussian function model 'mean' parameter missing. Aborting Context loading.");
            return false;
        } else
            mean = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("sigma", config) ).isNull() ) {
            AppendToLog("GaussianFunction Error: In definition of Gaussian function model 'sigma' parameter missing. Aborting Context loading.");
            return false;
        } else
            sigma = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("min", config) ).isNull() ) {
            AppendToLog("GaussianFunction Error: In definition of Gaussian function model 'min' parameter missing. Aborting Context loading.");
            return false;
        } else
            min = XmlCommon::getParameterValue(n).toDouble();

        if( ( n = XmlCommon::getParameterNode("max", config) ).isNull() ) {
            AppendToLog("GaussianFunction Error: In definition of Gaussian function model 'max' parameter missing. Aborting Context loading.");
            return false;
        } else
            max = XmlCommon::getParameterValue(n).toDouble();

    }

    return true;
}
