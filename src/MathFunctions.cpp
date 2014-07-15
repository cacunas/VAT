#include "MathFunctions.h"
#include <cmath>

double MathFunctions::ONE_DIV_SQRT_2PI = 0.39894228;

int MathFunctions::square(int arg) {
      return arg * arg;
}

double MathFunctions::square(double arg) {
      return arg * arg;
}

double MathFunctions::ceilOrFloor(double value) {
    if(value > 1.0)
        return 1.0;
    if(value < 0.0)
        return 0.0;
    return value;
}

bool MathFunctions::areEqual(double value1, double value2) {
    if (fabs(value1 - value2) <= NUMERIC_ERROR)
        return true;
    return false;
}

double MathFunctions::gaussian(double i_mean, double i_sigma, double i_x) {
    return exp( - square(i_x - i_mean) / ( 2 * square(i_sigma) ) );
}

double MathFunctions::normalised_gaussian(double i_mean, double i_sigma, double i_x) {
    return (ONE_DIV_SQRT_2PI/i_sigma)*exp( - square(i_x - i_mean) / ( 2 * square(i_sigma) ) );
}

int MathFunctions::roundInt(double i_value) {
    if (i_value > 0) {
        if (i_value - floor(i_value) > 0.5)
            return static_cast<int>(ceil(i_value));
        else
	    return static_cast<int>(floor(i_value)); 
    } else {
        if (ceil(i_value)-i_value > 0.5)
            return static_cast<int>(floor(i_value));
        else
	    return static_cast<int>(ceil(i_value));
    }
}

double MathFunctions::incrementalMean(double mean, double value, double Ncurrent) {
  return (value + mean*(Ncurrent-1))/(Ncurrent);
}

double MathFunctions::incrementalSigma(double mean, double sigma, double value, double Ncurrent) {
  double diff = value - mean;
  return sqrt( (sigma*sigma + diff*diff/Ncurrent)*(Ncurrent-1)/Ncurrent);
}

double MathFunctions::incrementalVariance(double mean, double var, double value, double Ncurrent) {
  double diff = value - mean;
  return (var + diff*diff/Ncurrent)*(Ncurrent-1)/Ncurrent;
}

