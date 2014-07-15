#ifndef _MATHFUNCTIONS_H_
#define _MATHFUNCTIONS_H_

#define NUMERIC_ERROR 1e-10

class MathFunctions {
    public:

        static double ONE_DIV_SQRT_2PI;

        static int square(int arg);

        static double square(double arg);

 /* A normal distribution is
  *                                square(x-m)
  *                  1          - --------------
  * y(x,m,s) = ------------ * e    2*square(s)
  *            sqrt(2*pi*s)
  *
  * This functions returns only
  *
  *                square(x-m)
  *              - -----------
  * y(x,m,s) = e   2*square(s)
  *
  * because we want that y(m,m,s) = 1. Remember that y(m+/-s,m,s) = 0.606
  */

        static double gaussian(double i_mean, double i_sigma, double i_x);

        //Returns the real one
        static double normalised_gaussian(double i_mean, double i_sigma, double i_x);

        static int roundInt(double i_value);

        static double ceilOrFloor(double value);

        static bool areEqual(double value1, double value2);

        static double incrementalMean(double mean, double value, double Ncurrent);
        static double incrementalSigma(double mean, double sigma, double value, double Ncurrent);
        static double incrementalVariance(double mean, double var, double value, double Ncurrent);

};

#endif // _MATHFUNCTIONS_H_
