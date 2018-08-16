/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:06 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipRoll.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t242;
  double t1620;
  double t1633;
  double t1686;
  double t1553;
  double t1559;
  double t1675;
  double t1680;
  double t1682;
  double t1694;
  double t1708;
  t242 = Cos(var1[5]);
  t1620 = Sin(var1[5]);
  t1633 = Cos(var1[6]);
  t1686 = Sin(var1[6]);
  t1553 = -1.*t242;
  t1559 = 1. + t1553;
  t1675 = -1.*t1633;
  t1680 = 1. + t1675;
  t1682 = -0.330988*t1680;
  t1694 = -0.90524*t1686;
  t1708 = 0. + t1682 + t1694;

  p_output1(0)=0. - 0.066675*t1559 - 0.260988*t1620 + 0.330988*t1620*t1633 - 0.90524*t1620*t1686 - 1.*t1620*t1708 - 0.066675*t242;
  p_output1(1)=0. - 0.260988*t1559 - 0.330988*t1633*t242 + 0.90524*t1686*t242 + t1708*t242;
  p_output1(2)=0. - 0.90524*(0. + t1633) - 0.90524*t1680 + 0.330988*t1686 - 0.330988*(0. + t1686);
}


       
void p_rHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
