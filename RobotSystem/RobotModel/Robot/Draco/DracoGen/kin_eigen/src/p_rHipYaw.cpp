/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:43 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipYaw.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t620;
  double t670;
  double t1048;
  double t1052;
  double t1203;
  double t2016;
  double t2360;
  double t2253;
  double t2369;
  double t660;
  double t3111;
  double t3112;
  double t3239;
  double t1111;
  double t1505;
  double t1958;
  double t3038;
  double t3042;
  double t3104;
  double t3892;
  double t3898;
  double t3900;
  t620 = Cos(var1[3]);
  t670 = Cos(var1[11]);
  t1048 = -1.*t670;
  t1052 = 1. + t1048;
  t1203 = Sin(var1[11]);
  t2016 = Cos(var1[5]);
  t2360 = Sin(var1[3]);
  t2253 = Sin(var1[4]);
  t2369 = Sin(var1[5]);
  t660 = Cos(var1[4]);
  t3111 = -1.*t2016*t2360;
  t3112 = t620*t2253*t2369;
  t3239 = t3111 + t3112;
  t1111 = -0.022225*t1052;
  t1505 = -0.086996*t1203;
  t1958 = 0. + t1111 + t1505;
  t3038 = -0.086996*t1052;
  t3042 = 0.022225*t1203;
  t3104 = 0. + t3038 + t3042;
  t3892 = t620*t2016;
  t3898 = t2360*t2253*t2369;
  t3900 = t3892 + t3898;

  p_output1(0)=0. + t3104*t3239 - 0.29508*(t2360*t2369 + t2016*t2253*t620) + t1958*t620*t660 - 0.086996*(-1.*t1203*t620*t660 + t3239*t670) - 0.022225*(t1203*t3239 + t620*t660*t670) + var1(0);
  p_output1(1)=0. + t3104*t3900 - 0.29508*(t2016*t2253*t2360 - 1.*t2369*t620) + t1958*t2360*t660 - 0.086996*(-1.*t1203*t2360*t660 + t3900*t670) - 0.022225*(t1203*t3900 + t2360*t660*t670) + var1(1);
  p_output1(2)=0. - 1.*t1958*t2253 - 0.29508*t2016*t660 + t2369*t3104*t660 - 0.022225*(t1203*t2369*t660 - 1.*t2253*t670) - 0.086996*(t1203*t2253 + t2369*t660*t670) + var1(2);
}


       
void p_rHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
