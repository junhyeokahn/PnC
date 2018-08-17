/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:52 GMT-05:00
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
  double t655;
  double t755;
  double t823;
  double t891;
  double t1440;
  double t2029;
  double t2460;
  double t2188;
  double t2738;
  double t682;
  double t3241;
  double t3244;
  double t3248;
  double t1022;
  double t1567;
  double t1606;
  double t3142;
  double t3147;
  double t3223;
  double t3890;
  double t3905;
  double t3917;
  t655 = Cos(var1[3]);
  t755 = Cos(var1[11]);
  t823 = -1.*t755;
  t891 = 1. + t823;
  t1440 = Sin(var1[11]);
  t2029 = Cos(var1[5]);
  t2460 = Sin(var1[3]);
  t2188 = Sin(var1[4]);
  t2738 = Sin(var1[5]);
  t682 = Cos(var1[4]);
  t3241 = -1.*t2029*t2460;
  t3244 = t655*t2188*t2738;
  t3248 = t3241 + t3244;
  t1022 = -0.022225*t891;
  t1567 = -0.086996*t1440;
  t1606 = 0. + t1022 + t1567;
  t3142 = -0.086996*t891;
  t3147 = 0.022225*t1440;
  t3223 = 0. + t3142 + t3147;
  t3890 = t655*t2029;
  t3905 = t2460*t2188*t2738;
  t3917 = t3890 + t3905;

  p_output1(0)=0. + t3223*t3248 - 0.29508*(t2460*t2738 + t2029*t2188*t655) + t1606*t655*t682 - 0.086996*(-1.*t1440*t655*t682 + t3248*t755) - 0.022225*(t1440*t3248 + t655*t682*t755) + var1(0);
  p_output1(1)=0. + t3223*t3917 - 0.29508*(t2029*t2188*t2460 - 1.*t2738*t655) + t1606*t2460*t682 - 0.086996*(-1.*t1440*t2460*t682 + t3917*t755) - 0.022225*(t1440*t3917 + t2460*t682*t755) + var1(1);
  p_output1(2)=0. - 1.*t1606*t2188 - 0.29508*t2029*t682 + t2738*t3223*t682 - 0.022225*(t1440*t2738*t682 - 1.*t2188*t755) - 0.086996*(t1440*t2188 + t2738*t682*t755) + var1(2);
}


       
void p_rHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
