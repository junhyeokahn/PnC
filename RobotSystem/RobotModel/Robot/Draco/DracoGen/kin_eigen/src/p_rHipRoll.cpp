/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:45 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t655;
  double t2195;
  double t2733;
  double t3144;
  double t4623;
  double t5039;
  double t5054;
  double t5052;
  double t5104;
  double t4865;
  double t4881;
  double t4901;
  double t4914;
  double t2192;
  double t5239;
  double t5247;
  double t5290;
  double t5053;
  double t5156;
  double t5167;
  double t5359;
  double t5360;
  double t5366;
  double t3219;
  double t4713;
  double t4773;
  double t4912;
  double t4939;
  double t5020;
  double t5203;
  double t5208;
  double t5213;
  double t5345;
  double t5346;
  double t5354;
  double t5627;
  double t5631;
  double t5633;
  double t5576;
  double t5590;
  double t5591;
  double t5672;
  double t5673;
  double t5709;
  double t5885;
  double t5888;
  double t5890;
  t655 = Cos(var1[3]);
  t2195 = Cos(var1[11]);
  t2733 = -1.*t2195;
  t3144 = 1. + t2733;
  t4623 = Sin(var1[11]);
  t5039 = Cos(var1[5]);
  t5054 = Sin(var1[3]);
  t5052 = Sin(var1[4]);
  t5104 = Sin(var1[5]);
  t4865 = Cos(var1[12]);
  t4881 = -1.*t4865;
  t4901 = 1. + t4881;
  t4914 = Sin(var1[12]);
  t2192 = Cos(var1[4]);
  t5239 = -1.*t5039*t5054;
  t5247 = t655*t5052*t5104;
  t5290 = t5239 + t5247;
  t5053 = t655*t5039*t5052;
  t5156 = t5054*t5104;
  t5167 = t5053 + t5156;
  t5359 = -1.*t655*t2192*t4623;
  t5360 = t2195*t5290;
  t5366 = t5359 + t5360;
  t3219 = -0.022225*t3144;
  t4713 = -0.086996*t4623;
  t4773 = 0. + t3219 + t4713;
  t4912 = -0.31508*t4901;
  t4939 = 0.156996*t4914;
  t5020 = 0. + t4912 + t4939;
  t5203 = -0.086996*t3144;
  t5208 = 0.022225*t4623;
  t5213 = 0. + t5203 + t5208;
  t5345 = -0.156996*t4901;
  t5346 = -0.31508*t4914;
  t5354 = 0. + t5345 + t5346;
  t5627 = t655*t5039;
  t5631 = t5054*t5052*t5104;
  t5633 = t5627 + t5631;
  t5576 = t5039*t5054*t5052;
  t5590 = -1.*t655*t5104;
  t5591 = t5576 + t5590;
  t5672 = -1.*t2192*t4623*t5054;
  t5673 = t2195*t5633;
  t5709 = t5672 + t5673;
  t5885 = t4623*t5052;
  t5888 = t2195*t2192*t5104;
  t5890 = t5885 + t5888;

  p_output1(0)=0. + t5020*t5167 + t5213*t5290 + t5354*t5366 - 0.156996*(t4914*t5167 + t4865*t5366) - 0.31508*(t4865*t5167 - 1.*t4914*t5366) + t2192*t4773*t655 - 0.022225*(t4623*t5290 + t2192*t2195*t655) + var1(0);
  p_output1(1)=0. + t2192*t4773*t5054 + t5020*t5591 + t5213*t5633 - 0.022225*(t2192*t2195*t5054 + t4623*t5633) + t5354*t5709 - 0.156996*(t4914*t5591 + t4865*t5709) - 0.31508*(t4865*t5591 - 1.*t4914*t5709) + var1(1);
  p_output1(2)=0. + t2192*t5020*t5039 - 1.*t4773*t5052 - 0.022225*(-1.*t2195*t5052 + t2192*t4623*t5104) + t2192*t5104*t5213 + t5354*t5890 - 0.156996*(t2192*t4914*t5039 + t4865*t5890) - 0.31508*(t2192*t4865*t5039 - 1.*t4914*t5890) + var1(2);
}


       
void p_rHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
