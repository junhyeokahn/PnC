/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:35 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t741;
  double t2469;
  double t2470;
  double t2978;
  double t2905;
  double t3650;
  double t3719;
  double t4340;
  double t3396;
  double t3937;
  double t3966;
  double t5524;
  double t5316;
  double t5351;
  double t5450;
  double t5608;
  double t5586;
  double t5594;
  double t5607;
  double t5547;
  double t5550;
  double t5551;
  double t5659;
  double t5667;
  double t5671;
  double t5638;
  double t5639;
  double t5642;
  double t5678;
  double t5682;
  double t5685;
  t741 = Cos(var1[3]);
  t2469 = Cos(var1[4]);
  t2470 = Cos(var1[6]);
  t2978 = Sin(var1[3]);
  t2905 = Cos(var1[5]);
  t3650 = Sin(var1[4]);
  t3719 = Sin(var1[5]);
  t4340 = Sin(var1[6]);
  t3396 = -1.*t2905*t2978;
  t3937 = t741*t3650*t3719;
  t3966 = t3396 + t3937;
  t5524 = Cos(var1[7]);
  t5316 = t741*t2905;
  t5351 = t2978*t3650*t3719;
  t5450 = t5316 + t5351;
  t5608 = Sin(var1[7]);
  t5586 = t741*t2905*t3650;
  t5594 = t2978*t3719;
  t5607 = t5586 + t5594;
  t5547 = t2470*t3966;
  t5550 = -1.*t741*t2469*t4340;
  t5551 = t5547 + t5550;
  t5659 = t2905*t2978*t3650;
  t5667 = -1.*t741*t3719;
  t5671 = t5659 + t5667;
  t5638 = t2470*t5450;
  t5639 = -1.*t2469*t2978*t4340;
  t5642 = t5638 + t5639;
  t5678 = t2469*t2470*t3719;
  t5682 = t3650*t4340;
  t5685 = t5678 + t5682;

  p_output1(0)=t3966*t4340 + t2469*t2470*t741;
  p_output1(1)=t2469*t2470*t2978 + t4340*t5450;
  p_output1(2)=-1.*t2470*t3650 + t2469*t3719*t4340;
  p_output1(3)=t5524*t5551 + t5607*t5608;
  p_output1(4)=t5524*t5642 + t5608*t5671;
  p_output1(5)=t2469*t2905*t5608 + t5524*t5685;
  p_output1(6)=t5524*t5607 - 1.*t5551*t5608;
  p_output1(7)=-1.*t5608*t5642 + t5524*t5671;
  p_output1(8)=t2469*t2905*t5524 - 1.*t5608*t5685;
}


       
void R_lHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
