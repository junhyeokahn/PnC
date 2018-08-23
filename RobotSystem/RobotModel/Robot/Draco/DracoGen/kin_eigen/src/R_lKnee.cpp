/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:20 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lKnee.h"

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
  double t2089;
  double t2284;
  double t1862;
  double t2281;
  double t2566;
  double t3294;
  double t3192;
  double t3197;
  double t3235;
  double t3260;
  double t3302;
  double t1505;
  double t3380;
  double t3385;
  double t3414;
  double t1582;
  double t2282;
  double t2913;
  double t3143;
  double t3166;
  double t3288;
  double t3315;
  double t3337;
  double t3344;
  double t3357;
  double t3365;
  double t3416;
  double t1400;
  double t3495;
  double t3496;
  double t3522;
  double t3465;
  double t3654;
  double t3681;
  double t3702;
  double t3485;
  double t3486;
  double t3491;
  double t3492;
  double t3530;
  double t3544;
  double t3548;
  double t3566;
  double t3594;
  double t3882;
  double t3884;
  double t3885;
  double t3840;
  double t3841;
  double t3842;
  double t3848;
  double t3854;
  double t3855;
  double t3371;
  double t3431;
  double t3437;
  double t3472;
  double t3473;
  double t3474;
  double t3615;
  double t3721;
  double t3733;
  double t3752;
  double t3811;
  double t3815;
  double t3872;
  double t3886;
  double t3888;
  double t3891;
  double t3898;
  double t3900;
  t2089 = Cos(var1[5]);
  t2284 = Sin(var1[3]);
  t1862 = Cos(var1[3]);
  t2281 = Sin(var1[4]);
  t2566 = Sin(var1[5]);
  t3294 = Cos(var1[4]);
  t3192 = Cos(var1[6]);
  t3197 = -1.*t2089*t2284;
  t3235 = t1862*t2281*t2566;
  t3260 = t3197 + t3235;
  t3302 = Sin(var1[6]);
  t1505 = Cos(var1[8]);
  t3380 = t1862*t3294*t3192;
  t3385 = t3260*t3302;
  t3414 = t3380 + t3385;
  t1582 = Cos(var1[7]);
  t2282 = t1862*t2089*t2281;
  t2913 = t2284*t2566;
  t3143 = t2282 + t2913;
  t3166 = t1582*t3143;
  t3288 = t3192*t3260;
  t3315 = -1.*t1862*t3294*t3302;
  t3337 = t3288 + t3315;
  t3344 = Sin(var1[7]);
  t3357 = -1.*t3337*t3344;
  t3365 = t3166 + t3357;
  t3416 = Sin(var1[8]);
  t1400 = Sin(var1[9]);
  t3495 = t1862*t2089;
  t3496 = t2284*t2281*t2566;
  t3522 = t3495 + t3496;
  t3465 = Cos(var1[9]);
  t3654 = t3294*t3192*t2284;
  t3681 = t3522*t3302;
  t3702 = t3654 + t3681;
  t3485 = t2089*t2284*t2281;
  t3486 = -1.*t1862*t2566;
  t3491 = t3485 + t3486;
  t3492 = t1582*t3491;
  t3530 = t3192*t3522;
  t3544 = -1.*t3294*t2284*t3302;
  t3548 = t3530 + t3544;
  t3566 = -1.*t3548*t3344;
  t3594 = t3492 + t3566;
  t3882 = -1.*t3192*t2281;
  t3884 = t3294*t2566*t3302;
  t3885 = t3882 + t3884;
  t3840 = t3294*t2089*t1582;
  t3841 = t3294*t3192*t2566;
  t3842 = t2281*t3302;
  t3848 = t3841 + t3842;
  t3854 = -1.*t3848*t3344;
  t3855 = t3840 + t3854;
  t3371 = t1505*t3365;
  t3431 = t3414*t3416;
  t3437 = t3371 + t3431;
  t3472 = t1505*t3414;
  t3473 = -1.*t3365*t3416;
  t3474 = t3472 + t3473;
  t3615 = t1505*t3594;
  t3721 = t3702*t3416;
  t3733 = t3615 + t3721;
  t3752 = t1505*t3702;
  t3811 = -1.*t3594*t3416;
  t3815 = t3752 + t3811;
  t3872 = t1505*t3855;
  t3886 = t3885*t3416;
  t3888 = t3872 + t3886;
  t3891 = t1505*t3885;
  t3898 = -1.*t3855*t3416;
  t3900 = t3891 + t3898;

  p_output1(0)=-1.*t1400*t3437 + t3465*t3474;
  p_output1(1)=-1.*t1400*t3733 + t3465*t3815;
  p_output1(2)=-1.*t1400*t3888 + t3465*t3900;
  p_output1(3)=t1582*t3337 + t3143*t3344;
  p_output1(4)=t3344*t3491 + t1582*t3548;
  p_output1(5)=t2089*t3294*t3344 + t1582*t3848;
  p_output1(6)=t3437*t3465 + t1400*t3474;
  p_output1(7)=t3465*t3733 + t1400*t3815;
  p_output1(8)=t3465*t3888 + t1400*t3900;
}


       
void R_lKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
