/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootBack.h"

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
  double t401;
  double t417;
  double t3200;
  double t57;
  double t3388;
  double t3396;
  double t3419;
  double t3431;
  double t3543;
  double t3548;
  double t3550;
  double t3577;
  double t3532;
  double t3537;
  double t3539;
  double t3601;
  double t3605;
  double t3613;
  double t3634;
  double t3637;
  double t3639;
  double t3648;
  double t3621;
  double t3624;
  double t3631;
  double t3683;
  double t3685;
  double t3686;
  double t180;
  double t208;
  double t839;
  double t911;
  double t1177;
  double t3267;
  double t3328;
  double t3424;
  double t3473;
  double t3474;
  double t3484;
  double t3496;
  double t3502;
  double t3553;
  double t3598;
  double t3599;
  double t3614;
  double t3615;
  double t3616;
  double t3754;
  double t3755;
  double t3759;
  double t3769;
  double t3770;
  double t3772;
  double t3645;
  double t3676;
  double t3678;
  double t3687;
  double t3688;
  double t3690;
  double t3775;
  double t3776;
  double t3779;
  double t3785;
  double t3788;
  double t3790;
  double t3846;
  double t3852;
  double t3853;
  double t3855;
  double t3857;
  double t3859;
  double t3861;
  double t3862;
  double t3869;
  double t3870;
  double t3871;
  t401 = Sin(var1[0]);
  t417 = Cos(var1[1]);
  t3200 = Sin(var1[1]);
  t57 = Cos(var1[0]);
  t3388 = Cos(var1[2]);
  t3396 = -1.*t3388;
  t3419 = 1. + t3396;
  t3431 = Sin(var1[2]);
  t3543 = Cos(var1[3]);
  t3548 = -1.*t3543;
  t3550 = 1. + t3548;
  t3577 = Sin(var1[3]);
  t3532 = t3388*t401*t3200;
  t3537 = t57*t3431;
  t3539 = t3532 + t3537;
  t3601 = t57*t3388;
  t3605 = -1.*t401*t3200*t3431;
  t3613 = t3601 + t3605;
  t3634 = Cos(var1[4]);
  t3637 = -1.*t3634;
  t3639 = 1. + t3637;
  t3648 = Sin(var1[4]);
  t3621 = t3543*t3539;
  t3624 = t3613*t3577;
  t3631 = t3621 + t3624;
  t3683 = t3543*t3613;
  t3685 = -1.*t3539*t3577;
  t3686 = t3683 + t3685;
  t180 = -1.*t57;
  t208 = 1. + t180;
  t839 = -1.*t417;
  t911 = 1. + t839;
  t1177 = 0.331012*t911;
  t3267 = -0.90524*t3200;
  t3328 = 0. + t1177 + t3267;
  t3424 = -0.97024*t3419;
  t3473 = -0.066675*t3431;
  t3474 = 0. + t3424 + t3473;
  t3484 = -0.066675*t3419;
  t3496 = 0.97024*t3431;
  t3502 = 0. + t3484 + t3496;
  t3553 = -1.45024*t3550;
  t3598 = -0.066675*t3577;
  t3599 = 0. + t3553 + t3598;
  t3614 = -0.066675*t3550;
  t3615 = 1.45024*t3577;
  t3616 = 0. + t3614 + t3615;
  t3754 = -1.*t57*t3388*t3200;
  t3755 = t401*t3431;
  t3759 = t3754 + t3755;
  t3769 = t3388*t401;
  t3770 = t57*t3200*t3431;
  t3772 = t3769 + t3770;
  t3645 = -1.93024*t3639;
  t3676 = -0.065597*t3648;
  t3678 = 0. + t3645 + t3676;
  t3687 = -0.065597*t3639;
  t3688 = 1.93024*t3648;
  t3690 = 0. + t3687 + t3688;
  t3775 = t3543*t3759;
  t3776 = t3772*t3577;
  t3779 = t3775 + t3776;
  t3785 = t3543*t3772;
  t3788 = -1.*t3759*t3577;
  t3790 = t3785 + t3788;
  t3846 = 0. + t417;
  t3852 = t3846*t3388;
  t3853 = 0. + t3852;
  t3855 = -1.*t3846*t3431;
  t3857 = 0. + t3855;
  t3859 = t3853*t3543;
  t3861 = t3857*t3577;
  t3862 = t3859 + t3861;
  t3869 = t3543*t3857;
  t3870 = -1.*t3853*t3577;
  t3871 = t3869 + t3870;

  p_output1(0)=0. - 0.066675*t208 + t3539*t3599 + t3613*t3616 + t3631*t3678 - 0.000645*(-1.*t3631*t3648 + t3634*t3686) - 1.990292*(t3631*t3634 + t3648*t3686) + t3686*t3690 + 0.261012*t401 - 1.*t3328*t401 + t3200*t3474*t401 - 0.341012*t401*t417 + t3502*t57;
  p_output1(1)=0. + 0.261012*t208 + t3599*t3759 + t3616*t3772 + t3678*t3779 + t3690*t3790 - 0.000645*(-1.*t3648*t3779 + t3634*t3790) - 1.990292*(t3634*t3779 + t3648*t3790) + 0.066675*t401 + t3502*t401 + t3328*t57 - 1.*t3200*t3474*t57 + 0.341012*t417*t57;
  p_output1(2)=0. - 0.331012*t3200 + 0.341012*(0. + t3200) + t3474*t3846 + t3599*t3853 + t3616*t3857 + t3678*t3862 + t3690*t3871 - 0.000645*(-1.*t3648*t3862 + t3634*t3871) - 1.990292*(t3634*t3862 + t3648*t3871) - 0.90524*t911;
}


       
void p_LeftFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
