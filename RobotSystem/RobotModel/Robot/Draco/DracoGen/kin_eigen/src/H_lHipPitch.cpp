/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:37 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipPitch.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t918;
  double t1903;
  double t2428;
  double t2191;
  double t2440;
  double t1704;
  double t2228;
  double t2461;
  double t2477;
  double t1252;
  double t2509;
  double t182;
  double t2707;
  double t3170;
  double t3176;
  double t3210;
  double t2951;
  double t3077;
  double t2880;
  double t2886;
  double t2926;
  double t2709;
  double t2788;
  double t2796;
  double t3508;
  double t3519;
  double t3539;
  double t3284;
  double t3445;
  double t3503;
  double t3800;
  double t3899;
  double t3928;
  double t2807;
  double t2960;
  double t2964;
  double t1806;
  double t2654;
  double t2663;
  double t3507;
  double t3563;
  double t3571;
  double t3165;
  double t3211;
  double t3215;
  double t3790;
  double t3949;
  double t4147;
  double t3675;
  double t3690;
  double t3756;
  double t5043;
  double t5171;
  double t5429;
  double t5443;
  double t4186;
  double t4370;
  double t4492;
  double t5703;
  double t5712;
  double t4771;
  double t4864;
  double t4878;
  double t2685;
  double t3108;
  double t3152;
  double t5180;
  double t5181;
  double t5189;
  double t5253;
  double t5326;
  double t5394;
  double t5449;
  double t5453;
  double t5466;
  double t5577;
  double t5583;
  double t5617;
  double t4543;
  double t4606;
  double t4651;
  double t5713;
  double t5722;
  double t5741;
  double t5772;
  double t5775;
  double t5788;
  double t4879;
  double t4902;
  double t4903;
  double t3242;
  double t3601;
  double t3665;
  double t4725;
  double t4726;
  double t4765;
  double t5005;
  double t5012;
  double t5040;
  double t3780;
  double t4149;
  double t4171;
  t918 = Cos(var1[3]);
  t1903 = Cos(var1[5]);
  t2428 = Sin(var1[4]);
  t2191 = Sin(var1[3]);
  t2440 = Sin(var1[5]);
  t1704 = Cos(var1[6]);
  t2228 = -1.*t1903*t2191;
  t2461 = t918*t2428*t2440;
  t2477 = t2228 + t2461;
  t1252 = Cos(var1[4]);
  t2509 = Sin(var1[6]);
  t182 = Cos(var1[8]);
  t2707 = Cos(var1[7]);
  t3170 = t918*t1903;
  t3176 = t2191*t2428*t2440;
  t3210 = t3170 + t3176;
  t2951 = Sin(var1[7]);
  t3077 = Sin(var1[8]);
  t2880 = t1704*t2477;
  t2886 = -1.*t918*t1252*t2509;
  t2926 = t2880 + t2886;
  t2709 = t918*t1903*t2428;
  t2788 = t2191*t2440;
  t2796 = t2709 + t2788;
  t3508 = t1704*t3210;
  t3519 = -1.*t1252*t2191*t2509;
  t3539 = t3508 + t3519;
  t3284 = t1903*t2191*t2428;
  t3445 = -1.*t918*t2440;
  t3503 = t3284 + t3445;
  t3800 = t1252*t1704*t2440;
  t3899 = t2428*t2509;
  t3928 = t3800 + t3899;
  t2807 = t2707*t2796;
  t2960 = -1.*t2926*t2951;
  t2964 = t2807 + t2960;
  t1806 = t918*t1252*t1704;
  t2654 = t2477*t2509;
  t2663 = t1806 + t2654;
  t3507 = t2707*t3503;
  t3563 = -1.*t3539*t2951;
  t3571 = t3507 + t3563;
  t3165 = t1252*t1704*t2191;
  t3211 = t3210*t2509;
  t3215 = t3165 + t3211;
  t3790 = t1252*t1903*t2707;
  t3949 = -1.*t3928*t2951;
  t4147 = t3790 + t3949;
  t3675 = -1.*t1704*t2428;
  t3690 = t1252*t2440*t2509;
  t3756 = t3675 + t3690;
  t5043 = -1.*t1704;
  t5171 = 1. + t5043;
  t5429 = -1.*t2707;
  t5443 = 1. + t5429;
  t4186 = t2707*t2926;
  t4370 = t2796*t2951;
  t4492 = t4186 + t4370;
  t5703 = -1.*t182;
  t5712 = 1. + t5703;
  t4771 = t182*t2964;
  t4864 = t2663*t3077;
  t4878 = t4771 + t4864;
  t2685 = t182*t2663;
  t3108 = -1.*t2964*t3077;
  t3152 = t2685 + t3108;
  t5180 = 0.087004*t5171;
  t5181 = 0.022225*t2509;
  t5189 = 0. + t5180 + t5181;
  t5253 = -0.022225*t5171;
  t5326 = 0.087004*t2509;
  t5394 = 0. + t5253 + t5326;
  t5449 = 0.157004*t5443;
  t5453 = -0.31508*t2951;
  t5466 = 0. + t5449 + t5453;
  t5577 = -0.31508*t5443;
  t5583 = -0.157004*t2951;
  t5617 = 0. + t5577 + t5583;
  t4543 = t2707*t3539;
  t4606 = t3503*t2951;
  t4651 = t4543 + t4606;
  t5713 = -0.38008*t5712;
  t5722 = -0.022225*t3077;
  t5741 = 0. + t5713 + t5722;
  t5772 = -0.022225*t5712;
  t5775 = 0.38008*t3077;
  t5788 = 0. + t5772 + t5775;
  t4879 = t182*t3571;
  t4902 = t3215*t3077;
  t4903 = t4879 + t4902;
  t3242 = t182*t3215;
  t3601 = -1.*t3571*t3077;
  t3665 = t3242 + t3601;
  t4725 = t2707*t3928;
  t4726 = t1252*t1903*t2951;
  t4765 = t4725 + t4726;
  t5005 = t182*t4147;
  t5012 = t3756*t3077;
  t5040 = t5005 + t5012;
  t3780 = t182*t3756;
  t4149 = -1.*t4147*t3077;
  t4171 = t3780 + t4149;

  p_output1(0)=t3152;
  p_output1(1)=t3665;
  p_output1(2)=t4171;
  p_output1(3)=0.;
  p_output1(4)=t4492;
  p_output1(5)=t4651;
  p_output1(6)=t4765;
  p_output1(7)=0.;
  p_output1(8)=t4878;
  p_output1(9)=t4903;
  p_output1(10)=t5040;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3152 + 0.167004*t4492 - 0.38008*t4878 + t2477*t5189 + t2926*t5466 + t2796*t5617 + t2964*t5741 + t2663*t5788 + t1252*t5394*t918 + var1(0);
  p_output1(13)=0. - 0.022225*t3665 + 0.167004*t4651 - 0.38008*t4903 + t3210*t5189 + t1252*t2191*t5394 + t3539*t5466 + t3503*t5617 + t3571*t5741 + t3215*t5788 + var1(1);
  p_output1(14)=0. - 0.022225*t4171 + 0.167004*t4765 - 0.38008*t5040 + t1252*t2440*t5189 - 1.*t2428*t5394 + t3928*t5466 + t1252*t1903*t5617 + t4147*t5741 + t3756*t5788 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
