/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:06 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootFront.h"

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
  double t900;
  double t1526;
  double t795;
  double t1173;
  double t1533;
  double t2518;
  double t1952;
  double t2371;
  double t2406;
  double t2441;
  double t2659;
  double t778;
  double t3139;
  double t3146;
  double t3170;
  double t788;
  double t1260;
  double t1699;
  double t1859;
  double t1897;
  double t2500;
  double t2664;
  double t2716;
  double t2746;
  double t2963;
  double t2981;
  double t3238;
  double t3400;
  double t3022;
  double t3271;
  double t3293;
  double t557;
  double t3433;
  double t3448;
  double t3481;
  double t3550;
  double t3349;
  double t3497;
  double t3515;
  double t272;
  double t3558;
  double t3676;
  double t3678;
  double t4074;
  double t4099;
  double t4190;
  double t4351;
  double t4415;
  double t4482;
  double t3863;
  double t3909;
  double t4020;
  double t4042;
  double t4202;
  double t4247;
  double t4256;
  double t4266;
  double t4322;
  double t4343;
  double t4507;
  double t4529;
  double t4560;
  double t4628;
  double t4735;
  double t4542;
  double t4738;
  double t4781;
  double t4793;
  double t4828;
  double t4847;
  double t5088;
  double t5090;
  double t5143;
  double t4919;
  double t4968;
  double t4976;
  double t4980;
  double t4992;
  double t5015;
  double t5038;
  double t5161;
  double t5162;
  double t5166;
  double t5168;
  double t5171;
  double t5164;
  double t5191;
  double t5202;
  double t5262;
  double t5265;
  double t5287;
  double t3543;
  double t3727;
  double t4792;
  double t4858;
  double t5225;
  double t5303;
  t900 = Cos(var1[5]);
  t1526 = Sin(var1[3]);
  t795 = Cos(var1[3]);
  t1173 = Sin(var1[4]);
  t1533 = Sin(var1[5]);
  t2518 = Cos(var1[4]);
  t1952 = Cos(var1[6]);
  t2371 = -1.*t900*t1526;
  t2406 = t795*t1173*t1533;
  t2441 = t2371 + t2406;
  t2659 = Sin(var1[6]);
  t778 = Cos(var1[8]);
  t3139 = t795*t2518*t1952;
  t3146 = t2441*t2659;
  t3170 = t3139 + t3146;
  t788 = Cos(var1[7]);
  t1260 = t795*t900*t1173;
  t1699 = t1526*t1533;
  t1859 = t1260 + t1699;
  t1897 = t788*t1859;
  t2500 = t1952*t2441;
  t2664 = -1.*t795*t2518*t2659;
  t2716 = t2500 + t2664;
  t2746 = Sin(var1[7]);
  t2963 = -1.*t2716*t2746;
  t2981 = t1897 + t2963;
  t3238 = Sin(var1[8]);
  t3400 = Cos(var1[9]);
  t3022 = t778*t2981;
  t3271 = t3170*t3238;
  t3293 = t3022 + t3271;
  t557 = Sin(var1[9]);
  t3433 = t778*t3170;
  t3448 = -1.*t2981*t3238;
  t3481 = t3433 + t3448;
  t3550 = Cos(var1[10]);
  t3349 = -1.*t557*t3293;
  t3497 = t3400*t3481;
  t3515 = t3349 + t3497;
  t272 = Sin(var1[10]);
  t3558 = t3400*t3293;
  t3676 = t557*t3481;
  t3678 = t3558 + t3676;
  t4074 = t795*t900;
  t4099 = t1526*t1173*t1533;
  t4190 = t4074 + t4099;
  t4351 = t2518*t1952*t1526;
  t4415 = t4190*t2659;
  t4482 = t4351 + t4415;
  t3863 = t900*t1526*t1173;
  t3909 = -1.*t795*t1533;
  t4020 = t3863 + t3909;
  t4042 = t788*t4020;
  t4202 = t1952*t4190;
  t4247 = -1.*t2518*t1526*t2659;
  t4256 = t4202 + t4247;
  t4266 = -1.*t4256*t2746;
  t4322 = t4042 + t4266;
  t4343 = t778*t4322;
  t4507 = t4482*t3238;
  t4529 = t4343 + t4507;
  t4560 = t778*t4482;
  t4628 = -1.*t4322*t3238;
  t4735 = t4560 + t4628;
  t4542 = -1.*t557*t4529;
  t4738 = t3400*t4735;
  t4781 = t4542 + t4738;
  t4793 = t3400*t4529;
  t4828 = t557*t4735;
  t4847 = t4793 + t4828;
  t5088 = -1.*t1952*t1173;
  t5090 = t2518*t1533*t2659;
  t5143 = t5088 + t5090;
  t4919 = t2518*t900*t788;
  t4968 = t2518*t1952*t1533;
  t4976 = t1173*t2659;
  t4980 = t4968 + t4976;
  t4992 = -1.*t4980*t2746;
  t5015 = t4919 + t4992;
  t5038 = t778*t5015;
  t5161 = t5143*t3238;
  t5162 = t5038 + t5161;
  t5166 = t778*t5143;
  t5168 = -1.*t5015*t3238;
  t5171 = t5166 + t5168;
  t5164 = -1.*t557*t5162;
  t5191 = t3400*t5171;
  t5202 = t5164 + t5191;
  t5262 = t3400*t5162;
  t5265 = t557*t5171;
  t5287 = t5262 + t5265;
  t3543 = t272*t3515;
  t3727 = t3550*t3678;
  t4792 = t272*t4781;
  t4858 = t3550*t4847;
  t5225 = t272*t5202;
  t5303 = t3550*t5287;

  p_output1(0)=t3543 + 0.000796*(t3515*t3550 - 1.*t272*t3678) + t3727;
  p_output1(1)=t4792 + 0.000796*(t3550*t4781 - 1.*t272*t4847) + t4858;
  p_output1(2)=t5225 + 0.000796*(t3550*t5202 - 1.*t272*t5287) + t5303;
  p_output1(3)=t1859*t2746 + t2716*t788;
  p_output1(4)=t2746*t4020 + t4256*t788;
  p_output1(5)=t4980*t788 + t2518*t2746*t900;
  p_output1(6)=-1.*t3515*t3550 + t272*t3678 + 0.000796*(t3543 + t3727);
  p_output1(7)=-1.*t3550*t4781 + t272*t4847 + 0.000796*(t4792 + t4858);
  p_output1(8)=-1.*t3550*t5202 + t272*t5287 + 0.000796*(t5225 + t5303);
}


       
void R_LeftFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
