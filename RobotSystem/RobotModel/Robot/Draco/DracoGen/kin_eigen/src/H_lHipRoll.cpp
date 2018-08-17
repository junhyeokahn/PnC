/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:34 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipRoll.h"

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
  double t81;
  double t482;
  double t1693;
  double t1907;
  double t1862;
  double t2032;
  double t2117;
  double t2463;
  double t1969;
  double t2274;
  double t2302;
  double t4081;
  double t2811;
  double t2894;
  double t2905;
  double t4712;
  double t4340;
  double t4554;
  double t4558;
  double t4206;
  double t4251;
  double t4265;
  double t5183;
  double t5186;
  double t5202;
  double t4943;
  double t4968;
  double t5119;
  double t5277;
  double t5316;
  double t5350;
  double t5527;
  double t5544;
  double t1741;
  double t2467;
  double t2604;
  double t5614;
  double t5618;
  double t4318;
  double t4727;
  double t4844;
  double t5427;
  double t5434;
  double t5450;
  double t5547;
  double t5550;
  double t5551;
  double t5586;
  double t5594;
  double t5604;
  double t2797;
  double t2978;
  double t3271;
  double t5624;
  double t5625;
  double t5638;
  double t5642;
  double t5651;
  double t5659;
  double t5166;
  double t5257;
  double t5270;
  double t5451;
  double t5459;
  double t5494;
  double t3274;
  double t3719;
  double t4071;
  double t5351;
  double t5380;
  double t5409;
  double t5500;
  double t5517;
  double t5524;
  t81 = Cos(var1[3]);
  t482 = Cos(var1[4]);
  t1693 = Cos(var1[6]);
  t1907 = Sin(var1[3]);
  t1862 = Cos(var1[5]);
  t2032 = Sin(var1[4]);
  t2117 = Sin(var1[5]);
  t2463 = Sin(var1[6]);
  t1969 = -1.*t1862*t1907;
  t2274 = t81*t2032*t2117;
  t2302 = t1969 + t2274;
  t4081 = Cos(var1[7]);
  t2811 = t81*t1862;
  t2894 = t1907*t2032*t2117;
  t2905 = t2811 + t2894;
  t4712 = Sin(var1[7]);
  t4340 = t81*t1862*t2032;
  t4554 = t1907*t2117;
  t4558 = t4340 + t4554;
  t4206 = t1693*t2302;
  t4251 = -1.*t81*t482*t2463;
  t4265 = t4206 + t4251;
  t5183 = t1862*t1907*t2032;
  t5186 = -1.*t81*t2117;
  t5202 = t5183 + t5186;
  t4943 = t1693*t2905;
  t4968 = -1.*t482*t1907*t2463;
  t5119 = t4943 + t4968;
  t5277 = t482*t1693*t2117;
  t5316 = t2032*t2463;
  t5350 = t5277 + t5316;
  t5527 = -1.*t1693;
  t5544 = 1. + t5527;
  t1741 = t81*t482*t1693;
  t2467 = t2302*t2463;
  t2604 = t1741 + t2467;
  t5614 = -1.*t4081;
  t5618 = 1. + t5614;
  t4318 = t4081*t4265;
  t4727 = t4558*t4712;
  t4844 = t4318 + t4727;
  t5427 = t4081*t4558;
  t5434 = -1.*t4265*t4712;
  t5450 = t5427 + t5434;
  t5547 = 0.087004*t5544;
  t5550 = 0.022225*t2463;
  t5551 = 0. + t5547 + t5550;
  t5586 = -0.022225*t5544;
  t5594 = 0.087004*t2463;
  t5604 = 0. + t5586 + t5594;
  t2797 = t482*t1693*t1907;
  t2978 = t2905*t2463;
  t3271 = t2797 + t2978;
  t5624 = 0.157004*t5618;
  t5625 = -0.31508*t4712;
  t5638 = 0. + t5624 + t5625;
  t5642 = -0.31508*t5618;
  t5651 = -0.157004*t4712;
  t5659 = 0. + t5642 + t5651;
  t5166 = t4081*t5119;
  t5257 = t5202*t4712;
  t5270 = t5166 + t5257;
  t5451 = t4081*t5202;
  t5459 = -1.*t5119*t4712;
  t5494 = t5451 + t5459;
  t3274 = -1.*t1693*t2032;
  t3719 = t482*t2117*t2463;
  t4071 = t3274 + t3719;
  t5351 = t4081*t5350;
  t5380 = t482*t1862*t4712;
  t5409 = t5351 + t5380;
  t5500 = t482*t1862*t4081;
  t5517 = -1.*t5350*t4712;
  t5524 = t5500 + t5517;

  p_output1(0)=t2604;
  p_output1(1)=t3271;
  p_output1(2)=t4071;
  p_output1(3)=0.;
  p_output1(4)=t4844;
  p_output1(5)=t5270;
  p_output1(6)=t5409;
  p_output1(7)=0.;
  p_output1(8)=t5450;
  p_output1(9)=t5494;
  p_output1(10)=t5524;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2604 + 0.157004*t4844 - 0.31508*t5450 + t2302*t5551 + t4265*t5638 + t4558*t5659 + t482*t5604*t81 + var1(0);
  p_output1(13)=0. - 0.022225*t3271 + 0.157004*t5270 - 0.31508*t5494 + t2905*t5551 + t1907*t482*t5604 + t5119*t5638 + t5202*t5659 + var1(1);
  p_output1(14)=0. - 0.022225*t4071 + 0.157004*t5409 - 0.31508*t5524 + t2117*t482*t5551 - 1.*t2032*t5604 + t5350*t5638 + t1862*t482*t5659 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
