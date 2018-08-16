/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:17 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootFront.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1817;
  double t1267;
  double t1316;
  double t1453;
  double t1870;
  double t624;
  double t2101;
  double t2104;
  double t2108;
  double t1484;
  double t1978;
  double t2030;
  double t2293;
  double t352;
  double t2759;
  double t2791;
  double t2832;
  double t2090;
  double t2365;
  double t2528;
  double t2836;
  double t3202;
  double t3240;
  double t3247;
  double t3148;
  double t3187;
  double t3195;
  double t3267;
  double t3271;
  double t3281;
  double t3200;
  double t3248;
  double t3250;
  double t3332;
  double t3336;
  double t3345;
  double t3348;
  double t3337;
  double t3338;
  double t3381;
  double t3383;
  double t3389;
  double t3339;
  double t3354;
  double t3360;
  double t2650;
  double t2876;
  double t3263;
  double t3290;
  double t3375;
  double t3394;
  double t3534;
  double t3535;
  double t3559;
  double t3561;
  double t3579;
  double t3582;
  double t3438;
  double t3032;
  double t3038;
  double t3065;
  double t3495;
  double t3498;
  double t3520;
  double t3527;
  double t3528;
  double t3530;
  double t3531;
  double t3539;
  double t3543;
  double t3545;
  double t3550;
  double t3553;
  double t3554;
  double t3565;
  double t3567;
  double t3568;
  double t3570;
  double t3574;
  double t3576;
  double t3583;
  double t3584;
  double t3589;
  double t3600;
  double t3601;
  double t3605;
  double t3471;
  double t3301;
  double t3317;
  double t3319;
  double t3424;
  double t3484;
  double t3396;
  double t3398;
  double t3400;
  t1817 = Cos(var1[0]);
  t1267 = Cos(var1[2]);
  t1316 = Sin(var1[0]);
  t1453 = Sin(var1[1]);
  t1870 = Sin(var1[2]);
  t624 = Cos(var1[3]);
  t2101 = t1817*t1267;
  t2104 = -1.*t1316*t1453*t1870;
  t2108 = t2101 + t2104;
  t1484 = t1267*t1316*t1453;
  t1978 = t1817*t1870;
  t2030 = t1484 + t1978;
  t2293 = Sin(var1[3]);
  t352 = Cos(var1[4]);
  t2759 = t624*t2108;
  t2791 = -1.*t2030*t2293;
  t2832 = t2759 + t2791;
  t2090 = t624*t2030;
  t2365 = t2108*t2293;
  t2528 = t2090 + t2365;
  t2836 = Sin(var1[4]);
  t3202 = t1267*t1316;
  t3240 = t1817*t1453*t1870;
  t3247 = t3202 + t3240;
  t3148 = -1.*t1817*t1267*t1453;
  t3187 = t1316*t1870;
  t3195 = t3148 + t3187;
  t3267 = t624*t3247;
  t3271 = -1.*t3195*t2293;
  t3281 = t3267 + t3271;
  t3200 = t624*t3195;
  t3248 = t3247*t2293;
  t3250 = t3200 + t3248;
  t3332 = Cos(var1[1]);
  t3336 = 0. + t3332;
  t3345 = -1.*t3336*t1870;
  t3348 = 0. + t3345;
  t3337 = t3336*t1267;
  t3338 = 0. + t3337;
  t3381 = t624*t3348;
  t3383 = -1.*t3338*t2293;
  t3389 = t3381 + t3383;
  t3339 = t3338*t624;
  t3354 = t3348*t2293;
  t3360 = t3339 + t3354;
  t2650 = t352*t2528;
  t2876 = t2832*t2836;
  t3263 = t352*t3250;
  t3290 = t3281*t2836;
  t3375 = t352*t3360;
  t3394 = t3389*t2836;
  t3534 = -1.*t1267;
  t3535 = 1. + t3534;
  t3559 = -1.*t624;
  t3561 = 1. + t3559;
  t3579 = -1.*t352;
  t3582 = 1. + t3579;
  t3438 = t2650 + t2876;
  t3032 = t352*t2832;
  t3038 = -1.*t2528*t2836;
  t3065 = t3032 + t3038;
  t3495 = -1.*t1817;
  t3498 = 1. + t3495;
  t3520 = -1.*t3332;
  t3527 = 1. + t3520;
  t3528 = 0.331012*t3527;
  t3530 = -0.90524*t1453;
  t3531 = 0. + t3528 + t3530;
  t3539 = -0.97024*t3535;
  t3543 = -0.066675*t1870;
  t3545 = 0. + t3539 + t3543;
  t3550 = -0.066675*t3535;
  t3553 = 0.97024*t1870;
  t3554 = 0. + t3550 + t3553;
  t3565 = -1.45024*t3561;
  t3567 = -0.066675*t2293;
  t3568 = 0. + t3565 + t3567;
  t3570 = -0.066675*t3561;
  t3574 = 1.45024*t2293;
  t3576 = 0. + t3570 + t3574;
  t3583 = -1.93024*t3582;
  t3584 = -0.065597*t2836;
  t3589 = 0. + t3583 + t3584;
  t3600 = -0.065597*t3582;
  t3601 = 1.93024*t2836;
  t3605 = 0. + t3600 + t3601;
  t3471 = t3263 + t3290;
  t3301 = t352*t3281;
  t3317 = -1.*t3250*t2836;
  t3319 = t3301 + t3317;
  t3424 = 0. + t1453;
  t3484 = t3375 + t3394;
  t3396 = t352*t3389;
  t3398 = -1.*t3360*t2836;
  t3400 = t3396 + t3398;

  p_output1(0)=t2650 + t2876 + 0.000796*t3065;
  p_output1(1)=t3263 + t3290 + 0.000796*t3319;
  p_output1(2)=t3375 + t3394 + 0.000796*t3400;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1316*t3332;
  p_output1(5)=t1817*t3332;
  p_output1(6)=t3424;
  p_output1(7)=0.;
  p_output1(8)=t2528*t2836 + 0.000796*t3438 - 1.*t2832*t352;
  p_output1(9)=t2836*t3250 + 0.000796*t3471 - 1.*t3281*t352;
  p_output1(10)=t2836*t3360 + 0.000796*t3484 - 1.*t3389*t352;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.261012*t1316 - 0.000525*t3065 - 0.341012*t1316*t3332 - 1.840292*t3438 - 0.066675*t3498 - 1.*t1316*t3531 + t1316*t1453*t3545 + t1817*t3554 + t2030*t3568 + t2108*t3576 + t2528*t3589 + t2832*t3605;
  p_output1(13)=0. + 0.066675*t1316 - 0.000525*t3319 + 0.341012*t1817*t3332 - 1.840292*t3471 + 0.261012*t3498 + t1817*t3531 - 1.*t1453*t1817*t3545 + t1316*t3554 + t3195*t3568 + t3247*t3576 + t3250*t3589 + t3281*t3605;
  p_output1(14)=0. - 0.331012*t1453 - 0.000525*t3400 + 0.341012*t3424 - 1.840292*t3484 - 0.90524*t3527 + t3336*t3545 + t3338*t3568 + t3348*t3576 + t3360*t3589 + t3389*t3605;
  p_output1(15)=1.;
}


       
void H_LeftFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
