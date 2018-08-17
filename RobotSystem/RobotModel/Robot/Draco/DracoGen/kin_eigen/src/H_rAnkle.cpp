/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:03 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rAnkle.h"

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
  double t493;
  double t675;
  double t873;
  double t687;
  double t874;
  double t590;
  double t624;
  double t381;
  double t800;
  double t902;
  double t934;
  double t1202;
  double t602;
  double t936;
  double t948;
  double t352;
  double t1333;
  double t1476;
  double t1480;
  double t1506;
  double t1511;
  double t1524;
  double t1536;
  double t1552;
  double t1563;
  double t1661;
  double t1693;
  double t1736;
  double t1165;
  double t1694;
  double t1696;
  double t334;
  double t1751;
  double t1754;
  double t1756;
  double t1870;
  double t1712;
  double t1769;
  double t1823;
  double t312;
  double t1872;
  double t1888;
  double t1895;
  double t2243;
  double t2298;
  double t2335;
  double t2174;
  double t2392;
  double t2419;
  double t2455;
  double t2478;
  double t2497;
  double t2533;
  double t2559;
  double t2572;
  double t2624;
  double t2653;
  double t2666;
  double t2435;
  double t2707;
  double t2724;
  double t2799;
  double t2802;
  double t2811;
  double t2747;
  double t2887;
  double t2952;
  double t2993;
  double t3010;
  double t3025;
  double t3080;
  double t3098;
  double t3140;
  double t3163;
  double t3182;
  double t3203;
  double t3214;
  double t3242;
  double t3264;
  double t3144;
  double t3320;
  double t3346;
  double t3422;
  double t3441;
  double t3502;
  double t3385;
  double t3528;
  double t3551;
  double t3613;
  double t3618;
  double t3627;
  double t1846;
  double t1906;
  double t2983;
  double t3038;
  double t3590;
  double t3635;
  double t4293;
  double t4306;
  double t4559;
  double t4602;
  double t3782;
  double t3819;
  double t3854;
  double t4849;
  double t4928;
  double t5111;
  double t5115;
  double t5269;
  double t5294;
  double t4016;
  double t1939;
  double t2005;
  double t2081;
  double t4376;
  double t4495;
  double t4519;
  double t4623;
  double t4626;
  double t4648;
  double t4684;
  double t4735;
  double t4740;
  double t4809;
  double t4817;
  double t4836;
  double t4934;
  double t4941;
  double t4964;
  double t3855;
  double t3862;
  double t3865;
  double t5063;
  double t5073;
  double t5075;
  double t5132;
  double t5147;
  double t5160;
  double t5191;
  double t5225;
  double t5249;
  double t5311;
  double t5339;
  double t5344;
  double t5369;
  double t5422;
  double t5444;
  double t4086;
  double t3042;
  double t3045;
  double t3049;
  double t3909;
  double t3910;
  double t3920;
  double t4180;
  double t3663;
  double t3718;
  double t3726;
  t493 = Cos(var1[3]);
  t675 = Cos(var1[5]);
  t873 = Sin(var1[4]);
  t687 = Sin(var1[3]);
  t874 = Sin(var1[5]);
  t590 = Cos(var1[4]);
  t624 = Sin(var1[11]);
  t381 = Cos(var1[11]);
  t800 = -1.*t675*t687;
  t902 = t493*t873*t874;
  t934 = t800 + t902;
  t1202 = Cos(var1[13]);
  t602 = t381*t493*t590;
  t936 = t624*t934;
  t948 = t602 + t936;
  t352 = Sin(var1[13]);
  t1333 = Cos(var1[12]);
  t1476 = t493*t675*t873;
  t1480 = t687*t874;
  t1506 = t1476 + t1480;
  t1511 = t1333*t1506;
  t1524 = Sin(var1[12]);
  t1536 = -1.*t493*t590*t624;
  t1552 = t381*t934;
  t1563 = t1536 + t1552;
  t1661 = -1.*t1524*t1563;
  t1693 = t1511 + t1661;
  t1736 = Cos(var1[14]);
  t1165 = t352*t948;
  t1694 = t1202*t1693;
  t1696 = t1165 + t1694;
  t334 = Sin(var1[14]);
  t1751 = t1202*t948;
  t1754 = -1.*t352*t1693;
  t1756 = t1751 + t1754;
  t1870 = Cos(var1[15]);
  t1712 = -1.*t334*t1696;
  t1769 = t1736*t1756;
  t1823 = t1712 + t1769;
  t312 = Sin(var1[15]);
  t1872 = t1736*t1696;
  t1888 = t334*t1756;
  t1895 = t1872 + t1888;
  t2243 = t493*t675;
  t2298 = t687*t873*t874;
  t2335 = t2243 + t2298;
  t2174 = t381*t590*t687;
  t2392 = t624*t2335;
  t2419 = t2174 + t2392;
  t2455 = t675*t687*t873;
  t2478 = -1.*t493*t874;
  t2497 = t2455 + t2478;
  t2533 = t1333*t2497;
  t2559 = -1.*t590*t624*t687;
  t2572 = t381*t2335;
  t2624 = t2559 + t2572;
  t2653 = -1.*t1524*t2624;
  t2666 = t2533 + t2653;
  t2435 = t352*t2419;
  t2707 = t1202*t2666;
  t2724 = t2435 + t2707;
  t2799 = t1202*t2419;
  t2802 = -1.*t352*t2666;
  t2811 = t2799 + t2802;
  t2747 = -1.*t334*t2724;
  t2887 = t1736*t2811;
  t2952 = t2747 + t2887;
  t2993 = t1736*t2724;
  t3010 = t334*t2811;
  t3025 = t2993 + t3010;
  t3080 = -1.*t381*t873;
  t3098 = t590*t624*t874;
  t3140 = t3080 + t3098;
  t3163 = t1333*t590*t675;
  t3182 = t624*t873;
  t3203 = t381*t590*t874;
  t3214 = t3182 + t3203;
  t3242 = -1.*t1524*t3214;
  t3264 = t3163 + t3242;
  t3144 = t352*t3140;
  t3320 = t1202*t3264;
  t3346 = t3144 + t3320;
  t3422 = t1202*t3140;
  t3441 = -1.*t352*t3264;
  t3502 = t3422 + t3441;
  t3385 = -1.*t334*t3346;
  t3528 = t1736*t3502;
  t3551 = t3385 + t3528;
  t3613 = t1736*t3346;
  t3618 = t334*t3502;
  t3627 = t3613 + t3618;
  t1846 = t312*t1823;
  t1906 = t1870*t1895;
  t2983 = t312*t2952;
  t3038 = t1870*t3025;
  t3590 = t312*t3551;
  t3635 = t1870*t3627;
  t4293 = -1.*t381;
  t4306 = 1. + t4293;
  t4559 = -1.*t1333;
  t4602 = 1. + t4559;
  t3782 = t1524*t1506;
  t3819 = t1333*t1563;
  t3854 = t3782 + t3819;
  t4849 = -1.*t1202;
  t4928 = 1. + t4849;
  t5111 = -1.*t1736;
  t5115 = 1. + t5111;
  t5269 = -1.*t1870;
  t5294 = 1. + t5269;
  t4016 = t1846 + t1906;
  t1939 = t1870*t1823;
  t2005 = -1.*t312*t1895;
  t2081 = t1939 + t2005;
  t4376 = -0.022225*t4306;
  t4495 = -0.086996*t624;
  t4519 = 0. + t4376 + t4495;
  t4623 = -0.31508*t4602;
  t4626 = 0.156996*t1524;
  t4648 = 0. + t4623 + t4626;
  t4684 = -0.086996*t4306;
  t4735 = 0.022225*t624;
  t4740 = 0. + t4684 + t4735;
  t4809 = -0.156996*t4602;
  t4817 = -0.31508*t1524;
  t4836 = 0. + t4809 + t4817;
  t4934 = -0.022225*t4928;
  t4941 = 0.38008*t352;
  t4964 = 0. + t4934 + t4941;
  t3855 = t1524*t2497;
  t3862 = t1333*t2624;
  t3865 = t3855 + t3862;
  t5063 = -0.38008*t4928;
  t5073 = -0.022225*t352;
  t5075 = 0. + t5063 + t5073;
  t5132 = -0.86008*t5115;
  t5147 = -0.022225*t334;
  t5160 = 0. + t5132 + t5147;
  t5191 = -0.022225*t5115;
  t5225 = 0.86008*t334;
  t5249 = 0. + t5191 + t5225;
  t5311 = -0.021147*t5294;
  t5339 = 1.34008*t312;
  t5344 = 0. + t5311 + t5339;
  t5369 = -1.34008*t5294;
  t5422 = -0.021147*t312;
  t5444 = 0. + t5369 + t5422;
  t4086 = t2983 + t3038;
  t3042 = t1870*t2952;
  t3045 = -1.*t312*t3025;
  t3049 = t3042 + t3045;
  t3909 = t590*t675*t1524;
  t3910 = t1333*t3214;
  t3920 = t3909 + t3910;
  t4180 = t3590 + t3635;
  t3663 = t1870*t3551;
  t3718 = -1.*t312*t3627;
  t3726 = t3663 + t3718;

  p_output1(0)=t1846 + t1906 + 0.000796*t2081;
  p_output1(1)=t2983 + t3038 + 0.000796*t3049;
  p_output1(2)=t3590 + t3635 + 0.000796*t3726;
  p_output1(3)=0.;
  p_output1(4)=t3854;
  p_output1(5)=t3865;
  p_output1(6)=t3920;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1823*t1870 + t1895*t312 + 0.000796*t4016;
  p_output1(9)=-1.*t1870*t2952 + t3025*t312 + 0.000796*t4086;
  p_output1(10)=-1.*t1870*t3551 + t312*t3627 + 0.000796*t4180;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.021147*t2081 - 0.166996*t3854 - 1.34008*t4016 + t1506*t4648 + t1563*t4836 + t1693*t5075 + t1696*t5160 + t1756*t5249 + t1823*t5344 + t1895*t5444 + t4519*t493*t590 + t4740*t934 + t4964*t948 + var1(0);
  p_output1(13)=0. - 0.021147*t3049 - 0.166996*t3865 - 1.34008*t4086 + t2497*t4648 + t2335*t4740 + t2624*t4836 + t2419*t4964 + t2666*t5075 + t2724*t5160 + t2811*t5249 + t2952*t5344 + t3025*t5444 + t4519*t590*t687 + var1(1);
  p_output1(14)=0. - 0.021147*t3726 - 0.166996*t3920 - 1.34008*t4180 + t3214*t4836 + t3140*t4964 + t3264*t5075 + t3346*t5160 + t3502*t5249 + t3551*t5344 + t3627*t5444 + t4648*t590*t675 - 1.*t4519*t873 + t4740*t590*t874 + var1(2);
  p_output1(15)=1.;
}


       
void H_rAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
