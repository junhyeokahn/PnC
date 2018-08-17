/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:04 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rAnkle.h"

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
  double t1041;
  double t1532;
  double t1646;
  double t1552;
  double t1660;
  double t1213;
  double t1511;
  double t850;
  double t1577;
  double t1694;
  double t1749;
  double t1846;
  double t1506;
  double t1751;
  double t1756;
  double t793;
  double t1872;
  double t1876;
  double t1888;
  double t1906;
  double t1932;
  double t2081;
  double t2123;
  double t2140;
  double t2155;
  double t2174;
  double t2198;
  double t2435;
  double t1769;
  double t2243;
  double t2335;
  double t669;
  double t2497;
  double t2533;
  double t2608;
  double t2811;
  double t2392;
  double t2643;
  double t2707;
  double t368;
  double t2887;
  double t2924;
  double t2983;
  double t3140;
  double t3144;
  double t3182;
  double t3077;
  double t3374;
  double t3496;
  double t3618;
  double t3635;
  double t3726;
  double t3727;
  double t3750;
  double t3770;
  double t3819;
  double t3854;
  double t3855;
  double t3590;
  double t3862;
  double t3865;
  double t3909;
  double t3920;
  double t3923;
  double t3908;
  double t4003;
  double t4016;
  double t4025;
  double t4036;
  double t4059;
  double t4495;
  double t4623;
  double t4626;
  double t4666;
  double t4684;
  double t4735;
  double t4740;
  double t4761;
  double t4767;
  double t4648;
  double t4782;
  double t4809;
  double t4836;
  double t4843;
  double t4934;
  double t4817;
  double t4936;
  double t4941;
  double t4964;
  double t4982;
  double t5036;
  double t2799;
  double t2993;
  double t4017;
  double t4091;
  double t4944;
  double t5063;
  t1041 = Cos(var1[3]);
  t1532 = Cos(var1[5]);
  t1646 = Sin(var1[4]);
  t1552 = Sin(var1[3]);
  t1660 = Sin(var1[5]);
  t1213 = Cos(var1[4]);
  t1511 = Sin(var1[11]);
  t850 = Cos(var1[11]);
  t1577 = -1.*t1532*t1552;
  t1694 = t1041*t1646*t1660;
  t1749 = t1577 + t1694;
  t1846 = Cos(var1[13]);
  t1506 = t850*t1041*t1213;
  t1751 = t1511*t1749;
  t1756 = t1506 + t1751;
  t793 = Sin(var1[13]);
  t1872 = Cos(var1[12]);
  t1876 = t1041*t1532*t1646;
  t1888 = t1552*t1660;
  t1906 = t1876 + t1888;
  t1932 = t1872*t1906;
  t2081 = Sin(var1[12]);
  t2123 = -1.*t1041*t1213*t1511;
  t2140 = t850*t1749;
  t2155 = t2123 + t2140;
  t2174 = -1.*t2081*t2155;
  t2198 = t1932 + t2174;
  t2435 = Cos(var1[14]);
  t1769 = t793*t1756;
  t2243 = t1846*t2198;
  t2335 = t1769 + t2243;
  t669 = Sin(var1[14]);
  t2497 = t1846*t1756;
  t2533 = -1.*t793*t2198;
  t2608 = t2497 + t2533;
  t2811 = Cos(var1[15]);
  t2392 = -1.*t669*t2335;
  t2643 = t2435*t2608;
  t2707 = t2392 + t2643;
  t368 = Sin(var1[15]);
  t2887 = t2435*t2335;
  t2924 = t669*t2608;
  t2983 = t2887 + t2924;
  t3140 = t1041*t1532;
  t3144 = t1552*t1646*t1660;
  t3182 = t3140 + t3144;
  t3077 = t850*t1213*t1552;
  t3374 = t1511*t3182;
  t3496 = t3077 + t3374;
  t3618 = t1532*t1552*t1646;
  t3635 = -1.*t1041*t1660;
  t3726 = t3618 + t3635;
  t3727 = t1872*t3726;
  t3750 = -1.*t1213*t1511*t1552;
  t3770 = t850*t3182;
  t3819 = t3750 + t3770;
  t3854 = -1.*t2081*t3819;
  t3855 = t3727 + t3854;
  t3590 = t793*t3496;
  t3862 = t1846*t3855;
  t3865 = t3590 + t3862;
  t3909 = t1846*t3496;
  t3920 = -1.*t793*t3855;
  t3923 = t3909 + t3920;
  t3908 = -1.*t669*t3865;
  t4003 = t2435*t3923;
  t4016 = t3908 + t4003;
  t4025 = t2435*t3865;
  t4036 = t669*t3923;
  t4059 = t4025 + t4036;
  t4495 = -1.*t850*t1646;
  t4623 = t1213*t1511*t1660;
  t4626 = t4495 + t4623;
  t4666 = t1872*t1213*t1532;
  t4684 = t1511*t1646;
  t4735 = t850*t1213*t1660;
  t4740 = t4684 + t4735;
  t4761 = -1.*t2081*t4740;
  t4767 = t4666 + t4761;
  t4648 = t793*t4626;
  t4782 = t1846*t4767;
  t4809 = t4648 + t4782;
  t4836 = t1846*t4626;
  t4843 = -1.*t793*t4767;
  t4934 = t4836 + t4843;
  t4817 = -1.*t669*t4809;
  t4936 = t2435*t4934;
  t4941 = t4817 + t4936;
  t4964 = t2435*t4809;
  t4982 = t669*t4934;
  t5036 = t4964 + t4982;
  t2799 = t368*t2707;
  t2993 = t2811*t2983;
  t4017 = t368*t4016;
  t4091 = t2811*t4059;
  t4944 = t368*t4941;
  t5063 = t2811*t5036;

  p_output1(0)=t2799 + t2993 + 0.000796*(t2707*t2811 - 1.*t2983*t368);
  p_output1(1)=t4017 + 0.000796*(t2811*t4016 - 1.*t368*t4059) + t4091;
  p_output1(2)=t4944 + 0.000796*(t2811*t4941 - 1.*t368*t5036) + t5063;
  p_output1(3)=t1906*t2081 + t1872*t2155;
  p_output1(4)=t2081*t3726 + t1872*t3819;
  p_output1(5)=t1213*t1532*t2081 + t1872*t4740;
  p_output1(6)=-1.*t2707*t2811 + 0.000796*(t2799 + t2993) + t2983*t368;
  p_output1(7)=-1.*t2811*t4016 + t368*t4059 + 0.000796*(t4017 + t4091);
  p_output1(8)=-1.*t2811*t4941 + t368*t5036 + 0.000796*(t4944 + t5063);
}


       
void R_rAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
