/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rKnee.h"

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
  double t591;
  double t928;
  double t996;
  double t937;
  double t1056;
  double t600;
  double t754;
  double t558;
  double t967;
  double t1141;
  double t1187;
  double t1560;
  double t601;
  double t1269;
  double t1538;
  double t376;
  double t1579;
  double t1583;
  double t1588;
  double t1601;
  double t1604;
  double t1609;
  double t1670;
  double t1683;
  double t1703;
  double t1750;
  double t1780;
  double t130;
  double t2078;
  double t2180;
  double t2190;
  double t1850;
  double t2067;
  double t2252;
  double t2266;
  double t2306;
  double t2316;
  double t2342;
  double t2390;
  double t2394;
  double t2396;
  double t2414;
  double t2495;
  double t2509;
  double t2718;
  double t2743;
  double t2745;
  double t2798;
  double t2814;
  double t2817;
  double t2848;
  double t2855;
  double t2868;
  double t1543;
  double t1810;
  double t1833;
  double t1875;
  double t1902;
  double t1995;
  double t2287;
  double t2522;
  double t2544;
  double t2552;
  double t2592;
  double t2625;
  double t2751;
  double t2889;
  double t2890;
  double t2895;
  double t2912;
  double t2917;
  double t3403;
  double t3410;
  double t3447;
  double t3525;
  double t2948;
  double t2954;
  double t2964;
  double t3892;
  double t4023;
  double t4208;
  double t4227;
  double t1849;
  double t2026;
  double t2048;
  double t3196;
  double t3206;
  double t3236;
  double t3419;
  double t3424;
  double t3430;
  double t3605;
  double t3653;
  double t3672;
  double t3725;
  double t3742;
  double t3797;
  double t3825;
  double t3830;
  double t3860;
  double t4062;
  double t4068;
  double t4073;
  double t2970;
  double t2977;
  double t3053;
  double t4152;
  double t4156;
  double t4163;
  double t4269;
  double t4314;
  double t4317;
  double t4364;
  double t4401;
  double t4460;
  double t2549;
  double t2690;
  double t2712;
  double t3245;
  double t3247;
  double t3261;
  double t3064;
  double t3086;
  double t3178;
  double t2893;
  double t2925;
  double t2943;
  double t3274;
  double t3378;
  double t3386;
  t591 = Cos(var1[3]);
  t928 = Cos(var1[5]);
  t996 = Sin(var1[4]);
  t937 = Sin(var1[3]);
  t1056 = Sin(var1[5]);
  t600 = Cos(var1[4]);
  t754 = Sin(var1[11]);
  t558 = Cos(var1[11]);
  t967 = -1.*t928*t937;
  t1141 = t591*t996*t1056;
  t1187 = t967 + t1141;
  t1560 = Cos(var1[13]);
  t601 = t558*t591*t600;
  t1269 = t754*t1187;
  t1538 = t601 + t1269;
  t376 = Sin(var1[13]);
  t1579 = Cos(var1[12]);
  t1583 = t591*t928*t996;
  t1588 = t937*t1056;
  t1601 = t1583 + t1588;
  t1604 = t1579*t1601;
  t1609 = Sin(var1[12]);
  t1670 = -1.*t591*t600*t754;
  t1683 = t558*t1187;
  t1703 = t1670 + t1683;
  t1750 = -1.*t1609*t1703;
  t1780 = t1604 + t1750;
  t130 = Sin(var1[14]);
  t2078 = t591*t928;
  t2180 = t937*t996*t1056;
  t2190 = t2078 + t2180;
  t1850 = Cos(var1[14]);
  t2067 = t558*t600*t937;
  t2252 = t754*t2190;
  t2266 = t2067 + t2252;
  t2306 = t928*t937*t996;
  t2316 = -1.*t591*t1056;
  t2342 = t2306 + t2316;
  t2390 = t1579*t2342;
  t2394 = -1.*t600*t754*t937;
  t2396 = t558*t2190;
  t2414 = t2394 + t2396;
  t2495 = -1.*t1609*t2414;
  t2509 = t2390 + t2495;
  t2718 = -1.*t558*t996;
  t2743 = t600*t754*t1056;
  t2745 = t2718 + t2743;
  t2798 = t1579*t600*t928;
  t2814 = t754*t996;
  t2817 = t558*t600*t1056;
  t2848 = t2814 + t2817;
  t2855 = -1.*t1609*t2848;
  t2868 = t2798 + t2855;
  t1543 = t376*t1538;
  t1810 = t1560*t1780;
  t1833 = t1543 + t1810;
  t1875 = t1560*t1538;
  t1902 = -1.*t376*t1780;
  t1995 = t1875 + t1902;
  t2287 = t376*t2266;
  t2522 = t1560*t2509;
  t2544 = t2287 + t2522;
  t2552 = t1560*t2266;
  t2592 = -1.*t376*t2509;
  t2625 = t2552 + t2592;
  t2751 = t376*t2745;
  t2889 = t1560*t2868;
  t2890 = t2751 + t2889;
  t2895 = t1560*t2745;
  t2912 = -1.*t376*t2868;
  t2917 = t2895 + t2912;
  t3403 = -1.*t558;
  t3410 = 1. + t3403;
  t3447 = -1.*t1579;
  t3525 = 1. + t3447;
  t2948 = t1609*t1601;
  t2954 = t1579*t1703;
  t2964 = t2948 + t2954;
  t3892 = -1.*t1560;
  t4023 = 1. + t3892;
  t4208 = -1.*t1850;
  t4227 = 1. + t4208;
  t1849 = -1.*t130*t1833;
  t2026 = t1850*t1995;
  t2048 = t1849 + t2026;
  t3196 = t1850*t1833;
  t3206 = t130*t1995;
  t3236 = t3196 + t3206;
  t3419 = -0.022225*t3410;
  t3424 = -0.086996*t754;
  t3430 = 0. + t3419 + t3424;
  t3605 = -0.31508*t3525;
  t3653 = 0.156996*t1609;
  t3672 = 0. + t3605 + t3653;
  t3725 = -0.086996*t3410;
  t3742 = 0.022225*t754;
  t3797 = 0. + t3725 + t3742;
  t3825 = -0.156996*t3525;
  t3830 = -0.31508*t1609;
  t3860 = 0. + t3825 + t3830;
  t4062 = -0.022225*t4023;
  t4068 = 0.38008*t376;
  t4073 = 0. + t4062 + t4068;
  t2970 = t1609*t2342;
  t2977 = t1579*t2414;
  t3053 = t2970 + t2977;
  t4152 = -0.38008*t4023;
  t4156 = -0.022225*t376;
  t4163 = 0. + t4152 + t4156;
  t4269 = -0.86008*t4227;
  t4314 = -0.022225*t130;
  t4317 = 0. + t4269 + t4314;
  t4364 = -0.022225*t4227;
  t4401 = 0.86008*t130;
  t4460 = 0. + t4364 + t4401;
  t2549 = -1.*t130*t2544;
  t2690 = t1850*t2625;
  t2712 = t2549 + t2690;
  t3245 = t1850*t2544;
  t3247 = t130*t2625;
  t3261 = t3245 + t3247;
  t3064 = t600*t928*t1609;
  t3086 = t1579*t2848;
  t3178 = t3064 + t3086;
  t2893 = -1.*t130*t2890;
  t2925 = t1850*t2917;
  t2943 = t2893 + t2925;
  t3274 = t1850*t2890;
  t3378 = t130*t2917;
  t3386 = t3274 + t3378;

  p_output1(0)=t2048;
  p_output1(1)=t2712;
  p_output1(2)=t2943;
  p_output1(3)=0.;
  p_output1(4)=t2964;
  p_output1(5)=t3053;
  p_output1(6)=t3178;
  p_output1(7)=0.;
  p_output1(8)=t3236;
  p_output1(9)=t3261;
  p_output1(10)=t3386;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2048 - 0.150246*t2964 - 0.86008*t3236 + t1601*t3672 + t1187*t3797 + t1703*t3860 + t1538*t4073 + t1780*t4163 + t1833*t4317 + t1995*t4460 + t3430*t591*t600 + var1(0);
  p_output1(13)=0. - 0.022225*t2712 - 0.150246*t3053 - 0.86008*t3261 + t2342*t3672 + t2190*t3797 + t2414*t3860 + t2266*t4073 + t2509*t4163 + t2544*t4317 + t2625*t4460 + t3430*t600*t937 + var1(1);
  p_output1(14)=0. - 0.022225*t2943 - 0.150246*t3178 - 0.86008*t3386 + t2848*t3860 + t2745*t4073 + t2868*t4163 + t2890*t4317 + t2917*t4460 + t1056*t3797*t600 + t3672*t600*t928 - 1.*t3430*t996 + var1(2);
  p_output1(15)=1.;
}


       
void H_rKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
