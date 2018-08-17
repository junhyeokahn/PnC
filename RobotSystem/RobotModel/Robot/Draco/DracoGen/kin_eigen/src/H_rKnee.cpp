/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:51 GMT-05:00
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
  double t256;
  double t261;
  double t265;
  double t262;
  double t269;
  double t258;
  double t260;
  double t250;
  double t264;
  double t271;
  double t272;
  double t756;
  double t259;
  double t322;
  double t332;
  double t246;
  double t760;
  double t772;
  double t881;
  double t992;
  double t1005;
  double t1262;
  double t1269;
  double t1315;
  double t1495;
  double t1499;
  double t1520;
  double t226;
  double t1895;
  double t1972;
  double t1987;
  double t1732;
  double t1827;
  double t2114;
  double t2150;
  double t2197;
  double t2228;
  double t2323;
  double t2381;
  double t2508;
  double t2590;
  double t2616;
  double t2655;
  double t2689;
  double t2838;
  double t2853;
  double t2865;
  double t2938;
  double t2958;
  double t3056;
  double t3195;
  double t3199;
  double t3207;
  double t755;
  double t1684;
  double t1703;
  double t1749;
  double t1771;
  double t1773;
  double t2151;
  double t2697;
  double t2701;
  double t2712;
  double t2713;
  double t2740;
  double t2872;
  double t3267;
  double t3277;
  double t3319;
  double t3347;
  double t3367;
  double t4111;
  double t4119;
  double t4273;
  double t4274;
  double t3473;
  double t3485;
  double t3500;
  double t4886;
  double t4895;
  double t5249;
  double t5274;
  double t1707;
  double t1775;
  double t1809;
  double t3598;
  double t3601;
  double t3617;
  double t4122;
  double t4151;
  double t4152;
  double t4299;
  double t4336;
  double t4338;
  double t4504;
  double t4582;
  double t4644;
  double t4684;
  double t4685;
  double t4800;
  double t4906;
  double t5067;
  double t5145;
  double t3501;
  double t3522;
  double t3530;
  double t5205;
  double t5223;
  double t5225;
  double t5298;
  double t5335;
  double t5343;
  double t5492;
  double t5499;
  double t5505;
  double t2706;
  double t2746;
  double t2801;
  double t3628;
  double t3648;
  double t3757;
  double t3547;
  double t3571;
  double t3572;
  double t3305;
  double t3375;
  double t3398;
  double t3836;
  double t3877;
  double t4057;
  t256 = Cos(var1[3]);
  t261 = Cos(var1[5]);
  t265 = Sin(var1[4]);
  t262 = Sin(var1[3]);
  t269 = Sin(var1[5]);
  t258 = Cos(var1[4]);
  t260 = Sin(var1[11]);
  t250 = Cos(var1[11]);
  t264 = -1.*t261*t262;
  t271 = t256*t265*t269;
  t272 = t264 + t271;
  t756 = Cos(var1[13]);
  t259 = t250*t256*t258;
  t322 = t260*t272;
  t332 = t259 + t322;
  t246 = Sin(var1[13]);
  t760 = Cos(var1[12]);
  t772 = t256*t261*t265;
  t881 = t262*t269;
  t992 = t772 + t881;
  t1005 = t760*t992;
  t1262 = Sin(var1[12]);
  t1269 = -1.*t256*t258*t260;
  t1315 = t250*t272;
  t1495 = t1269 + t1315;
  t1499 = -1.*t1262*t1495;
  t1520 = t1005 + t1499;
  t226 = Sin(var1[14]);
  t1895 = t256*t261;
  t1972 = t262*t265*t269;
  t1987 = t1895 + t1972;
  t1732 = Cos(var1[14]);
  t1827 = t250*t258*t262;
  t2114 = t260*t1987;
  t2150 = t1827 + t2114;
  t2197 = t261*t262*t265;
  t2228 = -1.*t256*t269;
  t2323 = t2197 + t2228;
  t2381 = t760*t2323;
  t2508 = -1.*t258*t260*t262;
  t2590 = t250*t1987;
  t2616 = t2508 + t2590;
  t2655 = -1.*t1262*t2616;
  t2689 = t2381 + t2655;
  t2838 = -1.*t250*t265;
  t2853 = t258*t260*t269;
  t2865 = t2838 + t2853;
  t2938 = t760*t258*t261;
  t2958 = t260*t265;
  t3056 = t250*t258*t269;
  t3195 = t2958 + t3056;
  t3199 = -1.*t1262*t3195;
  t3207 = t2938 + t3199;
  t755 = t246*t332;
  t1684 = t756*t1520;
  t1703 = t755 + t1684;
  t1749 = t756*t332;
  t1771 = -1.*t246*t1520;
  t1773 = t1749 + t1771;
  t2151 = t246*t2150;
  t2697 = t756*t2689;
  t2701 = t2151 + t2697;
  t2712 = t756*t2150;
  t2713 = -1.*t246*t2689;
  t2740 = t2712 + t2713;
  t2872 = t246*t2865;
  t3267 = t756*t3207;
  t3277 = t2872 + t3267;
  t3319 = t756*t2865;
  t3347 = -1.*t246*t3207;
  t3367 = t3319 + t3347;
  t4111 = -1.*t250;
  t4119 = 1. + t4111;
  t4273 = -1.*t760;
  t4274 = 1. + t4273;
  t3473 = t1262*t992;
  t3485 = t760*t1495;
  t3500 = t3473 + t3485;
  t4886 = -1.*t756;
  t4895 = 1. + t4886;
  t5249 = -1.*t1732;
  t5274 = 1. + t5249;
  t1707 = -1.*t226*t1703;
  t1775 = t1732*t1773;
  t1809 = t1707 + t1775;
  t3598 = t1732*t1703;
  t3601 = t226*t1773;
  t3617 = t3598 + t3601;
  t4122 = -0.022225*t4119;
  t4151 = -0.086996*t260;
  t4152 = 0. + t4122 + t4151;
  t4299 = -0.31508*t4274;
  t4336 = 0.156996*t1262;
  t4338 = 0. + t4299 + t4336;
  t4504 = -0.086996*t4119;
  t4582 = 0.022225*t260;
  t4644 = 0. + t4504 + t4582;
  t4684 = -0.156996*t4274;
  t4685 = -0.31508*t1262;
  t4800 = 0. + t4684 + t4685;
  t4906 = -0.022225*t4895;
  t5067 = 0.38008*t246;
  t5145 = 0. + t4906 + t5067;
  t3501 = t1262*t2323;
  t3522 = t760*t2616;
  t3530 = t3501 + t3522;
  t5205 = -0.38008*t4895;
  t5223 = -0.022225*t246;
  t5225 = 0. + t5205 + t5223;
  t5298 = -0.86008*t5274;
  t5335 = -0.022225*t226;
  t5343 = 0. + t5298 + t5335;
  t5492 = -0.022225*t5274;
  t5499 = 0.86008*t226;
  t5505 = 0. + t5492 + t5499;
  t2706 = -1.*t226*t2701;
  t2746 = t1732*t2740;
  t2801 = t2706 + t2746;
  t3628 = t1732*t2701;
  t3648 = t226*t2740;
  t3757 = t3628 + t3648;
  t3547 = t258*t261*t1262;
  t3571 = t760*t3195;
  t3572 = t3547 + t3571;
  t3305 = -1.*t226*t3277;
  t3375 = t1732*t3367;
  t3398 = t3305 + t3375;
  t3836 = t1732*t3277;
  t3877 = t226*t3367;
  t4057 = t3836 + t3877;

  p_output1(0)=t1809;
  p_output1(1)=t2801;
  p_output1(2)=t3398;
  p_output1(3)=0.;
  p_output1(4)=t3500;
  p_output1(5)=t3530;
  p_output1(6)=t3572;
  p_output1(7)=0.;
  p_output1(8)=t3617;
  p_output1(9)=t3757;
  p_output1(10)=t4057;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t1809 - 0.150246*t3500 - 0.86008*t3617 + t256*t258*t4152 + t272*t4644 + t1495*t4800 + t332*t5145 + t1520*t5225 + t1703*t5343 + t1773*t5505 + t4338*t992 + var1(0);
  p_output1(13)=0. - 0.022225*t2801 - 0.150246*t3530 - 0.86008*t3757 + t258*t262*t4152 + t2323*t4338 + t1987*t4644 + t2616*t4800 + t2150*t5145 + t2689*t5225 + t2701*t5343 + t2740*t5505 + var1(1);
  p_output1(14)=0. - 0.022225*t3398 - 0.150246*t3572 - 0.86008*t4057 - 1.*t265*t4152 + t258*t261*t4338 + t258*t269*t4644 + t3195*t4800 + t2865*t5145 + t3207*t5225 + t3277*t5343 + t3367*t5505 + var1(2);
  p_output1(15)=1.;
}


       
void H_rKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
