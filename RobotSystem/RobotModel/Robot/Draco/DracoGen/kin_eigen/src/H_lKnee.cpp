/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:48 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lKnee.h"

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
  double t733;
  double t803;
  double t622;
  double t759;
  double t837;
  double t1349;
  double t988;
  double t1015;
  double t1029;
  double t1130;
  double t1362;
  double t338;
  double t1635;
  double t1659;
  double t1669;
  double t372;
  double t794;
  double t854;
  double t881;
  double t959;
  double t1273;
  double t1402;
  double t1429;
  double t1463;
  double t1481;
  double t1548;
  double t1675;
  double t27;
  double t2299;
  double t2322;
  double t2336;
  double t1919;
  double t2402;
  double t2437;
  double t2463;
  double t2135;
  double t2142;
  double t2164;
  double t2170;
  double t2355;
  double t2359;
  double t2361;
  double t2365;
  double t2379;
  double t3125;
  double t3130;
  double t3152;
  double t2915;
  double t2955;
  double t2974;
  double t3052;
  double t3056;
  double t3058;
  double t1634;
  double t1796;
  double t1916;
  double t1942;
  double t1962;
  double t1965;
  double t2389;
  double t2499;
  double t2526;
  double t2749;
  double t2840;
  double t2867;
  double t3114;
  double t3215;
  double t3250;
  double t3309;
  double t3310;
  double t3312;
  double t3804;
  double t3818;
  double t3948;
  double t3963;
  double t3335;
  double t3347;
  double t3414;
  double t4081;
  double t4135;
  double t4243;
  double t4244;
  double t1918;
  double t1996;
  double t2096;
  double t3554;
  double t3560;
  double t3572;
  double t3836;
  double t3857;
  double t3864;
  double t3906;
  double t3922;
  double t3933;
  double t3970;
  double t3989;
  double t3997;
  double t4033;
  double t4035;
  double t4038;
  double t3445;
  double t3450;
  double t3471;
  double t4147;
  double t4196;
  double t4215;
  double t4229;
  double t4233;
  double t4238;
  double t4245;
  double t4251;
  double t4272;
  double t4284;
  double t4313;
  double t4315;
  double t2637;
  double t2870;
  double t2884;
  double t3595;
  double t3611;
  double t3761;
  double t3486;
  double t3505;
  double t3544;
  double t3304;
  double t3332;
  double t3334;
  double t3762;
  double t3776;
  double t3779;
  t733 = Cos(var1[5]);
  t803 = Sin(var1[3]);
  t622 = Cos(var1[3]);
  t759 = Sin(var1[4]);
  t837 = Sin(var1[5]);
  t1349 = Cos(var1[4]);
  t988 = Cos(var1[6]);
  t1015 = -1.*t733*t803;
  t1029 = t622*t759*t837;
  t1130 = t1015 + t1029;
  t1362 = Sin(var1[6]);
  t338 = Cos(var1[8]);
  t1635 = t622*t1349*t988;
  t1659 = t1130*t1362;
  t1669 = t1635 + t1659;
  t372 = Cos(var1[7]);
  t794 = t622*t733*t759;
  t854 = t803*t837;
  t881 = t794 + t854;
  t959 = t372*t881;
  t1273 = t988*t1130;
  t1402 = -1.*t622*t1349*t1362;
  t1429 = t1273 + t1402;
  t1463 = Sin(var1[7]);
  t1481 = -1.*t1429*t1463;
  t1548 = t959 + t1481;
  t1675 = Sin(var1[8]);
  t27 = Sin(var1[9]);
  t2299 = t622*t733;
  t2322 = t803*t759*t837;
  t2336 = t2299 + t2322;
  t1919 = Cos(var1[9]);
  t2402 = t1349*t988*t803;
  t2437 = t2336*t1362;
  t2463 = t2402 + t2437;
  t2135 = t733*t803*t759;
  t2142 = -1.*t622*t837;
  t2164 = t2135 + t2142;
  t2170 = t372*t2164;
  t2355 = t988*t2336;
  t2359 = -1.*t1349*t803*t1362;
  t2361 = t2355 + t2359;
  t2365 = -1.*t2361*t1463;
  t2379 = t2170 + t2365;
  t3125 = -1.*t988*t759;
  t3130 = t1349*t837*t1362;
  t3152 = t3125 + t3130;
  t2915 = t1349*t733*t372;
  t2955 = t1349*t988*t837;
  t2974 = t759*t1362;
  t3052 = t2955 + t2974;
  t3056 = -1.*t3052*t1463;
  t3058 = t2915 + t3056;
  t1634 = t338*t1548;
  t1796 = t1669*t1675;
  t1916 = t1634 + t1796;
  t1942 = t338*t1669;
  t1962 = -1.*t1548*t1675;
  t1965 = t1942 + t1962;
  t2389 = t338*t2379;
  t2499 = t2463*t1675;
  t2526 = t2389 + t2499;
  t2749 = t338*t2463;
  t2840 = -1.*t2379*t1675;
  t2867 = t2749 + t2840;
  t3114 = t338*t3058;
  t3215 = t3152*t1675;
  t3250 = t3114 + t3215;
  t3309 = t338*t3152;
  t3310 = -1.*t3058*t1675;
  t3312 = t3309 + t3310;
  t3804 = -1.*t988;
  t3818 = 1. + t3804;
  t3948 = -1.*t372;
  t3963 = 1. + t3948;
  t3335 = t372*t1429;
  t3347 = t881*t1463;
  t3414 = t3335 + t3347;
  t4081 = -1.*t338;
  t4135 = 1. + t4081;
  t4243 = -1.*t1919;
  t4244 = 1. + t4243;
  t1918 = -1.*t27*t1916;
  t1996 = t1919*t1965;
  t2096 = t1918 + t1996;
  t3554 = t1919*t1916;
  t3560 = t27*t1965;
  t3572 = t3554 + t3560;
  t3836 = 0.087004*t3818;
  t3857 = 0.022225*t1362;
  t3864 = 0. + t3836 + t3857;
  t3906 = -0.022225*t3818;
  t3922 = 0.087004*t1362;
  t3933 = 0. + t3906 + t3922;
  t3970 = 0.157004*t3963;
  t3989 = -0.31508*t1463;
  t3997 = 0. + t3970 + t3989;
  t4033 = -0.31508*t3963;
  t4035 = -0.157004*t1463;
  t4038 = 0. + t4033 + t4035;
  t3445 = t372*t2361;
  t3450 = t2164*t1463;
  t3471 = t3445 + t3450;
  t4147 = -0.38008*t4135;
  t4196 = -0.022225*t1675;
  t4215 = 0. + t4147 + t4196;
  t4229 = -0.022225*t4135;
  t4233 = 0.38008*t1675;
  t4238 = 0. + t4229 + t4233;
  t4245 = -0.86008*t4244;
  t4251 = -0.022225*t27;
  t4272 = 0. + t4245 + t4251;
  t4284 = -0.022225*t4244;
  t4313 = 0.86008*t27;
  t4315 = 0. + t4284 + t4313;
  t2637 = -1.*t27*t2526;
  t2870 = t1919*t2867;
  t2884 = t2637 + t2870;
  t3595 = t1919*t2526;
  t3611 = t27*t2867;
  t3761 = t3595 + t3611;
  t3486 = t372*t3052;
  t3505 = t1349*t733*t1463;
  t3544 = t3486 + t3505;
  t3304 = -1.*t27*t3250;
  t3332 = t1919*t3312;
  t3334 = t3304 + t3332;
  t3762 = t1919*t3250;
  t3776 = t27*t3312;
  t3779 = t3762 + t3776;

  p_output1(0)=t2096;
  p_output1(1)=t2884;
  p_output1(2)=t3334;
  p_output1(3)=0.;
  p_output1(4)=t3414;
  p_output1(5)=t3471;
  p_output1(6)=t3544;
  p_output1(7)=0.;
  p_output1(8)=t3572;
  p_output1(9)=t3761;
  p_output1(10)=t3779;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2096 + 0.150254*t3414 - 0.86008*t3572 + t1130*t3864 + t1429*t3997 + t1548*t4215 + t1669*t4238 + t1916*t4272 + t1965*t4315 + t1349*t3933*t622 + t4038*t881 + var1(0);
  p_output1(13)=0. - 0.022225*t2884 + 0.150254*t3471 - 0.86008*t3761 + t2336*t3864 + t2361*t3997 + t2164*t4038 + t2379*t4215 + t2463*t4238 + t2526*t4272 + t2867*t4315 + t1349*t3933*t803 + var1(1);
  p_output1(14)=0. - 0.022225*t3334 + 0.150254*t3544 - 0.86008*t3779 + t3052*t3997 + t3058*t4215 + t3152*t4238 + t3250*t4272 + t3312*t4315 + t1349*t4038*t733 - 1.*t3933*t759 + t1349*t3864*t837 + var1(2);
  p_output1(15)=1.;
}


       
void H_lKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
