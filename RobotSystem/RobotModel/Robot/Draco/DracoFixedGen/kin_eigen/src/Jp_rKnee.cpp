/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:12 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rKnee.h"

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
static void output1(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t61;
  double t160;
  double t1765;
  double t284;
  double t2164;
  double t2185;
  double t2190;
  double t2215;
  double t2282;
  double t2294;
  double t2295;
  double t2304;
  double t2316;
  double t2317;
  double t2321;
  double t2269;
  double t2274;
  double t2279;
  double t745;
  double t759;
  double t1179;
  double t1778;
  double t1779;
  double t2205;
  double t2220;
  double t2222;
  double t2234;
  double t2242;
  double t2249;
  double t2300;
  double t2311;
  double t2312;
  double t2322;
  double t2323;
  double t2325;
  double t2404;
  double t2407;
  double t2408;
  double t2393;
  double t2395;
  double t2399;
  double t2423;
  double t2424;
  double t2425;
  double t2410;
  double t2536;
  double t2538;
  double t2543;
  double t2514;
  double t2516;
  double t2519;
  double t2523;
  double t2526;
  double t2564;
  double t2566;
  double t2567;
  double t2348;
  double t2594;
  double t2415;
  double t2416;
  double t2551;
  double t2625;
  double t2626;
  double t2628;
  double t2631;
  double t2633;
  double t2578;
  double t2648;
  double t2649;
  double t2653;
  double t2584;
  double t2675;
  double t2677;
  double t2669;
  double t2671;
  t61 = Cos(var1[5]);
  t160 = Cos(var1[6]);
  t1765 = Sin(var1[6]);
  t284 = Sin(var1[5]);
  t2164 = Cos(var1[7]);
  t2185 = -1.*t2164;
  t2190 = 1. + t2185;
  t2215 = Sin(var1[7]);
  t2282 = Cos(var1[8]);
  t2294 = -1.*t2282;
  t2295 = 1. + t2294;
  t2304 = Sin(var1[8]);
  t2316 = -1.*t2164*t284;
  t2317 = -1.*t61*t1765*t2215;
  t2321 = t2316 + t2317;
  t2269 = t61*t2164*t1765;
  t2274 = -1.*t284*t2215;
  t2279 = t2269 + t2274;
  t745 = -1.*t160;
  t759 = 1. + t745;
  t1179 = -0.330988*t759;
  t1778 = -0.90524*t1765;
  t1779 = 0. + t1179 + t1778;
  t2205 = -0.97024*t2190;
  t2220 = -0.066675*t2215;
  t2222 = 0. + t2205 + t2220;
  t2234 = -0.066675*t2190;
  t2242 = 0.97024*t2215;
  t2249 = 0. + t2234 + t2242;
  t2300 = -1.45024*t2295;
  t2311 = -0.066675*t2304;
  t2312 = 0. + t2300 + t2311;
  t2322 = -0.066675*t2295;
  t2323 = 1.45024*t2304;
  t2325 = 0. + t2322 + t2323;
  t2404 = t61*t2164;
  t2407 = -1.*t284*t1765*t2215;
  t2408 = t2404 + t2407;
  t2393 = t2164*t284*t1765;
  t2395 = t61*t2215;
  t2399 = t2393 + t2395;
  t2423 = -0.90524*t160;
  t2424 = -0.330988*t1765;
  t2425 = t2423 + t2424;
  t2410 = t2282*t2408;
  t2536 = -1.*t2164*t284*t1765;
  t2538 = -1.*t61*t2215;
  t2543 = t2536 + t2538;
  t2514 = -0.066675*t2164;
  t2516 = -0.97024*t2215;
  t2519 = t2514 + t2516;
  t2523 = 0.97024*t2164;
  t2526 = t2523 + t2220;
  t2564 = t2164*t284;
  t2566 = t61*t1765*t2215;
  t2567 = t2564 + t2566;
  t2348 = t2282*t2279;
  t2594 = 0. + t160;
  t2415 = -1.*t2399*t2304;
  t2416 = t2410 + t2415;
  t2551 = -1.*t2408*t2304;
  t2625 = -0.066675*t2282;
  t2626 = -1.45024*t2304;
  t2628 = t2625 + t2626;
  t2631 = 1.45024*t2282;
  t2633 = t2631 + t2311;
  t2578 = t2282*t2567;
  t2648 = -1.*t61*t2164*t1765;
  t2649 = t284*t2215;
  t2653 = t2648 + t2649;
  t2584 = -1.*t2567*t2304;
  t2675 = -1.*t2594*t2215;
  t2677 = 0. + t2675;
  t2669 = t2594*t2164;
  t2671 = 0. + t2669;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=t2279*t2312 - 0.066675*(-1.*t2279*t2304 + t2282*t2321) + t2321*t2325 - 1.45024*(t2304*t2321 + t2348) - 0.066675*t284 - 1.*t2249*t284 - 0.260988*t61 + 0.324238*t160*t61 - 1.*t1779*t61 + t1765*t2222*t61;
  p_output1(16)=t2312*t2399 + t2325*t2408 - 1.45024*(t2282*t2399 + t2304*t2408) - 0.066675*t2416 - 0.260988*t284 + 0.324238*t160*t284 - 1.*t1779*t284 + t1765*t2222*t284 + 0.066675*t61 + t2249*t61;
  p_output1(17)=0;
  p_output1(18)=-0.324238*t1765*t284 + t160*t2222*t284 + t160*t2164*t2312*t284 - 1.*t160*t2215*t2325*t284 - 1.*t2425*t284 - 0.066675*(-1.*t160*t2215*t2282*t284 - 1.*t160*t2164*t2304*t284) - 1.45024*(t160*t2164*t2282*t284 - 1.*t160*t2215*t2304*t284);
  p_output1(19)=0.324238*t1765*t61 - 1.*t160*t2222*t61 - 1.*t160*t2164*t2312*t61 + t160*t2215*t2325*t61 + t2425*t61 - 0.066675*(t160*t2215*t2282*t61 + t160*t2164*t2304*t61) - 1.45024*(-1.*t160*t2164*t2282*t61 + t160*t2215*t2304*t61);
  p_output1(20)=0.006749999999999978*t160 + t1778 - 1.*t1765*t2222 - 0.066675*(t1765*t2215*t2282 + t1765*t2164*t2304) - 1.45024*(-1.*t1765*t2164*t2282 + t1765*t2215*t2304) - 1.*t1765*t2164*t2312 + t1765*t2215*t2325;
  p_output1(21)=t2312*t2408 + t2325*t2543 - 1.45024*(t2410 + t2304*t2543) - 0.066675*(t2282*t2543 + t2551) + t1765*t2519*t284 + t2526*t61;
  p_output1(22)=t2279*t2325 + t2312*t2567 - 1.45024*(t2279*t2304 + t2578) - 0.066675*(t2348 + t2584) + t2526*t284 - 1.*t1765*t2519*t61;
  p_output1(23)=-1.*t2215*t2312*t2594 - 1.*t2164*t2325*t2594 + t2519*t2594 - 1.45024*(-1.*t2215*t2282*t2594 - 1.*t2164*t2304*t2594) - 0.066675*(-1.*t2164*t2282*t2594 + t2215*t2304*t2594);
  p_output1(24)=-1.45024*t2416 - 0.066675*(-1.*t2282*t2399 + t2551) + t2399*t2628 + t2408*t2633;
  p_output1(25)=t2567*t2633 + t2628*t2653 - 0.066675*(t2584 - 1.*t2282*t2653) - 1.45024*(t2578 - 1.*t2304*t2653);
  p_output1(26)=t2628*t2671 + t2633*t2677 - 1.45024*(-1.*t2304*t2671 + t2282*t2677) - 0.066675*(-1.*t2282*t2671 - 1.*t2304*t2677);
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
}


       
void Jp_rKnee(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
