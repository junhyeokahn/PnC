/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:12 GMT-05:00
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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1317;
  double t1778;
  double t2016;
  double t1179;
  double t2232;
  double t427;
  double t2336;
  double t2395;
  double t2401;
  double t2269;
  double t2311;
  double t2323;
  double t1511;
  double t2234;
  double t2242;
  double t2379;
  double t2385;
  double t2388;
  double t2359;
  double t2361;
  double t2367;
  double t2418;
  double t2419;
  double t2404;
  double t2409;
  double t2486;
  double t2489;
  double t2509;
  double t2510;
  double t2266;
  double t2337;
  double t2348;
  double t2442;
  double t2443;
  double t2445;
  double t2462;
  double t2465;
  double t2471;
  double t2472;
  double t2473;
  double t2480;
  double t2481;
  double t2495;
  double t2497;
  double t2499;
  double t2501;
  double t2502;
  double t2507;
  double t2512;
  double t2514;
  double t2516;
  double t2523;
  double t2526;
  double t2529;
  double t2375;
  double t2390;
  double t2393;
  double t2446;
  double t2448;
  double t2452;
  double t2440;
  double t2417;
  double t2422;
  double t2423;
  double t2453;
  double t2454;
  double t2456;
  t1317 = Cos(var1[7]);
  t1778 = Sin(var1[5]);
  t2016 = Sin(var1[6]);
  t1179 = Cos(var1[5]);
  t2232 = Sin(var1[7]);
  t427 = Cos(var1[8]);
  t2336 = Sin(var1[8]);
  t2395 = Cos(var1[6]);
  t2401 = 0. + t2395;
  t2269 = t1317*t1778*t2016;
  t2311 = t1179*t2232;
  t2323 = t2269 + t2311;
  t1511 = t1179*t1317;
  t2234 = -1.*t1778*t2016*t2232;
  t2242 = t1511 + t2234;
  t2379 = -1.*t1179*t1317*t2016;
  t2385 = t1778*t2232;
  t2388 = t2379 + t2385;
  t2359 = t1317*t1778;
  t2361 = t1179*t2016*t2232;
  t2367 = t2359 + t2361;
  t2418 = t2401*t1317;
  t2419 = 0. + t2418;
  t2404 = -1.*t2401*t2232;
  t2409 = 0. + t2404;
  t2486 = -1.*t1317;
  t2489 = 1. + t2486;
  t2509 = -1.*t427;
  t2510 = 1. + t2509;
  t2266 = t427*t2242;
  t2337 = -1.*t2323*t2336;
  t2348 = t2266 + t2337;
  t2442 = t427*t2323;
  t2443 = t2242*t2336;
  t2445 = t2442 + t2443;
  t2462 = -1.*t1179;
  t2465 = 1. + t2462;
  t2471 = -1.*t2395;
  t2472 = 1. + t2471;
  t2473 = -0.330988*t2472;
  t2480 = -0.90524*t2016;
  t2481 = 0. + t2473 + t2480;
  t2495 = -0.97024*t2489;
  t2497 = -0.066675*t2232;
  t2499 = 0. + t2495 + t2497;
  t2501 = -0.066675*t2489;
  t2502 = 0.97024*t2232;
  t2507 = 0. + t2501 + t2502;
  t2512 = -1.45024*t2510;
  t2514 = -0.066675*t2336;
  t2516 = 0. + t2512 + t2514;
  t2523 = -0.066675*t2510;
  t2526 = 1.45024*t2336;
  t2529 = 0. + t2523 + t2526;
  t2375 = t427*t2367;
  t2390 = -1.*t2388*t2336;
  t2393 = t2375 + t2390;
  t2446 = t427*t2388;
  t2448 = t2367*t2336;
  t2452 = t2446 + t2448;
  t2440 = 0. + t2016;
  t2417 = t427*t2409;
  t2422 = -1.*t2419*t2336;
  t2423 = t2417 + t2422;
  t2453 = t2419*t427;
  t2454 = t2409*t2336;
  t2456 = t2453 + t2454;

  p_output1(0)=t2348;
  p_output1(1)=t2393;
  p_output1(2)=t2423;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1778*t2395;
  p_output1(5)=t1179*t2395;
  p_output1(6)=t2440;
  p_output1(7)=0.;
  p_output1(8)=t2445;
  p_output1(9)=t2452;
  p_output1(10)=t2456;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.260988*t1778 - 0.066675*t2348 + 0.324238*t1778*t2395 - 1.45024*t2445 - 0.066675*t2465 - 1.*t1778*t2481 + t1778*t2016*t2499 + t1179*t2507 + t2323*t2516 + t2242*t2529;
  p_output1(13)=0. + 0.066675*t1778 - 0.066675*t2393 - 0.324238*t1179*t2395 - 1.45024*t2452 - 0.260988*t2465 + t1179*t2481 - 1.*t1179*t2016*t2499 + t1778*t2507 + t2388*t2516 + t2367*t2529;
  p_output1(14)=0. + 0.330988*t2016 - 0.066675*t2423 - 0.324238*t2440 - 1.45024*t2456 - 0.90524*t2472 + t2401*t2499 + t2419*t2516 + t2409*t2529;
  p_output1(15)=1.;
}


       
void H_rKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
