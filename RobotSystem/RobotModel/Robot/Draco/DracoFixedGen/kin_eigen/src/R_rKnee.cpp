/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:13 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rKnee.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1574;
  double t2361;
  double t2367;
  double t1240;
  double t2375;
  double t1239;
  double t2445;
  double t2501;
  double t2502;
  double t2423;
  double t2438;
  double t2442;
  double t2242;
  double t2409;
  double t2417;
  double t2480;
  double t2484;
  double t2495;
  double t2466;
  double t2469;
  double t2470;
  double t2514;
  double t2516;
  double t2507;
  double t2508;
  t1574 = Cos(var1[7]);
  t2361 = Sin(var1[5]);
  t2367 = Sin(var1[6]);
  t1240 = Cos(var1[5]);
  t2375 = Sin(var1[7]);
  t1239 = Cos(var1[8]);
  t2445 = Sin(var1[8]);
  t2501 = Cos(var1[6]);
  t2502 = 0. + t2501;
  t2423 = t1574*t2361*t2367;
  t2438 = t1240*t2375;
  t2442 = t2423 + t2438;
  t2242 = t1240*t1574;
  t2409 = -1.*t2361*t2367*t2375;
  t2417 = t2242 + t2409;
  t2480 = -1.*t1240*t1574*t2367;
  t2484 = t2361*t2375;
  t2495 = t2480 + t2484;
  t2466 = t1574*t2361;
  t2469 = t1240*t2367*t2375;
  t2470 = t2466 + t2469;
  t2514 = t2502*t1574;
  t2516 = 0. + t2514;
  t2507 = -1.*t2502*t2375;
  t2508 = 0. + t2507;

  p_output1(0)=t1239*t2417 - 1.*t2442*t2445;
  p_output1(1)=t1239*t2470 - 1.*t2445*t2495;
  p_output1(2)=t1239*t2508 - 1.*t2445*t2516;
  p_output1(3)=-1.*t2361*t2501;
  p_output1(4)=t1240*t2501;
  p_output1(5)=0. + t2367;
  p_output1(6)=t1239*t2442 + t2417*t2445;
  p_output1(7)=t2445*t2470 + t1239*t2495;
  p_output1(8)=t2445*t2508 + t1239*t2516;
}


       
void R_rKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
