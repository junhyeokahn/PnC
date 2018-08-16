/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:58 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipPitch.h"

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
  double t404;
  double t420;
  double t362;
  double t421;
  double t439;
  double t456;
  double t462;
  double t538;
  double t539;
  double t477;
  double t486;
  double t488;
  double t413;
  double t441;
  double t443;
  double t518;
  double t519;
  double t525;
  double t527;
  double t528;
  double t530;
  double t532;
  double t541;
  double t543;
  double t544;
  double t549;
  double t552;
  double t558;
  double t492;
  double t500;
  double t502;
  double t446;
  double t449;
  double t450;
  double t516;
  double t517;
  double t475;
  double t464;
  double t465;
  t404 = Cos(var1[2]);
  t420 = Sin(var1[0]);
  t362 = Cos(var1[0]);
  t421 = Sin(var1[1]);
  t439 = Sin(var1[2]);
  t456 = Cos(var1[1]);
  t462 = 0. + t456;
  t538 = -1.*t404;
  t539 = 1. + t538;
  t477 = t404*t420*t421;
  t486 = t362*t439;
  t488 = t477 + t486;
  t413 = t362*t404;
  t441 = -1.*t420*t421*t439;
  t443 = t413 + t441;
  t518 = -1.*t362;
  t519 = 1. + t518;
  t525 = -1.*t456;
  t527 = 1. + t525;
  t528 = 0.331012*t527;
  t530 = -0.90524*t421;
  t532 = 0. + t528 + t530;
  t541 = -0.97024*t539;
  t543 = -0.066675*t439;
  t544 = 0. + t541 + t543;
  t549 = -0.066675*t539;
  t552 = 0.97024*t439;
  t558 = 0. + t549 + t552;
  t492 = -1.*t362*t404*t421;
  t500 = t420*t439;
  t502 = t492 + t500;
  t446 = t404*t420;
  t449 = t362*t421*t439;
  t450 = t446 + t449;
  t516 = t462*t404;
  t517 = 0. + t516;
  t475 = 0. + t421;
  t464 = -1.*t462*t439;
  t465 = 0. + t464;

  p_output1(0)=t443;
  p_output1(1)=t450;
  p_output1(2)=t465;
  p_output1(3)=0.;
  p_output1(4)=-1.*t420*t456;
  p_output1(5)=t362*t456;
  p_output1(6)=t475;
  p_output1(7)=0.;
  p_output1(8)=t488;
  p_output1(9)=t502;
  p_output1(10)=t517;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.261012*t420 - 0.066675*t443 - 0.341012*t420*t456 - 0.97024*t488 - 0.066675*t519 - 1.*t420*t532 + t420*t421*t544 + t362*t558;
  p_output1(13)=0. + 0.066675*t420 - 0.066675*t450 + 0.341012*t362*t456 - 0.97024*t502 + 0.261012*t519 + t362*t532 - 1.*t362*t421*t544 + t420*t558;
  p_output1(14)=0. - 0.331012*t421 - 0.066675*t465 + 0.341012*t475 - 0.97024*t517 - 0.90524*t527 + t462*t544;
  p_output1(15)=1.;
}


       
void H_lHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
