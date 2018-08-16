/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:58 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipPitch.h"

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
  double t306;
  double t341;
  double t379;
  double t360;
  double t391;
  double t393;
  double t394;
  double t399;
  double t368;
  double t373;
  double t376;
  double t381;
  double t384;
  double t395;
  double t404;
  double t407;
  double t420;
  double t421;
  double t422;
  double t489;
  double t492;
  double t493;
  double t477;
  double t479;
  double t481;
  double t532;
  double t533;
  double t534;
  double t539;
  double t541;
  double t435;
  double t437;
  double t438;
  double t588;
  t306 = Cos(var1[0]);
  t341 = Cos(var1[1]);
  t379 = Sin(var1[1]);
  t360 = Sin(var1[0]);
  t391 = Cos(var1[2]);
  t393 = -1.*t391;
  t394 = 1. + t393;
  t399 = Sin(var1[2]);
  t368 = -1.*t341;
  t373 = 1. + t368;
  t376 = 0.331012*t373;
  t381 = -0.90524*t379;
  t384 = 0. + t376 + t381;
  t395 = -0.97024*t394;
  t404 = -0.066675*t399;
  t407 = 0. + t395 + t404;
  t420 = -0.066675*t394;
  t421 = 0.97024*t399;
  t422 = 0. + t420 + t421;
  t489 = -0.90524*t341;
  t492 = 0.331012*t379;
  t493 = t489 + t492;
  t477 = t306*t391;
  t479 = -1.*t360*t379*t399;
  t481 = t477 + t479;
  t532 = -0.066675*t391;
  t533 = -0.97024*t399;
  t534 = t532 + t533;
  t539 = 0.97024*t391;
  t541 = t539 + t404;
  t435 = t306*t391*t379;
  t437 = -1.*t360*t399;
  t438 = t435 + t437;
  t588 = 0. + t341;

  p_output1(0)=0.261012*t306 - 0.341012*t306*t341 - 0.066675*t360 - 1.*t306*t384 - 0.066675*(-1.*t360*t391 - 1.*t306*t379*t399) + t306*t379*t407 - 1.*t360*t422 - 0.97024*t438;
  p_output1(1)=0.066675*t306 + 0.261012*t360 - 0.341012*t341*t360 - 1.*t360*t384 - 0.97024*(t360*t379*t391 + t306*t399) + t360*t379*t407 + t306*t422 - 0.066675*t481;
  p_output1(2)=0;
  p_output1(3)=0.341012*t360*t379 - 0.97024*t341*t360*t391 + 0.066675*t341*t360*t399 + t341*t360*t407 - 1.*t360*t493;
  p_output1(4)=-0.341012*t306*t379 + 0.97024*t306*t341*t391 - 0.066675*t306*t341*t399 - 1.*t306*t341*t407 + t306*t493;
  p_output1(5)=0.010000000000000009*t341 + t381 + 0.97024*t379*t391 - 0.066675*t379*t399 - 1.*t379*t407;
  p_output1(6)=-0.066675*(-1.*t360*t379*t391 - 1.*t306*t399) - 0.97024*t481 + t360*t379*t534 + t306*t541;
  p_output1(7)=-0.97024*(t360*t391 + t306*t379*t399) - 0.066675*t438 - 1.*t306*t379*t534 + t360*t541;
  p_output1(8)=0.066675*t391*t588 + 0.97024*t399*t588 + t534*t588;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0;
  p_output1(17)=0;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
}


       
void Jp_lHipPitch(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
