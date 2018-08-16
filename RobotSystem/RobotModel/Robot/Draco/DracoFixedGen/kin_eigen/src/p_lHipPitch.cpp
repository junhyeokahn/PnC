/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:57 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lHipPitch.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t300;
  double t313;
  double t333;
  double t198;
  double t346;
  double t348;
  double t349;
  double t351;
  double t281;
  double t291;
  double t327;
  double t331;
  double t332;
  double t338;
  double t339;
  double t350;
  double t353;
  double t354;
  double t359;
  double t360;
  double t361;
  double t426;
  t300 = Sin(var1[0]);
  t313 = Cos(var1[1]);
  t333 = Sin(var1[1]);
  t198 = Cos(var1[0]);
  t346 = Cos(var1[2]);
  t348 = -1.*t346;
  t349 = 1. + t348;
  t351 = Sin(var1[2]);
  t281 = -1.*t198;
  t291 = 1. + t281;
  t327 = -1.*t313;
  t331 = 1. + t327;
  t332 = 0.331012*t331;
  t338 = -0.90524*t333;
  t339 = 0. + t332 + t338;
  t350 = -0.97024*t349;
  t353 = -0.066675*t351;
  t354 = 0. + t350 + t353;
  t359 = -0.066675*t349;
  t360 = 0.97024*t351;
  t361 = 0. + t359 + t360;
  t426 = 0. + t313;

  p_output1(0)=0. - 0.066675*t291 + 0.261012*t300 - 0.341012*t300*t313 - 1.*t300*t339 - 0.97024*(t300*t333*t346 + t198*t351) - 0.066675*(t198*t346 - 1.*t300*t333*t351) + t300*t333*t354 + t198*t361;
  p_output1(1)=0. + 0.261012*t291 + 0.066675*t300 + 0.341012*t198*t313 + t198*t339 - 0.97024*(-1.*t198*t333*t346 + t300*t351) - 0.066675*(t300*t346 + t198*t333*t351) - 1.*t198*t333*t354 + t300*t361;
  p_output1(2)=0. - 0.90524*t331 - 0.331012*t333 + 0.341012*(0. + t333) + t354*t426 - 0.97024*(0. + t346*t426) - 0.066675*(0. - 1.*t351*t426);
}


       
void p_lHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
