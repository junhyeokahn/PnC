/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:22 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootFront.h"

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
  double t2103;
  double t2255;
  double t2496;
  double t2020;
  double t3105;
  double t1796;
  double t3795;
  double t3856;
  double t3993;
  double t2178;
  double t3278;
  double t3397;
  double t3994;
  double t4190;
  double t3522;
  double t4126;
  double t4131;
  double t1361;
  double t4192;
  double t4196;
  double t4201;
  double t4319;
  double t4325;
  double t4354;
  double t4288;
  double t4292;
  double t4299;
  double t4317;
  double t4356;
  double t4357;
  double t4363;
  double t4364;
  double t4370;
  double t4398;
  double t4400;
  double t4407;
  double t4408;
  double t4402;
  double t4403;
  double t4404;
  double t4412;
  double t4413;
  double t4422;
  double t4425;
  double t4426;
  double t4147;
  double t4233;
  double t4362;
  double t4377;
  double t4415;
  double t4428;
  t2103 = Cos(var1[7]);
  t2255 = Sin(var1[5]);
  t2496 = Sin(var1[6]);
  t2020 = Cos(var1[5]);
  t3105 = Sin(var1[7]);
  t1796 = Cos(var1[8]);
  t3795 = t2103*t2255*t2496;
  t3856 = t2020*t3105;
  t3993 = t3795 + t3856;
  t2178 = t2020*t2103;
  t3278 = -1.*t2255*t2496*t3105;
  t3397 = t2178 + t3278;
  t3994 = Sin(var1[8]);
  t4190 = Cos(var1[9]);
  t3522 = t1796*t3397;
  t4126 = -1.*t3993*t3994;
  t4131 = t3522 + t4126;
  t1361 = Sin(var1[9]);
  t4192 = t1796*t3993;
  t4196 = t3397*t3994;
  t4201 = t4192 + t4196;
  t4319 = -1.*t2020*t2103*t2496;
  t4325 = t2255*t3105;
  t4354 = t4319 + t4325;
  t4288 = t2103*t2255;
  t4292 = t2020*t2496*t3105;
  t4299 = t4288 + t4292;
  t4317 = t1796*t4299;
  t4356 = -1.*t4354*t3994;
  t4357 = t4317 + t4356;
  t4363 = t1796*t4354;
  t4364 = t4299*t3994;
  t4370 = t4363 + t4364;
  t4398 = Cos(var1[6]);
  t4400 = 0. + t4398;
  t4407 = t4400*t2103;
  t4408 = 0. + t4407;
  t4402 = -1.*t4400*t3105;
  t4403 = 0. + t4402;
  t4404 = t1796*t4403;
  t4412 = -1.*t4408*t3994;
  t4413 = t4404 + t4412;
  t4422 = t4408*t1796;
  t4425 = t4403*t3994;
  t4426 = t4422 + t4425;
  t4147 = t1361*t4131;
  t4233 = t4190*t4201;
  t4362 = t1361*t4357;
  t4377 = t4190*t4370;
  t4415 = t1361*t4413;
  t4428 = t4190*t4426;

  p_output1(0)=t4147 + 0.000796*(t4131*t4190 - 1.*t1361*t4201) + t4233;
  p_output1(1)=t4362 + 0.000796*(t4190*t4357 - 1.*t1361*t4370) + t4377;
  p_output1(2)=t4415 + 0.000796*(t4190*t4413 - 1.*t1361*t4426) + t4428;
  p_output1(3)=-1.*t2255*t4398;
  p_output1(4)=t2020*t4398;
  p_output1(5)=0. + t2496;
  p_output1(6)=-1.*t4131*t4190 + t1361*t4201 + 0.000796*(t4147 + t4233);
  p_output1(7)=-1.*t4190*t4357 + t1361*t4370 + 0.000796*(t4362 + t4377);
  p_output1(8)=-1.*t4190*t4413 + t1361*t4426 + 0.000796*(t4415 + t4428);
}


       
void R_RightFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
