/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:22 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootFront.h"

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
  double t1344;
  double t1376;
  double t1498;
  double t1120;
  double t1601;
  double t998;
  double t1839;
  double t1874;
  double t1896;
  double t1361;
  double t1677;
  double t1787;
  double t1904;
  double t1982;
  double t1796;
  double t1906;
  double t1976;
  double t455;
  double t1988;
  double t2020;
  double t2022;
  double t3794;
  double t3795;
  double t3933;
  double t3236;
  double t3278;
  double t3397;
  double t3522;
  double t3977;
  double t4016;
  double t4131;
  double t4147;
  double t4186;
  double t4202;
  double t4204;
  double t4216;
  double t4219;
  double t4205;
  double t4206;
  double t4215;
  double t4225;
  double t4228;
  double t4236;
  double t4249;
  double t4251;
  double t1980;
  double t2103;
  double t4126;
  double t4190;
  double t4233;
  double t4256;
  double t4394;
  double t4396;
  double t4409;
  double t4410;
  double t4432;
  double t4433;
  double t4319;
  double t2255;
  double t2318;
  double t2320;
  double t4374;
  double t4376;
  double t4385;
  double t4387;
  double t4390;
  double t4391;
  double t4392;
  double t4398;
  double t4400;
  double t4401;
  double t4403;
  double t4404;
  double t4407;
  double t4412;
  double t4413;
  double t4415;
  double t4425;
  double t4426;
  double t4428;
  double t4434;
  double t4436;
  double t4437;
  double t4441;
  double t4442;
  double t4444;
  double t4354;
  double t4192;
  double t4194;
  double t4196;
  double t4307;
  double t4363;
  double t4264;
  double t4268;
  double t4285;
  t1344 = Cos(var1[7]);
  t1376 = Sin(var1[5]);
  t1498 = Sin(var1[6]);
  t1120 = Cos(var1[5]);
  t1601 = Sin(var1[7]);
  t998 = Cos(var1[8]);
  t1839 = t1344*t1376*t1498;
  t1874 = t1120*t1601;
  t1896 = t1839 + t1874;
  t1361 = t1120*t1344;
  t1677 = -1.*t1376*t1498*t1601;
  t1787 = t1361 + t1677;
  t1904 = Sin(var1[8]);
  t1982 = Cos(var1[9]);
  t1796 = t998*t1787;
  t1906 = -1.*t1896*t1904;
  t1976 = t1796 + t1906;
  t455 = Sin(var1[9]);
  t1988 = t998*t1896;
  t2020 = t1787*t1904;
  t2022 = t1988 + t2020;
  t3794 = -1.*t1120*t1344*t1498;
  t3795 = t1376*t1601;
  t3933 = t3794 + t3795;
  t3236 = t1344*t1376;
  t3278 = t1120*t1498*t1601;
  t3397 = t3236 + t3278;
  t3522 = t998*t3397;
  t3977 = -1.*t3933*t1904;
  t4016 = t3522 + t3977;
  t4131 = t998*t3933;
  t4147 = t3397*t1904;
  t4186 = t4131 + t4147;
  t4202 = Cos(var1[6]);
  t4204 = 0. + t4202;
  t4216 = t4204*t1344;
  t4219 = 0. + t4216;
  t4205 = -1.*t4204*t1601;
  t4206 = 0. + t4205;
  t4215 = t998*t4206;
  t4225 = -1.*t4219*t1904;
  t4228 = t4215 + t4225;
  t4236 = t4219*t998;
  t4249 = t4206*t1904;
  t4251 = t4236 + t4249;
  t1980 = t455*t1976;
  t2103 = t1982*t2022;
  t4126 = t455*t4016;
  t4190 = t1982*t4186;
  t4233 = t455*t4228;
  t4256 = t1982*t4251;
  t4394 = -1.*t1344;
  t4396 = 1. + t4394;
  t4409 = -1.*t998;
  t4410 = 1. + t4409;
  t4432 = -1.*t1982;
  t4433 = 1. + t4432;
  t4319 = t1980 + t2103;
  t2255 = t1982*t1976;
  t2318 = -1.*t455*t2022;
  t2320 = t2255 + t2318;
  t4374 = -1.*t1120;
  t4376 = 1. + t4374;
  t4385 = -1.*t4202;
  t4387 = 1. + t4385;
  t4390 = -0.330988*t4387;
  t4391 = -0.90524*t1498;
  t4392 = 0. + t4390 + t4391;
  t4398 = -0.97024*t4396;
  t4400 = -0.066675*t1601;
  t4401 = 0. + t4398 + t4400;
  t4403 = -0.066675*t4396;
  t4404 = 0.97024*t1601;
  t4407 = 0. + t4403 + t4404;
  t4412 = -1.45024*t4410;
  t4413 = -0.066675*t1904;
  t4415 = 0. + t4412 + t4413;
  t4425 = -0.066675*t4410;
  t4426 = 1.45024*t1904;
  t4428 = 0. + t4425 + t4426;
  t4434 = -0.065597*t4433;
  t4436 = 1.93024*t455;
  t4437 = 0. + t4434 + t4436;
  t4441 = -1.93024*t4433;
  t4442 = -0.065597*t455;
  t4444 = 0. + t4441 + t4442;
  t4354 = t4126 + t4190;
  t4192 = t1982*t4016;
  t4194 = -1.*t455*t4186;
  t4196 = t4192 + t4194;
  t4307 = 0. + t1498;
  t4363 = t4233 + t4256;
  t4264 = t1982*t4228;
  t4268 = -1.*t455*t4251;
  t4285 = t4264 + t4268;

  p_output1(0)=t1980 + t2103 + 0.000796*t2320;
  p_output1(1)=t4126 + t4190 + 0.000796*t4196;
  p_output1(2)=t4233 + t4256 + 0.000796*t4285;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1376*t4202;
  p_output1(5)=t1120*t4202;
  p_output1(6)=t4307;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1976*t1982 + 0.000796*t4319 + t2022*t455;
  p_output1(9)=-1.*t1982*t4016 + 0.000796*t4354 + t4186*t455;
  p_output1(10)=-1.*t1982*t4228 + 0.000796*t4363 + t4251*t455;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.260988*t1376 - 0.000525*t2320 + 0.340988*t1376*t4202 - 1.840292*t4319 - 0.066675*t4376 - 1.*t1376*t4392 + t1376*t1498*t4401 + t1120*t4407 + t1896*t4415 + t1787*t4428 + t1976*t4437 + t2022*t4444;
  p_output1(13)=0. + 0.066675*t1376 - 0.000525*t4196 - 0.340988*t1120*t4202 - 1.840292*t4354 - 0.260988*t4376 + t1120*t4392 - 1.*t1120*t1498*t4401 + t1376*t4407 + t3933*t4415 + t3397*t4428 + t4016*t4437 + t4186*t4444;
  p_output1(14)=0. + 0.330988*t1498 - 0.000525*t4285 - 0.340988*t4307 - 1.840292*t4363 - 0.90524*t4387 + t4204*t4401 + t4219*t4415 + t4206*t4428 + t4228*t4437 + t4251*t4444;
  p_output1(15)=1.;
}


       
void H_RightFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
