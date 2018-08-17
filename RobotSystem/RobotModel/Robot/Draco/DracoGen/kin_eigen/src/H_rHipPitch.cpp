/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:58 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipPitch.h"

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
  double t678;
  double t1450;
  double t1528;
  double t1479;
  double t1607;
  double t1331;
  double t1437;
  double t563;
  double t1503;
  double t1760;
  double t2068;
  double t483;
  double t2177;
  double t2249;
  double t2646;
  double t3231;
  double t3234;
  double t3357;
  double t2377;
  double t2410;
  double t2425;
  double t2717;
  double t2720;
  double t2727;
  double t3587;
  double t3617;
  double t3817;
  double t4145;
  double t4168;
  double t4262;
  double t4956;
  double t4959;
  double t4960;
  double t1401;
  double t2080;
  double t2108;
  double t2480;
  double t2740;
  double t2788;
  double t3225;
  double t3406;
  double t3446;
  double t4125;
  double t4336;
  double t4429;
  double t4712;
  double t4802;
  double t4832;
  double t4925;
  double t4967;
  double t4985;
  double t6162;
  double t6200;
  double t6280;
  double t6355;
  double t5176;
  double t5231;
  double t5251;
  double t6538;
  double t6544;
  double t5815;
  double t5823;
  double t5914;
  double t2114;
  double t2810;
  double t2956;
  double t6216;
  double t6236;
  double t6265;
  double t6382;
  double t6393;
  double t6430;
  double t6450;
  double t6451;
  double t6459;
  double t6483;
  double t6501;
  double t6517;
  double t6546;
  double t6552;
  double t6574;
  double t5261;
  double t5351;
  double t5582;
  double t6636;
  double t6640;
  double t6641;
  double t5958;
  double t5989;
  double t5996;
  double t3468;
  double t4484;
  double t4622;
  double t5683;
  double t5769;
  double t5793;
  double t6037;
  double t6048;
  double t6052;
  double t4894;
  double t5020;
  double t5126;
  t678 = Cos(var1[3]);
  t1450 = Cos(var1[5]);
  t1528 = Sin(var1[4]);
  t1479 = Sin(var1[3]);
  t1607 = Sin(var1[5]);
  t1331 = Cos(var1[4]);
  t1437 = Sin(var1[11]);
  t563 = Cos(var1[11]);
  t1503 = -1.*t1450*t1479;
  t1760 = t678*t1528*t1607;
  t2068 = t1503 + t1760;
  t483 = Cos(var1[13]);
  t2177 = Sin(var1[13]);
  t2249 = Cos(var1[12]);
  t2646 = Sin(var1[12]);
  t3231 = t678*t1450;
  t3234 = t1479*t1528*t1607;
  t3357 = t3231 + t3234;
  t2377 = t678*t1450*t1528;
  t2410 = t1479*t1607;
  t2425 = t2377 + t2410;
  t2717 = -1.*t678*t1331*t1437;
  t2720 = t563*t2068;
  t2727 = t2717 + t2720;
  t3587 = t1450*t1479*t1528;
  t3617 = -1.*t678*t1607;
  t3817 = t3587 + t3617;
  t4145 = -1.*t1331*t1437*t1479;
  t4168 = t563*t3357;
  t4262 = t4145 + t4168;
  t4956 = t1437*t1528;
  t4959 = t563*t1331*t1607;
  t4960 = t4956 + t4959;
  t1401 = t563*t678*t1331;
  t2080 = t1437*t2068;
  t2108 = t1401 + t2080;
  t2480 = t2249*t2425;
  t2740 = -1.*t2646*t2727;
  t2788 = t2480 + t2740;
  t3225 = t563*t1331*t1479;
  t3406 = t1437*t3357;
  t3446 = t3225 + t3406;
  t4125 = t2249*t3817;
  t4336 = -1.*t2646*t4262;
  t4429 = t4125 + t4336;
  t4712 = -1.*t563*t1528;
  t4802 = t1331*t1437*t1607;
  t4832 = t4712 + t4802;
  t4925 = t2249*t1331*t1450;
  t4967 = -1.*t2646*t4960;
  t4985 = t4925 + t4967;
  t6162 = -1.*t563;
  t6200 = 1. + t6162;
  t6280 = -1.*t2249;
  t6355 = 1. + t6280;
  t5176 = t2646*t2425;
  t5231 = t2249*t2727;
  t5251 = t5176 + t5231;
  t6538 = -1.*t483;
  t6544 = 1. + t6538;
  t5815 = t2177*t2108;
  t5823 = t483*t2788;
  t5914 = t5815 + t5823;
  t2114 = t483*t2108;
  t2810 = -1.*t2177*t2788;
  t2956 = t2114 + t2810;
  t6216 = -0.022225*t6200;
  t6236 = -0.086996*t1437;
  t6265 = 0. + t6216 + t6236;
  t6382 = -0.31508*t6355;
  t6393 = 0.156996*t2646;
  t6430 = 0. + t6382 + t6393;
  t6450 = -0.086996*t6200;
  t6451 = 0.022225*t1437;
  t6459 = 0. + t6450 + t6451;
  t6483 = -0.156996*t6355;
  t6501 = -0.31508*t2646;
  t6517 = 0. + t6483 + t6501;
  t6546 = -0.022225*t6544;
  t6552 = 0.38008*t2177;
  t6574 = 0. + t6546 + t6552;
  t5261 = t2646*t3817;
  t5351 = t2249*t4262;
  t5582 = t5261 + t5351;
  t6636 = -0.38008*t6544;
  t6640 = -0.022225*t2177;
  t6641 = 0. + t6636 + t6640;
  t5958 = t2177*t3446;
  t5989 = t483*t4429;
  t5996 = t5958 + t5989;
  t3468 = t483*t3446;
  t4484 = -1.*t2177*t4429;
  t4622 = t3468 + t4484;
  t5683 = t1331*t1450*t2646;
  t5769 = t2249*t4960;
  t5793 = t5683 + t5769;
  t6037 = t2177*t4832;
  t6048 = t483*t4985;
  t6052 = t6037 + t6048;
  t4894 = t483*t4832;
  t5020 = -1.*t2177*t4985;
  t5126 = t4894 + t5020;

  p_output1(0)=t2956;
  p_output1(1)=t4622;
  p_output1(2)=t5126;
  p_output1(3)=0.;
  p_output1(4)=t5251;
  p_output1(5)=t5582;
  p_output1(6)=t5793;
  p_output1(7)=0.;
  p_output1(8)=t5914;
  p_output1(9)=t5996;
  p_output1(10)=t6052;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2956 - 0.166996*t5251 - 0.38008*t5914 + t2425*t6430 + t2068*t6459 + t2727*t6517 + t2108*t6574 + t2788*t6641 + t1331*t6265*t678 + var1(0);
  p_output1(13)=0. - 0.022225*t4622 - 0.166996*t5582 - 0.38008*t5996 + t1331*t1479*t6265 + t3817*t6430 + t3357*t6459 + t4262*t6517 + t3446*t6574 + t4429*t6641 + var1(1);
  p_output1(14)=0. - 0.022225*t5126 - 0.166996*t5793 - 0.38008*t6052 - 1.*t1528*t6265 + t1331*t1450*t6430 + t1331*t1607*t6459 + t4960*t6517 + t4832*t6574 + t4985*t6641 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
