/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipRoll.h"

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
  double t913;
  double t49;
  double t963;
  double t1665;
  double t1544;
  double t1566;
  double t1729;
  double t1806;
  double t1711;
  double t1945;
  double t2193;
  double t3901;
  double t4769;
  double t2551;
  double t2662;
  double t2775;
  double t4006;
  double t4614;
  double t4653;
  double t4822;
  double t4896;
  double t4897;
  double t5048;
  double t5100;
  double t5133;
  double t5228;
  double t5253;
  double t5257;
  double t5341;
  double t5346;
  double t5389;
  double t5795;
  double t5798;
  double t5862;
  double t5884;
  double t1046;
  double t2253;
  double t2388;
  double t4747;
  double t4905;
  double t5045;
  double t5589;
  double t5612;
  double t5629;
  double t5807;
  double t5825;
  double t5829;
  double t5900;
  double t5910;
  double t5983;
  double t6022;
  double t6043;
  double t6050;
  double t6056;
  double t6062;
  double t6073;
  double t2393;
  double t2834;
  double t3059;
  double t5196;
  double t5316;
  double t5318;
  double t5696;
  double t5709;
  double t5730;
  double t3397;
  double t3805;
  double t3828;
  double t5332;
  double t5396;
  double t5435;
  double t5751;
  double t5757;
  double t5782;
  t913 = Cos(var1[3]);
  t49 = Cos(var1[11]);
  t963 = Cos(var1[4]);
  t1665 = Sin(var1[3]);
  t1544 = Sin(var1[11]);
  t1566 = Cos(var1[5]);
  t1729 = Sin(var1[4]);
  t1806 = Sin(var1[5]);
  t1711 = -1.*t1566*t1665;
  t1945 = t913*t1729*t1806;
  t2193 = t1711 + t1945;
  t3901 = Sin(var1[12]);
  t4769 = Cos(var1[12]);
  t2551 = t913*t1566;
  t2662 = t1665*t1729*t1806;
  t2775 = t2551 + t2662;
  t4006 = t913*t1566*t1729;
  t4614 = t1665*t1806;
  t4653 = t4006 + t4614;
  t4822 = -1.*t913*t963*t1544;
  t4896 = t49*t2193;
  t4897 = t4822 + t4896;
  t5048 = t1566*t1665*t1729;
  t5100 = -1.*t913*t1806;
  t5133 = t5048 + t5100;
  t5228 = -1.*t963*t1544*t1665;
  t5253 = t49*t2775;
  t5257 = t5228 + t5253;
  t5341 = t1544*t1729;
  t5346 = t49*t963*t1806;
  t5389 = t5341 + t5346;
  t5795 = -1.*t49;
  t5798 = 1. + t5795;
  t5862 = -1.*t4769;
  t5884 = 1. + t5862;
  t1046 = t49*t913*t963;
  t2253 = t1544*t2193;
  t2388 = t1046 + t2253;
  t4747 = t3901*t4653;
  t4905 = t4769*t4897;
  t5045 = t4747 + t4905;
  t5589 = t4769*t4653;
  t5612 = -1.*t3901*t4897;
  t5629 = t5589 + t5612;
  t5807 = -0.022225*t5798;
  t5825 = -0.086996*t1544;
  t5829 = 0. + t5807 + t5825;
  t5900 = -0.31508*t5884;
  t5910 = 0.156996*t3901;
  t5983 = 0. + t5900 + t5910;
  t6022 = -0.086996*t5798;
  t6043 = 0.022225*t1544;
  t6050 = 0. + t6022 + t6043;
  t6056 = -0.156996*t5884;
  t6062 = -0.31508*t3901;
  t6073 = 0. + t6056 + t6062;
  t2393 = t49*t963*t1665;
  t2834 = t1544*t2775;
  t3059 = t2393 + t2834;
  t5196 = t3901*t5133;
  t5316 = t4769*t5257;
  t5318 = t5196 + t5316;
  t5696 = t4769*t5133;
  t5709 = -1.*t3901*t5257;
  t5730 = t5696 + t5709;
  t3397 = -1.*t49*t1729;
  t3805 = t963*t1544*t1806;
  t3828 = t3397 + t3805;
  t5332 = t963*t1566*t3901;
  t5396 = t4769*t5389;
  t5435 = t5332 + t5396;
  t5751 = t4769*t963*t1566;
  t5757 = -1.*t3901*t5389;
  t5782 = t5751 + t5757;

  p_output1(0)=t2388;
  p_output1(1)=t3059;
  p_output1(2)=t3828;
  p_output1(3)=0.;
  p_output1(4)=t5045;
  p_output1(5)=t5318;
  p_output1(6)=t5435;
  p_output1(7)=0.;
  p_output1(8)=t5629;
  p_output1(9)=t5730;
  p_output1(10)=t5782;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2388 - 0.156996*t5045 - 0.31508*t5629 + t4653*t5983 + t2193*t6050 + t4897*t6073 + t5829*t913*t963 + var1(0);
  p_output1(13)=0. - 0.022225*t3059 - 0.156996*t5318 - 0.31508*t5730 + t5133*t5983 + t2775*t6050 + t5257*t6073 + t1665*t5829*t963 + var1(1);
  p_output1(14)=0. - 0.022225*t3828 - 0.156996*t5435 - 0.31508*t5782 - 1.*t1729*t5829 + t5389*t6073 + t1566*t5983*t963 + t1806*t6050*t963 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
