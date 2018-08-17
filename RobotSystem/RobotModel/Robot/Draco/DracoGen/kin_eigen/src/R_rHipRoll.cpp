/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:56 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t2834;
  double t2109;
  double t2881;
  double t4060;
  double t3392;
  double t3483;
  double t4562;
  double t4614;
  double t4489;
  double t4653;
  double t4665;
  double t5840;
  double t6014;
  double t5318;
  double t5332;
  double t5435;
  double t5900;
  double t5910;
  double t5983;
  double t6022;
  double t6043;
  double t6050;
  double t6062;
  double t6073;
  double t6079;
  double t6102;
  double t6112;
  double t6113;
  double t6161;
  double t6174;
  double t6185;
  t2834 = Cos(var1[3]);
  t2109 = Cos(var1[11]);
  t2881 = Cos(var1[4]);
  t4060 = Sin(var1[3]);
  t3392 = Sin(var1[11]);
  t3483 = Cos(var1[5]);
  t4562 = Sin(var1[4]);
  t4614 = Sin(var1[5]);
  t4489 = -1.*t3483*t4060;
  t4653 = t2834*t4562*t4614;
  t4665 = t4489 + t4653;
  t5840 = Sin(var1[12]);
  t6014 = Cos(var1[12]);
  t5318 = t2834*t3483;
  t5332 = t4060*t4562*t4614;
  t5435 = t5318 + t5332;
  t5900 = t2834*t3483*t4562;
  t5910 = t4060*t4614;
  t5983 = t5900 + t5910;
  t6022 = -1.*t2834*t2881*t3392;
  t6043 = t2109*t4665;
  t6050 = t6022 + t6043;
  t6062 = t3483*t4060*t4562;
  t6073 = -1.*t2834*t4614;
  t6079 = t6062 + t6073;
  t6102 = -1.*t2881*t3392*t4060;
  t6112 = t2109*t5435;
  t6113 = t6102 + t6112;
  t6161 = t3392*t4562;
  t6174 = t2109*t2881*t4614;
  t6185 = t6161 + t6174;

  p_output1(0)=t2109*t2834*t2881 + t3392*t4665;
  p_output1(1)=t2109*t2881*t4060 + t3392*t5435;
  p_output1(2)=-1.*t2109*t4562 + t2881*t3392*t4614;
  p_output1(3)=t5840*t5983 + t6014*t6050;
  p_output1(4)=t5840*t6079 + t6014*t6113;
  p_output1(5)=t2881*t3483*t5840 + t6014*t6185;
  p_output1(6)=t5983*t6014 - 1.*t5840*t6050;
  p_output1(7)=t6014*t6079 - 1.*t5840*t6113;
  p_output1(8)=t2881*t3483*t6014 - 1.*t5840*t6185;
}


       
void R_rHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
