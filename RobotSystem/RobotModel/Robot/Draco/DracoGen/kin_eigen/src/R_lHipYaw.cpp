/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:15 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipYaw.h"

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
  double t847;
  double t1954;
  double t2215;
  double t4194;
  double t4119;
  double t4237;
  double t4304;
  double t4313;
  double t4215;
  double t4310;
  double t4311;
  double t4517;
  double t4529;
  double t4531;
  t847 = Cos(var1[3]);
  t1954 = Cos(var1[4]);
  t2215 = Cos(var1[6]);
  t4194 = Sin(var1[3]);
  t4119 = Cos(var1[5]);
  t4237 = Sin(var1[4]);
  t4304 = Sin(var1[5]);
  t4313 = Sin(var1[6]);
  t4215 = -1.*t4119*t4194;
  t4310 = t847*t4237*t4304;
  t4311 = t4215 + t4310;
  t4517 = t847*t4119;
  t4529 = t4194*t4237*t4304;
  t4531 = t4517 + t4529;

  p_output1(0)=t4311*t4313 + t1954*t2215*t847;
  p_output1(1)=t1954*t2215*t4194 + t4313*t4531;
  p_output1(2)=-1.*t2215*t4237 + t1954*t4304*t4313;
  p_output1(3)=t2215*t4311 - 1.*t1954*t4313*t847;
  p_output1(4)=-1.*t1954*t4194*t4313 + t2215*t4531;
  p_output1(5)=t1954*t2215*t4304 + t4237*t4313;
  p_output1(6)=t4194*t4304 + t4119*t4237*t847;
  p_output1(7)=t4119*t4194*t4237 - 1.*t4304*t847;
  p_output1(8)=t1954*t4119;
}


       
void R_lHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
