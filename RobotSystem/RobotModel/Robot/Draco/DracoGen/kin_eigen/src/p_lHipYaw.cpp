/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:30 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lHipYaw.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t4540;
  double t4654;
  double t4269;
  double t4633;
  double t4793;
  double t4920;
  double t4925;
  double t4932;
  double t4955;
  double t4883;
  double t4887;
  double t4915;
  double t4981;
  double t4949;
  double t4959;
  double t4969;
  double t4985;
  double t4988;
  double t4991;
  double t5092;
  double t5093;
  double t5095;
  t4540 = Cos(var1[5]);
  t4654 = Sin(var1[3]);
  t4269 = Cos(var1[3]);
  t4633 = Sin(var1[4]);
  t4793 = Sin(var1[5]);
  t4920 = Cos(var1[6]);
  t4925 = -1.*t4920;
  t4932 = 1. + t4925;
  t4955 = Sin(var1[6]);
  t4883 = -1.*t4540*t4654;
  t4887 = t4269*t4633*t4793;
  t4915 = t4883 + t4887;
  t4981 = Cos(var1[4]);
  t4949 = 0.087004*t4932;
  t4959 = 0.022225*t4955;
  t4969 = 0. + t4949 + t4959;
  t4985 = -0.022225*t4932;
  t4988 = 0.087004*t4955;
  t4991 = 0. + t4985 + t4988;
  t5092 = t4269*t4540;
  t5093 = t4654*t4633*t4793;
  t5095 = t5092 + t5093;

  p_output1(0)=0. - 0.29508*(t4269*t4540*t4633 + t4654*t4793) + t4915*t4969 - 0.022225*(t4915*t4955 + t4269*t4920*t4981) + 0.087004*(t4915*t4920 - 1.*t4269*t4955*t4981) + t4269*t4981*t4991 + var1(0);
  p_output1(1)=0. - 0.29508*(t4540*t4633*t4654 - 1.*t4269*t4793) + t4654*t4981*t4991 + t4969*t5095 + 0.087004*(-1.*t4654*t4955*t4981 + t4920*t5095) - 0.022225*(t4654*t4920*t4981 + t4955*t5095) + var1(1);
  p_output1(2)=0. - 0.29508*t4540*t4981 + t4793*t4969*t4981 + 0.087004*(t4633*t4955 + t4793*t4920*t4981) - 0.022225*(-1.*t4633*t4920 + t4793*t4955*t4981) - 1.*t4633*t4991 + var1(2);
}


       
void p_lHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
