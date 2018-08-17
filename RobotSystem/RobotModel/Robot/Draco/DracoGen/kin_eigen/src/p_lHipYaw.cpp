/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:40 GMT-05:00
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
  double t2604;
  double t3202;
  double t865;
  double t3047;
  double t3218;
  double t3714;
  double t3717;
  double t3730;
  double t3872;
  double t3626;
  double t3677;
  double t3700;
  double t4054;
  double t3733;
  double t3888;
  double t3902;
  double t4055;
  double t4093;
  double t4102;
  double t4417;
  double t4422;
  double t4436;
  t2604 = Cos(var1[5]);
  t3202 = Sin(var1[3]);
  t865 = Cos(var1[3]);
  t3047 = Sin(var1[4]);
  t3218 = Sin(var1[5]);
  t3714 = Cos(var1[6]);
  t3717 = -1.*t3714;
  t3730 = 1. + t3717;
  t3872 = Sin(var1[6]);
  t3626 = -1.*t2604*t3202;
  t3677 = t865*t3047*t3218;
  t3700 = t3626 + t3677;
  t4054 = Cos(var1[4]);
  t3733 = 0.087004*t3730;
  t3888 = 0.022225*t3872;
  t3902 = 0. + t3733 + t3888;
  t4055 = -0.022225*t3730;
  t4093 = 0.087004*t3872;
  t4102 = 0. + t4055 + t4093;
  t4417 = t865*t2604;
  t4422 = t3202*t3047*t3218;
  t4436 = t4417 + t4422;

  p_output1(0)=0. + t3700*t3902 + t4054*t4102*t865 - 0.29508*(t3202*t3218 + t2604*t3047*t865) - 0.022225*(t3700*t3872 + t3714*t4054*t865) + 0.087004*(t3700*t3714 - 1.*t3872*t4054*t865) + var1(0);
  p_output1(1)=0. + t3202*t4054*t4102 + t3902*t4436 + 0.087004*(-1.*t3202*t3872*t4054 + t3714*t4436) - 0.022225*(t3202*t3714*t4054 + t3872*t4436) - 0.29508*(t2604*t3047*t3202 - 1.*t3218*t865) + var1(1);
  p_output1(2)=0. - 0.29508*t2604*t4054 + t3218*t3902*t4054 + 0.087004*(t3047*t3872 + t3218*t3714*t4054) - 0.022225*(-1.*t3047*t3714 + t3218*t3872*t4054) - 1.*t3047*t4102 + var1(2);
}


       
void p_lHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
