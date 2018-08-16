/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:56 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipRoll.h"

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
  double t199;
  double t188;
  double t205;
  double t219;
  double t254;
  double t255;
  double t273;
  double t276;
  double t280;
  double t281;
  double t282;
  double t237;
  double t227;
  t199 = Sin(var1[0]);
  t188 = Cos(var1[0]);
  t205 = Cos(var1[1]);
  t219 = Sin(var1[1]);
  t254 = -1.*t188;
  t255 = 1. + t254;
  t273 = -1.*t205;
  t276 = 1. + t273;
  t280 = 0.331012*t276;
  t281 = -0.90524*t219;
  t282 = 0. + t280 + t281;
  t237 = 0. + t205;
  t227 = 0. + t219;

  p_output1(0)=t188;
  p_output1(1)=t199;
  p_output1(2)=0.;
  p_output1(3)=0.;
  p_output1(4)=-1.*t199*t205;
  p_output1(5)=t188*t205;
  p_output1(6)=t227;
  p_output1(7)=0.;
  p_output1(8)=t199*t219;
  p_output1(9)=-1.*t188*t219;
  p_output1(10)=t237;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.066675*t188 + 0.261012*t199 - 0.331012*t199*t205 - 0.90524*t199*t219 - 0.066675*t255 - 1.*t199*t282;
  p_output1(13)=0. + 0.331012*t188*t205 + 0.90524*t188*t219 + 0.261012*t255 + t188*t282;
  p_output1(14)=0. - 0.331012*t219 + 0.331012*t227 - 0.90524*t237 - 0.90524*t276;
  p_output1(15)=1.;
}


       
void H_lHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
