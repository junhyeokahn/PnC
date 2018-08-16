/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:56 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipRoll.h"

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
  double t235;
  double t231;
  double t265;
  double t270;
  t235 = Sin(var1[0]);
  t231 = Cos(var1[0]);
  t265 = Cos(var1[1]);
  t270 = Sin(var1[1]);

  p_output1(0)=t231;
  p_output1(1)=t235;
  p_output1(2)=0.;
  p_output1(3)=-1.*t235*t265;
  p_output1(4)=t231*t265;
  p_output1(5)=0. + t270;
  p_output1(6)=t235*t270;
  p_output1(7)=-1.*t231*t270;
  p_output1(8)=0. + t265;
}


       
void R_lHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
