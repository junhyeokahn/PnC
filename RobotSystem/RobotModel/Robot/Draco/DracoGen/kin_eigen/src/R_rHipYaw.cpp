/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:45 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipYaw.h"

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
  double t3351;
  double t1747;
  double t3466;
  double t4226;
  double t3906;
  double t3924;
  double t4441;
  double t4452;
  double t4339;
  double t4488;
  double t4544;
  double t4743;
  double t4798;
  double t4847;
  t3351 = Cos(var1[3]);
  t1747 = Cos(var1[11]);
  t3466 = Cos(var1[4]);
  t4226 = Sin(var1[3]);
  t3906 = Sin(var1[11]);
  t3924 = Cos(var1[5]);
  t4441 = Sin(var1[4]);
  t4452 = Sin(var1[5]);
  t4339 = -1.*t3924*t4226;
  t4488 = t3351*t4441*t4452;
  t4544 = t4339 + t4488;
  t4743 = t3351*t3924;
  t4798 = t4226*t4441*t4452;
  t4847 = t4743 + t4798;

  p_output1(0)=t1747*t3351*t3466 + t3906*t4544;
  p_output1(1)=t1747*t3466*t4226 + t3906*t4847;
  p_output1(2)=-1.*t1747*t4441 + t3466*t3906*t4452;
  p_output1(3)=-1.*t3351*t3466*t3906 + t1747*t4544;
  p_output1(4)=-1.*t3466*t3906*t4226 + t1747*t4847;
  p_output1(5)=t3906*t4441 + t1747*t3466*t4452;
  p_output1(6)=t3351*t3924*t4441 + t4226*t4452;
  p_output1(7)=t3924*t4226*t4441 - 1.*t3351*t4452;
  p_output1(8)=t3466*t3924;
}


       
void R_rHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
