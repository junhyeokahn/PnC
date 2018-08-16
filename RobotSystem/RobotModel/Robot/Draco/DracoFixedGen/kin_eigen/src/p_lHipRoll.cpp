/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t135;
  double t153;
  double t155;
  double t166;
  double t136;
  double t139;
  double t161;
  double t163;
  double t164;
  double t169;
  double t170;
  t135 = Cos(var1[0]);
  t153 = Sin(var1[0]);
  t155 = Cos(var1[1]);
  t166 = Sin(var1[1]);
  t136 = -1.*t135;
  t139 = 1. + t136;
  t161 = -1.*t155;
  t163 = 1. + t161;
  t164 = 0.331012*t163;
  t169 = -0.90524*t166;
  t170 = 0. + t164 + t169;

  p_output1(0)=0. - 0.066675*t135 - 0.066675*t139 + 0.261012*t153 - 0.331012*t153*t155 - 0.90524*t153*t166 - 1.*t153*t170;
  p_output1(1)=0. + 0.261012*t139 + 0.331012*t135*t155 + 0.90524*t135*t166 + t135*t170;
  p_output1(2)=0. - 0.90524*(0. + t155) - 0.90524*t163 - 0.331012*t166 + 0.331012*(0. + t166);
}


       
void p_lHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
