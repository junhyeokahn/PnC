/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:07 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1610;
  double t1688;
  double t1739;
  double t1766;
  double t1710;
  double t1717;
  double t1727;
  double t1742;
  double t1755;
  double t1801;
  double t1811;
  double t1812;
  t1610 = Cos(var1[5]);
  t1688 = Cos(var1[6]);
  t1739 = Sin(var1[6]);
  t1766 = Sin(var1[5]);
  t1710 = -1.*t1688;
  t1717 = 1. + t1710;
  t1727 = -0.330988*t1717;
  t1742 = -0.90524*t1739;
  t1755 = 0. + t1727 + t1742;
  t1801 = -0.90524*t1688;
  t1811 = -0.330988*t1739;
  t1812 = t1801 + t1811;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=0. - 0.260988*t1610 + 0.330988*t1610*t1688 - 0.90524*t1610*t1739 - 1.*t1610*t1755;
  p_output1(16)=-0.260988*t1766 + 0.330988*t1688*t1766 - 0.90524*t1739*t1766 - 1.*t1755*t1766;
  p_output1(17)=0;
  p_output1(18)=-0.90524*t1688*t1766 - 0.330988*t1739*t1766 - 1.*t1766*t1812;
  p_output1(19)=0.90524*t1610*t1688 + 0.330988*t1610*t1739 + t1610*t1812;
  p_output1(20)=0.;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
}


       
void Jp_rHipRoll(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
