/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:07 GMT-05:00
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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1673;
  double t842;
  double t1727;
  double t1758;
  double t1801;
  double t1811;
  double t1828;
  double t1835;
  double t1849;
  double t1851;
  double t1854;
  double t1783;
  double t1771;
  t1673 = Sin(var1[5]);
  t842 = Cos(var1[5]);
  t1727 = Cos(var1[6]);
  t1758 = Sin(var1[6]);
  t1801 = -1.*t842;
  t1811 = 1. + t1801;
  t1828 = -1.*t1727;
  t1835 = 1. + t1828;
  t1849 = -0.330988*t1835;
  t1851 = -0.90524*t1758;
  t1854 = 0. + t1849 + t1851;
  t1783 = 0. + t1727;
  t1771 = 0. + t1758;

  p_output1(0)=t842;
  p_output1(1)=t1673;
  p_output1(2)=0.;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1673*t1727;
  p_output1(5)=t1727*t842;
  p_output1(6)=t1771;
  p_output1(7)=0.;
  p_output1(8)=t1673*t1758;
  p_output1(9)=-1.*t1758*t842;
  p_output1(10)=t1783;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.260988*t1673 + 0.330988*t1673*t1727 - 0.90524*t1673*t1758 - 0.066675*t1811 - 1.*t1673*t1854 - 0.066675*t842;
  p_output1(13)=0. - 0.260988*t1811 - 0.330988*t1727*t842 + 0.90524*t1758*t842 + t1854*t842;
  p_output1(14)=0. + 0.330988*t1758 - 0.330988*t1771 - 0.90524*t1783 - 0.90524*t1835;
  p_output1(15)=1.;
}


       
void H_rHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
