/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:09 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipPitch.h"

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
  double t1887;
  double t1895;
  double t1923;
  double t789;
  double t1936;
  double t1937;
  double t1942;
  double t1947;
  double t1849;
  double t1865;
  double t1916;
  double t1921;
  double t1922;
  double t1925;
  double t1931;
  double t1946;
  double t1950;
  double t1952;
  double t1956;
  double t1963;
  double t1966;
  double t2046;
  t1887 = Sin(var1[5]);
  t1895 = Cos(var1[6]);
  t1923 = Sin(var1[6]);
  t789 = Cos(var1[5]);
  t1936 = Cos(var1[7]);
  t1937 = -1.*t1936;
  t1942 = 1. + t1937;
  t1947 = Sin(var1[7]);
  t1849 = -1.*t789;
  t1865 = 1. + t1849;
  t1916 = -1.*t1895;
  t1921 = 1. + t1916;
  t1922 = -0.330988*t1921;
  t1925 = -0.90524*t1923;
  t1931 = 0. + t1922 + t1925;
  t1946 = -0.97024*t1942;
  t1950 = -0.066675*t1947;
  t1952 = 0. + t1946 + t1950;
  t1956 = -0.066675*t1942;
  t1963 = 0.97024*t1947;
  t1966 = 0. + t1956 + t1963;
  t2046 = 0. + t1895;

  p_output1(0)=0. - 0.066675*t1865 - 0.260988*t1887 + 0.340988*t1887*t1895 - 1.*t1887*t1931 + t1887*t1923*t1952 + t1966*t789 - 0.066675*(-1.*t1887*t1923*t1947 + t1936*t789) - 0.97024*(t1887*t1923*t1936 + t1947*t789);
  p_output1(1)=0. - 0.260988*t1865 + 0.066675*t1887 + t1887*t1966 - 0.340988*t1895*t789 + t1931*t789 - 1.*t1923*t1952*t789 - 0.97024*(t1887*t1947 - 1.*t1923*t1936*t789) - 0.066675*(t1887*t1936 + t1923*t1947*t789);
  p_output1(2)=0. - 0.90524*t1921 + 0.330988*t1923 - 0.340988*(0. + t1923) + t1952*t2046 - 0.97024*(0. + t1936*t2046) - 0.066675*(0. - 1.*t1947*t2046);
}


       
void p_rHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
