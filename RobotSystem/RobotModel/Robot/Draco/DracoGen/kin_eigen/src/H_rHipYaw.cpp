/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:22 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipYaw.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t970;
  double t480;
  double t1104;
  double t2171;
  double t1922;
  double t2160;
  double t2185;
  double t2186;
  double t2172;
  double t2188;
  double t2196;
  double t2212;
  double t2219;
  double t2315;
  double t3042;
  double t3068;
  double t3113;
  double t3229;
  double t3253;
  double t2618;
  double t2632;
  double t2766;
  double t1108;
  double t2201;
  double t2202;
  double t3259;
  double t3299;
  double t3300;
  double t3119;
  double t3125;
  double t3193;
  double t3393;
  double t3396;
  double t3500;
  double t2851;
  double t2852;
  double t2893;
  double t2205;
  double t2369;
  double t2495;
  double t2897;
  double t2937;
  double t3021;
  double t2540;
  double t2547;
  double t2591;
  t970 = Cos(var1[3]);
  t480 = Cos(var1[11]);
  t1104 = Cos(var1[4]);
  t2171 = Sin(var1[3]);
  t1922 = Sin(var1[11]);
  t2160 = Cos(var1[5]);
  t2185 = Sin(var1[4]);
  t2186 = Sin(var1[5]);
  t2172 = -1.*t2160*t2171;
  t2188 = t970*t2185*t2186;
  t2196 = t2172 + t2188;
  t2212 = t970*t2160;
  t2219 = t2171*t2185*t2186;
  t2315 = t2212 + t2219;
  t3042 = t970*t2160*t2185;
  t3068 = t2171*t2186;
  t3113 = t3042 + t3068;
  t3229 = -1.*t480;
  t3253 = 1. + t3229;
  t2618 = -1.*t970*t1104*t1922;
  t2632 = t480*t2196;
  t2766 = t2618 + t2632;
  t1108 = t480*t970*t1104;
  t2201 = t1922*t2196;
  t2202 = t1108 + t2201;
  t3259 = -0.0222*t3253;
  t3299 = -0.087*t1922;
  t3300 = 0. + t3259 + t3299;
  t3119 = t2160*t2171*t2185;
  t3125 = -1.*t970*t2186;
  t3193 = t3119 + t3125;
  t3393 = -0.087*t3253;
  t3396 = 0.0222*t1922;
  t3500 = 0. + t3393 + t3396;
  t2851 = -1.*t1104*t1922*t2171;
  t2852 = t480*t2315;
  t2893 = t2851 + t2852;
  t2205 = t480*t1104*t2171;
  t2369 = t1922*t2315;
  t2495 = t2205 + t2369;
  t2897 = t1922*t2185;
  t2937 = t480*t1104*t2186;
  t3021 = t2897 + t2937;
  t2540 = -1.*t480*t2185;
  t2547 = t1104*t1922*t2186;
  t2591 = t2540 + t2547;

  p_output1(0)=t2202;
  p_output1(1)=t2495;
  p_output1(2)=t2591;
  p_output1(3)=0.;
  p_output1(4)=t2766;
  p_output1(5)=t2893;
  p_output1(6)=t3021;
  p_output1(7)=0.;
  p_output1(8)=t3113;
  p_output1(9)=t3193;
  p_output1(10)=t1104*t2160;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t2202 - 0.087*t2766 - 0.2951*t3113 + t2196*t3500 + t1104*t3300*t970 + var1(0);
  p_output1(13)=0. - 0.0222*t2495 - 0.087*t2893 - 0.2951*t3193 + t1104*t2171*t3300 + t2315*t3500 + var1(1);
  p_output1(14)=0. - 0.2951*t1104*t2160 - 0.0222*t2591 - 0.087*t3021 - 1.*t2185*t3300 + t1104*t2186*t3500 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
