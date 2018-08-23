/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:24 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipRoll.h"

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
  double t903;
  double t755;
  double t1187;
  double t2270;
  double t1965;
  double t1968;
  double t2506;
  double t2508;
  double t2324;
  double t2542;
  double t2577;
  double t3871;
  double t3925;
  double t3537;
  double t3765;
  double t3834;
  double t3907;
  double t3908;
  double t3910;
  double t3953;
  double t3960;
  double t4003;
  double t4092;
  double t4103;
  double t4132;
  double t4172;
  double t4210;
  double t4211;
  double t4286;
  double t4295;
  double t4312;
  t903 = Cos(var1[3]);
  t755 = Cos(var1[11]);
  t1187 = Cos(var1[4]);
  t2270 = Sin(var1[3]);
  t1965 = Sin(var1[11]);
  t1968 = Cos(var1[5]);
  t2506 = Sin(var1[4]);
  t2508 = Sin(var1[5]);
  t2324 = -1.*t1968*t2270;
  t2542 = t903*t2506*t2508;
  t2577 = t2324 + t2542;
  t3871 = Sin(var1[12]);
  t3925 = Cos(var1[12]);
  t3537 = t903*t1968;
  t3765 = t2270*t2506*t2508;
  t3834 = t3537 + t3765;
  t3907 = t903*t1968*t2506;
  t3908 = t2270*t2508;
  t3910 = t3907 + t3908;
  t3953 = -1.*t903*t1187*t1965;
  t3960 = t755*t2577;
  t4003 = t3953 + t3960;
  t4092 = t1968*t2270*t2506;
  t4103 = -1.*t903*t2508;
  t4132 = t4092 + t4103;
  t4172 = -1.*t1187*t1965*t2270;
  t4210 = t755*t3834;
  t4211 = t4172 + t4210;
  t4286 = t1965*t2506;
  t4295 = t755*t1187*t2508;
  t4312 = t4286 + t4295;

  p_output1(0)=t1965*t2577 + t1187*t755*t903;
  p_output1(1)=t1965*t3834 + t1187*t2270*t755;
  p_output1(2)=t1187*t1965*t2508 - 1.*t2506*t755;
  p_output1(3)=t3871*t3910 + t3925*t4003;
  p_output1(4)=t3871*t4132 + t3925*t4211;
  p_output1(5)=t1187*t1968*t3871 + t3925*t4312;
  p_output1(6)=t3910*t3925 - 1.*t3871*t4003;
  p_output1(7)=t3925*t4132 - 1.*t3871*t4211;
  p_output1(8)=t1187*t1968*t3925 - 1.*t3871*t4312;
}


       
void R_rHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
