/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:33 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t4006;
  double t5114;
  double t5116;
  double t5128;
  double t5147;
  double t5163;
  double t270;
  double t1069;
  double t3198;
  double t4784;
  double t5010;
  double t5019;
  double t5100;
  double t5226;
  double t5229;
  double t5232;
  double t5241;
  double t5202;
  double t5218;
  double t5221;
  double t5270;
  double t5272;
  double t5274;
  double t5134;
  double t5155;
  double t5157;
  double t5166;
  double t5170;
  double t5179;
  double t5381;
  double t5405;
  double t5407;
  double t5238;
  double t5257;
  double t5262;
  double t5277;
  double t5279;
  double t5284;
  double t5452;
  double t5454;
  double t5456;
  double t5459;
  double t5462;
  double t5478;
  double t5612;
  double t5618;
  double t5622;
  t4006 = Cos(var1[3]);
  t5114 = Cos(var1[6]);
  t5116 = -1.*t5114;
  t5128 = 1. + t5116;
  t5147 = Sin(var1[6]);
  t5163 = Cos(var1[4]);
  t270 = Cos(var1[5]);
  t1069 = Sin(var1[3]);
  t3198 = -1.*t270*t1069;
  t4784 = Sin(var1[4]);
  t5010 = Sin(var1[5]);
  t5019 = t4006*t4784*t5010;
  t5100 = t3198 + t5019;
  t5226 = Cos(var1[7]);
  t5229 = -1.*t5226;
  t5232 = 1. + t5229;
  t5241 = Sin(var1[7]);
  t5202 = t5114*t5100;
  t5218 = -1.*t4006*t5163*t5147;
  t5221 = t5202 + t5218;
  t5270 = t4006*t270*t4784;
  t5272 = t1069*t5010;
  t5274 = t5270 + t5272;
  t5134 = 0.087004*t5128;
  t5155 = 0.022225*t5147;
  t5157 = 0. + t5134 + t5155;
  t5166 = -0.022225*t5128;
  t5170 = 0.087004*t5147;
  t5179 = 0. + t5166 + t5170;
  t5381 = t4006*t270;
  t5405 = t1069*t4784*t5010;
  t5407 = t5381 + t5405;
  t5238 = 0.157004*t5232;
  t5257 = -0.31508*t5241;
  t5262 = 0. + t5238 + t5257;
  t5277 = -0.31508*t5232;
  t5279 = -0.157004*t5241;
  t5284 = 0. + t5277 + t5279;
  t5452 = t5114*t5407;
  t5454 = -1.*t5163*t1069*t5147;
  t5456 = t5452 + t5454;
  t5459 = t270*t1069*t4784;
  t5462 = -1.*t4006*t5010;
  t5478 = t5459 + t5462;
  t5612 = t5163*t5114*t5010;
  t5618 = t4784*t5147;
  t5622 = t5612 + t5618;

  p_output1(0)=0. + t5100*t5157 - 0.022225*(t5100*t5147 + t4006*t5114*t5163) + t4006*t5163*t5179 + t5221*t5262 - 0.31508*(-1.*t5221*t5241 + t5226*t5274) + 0.157004*(t5221*t5226 + t5241*t5274) + t5274*t5284 + var1(0);
  p_output1(1)=0. + t1069*t5163*t5179 + t5157*t5407 - 0.022225*(t1069*t5114*t5163 + t5147*t5407) + t5262*t5456 + t5284*t5478 - 0.31508*(-1.*t5241*t5456 + t5226*t5478) + 0.157004*(t5226*t5456 + t5241*t5478) + var1(1);
  p_output1(2)=0. + t5010*t5157*t5163 - 0.022225*(-1.*t4784*t5114 + t5010*t5147*t5163) - 1.*t4784*t5179 + t270*t5163*t5284 + t5262*t5622 + 0.157004*(t270*t5163*t5241 + t5226*t5622) - 0.31508*(t270*t5163*t5226 - 1.*t5241*t5622) + var1(2);
}


       
void p_lHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
