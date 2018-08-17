/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:47 GMT-05:00
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
  double t1121;
  double t235;
  double t1181;
  double t1207;
  double t1187;
  double t1206;
  double t1217;
  double t1221;
  double t1215;
  double t1253;
  double t1478;
  double t3385;
  double t3477;
  double t2149;
  double t2505;
  double t2782;
  double t3393;
  double t3427;
  double t3437;
  double t3600;
  double t3610;
  double t3696;
  double t3829;
  double t3867;
  double t3881;
  double t4037;
  double t4170;
  double t4174;
  double t4214;
  double t4222;
  double t4246;
  t1121 = Cos(var1[3]);
  t235 = Cos(var1[11]);
  t1181 = Cos(var1[4]);
  t1207 = Sin(var1[3]);
  t1187 = Sin(var1[11]);
  t1206 = Cos(var1[5]);
  t1217 = Sin(var1[4]);
  t1221 = Sin(var1[5]);
  t1215 = -1.*t1206*t1207;
  t1253 = t1121*t1217*t1221;
  t1478 = t1215 + t1253;
  t3385 = Sin(var1[12]);
  t3477 = Cos(var1[12]);
  t2149 = t1121*t1206;
  t2505 = t1207*t1217*t1221;
  t2782 = t2149 + t2505;
  t3393 = t1121*t1206*t1217;
  t3427 = t1207*t1221;
  t3437 = t3393 + t3427;
  t3600 = -1.*t1121*t1181*t1187;
  t3610 = t235*t1478;
  t3696 = t3600 + t3610;
  t3829 = t1206*t1207*t1217;
  t3867 = -1.*t1121*t1221;
  t3881 = t3829 + t3867;
  t4037 = -1.*t1181*t1187*t1207;
  t4170 = t235*t2782;
  t4174 = t4037 + t4170;
  t4214 = t1187*t1217;
  t4222 = t235*t1181*t1221;
  t4246 = t4214 + t4222;

  p_output1(0)=t1187*t1478 + t1121*t1181*t235;
  p_output1(1)=t1181*t1207*t235 + t1187*t2782;
  p_output1(2)=t1181*t1187*t1221 - 1.*t1217*t235;
  p_output1(3)=t3385*t3437 + t3477*t3696;
  p_output1(4)=t3385*t3881 + t3477*t4174;
  p_output1(5)=t1181*t1206*t3385 + t3477*t4246;
  p_output1(6)=t3437*t3477 - 1.*t3385*t3696;
  p_output1(7)=t3477*t3881 - 1.*t3385*t4174;
  p_output1(8)=t1181*t1206*t3477 - 1.*t3385*t4246;
}


       
void R_rHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
