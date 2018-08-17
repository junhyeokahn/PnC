/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:44 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t11;
  double t385;
  double t502;
  double t540;
  double t520;
  double t958;
  double t1081;
  double t3262;
  double t828;
  double t2430;
  double t3222;
  double t5044;
  double t3911;
  double t4015;
  double t4183;
  double t5286;
  double t5263;
  double t5273;
  double t5281;
  double t5182;
  double t5194;
  double t5232;
  double t5345;
  double t5347;
  double t5348;
  double t5307;
  double t5320;
  double t5325;
  double t5355;
  double t5357;
  double t5363;
  t11 = Cos(var1[3]);
  t385 = Cos(var1[4]);
  t502 = Cos(var1[6]);
  t540 = Sin(var1[3]);
  t520 = Cos(var1[5]);
  t958 = Sin(var1[4]);
  t1081 = Sin(var1[5]);
  t3262 = Sin(var1[6]);
  t828 = -1.*t520*t540;
  t2430 = t11*t958*t1081;
  t3222 = t828 + t2430;
  t5044 = Cos(var1[7]);
  t3911 = t11*t520;
  t4015 = t540*t958*t1081;
  t4183 = t3911 + t4015;
  t5286 = Sin(var1[7]);
  t5263 = t11*t520*t958;
  t5273 = t540*t1081;
  t5281 = t5263 + t5273;
  t5182 = t502*t3222;
  t5194 = -1.*t11*t385*t3262;
  t5232 = t5182 + t5194;
  t5345 = t520*t540*t958;
  t5347 = -1.*t11*t1081;
  t5348 = t5345 + t5347;
  t5307 = t502*t4183;
  t5320 = -1.*t385*t540*t3262;
  t5325 = t5307 + t5320;
  t5355 = t385*t502*t1081;
  t5357 = t958*t3262;
  t5363 = t5355 + t5357;

  p_output1(0)=t3222*t3262 + t11*t385*t502;
  p_output1(1)=t3262*t4183 + t385*t502*t540;
  p_output1(2)=t1081*t3262*t385 - 1.*t502*t958;
  p_output1(3)=t5044*t5232 + t5281*t5286;
  p_output1(4)=t5044*t5325 + t5286*t5348;
  p_output1(5)=t385*t520*t5286 + t5044*t5363;
  p_output1(6)=t5044*t5281 - 1.*t5232*t5286;
  p_output1(7)=-1.*t5286*t5325 + t5044*t5348;
  p_output1(8)=t385*t5044*t520 - 1.*t5286*t5363;
}


       
void R_lHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
