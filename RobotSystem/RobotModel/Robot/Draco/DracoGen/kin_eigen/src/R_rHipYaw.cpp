/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:54 GMT-05:00
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
  double t356;
  double t203;
  double t425;
  double t3890;
  double t2386;
  double t2590;
  double t3951;
  double t4114;
  double t3943;
  double t4153;
  double t4192;
  double t4577;
  double t4599;
  double t4682;
  t356 = Cos(var1[3]);
  t203 = Cos(var1[11]);
  t425 = Cos(var1[4]);
  t3890 = Sin(var1[3]);
  t2386 = Sin(var1[11]);
  t2590 = Cos(var1[5]);
  t3951 = Sin(var1[4]);
  t4114 = Sin(var1[5]);
  t3943 = -1.*t2590*t3890;
  t4153 = t356*t3951*t4114;
  t4192 = t3943 + t4153;
  t4577 = t356*t2590;
  t4599 = t3890*t3951*t4114;
  t4682 = t4577 + t4599;

  p_output1(0)=t2386*t4192 + t203*t356*t425;
  p_output1(1)=t203*t3890*t425 + t2386*t4682;
  p_output1(2)=-1.*t203*t3951 + t2386*t4114*t425;
  p_output1(3)=t203*t4192 - 1.*t2386*t356*t425;
  p_output1(4)=-1.*t2386*t3890*t425 + t203*t4682;
  p_output1(5)=t2386*t3951 + t203*t4114*t425;
  p_output1(6)=t2590*t356*t3951 + t3890*t4114;
  p_output1(7)=t2590*t3890*t3951 - 1.*t356*t4114;
  p_output1(8)=t2590*t425;
}


       
void R_rHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
