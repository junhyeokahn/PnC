/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:32 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipYaw.h"

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
  double t350;
  double t579;
  double t1023;
  double t3816;
  double t3232;
  double t4735;
  double t4758;
  double t5000;
  double t4518;
  double t4848;
  double t4855;
  double t5078;
  double t5080;
  double t5093;
  t350 = Cos(var1[3]);
  t579 = Cos(var1[4]);
  t1023 = Cos(var1[6]);
  t3816 = Sin(var1[3]);
  t3232 = Cos(var1[5]);
  t4735 = Sin(var1[4]);
  t4758 = Sin(var1[5]);
  t5000 = Sin(var1[6]);
  t4518 = -1.*t3232*t3816;
  t4848 = t350*t4735*t4758;
  t4855 = t4518 + t4848;
  t5078 = t350*t3232;
  t5080 = t3816*t4735*t4758;
  t5093 = t5078 + t5080;

  p_output1(0)=t4855*t5000 + t1023*t350*t579;
  p_output1(1)=t5000*t5093 + t1023*t3816*t579;
  p_output1(2)=-1.*t1023*t4735 + t4758*t5000*t579;
  p_output1(3)=t1023*t4855 - 1.*t350*t5000*t579;
  p_output1(4)=t1023*t5093 - 1.*t3816*t5000*t579;
  p_output1(5)=t4735*t5000 + t1023*t4758*t579;
  p_output1(6)=t3232*t350*t4735 + t3816*t4758;
  p_output1(7)=t3232*t3816*t4735 - 1.*t350*t4758;
  p_output1(8)=t3232*t579;
}


       
void R_lHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
