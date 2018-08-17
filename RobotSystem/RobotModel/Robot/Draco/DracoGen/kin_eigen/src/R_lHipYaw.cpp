/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:41 GMT-05:00
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
  double t110;
  double t1975;
  double t2172;
  double t2381;
  double t2351;
  double t2846;
  double t2897;
  double t3720;
  double t2773;
  double t3221;
  double t3586;
  double t4337;
  double t4344;
  double t4349;
  t110 = Cos(var1[3]);
  t1975 = Cos(var1[4]);
  t2172 = Cos(var1[6]);
  t2381 = Sin(var1[3]);
  t2351 = Cos(var1[5]);
  t2846 = Sin(var1[4]);
  t2897 = Sin(var1[5]);
  t3720 = Sin(var1[6]);
  t2773 = -1.*t2351*t2381;
  t3221 = t110*t2846*t2897;
  t3586 = t2773 + t3221;
  t4337 = t110*t2351;
  t4344 = t2381*t2846*t2897;
  t4349 = t4337 + t4344;

  p_output1(0)=t110*t1975*t2172 + t3586*t3720;
  p_output1(1)=t1975*t2172*t2381 + t3720*t4349;
  p_output1(2)=-1.*t2172*t2846 + t1975*t2897*t3720;
  p_output1(3)=t2172*t3586 - 1.*t110*t1975*t3720;
  p_output1(4)=-1.*t1975*t2381*t3720 + t2172*t4349;
  p_output1(5)=t1975*t2172*t2897 + t2846*t3720;
  p_output1(6)=t110*t2351*t2846 + t2381*t2897;
  p_output1(7)=t2351*t2381*t2846 - 1.*t110*t2897;
  p_output1(8)=t1975*t2351;
}


       
void R_lHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
