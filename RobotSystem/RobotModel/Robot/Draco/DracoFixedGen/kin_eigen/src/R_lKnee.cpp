/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:01 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lKnee.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t810;
  double t853;
  double t856;
  double t808;
  double t860;
  double t700;
  double t929;
  double t977;
  double t978;
  double t891;
  double t894;
  double t925;
  double t817;
  double t867;
  double t877;
  double t959;
  double t961;
  double t963;
  double t947;
  double t952;
  double t953;
  double t995;
  double t996;
  double t979;
  double t982;
  t810 = Cos(var1[2]);
  t853 = Sin(var1[0]);
  t856 = Sin(var1[1]);
  t808 = Cos(var1[0]);
  t860 = Sin(var1[2]);
  t700 = Cos(var1[3]);
  t929 = Sin(var1[3]);
  t977 = Cos(var1[1]);
  t978 = 0. + t977;
  t891 = t810*t853*t856;
  t894 = t808*t860;
  t925 = t891 + t894;
  t817 = t808*t810;
  t867 = -1.*t853*t856*t860;
  t877 = t817 + t867;
  t959 = -1.*t808*t810*t856;
  t961 = t853*t860;
  t963 = t959 + t961;
  t947 = t810*t853;
  t952 = t808*t856*t860;
  t953 = t947 + t952;
  t995 = t978*t810;
  t996 = 0. + t995;
  t979 = -1.*t978*t860;
  t982 = 0. + t979;

  p_output1(0)=t700*t877 - 1.*t925*t929;
  p_output1(1)=t700*t953 - 1.*t929*t963;
  p_output1(2)=t700*t982 - 1.*t929*t996;
  p_output1(3)=-1.*t853*t977;
  p_output1(4)=t808*t977;
  p_output1(5)=0. + t856;
  p_output1(6)=t700*t925 + t877*t929;
  p_output1(7)=t929*t953 + t700*t963;
  p_output1(8)=t929*t982 + t700*t996;
}


       
void R_lKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
