/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:23 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipRoll.h"

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
  double t158;
  double t387;
  double t703;
  double t723;
  double t2248;
  double t3387;
  double t3577;
  double t3396;
  double t3646;
  double t2732;
  double t2737;
  double t3154;
  double t3263;
  double t333;
  double t3893;
  double t3907;
  double t3908;
  double t3537;
  double t3683;
  double t3762;
  double t3975;
  double t4003;
  double t4051;
  double t757;
  double t2496;
  double t2515;
  double t3162;
  double t3280;
  double t3299;
  double t3809;
  double t3834;
  double t3864;
  double t3921;
  double t3925;
  double t3953;
  double t4442;
  double t4444;
  double t4450;
  double t4407;
  double t4411;
  double t4415;
  double t4460;
  double t4464;
  double t4466;
  double t4868;
  double t4871;
  double t4872;
  t158 = Cos(var1[3]);
  t387 = Cos(var1[11]);
  t703 = -1.*t387;
  t723 = 1. + t703;
  t2248 = Sin(var1[11]);
  t3387 = Cos(var1[5]);
  t3577 = Sin(var1[3]);
  t3396 = Sin(var1[4]);
  t3646 = Sin(var1[5]);
  t2732 = Cos(var1[12]);
  t2737 = -1.*t2732;
  t3154 = 1. + t2737;
  t3263 = Sin(var1[12]);
  t333 = Cos(var1[4]);
  t3893 = -1.*t3387*t3577;
  t3907 = t158*t3396*t3646;
  t3908 = t3893 + t3907;
  t3537 = t158*t3387*t3396;
  t3683 = t3577*t3646;
  t3762 = t3537 + t3683;
  t3975 = -1.*t158*t333*t2248;
  t4003 = t387*t3908;
  t4051 = t3975 + t4003;
  t757 = -0.0222*t723;
  t2496 = -0.087*t2248;
  t2515 = 0. + t757 + t2496;
  t3162 = -0.3151*t3154;
  t3280 = 0.157*t3263;
  t3299 = 0. + t3162 + t3280;
  t3809 = -0.087*t723;
  t3834 = 0.0222*t2248;
  t3864 = 0. + t3809 + t3834;
  t3921 = -0.157*t3154;
  t3925 = -0.3151*t3263;
  t3953 = 0. + t3921 + t3925;
  t4442 = t158*t3387;
  t4444 = t3577*t3396*t3646;
  t4450 = t4442 + t4444;
  t4407 = t3387*t3577*t3396;
  t4411 = -1.*t158*t3646;
  t4415 = t4407 + t4411;
  t4460 = -1.*t333*t2248*t3577;
  t4464 = t387*t4450;
  t4466 = t4460 + t4464;
  t4868 = t2248*t3396;
  t4871 = t387*t333*t3646;
  t4872 = t4868 + t4871;

  p_output1(0)=0. + t158*t2515*t333 + t3299*t3762 + t3864*t3908 - 0.0222*(t158*t333*t387 + t2248*t3908) + t3953*t4051 - 0.157*(t3263*t3762 + t2732*t4051) - 0.3151*(t2732*t3762 - 1.*t3263*t4051) + var1(0);
  p_output1(1)=0. + t2515*t333*t3577 + t3299*t4415 + t3864*t4450 - 0.0222*(t333*t3577*t387 + t2248*t4450) + t3953*t4466 - 0.157*(t3263*t4415 + t2732*t4466) - 0.3151*(t2732*t4415 - 1.*t3263*t4466) + var1(1);
  p_output1(2)=0. + t3299*t333*t3387 - 1.*t2515*t3396 + t333*t3646*t3864 - 0.0222*(t2248*t333*t3646 - 1.*t3396*t387) + t3953*t4872 - 0.157*(t3263*t333*t3387 + t2732*t4872) - 0.3151*(t2732*t333*t3387 - 1.*t3263*t4872) + var1(2);
}


       
void p_rHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
