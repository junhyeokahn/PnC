/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:15 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipYaw.h"

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
  double t822;
  double t1008;
  double t1050;
  double t2951;
  double t2455;
  double t3390;
  double t3498;
  double t3814;
  double t3215;
  double t3586;
  double t3610;
  double t4215;
  double t4237;
  double t4241;
  double t4539;
  double t4542;
  double t4556;
  double t4584;
  double t4589;
  double t4313;
  double t4366;
  double t4440;
  double t2453;
  double t4073;
  double t4137;
  double t4558;
  double t4572;
  double t4573;
  double t4593;
  double t4595;
  double t4596;
  double t4601;
  double t4610;
  double t4625;
  double t4490;
  double t4511;
  double t4517;
  double t4194;
  double t4269;
  double t4277;
  double t4529;
  double t4531;
  double t4537;
  double t4293;
  double t4310;
  double t4311;
  t822 = Cos(var1[3]);
  t1008 = Cos(var1[4]);
  t1050 = Cos(var1[6]);
  t2951 = Sin(var1[3]);
  t2455 = Cos(var1[5]);
  t3390 = Sin(var1[4]);
  t3498 = Sin(var1[5]);
  t3814 = Sin(var1[6]);
  t3215 = -1.*t2455*t2951;
  t3586 = t822*t3390*t3498;
  t3610 = t3215 + t3586;
  t4215 = t822*t2455;
  t4237 = t2951*t3390*t3498;
  t4241 = t4215 + t4237;
  t4539 = t822*t2455*t3390;
  t4542 = t2951*t3498;
  t4556 = t4539 + t4542;
  t4584 = -1.*t1050;
  t4589 = 1. + t4584;
  t4313 = t1050*t3610;
  t4366 = -1.*t822*t1008*t3814;
  t4440 = t4313 + t4366;
  t2453 = t822*t1008*t1050;
  t4073 = t3610*t3814;
  t4137 = t2453 + t4073;
  t4558 = t2455*t2951*t3390;
  t4572 = -1.*t822*t3498;
  t4573 = t4558 + t4572;
  t4593 = 0.087*t4589;
  t4595 = 0.0222*t3814;
  t4596 = 0. + t4593 + t4595;
  t4601 = -0.0222*t4589;
  t4610 = 0.087*t3814;
  t4625 = 0. + t4601 + t4610;
  t4490 = t1050*t4241;
  t4511 = -1.*t1008*t2951*t3814;
  t4517 = t4490 + t4511;
  t4194 = t1008*t1050*t2951;
  t4269 = t4241*t3814;
  t4277 = t4194 + t4269;
  t4529 = t1008*t1050*t3498;
  t4531 = t3390*t3814;
  t4537 = t4529 + t4531;
  t4293 = -1.*t1050*t3390;
  t4310 = t1008*t3498*t3814;
  t4311 = t4293 + t4310;

  p_output1(0)=t4137;
  p_output1(1)=t4277;
  p_output1(2)=t4311;
  p_output1(3)=0.;
  p_output1(4)=t4440;
  p_output1(5)=t4517;
  p_output1(6)=t4537;
  p_output1(7)=0.;
  p_output1(8)=t4556;
  p_output1(9)=t4573;
  p_output1(10)=t1008*t2455;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t4137 + 0.087*t4440 - 0.2951*t4556 + t3610*t4596 + t1008*t4625*t822 + var1(0);
  p_output1(13)=0. - 0.0222*t4277 + 0.087*t4517 - 0.2951*t4573 + t4241*t4596 + t1008*t2951*t4625 + var1(1);
  p_output1(14)=0. - 0.2951*t1008*t2455 - 0.0222*t4311 + 0.087*t4537 + t1008*t3498*t4596 - 1.*t3390*t4625 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
