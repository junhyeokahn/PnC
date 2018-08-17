/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:01 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rKnee.h"

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
  double t1199;
  double t2469;
  double t2943;
  double t2522;
  double t2970;
  double t1829;
  double t2416;
  double t538;
  double t2530;
  double t2977;
  double t3359;
  double t3634;
  double t2287;
  double t3424;
  double t3431;
  double t363;
  double t3653;
  double t3672;
  double t3684;
  double t3725;
  double t3742;
  double t3797;
  double t3810;
  double t3825;
  double t3830;
  double t3860;
  double t3881;
  double t129;
  double t4156;
  double t4163;
  double t4172;
  double t4062;
  double t4152;
  double t4269;
  double t4314;
  double t4346;
  double t4364;
  double t4382;
  double t4386;
  double t4401;
  double t4460;
  double t4461;
  double t4466;
  double t4475;
  double t4639;
  double t4674;
  double t4690;
  double t4739;
  double t4744;
  double t4746;
  double t4756;
  double t4772;
  double t4777;
  double t3605;
  double t3903;
  double t3907;
  double t4068;
  double t4070;
  double t4073;
  double t4317;
  double t4476;
  double t4479;
  double t4533;
  double t4556;
  double t4607;
  double t4716;
  double t4794;
  double t4835;
  double t4856;
  double t4875;
  double t4876;
  t1199 = Cos(var1[3]);
  t2469 = Cos(var1[5]);
  t2943 = Sin(var1[4]);
  t2522 = Sin(var1[3]);
  t2970 = Sin(var1[5]);
  t1829 = Cos(var1[4]);
  t2416 = Sin(var1[11]);
  t538 = Cos(var1[11]);
  t2530 = -1.*t2469*t2522;
  t2977 = t1199*t2943*t2970;
  t3359 = t2530 + t2977;
  t3634 = Cos(var1[13]);
  t2287 = t538*t1199*t1829;
  t3424 = t2416*t3359;
  t3431 = t2287 + t3424;
  t363 = Sin(var1[13]);
  t3653 = Cos(var1[12]);
  t3672 = t1199*t2469*t2943;
  t3684 = t2522*t2970;
  t3725 = t3672 + t3684;
  t3742 = t3653*t3725;
  t3797 = Sin(var1[12]);
  t3810 = -1.*t1199*t1829*t2416;
  t3825 = t538*t3359;
  t3830 = t3810 + t3825;
  t3860 = -1.*t3797*t3830;
  t3881 = t3742 + t3860;
  t129 = Sin(var1[14]);
  t4156 = t1199*t2469;
  t4163 = t2522*t2943*t2970;
  t4172 = t4156 + t4163;
  t4062 = Cos(var1[14]);
  t4152 = t538*t1829*t2522;
  t4269 = t2416*t4172;
  t4314 = t4152 + t4269;
  t4346 = t2469*t2522*t2943;
  t4364 = -1.*t1199*t2970;
  t4382 = t4346 + t4364;
  t4386 = t3653*t4382;
  t4401 = -1.*t1829*t2416*t2522;
  t4460 = t538*t4172;
  t4461 = t4401 + t4460;
  t4466 = -1.*t3797*t4461;
  t4475 = t4386 + t4466;
  t4639 = -1.*t538*t2943;
  t4674 = t1829*t2416*t2970;
  t4690 = t4639 + t4674;
  t4739 = t3653*t1829*t2469;
  t4744 = t2416*t2943;
  t4746 = t538*t1829*t2970;
  t4756 = t4744 + t4746;
  t4772 = -1.*t3797*t4756;
  t4777 = t4739 + t4772;
  t3605 = t363*t3431;
  t3903 = t3634*t3881;
  t3907 = t3605 + t3903;
  t4068 = t3634*t3431;
  t4070 = -1.*t363*t3881;
  t4073 = t4068 + t4070;
  t4317 = t363*t4314;
  t4476 = t3634*t4475;
  t4479 = t4317 + t4476;
  t4533 = t3634*t4314;
  t4556 = -1.*t363*t4475;
  t4607 = t4533 + t4556;
  t4716 = t363*t4690;
  t4794 = t3634*t4777;
  t4835 = t4716 + t4794;
  t4856 = t3634*t4690;
  t4875 = -1.*t363*t4777;
  t4876 = t4856 + t4875;

  p_output1(0)=-1.*t129*t3907 + t4062*t4073;
  p_output1(1)=-1.*t129*t4479 + t4062*t4607;
  p_output1(2)=-1.*t129*t4835 + t4062*t4876;
  p_output1(3)=t3725*t3797 + t3653*t3830;
  p_output1(4)=t3797*t4382 + t3653*t4461;
  p_output1(5)=t1829*t2469*t3797 + t3653*t4756;
  p_output1(6)=t3907*t4062 + t129*t4073;
  p_output1(7)=t4062*t4479 + t129*t4607;
  p_output1(8)=t4062*t4835 + t129*t4876;
}


       
void R_rKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
