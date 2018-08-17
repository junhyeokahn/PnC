/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipPitch.h"

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
  double t1054;
  double t1248;
  double t1580;
  double t1502;
  double t1590;
  double t1081;
  double t1239;
  double t501;
  double t1526;
  double t1746;
  double t1986;
  double t220;
  double t2421;
  double t2500;
  double t2724;
  double t3216;
  double t3423;
  double t3433;
  double t2514;
  double t2634;
  double t2662;
  double t2726;
  double t2727;
  double t2850;
  double t3514;
  double t3536;
  double t3537;
  double t3684;
  double t4082;
  double t4124;
  double t4361;
  double t4371;
  double t4393;
  double t1148;
  double t2159;
  double t2365;
  double t2696;
  double t2899;
  double t2970;
  double t3035;
  double t3467;
  double t3502;
  double t3592;
  double t4125;
  double t4163;
  double t4169;
  double t4198;
  double t4261;
  double t4355;
  double t4463;
  double t4730;
  double t5646;
  double t5662;
  double t5804;
  double t5830;
  double t4863;
  double t4963;
  double t4997;
  double t6016;
  double t6017;
  double t5431;
  double t5473;
  double t5489;
  double t2375;
  double t2986;
  double t3032;
  double t5681;
  double t5719;
  double t5745;
  double t5863;
  double t5872;
  double t5873;
  double t5888;
  double t5896;
  double t5898;
  double t5901;
  double t5902;
  double t5916;
  double t6018;
  double t6024;
  double t6029;
  double t5126;
  double t5137;
  double t5174;
  double t6050;
  double t6057;
  double t6058;
  double t5523;
  double t5528;
  double t5531;
  double t3513;
  double t4167;
  double t4168;
  double t5339;
  double t5346;
  double t5390;
  double t5571;
  double t5576;
  double t5620;
  double t4351;
  double t4750;
  double t4822;
  t1054 = Cos(var1[3]);
  t1248 = Cos(var1[5]);
  t1580 = Sin(var1[4]);
  t1502 = Sin(var1[3]);
  t1590 = Sin(var1[5]);
  t1081 = Cos(var1[4]);
  t1239 = Sin(var1[11]);
  t501 = Cos(var1[11]);
  t1526 = -1.*t1248*t1502;
  t1746 = t1054*t1580*t1590;
  t1986 = t1526 + t1746;
  t220 = Cos(var1[13]);
  t2421 = Sin(var1[13]);
  t2500 = Cos(var1[12]);
  t2724 = Sin(var1[12]);
  t3216 = t1054*t1248;
  t3423 = t1502*t1580*t1590;
  t3433 = t3216 + t3423;
  t2514 = t1054*t1248*t1580;
  t2634 = t1502*t1590;
  t2662 = t2514 + t2634;
  t2726 = -1.*t1054*t1081*t1239;
  t2727 = t501*t1986;
  t2850 = t2726 + t2727;
  t3514 = t1248*t1502*t1580;
  t3536 = -1.*t1054*t1590;
  t3537 = t3514 + t3536;
  t3684 = -1.*t1081*t1239*t1502;
  t4082 = t501*t3433;
  t4124 = t3684 + t4082;
  t4361 = t1239*t1580;
  t4371 = t501*t1081*t1590;
  t4393 = t4361 + t4371;
  t1148 = t501*t1054*t1081;
  t2159 = t1239*t1986;
  t2365 = t1148 + t2159;
  t2696 = t2500*t2662;
  t2899 = -1.*t2724*t2850;
  t2970 = t2696 + t2899;
  t3035 = t501*t1081*t1502;
  t3467 = t1239*t3433;
  t3502 = t3035 + t3467;
  t3592 = t2500*t3537;
  t4125 = -1.*t2724*t4124;
  t4163 = t3592 + t4125;
  t4169 = -1.*t501*t1580;
  t4198 = t1081*t1239*t1590;
  t4261 = t4169 + t4198;
  t4355 = t2500*t1081*t1248;
  t4463 = -1.*t2724*t4393;
  t4730 = t4355 + t4463;
  t5646 = -1.*t501;
  t5662 = 1. + t5646;
  t5804 = -1.*t2500;
  t5830 = 1. + t5804;
  t4863 = t2724*t2662;
  t4963 = t2500*t2850;
  t4997 = t4863 + t4963;
  t6016 = -1.*t220;
  t6017 = 1. + t6016;
  t5431 = t2421*t2365;
  t5473 = t220*t2970;
  t5489 = t5431 + t5473;
  t2375 = t220*t2365;
  t2986 = -1.*t2421*t2970;
  t3032 = t2375 + t2986;
  t5681 = -0.022225*t5662;
  t5719 = -0.086996*t1239;
  t5745 = 0. + t5681 + t5719;
  t5863 = -0.31508*t5830;
  t5872 = 0.156996*t2724;
  t5873 = 0. + t5863 + t5872;
  t5888 = -0.086996*t5662;
  t5896 = 0.022225*t1239;
  t5898 = 0. + t5888 + t5896;
  t5901 = -0.156996*t5830;
  t5902 = -0.31508*t2724;
  t5916 = 0. + t5901 + t5902;
  t6018 = -0.022225*t6017;
  t6024 = 0.38008*t2421;
  t6029 = 0. + t6018 + t6024;
  t5126 = t2724*t3537;
  t5137 = t2500*t4124;
  t5174 = t5126 + t5137;
  t6050 = -0.38008*t6017;
  t6057 = -0.022225*t2421;
  t6058 = 0. + t6050 + t6057;
  t5523 = t2421*t3502;
  t5528 = t220*t4163;
  t5531 = t5523 + t5528;
  t3513 = t220*t3502;
  t4167 = -1.*t2421*t4163;
  t4168 = t3513 + t4167;
  t5339 = t1081*t1248*t2724;
  t5346 = t2500*t4393;
  t5390 = t5339 + t5346;
  t5571 = t2421*t4261;
  t5576 = t220*t4730;
  t5620 = t5571 + t5576;
  t4351 = t220*t4261;
  t4750 = -1.*t2421*t4730;
  t4822 = t4351 + t4750;

  p_output1(0)=t3032;
  p_output1(1)=t4168;
  p_output1(2)=t4822;
  p_output1(3)=0.;
  p_output1(4)=t4997;
  p_output1(5)=t5174;
  p_output1(6)=t5390;
  p_output1(7)=0.;
  p_output1(8)=t5489;
  p_output1(9)=t5531;
  p_output1(10)=t5620;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3032 - 0.166996*t4997 - 0.38008*t5489 + t1054*t1081*t5745 + t2662*t5873 + t1986*t5898 + t2850*t5916 + t2365*t6029 + t2970*t6058 + var1(0);
  p_output1(13)=0. - 0.022225*t4168 - 0.166996*t5174 - 0.38008*t5531 + t1081*t1502*t5745 + t3537*t5873 + t3433*t5898 + t4124*t5916 + t3502*t6029 + t4163*t6058 + var1(1);
  p_output1(14)=0. - 0.022225*t4822 - 0.166996*t5390 - 0.38008*t5620 - 1.*t1580*t5745 + t1081*t1248*t5873 + t1081*t1590*t5898 + t4393*t5916 + t4261*t6029 + t4730*t6058 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
