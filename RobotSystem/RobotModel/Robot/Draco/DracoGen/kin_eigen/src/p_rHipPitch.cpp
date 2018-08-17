/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:47 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipPitch.h"

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
  double t638;
  double t2906;
  double t3115;
  double t3272;
  double t3383;
  double t4409;
  double t4668;
  double t4410;
  double t4681;
  double t3839;
  double t3882;
  double t4027;
  double t4248;
  double t2570;
  double t5101;
  double t5126;
  double t5339;
  double t4584;
  double t4718;
  double t4729;
  double t5416;
  double t5417;
  double t5428;
  double t5432;
  double t5440;
  double t5444;
  double t5503;
  double t5573;
  double t5576;
  double t5596;
  double t5745;
  double t5782;
  double t5824;
  double t3380;
  double t3423;
  double t3453;
  double t4199;
  double t4261;
  double t4309;
  double t4912;
  double t5035;
  double t5095;
  double t5390;
  double t5391;
  double t5398;
  double t6015;
  double t6016;
  double t6017;
  double t5473;
  double t5528;
  double t5531;
  double t5959;
  double t5964;
  double t5967;
  double t6023;
  double t6024;
  double t6026;
  double t5719;
  double t5727;
  double t5729;
  double t6034;
  double t6039;
  double t6047;
  double t6105;
  double t6110;
  double t6113;
  double t6217;
  double t6222;
  double t6241;
  double t6261;
  double t6264;
  double t6270;
  double t6312;
  double t6316;
  double t6317;
  t638 = Cos(var1[3]);
  t2906 = Cos(var1[11]);
  t3115 = -1.*t2906;
  t3272 = 1. + t3115;
  t3383 = Sin(var1[11]);
  t4409 = Cos(var1[5]);
  t4668 = Sin(var1[3]);
  t4410 = Sin(var1[4]);
  t4681 = Sin(var1[5]);
  t3839 = Cos(var1[12]);
  t3882 = -1.*t3839;
  t4027 = 1. + t3882;
  t4248 = Sin(var1[12]);
  t2570 = Cos(var1[4]);
  t5101 = -1.*t4409*t4668;
  t5126 = t638*t4410*t4681;
  t5339 = t5101 + t5126;
  t4584 = t638*t4409*t4410;
  t4718 = t4668*t4681;
  t4729 = t4584 + t4718;
  t5416 = -1.*t638*t2570*t3383;
  t5417 = t2906*t5339;
  t5428 = t5416 + t5417;
  t5432 = Cos(var1[13]);
  t5440 = -1.*t5432;
  t5444 = 1. + t5440;
  t5503 = Sin(var1[13]);
  t5573 = t2906*t638*t2570;
  t5576 = t3383*t5339;
  t5596 = t5573 + t5576;
  t5745 = t3839*t4729;
  t5782 = -1.*t4248*t5428;
  t5824 = t5745 + t5782;
  t3380 = -0.022225*t3272;
  t3423 = -0.086996*t3383;
  t3453 = 0. + t3380 + t3423;
  t4199 = -0.31508*t4027;
  t4261 = 0.156996*t4248;
  t4309 = 0. + t4199 + t4261;
  t4912 = -0.086996*t3272;
  t5035 = 0.022225*t3383;
  t5095 = 0. + t4912 + t5035;
  t5390 = -0.156996*t4027;
  t5391 = -0.31508*t4248;
  t5398 = 0. + t5390 + t5391;
  t6015 = t638*t4409;
  t6016 = t4668*t4410*t4681;
  t6017 = t6015 + t6016;
  t5473 = -0.022225*t5444;
  t5528 = 0.38008*t5503;
  t5531 = 0. + t5473 + t5528;
  t5959 = t4409*t4668*t4410;
  t5964 = -1.*t638*t4681;
  t5967 = t5959 + t5964;
  t6023 = -1.*t2570*t3383*t4668;
  t6024 = t2906*t6017;
  t6026 = t6023 + t6024;
  t5719 = -0.38008*t5444;
  t5727 = -0.022225*t5503;
  t5729 = 0. + t5719 + t5727;
  t6034 = t2906*t2570*t4668;
  t6039 = t3383*t6017;
  t6047 = t6034 + t6039;
  t6105 = t3839*t5967;
  t6110 = -1.*t4248*t6026;
  t6113 = t6105 + t6110;
  t6217 = t3383*t4410;
  t6222 = t2906*t2570*t4681;
  t6241 = t6217 + t6222;
  t6261 = -1.*t2906*t4410;
  t6264 = t2570*t3383*t4681;
  t6270 = t6261 + t6264;
  t6312 = t3839*t2570*t4409;
  t6316 = -1.*t4248*t6241;
  t6317 = t6312 + t6316;

  p_output1(0)=0. + t4309*t4729 + t5095*t5339 + t5398*t5428 - 0.166996*(t4248*t4729 + t3839*t5428) + t5531*t5596 + t5729*t5824 - 0.38008*(t5503*t5596 + t5432*t5824) - 0.022225*(t5432*t5596 - 1.*t5503*t5824) + t2570*t3453*t638 + var1(0);
  p_output1(1)=0. + t2570*t3453*t4668 + t4309*t5967 + t5095*t6017 + t5398*t6026 - 0.166996*(t4248*t5967 + t3839*t6026) + t5531*t6047 + t5729*t6113 - 0.38008*(t5503*t6047 + t5432*t6113) - 0.022225*(t5432*t6047 - 1.*t5503*t6113) + var1(1);
  p_output1(2)=0. + t2570*t4309*t4409 - 1.*t3453*t4410 + t2570*t4681*t5095 + t5398*t6241 - 0.166996*(t2570*t4248*t4409 + t3839*t6241) + t5531*t6270 + t5729*t6317 - 0.38008*(t5503*t6270 + t5432*t6317) - 0.022225*(t5432*t6270 - 1.*t5503*t6317) + var1(2);
}


       
void p_rHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
