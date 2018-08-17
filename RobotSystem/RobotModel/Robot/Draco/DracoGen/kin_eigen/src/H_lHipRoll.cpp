/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:43 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipRoll.h"

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
  double t1103;
  double t1160;
  double t1804;
  double t2340;
  double t2179;
  double t2733;
  double t2818;
  double t3035;
  double t2536;
  double t2896;
  double t2985;
  double t3675;
  double t3181;
  double t3186;
  double t3222;
  double t4333;
  double t4015;
  double t4183;
  double t4309;
  double t3704;
  double t3728;
  double t3794;
  double t4721;
  double t4737;
  double t4803;
  double t4497;
  double t4535;
  double t4667;
  double t4898;
  double t4937;
  double t4945;
  double t5190;
  double t5192;
  double t2043;
  double t3043;
  double t3097;
  double t5274;
  double t5277;
  double t3911;
  double t4369;
  double t4370;
  double t4987;
  double t5018;
  double t5044;
  double t5194;
  double t5196;
  double t5214;
  double t5232;
  double t5235;
  double t5249;
  double t3142;
  double t3262;
  double t3444;
  double t5281;
  double t5286;
  double t5301;
  double t5307;
  double t5320;
  double t5325;
  double t4681;
  double t4808;
  double t4857;
  double t5066;
  double t5107;
  double t5135;
  double t3480;
  double t3582;
  double t3599;
  double t4957;
  double t4965;
  double t4981;
  double t5142;
  double t5180;
  double t5182;
  t1103 = Cos(var1[3]);
  t1160 = Cos(var1[4]);
  t1804 = Cos(var1[6]);
  t2340 = Sin(var1[3]);
  t2179 = Cos(var1[5]);
  t2733 = Sin(var1[4]);
  t2818 = Sin(var1[5]);
  t3035 = Sin(var1[6]);
  t2536 = -1.*t2179*t2340;
  t2896 = t1103*t2733*t2818;
  t2985 = t2536 + t2896;
  t3675 = Cos(var1[7]);
  t3181 = t1103*t2179;
  t3186 = t2340*t2733*t2818;
  t3222 = t3181 + t3186;
  t4333 = Sin(var1[7]);
  t4015 = t1103*t2179*t2733;
  t4183 = t2340*t2818;
  t4309 = t4015 + t4183;
  t3704 = t1804*t2985;
  t3728 = -1.*t1103*t1160*t3035;
  t3794 = t3704 + t3728;
  t4721 = t2179*t2340*t2733;
  t4737 = -1.*t1103*t2818;
  t4803 = t4721 + t4737;
  t4497 = t1804*t3222;
  t4535 = -1.*t1160*t2340*t3035;
  t4667 = t4497 + t4535;
  t4898 = t1160*t1804*t2818;
  t4937 = t2733*t3035;
  t4945 = t4898 + t4937;
  t5190 = -1.*t1804;
  t5192 = 1. + t5190;
  t2043 = t1103*t1160*t1804;
  t3043 = t2985*t3035;
  t3097 = t2043 + t3043;
  t5274 = -1.*t3675;
  t5277 = 1. + t5274;
  t3911 = t3675*t3794;
  t4369 = t4309*t4333;
  t4370 = t3911 + t4369;
  t4987 = t3675*t4309;
  t5018 = -1.*t3794*t4333;
  t5044 = t4987 + t5018;
  t5194 = 0.087004*t5192;
  t5196 = 0.022225*t3035;
  t5214 = 0. + t5194 + t5196;
  t5232 = -0.022225*t5192;
  t5235 = 0.087004*t3035;
  t5249 = 0. + t5232 + t5235;
  t3142 = t1160*t1804*t2340;
  t3262 = t3222*t3035;
  t3444 = t3142 + t3262;
  t5281 = 0.157004*t5277;
  t5286 = -0.31508*t4333;
  t5301 = 0. + t5281 + t5286;
  t5307 = -0.31508*t5277;
  t5320 = -0.157004*t4333;
  t5325 = 0. + t5307 + t5320;
  t4681 = t3675*t4667;
  t4808 = t4803*t4333;
  t4857 = t4681 + t4808;
  t5066 = t3675*t4803;
  t5107 = -1.*t4667*t4333;
  t5135 = t5066 + t5107;
  t3480 = -1.*t1804*t2733;
  t3582 = t1160*t2818*t3035;
  t3599 = t3480 + t3582;
  t4957 = t3675*t4945;
  t4965 = t1160*t2179*t4333;
  t4981 = t4957 + t4965;
  t5142 = t1160*t2179*t3675;
  t5180 = -1.*t4945*t4333;
  t5182 = t5142 + t5180;

  p_output1(0)=t3097;
  p_output1(1)=t3444;
  p_output1(2)=t3599;
  p_output1(3)=0.;
  p_output1(4)=t4370;
  p_output1(5)=t4857;
  p_output1(6)=t4981;
  p_output1(7)=0.;
  p_output1(8)=t5044;
  p_output1(9)=t5135;
  p_output1(10)=t5182;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3097 + 0.157004*t4370 - 0.31508*t5044 + t2985*t5214 + t1103*t1160*t5249 + t3794*t5301 + t4309*t5325 + var1(0);
  p_output1(13)=0. - 0.022225*t3444 + 0.157004*t4857 - 0.31508*t5135 + t3222*t5214 + t1160*t2340*t5249 + t4667*t5301 + t4803*t5325 + var1(1);
  p_output1(14)=0. - 0.022225*t3599 + 0.157004*t4981 - 0.31508*t5182 + t1160*t2818*t5214 - 1.*t2733*t5249 + t4945*t5301 + t1160*t2179*t5325 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
