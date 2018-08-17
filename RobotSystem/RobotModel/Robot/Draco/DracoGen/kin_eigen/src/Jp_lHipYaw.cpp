/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:40 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipYaw.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t2058;
  double t1205;
  double t1264;
  double t1861;
  double t2069;
  double t2586;
  double t2612;
  double t3099;
  double t3526;
  double t2394;
  double t2409;
  double t2420;
  double t4044;
  double t3221;
  double t3586;
  double t3601;
  double t4088;
  double t4093;
  double t4167;
  double t4486;
  double t4531;
  double t4590;
  double t4400;
  double t4408;
  double t4417;
  double t4981;
  double t4996;
  double t4998;
  double t4635;
  double t4663;
  double t4675;
  double t5095;
  double t5098;
  double t5100;
  double t5107;
  double t5119;
  double t5224;
  double t5235;
  double t5258;
  double t4281;
  t2058 = Cos(var1[3]);
  t1205 = Cos(var1[5]);
  t1264 = Sin(var1[3]);
  t1861 = Sin(var1[4]);
  t2069 = Sin(var1[5]);
  t2586 = Cos(var1[6]);
  t2612 = -1.*t2586;
  t3099 = 1. + t2612;
  t3526 = Sin(var1[6]);
  t2394 = -1.*t2058*t1205;
  t2409 = -1.*t1264*t1861*t2069;
  t2420 = t2394 + t2409;
  t4044 = Cos(var1[4]);
  t3221 = 0.087004*t3099;
  t3586 = 0.022225*t3526;
  t3601 = 0. + t3221 + t3586;
  t4088 = -0.022225*t3099;
  t4093 = 0.087004*t3526;
  t4167 = 0. + t4088 + t4093;
  t4486 = -1.*t1205*t1264;
  t4531 = t2058*t1861*t2069;
  t4590 = t4486 + t4531;
  t4400 = t2058*t1205*t1861;
  t4408 = t1264*t2069;
  t4417 = t4400 + t4408;
  t4981 = t1205*t1264*t1861;
  t4996 = -1.*t2058*t2069;
  t4998 = t4981 + t4996;
  t4635 = t2586*t4590;
  t4663 = -1.*t2058*t4044*t3526;
  t4675 = t4635 + t4663;
  t5095 = 0.087004*t2586;
  t5098 = -0.022225*t3526;
  t5100 = t5095 + t5098;
  t5107 = 0.022225*t2586;
  t5119 = t5107 + t4093;
  t5224 = t2058*t1205;
  t5235 = t1264*t1861*t2069;
  t5258 = t5224 + t5235;
  t4281 = -1.*t4044*t2586*t1264;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-0.29508*(-1.*t1205*t1264*t1861 + t2058*t2069) + t2420*t3601 + 0.087004*(t2420*t2586 + t1264*t3526*t4044) - 1.*t1264*t4044*t4167 - 0.022225*(t2420*t3526 + t4281);
  p_output1(10)=t2058*t4044*t4167 - 0.29508*t4417 + t3601*t4590 - 0.022225*(t2058*t2586*t4044 + t3526*t4590) + 0.087004*t4675;
  p_output1(11)=0;
  p_output1(12)=-0.29508*t1205*t2058*t4044 + t2058*t2069*t3601*t4044 + 0.087004*(t1861*t2058*t3526 + t2058*t2069*t2586*t4044) - 0.022225*(-1.*t1861*t2058*t2586 + t2058*t2069*t3526*t4044) - 1.*t1861*t2058*t4167;
  p_output1(13)=-0.29508*t1205*t1264*t4044 + t1264*t2069*t3601*t4044 + 0.087004*(t1264*t1861*t3526 + t1264*t2069*t2586*t4044) - 0.022225*(-1.*t1264*t1861*t2586 + t1264*t2069*t3526*t4044) - 1.*t1264*t1861*t4167;
  p_output1(14)=0.29508*t1205*t1861 - 1.*t1861*t2069*t3601 - 0.022225*(-1.*t1861*t2069*t3526 - 1.*t2586*t4044) + 0.087004*(-1.*t1861*t2069*t2586 + t3526*t4044) - 1.*t4044*t4167;
  p_output1(15)=-0.29508*(t1205*t1264 - 1.*t1861*t2058*t2069) + 0.087004*t2586*t4417 - 0.022225*t3526*t4417 + t3601*t4417;
  p_output1(16)=-0.29508*t2420 + 0.087004*t2586*t4998 - 0.022225*t3526*t4998 + t3601*t4998;
  p_output1(17)=0.29508*t2069*t4044 + 0.087004*t1205*t2586*t4044 - 0.022225*t1205*t3526*t4044 + t1205*t3601*t4044;
  p_output1(18)=0.087004*(-1.*t2058*t2586*t4044 - 1.*t3526*t4590) - 0.022225*t4675 + t2058*t4044*t5100 + t4590*t5119;
  p_output1(19)=t1264*t4044*t5100 + t5119*t5258 - 0.022225*(-1.*t1264*t3526*t4044 + t2586*t5258) + 0.087004*(t4281 - 1.*t3526*t5258);
  p_output1(20)=-0.022225*(t1861*t3526 + t2069*t2586*t4044) + 0.087004*(t1861*t2586 - 1.*t2069*t3526*t4044) - 1.*t1861*t5100 + t2069*t4044*t5119;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_lHipYaw(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
