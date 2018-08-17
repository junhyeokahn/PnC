/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipRoll.h"

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
  double t1863;
  double t647;
  double t1172;
  double t1227;
  double t1580;
  double t3384;
  double t2734;
  double t3043;
  double t3473;
  double t2043;
  double t2156;
  double t2211;
  double t2507;
  double t113;
  double t4103;
  double t4105;
  double t4118;
  double t3362;
  double t3482;
  double t3530;
  double t4905;
  double t5045;
  double t5047;
  double t1266;
  double t1642;
  double t1668;
  double t2388;
  double t2551;
  double t2661;
  double t3828;
  double t3901;
  double t4018;
  double t4780;
  double t4810;
  double t4845;
  double t5445;
  double t5451;
  double t5500;
  double t5356;
  double t5389;
  double t5394;
  double t5588;
  double t5589;
  double t5596;
  double t5983;
  double t6022;
  double t6039;
  double t6264;
  double t6267;
  double t6268;
  double t6483;
  double t6509;
  double t6510;
  double t6659;
  double t6660;
  double t6665;
  double t6722;
  double t6726;
  double t6728;
  double t6957;
  double t6960;
  double t6963;
  double t6900;
  double t6903;
  double t6905;
  double t6909;
  double t6918;
  double t7004;
  double t7006;
  double t7010;
  double t5066;
  double t7033;
  double t7035;
  double t7129;
  double t7133;
  double t7134;
  double t5807;
  double t5808;
  double t5825;
  double t7200;
  double t7207;
  double t7222;
  double t7240;
  double t7243;
  double t7024;
  double t7026;
  double t7029;
  double t7119;
  double t7120;
  double t7121;
  t1863 = Sin(var1[3]);
  t647 = Cos(var1[11]);
  t1172 = -1.*t647;
  t1227 = 1. + t1172;
  t1580 = Sin(var1[11]);
  t3384 = Cos(var1[3]);
  t2734 = Cos(var1[5]);
  t3043 = Sin(var1[4]);
  t3473 = Sin(var1[5]);
  t2043 = Cos(var1[12]);
  t2156 = -1.*t2043;
  t2211 = 1. + t2156;
  t2507 = Sin(var1[12]);
  t113 = Cos(var1[4]);
  t4103 = -1.*t3384*t2734;
  t4105 = -1.*t1863*t3043*t3473;
  t4118 = t4103 + t4105;
  t3362 = -1.*t2734*t1863*t3043;
  t3482 = t3384*t3473;
  t3530 = t3362 + t3482;
  t4905 = t113*t1580*t1863;
  t5045 = t647*t4118;
  t5047 = t4905 + t5045;
  t1266 = -0.022225*t1227;
  t1642 = -0.086996*t1580;
  t1668 = 0. + t1266 + t1642;
  t2388 = -0.31508*t2211;
  t2551 = 0.156996*t2507;
  t2661 = 0. + t2388 + t2551;
  t3828 = -0.086996*t1227;
  t3901 = 0.022225*t1580;
  t4018 = 0. + t3828 + t3901;
  t4780 = -0.156996*t2211;
  t4810 = -0.31508*t2507;
  t4845 = 0. + t4780 + t4810;
  t5445 = -1.*t2734*t1863;
  t5451 = t3384*t3043*t3473;
  t5500 = t5445 + t5451;
  t5356 = t3384*t2734*t3043;
  t5389 = t1863*t3473;
  t5394 = t5356 + t5389;
  t5588 = -1.*t3384*t113*t1580;
  t5589 = t647*t5500;
  t5596 = t5588 + t5589;
  t5983 = t3384*t1580*t3043;
  t6022 = t647*t3384*t113*t3473;
  t6039 = t5983 + t6022;
  t6264 = t1580*t1863*t3043;
  t6267 = t647*t113*t1863*t3473;
  t6268 = t6264 + t6267;
  t6483 = t113*t1580;
  t6509 = -1.*t647*t3043*t3473;
  t6510 = t6483 + t6509;
  t6659 = t2734*t1863;
  t6660 = -1.*t3384*t3043*t3473;
  t6665 = t6659 + t6660;
  t6722 = t2734*t1863*t3043;
  t6726 = -1.*t3384*t3473;
  t6728 = t6722 + t6726;
  t6957 = -1.*t647*t3384*t113;
  t6960 = -1.*t1580*t5500;
  t6963 = t6957 + t6960;
  t6900 = -0.086996*t647;
  t6903 = -0.022225*t1580;
  t6905 = t6900 + t6903;
  t6909 = 0.022225*t647;
  t6918 = t6909 + t1642;
  t7004 = t3384*t2734;
  t7006 = t1863*t3043*t3473;
  t7010 = t7004 + t7006;
  t5066 = -1.*t647*t113*t1863;
  t7033 = -1.*t1580*t7010;
  t7035 = t5066 + t7033;
  t7129 = t647*t3043;
  t7133 = -1.*t113*t1580*t3473;
  t7134 = t7129 + t7133;
  t5807 = t2043*t5394;
  t5808 = -1.*t2507*t5596;
  t5825 = t5807 + t5808;
  t7200 = 0.156996*t2043;
  t7207 = t7200 + t4810;
  t7222 = -0.31508*t2043;
  t7240 = -0.156996*t2507;
  t7243 = t7222 + t7240;
  t7024 = -1.*t113*t1580*t1863;
  t7026 = t647*t7010;
  t7029 = t7024 + t7026;
  t7119 = t1580*t3043;
  t7120 = t647*t113*t3473;
  t7121 = t7119 + t7120;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t113*t1668*t1863 + t2661*t3530 + t4018*t4118 + t4845*t5047 - 0.156996*(t2507*t3530 + t2043*t5047) - 0.31508*(t2043*t3530 - 1.*t2507*t5047) - 0.022225*(t1580*t4118 + t5066);
  p_output1(10)=t113*t1668*t3384 + t2661*t5394 + t4018*t5500 + t4845*t5596 - 0.156996*(t2507*t5394 + t2043*t5596) - 0.31508*t5825 - 0.022225*(t1580*t5500 + t113*t3384*t647);
  p_output1(11)=0;
  p_output1(12)=t113*t2661*t2734*t3384 - 1.*t1668*t3043*t3384 + t113*t3384*t3473*t4018 + t4845*t6039 - 0.156996*(t113*t2507*t2734*t3384 + t2043*t6039) - 0.31508*(t113*t2043*t2734*t3384 - 1.*t2507*t6039) - 0.022225*(t113*t1580*t3384*t3473 - 1.*t3043*t3384*t647);
  p_output1(13)=t113*t1863*t2661*t2734 - 1.*t1668*t1863*t3043 + t113*t1863*t3473*t4018 + t4845*t6268 - 0.156996*(t113*t1863*t2507*t2734 + t2043*t6268) - 0.31508*(t113*t1863*t2043*t2734 - 1.*t2507*t6268) - 0.022225*(t113*t1580*t1863*t3473 - 1.*t1863*t3043*t647);
  p_output1(14)=-1.*t113*t1668 - 1.*t2661*t2734*t3043 - 1.*t3043*t3473*t4018 - 0.022225*(-1.*t1580*t3043*t3473 - 1.*t113*t647) + t4845*t6510 - 0.156996*(-1.*t2507*t2734*t3043 + t2043*t6510) - 0.31508*(-1.*t2043*t2734*t3043 - 1.*t2507*t6510);
  p_output1(15)=-0.022225*t1580*t5394 + t4018*t5394 + t4845*t5394*t647 + t2661*t6665 - 0.31508*(-1.*t2507*t5394*t647 + t2043*t6665) - 0.156996*(t2043*t5394*t647 + t2507*t6665);
  p_output1(16)=t2661*t4118 - 0.022225*t1580*t6728 + t4018*t6728 + t4845*t647*t6728 - 0.156996*(t2507*t4118 + t2043*t647*t6728) - 0.31508*(t2043*t4118 - 1.*t2507*t647*t6728);
  p_output1(17)=-0.022225*t113*t1580*t2734 - 1.*t113*t2661*t3473 + t113*t2734*t4018 + t113*t2734*t4845*t647 - 0.156996*(-1.*t113*t2507*t3473 + t113*t2043*t2734*t647) - 0.31508*(-1.*t113*t2043*t3473 - 1.*t113*t2507*t2734*t647);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
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
  p_output1(33)=-0.022225*t5596 + t113*t3384*t6905 + t5500*t6918 - 0.156996*t2043*t6963 + 0.31508*t2507*t6963 + t4845*t6963;
  p_output1(34)=t113*t1863*t6905 + t6918*t7010 - 0.022225*t7029 - 0.156996*t2043*t7035 + 0.31508*t2507*t7035 + t4845*t7035;
  p_output1(35)=-1.*t3043*t6905 + t113*t3473*t6918 - 0.022225*t7121 - 0.156996*t2043*t7134 + 0.31508*t2507*t7134 + t4845*t7134;
  p_output1(36)=-0.31508*(-1.*t2507*t5394 - 1.*t2043*t5596) - 0.156996*t5825 + t5394*t7207 + t5596*t7243;
  p_output1(37)=-0.31508*(-1.*t2507*t6728 - 1.*t2043*t7029) - 0.156996*(t2043*t6728 - 1.*t2507*t7029) + t6728*t7207 + t7029*t7243;
  p_output1(38)=-0.31508*(-1.*t113*t2507*t2734 - 1.*t2043*t7121) - 0.156996*(t113*t2043*t2734 - 1.*t2507*t7121) + t113*t2734*t7207 + t7121*t7243;
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


       
void Jp_rHipRoll(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
