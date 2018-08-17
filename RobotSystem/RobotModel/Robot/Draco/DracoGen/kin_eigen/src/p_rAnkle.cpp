/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:02 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rAnkle.h"

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
  double t105;
  double t849;
  double t887;
  double t888;
  double t918;
  double t1087;
  double t1772;
  double t1272;
  double t1790;
  double t929;
  double t943;
  double t953;
  double t977;
  double t838;
  double t2642;
  double t2747;
  double t2854;
  double t1524;
  double t1847;
  double t2223;
  double t3453;
  double t4116;
  double t4129;
  double t4191;
  double t4303;
  double t4312;
  double t4407;
  double t4497;
  double t4500;
  double t4519;
  double t4941;
  double t5001;
  double t5028;
  double t5065;
  double t5067;
  double t5144;
  double t5184;
  double t5243;
  double t5339;
  double t5362;
  double t5465;
  double t5485;
  double t5503;
  double t5516;
  double t5533;
  double t5543;
  double t5571;
  double t5610;
  double t5614;
  double t5615;
  double t5684;
  double t5695;
  double t5704;
  double t914;
  double t920;
  double t921;
  double t975;
  double t979;
  double t1020;
  double t2553;
  double t2562;
  double t2609;
  double t3011;
  double t3161;
  double t3173;
  double t5893;
  double t5898;
  double t5912;
  double t4392;
  double t4433;
  double t4480;
  double t5828;
  double t5841;
  double t5847;
  double t5945;
  double t5950;
  double t5953;
  double t4880;
  double t4882;
  double t4934;
  double t5147;
  double t5185;
  double t5213;
  double t5964;
  double t5967;
  double t5970;
  double t6009;
  double t6040;
  double t6042;
  double t5402;
  double t5417;
  double t5442;
  double t5546;
  double t5577;
  double t5578;
  double t6130;
  double t6138;
  double t6180;
  double t6213;
  double t6229;
  double t6253;
  double t5620;
  double t5649;
  double t5653;
  double t6276;
  double t6309;
  double t6327;
  double t6347;
  double t6351;
  double t6376;
  double t6760;
  double t6790;
  double t6840;
  double t6876;
  double t6877;
  double t6916;
  double t7057;
  double t7105;
  double t7119;
  double t7141;
  double t7170;
  double t7181;
  double t7192;
  double t7205;
  double t7218;
  double t7236;
  double t7257;
  double t7258;
  double t7264;
  double t7271;
  double t7272;
  t105 = Cos(var1[3]);
  t849 = Cos(var1[11]);
  t887 = -1.*t849;
  t888 = 1. + t887;
  t918 = Sin(var1[11]);
  t1087 = Cos(var1[5]);
  t1772 = Sin(var1[3]);
  t1272 = Sin(var1[4]);
  t1790 = Sin(var1[5]);
  t929 = Cos(var1[12]);
  t943 = -1.*t929;
  t953 = 1. + t943;
  t977 = Sin(var1[12]);
  t838 = Cos(var1[4]);
  t2642 = -1.*t1087*t1772;
  t2747 = t105*t1272*t1790;
  t2854 = t2642 + t2747;
  t1524 = t105*t1087*t1272;
  t1847 = t1772*t1790;
  t2223 = t1524 + t1847;
  t3453 = -1.*t105*t838*t918;
  t4116 = t849*t2854;
  t4129 = t3453 + t4116;
  t4191 = Cos(var1[13]);
  t4303 = -1.*t4191;
  t4312 = 1. + t4303;
  t4407 = Sin(var1[13]);
  t4497 = t849*t105*t838;
  t4500 = t918*t2854;
  t4519 = t4497 + t4500;
  t4941 = t929*t2223;
  t5001 = -1.*t977*t4129;
  t5028 = t4941 + t5001;
  t5065 = Cos(var1[14]);
  t5067 = -1.*t5065;
  t5144 = 1. + t5067;
  t5184 = Sin(var1[14]);
  t5243 = t4407*t4519;
  t5339 = t4191*t5028;
  t5362 = t5243 + t5339;
  t5465 = t4191*t4519;
  t5485 = -1.*t4407*t5028;
  t5503 = t5465 + t5485;
  t5516 = Cos(var1[15]);
  t5533 = -1.*t5516;
  t5543 = 1. + t5533;
  t5571 = Sin(var1[15]);
  t5610 = -1.*t5184*t5362;
  t5614 = t5065*t5503;
  t5615 = t5610 + t5614;
  t5684 = t5065*t5362;
  t5695 = t5184*t5503;
  t5704 = t5684 + t5695;
  t914 = -0.022225*t888;
  t920 = -0.086996*t918;
  t921 = 0. + t914 + t920;
  t975 = -0.31508*t953;
  t979 = 0.156996*t977;
  t1020 = 0. + t975 + t979;
  t2553 = -0.086996*t888;
  t2562 = 0.022225*t918;
  t2609 = 0. + t2553 + t2562;
  t3011 = -0.156996*t953;
  t3161 = -0.31508*t977;
  t3173 = 0. + t3011 + t3161;
  t5893 = t105*t1087;
  t5898 = t1772*t1272*t1790;
  t5912 = t5893 + t5898;
  t4392 = -0.022225*t4312;
  t4433 = 0.38008*t4407;
  t4480 = 0. + t4392 + t4433;
  t5828 = t1087*t1772*t1272;
  t5841 = -1.*t105*t1790;
  t5847 = t5828 + t5841;
  t5945 = -1.*t838*t918*t1772;
  t5950 = t849*t5912;
  t5953 = t5945 + t5950;
  t4880 = -0.38008*t4312;
  t4882 = -0.022225*t4407;
  t4934 = 0. + t4880 + t4882;
  t5147 = -0.86008*t5144;
  t5185 = -0.022225*t5184;
  t5213 = 0. + t5147 + t5185;
  t5964 = t849*t838*t1772;
  t5967 = t918*t5912;
  t5970 = t5964 + t5967;
  t6009 = t929*t5847;
  t6040 = -1.*t977*t5953;
  t6042 = t6009 + t6040;
  t5402 = -0.022225*t5144;
  t5417 = 0.86008*t5184;
  t5442 = 0. + t5402 + t5417;
  t5546 = -0.021147*t5543;
  t5577 = 1.34008*t5571;
  t5578 = 0. + t5546 + t5577;
  t6130 = t4407*t5970;
  t6138 = t4191*t6042;
  t6180 = t6130 + t6138;
  t6213 = t4191*t5970;
  t6229 = -1.*t4407*t6042;
  t6253 = t6213 + t6229;
  t5620 = -1.34008*t5543;
  t5649 = -0.021147*t5571;
  t5653 = 0. + t5620 + t5649;
  t6276 = -1.*t5184*t6180;
  t6309 = t5065*t6253;
  t6327 = t6276 + t6309;
  t6347 = t5065*t6180;
  t6351 = t5184*t6253;
  t6376 = t6347 + t6351;
  t6760 = t918*t1272;
  t6790 = t849*t838*t1790;
  t6840 = t6760 + t6790;
  t6876 = -1.*t849*t1272;
  t6877 = t838*t918*t1790;
  t6916 = t6876 + t6877;
  t7057 = t929*t838*t1087;
  t7105 = -1.*t977*t6840;
  t7119 = t7057 + t7105;
  t7141 = t4407*t6916;
  t7170 = t4191*t7119;
  t7181 = t7141 + t7170;
  t7192 = t4191*t6916;
  t7205 = -1.*t4407*t7119;
  t7218 = t7192 + t7205;
  t7236 = -1.*t5184*t7181;
  t7257 = t5065*t7218;
  t7258 = t7236 + t7257;
  t7264 = t5065*t7181;
  t7271 = t5184*t7218;
  t7272 = t7264 + t7271;

  p_output1(0)=0. + t1020*t2223 + t2609*t2854 + t3173*t4129 + t4480*t4519 + t4934*t5028 + t5213*t5362 + t5442*t5503 + t5578*t5615 + t5653*t5704 - 1.34008*(t5571*t5615 + t5516*t5704) - 0.021147*(t5516*t5615 - 1.*t5571*t5704) + t105*t838*t921 - 0.166996*(t4129*t929 + t2223*t977) + var1(0);
  p_output1(1)=0. + t1020*t5847 + t2609*t5912 + t3173*t5953 + t4480*t5970 + t4934*t6042 + t5213*t6180 + t5442*t6253 + t5578*t6327 + t5653*t6376 - 1.34008*(t5571*t6327 + t5516*t6376) - 0.021147*(t5516*t6327 - 1.*t5571*t6376) + t1772*t838*t921 - 0.166996*(t5953*t929 + t5847*t977) + var1(1);
  p_output1(2)=0. + t3173*t6840 + t4480*t6916 + t4934*t7119 + t5213*t7181 + t5442*t7218 + t5578*t7258 + t5653*t7272 - 1.34008*(t5571*t7258 + t5516*t7272) - 0.021147*(t5516*t7258 - 1.*t5571*t7272) + t1020*t1087*t838 + t1790*t2609*t838 - 1.*t1272*t921 - 0.166996*(t6840*t929 + t1087*t838*t977) + var1(2);
}


       
void p_rAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
