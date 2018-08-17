/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:58 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootBack.h"

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
  double t485;
  double t1507;
  double t1730;
  double t1794;
  double t1812;
  double t84;
  double t173;
  double t461;
  double t1116;
  double t1329;
  double t1363;
  double t1392;
  double t2038;
  double t2925;
  double t2955;
  double t2956;
  double t3057;
  double t2728;
  double t2883;
  double t2900;
  double t3139;
  double t3147;
  double t3214;
  double t4089;
  double t4128;
  double t4157;
  double t4165;
  double t3878;
  double t3991;
  double t4058;
  double t4211;
  double t4254;
  double t4255;
  double t4465;
  double t4501;
  double t4578;
  double t4716;
  double t4845;
  double t4928;
  double t4938;
  double t5066;
  double t5079;
  double t5121;
  double t5150;
  double t5151;
  double t5194;
  double t5305;
  double t5362;
  double t5364;
  double t5365;
  double t5420;
  double t5421;
  double t5433;
  double t1811;
  double t1893;
  double t1944;
  double t2101;
  double t2429;
  double t2458;
  double t5565;
  double t5567;
  double t5575;
  double t3044;
  double t3092;
  double t3097;
  double t3289;
  double t3299;
  double t3392;
  double t5616;
  double t5641;
  double t5649;
  double t5663;
  double t5669;
  double t5684;
  double t4158;
  double t4188;
  double t4194;
  double t4296;
  double t4381;
  double t4383;
  double t4664;
  double t4769;
  double t4801;
  double t5821;
  double t5845;
  double t5860;
  double t5921;
  double t5943;
  double t5961;
  double t5002;
  double t5015;
  double t5051;
  double t5198;
  double t5333;
  double t5356;
  double t5975;
  double t5981;
  double t5988;
  double t6007;
  double t6030;
  double t6042;
  double t5401;
  double t5403;
  double t5418;
  double t6068;
  double t6084;
  double t6089;
  double t6125;
  double t6127;
  double t6130;
  double t6400;
  double t6403;
  double t6438;
  double t6506;
  double t6534;
  double t6560;
  double t6568;
  double t6575;
  double t6579;
  double t6610;
  double t6615;
  double t6620;
  double t6623;
  double t6631;
  double t6633;
  double t6656;
  double t6658;
  double t6661;
  double t6665;
  double t6695;
  double t6699;
  t485 = Cos(var1[3]);
  t1507 = Cos(var1[6]);
  t1730 = -1.*t1507;
  t1794 = 1. + t1730;
  t1812 = Sin(var1[6]);
  t84 = Cos(var1[5]);
  t173 = Sin(var1[3]);
  t461 = -1.*t84*t173;
  t1116 = Sin(var1[4]);
  t1329 = Sin(var1[5]);
  t1363 = t485*t1116*t1329;
  t1392 = t461 + t1363;
  t2038 = Cos(var1[4]);
  t2925 = Cos(var1[7]);
  t2955 = -1.*t2925;
  t2956 = 1. + t2955;
  t3057 = Sin(var1[7]);
  t2728 = t1507*t1392;
  t2883 = -1.*t485*t2038*t1812;
  t2900 = t2728 + t2883;
  t3139 = t485*t84*t1116;
  t3147 = t173*t1329;
  t3214 = t3139 + t3147;
  t4089 = Cos(var1[8]);
  t4128 = -1.*t4089;
  t4157 = 1. + t4128;
  t4165 = Sin(var1[8]);
  t3878 = t2925*t3214;
  t3991 = -1.*t2900*t3057;
  t4058 = t3878 + t3991;
  t4211 = t485*t2038*t1507;
  t4254 = t1392*t1812;
  t4255 = t4211 + t4254;
  t4465 = Cos(var1[9]);
  t4501 = -1.*t4465;
  t4578 = 1. + t4501;
  t4716 = Sin(var1[9]);
  t4845 = t4089*t4058;
  t4928 = t4255*t4165;
  t4938 = t4845 + t4928;
  t5066 = t4089*t4255;
  t5079 = -1.*t4058*t4165;
  t5121 = t5066 + t5079;
  t5150 = Cos(var1[10]);
  t5151 = -1.*t5150;
  t5194 = 1. + t5151;
  t5305 = Sin(var1[10]);
  t5362 = -1.*t4716*t4938;
  t5364 = t4465*t5121;
  t5365 = t5362 + t5364;
  t5420 = t4465*t4938;
  t5421 = t4716*t5121;
  t5433 = t5420 + t5421;
  t1811 = 0.087004*t1794;
  t1893 = 0.022225*t1812;
  t1944 = 0. + t1811 + t1893;
  t2101 = -0.022225*t1794;
  t2429 = 0.087004*t1812;
  t2458 = 0. + t2101 + t2429;
  t5565 = t485*t84;
  t5567 = t173*t1116*t1329;
  t5575 = t5565 + t5567;
  t3044 = 0.157004*t2956;
  t3092 = -0.31508*t3057;
  t3097 = 0. + t3044 + t3092;
  t3289 = -0.31508*t2956;
  t3299 = -0.157004*t3057;
  t3392 = 0. + t3289 + t3299;
  t5616 = t1507*t5575;
  t5641 = -1.*t2038*t173*t1812;
  t5649 = t5616 + t5641;
  t5663 = t84*t173*t1116;
  t5669 = -1.*t485*t1329;
  t5684 = t5663 + t5669;
  t4158 = -0.38008*t4157;
  t4188 = -0.022225*t4165;
  t4194 = 0. + t4158 + t4188;
  t4296 = -0.022225*t4157;
  t4381 = 0.38008*t4165;
  t4383 = 0. + t4296 + t4381;
  t4664 = -0.86008*t4578;
  t4769 = -0.022225*t4716;
  t4801 = 0. + t4664 + t4769;
  t5821 = t2925*t5684;
  t5845 = -1.*t5649*t3057;
  t5860 = t5821 + t5845;
  t5921 = t2038*t1507*t173;
  t5943 = t5575*t1812;
  t5961 = t5921 + t5943;
  t5002 = -0.022225*t4578;
  t5015 = 0.86008*t4716;
  t5051 = 0. + t5002 + t5015;
  t5198 = -0.021147*t5194;
  t5333 = 1.34008*t5305;
  t5356 = 0. + t5198 + t5333;
  t5975 = t4089*t5860;
  t5981 = t5961*t4165;
  t5988 = t5975 + t5981;
  t6007 = t4089*t5961;
  t6030 = -1.*t5860*t4165;
  t6042 = t6007 + t6030;
  t5401 = -1.34008*t5194;
  t5403 = -0.021147*t5305;
  t5418 = 0. + t5401 + t5403;
  t6068 = -1.*t4716*t5988;
  t6084 = t4465*t6042;
  t6089 = t6068 + t6084;
  t6125 = t4465*t5988;
  t6127 = t4716*t6042;
  t6130 = t6125 + t6127;
  t6400 = t2038*t1507*t1329;
  t6403 = t1116*t1812;
  t6438 = t6400 + t6403;
  t6506 = t2038*t84*t2925;
  t6534 = -1.*t6438*t3057;
  t6560 = t6506 + t6534;
  t6568 = -1.*t1507*t1116;
  t6575 = t2038*t1329*t1812;
  t6579 = t6568 + t6575;
  t6610 = t4089*t6560;
  t6615 = t6579*t4165;
  t6620 = t6610 + t6615;
  t6623 = t4089*t6579;
  t6631 = -1.*t6560*t4165;
  t6633 = t6623 + t6631;
  t6656 = -1.*t4716*t6620;
  t6658 = t4465*t6633;
  t6661 = t6656 + t6658;
  t6665 = t4465*t6620;
  t6695 = t4716*t6633;
  t6699 = t6665 + t6695;

  p_output1(0)=0. + t1392*t1944 + t2900*t3097 + 0.167004*(t2900*t2925 + t3057*t3214) + t3214*t3392 + t4058*t4194 + t4255*t4383 + t2038*t2458*t485 + t4801*t4938 + t5051*t5121 + t5356*t5365 + t5418*t5433 - 1.400132*(t5305*t5365 + t5150*t5433) + 0.043805*(t5150*t5365 - 1.*t5305*t5433) + var1(0);
  p_output1(1)=0. + t173*t2038*t2458 + t1944*t5575 + t3097*t5649 + t3392*t5684 + 0.167004*(t2925*t5649 + t3057*t5684) + t4194*t5860 + t4383*t5961 + t4801*t5988 + t5051*t6042 + t5356*t6089 + t5418*t6130 - 1.400132*(t5305*t6089 + t5150*t6130) + 0.043805*(t5150*t6089 - 1.*t5305*t6130) + var1(1);
  p_output1(2)=0. + t1329*t1944*t2038 - 1.*t1116*t2458 + t3097*t6438 + t4194*t6560 + t4383*t6579 + t4801*t6620 + t5051*t6633 + t5356*t6661 + t5418*t6699 - 1.400132*(t5305*t6661 + t5150*t6699) + 0.043805*(t5150*t6661 - 1.*t5305*t6699) + t2038*t3392*t84 + 0.167004*(t2925*t6438 + t2038*t3057*t84) + var1(2);
}


       
void p_LeftFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
