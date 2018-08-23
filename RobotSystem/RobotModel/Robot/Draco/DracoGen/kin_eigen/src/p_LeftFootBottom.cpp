/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:29 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootBottom.h"

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
  double t1260;
  double t1914;
  double t2013;
  double t2028;
  double t2479;
  double t481;
  double t550;
  double t803;
  double t1316;
  double t1317;
  double t1389;
  double t1443;
  double t3005;
  double t3742;
  double t3756;
  double t3781;
  double t4005;
  double t3606;
  double t3699;
  double t3739;
  double t4166;
  double t4170;
  double t4178;
  double t4717;
  double t4740;
  double t4744;
  double t5059;
  double t4665;
  double t4702;
  double t4712;
  double t5285;
  double t5294;
  double t5332;
  double t5542;
  double t5590;
  double t5596;
  double t5710;
  double t5993;
  double t5997;
  double t6010;
  double t6158;
  double t6191;
  double t6201;
  double t6224;
  double t6241;
  double t6244;
  double t6264;
  double t6292;
  double t6371;
  double t6377;
  double t6388;
  double t6391;
  double t6425;
  double t2371;
  double t2527;
  double t2729;
  double t3194;
  double t3424;
  double t3440;
  double t6579;
  double t6589;
  double t6591;
  double t3956;
  double t4030;
  double t4031;
  double t4179;
  double t4191;
  double t4193;
  double t6616;
  double t6655;
  double t6681;
  double t6694;
  double t6714;
  double t6745;
  double t4865;
  double t5068;
  double t5191;
  double t5357;
  double t5380;
  double t5428;
  double t5598;
  double t5931;
  double t5949;
  double t6782;
  double t6807;
  double t6808;
  double t6825;
  double t6829;
  double t6831;
  double t6070;
  double t6098;
  double t6157;
  double t6255;
  double t6270;
  double t6284;
  double t6860;
  double t6878;
  double t6902;
  double t6917;
  double t6923;
  double t6927;
  double t6381;
  double t6383;
  double t6387;
  double t6949;
  double t6958;
  double t6968;
  double t6987;
  double t6991;
  double t6995;
  double t7127;
  double t7143;
  double t7145;
  double t7195;
  double t7210;
  double t7220;
  double t7225;
  double t7228;
  double t7253;
  double t7256;
  double t7258;
  double t7260;
  double t7279;
  double t7281;
  double t7283;
  double t7288;
  double t7295;
  double t7309;
  double t7314;
  double t7320;
  double t7323;
  t1260 = Cos(var1[3]);
  t1914 = Cos(var1[6]);
  t2013 = -1.*t1914;
  t2028 = 1. + t2013;
  t2479 = Sin(var1[6]);
  t481 = Cos(var1[5]);
  t550 = Sin(var1[3]);
  t803 = -1.*t481*t550;
  t1316 = Sin(var1[4]);
  t1317 = Sin(var1[5]);
  t1389 = t1260*t1316*t1317;
  t1443 = t803 + t1389;
  t3005 = Cos(var1[4]);
  t3742 = Cos(var1[7]);
  t3756 = -1.*t3742;
  t3781 = 1. + t3756;
  t4005 = Sin(var1[7]);
  t3606 = t1914*t1443;
  t3699 = -1.*t1260*t3005*t2479;
  t3739 = t3606 + t3699;
  t4166 = t1260*t481*t1316;
  t4170 = t550*t1317;
  t4178 = t4166 + t4170;
  t4717 = Cos(var1[8]);
  t4740 = -1.*t4717;
  t4744 = 1. + t4740;
  t5059 = Sin(var1[8]);
  t4665 = t3742*t4178;
  t4702 = -1.*t3739*t4005;
  t4712 = t4665 + t4702;
  t5285 = t1260*t3005*t1914;
  t5294 = t1443*t2479;
  t5332 = t5285 + t5294;
  t5542 = Cos(var1[9]);
  t5590 = -1.*t5542;
  t5596 = 1. + t5590;
  t5710 = Sin(var1[9]);
  t5993 = t4717*t4712;
  t5997 = t5332*t5059;
  t6010 = t5993 + t5997;
  t6158 = t4717*t5332;
  t6191 = -1.*t4712*t5059;
  t6201 = t6158 + t6191;
  t6224 = Cos(var1[10]);
  t6241 = -1.*t6224;
  t6244 = 1. + t6241;
  t6264 = Sin(var1[10]);
  t6292 = -1.*t5710*t6010;
  t6371 = t5542*t6201;
  t6377 = t6292 + t6371;
  t6388 = t5542*t6010;
  t6391 = t5710*t6201;
  t6425 = t6388 + t6391;
  t2371 = 0.087*t2028;
  t2527 = 0.0222*t2479;
  t2729 = 0. + t2371 + t2527;
  t3194 = -0.0222*t2028;
  t3424 = 0.087*t2479;
  t3440 = 0. + t3194 + t3424;
  t6579 = t1260*t481;
  t6589 = t550*t1316*t1317;
  t6591 = t6579 + t6589;
  t3956 = 0.157*t3781;
  t4030 = -0.3151*t4005;
  t4031 = 0. + t3956 + t4030;
  t4179 = -0.3151*t3781;
  t4191 = -0.157*t4005;
  t4193 = 0. + t4179 + t4191;
  t6616 = t1914*t6591;
  t6655 = -1.*t3005*t550*t2479;
  t6681 = t6616 + t6655;
  t6694 = t481*t550*t1316;
  t6714 = -1.*t1260*t1317;
  t6745 = t6694 + t6714;
  t4865 = -0.3801*t4744;
  t5068 = -0.0222*t5059;
  t5191 = 0. + t4865 + t5068;
  t5357 = -0.0222*t4744;
  t5380 = 0.3801*t5059;
  t5428 = 0. + t5357 + t5380;
  t5598 = -0.8601*t5596;
  t5931 = -0.0222*t5710;
  t5949 = 0. + t5598 + t5931;
  t6782 = t3742*t6745;
  t6807 = -1.*t6681*t4005;
  t6808 = t6782 + t6807;
  t6825 = t3005*t1914*t550;
  t6829 = t6591*t2479;
  t6831 = t6825 + t6829;
  t6070 = -0.0222*t5596;
  t6098 = 0.8601*t5710;
  t6157 = 0. + t6070 + t6098;
  t6255 = -0.0211*t6244;
  t6270 = 1.3401*t6264;
  t6284 = 0. + t6255 + t6270;
  t6860 = t4717*t6808;
  t6878 = t6831*t5059;
  t6902 = t6860 + t6878;
  t6917 = t4717*t6831;
  t6923 = -1.*t6808*t5059;
  t6927 = t6917 + t6923;
  t6381 = -1.3401*t6244;
  t6383 = -0.0211*t6264;
  t6387 = 0. + t6381 + t6383;
  t6949 = -1.*t5710*t6902;
  t6958 = t5542*t6927;
  t6968 = t6949 + t6958;
  t6987 = t5542*t6902;
  t6991 = t5710*t6927;
  t6995 = t6987 + t6991;
  t7127 = t3005*t1914*t1317;
  t7143 = t1316*t2479;
  t7145 = t7127 + t7143;
  t7195 = t3005*t481*t3742;
  t7210 = -1.*t7145*t4005;
  t7220 = t7195 + t7210;
  t7225 = -1.*t1914*t1316;
  t7228 = t3005*t1317*t2479;
  t7253 = t7225 + t7228;
  t7256 = t4717*t7220;
  t7258 = t7253*t5059;
  t7260 = t7256 + t7258;
  t7279 = t4717*t7253;
  t7281 = -1.*t7220*t5059;
  t7283 = t7279 + t7281;
  t7288 = -1.*t5710*t7260;
  t7295 = t5542*t7283;
  t7309 = t7288 + t7295;
  t7314 = t5542*t7260;
  t7320 = t5710*t7283;
  t7323 = t7314 + t7320;

  p_output1(0)=0. + t1443*t2729 + t1260*t3005*t3440 + t3739*t4031 + 0.167*(t3739*t3742 + t4005*t4178) + t4178*t4193 + t4712*t5191 + t5332*t5428 + t5949*t6010 + t6157*t6201 + t6284*t6377 + t6387*t6425 - 1.325152*(t6264*t6377 + t6224*t6425) + 0.043912*(t6224*t6377 - 1.*t6264*t6425) + var1(0);
  p_output1(1)=0. + t3005*t3440*t550 + t2729*t6591 + t4031*t6681 + t4193*t6745 + 0.167*(t3742*t6681 + t4005*t6745) + t5191*t6808 + t5428*t6831 + t5949*t6902 + t6157*t6927 + t6284*t6968 + t6387*t6995 - 1.325152*(t6264*t6968 + t6224*t6995) + 0.043912*(t6224*t6968 - 1.*t6264*t6995) + var1(1);
  p_output1(2)=0. + t1317*t2729*t3005 - 1.*t1316*t3440 + t3005*t4193*t481 + t4031*t7145 + 0.167*(t3005*t4005*t481 + t3742*t7145) + t5191*t7220 + t5428*t7253 + t5949*t7260 + t6157*t7283 + t6284*t7309 + t6387*t7323 - 1.325152*(t6264*t7309 + t6224*t7323) + 0.043912*(t6224*t7309 - 1.*t6264*t7323) + var1(2);
}


       
void p_LeftFootBottom(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
