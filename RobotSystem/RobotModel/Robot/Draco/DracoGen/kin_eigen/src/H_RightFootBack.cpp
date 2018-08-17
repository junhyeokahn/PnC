/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:13 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootBack.h"

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
  double t468;
  double t710;
  double t727;
  double t723;
  double t733;
  double t481;
  double t527;
  double t467;
  double t726;
  double t792;
  double t990;
  double t1151;
  double t492;
  double t992;
  double t1015;
  double t432;
  double t1153;
  double t1170;
  double t1188;
  double t1192;
  double t1250;
  double t1267;
  double t1287;
  double t1307;
  double t1321;
  double t1382;
  double t1397;
  double t1447;
  double t1134;
  double t1412;
  double t1417;
  double t240;
  double t1470;
  double t1486;
  double t1487;
  double t1595;
  double t1427;
  double t1492;
  double t1496;
  double t209;
  double t1621;
  double t1899;
  double t1912;
  double t2260;
  double t2286;
  double t2307;
  double t2192;
  double t2309;
  double t2330;
  double t2345;
  double t2360;
  double t2400;
  double t2407;
  double t2411;
  double t2418;
  double t2448;
  double t2481;
  double t2575;
  double t2344;
  double t2589;
  double t2602;
  double t2621;
  double t2653;
  double t2688;
  double t2616;
  double t2692;
  double t2694;
  double t2703;
  double t2708;
  double t2709;
  double t2822;
  double t2837;
  double t2875;
  double t2910;
  double t2916;
  double t2919;
  double t2928;
  double t2934;
  double t2975;
  double t2906;
  double t2978;
  double t3021;
  double t3033;
  double t3066;
  double t3076;
  double t3029;
  double t3081;
  double t3179;
  double t3268;
  double t3294;
  double t3297;
  double t1589;
  double t1999;
  double t2701;
  double t2723;
  double t3201;
  double t3330;
  double t3953;
  double t3965;
  double t4037;
  double t4047;
  double t3383;
  double t3389;
  double t3394;
  double t4260;
  double t4273;
  double t4399;
  double t4400;
  double t4625;
  double t4627;
  double t3615;
  double t2074;
  double t2107;
  double t2133;
  double t3966;
  double t3991;
  double t3992;
  double t4078;
  double t4106;
  double t4110;
  double t4160;
  double t4182;
  double t4203;
  double t4230;
  double t4231;
  double t4232;
  double t4286;
  double t4299;
  double t4318;
  double t3412;
  double t3480;
  double t3508;
  double t4350;
  double t4367;
  double t4381;
  double t4485;
  double t4509;
  double t4576;
  double t4586;
  double t4592;
  double t4593;
  double t4636;
  double t4640;
  double t4663;
  double t4679;
  double t4701;
  double t4702;
  double t3791;
  double t2750;
  double t2755;
  double t2759;
  double t3578;
  double t3589;
  double t3591;
  double t3877;
  double t3333;
  double t3341;
  double t3370;
  t468 = Cos(var1[3]);
  t710 = Cos(var1[5]);
  t727 = Sin(var1[4]);
  t723 = Sin(var1[3]);
  t733 = Sin(var1[5]);
  t481 = Cos(var1[4]);
  t527 = Sin(var1[11]);
  t467 = Cos(var1[11]);
  t726 = -1.*t710*t723;
  t792 = t468*t727*t733;
  t990 = t726 + t792;
  t1151 = Cos(var1[13]);
  t492 = t467*t468*t481;
  t992 = t527*t990;
  t1015 = t492 + t992;
  t432 = Sin(var1[13]);
  t1153 = Cos(var1[12]);
  t1170 = t468*t710*t727;
  t1188 = t723*t733;
  t1192 = t1170 + t1188;
  t1250 = t1153*t1192;
  t1267 = Sin(var1[12]);
  t1287 = -1.*t468*t481*t527;
  t1307 = t467*t990;
  t1321 = t1287 + t1307;
  t1382 = -1.*t1267*t1321;
  t1397 = t1250 + t1382;
  t1447 = Cos(var1[14]);
  t1134 = t432*t1015;
  t1412 = t1151*t1397;
  t1417 = t1134 + t1412;
  t240 = Sin(var1[14]);
  t1470 = t1151*t1015;
  t1486 = -1.*t432*t1397;
  t1487 = t1470 + t1486;
  t1595 = Cos(var1[15]);
  t1427 = -1.*t240*t1417;
  t1492 = t1447*t1487;
  t1496 = t1427 + t1492;
  t209 = Sin(var1[15]);
  t1621 = t1447*t1417;
  t1899 = t240*t1487;
  t1912 = t1621 + t1899;
  t2260 = t468*t710;
  t2286 = t723*t727*t733;
  t2307 = t2260 + t2286;
  t2192 = t467*t481*t723;
  t2309 = t527*t2307;
  t2330 = t2192 + t2309;
  t2345 = t710*t723*t727;
  t2360 = -1.*t468*t733;
  t2400 = t2345 + t2360;
  t2407 = t1153*t2400;
  t2411 = -1.*t481*t527*t723;
  t2418 = t467*t2307;
  t2448 = t2411 + t2418;
  t2481 = -1.*t1267*t2448;
  t2575 = t2407 + t2481;
  t2344 = t432*t2330;
  t2589 = t1151*t2575;
  t2602 = t2344 + t2589;
  t2621 = t1151*t2330;
  t2653 = -1.*t432*t2575;
  t2688 = t2621 + t2653;
  t2616 = -1.*t240*t2602;
  t2692 = t1447*t2688;
  t2694 = t2616 + t2692;
  t2703 = t1447*t2602;
  t2708 = t240*t2688;
  t2709 = t2703 + t2708;
  t2822 = -1.*t467*t727;
  t2837 = t481*t527*t733;
  t2875 = t2822 + t2837;
  t2910 = t1153*t481*t710;
  t2916 = t527*t727;
  t2919 = t467*t481*t733;
  t2928 = t2916 + t2919;
  t2934 = -1.*t1267*t2928;
  t2975 = t2910 + t2934;
  t2906 = t432*t2875;
  t2978 = t1151*t2975;
  t3021 = t2906 + t2978;
  t3033 = t1151*t2875;
  t3066 = -1.*t432*t2975;
  t3076 = t3033 + t3066;
  t3029 = -1.*t240*t3021;
  t3081 = t1447*t3076;
  t3179 = t3029 + t3081;
  t3268 = t1447*t3021;
  t3294 = t240*t3076;
  t3297 = t3268 + t3294;
  t1589 = t209*t1496;
  t1999 = t1595*t1912;
  t2701 = t209*t2694;
  t2723 = t1595*t2709;
  t3201 = t209*t3179;
  t3330 = t1595*t3297;
  t3953 = -1.*t467;
  t3965 = 1. + t3953;
  t4037 = -1.*t1153;
  t4047 = 1. + t4037;
  t3383 = t1267*t1192;
  t3389 = t1153*t1321;
  t3394 = t3383 + t3389;
  t4260 = -1.*t1151;
  t4273 = 1. + t4260;
  t4399 = -1.*t1447;
  t4400 = 1. + t4399;
  t4625 = -1.*t1595;
  t4627 = 1. + t4625;
  t3615 = t1589 + t1999;
  t2074 = t1595*t1496;
  t2107 = -1.*t209*t1912;
  t2133 = t2074 + t2107;
  t3966 = -0.022225*t3965;
  t3991 = -0.086996*t527;
  t3992 = 0. + t3966 + t3991;
  t4078 = -0.31508*t4047;
  t4106 = 0.156996*t1267;
  t4110 = 0. + t4078 + t4106;
  t4160 = -0.086996*t3965;
  t4182 = 0.022225*t527;
  t4203 = 0. + t4160 + t4182;
  t4230 = -0.156996*t4047;
  t4231 = -0.31508*t1267;
  t4232 = 0. + t4230 + t4231;
  t4286 = -0.022225*t4273;
  t4299 = 0.38008*t432;
  t4318 = 0. + t4286 + t4299;
  t3412 = t1267*t2400;
  t3480 = t1153*t2448;
  t3508 = t3412 + t3480;
  t4350 = -0.38008*t4273;
  t4367 = -0.022225*t432;
  t4381 = 0. + t4350 + t4367;
  t4485 = -0.86008*t4400;
  t4509 = -0.022225*t240;
  t4576 = 0. + t4485 + t4509;
  t4586 = -0.022225*t4400;
  t4592 = 0.86008*t240;
  t4593 = 0. + t4586 + t4592;
  t4636 = -0.021147*t4627;
  t4640 = 1.34008*t209;
  t4663 = 0. + t4636 + t4640;
  t4679 = -1.34008*t4627;
  t4701 = -0.021147*t209;
  t4702 = 0. + t4679 + t4701;
  t3791 = t2701 + t2723;
  t2750 = t1595*t2694;
  t2755 = -1.*t209*t2709;
  t2759 = t2750 + t2755;
  t3578 = t481*t710*t1267;
  t3589 = t1153*t2928;
  t3591 = t3578 + t3589;
  t3877 = t3201 + t3330;
  t3333 = t1595*t3179;
  t3341 = -1.*t209*t3297;
  t3370 = t3333 + t3341;

  p_output1(0)=t1589 + t1999 + 0.000796*t2133;
  p_output1(1)=t2701 + t2723 + 0.000796*t2759;
  p_output1(2)=t3201 + t3330 + 0.000796*t3370;
  p_output1(3)=0.;
  p_output1(4)=t3394;
  p_output1(5)=t3508;
  p_output1(6)=t3591;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1496*t1595 + t1912*t209 + 0.000796*t3615;
  p_output1(9)=-1.*t1595*t2694 + t209*t2709 + 0.000796*t3791;
  p_output1(10)=-1.*t1595*t3179 + t209*t3297 + 0.000796*t3877;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043805*t2133 - 0.166996*t3394 - 1.400132*t3615 + t1192*t4110 + t1321*t4232 + t1015*t4318 + t1397*t4381 + t1417*t4576 + t1487*t4593 + t1496*t4663 + t1912*t4702 + t3992*t468*t481 + t4203*t990 + var1(0);
  p_output1(13)=0. + 0.043805*t2759 - 0.166996*t3508 - 1.400132*t3791 + t2400*t4110 + t2307*t4203 + t2448*t4232 + t2330*t4318 + t2575*t4381 + t2602*t4576 + t2688*t4593 + t2694*t4663 + t2709*t4702 + t3992*t481*t723 + var1(1);
  p_output1(14)=0. + 0.043805*t3370 - 0.166996*t3591 - 1.400132*t3877 + t2928*t4232 + t2875*t4318 + t2975*t4381 + t3021*t4576 + t3076*t4593 + t3179*t4663 + t3297*t4702 + t4110*t481*t710 - 1.*t3992*t727 + t4203*t481*t733 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
