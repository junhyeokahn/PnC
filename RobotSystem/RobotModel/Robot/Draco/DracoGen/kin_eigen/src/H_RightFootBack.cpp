/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:05 GMT-05:00
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
  double t587;
  double t810;
  double t998;
  double t831;
  double t1000;
  double t686;
  double t754;
  double t503;
  double t906;
  double t1011;
  double t1053;
  double t1272;
  double t743;
  double t1214;
  double t1228;
  double t231;
  double t1285;
  double t1288;
  double t1319;
  double t1329;
  double t1402;
  double t1403;
  double t1446;
  double t1458;
  double t1485;
  double t1490;
  double t1519;
  double t1596;
  double t1245;
  double t1522;
  double t1526;
  double t122;
  double t1633;
  double t1653;
  double t1728;
  double t1923;
  double t1573;
  double t1742;
  double t1757;
  double t52;
  double t1924;
  double t1988;
  double t2188;
  double t2361;
  double t2375;
  double t2455;
  double t2326;
  double t2468;
  double t2469;
  double t2538;
  double t2543;
  double t2548;
  double t2555;
  double t2567;
  double t2578;
  double t2588;
  double t2610;
  double t2644;
  double t2492;
  double t2667;
  double t2700;
  double t2810;
  double t2871;
  double t2876;
  double t2733;
  double t2899;
  double t2905;
  double t2978;
  double t2988;
  double t3034;
  double t3165;
  double t3190;
  double t3196;
  double t3205;
  double t3252;
  double t3256;
  double t3264;
  double t3349;
  double t3358;
  double t3202;
  double t3362;
  double t3368;
  double t3377;
  double t3384;
  double t3390;
  double t3373;
  double t3396;
  double t3404;
  double t3469;
  double t3475;
  double t3497;
  double t1899;
  double t2195;
  double t2937;
  double t3039;
  double t3422;
  double t3510;
  double t4127;
  double t4172;
  double t4308;
  double t4331;
  double t3655;
  double t3688;
  double t3712;
  double t4563;
  double t4569;
  double t4747;
  double t4749;
  double t4851;
  double t4854;
  double t3812;
  double t2212;
  double t2235;
  double t2250;
  double t4204;
  double t4235;
  double t4263;
  double t4340;
  double t4346;
  double t4440;
  double t4450;
  double t4474;
  double t4479;
  double t4508;
  double t4518;
  double t4524;
  double t4572;
  double t4586;
  double t4626;
  double t3722;
  double t3733;
  double t3736;
  double t4678;
  double t4722;
  double t4728;
  double t4761;
  double t4795;
  double t4809;
  double t4812;
  double t4813;
  double t4848;
  double t4867;
  double t4880;
  double t4889;
  double t4919;
  double t4940;
  double t4941;
  double t3880;
  double t3055;
  double t3062;
  double t3074;
  double t3751;
  double t3759;
  double t3760;
  double t4014;
  double t3593;
  double t3611;
  double t3639;
  t587 = Cos(var1[3]);
  t810 = Cos(var1[5]);
  t998 = Sin(var1[4]);
  t831 = Sin(var1[3]);
  t1000 = Sin(var1[5]);
  t686 = Cos(var1[4]);
  t754 = Sin(var1[11]);
  t503 = Cos(var1[11]);
  t906 = -1.*t810*t831;
  t1011 = t587*t998*t1000;
  t1053 = t906 + t1011;
  t1272 = Cos(var1[13]);
  t743 = t503*t587*t686;
  t1214 = t754*t1053;
  t1228 = t743 + t1214;
  t231 = Sin(var1[13]);
  t1285 = Cos(var1[12]);
  t1288 = t587*t810*t998;
  t1319 = t831*t1000;
  t1329 = t1288 + t1319;
  t1402 = t1285*t1329;
  t1403 = Sin(var1[12]);
  t1446 = -1.*t587*t686*t754;
  t1458 = t503*t1053;
  t1485 = t1446 + t1458;
  t1490 = -1.*t1403*t1485;
  t1519 = t1402 + t1490;
  t1596 = Cos(var1[14]);
  t1245 = t231*t1228;
  t1522 = t1272*t1519;
  t1526 = t1245 + t1522;
  t122 = Sin(var1[14]);
  t1633 = t1272*t1228;
  t1653 = -1.*t231*t1519;
  t1728 = t1633 + t1653;
  t1923 = Cos(var1[15]);
  t1573 = -1.*t122*t1526;
  t1742 = t1596*t1728;
  t1757 = t1573 + t1742;
  t52 = Sin(var1[15]);
  t1924 = t1596*t1526;
  t1988 = t122*t1728;
  t2188 = t1924 + t1988;
  t2361 = t587*t810;
  t2375 = t831*t998*t1000;
  t2455 = t2361 + t2375;
  t2326 = t503*t686*t831;
  t2468 = t754*t2455;
  t2469 = t2326 + t2468;
  t2538 = t810*t831*t998;
  t2543 = -1.*t587*t1000;
  t2548 = t2538 + t2543;
  t2555 = t1285*t2548;
  t2567 = -1.*t686*t754*t831;
  t2578 = t503*t2455;
  t2588 = t2567 + t2578;
  t2610 = -1.*t1403*t2588;
  t2644 = t2555 + t2610;
  t2492 = t231*t2469;
  t2667 = t1272*t2644;
  t2700 = t2492 + t2667;
  t2810 = t1272*t2469;
  t2871 = -1.*t231*t2644;
  t2876 = t2810 + t2871;
  t2733 = -1.*t122*t2700;
  t2899 = t1596*t2876;
  t2905 = t2733 + t2899;
  t2978 = t1596*t2700;
  t2988 = t122*t2876;
  t3034 = t2978 + t2988;
  t3165 = -1.*t503*t998;
  t3190 = t686*t754*t1000;
  t3196 = t3165 + t3190;
  t3205 = t1285*t686*t810;
  t3252 = t754*t998;
  t3256 = t503*t686*t1000;
  t3264 = t3252 + t3256;
  t3349 = -1.*t1403*t3264;
  t3358 = t3205 + t3349;
  t3202 = t231*t3196;
  t3362 = t1272*t3358;
  t3368 = t3202 + t3362;
  t3377 = t1272*t3196;
  t3384 = -1.*t231*t3358;
  t3390 = t3377 + t3384;
  t3373 = -1.*t122*t3368;
  t3396 = t1596*t3390;
  t3404 = t3373 + t3396;
  t3469 = t1596*t3368;
  t3475 = t122*t3390;
  t3497 = t3469 + t3475;
  t1899 = t52*t1757;
  t2195 = t1923*t2188;
  t2937 = t52*t2905;
  t3039 = t1923*t3034;
  t3422 = t52*t3404;
  t3510 = t1923*t3497;
  t4127 = -1.*t503;
  t4172 = 1. + t4127;
  t4308 = -1.*t1285;
  t4331 = 1. + t4308;
  t3655 = t1403*t1329;
  t3688 = t1285*t1485;
  t3712 = t3655 + t3688;
  t4563 = -1.*t1272;
  t4569 = 1. + t4563;
  t4747 = -1.*t1596;
  t4749 = 1. + t4747;
  t4851 = -1.*t1923;
  t4854 = 1. + t4851;
  t3812 = t1899 + t2195;
  t2212 = t1923*t1757;
  t2235 = -1.*t52*t2188;
  t2250 = t2212 + t2235;
  t4204 = -0.022225*t4172;
  t4235 = -0.086996*t754;
  t4263 = 0. + t4204 + t4235;
  t4340 = -0.31508*t4331;
  t4346 = 0.156996*t1403;
  t4440 = 0. + t4340 + t4346;
  t4450 = -0.086996*t4172;
  t4474 = 0.022225*t754;
  t4479 = 0. + t4450 + t4474;
  t4508 = -0.156996*t4331;
  t4518 = -0.31508*t1403;
  t4524 = 0. + t4508 + t4518;
  t4572 = -0.022225*t4569;
  t4586 = 0.38008*t231;
  t4626 = 0. + t4572 + t4586;
  t3722 = t1403*t2548;
  t3733 = t1285*t2588;
  t3736 = t3722 + t3733;
  t4678 = -0.38008*t4569;
  t4722 = -0.022225*t231;
  t4728 = 0. + t4678 + t4722;
  t4761 = -0.86008*t4749;
  t4795 = -0.022225*t122;
  t4809 = 0. + t4761 + t4795;
  t4812 = -0.022225*t4749;
  t4813 = 0.86008*t122;
  t4848 = 0. + t4812 + t4813;
  t4867 = -0.021147*t4854;
  t4880 = 1.34008*t52;
  t4889 = 0. + t4867 + t4880;
  t4919 = -1.34008*t4854;
  t4940 = -0.021147*t52;
  t4941 = 0. + t4919 + t4940;
  t3880 = t2937 + t3039;
  t3055 = t1923*t2905;
  t3062 = -1.*t52*t3034;
  t3074 = t3055 + t3062;
  t3751 = t686*t810*t1403;
  t3759 = t1285*t3264;
  t3760 = t3751 + t3759;
  t4014 = t3422 + t3510;
  t3593 = t1923*t3404;
  t3611 = -1.*t52*t3497;
  t3639 = t3593 + t3611;

  p_output1(0)=t1899 + t2195 + 0.000796*t2250;
  p_output1(1)=t2937 + t3039 + 0.000796*t3074;
  p_output1(2)=t3422 + t3510 + 0.000796*t3639;
  p_output1(3)=0.;
  p_output1(4)=t3712;
  p_output1(5)=t3736;
  p_output1(6)=t3760;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1757*t1923 + 0.000796*t3812 + t2188*t52;
  p_output1(9)=-1.*t1923*t2905 + 0.000796*t3880 + t3034*t52;
  p_output1(10)=-1.*t1923*t3404 + 0.000796*t4014 + t3497*t52;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043805*t2250 - 0.166996*t3712 - 1.400132*t3812 + t1329*t4440 + t1053*t4479 + t1485*t4524 + t1228*t4626 + t1519*t4728 + t1526*t4809 + t1728*t4848 + t1757*t4889 + t2188*t4941 + t4263*t587*t686 + var1(0);
  p_output1(13)=0. + 0.043805*t3074 - 0.166996*t3736 - 1.400132*t3880 + t2548*t4440 + t2455*t4479 + t2588*t4524 + t2469*t4626 + t2644*t4728 + t2700*t4809 + t2876*t4848 + t2905*t4889 + t3034*t4941 + t4263*t686*t831 + var1(1);
  p_output1(14)=0. + 0.043805*t3639 - 0.166996*t3760 - 1.400132*t4014 + t3264*t4524 + t3196*t4626 + t3358*t4728 + t3368*t4809 + t3390*t4848 + t3404*t4889 + t3497*t4941 + t1000*t4479*t686 + t4440*t686*t810 - 1.*t4263*t998 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
