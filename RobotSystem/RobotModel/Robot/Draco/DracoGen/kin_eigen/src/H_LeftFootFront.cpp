/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:06 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootFront.h"

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
  double t585;
  double t859;
  double t550;
  double t756;
  double t1054;
  double t1537;
  double t1314;
  double t1405;
  double t1468;
  double t1526;
  double t1572;
  double t411;
  double t1720;
  double t1838;
  double t1859;
  double t497;
  double t778;
  double t1064;
  double t1224;
  double t1260;
  double t1533;
  double t1590;
  double t1640;
  double t1647;
  double t1663;
  double t1673;
  double t1948;
  double t2056;
  double t1699;
  double t1952;
  double t1971;
  double t367;
  double t2093;
  double t2171;
  double t2181;
  double t2334;
  double t2040;
  double t2212;
  double t2220;
  double t198;
  double t2368;
  double t2371;
  double t2404;
  double t2664;
  double t2670;
  double t2716;
  double t3056;
  double t3139;
  double t3168;
  double t2555;
  double t2580;
  double t2622;
  double t2659;
  double t2783;
  double t2812;
  double t2924;
  double t2968;
  double t2982;
  double t3022;
  double t3170;
  double t3208;
  double t3271;
  double t3280;
  double t3349;
  double t3220;
  double t3400;
  double t3438;
  double t3481;
  double t3515;
  double t3532;
  double t3738;
  double t3752;
  double t3763;
  double t3671;
  double t3676;
  double t3678;
  double t3693;
  double t3699;
  double t3703;
  double t3727;
  double t3770;
  double t3803;
  double t3827;
  double t3829;
  double t3839;
  double t3809;
  double t3853;
  double t3862;
  double t3909;
  double t3993;
  double t4003;
  double t2224;
  double t2406;
  double t3472;
  double t3543;
  double t3863;
  double t4020;
  double t4918;
  double t4922;
  double t5157;
  double t5160;
  double t4415;
  double t4432;
  double t4459;
  double t5239;
  double t5249;
  double t5339;
  double t5342;
  double t5444;
  double t5464;
  double t4735;
  double t2441;
  double t2449;
  double t2500;
  double t4968;
  double t4980;
  double t4992;
  double t5088;
  double t5090;
  double t5099;
  double t5161;
  double t5162;
  double t5164;
  double t5168;
  double t5171;
  double t5191;
  double t4482;
  double t4507;
  double t4542;
  double t5262;
  double t5265;
  double t5287;
  double t5308;
  double t5311;
  double t5333;
  double t5344;
  double t5350;
  double t5369;
  double t5393;
  double t5401;
  double t5413;
  double t5466;
  double t5469;
  double t5508;
  double t5546;
  double t5620;
  double t5623;
  double t4793;
  double t3550;
  double t3555;
  double t3558;
  double t4560;
  double t4569;
  double t4597;
  double t4868;
  double t4074;
  double t4098;
  double t4181;
  t585 = Cos(var1[5]);
  t859 = Sin(var1[3]);
  t550 = Cos(var1[3]);
  t756 = Sin(var1[4]);
  t1054 = Sin(var1[5]);
  t1537 = Cos(var1[4]);
  t1314 = Cos(var1[6]);
  t1405 = -1.*t585*t859;
  t1468 = t550*t756*t1054;
  t1526 = t1405 + t1468;
  t1572 = Sin(var1[6]);
  t411 = Cos(var1[8]);
  t1720 = t550*t1537*t1314;
  t1838 = t1526*t1572;
  t1859 = t1720 + t1838;
  t497 = Cos(var1[7]);
  t778 = t550*t585*t756;
  t1064 = t859*t1054;
  t1224 = t778 + t1064;
  t1260 = t497*t1224;
  t1533 = t1314*t1526;
  t1590 = -1.*t550*t1537*t1572;
  t1640 = t1533 + t1590;
  t1647 = Sin(var1[7]);
  t1663 = -1.*t1640*t1647;
  t1673 = t1260 + t1663;
  t1948 = Sin(var1[8]);
  t2056 = Cos(var1[9]);
  t1699 = t411*t1673;
  t1952 = t1859*t1948;
  t1971 = t1699 + t1952;
  t367 = Sin(var1[9]);
  t2093 = t411*t1859;
  t2171 = -1.*t1673*t1948;
  t2181 = t2093 + t2171;
  t2334 = Cos(var1[10]);
  t2040 = -1.*t367*t1971;
  t2212 = t2056*t2181;
  t2220 = t2040 + t2212;
  t198 = Sin(var1[10]);
  t2368 = t2056*t1971;
  t2371 = t367*t2181;
  t2404 = t2368 + t2371;
  t2664 = t550*t585;
  t2670 = t859*t756*t1054;
  t2716 = t2664 + t2670;
  t3056 = t1537*t1314*t859;
  t3139 = t2716*t1572;
  t3168 = t3056 + t3139;
  t2555 = t585*t859*t756;
  t2580 = -1.*t550*t1054;
  t2622 = t2555 + t2580;
  t2659 = t497*t2622;
  t2783 = t1314*t2716;
  t2812 = -1.*t1537*t859*t1572;
  t2924 = t2783 + t2812;
  t2968 = -1.*t2924*t1647;
  t2982 = t2659 + t2968;
  t3022 = t411*t2982;
  t3170 = t3168*t1948;
  t3208 = t3022 + t3170;
  t3271 = t411*t3168;
  t3280 = -1.*t2982*t1948;
  t3349 = t3271 + t3280;
  t3220 = -1.*t367*t3208;
  t3400 = t2056*t3349;
  t3438 = t3220 + t3400;
  t3481 = t2056*t3208;
  t3515 = t367*t3349;
  t3532 = t3481 + t3515;
  t3738 = -1.*t1314*t756;
  t3752 = t1537*t1054*t1572;
  t3763 = t3738 + t3752;
  t3671 = t1537*t585*t497;
  t3676 = t1537*t1314*t1054;
  t3678 = t756*t1572;
  t3693 = t3676 + t3678;
  t3699 = -1.*t3693*t1647;
  t3703 = t3671 + t3699;
  t3727 = t411*t3703;
  t3770 = t3763*t1948;
  t3803 = t3727 + t3770;
  t3827 = t411*t3763;
  t3829 = -1.*t3703*t1948;
  t3839 = t3827 + t3829;
  t3809 = -1.*t367*t3803;
  t3853 = t2056*t3839;
  t3862 = t3809 + t3853;
  t3909 = t2056*t3803;
  t3993 = t367*t3839;
  t4003 = t3909 + t3993;
  t2224 = t198*t2220;
  t2406 = t2334*t2404;
  t3472 = t198*t3438;
  t3543 = t2334*t3532;
  t3863 = t198*t3862;
  t4020 = t2334*t4003;
  t4918 = -1.*t1314;
  t4922 = 1. + t4918;
  t5157 = -1.*t497;
  t5160 = 1. + t5157;
  t4415 = t497*t1640;
  t4432 = t1224*t1647;
  t4459 = t4415 + t4432;
  t5239 = -1.*t411;
  t5249 = 1. + t5239;
  t5339 = -1.*t2056;
  t5342 = 1. + t5339;
  t5444 = -1.*t2334;
  t5464 = 1. + t5444;
  t4735 = t2224 + t2406;
  t2441 = t2334*t2220;
  t2449 = -1.*t198*t2404;
  t2500 = t2441 + t2449;
  t4968 = 0.087004*t4922;
  t4980 = 0.022225*t1572;
  t4992 = 0. + t4968 + t4980;
  t5088 = -0.022225*t4922;
  t5090 = 0.087004*t1572;
  t5099 = 0. + t5088 + t5090;
  t5161 = 0.157004*t5160;
  t5162 = -0.31508*t1647;
  t5164 = 0. + t5161 + t5162;
  t5168 = -0.31508*t5160;
  t5171 = -0.157004*t1647;
  t5191 = 0. + t5168 + t5171;
  t4482 = t497*t2924;
  t4507 = t2622*t1647;
  t4542 = t4482 + t4507;
  t5262 = -0.38008*t5249;
  t5265 = -0.022225*t1948;
  t5287 = 0. + t5262 + t5265;
  t5308 = -0.022225*t5249;
  t5311 = 0.38008*t1948;
  t5333 = 0. + t5308 + t5311;
  t5344 = -0.86008*t5342;
  t5350 = -0.022225*t367;
  t5369 = 0. + t5344 + t5350;
  t5393 = -0.022225*t5342;
  t5401 = 0.86008*t367;
  t5413 = 0. + t5393 + t5401;
  t5466 = -0.021147*t5464;
  t5469 = 1.34008*t198;
  t5508 = 0. + t5466 + t5469;
  t5546 = -1.34008*t5464;
  t5620 = -0.021147*t198;
  t5623 = 0. + t5546 + t5620;
  t4793 = t3472 + t3543;
  t3550 = t2334*t3438;
  t3555 = -1.*t198*t3532;
  t3558 = t3550 + t3555;
  t4560 = t497*t3693;
  t4569 = t1537*t585*t1647;
  t4597 = t4560 + t4569;
  t4868 = t3863 + t4020;
  t4074 = t2334*t3862;
  t4098 = -1.*t198*t4003;
  t4181 = t4074 + t4098;

  p_output1(0)=t2224 + t2406 + 0.000796*t2500;
  p_output1(1)=t3472 + t3543 + 0.000796*t3558;
  p_output1(2)=t3863 + t4020 + 0.000796*t4181;
  p_output1(3)=0.;
  p_output1(4)=t4459;
  p_output1(5)=t4542;
  p_output1(6)=t4597;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2220*t2334 + t198*t2404 + 0.000796*t4735;
  p_output1(9)=-1.*t2334*t3438 + t198*t3532 + 0.000796*t4793;
  p_output1(10)=-1.*t2334*t3862 + t198*t4003 + 0.000796*t4868;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043925*t2500 + 0.167004*t4459 - 1.250132*t4735 + t1526*t4992 + t1640*t5164 + t1224*t5191 + t1673*t5287 + t1859*t5333 + t1971*t5369 + t2181*t5413 + t1537*t5099*t550 + t2220*t5508 + t2404*t5623 + var1(0);
  p_output1(13)=0. + 0.043925*t3558 + 0.167004*t4542 - 1.250132*t4793 + t2716*t4992 + t2924*t5164 + t2622*t5191 + t2982*t5287 + t3168*t5333 + t3208*t5369 + t3349*t5413 + t3438*t5508 + t3532*t5623 + t1537*t5099*t859 + var1(1);
  p_output1(14)=0. + 0.043925*t4181 + 0.167004*t4597 - 1.250132*t4868 + t1054*t1537*t4992 + t3693*t5164 + t3703*t5287 + t3763*t5333 + t3803*t5369 + t3839*t5413 + t3862*t5508 + t4003*t5623 + t1537*t5191*t585 - 1.*t5099*t756 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
