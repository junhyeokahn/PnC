/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:33 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootBack.h"

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
  double t485;
  double t374;
  double t489;
  double t375;
  double t508;
  double t122;
  double t351;
  double t628;
  double t645;
  double t947;
  double t856;
  double t899;
  double t945;
  double t463;
  double t513;
  double t548;
  double t603;
  double t715;
  double t750;
  double t1846;
  double t1938;
  double t1945;
  double t2020;
  double t1883;
  double t1892;
  double t1905;
  double t2088;
  double t2299;
  double t1912;
  double t2192;
  double t2258;
  double t1836;
  double t2337;
  double t2377;
  double t2451;
  double t2564;
  double t2262;
  double t2529;
  double t2545;
  double t1797;
  double t2582;
  double t2600;
  double t2614;
  double t1410;
  double t1477;
  double t1483;
  double t1023;
  double t1076;
  double t1111;
  double t1119;
  double t1204;
  double t1330;
  double t2808;
  double t2824;
  double t2873;
  double t2721;
  double t2731;
  double t2759;
  double t2792;
  double t2907;
  double t2948;
  double t2994;
  double t3021;
  double t3039;
  double t2970;
  double t3123;
  double t3126;
  double t3170;
  double t3176;
  double t3179;
  double t1659;
  double t1693;
  double t1731;
  double t3323;
  double t3330;
  double t3336;
  double t3255;
  double t3268;
  double t3291;
  double t3301;
  double t3361;
  double t3388;
  double t3463;
  double t3493;
  double t3514;
  double t3392;
  double t3533;
  double t3534;
  double t3575;
  double t3580;
  double t3582;
  double t4176;
  double t4205;
  double t4349;
  double t4391;
  double t771;
  double t948;
  double t987;
  double t4669;
  double t4671;
  double t4761;
  double t4762;
  double t4897;
  double t4903;
  double t3736;
  double t3744;
  double t3783;
  double t2660;
  double t2667;
  double t2682;
  double t4232;
  double t4233;
  double t4249;
  double t4323;
  double t4334;
  double t4336;
  double t4400;
  double t4454;
  double t4541;
  double t4620;
  double t4634;
  double t4642;
  double t1358;
  double t1574;
  double t1640;
  double t4672;
  double t4677;
  double t4679;
  double t4710;
  double t4716;
  double t4726;
  double t4785;
  double t4808;
  double t4814;
  double t4849;
  double t4853;
  double t4857;
  double t4911;
  double t4917;
  double t4931;
  double t4952;
  double t4954;
  double t4955;
  double t3875;
  double t3879;
  double t3926;
  double t3214;
  double t3224;
  double t3226;
  double t1749;
  double t1765;
  double t1777;
  double t4043;
  double t4066;
  double t4085;
  double t3602;
  double t3612;
  double t3614;
  t485 = Cos(var1[3]);
  t374 = Cos(var1[5]);
  t489 = Sin(var1[4]);
  t375 = Sin(var1[3]);
  t508 = Sin(var1[5]);
  t122 = Cos(var1[7]);
  t351 = Cos(var1[6]);
  t628 = Cos(var1[4]);
  t645 = Sin(var1[6]);
  t947 = Sin(var1[7]);
  t856 = t485*t374*t489;
  t899 = t375*t508;
  t945 = t856 + t899;
  t463 = -1.*t374*t375;
  t513 = t485*t489*t508;
  t548 = t463 + t513;
  t603 = t351*t548;
  t715 = -1.*t485*t628*t645;
  t750 = t603 + t715;
  t1846 = Cos(var1[8]);
  t1938 = t485*t628*t351;
  t1945 = t548*t645;
  t2020 = t1938 + t1945;
  t1883 = t122*t945;
  t1892 = -1.*t750*t947;
  t1905 = t1883 + t1892;
  t2088 = Sin(var1[8]);
  t2299 = Cos(var1[9]);
  t1912 = t1846*t1905;
  t2192 = t2020*t2088;
  t2258 = t1912 + t2192;
  t1836 = Sin(var1[9]);
  t2337 = t1846*t2020;
  t2377 = -1.*t1905*t2088;
  t2451 = t2337 + t2377;
  t2564 = Cos(var1[10]);
  t2262 = -1.*t1836*t2258;
  t2529 = t2299*t2451;
  t2545 = t2262 + t2529;
  t1797 = Sin(var1[10]);
  t2582 = t2299*t2258;
  t2600 = t1836*t2451;
  t2614 = t2582 + t2600;
  t1410 = t374*t375*t489;
  t1477 = -1.*t485*t508;
  t1483 = t1410 + t1477;
  t1023 = t485*t374;
  t1076 = t375*t489*t508;
  t1111 = t1023 + t1076;
  t1119 = t351*t1111;
  t1204 = -1.*t628*t375*t645;
  t1330 = t1119 + t1204;
  t2808 = t628*t351*t375;
  t2824 = t1111*t645;
  t2873 = t2808 + t2824;
  t2721 = t122*t1483;
  t2731 = -1.*t1330*t947;
  t2759 = t2721 + t2731;
  t2792 = t1846*t2759;
  t2907 = t2873*t2088;
  t2948 = t2792 + t2907;
  t2994 = t1846*t2873;
  t3021 = -1.*t2759*t2088;
  t3039 = t2994 + t3021;
  t2970 = -1.*t1836*t2948;
  t3123 = t2299*t3039;
  t3126 = t2970 + t3123;
  t3170 = t2299*t2948;
  t3176 = t1836*t3039;
  t3179 = t3170 + t3176;
  t1659 = t628*t351*t508;
  t1693 = t489*t645;
  t1731 = t1659 + t1693;
  t3323 = -1.*t351*t489;
  t3330 = t628*t508*t645;
  t3336 = t3323 + t3330;
  t3255 = t628*t374*t122;
  t3268 = -1.*t1731*t947;
  t3291 = t3255 + t3268;
  t3301 = t1846*t3291;
  t3361 = t3336*t2088;
  t3388 = t3301 + t3361;
  t3463 = t1846*t3336;
  t3493 = -1.*t3291*t2088;
  t3514 = t3463 + t3493;
  t3392 = -1.*t1836*t3388;
  t3533 = t2299*t3514;
  t3534 = t3392 + t3533;
  t3575 = t2299*t3388;
  t3580 = t1836*t3514;
  t3582 = t3575 + t3580;
  t4176 = -1.*t351;
  t4205 = 1. + t4176;
  t4349 = -1.*t122;
  t4391 = 1. + t4349;
  t771 = t122*t750;
  t948 = t945*t947;
  t987 = t771 + t948;
  t4669 = -1.*t1846;
  t4671 = 1. + t4669;
  t4761 = -1.*t2299;
  t4762 = 1. + t4761;
  t4897 = -1.*t2564;
  t4903 = 1. + t4897;
  t3736 = t1797*t2545;
  t3744 = t2564*t2614;
  t3783 = t3736 + t3744;
  t2660 = t2564*t2545;
  t2667 = -1.*t1797*t2614;
  t2682 = t2660 + t2667;
  t4232 = 0.087*t4205;
  t4233 = 0.0222*t645;
  t4249 = 0. + t4232 + t4233;
  t4323 = -0.0222*t4205;
  t4334 = 0.087*t645;
  t4336 = 0. + t4323 + t4334;
  t4400 = 0.157*t4391;
  t4454 = -0.3151*t947;
  t4541 = 0. + t4400 + t4454;
  t4620 = -0.3151*t4391;
  t4634 = -0.157*t947;
  t4642 = 0. + t4620 + t4634;
  t1358 = t122*t1330;
  t1574 = t1483*t947;
  t1640 = t1358 + t1574;
  t4672 = -0.3801*t4671;
  t4677 = -0.0222*t2088;
  t4679 = 0. + t4672 + t4677;
  t4710 = -0.0222*t4671;
  t4716 = 0.3801*t2088;
  t4726 = 0. + t4710 + t4716;
  t4785 = -0.8601*t4762;
  t4808 = -0.0222*t1836;
  t4814 = 0. + t4785 + t4808;
  t4849 = -0.0222*t4762;
  t4853 = 0.8601*t1836;
  t4857 = 0. + t4849 + t4853;
  t4911 = -0.0211*t4903;
  t4917 = 1.3401*t1797;
  t4931 = 0. + t4911 + t4917;
  t4952 = -1.3401*t4903;
  t4954 = -0.0211*t1797;
  t4955 = 0. + t4952 + t4954;
  t3875 = t1797*t3126;
  t3879 = t2564*t3179;
  t3926 = t3875 + t3879;
  t3214 = t2564*t3126;
  t3224 = -1.*t1797*t3179;
  t3226 = t3214 + t3224;
  t1749 = t122*t1731;
  t1765 = t628*t374*t947;
  t1777 = t1749 + t1765;
  t4043 = t1797*t3534;
  t4066 = t2564*t3582;
  t4085 = t4043 + t4066;
  t3602 = t2564*t3534;
  t3612 = -1.*t1797*t3582;
  t3614 = t3602 + t3612;

  p_output1(0)=t987;
  p_output1(1)=t1640;
  p_output1(2)=t1777;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1797*t2545 - 1.*t2564*t2614 - 0.000796*t2682;
  p_output1(5)=-1.*t1797*t3126 - 1.*t2564*t3179 - 0.000796*t3226;
  p_output1(6)=-1.*t1797*t3534 - 1.*t2564*t3582 - 0.000796*t3614;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2545*t2564 + t1797*t2614 + 0.000796*t3783;
  p_output1(9)=-1.*t2564*t3126 + t1797*t3179 + 0.000796*t3926;
  p_output1(10)=-1.*t2564*t3534 + t1797*t3582 + 0.000796*t4085;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043912*t2682 - 1.325152*t3783 + t1905*t4679 + t2020*t4726 + t2258*t4814 + t2451*t4857 + t2545*t4931 + t2614*t4955 + t4249*t548 + t4336*t485*t628 + t4541*t750 + t4642*t945 + 0.092*t987 + var1(0);
  p_output1(13)=0. + 0.092*t1640 + 0.043912*t3226 - 1.325152*t3926 + t1111*t4249 + t1330*t4541 + t1483*t4642 + t2759*t4679 + t2873*t4726 + t2948*t4814 + t3039*t4857 + t3126*t4931 + t3179*t4955 + t375*t4336*t628 + var1(1);
  p_output1(14)=0. + 0.092*t1777 + 0.043912*t3614 - 1.325152*t4085 + t1731*t4541 + t3291*t4679 + t3336*t4726 + t3388*t4814 + t3514*t4857 - 1.*t4336*t489 + t3534*t4931 + t3582*t4955 + t374*t4642*t628 + t4249*t508*t628 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
