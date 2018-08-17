/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:57 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootFront.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t846;
  double t1276;
  double t152;
  double t865;
  double t1459;
  double t2124;
  double t1696;
  double t1951;
  double t2038;
  double t2094;
  double t2211;
  double t142;
  double t2453;
  double t2458;
  double t2471;
  double t148;
  double t971;
  double t1507;
  double t1673;
  double t1680;
  double t2101;
  double t2239;
  double t2251;
  double t2275;
  double t2335;
  double t2418;
  double t2542;
  double t2622;
  double t2429;
  double t2550;
  double t2598;
  double t131;
  double t2710;
  double t2743;
  double t2770;
  double t2940;
  double t2619;
  double t2794;
  double t2857;
  double t120;
  double t2955;
  double t2956;
  double t3054;
  double t3441;
  double t3455;
  double t3546;
  double t3728;
  double t3798;
  double t3945;
  double t3214;
  double t3264;
  double t3289;
  double t3299;
  double t3564;
  double t3620;
  double t3624;
  double t3644;
  double t3646;
  double t3709;
  double t3991;
  double t4004;
  double t4031;
  double t4044;
  double t4058;
  double t4007;
  double t4089;
  double t4153;
  double t4188;
  double t4201;
  double t4203;
  double t4604;
  double t4664;
  double t4753;
  double t4459;
  double t4465;
  double t4501;
  double t4515;
  double t4529;
  double t4533;
  double t4578;
  double t4769;
  double t4794;
  double t4801;
  double t4810;
  double t4845;
  double t4796;
  double t4928;
  double t4934;
  double t4962;
  double t4966;
  double t4967;
  double t2883;
  double t3057;
  double t4158;
  double t4254;
  double t4938;
  double t5002;
  t846 = Cos(var1[5]);
  t1276 = Sin(var1[3]);
  t152 = Cos(var1[3]);
  t865 = Sin(var1[4]);
  t1459 = Sin(var1[5]);
  t2124 = Cos(var1[4]);
  t1696 = Cos(var1[6]);
  t1951 = -1.*t846*t1276;
  t2038 = t152*t865*t1459;
  t2094 = t1951 + t2038;
  t2211 = Sin(var1[6]);
  t142 = Cos(var1[8]);
  t2453 = t152*t2124*t1696;
  t2458 = t2094*t2211;
  t2471 = t2453 + t2458;
  t148 = Cos(var1[7]);
  t971 = t152*t846*t865;
  t1507 = t1276*t1459;
  t1673 = t971 + t1507;
  t1680 = t148*t1673;
  t2101 = t1696*t2094;
  t2239 = -1.*t152*t2124*t2211;
  t2251 = t2101 + t2239;
  t2275 = Sin(var1[7]);
  t2335 = -1.*t2251*t2275;
  t2418 = t1680 + t2335;
  t2542 = Sin(var1[8]);
  t2622 = Cos(var1[9]);
  t2429 = t142*t2418;
  t2550 = t2471*t2542;
  t2598 = t2429 + t2550;
  t131 = Sin(var1[9]);
  t2710 = t142*t2471;
  t2743 = -1.*t2418*t2542;
  t2770 = t2710 + t2743;
  t2940 = Cos(var1[10]);
  t2619 = -1.*t131*t2598;
  t2794 = t2622*t2770;
  t2857 = t2619 + t2794;
  t120 = Sin(var1[10]);
  t2955 = t2622*t2598;
  t2956 = t131*t2770;
  t3054 = t2955 + t2956;
  t3441 = t152*t846;
  t3455 = t1276*t865*t1459;
  t3546 = t3441 + t3455;
  t3728 = t2124*t1696*t1276;
  t3798 = t3546*t2211;
  t3945 = t3728 + t3798;
  t3214 = t846*t1276*t865;
  t3264 = -1.*t152*t1459;
  t3289 = t3214 + t3264;
  t3299 = t148*t3289;
  t3564 = t1696*t3546;
  t3620 = -1.*t2124*t1276*t2211;
  t3624 = t3564 + t3620;
  t3644 = -1.*t3624*t2275;
  t3646 = t3299 + t3644;
  t3709 = t142*t3646;
  t3991 = t3945*t2542;
  t4004 = t3709 + t3991;
  t4031 = t142*t3945;
  t4044 = -1.*t3646*t2542;
  t4058 = t4031 + t4044;
  t4007 = -1.*t131*t4004;
  t4089 = t2622*t4058;
  t4153 = t4007 + t4089;
  t4188 = t2622*t4004;
  t4201 = t131*t4058;
  t4203 = t4188 + t4201;
  t4604 = -1.*t1696*t865;
  t4664 = t2124*t1459*t2211;
  t4753 = t4604 + t4664;
  t4459 = t2124*t846*t148;
  t4465 = t2124*t1696*t1459;
  t4501 = t865*t2211;
  t4515 = t4465 + t4501;
  t4529 = -1.*t4515*t2275;
  t4533 = t4459 + t4529;
  t4578 = t142*t4533;
  t4769 = t4753*t2542;
  t4794 = t4578 + t4769;
  t4801 = t142*t4753;
  t4810 = -1.*t4533*t2542;
  t4845 = t4801 + t4810;
  t4796 = -1.*t131*t4794;
  t4928 = t2622*t4845;
  t4934 = t4796 + t4928;
  t4962 = t2622*t4794;
  t4966 = t131*t4845;
  t4967 = t4962 + t4966;
  t2883 = t120*t2857;
  t3057 = t2940*t3054;
  t4158 = t120*t4153;
  t4254 = t2940*t4203;
  t4938 = t120*t4934;
  t5002 = t2940*t4967;

  p_output1(0)=t2883 + 0.000796*(t2857*t2940 - 1.*t120*t3054) + t3057;
  p_output1(1)=t4158 + 0.000796*(t2940*t4153 - 1.*t120*t4203) + t4254;
  p_output1(2)=t4938 + 0.000796*(t2940*t4934 - 1.*t120*t4967) + t5002;
  p_output1(3)=t148*t2251 + t1673*t2275;
  p_output1(4)=t2275*t3289 + t148*t3624;
  p_output1(5)=t148*t4515 + t2124*t2275*t846;
  p_output1(6)=-1.*t2857*t2940 + t120*t3054 + 0.000796*(t2883 + t3057);
  p_output1(7)=-1.*t2940*t4153 + t120*t4203 + 0.000796*(t4158 + t4254);
  p_output1(8)=-1.*t2940*t4934 + t120*t4967 + 0.000796*(t4938 + t5002);
}


       
void R_LeftFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
