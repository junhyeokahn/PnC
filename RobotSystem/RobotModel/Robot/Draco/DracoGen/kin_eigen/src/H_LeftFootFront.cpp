/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:57 GMT-05:00
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
  double t715;
  double t750;
  double t676;
  double t716;
  double t775;
  double t1386;
  double t962;
  double t990;
  double t1161;
  double t1184;
  double t1465;
  double t611;
  double t1672;
  double t1673;
  double t1680;
  double t664;
  double t744;
  double t865;
  double t932;
  double t954;
  double t1276;
  double t1517;
  double t1548;
  double t1549;
  double t1563;
  double t1584;
  double t1681;
  double t1725;
  double t1591;
  double t1696;
  double t1719;
  double t551;
  double t1748;
  double t1756;
  double t1902;
  double t1932;
  double t1720;
  double t1914;
  double t1916;
  double t379;
  double t1948;
  double t1951;
  double t2037;
  double t2251;
  double t2291;
  double t2335;
  double t2453;
  double t2458;
  double t2471;
  double t2211;
  double t2223;
  double t2239;
  double t2244;
  double t2339;
  double t2356;
  double t2358;
  double t2379;
  double t2408;
  double t2444;
  double t2619;
  double t2643;
  double t2710;
  double t2738;
  double t2791;
  double t2681;
  double t2794;
  double t2803;
  double t2883;
  double t2940;
  double t2952;
  double t3154;
  double t3155;
  double t3168;
  double t3050;
  double t3054;
  double t3057;
  double t3065;
  double t3084;
  double t3087;
  double t3097;
  double t3214;
  double t3225;
  double t3264;
  double t3268;
  double t3295;
  double t3244;
  double t3299;
  double t3386;
  double t3455;
  double t3458;
  double t3529;
  double t1927;
  double t2038;
  double t2857;
  double t2955;
  double t3441;
  double t3546;
  double t4720;
  double t4745;
  double t4859;
  double t4870;
  double t3646;
  double t3709;
  double t3728;
  double t5064;
  double t5076;
  double t5212;
  double t5285;
  double t5406;
  double t5412;
  double t4083;
  double t2094;
  double t2097;
  double t2101;
  double t4753;
  double t4769;
  double t4794;
  double t4801;
  double t4810;
  double t4814;
  double t4928;
  double t4934;
  double t4938;
  double t5002;
  double t5025;
  double t5038;
  double t3808;
  double t3847;
  double t3896;
  double t5079;
  double t5094;
  double t5121;
  double t5149;
  double t5150;
  double t5151;
  double t5305;
  double t5332;
  double t5356;
  double t5364;
  double t5365;
  double t5401;
  double t5418;
  double t5420;
  double t5421;
  double t5445;
  double t5458;
  double t5463;
  double t4424;
  double t2956;
  double t2977;
  double t2997;
  double t4005;
  double t4007;
  double t4044;
  double t4578;
  double t3564;
  double t3581;
  double t3620;
  t715 = Cos(var1[5]);
  t750 = Sin(var1[3]);
  t676 = Cos(var1[3]);
  t716 = Sin(var1[4]);
  t775 = Sin(var1[5]);
  t1386 = Cos(var1[4]);
  t962 = Cos(var1[6]);
  t990 = -1.*t715*t750;
  t1161 = t676*t716*t775;
  t1184 = t990 + t1161;
  t1465 = Sin(var1[6]);
  t611 = Cos(var1[8]);
  t1672 = t676*t1386*t962;
  t1673 = t1184*t1465;
  t1680 = t1672 + t1673;
  t664 = Cos(var1[7]);
  t744 = t676*t715*t716;
  t865 = t750*t775;
  t932 = t744 + t865;
  t954 = t664*t932;
  t1276 = t962*t1184;
  t1517 = -1.*t676*t1386*t1465;
  t1548 = t1276 + t1517;
  t1549 = Sin(var1[7]);
  t1563 = -1.*t1548*t1549;
  t1584 = t954 + t1563;
  t1681 = Sin(var1[8]);
  t1725 = Cos(var1[9]);
  t1591 = t611*t1584;
  t1696 = t1680*t1681;
  t1719 = t1591 + t1696;
  t551 = Sin(var1[9]);
  t1748 = t611*t1680;
  t1756 = -1.*t1584*t1681;
  t1902 = t1748 + t1756;
  t1932 = Cos(var1[10]);
  t1720 = -1.*t551*t1719;
  t1914 = t1725*t1902;
  t1916 = t1720 + t1914;
  t379 = Sin(var1[10]);
  t1948 = t1725*t1719;
  t1951 = t551*t1902;
  t2037 = t1948 + t1951;
  t2251 = t676*t715;
  t2291 = t750*t716*t775;
  t2335 = t2251 + t2291;
  t2453 = t1386*t962*t750;
  t2458 = t2335*t1465;
  t2471 = t2453 + t2458;
  t2211 = t715*t750*t716;
  t2223 = -1.*t676*t775;
  t2239 = t2211 + t2223;
  t2244 = t664*t2239;
  t2339 = t962*t2335;
  t2356 = -1.*t1386*t750*t1465;
  t2358 = t2339 + t2356;
  t2379 = -1.*t2358*t1549;
  t2408 = t2244 + t2379;
  t2444 = t611*t2408;
  t2619 = t2471*t1681;
  t2643 = t2444 + t2619;
  t2710 = t611*t2471;
  t2738 = -1.*t2408*t1681;
  t2791 = t2710 + t2738;
  t2681 = -1.*t551*t2643;
  t2794 = t1725*t2791;
  t2803 = t2681 + t2794;
  t2883 = t1725*t2643;
  t2940 = t551*t2791;
  t2952 = t2883 + t2940;
  t3154 = -1.*t962*t716;
  t3155 = t1386*t775*t1465;
  t3168 = t3154 + t3155;
  t3050 = t1386*t715*t664;
  t3054 = t1386*t962*t775;
  t3057 = t716*t1465;
  t3065 = t3054 + t3057;
  t3084 = -1.*t3065*t1549;
  t3087 = t3050 + t3084;
  t3097 = t611*t3087;
  t3214 = t3168*t1681;
  t3225 = t3097 + t3214;
  t3264 = t611*t3168;
  t3268 = -1.*t3087*t1681;
  t3295 = t3264 + t3268;
  t3244 = -1.*t551*t3225;
  t3299 = t1725*t3295;
  t3386 = t3244 + t3299;
  t3455 = t1725*t3225;
  t3458 = t551*t3295;
  t3529 = t3455 + t3458;
  t1927 = t379*t1916;
  t2038 = t1932*t2037;
  t2857 = t379*t2803;
  t2955 = t1932*t2952;
  t3441 = t379*t3386;
  t3546 = t1932*t3529;
  t4720 = -1.*t962;
  t4745 = 1. + t4720;
  t4859 = -1.*t664;
  t4870 = 1. + t4859;
  t3646 = t664*t1548;
  t3709 = t932*t1549;
  t3728 = t3646 + t3709;
  t5064 = -1.*t611;
  t5076 = 1. + t5064;
  t5212 = -1.*t1725;
  t5285 = 1. + t5212;
  t5406 = -1.*t1932;
  t5412 = 1. + t5406;
  t4083 = t1927 + t2038;
  t2094 = t1932*t1916;
  t2097 = -1.*t379*t2037;
  t2101 = t2094 + t2097;
  t4753 = 0.087004*t4745;
  t4769 = 0.022225*t1465;
  t4794 = 0. + t4753 + t4769;
  t4801 = -0.022225*t4745;
  t4810 = 0.087004*t1465;
  t4814 = 0. + t4801 + t4810;
  t4928 = 0.157004*t4870;
  t4934 = -0.31508*t1549;
  t4938 = 0. + t4928 + t4934;
  t5002 = -0.31508*t4870;
  t5025 = -0.157004*t1549;
  t5038 = 0. + t5002 + t5025;
  t3808 = t664*t2358;
  t3847 = t2239*t1549;
  t3896 = t3808 + t3847;
  t5079 = -0.38008*t5076;
  t5094 = -0.022225*t1681;
  t5121 = 0. + t5079 + t5094;
  t5149 = -0.022225*t5076;
  t5150 = 0.38008*t1681;
  t5151 = 0. + t5149 + t5150;
  t5305 = -0.86008*t5285;
  t5332 = -0.022225*t551;
  t5356 = 0. + t5305 + t5332;
  t5364 = -0.022225*t5285;
  t5365 = 0.86008*t551;
  t5401 = 0. + t5364 + t5365;
  t5418 = -0.021147*t5412;
  t5420 = 1.34008*t379;
  t5421 = 0. + t5418 + t5420;
  t5445 = -1.34008*t5412;
  t5458 = -0.021147*t379;
  t5463 = 0. + t5445 + t5458;
  t4424 = t2857 + t2955;
  t2956 = t1932*t2803;
  t2977 = -1.*t379*t2952;
  t2997 = t2956 + t2977;
  t4005 = t664*t3065;
  t4007 = t1386*t715*t1549;
  t4044 = t4005 + t4007;
  t4578 = t3441 + t3546;
  t3564 = t1932*t3386;
  t3581 = -1.*t379*t3529;
  t3620 = t3564 + t3581;

  p_output1(0)=t1927 + t2038 + 0.000796*t2101;
  p_output1(1)=t2857 + t2955 + 0.000796*t2997;
  p_output1(2)=t3441 + t3546 + 0.000796*t3620;
  p_output1(3)=0.;
  p_output1(4)=t3728;
  p_output1(5)=t3896;
  p_output1(6)=t4044;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1916*t1932 + t2037*t379 + 0.000796*t4083;
  p_output1(9)=-1.*t1932*t2803 + t2952*t379 + 0.000796*t4424;
  p_output1(10)=-1.*t1932*t3386 + t3529*t379 + 0.000796*t4578;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043925*t2101 + 0.167004*t3728 - 1.250132*t4083 + t1184*t4794 + t1548*t4938 + t1584*t5121 + t1680*t5151 + t1719*t5356 + t1902*t5401 + t1916*t5421 + t2037*t5463 + t1386*t4814*t676 + t5038*t932 + var1(0);
  p_output1(13)=0. + 0.043925*t2997 + 0.167004*t3896 - 1.250132*t4424 + t2335*t4794 + t2358*t4938 + t2239*t5038 + t2408*t5121 + t2471*t5151 + t2643*t5356 + t2791*t5401 + t2803*t5421 + t2952*t5463 + t1386*t4814*t750 + var1(1);
  p_output1(14)=0. + 0.043925*t3620 + 0.167004*t4044 - 1.250132*t4578 + t3065*t4938 + t3087*t5121 + t3168*t5151 + t3225*t5356 + t3295*t5401 + t3386*t5421 + t3529*t5463 + t1386*t5038*t715 - 1.*t4814*t716 + t1386*t4794*t775 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
