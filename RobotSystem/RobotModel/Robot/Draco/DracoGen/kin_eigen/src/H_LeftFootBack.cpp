/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:08 GMT-05:00
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
  double t388;
  double t438;
  double t371;
  double t398;
  double t443;
  double t1062;
  double t840;
  double t860;
  double t945;
  double t999;
  double t1204;
  double t179;
  double t1455;
  double t1504;
  double t1507;
  double t325;
  double t421;
  double t802;
  double t829;
  double t831;
  double t1031;
  double t1242;
  double t1291;
  double t1305;
  double t1380;
  double t1396;
  double t1731;
  double t1775;
  double t1439;
  double t1733;
  double t1739;
  double t121;
  double t1782;
  double t1799;
  double t1889;
  double t2001;
  double t1762;
  double t1892;
  double t1950;
  double t86;
  double t2052;
  double t2158;
  double t2161;
  double t2583;
  double t2631;
  double t2658;
  double t2907;
  double t2961;
  double t2962;
  double t2434;
  double t2528;
  double t2564;
  double t2578;
  double t2671;
  double t2674;
  double t2765;
  double t2813;
  double t2832;
  double t2891;
  double t2980;
  double t3061;
  double t3069;
  double t3089;
  double t3136;
  double t3067;
  double t3159;
  double t3198;
  double t3298;
  double t3324;
  double t3358;
  double t3604;
  double t3626;
  double t3641;
  double t3509;
  double t3529;
  double t3541;
  double t3553;
  double t3568;
  double t3583;
  double t3598;
  double t3690;
  double t3695;
  double t3775;
  double t3784;
  double t3837;
  double t3710;
  double t3843;
  double t3847;
  double t3887;
  double t4002;
  double t4034;
  double t1957;
  double t2227;
  double t3292;
  double t3364;
  double t3858;
  double t4085;
  double t4831;
  double t4834;
  double t5008;
  double t5035;
  double t4231;
  double t4240;
  double t4301;
  double t5129;
  double t5134;
  double t5244;
  double t5334;
  double t5447;
  double t5453;
  double t4414;
  double t2228;
  double t2282;
  double t2294;
  double t4837;
  double t4844;
  double t4899;
  double t4915;
  double t4923;
  double t4971;
  double t5040;
  double t5078;
  double t5081;
  double t5095;
  double t5098;
  double t5116;
  double t4309;
  double t4328;
  double t4351;
  double t5143;
  double t5145;
  double t5153;
  double t5180;
  double t5233;
  double t5235;
  double t5369;
  double t5380;
  double t5393;
  double t5418;
  double t5421;
  double t5425;
  double t5466;
  double t5480;
  double t5528;
  double t5623;
  double t5632;
  double t5648;
  double t4507;
  double t3400;
  double t3416;
  double t3469;
  double t4372;
  double t4373;
  double t4374;
  double t4735;
  double t4094;
  double t4154;
  double t4170;
  t388 = Cos(var1[5]);
  t438 = Sin(var1[3]);
  t371 = Cos(var1[3]);
  t398 = Sin(var1[4]);
  t443 = Sin(var1[5]);
  t1062 = Cos(var1[4]);
  t840 = Cos(var1[6]);
  t860 = -1.*t388*t438;
  t945 = t371*t398*t443;
  t999 = t860 + t945;
  t1204 = Sin(var1[6]);
  t179 = Cos(var1[8]);
  t1455 = t371*t1062*t840;
  t1504 = t999*t1204;
  t1507 = t1455 + t1504;
  t325 = Cos(var1[7]);
  t421 = t371*t388*t398;
  t802 = t438*t443;
  t829 = t421 + t802;
  t831 = t325*t829;
  t1031 = t840*t999;
  t1242 = -1.*t371*t1062*t1204;
  t1291 = t1031 + t1242;
  t1305 = Sin(var1[7]);
  t1380 = -1.*t1291*t1305;
  t1396 = t831 + t1380;
  t1731 = Sin(var1[8]);
  t1775 = Cos(var1[9]);
  t1439 = t179*t1396;
  t1733 = t1507*t1731;
  t1739 = t1439 + t1733;
  t121 = Sin(var1[9]);
  t1782 = t179*t1507;
  t1799 = -1.*t1396*t1731;
  t1889 = t1782 + t1799;
  t2001 = Cos(var1[10]);
  t1762 = -1.*t121*t1739;
  t1892 = t1775*t1889;
  t1950 = t1762 + t1892;
  t86 = Sin(var1[10]);
  t2052 = t1775*t1739;
  t2158 = t121*t1889;
  t2161 = t2052 + t2158;
  t2583 = t371*t388;
  t2631 = t438*t398*t443;
  t2658 = t2583 + t2631;
  t2907 = t1062*t840*t438;
  t2961 = t2658*t1204;
  t2962 = t2907 + t2961;
  t2434 = t388*t438*t398;
  t2528 = -1.*t371*t443;
  t2564 = t2434 + t2528;
  t2578 = t325*t2564;
  t2671 = t840*t2658;
  t2674 = -1.*t1062*t438*t1204;
  t2765 = t2671 + t2674;
  t2813 = -1.*t2765*t1305;
  t2832 = t2578 + t2813;
  t2891 = t179*t2832;
  t2980 = t2962*t1731;
  t3061 = t2891 + t2980;
  t3069 = t179*t2962;
  t3089 = -1.*t2832*t1731;
  t3136 = t3069 + t3089;
  t3067 = -1.*t121*t3061;
  t3159 = t1775*t3136;
  t3198 = t3067 + t3159;
  t3298 = t1775*t3061;
  t3324 = t121*t3136;
  t3358 = t3298 + t3324;
  t3604 = -1.*t840*t398;
  t3626 = t1062*t443*t1204;
  t3641 = t3604 + t3626;
  t3509 = t1062*t388*t325;
  t3529 = t1062*t840*t443;
  t3541 = t398*t1204;
  t3553 = t3529 + t3541;
  t3568 = -1.*t3553*t1305;
  t3583 = t3509 + t3568;
  t3598 = t179*t3583;
  t3690 = t3641*t1731;
  t3695 = t3598 + t3690;
  t3775 = t179*t3641;
  t3784 = -1.*t3583*t1731;
  t3837 = t3775 + t3784;
  t3710 = -1.*t121*t3695;
  t3843 = t1775*t3837;
  t3847 = t3710 + t3843;
  t3887 = t1775*t3695;
  t4002 = t121*t3837;
  t4034 = t3887 + t4002;
  t1957 = t86*t1950;
  t2227 = t2001*t2161;
  t3292 = t86*t3198;
  t3364 = t2001*t3358;
  t3858 = t86*t3847;
  t4085 = t2001*t4034;
  t4831 = -1.*t840;
  t4834 = 1. + t4831;
  t5008 = -1.*t325;
  t5035 = 1. + t5008;
  t4231 = t325*t1291;
  t4240 = t829*t1305;
  t4301 = t4231 + t4240;
  t5129 = -1.*t179;
  t5134 = 1. + t5129;
  t5244 = -1.*t1775;
  t5334 = 1. + t5244;
  t5447 = -1.*t2001;
  t5453 = 1. + t5447;
  t4414 = t1957 + t2227;
  t2228 = t2001*t1950;
  t2282 = -1.*t86*t2161;
  t2294 = t2228 + t2282;
  t4837 = 0.087004*t4834;
  t4844 = 0.022225*t1204;
  t4899 = 0. + t4837 + t4844;
  t4915 = -0.022225*t4834;
  t4923 = 0.087004*t1204;
  t4971 = 0. + t4915 + t4923;
  t5040 = 0.157004*t5035;
  t5078 = -0.31508*t1305;
  t5081 = 0. + t5040 + t5078;
  t5095 = -0.31508*t5035;
  t5098 = -0.157004*t1305;
  t5116 = 0. + t5095 + t5098;
  t4309 = t325*t2765;
  t4328 = t2564*t1305;
  t4351 = t4309 + t4328;
  t5143 = -0.38008*t5134;
  t5145 = -0.022225*t1731;
  t5153 = 0. + t5143 + t5145;
  t5180 = -0.022225*t5134;
  t5233 = 0.38008*t1731;
  t5235 = 0. + t5180 + t5233;
  t5369 = -0.86008*t5334;
  t5380 = -0.022225*t121;
  t5393 = 0. + t5369 + t5380;
  t5418 = -0.022225*t5334;
  t5421 = 0.86008*t121;
  t5425 = 0. + t5418 + t5421;
  t5466 = -0.021147*t5453;
  t5480 = 1.34008*t86;
  t5528 = 0. + t5466 + t5480;
  t5623 = -1.34008*t5453;
  t5632 = -0.021147*t86;
  t5648 = 0. + t5623 + t5632;
  t4507 = t3292 + t3364;
  t3400 = t2001*t3198;
  t3416 = -1.*t86*t3358;
  t3469 = t3400 + t3416;
  t4372 = t325*t3553;
  t4373 = t1062*t388*t1305;
  t4374 = t4372 + t4373;
  t4735 = t3858 + t4085;
  t4094 = t2001*t3847;
  t4154 = -1.*t86*t4034;
  t4170 = t4094 + t4154;

  p_output1(0)=t1957 + t2227 + 0.000796*t2294;
  p_output1(1)=t3292 + t3364 + 0.000796*t3469;
  p_output1(2)=t3858 + t4085 + 0.000796*t4170;
  p_output1(3)=0.;
  p_output1(4)=t4301;
  p_output1(5)=t4351;
  p_output1(6)=t4374;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1950*t2001 + 0.000796*t4414 + t2161*t86;
  p_output1(9)=-1.*t2001*t3198 + 0.000796*t4507 + t3358*t86;
  p_output1(10)=-1.*t2001*t3847 + 0.000796*t4735 + t4034*t86;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043805*t2294 + 0.167004*t4301 - 1.400132*t4414 + t1062*t371*t4971 + t1291*t5081 + t1396*t5153 + t1507*t5235 + t1739*t5393 + t1889*t5425 + t1950*t5528 + t2161*t5648 + t5116*t829 + t4899*t999 + var1(0);
  p_output1(13)=0. + 0.043805*t3469 + 0.167004*t4351 - 1.400132*t4507 + t2658*t4899 + t1062*t438*t4971 + t2765*t5081 + t2564*t5116 + t2832*t5153 + t2962*t5235 + t3061*t5393 + t3136*t5425 + t3198*t5528 + t3358*t5648 + var1(1);
  p_output1(14)=0. + 0.043805*t4170 + 0.167004*t4374 - 1.400132*t4735 + t1062*t443*t4899 - 1.*t398*t4971 + t3553*t5081 + t1062*t388*t5116 + t3583*t5153 + t3641*t5235 + t3695*t5393 + t3837*t5425 + t3847*t5528 + t4034*t5648 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
