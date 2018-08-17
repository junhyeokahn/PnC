/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:50 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rKnee.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t357;
  double t176;
  double t184;
  double t203;
  double t317;
  double t766;
  double t575;
  double t581;
  double t834;
  double t369;
  double t512;
  double t519;
  double t534;
  double t71;
  double t1219;
  double t1242;
  double t1262;
  double t609;
  double t881;
  double t887;
  double t1749;
  double t1809;
  double t1810;
  double t1861;
  double t2127;
  double t2148;
  double t2175;
  double t2519;
  double t2655;
  double t2689;
  double t3122;
  double t3127;
  double t3160;
  double t3260;
  double t3346;
  double t3370;
  double t3445;
  double t4151;
  double t4152;
  double t4294;
  double t4582;
  double t4599;
  double t4629;
  double t266;
  double t321;
  double t326;
  double t521;
  double t563;
  double t573;
  double t992;
  double t1005;
  double t1186;
  double t1385;
  double t1555;
  double t1557;
  double t5665;
  double t5705;
  double t5716;
  double t2150;
  double t2353;
  double t2508;
  double t5516;
  double t5528;
  double t5533;
  double t5764;
  double t5888;
  double t5903;
  double t2801;
  double t2817;
  double t2837;
  double t3375;
  double t3473;
  double t3547;
  double t6101;
  double t6114;
  double t6148;
  double t6408;
  double t6433;
  double t6460;
  double t4338;
  double t4504;
  double t4530;
  double t6473;
  double t6485;
  double t6487;
  double t6516;
  double t6520;
  double t6526;
  double t6786;
  double t6790;
  double t6796;
  double t6814;
  double t6816;
  double t6828;
  double t6912;
  double t6923;
  double t6931;
  double t6939;
  double t6943;
  double t6948;
  double t6974;
  double t6975;
  double t6998;
  double t7125;
  double t7131;
  double t7133;
  double t7138;
  double t7151;
  double t7158;
  double t7229;
  double t7231;
  double t7232;
  double t7238;
  double t7241;
  double t7243;
  double t7252;
  double t7270;
  double t7273;
  double t7344;
  double t7347;
  double t7349;
  double t7359;
  double t7370;
  double t7381;
  double t7416;
  double t7418;
  double t7420;
  double t7438;
  double t7443;
  double t7448;
  double t7450;
  double t7453;
  double t7455;
  double t7511;
  double t7516;
  double t7521;
  double t7539;
  double t7544;
  double t7546;
  double t7574;
  double t7578;
  double t7581;
  double t7588;
  double t7589;
  double t7604;
  double t7691;
  double t7701;
  double t7711;
  double t7727;
  double t7730;
  double t7736;
  double t7753;
  double t7755;
  double t7759;
  double t7777;
  double t7792;
  double t7805;
  double t7871;
  double t7882;
  double t7884;
  double t7918;
  double t7929;
  double t7931;
  double t7936;
  double t7937;
  double t7940;
  double t8006;
  double t8009;
  double t8015;
  double t8037;
  double t8040;
  double t8043;
  double t8045;
  double t8048;
  double t8051;
  double t7990;
  double t7992;
  double t7995;
  double t8000;
  double t8002;
  double t8099;
  double t8111;
  double t8112;
  double t8135;
  double t8145;
  double t8119;
  double t8121;
  double t8123;
  double t8165;
  double t8172;
  double t8175;
  double t8182;
  double t8185;
  double t8186;
  double t8268;
  double t8270;
  double t8272;
  double t8233;
  double t8242;
  double t8247;
  double t8284;
  double t8289;
  double t8290;
  double t8292;
  double t8295;
  double t8298;
  double t8377;
  double t8385;
  double t8389;
  double t8328;
  double t8330;
  double t8340;
  double t8345;
  double t8346;
  double t8439;
  double t8454;
  double t8457;
  double t8535;
  double t8547;
  double t8560;
  double t8688;
  double t8694;
  double t8695;
  double t6597;
  double t8652;
  double t8653;
  double t8666;
  double t8670;
  double t8681;
  double t8480;
  double t8488;
  double t8491;
  double t8758;
  double t8765;
  double t8768;
  double t8782;
  double t8790;
  double t8792;
  double t8798;
  double t8803;
  double t8804;
  double t8580;
  double t8581;
  double t8582;
  double t8869;
  double t8872;
  double t8880;
  double t8887;
  double t8901;
  double t8906;
  double t8917;
  double t8932;
  double t8939;
  double t6550;
  double t6599;
  double t8735;
  double t8998;
  double t9002;
  double t9008;
  double t9023;
  double t9027;
  double t9053;
  double t9054;
  double t9056;
  double t8837;
  double t8860;
  double t9111;
  double t9116;
  double t9118;
  double t8957;
  double t8978;
  t357 = Sin(var1[3]);
  t176 = Cos(var1[11]);
  t184 = -1.*t176;
  t203 = 1. + t184;
  t317 = Sin(var1[11]);
  t766 = Cos(var1[3]);
  t575 = Cos(var1[5]);
  t581 = Sin(var1[4]);
  t834 = Sin(var1[5]);
  t369 = Cos(var1[12]);
  t512 = -1.*t369;
  t519 = 1. + t512;
  t534 = Sin(var1[12]);
  t71 = Cos(var1[4]);
  t1219 = -1.*t766*t575;
  t1242 = -1.*t357*t581*t834;
  t1262 = t1219 + t1242;
  t609 = -1.*t575*t357*t581;
  t881 = t766*t834;
  t887 = t609 + t881;
  t1749 = t71*t317*t357;
  t1809 = t176*t1262;
  t1810 = t1749 + t1809;
  t1861 = Cos(var1[13]);
  t2127 = -1.*t1861;
  t2148 = 1. + t2127;
  t2175 = Sin(var1[13]);
  t2519 = -1.*t176*t71*t357;
  t2655 = t317*t1262;
  t2689 = t2519 + t2655;
  t3122 = t369*t887;
  t3127 = -1.*t534*t1810;
  t3160 = t3122 + t3127;
  t3260 = Cos(var1[14]);
  t3346 = -1.*t3260;
  t3370 = 1. + t3346;
  t3445 = Sin(var1[14]);
  t4151 = t2175*t2689;
  t4152 = t1861*t3160;
  t4294 = t4151 + t4152;
  t4582 = t1861*t2689;
  t4599 = -1.*t2175*t3160;
  t4629 = t4582 + t4599;
  t266 = -0.022225*t203;
  t321 = -0.086996*t317;
  t326 = 0. + t266 + t321;
  t521 = -0.31508*t519;
  t563 = 0.156996*t534;
  t573 = 0. + t521 + t563;
  t992 = -0.086996*t203;
  t1005 = 0.022225*t317;
  t1186 = 0. + t992 + t1005;
  t1385 = -0.156996*t519;
  t1555 = -0.31508*t534;
  t1557 = 0. + t1385 + t1555;
  t5665 = -1.*t575*t357;
  t5705 = t766*t581*t834;
  t5716 = t5665 + t5705;
  t2150 = -0.022225*t2148;
  t2353 = 0.38008*t2175;
  t2508 = 0. + t2150 + t2353;
  t5516 = t766*t575*t581;
  t5528 = t357*t834;
  t5533 = t5516 + t5528;
  t5764 = -1.*t766*t71*t317;
  t5888 = t176*t5716;
  t5903 = t5764 + t5888;
  t2801 = -0.38008*t2148;
  t2817 = -0.022225*t2175;
  t2837 = 0. + t2801 + t2817;
  t3375 = -0.86008*t3370;
  t3473 = -0.022225*t3445;
  t3547 = 0. + t3375 + t3473;
  t6101 = t176*t766*t71;
  t6114 = t317*t5716;
  t6148 = t6101 + t6114;
  t6408 = t369*t5533;
  t6433 = -1.*t534*t5903;
  t6460 = t6408 + t6433;
  t4338 = -0.022225*t3370;
  t4504 = 0.86008*t3445;
  t4530 = 0. + t4338 + t4504;
  t6473 = t2175*t6148;
  t6485 = t1861*t6460;
  t6487 = t6473 + t6485;
  t6516 = t1861*t6148;
  t6520 = -1.*t2175*t6460;
  t6526 = t6516 + t6520;
  t6786 = t766*t317*t581;
  t6790 = t176*t766*t71*t834;
  t6796 = t6786 + t6790;
  t6814 = -1.*t176*t766*t581;
  t6816 = t766*t71*t317*t834;
  t6828 = t6814 + t6816;
  t6912 = t369*t766*t71*t575;
  t6923 = -1.*t534*t6796;
  t6931 = t6912 + t6923;
  t6939 = t2175*t6828;
  t6943 = t1861*t6931;
  t6948 = t6939 + t6943;
  t6974 = t1861*t6828;
  t6975 = -1.*t2175*t6931;
  t6998 = t6974 + t6975;
  t7125 = t317*t357*t581;
  t7131 = t176*t71*t357*t834;
  t7133 = t7125 + t7131;
  t7138 = -1.*t176*t357*t581;
  t7151 = t71*t317*t357*t834;
  t7158 = t7138 + t7151;
  t7229 = t369*t71*t575*t357;
  t7231 = -1.*t534*t7133;
  t7232 = t7229 + t7231;
  t7238 = t2175*t7158;
  t7241 = t1861*t7232;
  t7243 = t7238 + t7241;
  t7252 = t1861*t7158;
  t7270 = -1.*t2175*t7232;
  t7273 = t7252 + t7270;
  t7344 = t71*t317;
  t7347 = -1.*t176*t581*t834;
  t7349 = t7344 + t7347;
  t7359 = -1.*t176*t71;
  t7370 = -1.*t317*t581*t834;
  t7381 = t7359 + t7370;
  t7416 = -1.*t369*t575*t581;
  t7418 = -1.*t534*t7349;
  t7420 = t7416 + t7418;
  t7438 = t2175*t7381;
  t7443 = t1861*t7420;
  t7448 = t7438 + t7443;
  t7450 = t1861*t7381;
  t7453 = -1.*t2175*t7420;
  t7455 = t7450 + t7453;
  t7511 = t575*t357;
  t7516 = -1.*t766*t581*t834;
  t7521 = t7511 + t7516;
  t7539 = -1.*t176*t534*t5533;
  t7544 = t369*t7521;
  t7546 = t7539 + t7544;
  t7574 = t317*t2175*t5533;
  t7578 = t1861*t7546;
  t7581 = t7574 + t7578;
  t7588 = t1861*t317*t5533;
  t7589 = -1.*t2175*t7546;
  t7604 = t7588 + t7589;
  t7691 = t575*t357*t581;
  t7701 = -1.*t766*t834;
  t7711 = t7691 + t7701;
  t7727 = -1.*t176*t534*t7711;
  t7730 = t369*t1262;
  t7736 = t7727 + t7730;
  t7753 = t317*t2175*t7711;
  t7755 = t1861*t7736;
  t7759 = t7753 + t7755;
  t7777 = t1861*t317*t7711;
  t7792 = -1.*t2175*t7736;
  t7805 = t7777 + t7792;
  t7871 = -1.*t176*t71*t575*t534;
  t7882 = -1.*t369*t71*t834;
  t7884 = t7871 + t7882;
  t7918 = t71*t575*t317*t2175;
  t7929 = t1861*t7884;
  t7931 = t7918 + t7929;
  t7936 = t1861*t71*t575*t317;
  t7937 = -1.*t2175*t7884;
  t7940 = t7936 + t7937;
  t8006 = -1.*t176*t766*t71;
  t8009 = -1.*t317*t5716;
  t8015 = t8006 + t8009;
  t8037 = t2175*t5903;
  t8040 = -1.*t1861*t534*t8015;
  t8043 = t8037 + t8040;
  t8045 = t1861*t5903;
  t8048 = t534*t2175*t8015;
  t8051 = t8045 + t8048;
  t7990 = -0.086996*t176;
  t7992 = -0.022225*t317;
  t7995 = t7990 + t7992;
  t8000 = 0.022225*t176;
  t8002 = t8000 + t321;
  t8099 = t766*t575;
  t8111 = t357*t581*t834;
  t8112 = t8099 + t8111;
  t8135 = -1.*t317*t8112;
  t8145 = t2519 + t8135;
  t8119 = -1.*t71*t317*t357;
  t8121 = t176*t8112;
  t8123 = t8119 + t8121;
  t8165 = t2175*t8123;
  t8172 = -1.*t1861*t534*t8145;
  t8175 = t8165 + t8172;
  t8182 = t1861*t8123;
  t8185 = t534*t2175*t8145;
  t8186 = t8182 + t8185;
  t8268 = t176*t581;
  t8270 = -1.*t71*t317*t834;
  t8272 = t8268 + t8270;
  t8233 = t317*t581;
  t8242 = t176*t71*t834;
  t8247 = t8233 + t8242;
  t8284 = t2175*t8247;
  t8289 = -1.*t1861*t534*t8272;
  t8290 = t8284 + t8289;
  t8292 = t1861*t8247;
  t8295 = t534*t2175*t8272;
  t8298 = t8292 + t8295;
  t8377 = -1.*t534*t5533;
  t8385 = -1.*t369*t5903;
  t8389 = t8377 + t8385;
  t8328 = 0.156996*t369;
  t8330 = t8328 + t1555;
  t8340 = -0.31508*t369;
  t8345 = -0.156996*t534;
  t8346 = t8340 + t8345;
  t8439 = -1.*t534*t7711;
  t8454 = -1.*t369*t8123;
  t8457 = t8439 + t8454;
  t8535 = -1.*t71*t575*t534;
  t8547 = -1.*t369*t8247;
  t8560 = t8535 + t8547;
  t8688 = -1.*t2175*t6148;
  t8694 = -1.*t1861*t6460;
  t8695 = t8688 + t8694;
  t6597 = t3260*t6526;
  t8652 = 0.38008*t1861;
  t8653 = t8652 + t2817;
  t8666 = -0.022225*t1861;
  t8670 = -0.38008*t2175;
  t8681 = t8666 + t8670;
  t8480 = t369*t7711;
  t8488 = -1.*t534*t8123;
  t8491 = t8480 + t8488;
  t8758 = t176*t71*t357;
  t8765 = t317*t8112;
  t8768 = t8758 + t8765;
  t8782 = -1.*t2175*t8768;
  t8790 = -1.*t1861*t8491;
  t8792 = t8782 + t8790;
  t8798 = t1861*t8768;
  t8803 = -1.*t2175*t8491;
  t8804 = t8798 + t8803;
  t8580 = t369*t71*t575;
  t8581 = -1.*t534*t8247;
  t8582 = t8580 + t8581;
  t8869 = -1.*t176*t581;
  t8872 = t71*t317*t834;
  t8880 = t8869 + t8872;
  t8887 = -1.*t2175*t8880;
  t8901 = -1.*t1861*t8582;
  t8906 = t8887 + t8901;
  t8917 = t1861*t8880;
  t8932 = -1.*t2175*t8582;
  t8939 = t8917 + t8932;
  t6550 = -1.*t3445*t6487;
  t6599 = t6550 + t6597;
  t8735 = -1.*t3445*t6526;
  t8998 = -0.022225*t3260;
  t9002 = -0.86008*t3445;
  t9008 = t8998 + t9002;
  t9023 = 0.86008*t3260;
  t9027 = t9023 + t3473;
  t9053 = t2175*t8768;
  t9054 = t1861*t8491;
  t9056 = t9053 + t9054;
  t8837 = t3260*t8804;
  t8860 = -1.*t3445*t8804;
  t9111 = t2175*t8880;
  t9116 = t1861*t8582;
  t9118 = t9111 + t9116;
  t8957 = t3260*t8939;
  t8978 = -1.*t3445*t8939;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1186*t1262 + t1557*t1810 + t2508*t2689 + t2837*t3160 + t3547*t4294 + t4530*t4629 - 0.022225*(-1.*t3445*t4294 + t3260*t4629) - 0.86008*(t3260*t4294 + t3445*t4629) - 1.*t326*t357*t71 + t573*t887 - 0.150246*(t1810*t369 + t534*t887);
  p_output1(10)=t1186*t5716 + t5533*t573 + t1557*t5903 - 0.150246*(t534*t5533 + t369*t5903) + t2508*t6148 + t2837*t6460 + t3547*t6487 + t4530*t6526 - 0.86008*(t3260*t6487 + t3445*t6526) - 0.022225*t6599 + t326*t71*t766;
  p_output1(11)=0;
  p_output1(12)=t1557*t6796 + t2508*t6828 + t2837*t6931 + t3547*t6948 + t4530*t6998 - 0.022225*(-1.*t3445*t6948 + t3260*t6998) - 0.86008*(t3260*t6948 + t3445*t6998) - 1.*t326*t581*t766 + t573*t575*t71*t766 - 0.150246*(t369*t6796 + t534*t575*t71*t766) + t1186*t71*t766*t834;
  p_output1(13)=-1.*t326*t357*t581 + t357*t573*t575*t71 + t1557*t7133 - 0.150246*(t357*t534*t575*t71 + t369*t7133) + t2508*t7158 + t2837*t7232 + t3547*t7243 + t4530*t7273 - 0.022225*(-1.*t3445*t7243 + t3260*t7273) - 0.86008*(t3260*t7243 + t3445*t7273) + t1186*t357*t71*t834;
  p_output1(14)=-1.*t573*t575*t581 - 1.*t326*t71 + t1557*t7349 - 0.150246*(-1.*t534*t575*t581 + t369*t7349) + t2508*t7381 + t2837*t7420 + t3547*t7448 + t4530*t7455 - 0.022225*(-1.*t3445*t7448 + t3260*t7455) - 0.86008*(t3260*t7448 + t3445*t7455) - 1.*t1186*t581*t834;
  p_output1(15)=t1186*t5533 + t1557*t176*t5533 + t2508*t317*t5533 + t573*t7521 - 0.150246*(t176*t369*t5533 + t534*t7521) + t2837*t7546 + t3547*t7581 + t4530*t7604 - 0.022225*(-1.*t3445*t7581 + t3260*t7604) - 0.86008*(t3260*t7581 + t3445*t7604);
  p_output1(16)=t1262*t573 + t1186*t7711 + t1557*t176*t7711 + t2508*t317*t7711 - 0.150246*(t1262*t534 + t176*t369*t7711) + t2837*t7736 + t3547*t7759 + t4530*t7805 - 0.022225*(-1.*t3445*t7759 + t3260*t7805) - 0.86008*(t3260*t7759 + t3445*t7805);
  p_output1(17)=t1186*t575*t71 + t1557*t176*t575*t71 + t2508*t317*t575*t71 + t2837*t7884 + t3547*t7931 + t4530*t7940 - 0.022225*(-1.*t3445*t7931 + t3260*t7940) - 0.86008*(t3260*t7931 + t3445*t7940) - 1.*t573*t71*t834 - 0.150246*(t176*t369*t575*t71 - 1.*t534*t71*t834);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t2508*t5903 + t71*t766*t7995 + t5716*t8002 + t1557*t8015 - 0.150246*t369*t8015 - 1.*t2837*t534*t8015 + t3547*t8043 + t4530*t8051 - 0.022225*(-1.*t3445*t8043 + t3260*t8051) - 0.86008*(t3260*t8043 + t3445*t8051);
  p_output1(34)=t357*t71*t7995 + t8002*t8112 + t2508*t8123 + t1557*t8145 - 0.150246*t369*t8145 - 1.*t2837*t534*t8145 + t3547*t8175 + t4530*t8186 - 0.022225*(-1.*t3445*t8175 + t3260*t8186) - 0.86008*(t3260*t8175 + t3445*t8186);
  p_output1(35)=-1.*t581*t7995 + t2508*t8247 + t1557*t8272 - 0.150246*t369*t8272 - 1.*t2837*t534*t8272 + t3547*t8290 + t4530*t8298 - 0.022225*(-1.*t3445*t8290 + t3260*t8298) - 0.86008*(t3260*t8290 + t3445*t8298) + t71*t8002*t834;
  p_output1(36)=-0.150246*t6460 + t5533*t8330 + t5903*t8346 + t2837*t8389 + t1861*t3547*t8389 - 1.*t2175*t4530*t8389 - 0.022225*(-1.*t2175*t3260*t8389 - 1.*t1861*t3445*t8389) - 0.86008*(t1861*t3260*t8389 - 1.*t2175*t3445*t8389);
  p_output1(37)=t7711*t8330 + t8123*t8346 + t2837*t8457 + t1861*t3547*t8457 - 1.*t2175*t4530*t8457 - 0.022225*(-1.*t2175*t3260*t8457 - 1.*t1861*t3445*t8457) - 0.86008*(t1861*t3260*t8457 - 1.*t2175*t3445*t8457) - 0.150246*t8491;
  p_output1(38)=t575*t71*t8330 + t8247*t8346 + t2837*t8560 + t1861*t3547*t8560 - 1.*t2175*t4530*t8560 - 0.022225*(-1.*t2175*t3260*t8560 - 1.*t1861*t3445*t8560) - 0.86008*(t1861*t3260*t8560 - 1.*t2175*t3445*t8560) - 0.150246*t8582;
  p_output1(39)=t3547*t6526 + t6148*t8653 + t6460*t8681 + t4530*t8695 - 0.86008*(t6597 + t3445*t8695) - 0.022225*(t3260*t8695 + t8735);
  p_output1(40)=t8491*t8681 + t8653*t8768 + t4530*t8792 + t3547*t8804 - 0.86008*(t3445*t8792 + t8837) - 0.022225*(t3260*t8792 + t8860);
  p_output1(41)=t8582*t8681 + t8653*t8880 + t4530*t8906 + t3547*t8939 - 0.86008*(t3445*t8906 + t8957) - 0.022225*(t3260*t8906 + t8978);
  p_output1(42)=-0.86008*t6599 - 0.022225*(-1.*t3260*t6487 + t8735) + t6487*t9008 + t6526*t9027;
  p_output1(43)=t8804*t9027 + t9008*t9056 - 0.022225*(t8860 - 1.*t3260*t9056) - 0.86008*(t8837 - 1.*t3445*t9056);
  p_output1(44)=t8939*t9027 + t9008*t9118 - 0.022225*(t8978 - 1.*t3260*t9118) - 0.86008*(t8957 - 1.*t3445*t9118);
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rKnee(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
