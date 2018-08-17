/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:39 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lKnee.h"

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
  double t502;
  double t835;
  double t892;
  double t924;
  double t1093;
  double t154;
  double t287;
  double t404;
  double t712;
  double t717;
  double t752;
  double t777;
  double t1238;
  double t1897;
  double t2232;
  double t2245;
  double t2254;
  double t1815;
  double t1834;
  double t1843;
  double t2425;
  double t2435;
  double t2456;
  double t2979;
  double t2984;
  double t3044;
  double t3328;
  double t2847;
  double t2915;
  double t2974;
  double t3642;
  double t3695;
  double t3706;
  double t3926;
  double t3934;
  double t3976;
  double t3997;
  double t4135;
  double t4160;
  double t4343;
  double t4605;
  double t4641;
  double t4663;
  double t928;
  double t1139;
  double t1151;
  double t1524;
  double t1577;
  double t1598;
  double t5015;
  double t5022;
  double t5117;
  double t2252;
  double t2308;
  double t2348;
  double t2487;
  double t2510;
  double t2615;
  double t5287;
  double t5300;
  double t5337;
  double t5361;
  double t5368;
  double t5369;
  double t3312;
  double t3399;
  double t3521;
  double t3732;
  double t3779;
  double t3868;
  double t3984;
  double t4074;
  double t4091;
  double t5560;
  double t5567;
  double t5696;
  double t5732;
  double t5797;
  double t5839;
  double t4562;
  double t4567;
  double t4581;
  double t5937;
  double t5948;
  double t5970;
  double t6003;
  double t6010;
  double t6014;
  double t6154;
  double t6215;
  double t6218;
  double t6280;
  double t6281;
  double t6298;
  double t6326;
  double t6331;
  double t6350;
  double t6369;
  double t6394;
  double t6395;
  double t6405;
  double t6406;
  double t6410;
  double t6526;
  double t6532;
  double t6535;
  double t6628;
  double t6641;
  double t6657;
  double t6662;
  double t6668;
  double t6674;
  double t6683;
  double t6689;
  double t6701;
  double t6709;
  double t6715;
  double t6726;
  double t6800;
  double t6802;
  double t6806;
  double t6834;
  double t6837;
  double t6840;
  double t6877;
  double t6880;
  double t6888;
  double t6891;
  double t6892;
  double t6902;
  double t6925;
  double t6930;
  double t6946;
  double t7059;
  double t7064;
  double t7066;
  double t7101;
  double t7127;
  double t7130;
  double t7148;
  double t7151;
  double t7152;
  double t7163;
  double t7171;
  double t7174;
  double t7264;
  double t7268;
  double t7271;
  double t7359;
  double t7360;
  double t7363;
  double t7417;
  double t7450;
  double t7451;
  double t7483;
  double t7509;
  double t7518;
  double t7635;
  double t7637;
  double t7640;
  double t7651;
  double t7665;
  double t7671;
  double t7676;
  double t7684;
  double t7686;
  double t7842;
  double t7850;
  double t7861;
  double t7924;
  double t7954;
  double t7960;
  double t7992;
  double t7998;
  double t8001;
  double t7816;
  double t7821;
  double t7822;
  double t7825;
  double t7830;
  double t8119;
  double t8131;
  double t8137;
  double t8151;
  double t8176;
  double t8230;
  double t8237;
  double t8260;
  double t8277;
  double t8281;
  double t8285;
  double t8291;
  double t8297;
  double t8299;
  double t8488;
  double t8489;
  double t8499;
  double t8553;
  double t8558;
  double t8562;
  double t8627;
  double t8650;
  double t8651;
  double t8668;
  double t8704;
  double t8710;
  double t8879;
  double t8884;
  double t8891;
  double t8790;
  double t8794;
  double t8808;
  double t8830;
  double t8866;
  double t9067;
  double t9079;
  double t9085;
  double t9328;
  double t9329;
  double t9332;
  double t9508;
  double t9521;
  double t9533;
  double t6034;
  double t9129;
  double t9134;
  double t9152;
  double t9466;
  double t9474;
  double t9478;
  double t9483;
  double t9486;
  double t9601;
  double t9619;
  double t9621;
  double t9627;
  double t9630;
  double t9631;
  double t9637;
  double t9639;
  double t9640;
  double t9353;
  double t9356;
  double t9363;
  double t9671;
  double t9673;
  double t9674;
  double t9677;
  double t9680;
  double t9682;
  double t9688;
  double t9690;
  double t9691;
  double t6030;
  double t6036;
  double t9573;
  double t9709;
  double t9712;
  double t9713;
  double t9715;
  double t9717;
  double t9731;
  double t9732;
  double t9733;
  double t9645;
  double t9658;
  double t9744;
  double t9745;
  double t9746;
  double t9695;
  double t9704;
  t502 = Sin(var1[3]);
  t835 = Cos(var1[6]);
  t892 = -1.*t835;
  t924 = 1. + t892;
  t1093 = Sin(var1[6]);
  t154 = Cos(var1[3]);
  t287 = Cos(var1[5]);
  t404 = -1.*t154*t287;
  t712 = Sin(var1[4]);
  t717 = Sin(var1[5]);
  t752 = -1.*t502*t712*t717;
  t777 = t404 + t752;
  t1238 = Cos(var1[4]);
  t1897 = Cos(var1[7]);
  t2232 = -1.*t1897;
  t2245 = 1. + t2232;
  t2254 = Sin(var1[7]);
  t1815 = t835*t777;
  t1834 = t1238*t502*t1093;
  t1843 = t1815 + t1834;
  t2425 = -1.*t287*t502*t712;
  t2435 = t154*t717;
  t2456 = t2425 + t2435;
  t2979 = Cos(var1[8]);
  t2984 = -1.*t2979;
  t3044 = 1. + t2984;
  t3328 = Sin(var1[8]);
  t2847 = t1897*t2456;
  t2915 = -1.*t1843*t2254;
  t2974 = t2847 + t2915;
  t3642 = -1.*t1238*t835*t502;
  t3695 = t777*t1093;
  t3706 = t3642 + t3695;
  t3926 = Cos(var1[9]);
  t3934 = -1.*t3926;
  t3976 = 1. + t3934;
  t3997 = Sin(var1[9]);
  t4135 = t2979*t2974;
  t4160 = t3706*t3328;
  t4343 = t4135 + t4160;
  t4605 = t2979*t3706;
  t4641 = -1.*t2974*t3328;
  t4663 = t4605 + t4641;
  t928 = 0.087004*t924;
  t1139 = 0.022225*t1093;
  t1151 = 0. + t928 + t1139;
  t1524 = -0.022225*t924;
  t1577 = 0.087004*t1093;
  t1598 = 0. + t1524 + t1577;
  t5015 = -1.*t287*t502;
  t5022 = t154*t712*t717;
  t5117 = t5015 + t5022;
  t2252 = 0.157004*t2245;
  t2308 = -0.31508*t2254;
  t2348 = 0. + t2252 + t2308;
  t2487 = -0.31508*t2245;
  t2510 = -0.157004*t2254;
  t2615 = 0. + t2487 + t2510;
  t5287 = t835*t5117;
  t5300 = -1.*t154*t1238*t1093;
  t5337 = t5287 + t5300;
  t5361 = t154*t287*t712;
  t5368 = t502*t717;
  t5369 = t5361 + t5368;
  t3312 = -0.38008*t3044;
  t3399 = -0.022225*t3328;
  t3521 = 0. + t3312 + t3399;
  t3732 = -0.022225*t3044;
  t3779 = 0.38008*t3328;
  t3868 = 0. + t3732 + t3779;
  t3984 = -0.86008*t3976;
  t4074 = -0.022225*t3997;
  t4091 = 0. + t3984 + t4074;
  t5560 = t1897*t5369;
  t5567 = -1.*t5337*t2254;
  t5696 = t5560 + t5567;
  t5732 = t154*t1238*t835;
  t5797 = t5117*t1093;
  t5839 = t5732 + t5797;
  t4562 = -0.022225*t3976;
  t4567 = 0.86008*t3997;
  t4581 = 0. + t4562 + t4567;
  t5937 = t2979*t5696;
  t5948 = t5839*t3328;
  t5970 = t5937 + t5948;
  t6003 = t2979*t5839;
  t6010 = -1.*t5696*t3328;
  t6014 = t6003 + t6010;
  t6154 = t154*t1238*t835*t717;
  t6215 = t154*t712*t1093;
  t6218 = t6154 + t6215;
  t6280 = t154*t1238*t287*t1897;
  t6281 = -1.*t6218*t2254;
  t6298 = t6280 + t6281;
  t6326 = -1.*t154*t835*t712;
  t6331 = t154*t1238*t717*t1093;
  t6350 = t6326 + t6331;
  t6369 = t2979*t6298;
  t6394 = t6350*t3328;
  t6395 = t6369 + t6394;
  t6405 = t2979*t6350;
  t6406 = -1.*t6298*t3328;
  t6410 = t6405 + t6406;
  t6526 = t1238*t835*t502*t717;
  t6532 = t502*t712*t1093;
  t6535 = t6526 + t6532;
  t6628 = t1238*t287*t1897*t502;
  t6641 = -1.*t6535*t2254;
  t6657 = t6628 + t6641;
  t6662 = -1.*t835*t502*t712;
  t6668 = t1238*t502*t717*t1093;
  t6674 = t6662 + t6668;
  t6683 = t2979*t6657;
  t6689 = t6674*t3328;
  t6701 = t6683 + t6689;
  t6709 = t2979*t6674;
  t6715 = -1.*t6657*t3328;
  t6726 = t6709 + t6715;
  t6800 = -1.*t835*t712*t717;
  t6802 = t1238*t1093;
  t6806 = t6800 + t6802;
  t6834 = -1.*t287*t1897*t712;
  t6837 = -1.*t6806*t2254;
  t6840 = t6834 + t6837;
  t6877 = -1.*t1238*t835;
  t6880 = -1.*t712*t717*t1093;
  t6888 = t6877 + t6880;
  t6891 = t2979*t6840;
  t6892 = t6888*t3328;
  t6902 = t6891 + t6892;
  t6925 = t2979*t6888;
  t6930 = -1.*t6840*t3328;
  t6946 = t6925 + t6930;
  t7059 = t287*t502;
  t7064 = -1.*t154*t712*t717;
  t7066 = t7059 + t7064;
  t7101 = t1897*t7066;
  t7127 = -1.*t835*t5369*t2254;
  t7130 = t7101 + t7127;
  t7148 = t2979*t7130;
  t7151 = t5369*t1093*t3328;
  t7152 = t7148 + t7151;
  t7163 = t2979*t5369*t1093;
  t7171 = -1.*t7130*t3328;
  t7174 = t7163 + t7171;
  t7264 = t287*t502*t712;
  t7268 = -1.*t154*t717;
  t7271 = t7264 + t7268;
  t7359 = t1897*t777;
  t7360 = -1.*t835*t7271*t2254;
  t7363 = t7359 + t7360;
  t7417 = t2979*t7363;
  t7450 = t7271*t1093*t3328;
  t7451 = t7417 + t7450;
  t7483 = t2979*t7271*t1093;
  t7509 = -1.*t7363*t3328;
  t7518 = t7483 + t7509;
  t7635 = -1.*t1238*t1897*t717;
  t7637 = -1.*t1238*t287*t835*t2254;
  t7640 = t7635 + t7637;
  t7651 = t2979*t7640;
  t7665 = t1238*t287*t1093*t3328;
  t7671 = t7651 + t7665;
  t7676 = t1238*t287*t2979*t1093;
  t7684 = -1.*t7640*t3328;
  t7686 = t7676 + t7684;
  t7842 = -1.*t154*t1238*t835;
  t7850 = -1.*t5117*t1093;
  t7861 = t7842 + t7850;
  t7924 = -1.*t2979*t7861*t2254;
  t7954 = t5337*t3328;
  t7960 = t7924 + t7954;
  t7992 = t2979*t5337;
  t7998 = t7861*t2254*t3328;
  t8001 = t7992 + t7998;
  t7816 = 0.087004*t835;
  t7821 = -0.022225*t1093;
  t7822 = t7816 + t7821;
  t7825 = 0.022225*t835;
  t7830 = t7825 + t1577;
  t8119 = t154*t287;
  t8131 = t502*t712*t717;
  t8137 = t8119 + t8131;
  t8151 = -1.*t8137*t1093;
  t8176 = t3642 + t8151;
  t8230 = t835*t8137;
  t8237 = -1.*t1238*t502*t1093;
  t8260 = t8230 + t8237;
  t8277 = -1.*t2979*t8176*t2254;
  t8281 = t8260*t3328;
  t8285 = t8277 + t8281;
  t8291 = t2979*t8260;
  t8297 = t8176*t2254*t3328;
  t8299 = t8291 + t8297;
  t8488 = t835*t712;
  t8489 = -1.*t1238*t717*t1093;
  t8499 = t8488 + t8489;
  t8553 = t1238*t835*t717;
  t8558 = t712*t1093;
  t8562 = t8553 + t8558;
  t8627 = -1.*t2979*t8499*t2254;
  t8650 = t8562*t3328;
  t8651 = t8627 + t8650;
  t8668 = t2979*t8562;
  t8704 = t8499*t2254*t3328;
  t8710 = t8668 + t8704;
  t8879 = -1.*t1897*t5337;
  t8884 = -1.*t5369*t2254;
  t8891 = t8879 + t8884;
  t8790 = -0.157004*t1897;
  t8794 = t8790 + t2308;
  t8808 = -0.31508*t1897;
  t8830 = 0.157004*t2254;
  t8866 = t8808 + t8830;
  t9067 = -1.*t1897*t8260;
  t9079 = -1.*t7271*t2254;
  t9085 = t9067 + t9079;
  t9328 = -1.*t1897*t8562;
  t9329 = -1.*t1238*t287*t2254;
  t9332 = t9328 + t9329;
  t9508 = -1.*t2979*t5696;
  t9521 = -1.*t5839*t3328;
  t9533 = t9508 + t9521;
  t6034 = t3926*t6014;
  t9129 = t1897*t7271;
  t9134 = -1.*t8260*t2254;
  t9152 = t9129 + t9134;
  t9466 = -0.022225*t2979;
  t9474 = -0.38008*t3328;
  t9478 = t9466 + t9474;
  t9483 = 0.38008*t2979;
  t9486 = t9483 + t3399;
  t9601 = t1238*t835*t502;
  t9619 = t8137*t1093;
  t9621 = t9601 + t9619;
  t9627 = -1.*t2979*t9152;
  t9630 = -1.*t9621*t3328;
  t9631 = t9627 + t9630;
  t9637 = t2979*t9621;
  t9639 = -1.*t9152*t3328;
  t9640 = t9637 + t9639;
  t9353 = t1238*t287*t1897;
  t9356 = -1.*t8562*t2254;
  t9363 = t9353 + t9356;
  t9671 = -1.*t835*t712;
  t9673 = t1238*t717*t1093;
  t9674 = t9671 + t9673;
  t9677 = -1.*t2979*t9363;
  t9680 = -1.*t9674*t3328;
  t9682 = t9677 + t9680;
  t9688 = t2979*t9674;
  t9690 = -1.*t9363*t3328;
  t9691 = t9688 + t9690;
  t6030 = -1.*t3997*t5970;
  t6036 = t6030 + t6034;
  t9573 = -1.*t3997*t6014;
  t9709 = -0.022225*t3926;
  t9712 = -0.86008*t3997;
  t9713 = t9709 + t9712;
  t9715 = 0.86008*t3926;
  t9717 = t9715 + t4074;
  t9731 = t2979*t9152;
  t9732 = t9621*t3328;
  t9733 = t9731 + t9732;
  t9645 = t3926*t9640;
  t9658 = -1.*t3997*t9640;
  t9744 = t2979*t9363;
  t9745 = t9674*t3328;
  t9746 = t9744 + t9745;
  t9695 = t3926*t9691;
  t9704 = -1.*t3997*t9691;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1843*t2348 + 0.150254*(t1843*t1897 + t2254*t2456) + t2456*t2615 + t2974*t3521 + t3706*t3868 + t4091*t4343 + t4581*t4663 - 0.022225*(-1.*t3997*t4343 + t3926*t4663) - 0.86008*(t3926*t4343 + t3997*t4663) - 1.*t1238*t1598*t502 + t1151*t777;
  p_output1(10)=t1238*t154*t1598 + t1151*t5117 + t2348*t5337 + t2615*t5369 + 0.150254*(t1897*t5337 + t2254*t5369) + t3521*t5696 + t3868*t5839 + t4091*t5970 + t4581*t6014 - 0.86008*(t3926*t5970 + t3997*t6014) - 0.022225*t6036;
  p_output1(11)=0;
  p_output1(12)=t1238*t154*t2615*t287 + t2348*t6218 + 0.150254*(t1238*t154*t2254*t287 + t1897*t6218) + t3521*t6298 + t3868*t6350 + t4091*t6395 + t4581*t6410 - 0.022225*(-1.*t3997*t6395 + t3926*t6410) - 0.86008*(t3926*t6395 + t3997*t6410) - 1.*t154*t1598*t712 + t1151*t1238*t154*t717;
  p_output1(13)=t1238*t2615*t287*t502 + t2348*t6535 + 0.150254*(t1238*t2254*t287*t502 + t1897*t6535) + t3521*t6657 + t3868*t6674 + t4091*t6701 + t4581*t6726 - 0.022225*(-1.*t3997*t6701 + t3926*t6726) - 0.86008*(t3926*t6701 + t3997*t6726) - 1.*t1598*t502*t712 + t1151*t1238*t502*t717;
  p_output1(14)=-1.*t1238*t1598 + t2348*t6806 + t3521*t6840 + t3868*t6888 + t4091*t6902 + t4581*t6946 - 0.022225*(-1.*t3997*t6902 + t3926*t6946) - 0.86008*(t3926*t6902 + t3997*t6946) - 1.*t2615*t287*t712 + 0.150254*(t1897*t6806 - 1.*t2254*t287*t712) - 1.*t1151*t712*t717;
  p_output1(15)=t1151*t5369 + t1093*t3868*t5369 + t2615*t7066 + t3521*t7130 + t4091*t7152 + t4581*t7174 - 0.022225*(-1.*t3997*t7152 + t3926*t7174) - 0.86008*(t3926*t7152 + t3997*t7174) + t2348*t5369*t835 + 0.150254*(t2254*t7066 + t1897*t5369*t835);
  p_output1(16)=t1151*t7271 + t1093*t3868*t7271 + t3521*t7363 + t4091*t7451 + t4581*t7518 - 0.022225*(-1.*t3997*t7451 + t3926*t7518) - 0.86008*(t3926*t7451 + t3997*t7518) + t2615*t777 + t2348*t7271*t835 + 0.150254*(t2254*t777 + t1897*t7271*t835);
  p_output1(17)=t1151*t1238*t287 + t1093*t1238*t287*t3868 - 1.*t1238*t2615*t717 + t3521*t7640 + t4091*t7671 + t4581*t7686 - 0.022225*(-1.*t3997*t7671 + t3926*t7686) - 0.86008*(t3926*t7671 + t3997*t7686) + t1238*t2348*t287*t835 + 0.150254*(-1.*t1238*t2254*t717 + t1238*t1897*t287*t835);
  p_output1(18)=t3868*t5337 + t1238*t154*t7822 + t5117*t7830 + 0.150254*t1897*t7861 + t2348*t7861 - 1.*t2254*t3521*t7861 + t4091*t7960 + t4581*t8001 - 0.022225*(-1.*t3997*t7960 + t3926*t8001) - 0.86008*(t3926*t7960 + t3997*t8001);
  p_output1(19)=t1238*t502*t7822 + t7830*t8137 + 0.150254*t1897*t8176 + t2348*t8176 - 1.*t2254*t3521*t8176 + t3868*t8260 + t4091*t8285 + t4581*t8299 - 0.022225*(-1.*t3997*t8285 + t3926*t8299) - 0.86008*(t3926*t8285 + t3997*t8299);
  p_output1(20)=-1.*t712*t7822 + t1238*t717*t7830 + 0.150254*t1897*t8499 + t2348*t8499 - 1.*t2254*t3521*t8499 + t3868*t8562 + t4091*t8651 + t4581*t8710 - 0.022225*(-1.*t3997*t8651 + t3926*t8710) - 0.86008*(t3926*t8651 + t3997*t8710);
  p_output1(21)=0.150254*t5696 + t5369*t8794 + t5337*t8866 + t3521*t8891 + t2979*t4091*t8891 - 1.*t3328*t4581*t8891 - 0.022225*(-1.*t3328*t3926*t8891 - 1.*t2979*t3997*t8891) - 0.86008*(t2979*t3926*t8891 - 1.*t3328*t3997*t8891);
  p_output1(22)=t7271*t8794 + t8260*t8866 + t3521*t9085 + t2979*t4091*t9085 - 1.*t3328*t4581*t9085 - 0.022225*(-1.*t3328*t3926*t9085 - 1.*t2979*t3997*t9085) - 0.86008*(t2979*t3926*t9085 - 1.*t3328*t3997*t9085) + 0.150254*t9152;
  p_output1(23)=t1238*t287*t8794 + t8562*t8866 + t3521*t9332 + t2979*t4091*t9332 - 1.*t3328*t4581*t9332 - 0.022225*(-1.*t3328*t3926*t9332 - 1.*t2979*t3997*t9332) - 0.86008*(t2979*t3926*t9332 - 1.*t3328*t3997*t9332) + 0.150254*t9363;
  p_output1(24)=t4091*t6014 + t5696*t9478 + t5839*t9486 + t4581*t9533 - 0.86008*(t6034 + t3997*t9533) - 0.022225*(t3926*t9533 + t9573);
  p_output1(25)=t9152*t9478 + t9486*t9621 + t4581*t9631 + t4091*t9640 - 0.86008*(t3997*t9631 + t9645) - 0.022225*(t3926*t9631 + t9658);
  p_output1(26)=t9363*t9478 + t9486*t9674 + t4581*t9682 + t4091*t9691 - 0.86008*(t3997*t9682 + t9695) - 0.022225*(t3926*t9682 + t9704);
  p_output1(27)=-0.86008*t6036 - 0.022225*(-1.*t3926*t5970 + t9573) + t5970*t9713 + t6014*t9717;
  p_output1(28)=t9640*t9717 + t9713*t9733 - 0.022225*(t9658 - 1.*t3926*t9733) - 0.86008*(t9645 - 1.*t3997*t9733);
  p_output1(29)=t9691*t9717 + t9713*t9746 - 0.022225*(t9704 - 1.*t3926*t9746) - 0.86008*(t9695 - 1.*t3997*t9746);
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_lKnee(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
