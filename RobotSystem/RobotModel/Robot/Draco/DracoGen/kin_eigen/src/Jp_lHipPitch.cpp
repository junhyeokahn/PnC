/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:36 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipPitch.h"

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
  double t586;
  double t1635;
  double t1770;
  double t1898;
  double t2350;
  double t423;
  double t439;
  double t551;
  double t641;
  double t1438;
  double t1591;
  double t1617;
  double t2518;
  double t2753;
  double t2763;
  double t2777;
  double t2873;
  double t2621;
  double t2654;
  double t2741;
  double t2921;
  double t2926;
  double t2960;
  double t3186;
  double t3187;
  double t3194;
  double t3231;
  double t3170;
  double t3178;
  double t3184;
  double t3358;
  double t3426;
  double t3444;
  double t1903;
  double t2380;
  double t2424;
  double t2524;
  double t2528;
  double t2575;
  double t4638;
  double t4771;
  double t4778;
  double t2788;
  double t2880;
  double t2886;
  double t2964;
  double t3011;
  double t3066;
  double t5012;
  double t5107;
  double t5113;
  double t5181;
  double t5202;
  double t5315;
  double t3211;
  double t3242;
  double t3250;
  double t3445;
  double t3563;
  double t3571;
  double t5651;
  double t5677;
  double t5687;
  double t5713;
  double t5722;
  double t5744;
  double t5844;
  double t5862;
  double t5879;
  double t5930;
  double t5941;
  double t5951;
  double t5974;
  double t5982;
  double t5984;
  double t6063;
  double t6085;
  double t6087;
  double t6158;
  double t6162;
  double t6171;
  double t6175;
  double t6220;
  double t6232;
  double t6325;
  double t6326;
  double t6327;
  double t6352;
  double t6358;
  double t6365;
  double t6373;
  double t6374;
  double t6375;
  double t6446;
  double t6447;
  double t6452;
  double t6487;
  double t6488;
  double t6490;
  double t6541;
  double t6546;
  double t6549;
  double t6594;
  double t6617;
  double t6627;
  double t6747;
  double t6750;
  double t6751;
  double t6841;
  double t6848;
  double t6857;
  double t6800;
  double t6807;
  double t6808;
  double t6826;
  double t6827;
  double t6910;
  double t6925;
  double t6928;
  double t6936;
  double t6942;
  double t6997;
  double t7000;
  double t7004;
  double t7085;
  double t7088;
  double t7096;
  double t7139;
  double t7148;
  double t7150;
  double t7227;
  double t7230;
  double t7264;
  double t7198;
  double t7204;
  double t7210;
  double t7211;
  double t7215;
  double t7295;
  double t7296;
  double t7306;
  double t7440;
  double t7445;
  double t7450;
  double t5828;
  double t5833;
  double t5835;
  double t7363;
  double t7377;
  double t7380;
  double t7535;
  double t7539;
  double t7548;
  double t7577;
  double t7578;
  double t7655;
  double t7660;
  double t7661;
  double t7461;
  double t7471;
  double t7477;
  double t7772;
  double t7799;
  double t7801;
  t586 = Sin(var1[3]);
  t1635 = Cos(var1[6]);
  t1770 = -1.*t1635;
  t1898 = 1. + t1770;
  t2350 = Sin(var1[6]);
  t423 = Cos(var1[3]);
  t439 = Cos(var1[5]);
  t551 = -1.*t423*t439;
  t641 = Sin(var1[4]);
  t1438 = Sin(var1[5]);
  t1591 = -1.*t586*t641*t1438;
  t1617 = t551 + t1591;
  t2518 = Cos(var1[4]);
  t2753 = Cos(var1[7]);
  t2763 = -1.*t2753;
  t2777 = 1. + t2763;
  t2873 = Sin(var1[7]);
  t2621 = t1635*t1617;
  t2654 = t2518*t586*t2350;
  t2741 = t2621 + t2654;
  t2921 = -1.*t439*t586*t641;
  t2926 = t423*t1438;
  t2960 = t2921 + t2926;
  t3186 = Cos(var1[8]);
  t3187 = -1.*t3186;
  t3194 = 1. + t3187;
  t3231 = Sin(var1[8]);
  t3170 = t2753*t2960;
  t3178 = -1.*t2741*t2873;
  t3184 = t3170 + t3178;
  t3358 = -1.*t2518*t1635*t586;
  t3426 = t1617*t2350;
  t3444 = t3358 + t3426;
  t1903 = 0.087004*t1898;
  t2380 = 0.022225*t2350;
  t2424 = 0. + t1903 + t2380;
  t2524 = -0.022225*t1898;
  t2528 = 0.087004*t2350;
  t2575 = 0. + t2524 + t2528;
  t4638 = -1.*t439*t586;
  t4771 = t423*t641*t1438;
  t4778 = t4638 + t4771;
  t2788 = 0.157004*t2777;
  t2880 = -0.31508*t2873;
  t2886 = 0. + t2788 + t2880;
  t2964 = -0.31508*t2777;
  t3011 = -0.157004*t2873;
  t3066 = 0. + t2964 + t3011;
  t5012 = t1635*t4778;
  t5107 = -1.*t423*t2518*t2350;
  t5113 = t5012 + t5107;
  t5181 = t423*t439*t641;
  t5202 = t586*t1438;
  t5315 = t5181 + t5202;
  t3211 = -0.38008*t3194;
  t3242 = -0.022225*t3231;
  t3250 = 0. + t3211 + t3242;
  t3445 = -0.022225*t3194;
  t3563 = 0.38008*t3231;
  t3571 = 0. + t3445 + t3563;
  t5651 = t2753*t5315;
  t5677 = -1.*t5113*t2873;
  t5687 = t5651 + t5677;
  t5713 = t423*t2518*t1635;
  t5722 = t4778*t2350;
  t5744 = t5713 + t5722;
  t5844 = t423*t2518*t1635*t1438;
  t5862 = t423*t641*t2350;
  t5879 = t5844 + t5862;
  t5930 = t423*t2518*t439*t2753;
  t5941 = -1.*t5879*t2873;
  t5951 = t5930 + t5941;
  t5974 = -1.*t423*t1635*t641;
  t5982 = t423*t2518*t1438*t2350;
  t5984 = t5974 + t5982;
  t6063 = t2518*t1635*t586*t1438;
  t6085 = t586*t641*t2350;
  t6087 = t6063 + t6085;
  t6158 = t2518*t439*t2753*t586;
  t6162 = -1.*t6087*t2873;
  t6171 = t6158 + t6162;
  t6175 = -1.*t1635*t586*t641;
  t6220 = t2518*t586*t1438*t2350;
  t6232 = t6175 + t6220;
  t6325 = -1.*t1635*t641*t1438;
  t6326 = t2518*t2350;
  t6327 = t6325 + t6326;
  t6352 = -1.*t439*t2753*t641;
  t6358 = -1.*t6327*t2873;
  t6365 = t6352 + t6358;
  t6373 = -1.*t2518*t1635;
  t6374 = -1.*t641*t1438*t2350;
  t6375 = t6373 + t6374;
  t6446 = t439*t586;
  t6447 = -1.*t423*t641*t1438;
  t6452 = t6446 + t6447;
  t6487 = t2753*t6452;
  t6488 = -1.*t1635*t5315*t2873;
  t6490 = t6487 + t6488;
  t6541 = t439*t586*t641;
  t6546 = -1.*t423*t1438;
  t6549 = t6541 + t6546;
  t6594 = t2753*t1617;
  t6617 = -1.*t1635*t6549*t2873;
  t6627 = t6594 + t6617;
  t6747 = -1.*t2518*t2753*t1438;
  t6750 = -1.*t2518*t439*t1635*t2873;
  t6751 = t6747 + t6750;
  t6841 = -1.*t423*t2518*t1635;
  t6848 = -1.*t4778*t2350;
  t6857 = t6841 + t6848;
  t6800 = 0.087004*t1635;
  t6807 = -0.022225*t2350;
  t6808 = t6800 + t6807;
  t6826 = 0.022225*t1635;
  t6827 = t6826 + t2528;
  t6910 = t423*t439;
  t6925 = t586*t641*t1438;
  t6928 = t6910 + t6925;
  t6936 = -1.*t6928*t2350;
  t6942 = t3358 + t6936;
  t6997 = t1635*t6928;
  t7000 = -1.*t2518*t586*t2350;
  t7004 = t6997 + t7000;
  t7085 = t1635*t641;
  t7088 = -1.*t2518*t1438*t2350;
  t7096 = t7085 + t7088;
  t7139 = t2518*t1635*t1438;
  t7148 = t641*t2350;
  t7150 = t7139 + t7148;
  t7227 = -1.*t2753*t5113;
  t7230 = -1.*t5315*t2873;
  t7264 = t7227 + t7230;
  t7198 = -0.157004*t2753;
  t7204 = t7198 + t2880;
  t7210 = -0.31508*t2753;
  t7211 = 0.157004*t2873;
  t7215 = t7210 + t7211;
  t7295 = -1.*t2753*t7004;
  t7296 = -1.*t6549*t2873;
  t7306 = t7295 + t7296;
  t7440 = -1.*t2753*t7150;
  t7445 = -1.*t2518*t439*t2873;
  t7450 = t7440 + t7445;
  t5828 = t3186*t5744;
  t5833 = -1.*t5687*t3231;
  t5835 = t5828 + t5833;
  t7363 = t2753*t6549;
  t7377 = -1.*t7004*t2873;
  t7380 = t7363 + t7377;
  t7535 = -0.022225*t3186;
  t7539 = -0.38008*t3231;
  t7548 = t7535 + t7539;
  t7577 = 0.38008*t3186;
  t7578 = t7577 + t3242;
  t7655 = t2518*t1635*t586;
  t7660 = t6928*t2350;
  t7661 = t7655 + t7660;
  t7461 = t2518*t439*t2753;
  t7471 = -1.*t7150*t2873;
  t7477 = t7461 + t7471;
  t7772 = -1.*t1635*t641;
  t7799 = t2518*t1438*t2350;
  t7801 = t7772 + t7799;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1617*t2424 + t2741*t2886 + 0.167004*(t2741*t2753 + t2873*t2960) + t2960*t3066 + t3184*t3250 - 0.022225*(-1.*t3184*t3231 + t3186*t3444) - 0.38008*(t3184*t3186 + t3231*t3444) + t3444*t3571 - 1.*t2518*t2575*t586;
  p_output1(10)=t2518*t2575*t423 + t2424*t4778 + t2886*t5113 + t3066*t5315 + 0.167004*(t2753*t5113 + t2873*t5315) + t3250*t5687 + t3571*t5744 - 0.38008*(t3186*t5687 + t3231*t5744) - 0.022225*t5835;
  p_output1(11)=0;
  p_output1(12)=t1438*t2424*t2518*t423 + t2518*t3066*t423*t439 + t2886*t5879 + 0.167004*(t2518*t2873*t423*t439 + t2753*t5879) + t3250*t5951 + t3571*t5984 - 0.022225*(-1.*t3231*t5951 + t3186*t5984) - 0.38008*(t3186*t5951 + t3231*t5984) - 1.*t2575*t423*t641;
  p_output1(13)=t1438*t2424*t2518*t586 + t2518*t3066*t439*t586 + t2886*t6087 + 0.167004*(t2518*t2873*t439*t586 + t2753*t6087) + t3250*t6171 + t3571*t6232 - 0.022225*(-1.*t3231*t6171 + t3186*t6232) - 0.38008*(t3186*t6171 + t3231*t6232) - 1.*t2575*t586*t641;
  p_output1(14)=-1.*t2518*t2575 + t2886*t6327 + t3250*t6365 + t3571*t6375 - 0.022225*(-1.*t3231*t6365 + t3186*t6375) - 0.38008*(t3186*t6365 + t3231*t6375) - 1.*t1438*t2424*t641 - 1.*t3066*t439*t641 + 0.167004*(t2753*t6327 - 1.*t2873*t439*t641);
  p_output1(15)=t2424*t5315 + t1635*t2886*t5315 + t2350*t3571*t5315 + t3066*t6452 + 0.167004*(t1635*t2753*t5315 + t2873*t6452) + t3250*t6490 - 0.38008*(t2350*t3231*t5315 + t3186*t6490) - 0.022225*(t2350*t3186*t5315 - 1.*t3231*t6490);
  p_output1(16)=t1617*t3066 + t2424*t6549 + t1635*t2886*t6549 + t2350*t3571*t6549 + 0.167004*(t1617*t2873 + t1635*t2753*t6549) + t3250*t6627 - 0.38008*(t2350*t3231*t6549 + t3186*t6627) - 0.022225*(t2350*t3186*t6549 - 1.*t3231*t6627);
  p_output1(17)=-1.*t1438*t2518*t3066 + t2424*t2518*t439 + t1635*t2518*t2886*t439 + t2350*t2518*t3571*t439 + 0.167004*(-1.*t1438*t2518*t2873 + t1635*t2518*t2753*t439) + t3250*t6751 - 0.38008*(t2350*t2518*t3231*t439 + t3186*t6751) - 0.022225*(t2350*t2518*t3186*t439 - 1.*t3231*t6751);
  p_output1(18)=t3571*t5113 + t2518*t423*t6808 + t4778*t6827 + 0.167004*t2753*t6857 + t2886*t6857 - 1.*t2873*t3250*t6857 - 0.38008*(t3231*t5113 - 1.*t2873*t3186*t6857) - 0.022225*(t3186*t5113 + t2873*t3231*t6857);
  p_output1(19)=t2518*t586*t6808 + t6827*t6928 + 0.167004*t2753*t6942 + t2886*t6942 - 1.*t2873*t3250*t6942 + t3571*t7004 - 0.022225*(t2873*t3231*t6942 + t3186*t7004) - 0.38008*(-1.*t2873*t3186*t6942 + t3231*t7004);
  p_output1(20)=-1.*t641*t6808 + t1438*t2518*t6827 + 0.167004*t2753*t7096 + t2886*t7096 - 1.*t2873*t3250*t7096 + t3571*t7150 - 0.022225*(t2873*t3231*t7096 + t3186*t7150) - 0.38008*(-1.*t2873*t3186*t7096 + t3231*t7150);
  p_output1(21)=0.167004*t5687 + t5315*t7204 + t5113*t7215 - 0.38008*t3186*t7264 + 0.022225*t3231*t7264 + t3250*t7264;
  p_output1(22)=t6549*t7204 + t7004*t7215 - 0.38008*t3186*t7306 + 0.022225*t3231*t7306 + t3250*t7306 + 0.167004*t7380;
  p_output1(23)=t2518*t439*t7204 + t7150*t7215 - 0.38008*t3186*t7450 + 0.022225*t3231*t7450 + t3250*t7450 + 0.167004*t7477;
  p_output1(24)=-0.022225*(-1.*t3186*t5687 - 1.*t3231*t5744) - 0.38008*t5835 + t5687*t7548 + t5744*t7578;
  p_output1(25)=t7380*t7548 + t7578*t7661 - 0.38008*(-1.*t3231*t7380 + t3186*t7661) - 0.022225*(-1.*t3186*t7380 - 1.*t3231*t7661);
  p_output1(26)=t7477*t7548 + t7578*t7801 - 0.38008*(-1.*t3231*t7477 + t3186*t7801) - 0.022225*(-1.*t3186*t7477 - 1.*t3231*t7801);
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
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


       
void Jp_lHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
