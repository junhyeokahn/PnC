/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:57 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipPitch.h"

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
  double t978;
  double t185;
  double t419;
  double t466;
  double t593;
  double t1867;
  double t1778;
  double t1793;
  double t1904;
  double t1178;
  double t1198;
  double t1591;
  double t1753;
  double t127;
  double t2512;
  double t2561;
  double t2720;
  double t1815;
  double t1922;
  double t1954;
  double t3155;
  double t3191;
  double t3426;
  double t3602;
  double t3628;
  double t3633;
  double t3941;
  double t4438;
  double t4484;
  double t4656;
  double t5261;
  double t5284;
  double t5322;
  double t563;
  double t903;
  double t941;
  double t1607;
  double t1760;
  double t1763;
  double t2249;
  double t2277;
  double t2383;
  double t2788;
  double t2926;
  double t3100;
  double t6127;
  double t6132;
  double t6199;
  double t3940;
  double t3986;
  double t4336;
  double t6048;
  double t6052;
  double t6065;
  double t6208;
  double t6216;
  double t6227;
  double t5020;
  double t5039;
  double t5101;
  double t6280;
  double t6382;
  double t6391;
  double t6483;
  double t6511;
  double t6513;
  double t6659;
  double t6669;
  double t6678;
  double t6680;
  double t6681;
  double t6682;
  double t6840;
  double t6858;
  double t6862;
  double t6976;
  double t6988;
  double t6989;
  double t7004;
  double t7026;
  double t7028;
  double t7091;
  double t7102;
  double t7103;
  double t7207;
  double t7211;
  double t7213;
  double t7230;
  double t7240;
  double t7243;
  double t7286;
  double t7291;
  double t7294;
  double t7402;
  double t7421;
  double t7432;
  double t7435;
  double t7440;
  double t7442;
  double t7495;
  double t7503;
  double t7510;
  double t7544;
  double t7545;
  double t7554;
  double t7642;
  double t7644;
  double t7647;
  double t7737;
  double t7741;
  double t7754;
  double t7704;
  double t7705;
  double t7707;
  double t7709;
  double t7711;
  double t7830;
  double t7835;
  double t7839;
  double t7859;
  double t7862;
  double t7844;
  double t7846;
  double t7854;
  double t7934;
  double t7937;
  double t7944;
  double t7916;
  double t7922;
  double t7923;
  double t8038;
  double t8042;
  double t8047;
  double t8011;
  double t8016;
  double t8022;
  double t8028;
  double t8030;
  double t8090;
  double t8109;
  double t8118;
  double t8201;
  double t8203;
  double t8204;
  double t6574;
  double t6605;
  double t6609;
  double t8245;
  double t8254;
  double t8261;
  double t8263;
  double t8266;
  double t8151;
  double t8154;
  double t8173;
  double t8311;
  double t8312;
  double t8315;
  double t8218;
  double t8227;
  double t8229;
  double t8381;
  double t8383;
  double t8384;
  t978 = Sin(var1[3]);
  t185 = Cos(var1[11]);
  t419 = -1.*t185;
  t466 = 1. + t419;
  t593 = Sin(var1[11]);
  t1867 = Cos(var1[3]);
  t1778 = Cos(var1[5]);
  t1793 = Sin(var1[4]);
  t1904 = Sin(var1[5]);
  t1178 = Cos(var1[12]);
  t1198 = -1.*t1178;
  t1591 = 1. + t1198;
  t1753 = Sin(var1[12]);
  t127 = Cos(var1[4]);
  t2512 = -1.*t1867*t1778;
  t2561 = -1.*t978*t1793*t1904;
  t2720 = t2512 + t2561;
  t1815 = -1.*t1778*t978*t1793;
  t1922 = t1867*t1904;
  t1954 = t1815 + t1922;
  t3155 = t127*t593*t978;
  t3191 = t185*t2720;
  t3426 = t3155 + t3191;
  t3602 = Cos(var1[13]);
  t3628 = -1.*t3602;
  t3633 = 1. + t3628;
  t3941 = Sin(var1[13]);
  t4438 = -1.*t185*t127*t978;
  t4484 = t593*t2720;
  t4656 = t4438 + t4484;
  t5261 = t1178*t1954;
  t5284 = -1.*t1753*t3426;
  t5322 = t5261 + t5284;
  t563 = -0.022225*t466;
  t903 = -0.086996*t593;
  t941 = 0. + t563 + t903;
  t1607 = -0.31508*t1591;
  t1760 = 0.156996*t1753;
  t1763 = 0. + t1607 + t1760;
  t2249 = -0.086996*t466;
  t2277 = 0.022225*t593;
  t2383 = 0. + t2249 + t2277;
  t2788 = -0.156996*t1591;
  t2926 = -0.31508*t1753;
  t3100 = 0. + t2788 + t2926;
  t6127 = -1.*t1778*t978;
  t6132 = t1867*t1793*t1904;
  t6199 = t6127 + t6132;
  t3940 = -0.022225*t3633;
  t3986 = 0.38008*t3941;
  t4336 = 0. + t3940 + t3986;
  t6048 = t1867*t1778*t1793;
  t6052 = t978*t1904;
  t6065 = t6048 + t6052;
  t6208 = -1.*t1867*t127*t593;
  t6216 = t185*t6199;
  t6227 = t6208 + t6216;
  t5020 = -0.38008*t3633;
  t5039 = -0.022225*t3941;
  t5101 = 0. + t5020 + t5039;
  t6280 = t185*t1867*t127;
  t6382 = t593*t6199;
  t6391 = t6280 + t6382;
  t6483 = t1178*t6065;
  t6511 = -1.*t1753*t6227;
  t6513 = t6483 + t6511;
  t6659 = t1867*t593*t1793;
  t6669 = t185*t1867*t127*t1904;
  t6678 = t6659 + t6669;
  t6680 = -1.*t185*t1867*t1793;
  t6681 = t1867*t127*t593*t1904;
  t6682 = t6680 + t6681;
  t6840 = t1178*t1867*t127*t1778;
  t6858 = -1.*t1753*t6678;
  t6862 = t6840 + t6858;
  t6976 = t593*t978*t1793;
  t6988 = t185*t127*t978*t1904;
  t6989 = t6976 + t6988;
  t7004 = -1.*t185*t978*t1793;
  t7026 = t127*t593*t978*t1904;
  t7028 = t7004 + t7026;
  t7091 = t1178*t127*t1778*t978;
  t7102 = -1.*t1753*t6989;
  t7103 = t7091 + t7102;
  t7207 = t127*t593;
  t7211 = -1.*t185*t1793*t1904;
  t7213 = t7207 + t7211;
  t7230 = -1.*t185*t127;
  t7240 = -1.*t593*t1793*t1904;
  t7243 = t7230 + t7240;
  t7286 = -1.*t1178*t1778*t1793;
  t7291 = -1.*t1753*t7213;
  t7294 = t7286 + t7291;
  t7402 = t1778*t978;
  t7421 = -1.*t1867*t1793*t1904;
  t7432 = t7402 + t7421;
  t7435 = -1.*t185*t1753*t6065;
  t7440 = t1178*t7432;
  t7442 = t7435 + t7440;
  t7495 = t1778*t978*t1793;
  t7503 = -1.*t1867*t1904;
  t7510 = t7495 + t7503;
  t7544 = -1.*t185*t1753*t7510;
  t7545 = t1178*t2720;
  t7554 = t7544 + t7545;
  t7642 = -1.*t185*t127*t1778*t1753;
  t7644 = -1.*t1178*t127*t1904;
  t7647 = t7642 + t7644;
  t7737 = -1.*t185*t1867*t127;
  t7741 = -1.*t593*t6199;
  t7754 = t7737 + t7741;
  t7704 = -0.086996*t185;
  t7705 = -0.022225*t593;
  t7707 = t7704 + t7705;
  t7709 = 0.022225*t185;
  t7711 = t7709 + t903;
  t7830 = t1867*t1778;
  t7835 = t978*t1793*t1904;
  t7839 = t7830 + t7835;
  t7859 = -1.*t593*t7839;
  t7862 = t4438 + t7859;
  t7844 = -1.*t127*t593*t978;
  t7846 = t185*t7839;
  t7854 = t7844 + t7846;
  t7934 = t185*t1793;
  t7937 = -1.*t127*t593*t1904;
  t7944 = t7934 + t7937;
  t7916 = t593*t1793;
  t7922 = t185*t127*t1904;
  t7923 = t7916 + t7922;
  t8038 = -1.*t1753*t6065;
  t8042 = -1.*t1178*t6227;
  t8047 = t8038 + t8042;
  t8011 = 0.156996*t1178;
  t8016 = t8011 + t2926;
  t8022 = -0.31508*t1178;
  t8028 = -0.156996*t1753;
  t8030 = t8022 + t8028;
  t8090 = -1.*t1753*t7510;
  t8109 = -1.*t1178*t7854;
  t8118 = t8090 + t8109;
  t8201 = -1.*t127*t1778*t1753;
  t8203 = -1.*t1178*t7923;
  t8204 = t8201 + t8203;
  t6574 = t3602*t6391;
  t6605 = -1.*t3941*t6513;
  t6609 = t6574 + t6605;
  t8245 = 0.38008*t3602;
  t8254 = t8245 + t5039;
  t8261 = -0.022225*t3602;
  t8263 = -0.38008*t3941;
  t8266 = t8261 + t8263;
  t8151 = t1178*t7510;
  t8154 = -1.*t1753*t7854;
  t8173 = t8151 + t8154;
  t8311 = t185*t127*t978;
  t8312 = t593*t7839;
  t8315 = t8311 + t8312;
  t8218 = t1178*t127*t1778;
  t8227 = -1.*t1753*t7923;
  t8229 = t8218 + t8227;
  t8381 = -1.*t185*t1793;
  t8383 = t127*t593*t1904;
  t8384 = t8381 + t8383;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1763*t1954 + t2383*t2720 + t3100*t3426 - 0.166996*(t1753*t1954 + t1178*t3426) + t4336*t4656 + t5101*t5322 - 0.38008*(t3941*t4656 + t3602*t5322) - 0.022225*(t3602*t4656 - 1.*t3941*t5322) - 1.*t127*t941*t978;
  p_output1(10)=t1763*t6065 + t2383*t6199 + t3100*t6227 - 0.166996*(t1753*t6065 + t1178*t6227) + t4336*t6391 + t5101*t6513 - 0.38008*(t3941*t6391 + t3602*t6513) - 0.022225*t6609 + t127*t1867*t941;
  p_output1(11)=0;
  p_output1(12)=t127*t1763*t1778*t1867 + t127*t1867*t1904*t2383 + t3100*t6678 - 0.166996*(t127*t1753*t1778*t1867 + t1178*t6678) + t4336*t6682 + t5101*t6862 - 0.38008*(t3941*t6682 + t3602*t6862) - 0.022225*(t3602*t6682 - 1.*t3941*t6862) - 1.*t1793*t1867*t941;
  p_output1(13)=t3100*t6989 + t4336*t7028 + t5101*t7103 - 0.38008*(t3941*t7028 + t3602*t7103) - 0.022225*(t3602*t7028 - 1.*t3941*t7103) + t127*t1763*t1778*t978 + t127*t1904*t2383*t978 - 1.*t1793*t941*t978 - 0.166996*(t1178*t6989 + t127*t1753*t1778*t978);
  p_output1(14)=-1.*t1763*t1778*t1793 - 1.*t1793*t1904*t2383 + t3100*t7213 - 0.166996*(-1.*t1753*t1778*t1793 + t1178*t7213) + t4336*t7243 + t5101*t7294 - 0.38008*(t3941*t7243 + t3602*t7294) - 0.022225*(t3602*t7243 - 1.*t3941*t7294) - 1.*t127*t941;
  p_output1(15)=t2383*t6065 + t185*t3100*t6065 + t4336*t593*t6065 + t1763*t7432 - 0.166996*(t1178*t185*t6065 + t1753*t7432) + t5101*t7442 - 0.38008*(t3941*t593*t6065 + t3602*t7442) - 0.022225*(t3602*t593*t6065 - 1.*t3941*t7442);
  p_output1(16)=t1763*t2720 + t2383*t7510 + t185*t3100*t7510 + t4336*t593*t7510 - 0.166996*(t1753*t2720 + t1178*t185*t7510) + t5101*t7554 - 0.38008*(t3941*t593*t7510 + t3602*t7554) - 0.022225*(t3602*t593*t7510 - 1.*t3941*t7554);
  p_output1(17)=-1.*t127*t1763*t1904 - 0.166996*(t1178*t127*t1778*t185 - 1.*t127*t1753*t1904) + t127*t1778*t2383 + t127*t1778*t185*t3100 + t127*t1778*t4336*t593 + t5101*t7647 - 0.38008*(t127*t1778*t3941*t593 + t3602*t7647) - 0.022225*(t127*t1778*t3602*t593 - 1.*t3941*t7647);
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
  p_output1(33)=t4336*t6227 + t127*t1867*t7707 + t6199*t7711 - 0.166996*t1178*t7754 + t3100*t7754 - 1.*t1753*t5101*t7754 - 0.38008*(t3941*t6227 - 1.*t1753*t3602*t7754) - 0.022225*(t3602*t6227 + t1753*t3941*t7754);
  p_output1(34)=t7711*t7839 + t4336*t7854 - 0.166996*t1178*t7862 + t3100*t7862 - 1.*t1753*t5101*t7862 - 0.38008*(t3941*t7854 - 1.*t1753*t3602*t7862) - 0.022225*(t3602*t7854 + t1753*t3941*t7862) + t127*t7707*t978;
  p_output1(35)=-1.*t1793*t7707 + t127*t1904*t7711 + t4336*t7923 - 0.166996*t1178*t7944 + t3100*t7944 - 1.*t1753*t5101*t7944 - 0.38008*(t3941*t7923 - 1.*t1753*t3602*t7944) - 0.022225*(t3602*t7923 + t1753*t3941*t7944);
  p_output1(36)=-0.166996*t6513 + t6065*t8016 + t6227*t8030 - 0.38008*t3602*t8047 + 0.022225*t3941*t8047 + t5101*t8047;
  p_output1(37)=t7510*t8016 + t7854*t8030 - 0.38008*t3602*t8118 + 0.022225*t3941*t8118 + t5101*t8118 - 0.166996*t8173;
  p_output1(38)=t127*t1778*t8016 + t7923*t8030 - 0.38008*t3602*t8204 + 0.022225*t3941*t8204 + t5101*t8204 - 0.166996*t8229;
  p_output1(39)=-0.022225*(-1.*t3941*t6391 - 1.*t3602*t6513) - 0.38008*t6609 + t6391*t8254 + t6513*t8266;
  p_output1(40)=t8173*t8266 + t8254*t8315 - 0.38008*(-1.*t3941*t8173 + t3602*t8315) - 0.022225*(-1.*t3602*t8173 - 1.*t3941*t8315);
  p_output1(41)=t8229*t8266 + t8254*t8384 - 0.38008*(-1.*t3941*t8229 + t3602*t8384) - 0.022225*(-1.*t3602*t8229 - 1.*t3941*t8384);
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
