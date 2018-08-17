/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootBottom.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t29;
  double t193;
  double t354;
  double t1204;
  double t1486;
  double t3392;
  double t3462;
  double t3441;
  double t3472;
  double t2260;
  double t2599;
  double t2607;
  double t2972;
  double t80;
  double t3692;
  double t3701;
  double t3725;
  double t3455;
  double t3479;
  double t3576;
  double t3837;
  double t3900;
  double t4003;
  double t4130;
  double t4146;
  double t4186;
  double t4356;
  double t4494;
  double t4650;
  double t4666;
  double t4920;
  double t4955;
  double t4981;
  double t5014;
  double t5021;
  double t5109;
  double t5142;
  double t5198;
  double t5218;
  double t5262;
  double t5420;
  double t5441;
  double t5449;
  double t5459;
  double t5463;
  double t5466;
  double t5488;
  double t5514;
  double t5534;
  double t5554;
  double t5567;
  double t5574;
  double t5577;
  double t1300;
  double t1601;
  double t1735;
  double t2877;
  double t3052;
  double t3179;
  double t3616;
  double t3626;
  double t3636;
  double t3749;
  double t3750;
  double t3761;
  double t5760;
  double t5788;
  double t5790;
  double t4264;
  double t4375;
  double t4403;
  double t5721;
  double t5724;
  double t5733;
  double t5835;
  double t5867;
  double t5887;
  double t4872;
  double t4891;
  double t4915;
  double t5119;
  double t5143;
  double t5196;
  double t5907;
  double t5910;
  double t5934;
  double t5989;
  double t6017;
  double t6022;
  double t5356;
  double t5397;
  double t5401;
  double t5480;
  double t5500;
  double t5509;
  double t6055;
  double t6085;
  double t6116;
  double t6127;
  double t6144;
  double t6151;
  double t5563;
  double t5564;
  double t5565;
  double t6182;
  double t6221;
  double t6228;
  double t6250;
  double t6298;
  double t6319;
  double t6575;
  double t6578;
  double t6579;
  double t6610;
  double t6615;
  double t6617;
  double t6637;
  double t6649;
  double t6650;
  double t6665;
  double t6667;
  double t6685;
  double t6700;
  double t6714;
  double t6724;
  double t6754;
  double t6755;
  double t6760;
  double t6790;
  double t6791;
  double t6801;
  t29 = Cos(var1[3]);
  t193 = Cos(var1[11]);
  t354 = -1.*t193;
  t1204 = 1. + t354;
  t1486 = Sin(var1[11]);
  t3392 = Cos(var1[5]);
  t3462 = Sin(var1[3]);
  t3441 = Sin(var1[4]);
  t3472 = Sin(var1[5]);
  t2260 = Cos(var1[12]);
  t2599 = -1.*t2260;
  t2607 = 1. + t2599;
  t2972 = Sin(var1[12]);
  t80 = Cos(var1[4]);
  t3692 = -1.*t3392*t3462;
  t3701 = t29*t3441*t3472;
  t3725 = t3692 + t3701;
  t3455 = t29*t3392*t3441;
  t3479 = t3462*t3472;
  t3576 = t3455 + t3479;
  t3837 = -1.*t29*t80*t1486;
  t3900 = t193*t3725;
  t4003 = t3837 + t3900;
  t4130 = Cos(var1[13]);
  t4146 = -1.*t4130;
  t4186 = 1. + t4146;
  t4356 = Sin(var1[13]);
  t4494 = t193*t29*t80;
  t4650 = t1486*t3725;
  t4666 = t4494 + t4650;
  t4920 = t2260*t3576;
  t4955 = -1.*t2972*t4003;
  t4981 = t4920 + t4955;
  t5014 = Cos(var1[14]);
  t5021 = -1.*t5014;
  t5109 = 1. + t5021;
  t5142 = Sin(var1[14]);
  t5198 = t4356*t4666;
  t5218 = t4130*t4981;
  t5262 = t5198 + t5218;
  t5420 = t4130*t4666;
  t5441 = -1.*t4356*t4981;
  t5449 = t5420 + t5441;
  t5459 = Cos(var1[15]);
  t5463 = -1.*t5459;
  t5466 = 1. + t5463;
  t5488 = Sin(var1[15]);
  t5514 = -1.*t5142*t5262;
  t5534 = t5014*t5449;
  t5554 = t5514 + t5534;
  t5567 = t5014*t5262;
  t5574 = t5142*t5449;
  t5577 = t5567 + t5574;
  t1300 = -0.022225*t1204;
  t1601 = -0.086996*t1486;
  t1735 = 0. + t1300 + t1601;
  t2877 = -0.31508*t2607;
  t3052 = 0.156996*t2972;
  t3179 = 0. + t2877 + t3052;
  t3616 = -0.086996*t1204;
  t3626 = 0.022225*t1486;
  t3636 = 0. + t3616 + t3626;
  t3749 = -0.156996*t2607;
  t3750 = -0.31508*t2972;
  t3761 = 0. + t3749 + t3750;
  t5760 = t29*t3392;
  t5788 = t3462*t3441*t3472;
  t5790 = t5760 + t5788;
  t4264 = -0.022225*t4186;
  t4375 = 0.38008*t4356;
  t4403 = 0. + t4264 + t4375;
  t5721 = t3392*t3462*t3441;
  t5724 = -1.*t29*t3472;
  t5733 = t5721 + t5724;
  t5835 = -1.*t80*t1486*t3462;
  t5867 = t193*t5790;
  t5887 = t5835 + t5867;
  t4872 = -0.38008*t4186;
  t4891 = -0.022225*t4356;
  t4915 = 0. + t4872 + t4891;
  t5119 = -0.86008*t5109;
  t5143 = -0.022225*t5142;
  t5196 = 0. + t5119 + t5143;
  t5907 = t193*t80*t3462;
  t5910 = t1486*t5790;
  t5934 = t5907 + t5910;
  t5989 = t2260*t5733;
  t6017 = -1.*t2972*t5887;
  t6022 = t5989 + t6017;
  t5356 = -0.022225*t5109;
  t5397 = 0.86008*t5142;
  t5401 = 0. + t5356 + t5397;
  t5480 = -0.021147*t5466;
  t5500 = 1.34008*t5488;
  t5509 = 0. + t5480 + t5500;
  t6055 = t4356*t5934;
  t6085 = t4130*t6022;
  t6116 = t6055 + t6085;
  t6127 = t4130*t5934;
  t6144 = -1.*t4356*t6022;
  t6151 = t6127 + t6144;
  t5563 = -1.34008*t5466;
  t5564 = -0.021147*t5488;
  t5565 = 0. + t5563 + t5564;
  t6182 = -1.*t5142*t6116;
  t6221 = t5014*t6151;
  t6228 = t6182 + t6221;
  t6250 = t5014*t6116;
  t6298 = t5142*t6151;
  t6319 = t6250 + t6298;
  t6575 = t1486*t3441;
  t6578 = t193*t80*t3472;
  t6579 = t6575 + t6578;
  t6610 = -1.*t193*t3441;
  t6615 = t80*t1486*t3472;
  t6617 = t6610 + t6615;
  t6637 = t2260*t80*t3392;
  t6649 = -1.*t2972*t6579;
  t6650 = t6637 + t6649;
  t6665 = t4356*t6617;
  t6667 = t4130*t6650;
  t6685 = t6665 + t6667;
  t6700 = t4130*t6617;
  t6714 = -1.*t4356*t6650;
  t6724 = t6700 + t6714;
  t6754 = -1.*t5142*t6685;
  t6755 = t5014*t6724;
  t6760 = t6754 + t6755;
  t6790 = t5014*t6685;
  t6791 = t5142*t6724;
  t6801 = t6790 + t6791;

  p_output1(0)=0. + t3179*t3576 + t3636*t3725 + t3761*t4003 - 0.166996*(t2972*t3576 + t2260*t4003) + t4403*t4666 + t4915*t4981 + t5196*t5262 + t5401*t5449 + t5509*t5554 + t5565*t5577 - 1.325132*(t5488*t5554 + t5459*t5577) + 0.043865*(t5459*t5554 - 1.*t5488*t5577) + t1735*t29*t80 + var1(0);
  p_output1(1)=0. + t3179*t5733 + t3636*t5790 + t3761*t5887 - 0.166996*(t2972*t5733 + t2260*t5887) + t4403*t5934 + t4915*t6022 + t5196*t6116 + t5401*t6151 + t5509*t6228 + t5565*t6319 - 1.325132*(t5488*t6228 + t5459*t6319) + 0.043865*(t5459*t6228 - 1.*t5488*t6319) + t1735*t3462*t80 + var1(1);
  p_output1(2)=0. - 1.*t1735*t3441 + t3761*t6579 + t4403*t6617 + t4915*t6650 + t5196*t6685 + t5401*t6724 + t5509*t6760 + t5565*t6801 - 1.325132*(t5488*t6760 + t5459*t6801) + 0.043865*(t5459*t6760 - 1.*t5488*t6801) + t3179*t3392*t80 + t3472*t3636*t80 - 0.166996*(t2260*t6579 + t2972*t3392*t80) + var1(2);
}


       
void p_RightFootBottom(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
