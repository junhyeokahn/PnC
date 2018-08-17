/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:04 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootFront.h"

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
  double t482;
  double t1084;
  double t1102;
  double t1194;
  double t1284;
  double t47;
  double t137;
  double t220;
  double t529;
  double t541;
  double t883;
  double t974;
  double t1541;
  double t2130;
  double t2140;
  double t2145;
  double t2234;
  double t1769;
  double t1888;
  double t1932;
  double t2435;
  double t2477;
  double t2643;
  double t3077;
  double t3106;
  double t3140;
  double t3618;
  double t3042;
  double t3049;
  double t3050;
  double t3770;
  double t3862;
  double t3866;
  double t4025;
  double t4036;
  double t4091;
  double t4376;
  double t4600;
  double t4623;
  double t4648;
  double t4793;
  double t4814;
  double t4836;
  double t4944;
  double t4981;
  double t4982;
  double t5077;
  double t5132;
  double t5136;
  double t5147;
  double t5249;
  double t5265;
  double t5275;
  double t1222;
  double t1297;
  double t1318;
  double t1647;
  double t1651;
  double t1715;
  double t5520;
  double t5525;
  double t5540;
  double t2181;
  double t2243;
  double t2291;
  double t2799;
  double t2887;
  double t2922;
  double t5580;
  double t5593;
  double t5617;
  double t5653;
  double t5661;
  double t5695;
  double t3144;
  double t3689;
  double t3703;
  double t3909;
  double t3923;
  double t4003;
  double t4193;
  double t4402;
  double t4432;
  double t5786;
  double t5810;
  double t5819;
  double t5828;
  double t5856;
  double t5870;
  double t4684;
  double t4735;
  double t4738;
  double t5073;
  double t5079;
  double t5090;
  double t5878;
  double t5887;
  double t5892;
  double t5909;
  double t5912;
  double t5917;
  double t5191;
  double t5225;
  double t5239;
  double t5926;
  double t5930;
  double t5960;
  double t5970;
  double t5972;
  double t5976;
  double t6128;
  double t6149;
  double t6156;
  double t6235;
  double t6251;
  double t6253;
  double t6282;
  double t6309;
  double t6313;
  double t6324;
  double t6346;
  double t6351;
  double t6386;
  double t6401;
  double t6421;
  double t6432;
  double t6437;
  double t6442;
  double t6464;
  double t6472;
  double t6486;
  t482 = Cos(var1[3]);
  t1084 = Cos(var1[6]);
  t1102 = -1.*t1084;
  t1194 = 1. + t1102;
  t1284 = Sin(var1[6]);
  t47 = Cos(var1[5]);
  t137 = Sin(var1[3]);
  t220 = -1.*t47*t137;
  t529 = Sin(var1[4]);
  t541 = Sin(var1[5]);
  t883 = t482*t529*t541;
  t974 = t220 + t883;
  t1541 = Cos(var1[4]);
  t2130 = Cos(var1[7]);
  t2140 = -1.*t2130;
  t2145 = 1. + t2140;
  t2234 = Sin(var1[7]);
  t1769 = t1084*t974;
  t1888 = -1.*t482*t1541*t1284;
  t1932 = t1769 + t1888;
  t2435 = t482*t47*t529;
  t2477 = t137*t541;
  t2643 = t2435 + t2477;
  t3077 = Cos(var1[8]);
  t3106 = -1.*t3077;
  t3140 = 1. + t3106;
  t3618 = Sin(var1[8]);
  t3042 = t2130*t2643;
  t3049 = -1.*t1932*t2234;
  t3050 = t3042 + t3049;
  t3770 = t482*t1541*t1084;
  t3862 = t974*t1284;
  t3866 = t3770 + t3862;
  t4025 = Cos(var1[9]);
  t4036 = -1.*t4025;
  t4091 = 1. + t4036;
  t4376 = Sin(var1[9]);
  t4600 = t3077*t3050;
  t4623 = t3866*t3618;
  t4648 = t4600 + t4623;
  t4793 = t3077*t3866;
  t4814 = -1.*t3050*t3618;
  t4836 = t4793 + t4814;
  t4944 = Cos(var1[10]);
  t4981 = -1.*t4944;
  t4982 = 1. + t4981;
  t5077 = Sin(var1[10]);
  t5132 = -1.*t4376*t4648;
  t5136 = t4025*t4836;
  t5147 = t5132 + t5136;
  t5249 = t4025*t4648;
  t5265 = t4376*t4836;
  t5275 = t5249 + t5265;
  t1222 = 0.087004*t1194;
  t1297 = 0.022225*t1284;
  t1318 = 0. + t1222 + t1297;
  t1647 = -0.022225*t1194;
  t1651 = 0.087004*t1284;
  t1715 = 0. + t1647 + t1651;
  t5520 = t482*t47;
  t5525 = t137*t529*t541;
  t5540 = t5520 + t5525;
  t2181 = 0.157004*t2145;
  t2243 = -0.31508*t2234;
  t2291 = 0. + t2181 + t2243;
  t2799 = -0.31508*t2145;
  t2887 = -0.157004*t2234;
  t2922 = 0. + t2799 + t2887;
  t5580 = t1084*t5540;
  t5593 = -1.*t1541*t137*t1284;
  t5617 = t5580 + t5593;
  t5653 = t47*t137*t529;
  t5661 = -1.*t482*t541;
  t5695 = t5653 + t5661;
  t3144 = -0.38008*t3140;
  t3689 = -0.022225*t3618;
  t3703 = 0. + t3144 + t3689;
  t3909 = -0.022225*t3140;
  t3923 = 0.38008*t3618;
  t4003 = 0. + t3909 + t3923;
  t4193 = -0.86008*t4091;
  t4402 = -0.022225*t4376;
  t4432 = 0. + t4193 + t4402;
  t5786 = t2130*t5695;
  t5810 = -1.*t5617*t2234;
  t5819 = t5786 + t5810;
  t5828 = t1541*t1084*t137;
  t5856 = t5540*t1284;
  t5870 = t5828 + t5856;
  t4684 = -0.022225*t4091;
  t4735 = 0.86008*t4376;
  t4738 = 0. + t4684 + t4735;
  t5073 = -0.021147*t4982;
  t5079 = 1.34008*t5077;
  t5090 = 0. + t5073 + t5079;
  t5878 = t3077*t5819;
  t5887 = t5870*t3618;
  t5892 = t5878 + t5887;
  t5909 = t3077*t5870;
  t5912 = -1.*t5819*t3618;
  t5917 = t5909 + t5912;
  t5191 = -1.34008*t4982;
  t5225 = -0.021147*t5077;
  t5239 = 0. + t5191 + t5225;
  t5926 = -1.*t4376*t5892;
  t5930 = t4025*t5917;
  t5960 = t5926 + t5930;
  t5970 = t4025*t5892;
  t5972 = t4376*t5917;
  t5976 = t5970 + t5972;
  t6128 = t1541*t1084*t541;
  t6149 = t529*t1284;
  t6156 = t6128 + t6149;
  t6235 = t1541*t47*t2130;
  t6251 = -1.*t6156*t2234;
  t6253 = t6235 + t6251;
  t6282 = -1.*t1084*t529;
  t6309 = t1541*t541*t1284;
  t6313 = t6282 + t6309;
  t6324 = t3077*t6253;
  t6346 = t6313*t3618;
  t6351 = t6324 + t6346;
  t6386 = t3077*t6313;
  t6401 = -1.*t6253*t3618;
  t6421 = t6386 + t6401;
  t6432 = -1.*t4376*t6351;
  t6437 = t4025*t6421;
  t6442 = t6432 + t6437;
  t6464 = t4025*t6351;
  t6472 = t4376*t6421;
  t6486 = t6464 + t6472;

  p_output1(0)=0. + t1932*t2291 + 0.167004*(t1932*t2130 + t2234*t2643) + t2643*t2922 + t3050*t3703 + t3866*t4003 + t4432*t4648 + t1541*t1715*t482 + t4738*t4836 + t5090*t5147 + t5239*t5275 - 1.250132*(t5077*t5147 + t4944*t5275) + 0.043925*(t4944*t5147 - 1.*t5077*t5275) + t1318*t974 + var1(0);
  p_output1(1)=0. + t137*t1541*t1715 + t1318*t5540 + t2291*t5617 + t2922*t5695 + 0.167004*(t2130*t5617 + t2234*t5695) + t3703*t5819 + t4003*t5870 + t4432*t5892 + t4738*t5917 + t5090*t5960 + t5239*t5976 - 1.250132*(t5077*t5960 + t4944*t5976) + 0.043925*(t4944*t5960 - 1.*t5077*t5976) + var1(1);
  p_output1(2)=0. + t1541*t2922*t47 - 1.*t1715*t529 + t1318*t1541*t541 + t2291*t6156 + 0.167004*(t1541*t2234*t47 + t2130*t6156) + t3703*t6253 + t4003*t6313 + t4432*t6351 + t4738*t6421 + t5090*t6442 + t5239*t6486 - 1.250132*(t5077*t6442 + t4944*t6486) + 0.043925*(t4944*t6442 - 1.*t5077*t6486) + var1(2);
}


       
void p_LeftFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
