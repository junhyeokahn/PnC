/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:11 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootFront.h"

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
  double t476;
  double t832;
  double t1021;
  double t857;
  double t1030;
  double t645;
  double t806;
  double t471;
  double t939;
  double t1039;
  double t1059;
  double t1215;
  double t768;
  double t1135;
  double t1159;
  double t461;
  double t1271;
  double t1295;
  double t1323;
  double t1324;
  double t1346;
  double t1381;
  double t1429;
  double t1457;
  double t1491;
  double t1493;
  double t1521;
  double t1620;
  double t1212;
  double t1542;
  double t1556;
  double t319;
  double t1623;
  double t1650;
  double t1685;
  double t1776;
  double t1616;
  double t1686;
  double t1719;
  double t54;
  double t1813;
  double t1831;
  double t1852;
  double t2445;
  double t2476;
  double t2484;
  double t2411;
  double t2487;
  double t2488;
  double t2510;
  double t2593;
  double t2612;
  double t2617;
  double t2627;
  double t2708;
  double t2711;
  double t2764;
  double t2772;
  double t2491;
  double t2789;
  double t2791;
  double t2900;
  double t2901;
  double t2903;
  double t2860;
  double t2945;
  double t2987;
  double t3020;
  double t3048;
  double t3055;
  double t3153;
  double t3157;
  double t3159;
  double t3174;
  double t3184;
  double t3188;
  double t3192;
  double t3193;
  double t3194;
  double t3171;
  double t3209;
  double t3216;
  double t3237;
  double t3240;
  double t3253;
  double t3230;
  double t3254;
  double t3256;
  double t3265;
  double t3275;
  double t3281;
  double t1767;
  double t1854;
  double t2992;
  double t3076;
  double t3260;
  double t3285;
  double t3735;
  double t3780;
  double t3931;
  double t3954;
  double t3345;
  double t3353;
  double t3367;
  double t4139;
  double t4144;
  double t4300;
  double t4301;
  double t4447;
  double t4449;
  double t3517;
  double t1921;
  double t2034;
  double t2126;
  double t3812;
  double t3846;
  double t3875;
  double t4005;
  double t4014;
  double t4047;
  double t4085;
  double t4106;
  double t4109;
  double t4122;
  double t4123;
  double t4128;
  double t4157;
  double t4169;
  double t4204;
  double t3372;
  double t3377;
  double t3417;
  double t4267;
  double t4280;
  double t4285;
  double t4327;
  double t4348;
  double t4367;
  double t4377;
  double t4380;
  double t4395;
  double t4517;
  double t4572;
  double t4576;
  double t4583;
  double t4584;
  double t4586;
  double t3650;
  double t3099;
  double t3112;
  double t3137;
  double t3439;
  double t3452;
  double t3488;
  double t3697;
  double t3295;
  double t3306;
  double t3328;
  t476 = Cos(var1[3]);
  t832 = Cos(var1[5]);
  t1021 = Sin(var1[4]);
  t857 = Sin(var1[3]);
  t1030 = Sin(var1[5]);
  t645 = Cos(var1[4]);
  t806 = Sin(var1[11]);
  t471 = Cos(var1[11]);
  t939 = -1.*t832*t857;
  t1039 = t476*t1021*t1030;
  t1059 = t939 + t1039;
  t1215 = Cos(var1[13]);
  t768 = t471*t476*t645;
  t1135 = t806*t1059;
  t1159 = t768 + t1135;
  t461 = Sin(var1[13]);
  t1271 = Cos(var1[12]);
  t1295 = t476*t832*t1021;
  t1323 = t857*t1030;
  t1324 = t1295 + t1323;
  t1346 = t1271*t1324;
  t1381 = Sin(var1[12]);
  t1429 = -1.*t476*t645*t806;
  t1457 = t471*t1059;
  t1491 = t1429 + t1457;
  t1493 = -1.*t1381*t1491;
  t1521 = t1346 + t1493;
  t1620 = Cos(var1[14]);
  t1212 = t461*t1159;
  t1542 = t1215*t1521;
  t1556 = t1212 + t1542;
  t319 = Sin(var1[14]);
  t1623 = t1215*t1159;
  t1650 = -1.*t461*t1521;
  t1685 = t1623 + t1650;
  t1776 = Cos(var1[15]);
  t1616 = -1.*t319*t1556;
  t1686 = t1620*t1685;
  t1719 = t1616 + t1686;
  t54 = Sin(var1[15]);
  t1813 = t1620*t1556;
  t1831 = t319*t1685;
  t1852 = t1813 + t1831;
  t2445 = t476*t832;
  t2476 = t857*t1021*t1030;
  t2484 = t2445 + t2476;
  t2411 = t471*t645*t857;
  t2487 = t806*t2484;
  t2488 = t2411 + t2487;
  t2510 = t832*t857*t1021;
  t2593 = -1.*t476*t1030;
  t2612 = t2510 + t2593;
  t2617 = t1271*t2612;
  t2627 = -1.*t645*t806*t857;
  t2708 = t471*t2484;
  t2711 = t2627 + t2708;
  t2764 = -1.*t1381*t2711;
  t2772 = t2617 + t2764;
  t2491 = t461*t2488;
  t2789 = t1215*t2772;
  t2791 = t2491 + t2789;
  t2900 = t1215*t2488;
  t2901 = -1.*t461*t2772;
  t2903 = t2900 + t2901;
  t2860 = -1.*t319*t2791;
  t2945 = t1620*t2903;
  t2987 = t2860 + t2945;
  t3020 = t1620*t2791;
  t3048 = t319*t2903;
  t3055 = t3020 + t3048;
  t3153 = -1.*t471*t1021;
  t3157 = t645*t806*t1030;
  t3159 = t3153 + t3157;
  t3174 = t1271*t645*t832;
  t3184 = t806*t1021;
  t3188 = t471*t645*t1030;
  t3192 = t3184 + t3188;
  t3193 = -1.*t1381*t3192;
  t3194 = t3174 + t3193;
  t3171 = t461*t3159;
  t3209 = t1215*t3194;
  t3216 = t3171 + t3209;
  t3237 = t1215*t3159;
  t3240 = -1.*t461*t3194;
  t3253 = t3237 + t3240;
  t3230 = -1.*t319*t3216;
  t3254 = t1620*t3253;
  t3256 = t3230 + t3254;
  t3265 = t1620*t3216;
  t3275 = t319*t3253;
  t3281 = t3265 + t3275;
  t1767 = t54*t1719;
  t1854 = t1776*t1852;
  t2992 = t54*t2987;
  t3076 = t1776*t3055;
  t3260 = t54*t3256;
  t3285 = t1776*t3281;
  t3735 = -1.*t471;
  t3780 = 1. + t3735;
  t3931 = -1.*t1271;
  t3954 = 1. + t3931;
  t3345 = t1381*t1324;
  t3353 = t1271*t1491;
  t3367 = t3345 + t3353;
  t4139 = -1.*t1215;
  t4144 = 1. + t4139;
  t4300 = -1.*t1620;
  t4301 = 1. + t4300;
  t4447 = -1.*t1776;
  t4449 = 1. + t4447;
  t3517 = t1767 + t1854;
  t1921 = t1776*t1719;
  t2034 = -1.*t54*t1852;
  t2126 = t1921 + t2034;
  t3812 = -0.022225*t3780;
  t3846 = -0.086996*t806;
  t3875 = 0. + t3812 + t3846;
  t4005 = -0.31508*t3954;
  t4014 = 0.156996*t1381;
  t4047 = 0. + t4005 + t4014;
  t4085 = -0.086996*t3780;
  t4106 = 0.022225*t806;
  t4109 = 0. + t4085 + t4106;
  t4122 = -0.156996*t3954;
  t4123 = -0.31508*t1381;
  t4128 = 0. + t4122 + t4123;
  t4157 = -0.022225*t4144;
  t4169 = 0.38008*t461;
  t4204 = 0. + t4157 + t4169;
  t3372 = t1381*t2612;
  t3377 = t1271*t2711;
  t3417 = t3372 + t3377;
  t4267 = -0.38008*t4144;
  t4280 = -0.022225*t461;
  t4285 = 0. + t4267 + t4280;
  t4327 = -0.86008*t4301;
  t4348 = -0.022225*t319;
  t4367 = 0. + t4327 + t4348;
  t4377 = -0.022225*t4301;
  t4380 = 0.86008*t319;
  t4395 = 0. + t4377 + t4380;
  t4517 = -0.021147*t4449;
  t4572 = 1.34008*t54;
  t4576 = 0. + t4517 + t4572;
  t4583 = -1.34008*t4449;
  t4584 = -0.021147*t54;
  t4586 = 0. + t4583 + t4584;
  t3650 = t2992 + t3076;
  t3099 = t1776*t2987;
  t3112 = -1.*t54*t3055;
  t3137 = t3099 + t3112;
  t3439 = t645*t832*t1381;
  t3452 = t1271*t3192;
  t3488 = t3439 + t3452;
  t3697 = t3260 + t3285;
  t3295 = t1776*t3256;
  t3306 = -1.*t54*t3281;
  t3328 = t3295 + t3306;

  p_output1(0)=t1767 + t1854 + 0.000796*t2126;
  p_output1(1)=t2992 + t3076 + 0.000796*t3137;
  p_output1(2)=t3260 + t3285 + 0.000796*t3328;
  p_output1(3)=0.;
  p_output1(4)=t3367;
  p_output1(5)=t3417;
  p_output1(6)=t3488;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1719*t1776 + 0.000796*t3517 + t1852*t54;
  p_output1(9)=-1.*t1776*t2987 + 0.000796*t3650 + t3055*t54;
  p_output1(10)=-1.*t1776*t3256 + 0.000796*t3697 + t3281*t54;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043925*t2126 - 0.166996*t3367 - 1.250132*t3517 + t1324*t4047 + t1059*t4109 + t1491*t4128 + t1159*t4204 + t1521*t4285 + t1556*t4367 + t1685*t4395 + t1719*t4576 + t1852*t4586 + t3875*t476*t645 + var1(0);
  p_output1(13)=0. + 0.043925*t3137 - 0.166996*t3417 - 1.250132*t3650 + t2612*t4047 + t2484*t4109 + t2711*t4128 + t2488*t4204 + t2772*t4285 + t2791*t4367 + t2903*t4395 + t2987*t4576 + t3055*t4586 + t3875*t645*t857 + var1(1);
  p_output1(14)=0. + 0.043925*t3328 - 0.166996*t3488 - 1.250132*t3697 - 1.*t1021*t3875 + t3192*t4128 + t3159*t4204 + t3194*t4285 + t3216*t4367 + t3253*t4395 + t3256*t4576 + t3281*t4586 + t1030*t4109*t645 + t4047*t645*t832 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
