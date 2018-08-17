/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootBottom.h"

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
  double t503;
  double t771;
  double t427;
  double t669;
  double t816;
  double t1349;
  double t1195;
  double t1228;
  double t1255;
  double t1256;
  double t1533;
  double t192;
  double t1719;
  double t1803;
  double t1819;
  double t366;
  double t728;
  double t893;
  double t907;
  double t911;
  double t1292;
  double t1554;
  double t1593;
  double t1600;
  double t1657;
  double t1683;
  double t1821;
  double t1938;
  double t1692;
  double t1847;
  double t1888;
  double t36;
  double t1965;
  double t1967;
  double t1971;
  double t2201;
  double t1892;
  double t2006;
  double t2047;
  double t34;
  double t2208;
  double t2234;
  double t2267;
  double t2822;
  double t2824;
  double t2831;
  double t3049;
  double t3060;
  double t3154;
  double t2723;
  double t2754;
  double t2785;
  double t2789;
  double t2833;
  double t2886;
  double t2913;
  double t2945;
  double t2947;
  double t2953;
  double t3185;
  double t3193;
  double t3203;
  double t3209;
  double t3226;
  double t3200;
  double t3264;
  double t3301;
  double t3327;
  double t3340;
  double t3344;
  double t3668;
  double t3697;
  double t3707;
  double t3594;
  double t3613;
  double t3627;
  double t3630;
  double t3631;
  double t3653;
  double t3657;
  double t3714;
  double t3722;
  double t3753;
  double t3765;
  double t3770;
  double t3751;
  double t3773;
  double t3832;
  double t3872;
  double t3893;
  double t3898;
  double t2084;
  double t2352;
  double t3303;
  double t3454;
  double t3864;
  double t3903;
  double t4527;
  double t4553;
  double t4782;
  double t4807;
  double t4016;
  double t4021;
  double t4028;
  double t5026;
  double t5027;
  double t5207;
  double t5211;
  double t5348;
  double t5359;
  double t4310;
  double t2376;
  double t2390;
  double t2457;
  double t4566;
  double t4578;
  double t4608;
  double t4680;
  double t4698;
  double t4710;
  double t4810;
  double t4884;
  double t4897;
  double t4918;
  double t4934;
  double t4986;
  double t4050;
  double t4058;
  double t4116;
  double t5042;
  double t5063;
  double t5077;
  double t5110;
  double t5123;
  double t5171;
  double t5216;
  double t5219;
  double t5230;
  double t5259;
  double t5289;
  double t5299;
  double t5374;
  double t5384;
  double t5393;
  double t5401;
  double t5404;
  double t5418;
  double t4379;
  double t3464;
  double t3465;
  double t3509;
  double t4133;
  double t4164;
  double t4178;
  double t4491;
  double t3930;
  double t3931;
  double t3958;
  t503 = Cos(var1[5]);
  t771 = Sin(var1[3]);
  t427 = Cos(var1[3]);
  t669 = Sin(var1[4]);
  t816 = Sin(var1[5]);
  t1349 = Cos(var1[4]);
  t1195 = Cos(var1[6]);
  t1228 = -1.*t503*t771;
  t1255 = t427*t669*t816;
  t1256 = t1228 + t1255;
  t1533 = Sin(var1[6]);
  t192 = Cos(var1[8]);
  t1719 = t427*t1349*t1195;
  t1803 = t1256*t1533;
  t1819 = t1719 + t1803;
  t366 = Cos(var1[7]);
  t728 = t427*t503*t669;
  t893 = t771*t816;
  t907 = t728 + t893;
  t911 = t366*t907;
  t1292 = t1195*t1256;
  t1554 = -1.*t427*t1349*t1533;
  t1593 = t1292 + t1554;
  t1600 = Sin(var1[7]);
  t1657 = -1.*t1593*t1600;
  t1683 = t911 + t1657;
  t1821 = Sin(var1[8]);
  t1938 = Cos(var1[9]);
  t1692 = t192*t1683;
  t1847 = t1819*t1821;
  t1888 = t1692 + t1847;
  t36 = Sin(var1[9]);
  t1965 = t192*t1819;
  t1967 = -1.*t1683*t1821;
  t1971 = t1965 + t1967;
  t2201 = Cos(var1[10]);
  t1892 = -1.*t36*t1888;
  t2006 = t1938*t1971;
  t2047 = t1892 + t2006;
  t34 = Sin(var1[10]);
  t2208 = t1938*t1888;
  t2234 = t36*t1971;
  t2267 = t2208 + t2234;
  t2822 = t427*t503;
  t2824 = t771*t669*t816;
  t2831 = t2822 + t2824;
  t3049 = t1349*t1195*t771;
  t3060 = t2831*t1533;
  t3154 = t3049 + t3060;
  t2723 = t503*t771*t669;
  t2754 = -1.*t427*t816;
  t2785 = t2723 + t2754;
  t2789 = t366*t2785;
  t2833 = t1195*t2831;
  t2886 = -1.*t1349*t771*t1533;
  t2913 = t2833 + t2886;
  t2945 = -1.*t2913*t1600;
  t2947 = t2789 + t2945;
  t2953 = t192*t2947;
  t3185 = t3154*t1821;
  t3193 = t2953 + t3185;
  t3203 = t192*t3154;
  t3209 = -1.*t2947*t1821;
  t3226 = t3203 + t3209;
  t3200 = -1.*t36*t3193;
  t3264 = t1938*t3226;
  t3301 = t3200 + t3264;
  t3327 = t1938*t3193;
  t3340 = t36*t3226;
  t3344 = t3327 + t3340;
  t3668 = -1.*t1195*t669;
  t3697 = t1349*t816*t1533;
  t3707 = t3668 + t3697;
  t3594 = t1349*t503*t366;
  t3613 = t1349*t1195*t816;
  t3627 = t669*t1533;
  t3630 = t3613 + t3627;
  t3631 = -1.*t3630*t1600;
  t3653 = t3594 + t3631;
  t3657 = t192*t3653;
  t3714 = t3707*t1821;
  t3722 = t3657 + t3714;
  t3753 = t192*t3707;
  t3765 = -1.*t3653*t1821;
  t3770 = t3753 + t3765;
  t3751 = -1.*t36*t3722;
  t3773 = t1938*t3770;
  t3832 = t3751 + t3773;
  t3872 = t1938*t3722;
  t3893 = t36*t3770;
  t3898 = t3872 + t3893;
  t2084 = t34*t2047;
  t2352 = t2201*t2267;
  t3303 = t34*t3301;
  t3454 = t2201*t3344;
  t3864 = t34*t3832;
  t3903 = t2201*t3898;
  t4527 = -1.*t1195;
  t4553 = 1. + t4527;
  t4782 = -1.*t366;
  t4807 = 1. + t4782;
  t4016 = t366*t1593;
  t4021 = t907*t1600;
  t4028 = t4016 + t4021;
  t5026 = -1.*t192;
  t5027 = 1. + t5026;
  t5207 = -1.*t1938;
  t5211 = 1. + t5207;
  t5348 = -1.*t2201;
  t5359 = 1. + t5348;
  t4310 = t2084 + t2352;
  t2376 = t2201*t2047;
  t2390 = -1.*t34*t2267;
  t2457 = t2376 + t2390;
  t4566 = 0.087004*t4553;
  t4578 = 0.022225*t1533;
  t4608 = 0. + t4566 + t4578;
  t4680 = -0.022225*t4553;
  t4698 = 0.087004*t1533;
  t4710 = 0. + t4680 + t4698;
  t4810 = 0.157004*t4807;
  t4884 = -0.31508*t1600;
  t4897 = 0. + t4810 + t4884;
  t4918 = -0.31508*t4807;
  t4934 = -0.157004*t1600;
  t4986 = 0. + t4918 + t4934;
  t4050 = t366*t2913;
  t4058 = t2785*t1600;
  t4116 = t4050 + t4058;
  t5042 = -0.38008*t5027;
  t5063 = -0.022225*t1821;
  t5077 = 0. + t5042 + t5063;
  t5110 = -0.022225*t5027;
  t5123 = 0.38008*t1821;
  t5171 = 0. + t5110 + t5123;
  t5216 = -0.86008*t5211;
  t5219 = -0.022225*t36;
  t5230 = 0. + t5216 + t5219;
  t5259 = -0.022225*t5211;
  t5289 = 0.86008*t36;
  t5299 = 0. + t5259 + t5289;
  t5374 = -0.021147*t5359;
  t5384 = 1.34008*t34;
  t5393 = 0. + t5374 + t5384;
  t5401 = -1.34008*t5359;
  t5404 = -0.021147*t34;
  t5418 = 0. + t5401 + t5404;
  t4379 = t3303 + t3454;
  t3464 = t2201*t3301;
  t3465 = -1.*t34*t3344;
  t3509 = t3464 + t3465;
  t4133 = t366*t3630;
  t4164 = t1349*t503*t1600;
  t4178 = t4133 + t4164;
  t4491 = t3864 + t3903;
  t3930 = t2201*t3832;
  t3931 = -1.*t34*t3898;
  t3958 = t3930 + t3931;

  p_output1(0)=t2084 + t2352 + 0.000796*t2457;
  p_output1(1)=t3303 + t3454 + 0.000796*t3509;
  p_output1(2)=t3864 + t3903 + 0.000796*t3958;
  p_output1(3)=0.;
  p_output1(4)=t4028;
  p_output1(5)=t4116;
  p_output1(6)=t4178;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2047*t2201 + t2267*t34 + 0.000796*t4310;
  p_output1(9)=-1.*t2201*t3301 + t3344*t34 + 0.000796*t4379;
  p_output1(10)=-1.*t2201*t3832 + t34*t3898 + 0.000796*t4491;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043865*t2457 + 0.167004*t4028 - 1.325132*t4310 + t1256*t4608 + t1349*t427*t4710 + t1593*t4897 + t1683*t5077 + t1819*t5171 + t1888*t5230 + t1971*t5299 + t2047*t5393 + t2267*t5418 + t4986*t907 + var1(0);
  p_output1(13)=0. + 0.043865*t3509 + 0.167004*t4116 - 1.325132*t4379 + t2831*t4608 + t2913*t4897 + t2785*t4986 + t2947*t5077 + t3154*t5171 + t3193*t5230 + t3226*t5299 + t3301*t5393 + t3344*t5418 + t1349*t4710*t771 + var1(1);
  p_output1(14)=0. + 0.043865*t3958 + 0.167004*t4178 - 1.325132*t4491 + t3630*t4897 + t1349*t4986*t503 + t3653*t5077 + t3707*t5171 + t3722*t5230 + t3770*t5299 + t3832*t5393 + t3898*t5418 - 1.*t4710*t669 + t1349*t4608*t816 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootBottom(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
