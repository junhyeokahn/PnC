/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:55 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBottom.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t1256;
  double t1692;
  double t944;
  double t1278;
  double t1700;
  double t2376;
  double t1885;
  double t1911;
  double t1954;
  double t2178;
  double t2457;
  double t728;
  double t2831;
  double t2953;
  double t3049;
  double t893;
  double t1675;
  double t1719;
  double t1722;
  double t1847;
  double t2352;
  double t2511;
  double t2669;
  double t2766;
  double t2777;
  double t2789;
  double t3085;
  double t3226;
  double t2822;
  double t3154;
  double t3185;
  double t595;
  double t3264;
  double t3303;
  double t3340;
  double t3456;
  double t3203;
  double t3386;
  double t3454;
  double t306;
  double t3464;
  double t3509;
  double t3519;
  double t3939;
  double t3958;
  double t3960;
  double t4116;
  double t4133;
  double t4164;
  double t3861;
  double t3872;
  double t3893;
  double t3930;
  double t4001;
  double t4016;
  double t4021;
  double t4028;
  double t4050;
  double t4058;
  double t4284;
  double t4310;
  double t4344;
  double t4376;
  double t4395;
  double t4311;
  double t4402;
  double t4429;
  double t4491;
  double t4514;
  double t4546;
  double t4918;
  double t4928;
  double t4934;
  double t4698;
  double t4777;
  double t4810;
  double t4828;
  double t4884;
  double t4897;
  double t4910;
  double t4957;
  double t4974;
  double t4984;
  double t4986;
  double t4990;
  double t4975;
  double t5023;
  double t5042;
  double t5077;
  double t5089;
  double t5110;
  double t3455;
  double t3627;
  double t4483;
  double t4566;
  double t5063;
  double t5123;
  t1256 = Cos(var1[5]);
  t1692 = Sin(var1[3]);
  t944 = Cos(var1[3]);
  t1278 = Sin(var1[4]);
  t1700 = Sin(var1[5]);
  t2376 = Cos(var1[4]);
  t1885 = Cos(var1[6]);
  t1911 = -1.*t1256*t1692;
  t1954 = t944*t1278*t1700;
  t2178 = t1911 + t1954;
  t2457 = Sin(var1[6]);
  t728 = Cos(var1[8]);
  t2831 = t944*t2376*t1885;
  t2953 = t2178*t2457;
  t3049 = t2831 + t2953;
  t893 = Cos(var1[7]);
  t1675 = t944*t1256*t1278;
  t1719 = t1692*t1700;
  t1722 = t1675 + t1719;
  t1847 = t893*t1722;
  t2352 = t1885*t2178;
  t2511 = -1.*t944*t2376*t2457;
  t2669 = t2352 + t2511;
  t2766 = Sin(var1[7]);
  t2777 = -1.*t2669*t2766;
  t2789 = t1847 + t2777;
  t3085 = Sin(var1[8]);
  t3226 = Cos(var1[9]);
  t2822 = t728*t2789;
  t3154 = t3049*t3085;
  t3185 = t2822 + t3154;
  t595 = Sin(var1[9]);
  t3264 = t728*t3049;
  t3303 = -1.*t2789*t3085;
  t3340 = t3264 + t3303;
  t3456 = Cos(var1[10]);
  t3203 = -1.*t595*t3185;
  t3386 = t3226*t3340;
  t3454 = t3203 + t3386;
  t306 = Sin(var1[10]);
  t3464 = t3226*t3185;
  t3509 = t595*t3340;
  t3519 = t3464 + t3509;
  t3939 = t944*t1256;
  t3958 = t1692*t1278*t1700;
  t3960 = t3939 + t3958;
  t4116 = t2376*t1885*t1692;
  t4133 = t3960*t2457;
  t4164 = t4116 + t4133;
  t3861 = t1256*t1692*t1278;
  t3872 = -1.*t944*t1700;
  t3893 = t3861 + t3872;
  t3930 = t893*t3893;
  t4001 = t1885*t3960;
  t4016 = -1.*t2376*t1692*t2457;
  t4021 = t4001 + t4016;
  t4028 = -1.*t4021*t2766;
  t4050 = t3930 + t4028;
  t4058 = t728*t4050;
  t4284 = t4164*t3085;
  t4310 = t4058 + t4284;
  t4344 = t728*t4164;
  t4376 = -1.*t4050*t3085;
  t4395 = t4344 + t4376;
  t4311 = -1.*t595*t4310;
  t4402 = t3226*t4395;
  t4429 = t4311 + t4402;
  t4491 = t3226*t4310;
  t4514 = t595*t4395;
  t4546 = t4491 + t4514;
  t4918 = -1.*t1885*t1278;
  t4928 = t2376*t1700*t2457;
  t4934 = t4918 + t4928;
  t4698 = t2376*t1256*t893;
  t4777 = t2376*t1885*t1700;
  t4810 = t1278*t2457;
  t4828 = t4777 + t4810;
  t4884 = -1.*t4828*t2766;
  t4897 = t4698 + t4884;
  t4910 = t728*t4897;
  t4957 = t4934*t3085;
  t4974 = t4910 + t4957;
  t4984 = t728*t4934;
  t4986 = -1.*t4897*t3085;
  t4990 = t4984 + t4986;
  t4975 = -1.*t595*t4974;
  t5023 = t3226*t4990;
  t5042 = t4975 + t5023;
  t5077 = t3226*t4974;
  t5089 = t595*t4990;
  t5110 = t5077 + t5089;
  t3455 = t306*t3454;
  t3627 = t3456*t3519;
  t4483 = t306*t4429;
  t4566 = t3456*t4546;
  t5063 = t306*t5042;
  t5123 = t3456*t5110;

  p_output1(0)=t3455 + 0.000796*(t3454*t3456 - 1.*t306*t3519) + t3627;
  p_output1(1)=t4483 + 0.000796*(t3456*t4429 - 1.*t306*t4546) + t4566;
  p_output1(2)=t5063 + 0.000796*(t3456*t5042 - 1.*t306*t5110) + t5123;
  p_output1(3)=t1722*t2766 + t2669*t893;
  p_output1(4)=t2766*t3893 + t4021*t893;
  p_output1(5)=t1256*t2376*t2766 + t4828*t893;
  p_output1(6)=-1.*t3454*t3456 + t306*t3519 + 0.000796*(t3455 + t3627);
  p_output1(7)=-1.*t3456*t4429 + t306*t4546 + 0.000796*(t4483 + t4566);
  p_output1(8)=-1.*t3456*t5042 + t306*t5110 + 0.000796*(t5063 + t5123);
}


       
void R_LeftFootBottom(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
