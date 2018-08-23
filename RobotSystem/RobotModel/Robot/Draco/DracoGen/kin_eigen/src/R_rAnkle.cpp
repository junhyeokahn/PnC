/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:29 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rAnkle.h"

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
  double t1490;
  double t1629;
  double t1774;
  double t1631;
  double t1786;
  double t1498;
  double t1623;
  double t1466;
  double t1701;
  double t1847;
  double t1867;
  double t2028;
  double t1597;
  double t1914;
  double t1946;
  double t974;
  double t2371;
  double t2443;
  double t2479;
  double t2527;
  double t2619;
  double t2694;
  double t2784;
  double t2805;
  double t2870;
  double t2871;
  double t2872;
  double t3049;
  double t2013;
  double t2911;
  double t2950;
  double t710;
  double t3094;
  double t3100;
  double t3134;
  double t3434;
  double t3037;
  double t3194;
  double t3262;
  double t702;
  double t3440;
  double t3606;
  double t3722;
  double t4179;
  double t4191;
  double t4193;
  double t4178;
  double t4204;
  double t4212;
  double t4385;
  double t4409;
  double t4438;
  double t4472;
  double t4498;
  double t4516;
  double t4535;
  double t4649;
  double t4659;
  double t4372;
  double t4712;
  double t4723;
  double t4865;
  double t4988;
  double t5059;
  double t4731;
  double t5068;
  double t5248;
  double t5294;
  double t5357;
  double t5362;
  double t5537;
  double t5542;
  double t5590;
  double t5598;
  double t5679;
  double t5710;
  double t5727;
  double t5914;
  double t5940;
  double t5596;
  double t5993;
  double t6003;
  double t6010;
  double t6015;
  double t6016;
  double t6006;
  double t6038;
  double t6061;
  double t6098;
  double t6103;
  double t6105;
  double t3419;
  double t3781;
  double t5285;
  double t5380;
  double t6070;
  double t6109;
  t1490 = Cos(var1[3]);
  t1629 = Cos(var1[5]);
  t1774 = Sin(var1[4]);
  t1631 = Sin(var1[3]);
  t1786 = Sin(var1[5]);
  t1498 = Cos(var1[4]);
  t1623 = Sin(var1[11]);
  t1466 = Cos(var1[11]);
  t1701 = -1.*t1629*t1631;
  t1847 = t1490*t1774*t1786;
  t1867 = t1701 + t1847;
  t2028 = Cos(var1[13]);
  t1597 = t1466*t1490*t1498;
  t1914 = t1623*t1867;
  t1946 = t1597 + t1914;
  t974 = Sin(var1[13]);
  t2371 = Cos(var1[12]);
  t2443 = t1490*t1629*t1774;
  t2479 = t1631*t1786;
  t2527 = t2443 + t2479;
  t2619 = t2371*t2527;
  t2694 = Sin(var1[12]);
  t2784 = -1.*t1490*t1498*t1623;
  t2805 = t1466*t1867;
  t2870 = t2784 + t2805;
  t2871 = -1.*t2694*t2870;
  t2872 = t2619 + t2871;
  t3049 = Cos(var1[14]);
  t2013 = t974*t1946;
  t2911 = t2028*t2872;
  t2950 = t2013 + t2911;
  t710 = Sin(var1[14]);
  t3094 = t2028*t1946;
  t3100 = -1.*t974*t2872;
  t3134 = t3094 + t3100;
  t3434 = Cos(var1[15]);
  t3037 = -1.*t710*t2950;
  t3194 = t3049*t3134;
  t3262 = t3037 + t3194;
  t702 = Sin(var1[15]);
  t3440 = t3049*t2950;
  t3606 = t710*t3134;
  t3722 = t3440 + t3606;
  t4179 = t1490*t1629;
  t4191 = t1631*t1774*t1786;
  t4193 = t4179 + t4191;
  t4178 = t1466*t1498*t1631;
  t4204 = t1623*t4193;
  t4212 = t4178 + t4204;
  t4385 = t1629*t1631*t1774;
  t4409 = -1.*t1490*t1786;
  t4438 = t4385 + t4409;
  t4472 = t2371*t4438;
  t4498 = -1.*t1498*t1623*t1631;
  t4516 = t1466*t4193;
  t4535 = t4498 + t4516;
  t4649 = -1.*t2694*t4535;
  t4659 = t4472 + t4649;
  t4372 = t974*t4212;
  t4712 = t2028*t4659;
  t4723 = t4372 + t4712;
  t4865 = t2028*t4212;
  t4988 = -1.*t974*t4659;
  t5059 = t4865 + t4988;
  t4731 = -1.*t710*t4723;
  t5068 = t3049*t5059;
  t5248 = t4731 + t5068;
  t5294 = t3049*t4723;
  t5357 = t710*t5059;
  t5362 = t5294 + t5357;
  t5537 = -1.*t1466*t1774;
  t5542 = t1498*t1623*t1786;
  t5590 = t5537 + t5542;
  t5598 = t2371*t1498*t1629;
  t5679 = t1623*t1774;
  t5710 = t1466*t1498*t1786;
  t5727 = t5679 + t5710;
  t5914 = -1.*t2694*t5727;
  t5940 = t5598 + t5914;
  t5596 = t974*t5590;
  t5993 = t2028*t5940;
  t6003 = t5596 + t5993;
  t6010 = t2028*t5590;
  t6015 = -1.*t974*t5940;
  t6016 = t6010 + t6015;
  t6006 = -1.*t710*t6003;
  t6038 = t3049*t6016;
  t6061 = t6006 + t6038;
  t6098 = t3049*t6003;
  t6103 = t710*t6016;
  t6105 = t6098 + t6103;
  t3419 = t702*t3262;
  t3781 = t3434*t3722;
  t5285 = t702*t5248;
  t5380 = t3434*t5362;
  t6070 = t702*t6061;
  t6109 = t3434*t6105;

  p_output1(0)=t3419 + t3781 + 0.000796*(t3262*t3434 - 1.*t3722*t702);
  p_output1(1)=t5285 + t5380 + 0.000796*(t3434*t5248 - 1.*t5362*t702);
  p_output1(2)=t6070 + t6109 + 0.000796*(t3434*t6061 - 1.*t6105*t702);
  p_output1(3)=t2527*t2694 + t2371*t2870;
  p_output1(4)=t2694*t4438 + t2371*t4535;
  p_output1(5)=t1498*t1629*t2694 + t2371*t5727;
  p_output1(6)=-1.*t3262*t3434 + 0.000796*(t3419 + t3781) + t3722*t702;
  p_output1(7)=-1.*t3434*t5248 + 0.000796*(t5285 + t5380) + t5362*t702;
  p_output1(8)=-1.*t3434*t6061 + 0.000796*(t6070 + t6109) + t6105*t702;
}


       
void R_rAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
