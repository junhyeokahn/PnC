/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:44 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipYaw.h"

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
  double t2504;
  double t1234;
  double t1342;
  double t1384;
  double t1833;
  double t3112;
  double t2923;
  double t3038;
  double t3123;
  double t654;
  double t3672;
  double t3674;
  double t3751;
  double t1505;
  double t1961;
  double t2349;
  double t3634;
  double t3663;
  double t3666;
  double t4089;
  double t4133;
  double t4244;
  double t3924;
  double t3960;
  double t4050;
  double t5391;
  double t5395;
  double t5400;
  double t4400;
  double t4403;
  double t4441;
  double t5546;
  double t5553;
  double t5564;
  double t5573;
  double t5576;
  double t5709;
  double t5714;
  double t5725;
  double t3891;
  t2504 = Sin(var1[3]);
  t1234 = Cos(var1[11]);
  t1342 = -1.*t1234;
  t1384 = 1. + t1342;
  t1833 = Sin(var1[11]);
  t3112 = Cos(var1[3]);
  t2923 = Cos(var1[5]);
  t3038 = Sin(var1[4]);
  t3123 = Sin(var1[5]);
  t654 = Cos(var1[4]);
  t3672 = -1.*t3112*t2923;
  t3674 = -1.*t2504*t3038*t3123;
  t3751 = t3672 + t3674;
  t1505 = -0.022225*t1384;
  t1961 = -0.086996*t1833;
  t2349 = 0. + t1505 + t1961;
  t3634 = -0.086996*t1384;
  t3663 = 0.022225*t1833;
  t3666 = 0. + t3634 + t3663;
  t4089 = -1.*t2923*t2504;
  t4133 = t3112*t3038*t3123;
  t4244 = t4089 + t4133;
  t3924 = t3112*t2923*t3038;
  t3960 = t2504*t3123;
  t4050 = t3924 + t3960;
  t5391 = t2923*t2504*t3038;
  t5395 = -1.*t3112*t3123;
  t5400 = t5391 + t5395;
  t4400 = -1.*t3112*t654*t1833;
  t4403 = t1234*t4244;
  t4441 = t4400 + t4403;
  t5546 = -0.086996*t1234;
  t5553 = -0.022225*t1833;
  t5564 = t5546 + t5553;
  t5573 = 0.022225*t1234;
  t5576 = t5573 + t1961;
  t5709 = t3112*t2923;
  t5714 = t2504*t3038*t3123;
  t5725 = t5709 + t5714;
  t3891 = -1.*t1234*t654*t2504;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-0.29508*(-1.*t2504*t2923*t3038 + t3112*t3123) + t3666*t3751 - 0.022225*(t1833*t3751 + t3891) - 1.*t2349*t2504*t654 - 0.086996*(t1234*t3751 + t1833*t2504*t654);
  p_output1(10)=-0.29508*t4050 + t3666*t4244 - 0.086996*t4441 + t2349*t3112*t654 - 0.022225*(t1833*t4244 + t1234*t3112*t654);
  p_output1(11)=0;
  p_output1(12)=-1.*t2349*t3038*t3112 - 0.29508*t2923*t3112*t654 + t3112*t3123*t3666*t654 - 0.086996*(t1833*t3038*t3112 + t1234*t3112*t3123*t654) - 0.022225*(-1.*t1234*t3038*t3112 + t1833*t3112*t3123*t654);
  p_output1(13)=-1.*t2349*t2504*t3038 - 0.29508*t2504*t2923*t654 + t2504*t3123*t3666*t654 - 0.086996*(t1833*t2504*t3038 + t1234*t2504*t3123*t654) - 0.022225*(-1.*t1234*t2504*t3038 + t1833*t2504*t3123*t654);
  p_output1(14)=0.29508*t2923*t3038 - 1.*t3038*t3123*t3666 - 1.*t2349*t654 - 0.022225*(-1.*t1833*t3038*t3123 - 1.*t1234*t654) - 0.086996*(-1.*t1234*t3038*t3123 + t1833*t654);
  p_output1(15)=-0.29508*(t2504*t2923 - 1.*t3038*t3112*t3123) - 0.086996*t1234*t4050 - 0.022225*t1833*t4050 + t3666*t4050;
  p_output1(16)=-0.29508*t3751 - 0.086996*t1234*t5400 - 0.022225*t1833*t5400 + t3666*t5400;
  p_output1(17)=-0.086996*t1234*t2923*t654 - 0.022225*t1833*t2923*t654 + 0.29508*t3123*t654 + t2923*t3666*t654;
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
  p_output1(33)=-0.022225*t4441 + t4244*t5576 + t3112*t5564*t654 - 0.086996*(-1.*t1833*t4244 - 1.*t1234*t3112*t654);
  p_output1(34)=t5576*t5725 - 0.086996*(t3891 - 1.*t1833*t5725) + t2504*t5564*t654 - 0.022225*(t1234*t5725 - 1.*t1833*t2504*t654);
  p_output1(35)=-1.*t3038*t5564 + t3123*t5576*t654 - 0.022225*(t1833*t3038 + t1234*t3123*t654) - 0.086996*(t1234*t3038 - 1.*t1833*t3123*t654);
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


       
void Jp_rHipYaw(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
