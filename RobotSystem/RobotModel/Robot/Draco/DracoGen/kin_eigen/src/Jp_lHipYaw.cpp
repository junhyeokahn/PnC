/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:15 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipYaw.h"

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
  double t1605;
  double t310;
  double t311;
  double t1573;
  double t1753;
  double t3687;
  double t4024;
  double t4036;
  double t4096;
  double t3236;
  double t3270;
  double t3610;
  double t4188;
  double t4073;
  double t4106;
  double t4128;
  double t4194;
  double t4215;
  double t4225;
  double t4522;
  double t4525;
  double t4526;
  double t4471;
  double t4490;
  double t4511;
  double t4906;
  double t4909;
  double t4915;
  double t4531;
  double t4532;
  double t4539;
  double t4973;
  double t4980;
  double t4996;
  double t5007;
  double t5019;
  double t5119;
  double t5130;
  double t5131;
  double t4300;
  t1605 = Cos(var1[3]);
  t310 = Cos(var1[5]);
  t311 = Sin(var1[3]);
  t1573 = Sin(var1[4]);
  t1753 = Sin(var1[5]);
  t3687 = Cos(var1[6]);
  t4024 = -1.*t3687;
  t4036 = 1. + t4024;
  t4096 = Sin(var1[6]);
  t3236 = -1.*t1605*t310;
  t3270 = -1.*t311*t1573*t1753;
  t3610 = t3236 + t3270;
  t4188 = Cos(var1[4]);
  t4073 = 0.087*t4036;
  t4106 = 0.0222*t4096;
  t4128 = 0. + t4073 + t4106;
  t4194 = -0.0222*t4036;
  t4215 = 0.087*t4096;
  t4225 = 0. + t4194 + t4215;
  t4522 = -1.*t310*t311;
  t4525 = t1605*t1573*t1753;
  t4526 = t4522 + t4525;
  t4471 = t1605*t310*t1573;
  t4490 = t311*t1753;
  t4511 = t4471 + t4490;
  t4906 = t310*t311*t1573;
  t4909 = -1.*t1605*t1753;
  t4915 = t4906 + t4909;
  t4531 = t3687*t4526;
  t4532 = -1.*t1605*t4188*t4096;
  t4539 = t4531 + t4532;
  t4973 = 0.087*t3687;
  t4980 = -0.0222*t4096;
  t4996 = t4973 + t4980;
  t5007 = 0.0222*t3687;
  t5019 = t5007 + t4215;
  t5119 = t1605*t310;
  t5130 = t311*t1573*t1753;
  t5131 = t5119 + t5130;
  t4300 = -1.*t4188*t3687*t311;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-0.2951*(t1605*t1753 - 1.*t1573*t310*t311) + t3610*t4128 + 0.087*(t3610*t3687 + t311*t4096*t4188) - 1.*t311*t4188*t4225 - 0.0222*(t3610*t4096 + t4300);
  p_output1(10)=t1605*t4188*t4225 - 0.2951*t4511 + t4128*t4526 - 0.0222*(t1605*t3687*t4188 + t4096*t4526) + 0.087*t4539;
  p_output1(11)=0;
  p_output1(12)=-0.2951*t1605*t310*t4188 + t1605*t1753*t4128*t4188 + 0.087*(t1573*t1605*t4096 + t1605*t1753*t3687*t4188) - 0.0222*(-1.*t1573*t1605*t3687 + t1605*t1753*t4096*t4188) - 1.*t1573*t1605*t4225;
  p_output1(13)=-0.2951*t310*t311*t4188 + t1753*t311*t4128*t4188 + 0.087*(t1573*t311*t4096 + t1753*t311*t3687*t4188) - 0.0222*(-1.*t1573*t311*t3687 + t1753*t311*t4096*t4188) - 1.*t1573*t311*t4225;
  p_output1(14)=0.2951*t1573*t310 - 1.*t1573*t1753*t4128 - 0.0222*(-1.*t1573*t1753*t4096 - 1.*t3687*t4188) + 0.087*(-1.*t1573*t1753*t3687 + t4096*t4188) - 1.*t4188*t4225;
  p_output1(15)=-0.2951*(-1.*t1573*t1605*t1753 + t310*t311) + 0.087*t3687*t4511 - 0.0222*t4096*t4511 + t4128*t4511;
  p_output1(16)=-0.2951*t3610 + 0.087*t3687*t4915 - 0.0222*t4096*t4915 + t4128*t4915;
  p_output1(17)=0.2951*t1753*t4188 + 0.087*t310*t3687*t4188 - 0.0222*t310*t4096*t4188 + t310*t4128*t4188;
  p_output1(18)=0.087*(-1.*t1605*t3687*t4188 - 1.*t4096*t4526) - 0.0222*t4539 + t1605*t4188*t4996 + t4526*t5019;
  p_output1(19)=t311*t4188*t4996 + t5019*t5131 - 0.0222*(-1.*t311*t4096*t4188 + t3687*t5131) + 0.087*(t4300 - 1.*t4096*t5131);
  p_output1(20)=-0.0222*(t1573*t4096 + t1753*t3687*t4188) + 0.087*(t1573*t3687 - 1.*t1753*t4096*t4188) - 1.*t1573*t4996 + t1753*t4188*t5019;
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


       
void Jp_lHipYaw(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
