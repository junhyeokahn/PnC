/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:41 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipYaw.h"

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
  double t119;
  double t250;
  double t361;
  double t1329;
  double t1324;
  double t1797;
  double t1934;
  double t2319;
  double t1491;
  double t1975;
  double t2196;
  double t2710;
  double t2816;
  double t3221;
  double t4422;
  double t4531;
  double t4621;
  double t4710;
  double t4714;
  double t4220;
  double t4236;
  double t4250;
  double t1210;
  double t2370;
  double t2385;
  double t4627;
  double t4635;
  double t4675;
  double t4715;
  double t4718;
  double t4719;
  double t4724;
  double t4734;
  double t4736;
  double t4252;
  double t4307;
  double t4337;
  double t2420;
  double t3586;
  double t3811;
  double t4344;
  double t4349;
  double t4400;
  double t4088;
  double t4093;
  double t4216;
  t119 = Cos(var1[3]);
  t250 = Cos(var1[4]);
  t361 = Cos(var1[6]);
  t1329 = Sin(var1[3]);
  t1324 = Cos(var1[5]);
  t1797 = Sin(var1[4]);
  t1934 = Sin(var1[5]);
  t2319 = Sin(var1[6]);
  t1491 = -1.*t1324*t1329;
  t1975 = t119*t1797*t1934;
  t2196 = t1491 + t1975;
  t2710 = t119*t1324;
  t2816 = t1329*t1797*t1934;
  t3221 = t2710 + t2816;
  t4422 = t119*t1324*t1797;
  t4531 = t1329*t1934;
  t4621 = t4422 + t4531;
  t4710 = -1.*t361;
  t4714 = 1. + t4710;
  t4220 = t361*t2196;
  t4236 = -1.*t119*t250*t2319;
  t4250 = t4220 + t4236;
  t1210 = t119*t250*t361;
  t2370 = t2196*t2319;
  t2385 = t1210 + t2370;
  t4627 = t1324*t1329*t1797;
  t4635 = -1.*t119*t1934;
  t4675 = t4627 + t4635;
  t4715 = 0.087004*t4714;
  t4718 = 0.022225*t2319;
  t4719 = 0. + t4715 + t4718;
  t4724 = -0.022225*t4714;
  t4734 = 0.087004*t2319;
  t4736 = 0. + t4724 + t4734;
  t4252 = t361*t3221;
  t4307 = -1.*t250*t1329*t2319;
  t4337 = t4252 + t4307;
  t2420 = t250*t361*t1329;
  t3586 = t3221*t2319;
  t3811 = t2420 + t3586;
  t4344 = t250*t361*t1934;
  t4349 = t1797*t2319;
  t4400 = t4344 + t4349;
  t4088 = -1.*t361*t1797;
  t4093 = t250*t1934*t2319;
  t4216 = t4088 + t4093;

  p_output1(0)=t2385;
  p_output1(1)=t3811;
  p_output1(2)=t4216;
  p_output1(3)=0.;
  p_output1(4)=t4250;
  p_output1(5)=t4337;
  p_output1(6)=t4400;
  p_output1(7)=0.;
  p_output1(8)=t4621;
  p_output1(9)=t4675;
  p_output1(10)=t1324*t250;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2385 + 0.087004*t4250 - 0.29508*t4621 + t2196*t4719 + t119*t250*t4736 + var1(0);
  p_output1(13)=0. - 0.022225*t3811 + 0.087004*t4337 - 0.29508*t4675 + t3221*t4719 + t1329*t250*t4736 + var1(1);
  p_output1(14)=0. - 0.29508*t1324*t250 - 0.022225*t4216 + 0.087004*t4400 + t1934*t250*t4719 - 1.*t1797*t4736 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
