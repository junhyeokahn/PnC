/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:42 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lHipRoll.h"

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
  double t3279;
  double t4299;
  double t4400;
  double t4531;
  double t4627;
  double t4719;
  double t1643;
  double t1809;
  double t2019;
  double t3801;
  double t3980;
  double t4254;
  double t4295;
  double t4838;
  double t4839;
  double t4846;
  double t4874;
  double t4808;
  double t4809;
  double t4837;
  double t4913;
  double t4914;
  double t4915;
  double t4621;
  double t4680;
  double t4681;
  double t4721;
  double t4724;
  double t4734;
  double t5012;
  double t5013;
  double t5018;
  double t4857;
  double t4898;
  double t4903;
  double t4916;
  double t4917;
  double t4930;
  double t5095;
  double t5098;
  double t5103;
  double t5119;
  double t5129;
  double t5130;
  double t5301;
  double t5304;
  double t5306;
  t3279 = Cos(var1[3]);
  t4299 = Cos(var1[6]);
  t4400 = -1.*t4299;
  t4531 = 1. + t4400;
  t4627 = Sin(var1[6]);
  t4719 = Cos(var1[4]);
  t1643 = Cos(var1[5]);
  t1809 = Sin(var1[3]);
  t2019 = -1.*t1643*t1809;
  t3801 = Sin(var1[4]);
  t3980 = Sin(var1[5]);
  t4254 = t3279*t3801*t3980;
  t4295 = t2019 + t4254;
  t4838 = Cos(var1[7]);
  t4839 = -1.*t4838;
  t4846 = 1. + t4839;
  t4874 = Sin(var1[7]);
  t4808 = t4299*t4295;
  t4809 = -1.*t3279*t4719*t4627;
  t4837 = t4808 + t4809;
  t4913 = t3279*t1643*t3801;
  t4914 = t1809*t3980;
  t4915 = t4913 + t4914;
  t4621 = 0.087004*t4531;
  t4680 = 0.022225*t4627;
  t4681 = 0. + t4621 + t4680;
  t4721 = -0.022225*t4531;
  t4724 = 0.087004*t4627;
  t4734 = 0. + t4721 + t4724;
  t5012 = t3279*t1643;
  t5013 = t1809*t3801*t3980;
  t5018 = t5012 + t5013;
  t4857 = 0.157004*t4846;
  t4898 = -0.31508*t4874;
  t4903 = 0. + t4857 + t4898;
  t4916 = -0.31508*t4846;
  t4917 = -0.157004*t4874;
  t4930 = 0. + t4916 + t4917;
  t5095 = t4299*t5018;
  t5098 = -1.*t4719*t1809*t4627;
  t5103 = t5095 + t5098;
  t5119 = t1643*t1809*t3801;
  t5129 = -1.*t3279*t3980;
  t5130 = t5119 + t5129;
  t5301 = t4719*t4299*t3980;
  t5304 = t3801*t4627;
  t5306 = t5301 + t5304;

  p_output1(0)=0. + t4295*t4681 - 0.022225*(t4295*t4627 + t3279*t4299*t4719) + t3279*t4719*t4734 + t4837*t4903 - 0.31508*(-1.*t4837*t4874 + t4838*t4915) + 0.157004*(t4837*t4838 + t4874*t4915) + t4915*t4930 + var1(0);
  p_output1(1)=0. + t1809*t4719*t4734 + t4681*t5018 - 0.022225*(t1809*t4299*t4719 + t4627*t5018) + t4903*t5103 + t4930*t5130 - 0.31508*(-1.*t4874*t5103 + t4838*t5130) + 0.157004*(t4838*t5103 + t4874*t5130) + var1(1);
  p_output1(2)=0. + t3980*t4681*t4719 - 0.022225*(-1.*t3801*t4299 + t3980*t4627*t4719) - 1.*t3801*t4734 + t1643*t4719*t4930 + t4903*t5306 + 0.157004*(t1643*t4719*t4874 + t4838*t5306) - 0.31508*(t1643*t4719*t4838 - 1.*t4874*t5306) + var1(2);
}


       
void p_lHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
