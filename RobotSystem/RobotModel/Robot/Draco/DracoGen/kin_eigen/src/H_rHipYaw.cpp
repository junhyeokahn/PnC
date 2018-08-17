/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:53 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipYaw.h"

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
  double t2063;
  double t1045;
  double t2208;
  double t3131;
  double t2428;
  double t3037;
  double t3223;
  double t3506;
  double t3181;
  double t3576;
  double t3584;
  double t3859;
  double t3890;
  double t3943;
  double t4760;
  double t4779;
  double t4797;
  double t4949;
  double t4975;
  double t4210;
  double t4218;
  double t4318;
  double t2386;
  double t3621;
  double t3640;
  double t4990;
  double t5037;
  double t5043;
  double t4808;
  double t4887;
  double t4946;
  double t5074;
  double t5089;
  double t5092;
  double t4370;
  double t4577;
  double t4599;
  double t3706;
  double t4114;
  double t4153;
  double t4682;
  double t4696;
  double t4703;
  double t4175;
  double t4192;
  double t4205;
  t2063 = Cos(var1[3]);
  t1045 = Cos(var1[11]);
  t2208 = Cos(var1[4]);
  t3131 = Sin(var1[3]);
  t2428 = Sin(var1[11]);
  t3037 = Cos(var1[5]);
  t3223 = Sin(var1[4]);
  t3506 = Sin(var1[5]);
  t3181 = -1.*t3037*t3131;
  t3576 = t2063*t3223*t3506;
  t3584 = t3181 + t3576;
  t3859 = t2063*t3037;
  t3890 = t3131*t3223*t3506;
  t3943 = t3859 + t3890;
  t4760 = t2063*t3037*t3223;
  t4779 = t3131*t3506;
  t4797 = t4760 + t4779;
  t4949 = -1.*t1045;
  t4975 = 1. + t4949;
  t4210 = -1.*t2063*t2208*t2428;
  t4218 = t1045*t3584;
  t4318 = t4210 + t4218;
  t2386 = t1045*t2063*t2208;
  t3621 = t2428*t3584;
  t3640 = t2386 + t3621;
  t4990 = -0.022225*t4975;
  t5037 = -0.086996*t2428;
  t5043 = 0. + t4990 + t5037;
  t4808 = t3037*t3131*t3223;
  t4887 = -1.*t2063*t3506;
  t4946 = t4808 + t4887;
  t5074 = -0.086996*t4975;
  t5089 = 0.022225*t2428;
  t5092 = 0. + t5074 + t5089;
  t4370 = -1.*t2208*t2428*t3131;
  t4577 = t1045*t3943;
  t4599 = t4370 + t4577;
  t3706 = t1045*t2208*t3131;
  t4114 = t2428*t3943;
  t4153 = t3706 + t4114;
  t4682 = t2428*t3223;
  t4696 = t1045*t2208*t3506;
  t4703 = t4682 + t4696;
  t4175 = -1.*t1045*t3223;
  t4192 = t2208*t2428*t3506;
  t4205 = t4175 + t4192;

  p_output1(0)=t3640;
  p_output1(1)=t4153;
  p_output1(2)=t4205;
  p_output1(3)=0.;
  p_output1(4)=t4318;
  p_output1(5)=t4599;
  p_output1(6)=t4703;
  p_output1(7)=0.;
  p_output1(8)=t4797;
  p_output1(9)=t4946;
  p_output1(10)=t2208*t3037;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3640 - 0.086996*t4318 - 0.29508*t4797 + t2063*t2208*t5043 + t3584*t5092 + var1(0);
  p_output1(13)=0. - 0.022225*t4153 - 0.086996*t4599 - 0.29508*t4946 + t2208*t3131*t5043 + t3943*t5092 + var1(1);
  p_output1(14)=0. - 0.29508*t2208*t3037 - 0.022225*t4205 - 0.086996*t4703 - 1.*t3223*t5043 + t2208*t3506*t5092 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
