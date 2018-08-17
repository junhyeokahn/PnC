/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:53 GMT-05:00
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
  double t2035;
  double t455;
  double t659;
  double t810;
  double t1201;
  double t3321;
  double t2836;
  double t3082;
  double t3494;
  double t265;
  double t3768;
  double t3799;
  double t3859;
  double t1045;
  double t1229;
  double t1977;
  double t3640;
  double t3706;
  double t3745;
  double t4567;
  double t4577;
  double t4589;
  double t4318;
  double t4370;
  double t4521;
  double t5676;
  double t5681;
  double t5686;
  double t4657;
  double t4664;
  double t4682;
  double t5842;
  double t5862;
  double t5873;
  double t5924;
  double t5931;
  double t6069;
  double t6112;
  double t6124;
  double t4159;
  t2035 = Sin(var1[3]);
  t455 = Cos(var1[11]);
  t659 = -1.*t455;
  t810 = 1. + t659;
  t1201 = Sin(var1[11]);
  t3321 = Cos(var1[3]);
  t2836 = Cos(var1[5]);
  t3082 = Sin(var1[4]);
  t3494 = Sin(var1[5]);
  t265 = Cos(var1[4]);
  t3768 = -1.*t3321*t2836;
  t3799 = -1.*t2035*t3082*t3494;
  t3859 = t3768 + t3799;
  t1045 = -0.022225*t810;
  t1229 = -0.086996*t1201;
  t1977 = 0. + t1045 + t1229;
  t3640 = -0.086996*t810;
  t3706 = 0.022225*t1201;
  t3745 = 0. + t3640 + t3706;
  t4567 = -1.*t2836*t2035;
  t4577 = t3321*t3082*t3494;
  t4589 = t4567 + t4577;
  t4318 = t3321*t2836*t3082;
  t4370 = t2035*t3494;
  t4521 = t4318 + t4370;
  t5676 = t2836*t2035*t3082;
  t5681 = -1.*t3321*t3494;
  t5686 = t5676 + t5681;
  t4657 = -1.*t3321*t265*t1201;
  t4664 = t455*t4589;
  t4682 = t4657 + t4664;
  t5842 = -0.086996*t455;
  t5862 = -0.022225*t1201;
  t5873 = t5842 + t5862;
  t5924 = 0.022225*t455;
  t5931 = t5924 + t1229;
  t6069 = t3321*t2836;
  t6112 = t2035*t3082*t3494;
  t6124 = t6069 + t6112;
  t4159 = -1.*t455*t265*t2035;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t1977*t2035*t265 - 0.29508*(-1.*t2035*t2836*t3082 + t3321*t3494) + t3745*t3859 - 0.022225*(t1201*t3859 + t4159) - 0.086996*(t1201*t2035*t265 + t3859*t455);
  p_output1(10)=t1977*t265*t3321 - 0.29508*t4521 + t3745*t4589 - 0.022225*(t265*t3321*t455 + t1201*t4589) - 0.086996*t4682;
  p_output1(11)=0;
  p_output1(12)=-0.29508*t265*t2836*t3321 - 1.*t1977*t3082*t3321 + t265*t3321*t3494*t3745 - 0.022225*(t1201*t265*t3321*t3494 - 1.*t3082*t3321*t455) - 0.086996*(t1201*t3082*t3321 + t265*t3321*t3494*t455);
  p_output1(13)=-0.29508*t2035*t265*t2836 - 1.*t1977*t2035*t3082 + t2035*t265*t3494*t3745 - 0.022225*(t1201*t2035*t265*t3494 - 1.*t2035*t3082*t455) - 0.086996*(t1201*t2035*t3082 + t2035*t265*t3494*t455);
  p_output1(14)=-1.*t1977*t265 + 0.29508*t2836*t3082 - 1.*t3082*t3494*t3745 - 0.022225*(-1.*t1201*t3082*t3494 - 1.*t265*t455) - 0.086996*(t1201*t265 - 1.*t3082*t3494*t455);
  p_output1(15)=-0.29508*(t2035*t2836 - 1.*t3082*t3321*t3494) - 0.022225*t1201*t4521 + t3745*t4521 - 0.086996*t4521*t455;
  p_output1(16)=-0.29508*t3859 - 0.022225*t1201*t5686 + t3745*t5686 - 0.086996*t455*t5686;
  p_output1(17)=-0.022225*t1201*t265*t2836 + 0.29508*t265*t3494 + t265*t2836*t3745 - 0.086996*t265*t2836*t455;
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
  p_output1(33)=-0.086996*(-1.*t265*t3321*t455 - 1.*t1201*t4589) - 0.022225*t4682 + t265*t3321*t5873 + t4589*t5931;
  p_output1(34)=t2035*t265*t5873 + t5931*t6124 - 0.086996*(t4159 - 1.*t1201*t6124) - 0.022225*(-1.*t1201*t2035*t265 + t455*t6124);
  p_output1(35)=-0.086996*(-1.*t1201*t265*t3494 + t3082*t455) - 0.022225*(t1201*t3082 + t265*t3494*t455) - 1.*t3082*t5873 + t265*t3494*t5931;
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
