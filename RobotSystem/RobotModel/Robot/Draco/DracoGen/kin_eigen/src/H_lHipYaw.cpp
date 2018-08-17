/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:32 GMT-05:00
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
  double t1408;
  double t1968;
  double t2868;
  double t3118;
  double t3097;
  double t3360;
  double t3368;
  double t3405;
  double t3268;
  double t3395;
  double t3401;
  double t3763;
  double t3816;
  double t4735;
  double t5100;
  double t5108;
  double t5114;
  double t5141;
  double t5144;
  double t5010;
  double t5018;
  double t5028;
  double t2903;
  double t3490;
  double t3668;
  double t5116;
  double t5124;
  double t5128;
  double t5147;
  double t5152;
  double t5155;
  double t5161;
  double t5163;
  double t5164;
  double t5036;
  double t5065;
  double t5078;
  double t3707;
  double t4758;
  double t4874;
  double t5080;
  double t5093;
  double t5098;
  double t4969;
  double t4970;
  double t5000;
  t1408 = Cos(var1[3]);
  t1968 = Cos(var1[4]);
  t2868 = Cos(var1[6]);
  t3118 = Sin(var1[3]);
  t3097 = Cos(var1[5]);
  t3360 = Sin(var1[4]);
  t3368 = Sin(var1[5]);
  t3405 = Sin(var1[6]);
  t3268 = -1.*t3097*t3118;
  t3395 = t1408*t3360*t3368;
  t3401 = t3268 + t3395;
  t3763 = t1408*t3097;
  t3816 = t3118*t3360*t3368;
  t4735 = t3763 + t3816;
  t5100 = t1408*t3097*t3360;
  t5108 = t3118*t3368;
  t5114 = t5100 + t5108;
  t5141 = -1.*t2868;
  t5144 = 1. + t5141;
  t5010 = t2868*t3401;
  t5018 = -1.*t1408*t1968*t3405;
  t5028 = t5010 + t5018;
  t2903 = t1408*t1968*t2868;
  t3490 = t3401*t3405;
  t3668 = t2903 + t3490;
  t5116 = t3097*t3118*t3360;
  t5124 = -1.*t1408*t3368;
  t5128 = t5116 + t5124;
  t5147 = 0.087004*t5144;
  t5152 = 0.022225*t3405;
  t5155 = 0. + t5147 + t5152;
  t5161 = -0.022225*t5144;
  t5163 = 0.087004*t3405;
  t5164 = 0. + t5161 + t5163;
  t5036 = t2868*t4735;
  t5065 = -1.*t1968*t3118*t3405;
  t5078 = t5036 + t5065;
  t3707 = t1968*t2868*t3118;
  t4758 = t4735*t3405;
  t4874 = t3707 + t4758;
  t5080 = t1968*t2868*t3368;
  t5093 = t3360*t3405;
  t5098 = t5080 + t5093;
  t4969 = -1.*t2868*t3360;
  t4970 = t1968*t3368*t3405;
  t5000 = t4969 + t4970;

  p_output1(0)=t3668;
  p_output1(1)=t4874;
  p_output1(2)=t5000;
  p_output1(3)=0.;
  p_output1(4)=t5028;
  p_output1(5)=t5078;
  p_output1(6)=t5098;
  p_output1(7)=0.;
  p_output1(8)=t5114;
  p_output1(9)=t5128;
  p_output1(10)=t1968*t3097;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3668 + 0.087004*t5028 - 0.29508*t5114 + t3401*t5155 + t1408*t1968*t5164 + var1(0);
  p_output1(13)=0. - 0.022225*t4874 + 0.087004*t5078 - 0.29508*t5128 + t4735*t5155 + t1968*t3118*t5164 + var1(1);
  p_output1(14)=0. - 0.29508*t1968*t3097 - 0.022225*t5000 + 0.087004*t5098 + t1968*t3368*t5155 - 1.*t3360*t5164 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
