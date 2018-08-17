/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:31 GMT-05:00
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
  double t2902;
  double t947;
  double t987;
  double t1120;
  double t3293;
  double t4291;
  double t4353;
  double t4359;
  double t4742;
  double t3713;
  double t3718;
  double t3816;
  double t4959;
  double t4735;
  double t4760;
  double t4763;
  double t4969;
  double t4970;
  double t4992;
  double t5105;
  double t5108;
  double t5109;
  double t5092;
  double t5093;
  double t5098;
  double t5323;
  double t5344;
  double t5350;
  double t5120;
  double t5121;
  double t5124;
  double t5434;
  double t5447;
  double t5448;
  double t5451;
  double t5452;
  double t5495;
  double t5498;
  double t5507;
  double t5058;
  t2902 = Cos(var1[3]);
  t947 = Cos(var1[5]);
  t987 = Sin(var1[3]);
  t1120 = Sin(var1[4]);
  t3293 = Sin(var1[5]);
  t4291 = Cos(var1[6]);
  t4353 = -1.*t4291;
  t4359 = 1. + t4353;
  t4742 = Sin(var1[6]);
  t3713 = -1.*t2902*t947;
  t3718 = -1.*t987*t1120*t3293;
  t3816 = t3713 + t3718;
  t4959 = Cos(var1[4]);
  t4735 = 0.087004*t4359;
  t4760 = 0.022225*t4742;
  t4763 = 0. + t4735 + t4760;
  t4969 = -0.022225*t4359;
  t4970 = 0.087004*t4742;
  t4992 = 0. + t4969 + t4970;
  t5105 = -1.*t947*t987;
  t5108 = t2902*t1120*t3293;
  t5109 = t5105 + t5108;
  t5092 = t2902*t947*t1120;
  t5093 = t987*t3293;
  t5098 = t5092 + t5093;
  t5323 = t947*t987*t1120;
  t5344 = -1.*t2902*t3293;
  t5350 = t5323 + t5344;
  t5120 = t4291*t5109;
  t5121 = -1.*t2902*t4959*t4742;
  t5124 = t5120 + t5121;
  t5434 = 0.087004*t4291;
  t5447 = -0.022225*t4742;
  t5448 = t5434 + t5447;
  t5451 = 0.022225*t4291;
  t5452 = t5451 + t4970;
  t5495 = t2902*t947;
  t5498 = t987*t1120*t3293;
  t5507 = t5495 + t5498;
  t5058 = -1.*t4959*t4291*t987;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t3816*t4763 - 0.022225*(t3816*t4742 + t5058) - 1.*t4959*t4992*t987 + 0.087004*(t3816*t4291 + t4742*t4959*t987) - 0.29508*(t2902*t3293 - 1.*t1120*t947*t987);
  p_output1(10)=t2902*t4959*t4992 - 0.29508*t5098 + t4763*t5109 - 0.022225*(t2902*t4291*t4959 + t4742*t5109) + 0.087004*t5124;
  p_output1(11)=0;
  p_output1(12)=t2902*t3293*t4763*t4959 + 0.087004*(t1120*t2902*t4742 + t2902*t3293*t4291*t4959) - 0.022225*(-1.*t1120*t2902*t4291 + t2902*t3293*t4742*t4959) - 1.*t1120*t2902*t4992 - 0.29508*t2902*t4959*t947;
  p_output1(13)=t3293*t4763*t4959*t987 - 1.*t1120*t4992*t987 - 0.29508*t4959*t947*t987 + 0.087004*(t1120*t4742*t987 + t3293*t4291*t4959*t987) - 0.022225*(-1.*t1120*t4291*t987 + t3293*t4742*t4959*t987);
  p_output1(14)=-1.*t1120*t3293*t4763 - 0.022225*(-1.*t1120*t3293*t4742 - 1.*t4291*t4959) + 0.087004*(-1.*t1120*t3293*t4291 + t4742*t4959) - 1.*t4959*t4992 + 0.29508*t1120*t947;
  p_output1(15)=0.087004*t4291*t5098 - 0.022225*t4742*t5098 + t4763*t5098 - 0.29508*(-1.*t1120*t2902*t3293 + t947*t987);
  p_output1(16)=-0.29508*t3816 + 0.087004*t4291*t5350 - 0.022225*t4742*t5350 + t4763*t5350;
  p_output1(17)=0.29508*t3293*t4959 + 0.087004*t4291*t4959*t947 - 0.022225*t4742*t4959*t947 + t4763*t4959*t947;
  p_output1(18)=0.087004*(-1.*t2902*t4291*t4959 - 1.*t4742*t5109) - 0.022225*t5124 + t2902*t4959*t5448 + t5109*t5452;
  p_output1(19)=t5452*t5507 + 0.087004*(t5058 - 1.*t4742*t5507) + t4959*t5448*t987 - 0.022225*(t4291*t5507 - 1.*t4742*t4959*t987);
  p_output1(20)=-0.022225*(t1120*t4742 + t3293*t4291*t4959) + 0.087004*(t1120*t4291 - 1.*t3293*t4742*t4959) - 1.*t1120*t5448 + t3293*t4959*t5452;
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
