/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:22 GMT-05:00
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
  double t2125;
  double t828;
  double t911;
  double t960;
  double t1634;
  double t2168;
  double t2147;
  double t2152;
  double t2169;
  double t231;
  double t2193;
  double t2194;
  double t2196;
  double t1560;
  double t1644;
  double t2067;
  double t2186;
  double t2188;
  double t2190;
  double t2742;
  double t2766;
  double t2850;
  double t2495;
  double t2591;
  double t2618;
  double t4344;
  double t4365;
  double t4383;
  double t2867;
  double t2877;
  double t2893;
  double t4442;
  double t4444;
  double t4449;
  double t4451;
  double t4460;
  double t4578;
  double t4602;
  double t4658;
  double t2213;
  t2125 = Sin(var1[3]);
  t828 = Cos(var1[11]);
  t911 = -1.*t828;
  t960 = 1. + t911;
  t1634 = Sin(var1[11]);
  t2168 = Cos(var1[3]);
  t2147 = Cos(var1[5]);
  t2152 = Sin(var1[4]);
  t2169 = Sin(var1[5]);
  t231 = Cos(var1[4]);
  t2193 = -1.*t2168*t2147;
  t2194 = -1.*t2125*t2152*t2169;
  t2196 = t2193 + t2194;
  t1560 = -0.0222*t960;
  t1644 = -0.087*t1634;
  t2067 = 0. + t1560 + t1644;
  t2186 = -0.087*t960;
  t2188 = 0.0222*t1634;
  t2190 = 0. + t2186 + t2188;
  t2742 = -1.*t2147*t2125;
  t2766 = t2168*t2152*t2169;
  t2850 = t2742 + t2766;
  t2495 = t2168*t2147*t2152;
  t2591 = t2125*t2169;
  t2618 = t2495 + t2591;
  t4344 = t2147*t2125*t2152;
  t4365 = -1.*t2168*t2169;
  t4383 = t4344 + t4365;
  t2867 = -1.*t2168*t231*t1634;
  t2877 = t828*t2850;
  t2893 = t2867 + t2877;
  t4442 = -0.087*t828;
  t4444 = -0.0222*t1634;
  t4449 = t4442 + t4444;
  t4451 = 0.0222*t828;
  t4460 = t4451 + t1644;
  t4578 = t2168*t2147;
  t4602 = t2125*t2152*t2169;
  t4658 = t4578 + t4602;
  t2213 = -1.*t828*t231*t2125;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-0.2951*(-1.*t2125*t2147*t2152 + t2168*t2169) + t2190*t2196 - 0.0222*(t1634*t2196 + t2213) - 1.*t2067*t2125*t231 - 0.087*(t1634*t2125*t231 + t2196*t828);
  p_output1(10)=t2067*t2168*t231 - 0.2951*t2618 + t2190*t2850 - 0.087*t2893 - 0.0222*(t1634*t2850 + t2168*t231*t828);
  p_output1(11)=0;
  p_output1(12)=-1.*t2067*t2152*t2168 - 0.2951*t2147*t2168*t231 + t2168*t2169*t2190*t231 - 0.0222*(t1634*t2168*t2169*t231 - 1.*t2152*t2168*t828) - 0.087*(t1634*t2152*t2168 + t2168*t2169*t231*t828);
  p_output1(13)=-1.*t2067*t2125*t2152 - 0.2951*t2125*t2147*t231 + t2125*t2169*t2190*t231 - 0.0222*(t1634*t2125*t2169*t231 - 1.*t2125*t2152*t828) - 0.087*(t1634*t2125*t2152 + t2125*t2169*t231*t828);
  p_output1(14)=0.2951*t2147*t2152 - 1.*t2152*t2169*t2190 - 1.*t2067*t231 - 0.087*(t1634*t231 - 1.*t2152*t2169*t828) - 0.0222*(-1.*t1634*t2152*t2169 - 1.*t231*t828);
  p_output1(15)=-0.2951*(t2125*t2147 - 1.*t2152*t2168*t2169) - 0.0222*t1634*t2618 + t2190*t2618 - 0.087*t2618*t828;
  p_output1(16)=-0.2951*t2196 - 0.0222*t1634*t4383 + t2190*t4383 - 0.087*t4383*t828;
  p_output1(17)=-0.0222*t1634*t2147*t231 + 0.2951*t2169*t231 + t2147*t2190*t231 - 0.087*t2147*t231*t828;
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
  p_output1(33)=-0.0222*t2893 + t2168*t231*t4449 + t2850*t4460 - 0.087*(-1.*t1634*t2850 - 1.*t2168*t231*t828);
  p_output1(34)=t2125*t231*t4449 + t4460*t4658 - 0.087*(t2213 - 1.*t1634*t4658) - 0.0222*(-1.*t1634*t2125*t231 + t4658*t828);
  p_output1(35)=-1.*t2152*t4449 + t2169*t231*t4460 - 0.087*(-1.*t1634*t2169*t231 + t2152*t828) - 0.0222*(t1634*t2152 + t2169*t231*t828);
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
