/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:25 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootBack.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t2127;
  double t2457;
  double t2812;
  double t1544;
  double t2819;
  double t1258;
  double t3828;
  double t3867;
  double t3873;
  double t2455;
  double t3062;
  double t3482;
  double t3896;
  double t4199;
  double t3560;
  double t3984;
  double t4085;
  double t903;
  double t4286;
  double t4298;
  double t4364;
  double t4658;
  double t4660;
  double t4668;
  double t4607;
  double t4628;
  double t4629;
  double t4655;
  double t4670;
  double t4673;
  double t4695;
  double t4696;
  double t4710;
  double t4738;
  double t4739;
  double t4750;
  double t4757;
  double t4745;
  double t4747;
  double t4749;
  double t4758;
  double t4767;
  double t4777;
  double t4785;
  double t4791;
  double t4106;
  double t4377;
  double t4692;
  double t4711;
  double t4775;
  double t4800;
  t2127 = Cos(var1[7]);
  t2457 = Sin(var1[5]);
  t2812 = Sin(var1[6]);
  t1544 = Cos(var1[5]);
  t2819 = Sin(var1[7]);
  t1258 = Cos(var1[8]);
  t3828 = t2127*t2457*t2812;
  t3867 = t1544*t2819;
  t3873 = t3828 + t3867;
  t2455 = t1544*t2127;
  t3062 = -1.*t2457*t2812*t2819;
  t3482 = t2455 + t3062;
  t3896 = Sin(var1[8]);
  t4199 = Cos(var1[9]);
  t3560 = t1258*t3482;
  t3984 = -1.*t3873*t3896;
  t4085 = t3560 + t3984;
  t903 = Sin(var1[9]);
  t4286 = t1258*t3873;
  t4298 = t3482*t3896;
  t4364 = t4286 + t4298;
  t4658 = -1.*t1544*t2127*t2812;
  t4660 = t2457*t2819;
  t4668 = t4658 + t4660;
  t4607 = t2127*t2457;
  t4628 = t1544*t2812*t2819;
  t4629 = t4607 + t4628;
  t4655 = t1258*t4629;
  t4670 = -1.*t4668*t3896;
  t4673 = t4655 + t4670;
  t4695 = t1258*t4668;
  t4696 = t4629*t3896;
  t4710 = t4695 + t4696;
  t4738 = Cos(var1[6]);
  t4739 = 0. + t4738;
  t4750 = t4739*t2127;
  t4757 = 0. + t4750;
  t4745 = -1.*t4739*t2819;
  t4747 = 0. + t4745;
  t4749 = t1258*t4747;
  t4758 = -1.*t4757*t3896;
  t4767 = t4749 + t4758;
  t4777 = t4757*t1258;
  t4785 = t4747*t3896;
  t4791 = t4777 + t4785;
  t4106 = t903*t4085;
  t4377 = t4199*t4364;
  t4692 = t903*t4673;
  t4711 = t4199*t4710;
  t4775 = t903*t4767;
  t4800 = t4199*t4791;

  p_output1(0)=t4106 + t4377 + 0.000796*(t4085*t4199 - 1.*t4364*t903);
  p_output1(1)=t4692 + t4711 + 0.000796*(t4199*t4673 - 1.*t4710*t903);
  p_output1(2)=t4775 + t4800 + 0.000796*(t4199*t4767 - 1.*t4791*t903);
  p_output1(3)=-1.*t2457*t4738;
  p_output1(4)=t1544*t4738;
  p_output1(5)=0. + t2812;
  p_output1(6)=-1.*t4085*t4199 + 0.000796*(t4106 + t4377) + t4364*t903;
  p_output1(7)=-1.*t4199*t4673 + 0.000796*(t4692 + t4711) + t4710*t903;
  p_output1(8)=-1.*t4199*t4767 + 0.000796*(t4775 + t4800) + t4791*t903;
}


       
void R_RightFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
