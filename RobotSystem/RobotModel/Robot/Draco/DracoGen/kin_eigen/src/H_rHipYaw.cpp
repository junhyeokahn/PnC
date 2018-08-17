/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:44 GMT-05:00
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
  double t1394;
  double t1330;
  double t2614;
  double t2891;
  double t2772;
  double t2780;
  double t2995;
  double t3109;
  double t2980;
  double t3266;
  double t3351;
  double t3751;
  double t3821;
  double t3855;
  double t4452;
  double t4488;
  double t4544;
  double t4643;
  double t4734;
  double t3907;
  double t3924;
  double t3960;
  double t2742;
  double t3364;
  double t3492;
  double t4743;
  double t4798;
  double t4819;
  double t4553;
  double t4555;
  double t4623;
  double t4860;
  double t4865;
  double t4866;
  double t4017;
  double t4050;
  double t4133;
  double t3634;
  double t3872;
  double t3892;
  double t4226;
  double t4339;
  double t4441;
  double t3898;
  double t3901;
  double t3906;
  t1394 = Cos(var1[3]);
  t1330 = Cos(var1[11]);
  t2614 = Cos(var1[4]);
  t2891 = Sin(var1[3]);
  t2772 = Sin(var1[11]);
  t2780 = Cos(var1[5]);
  t2995 = Sin(var1[4]);
  t3109 = Sin(var1[5]);
  t2980 = -1.*t2780*t2891;
  t3266 = t1394*t2995*t3109;
  t3351 = t2980 + t3266;
  t3751 = t1394*t2780;
  t3821 = t2891*t2995*t3109;
  t3855 = t3751 + t3821;
  t4452 = t1394*t2780*t2995;
  t4488 = t2891*t3109;
  t4544 = t4452 + t4488;
  t4643 = -1.*t1330;
  t4734 = 1. + t4643;
  t3907 = -1.*t1394*t2614*t2772;
  t3924 = t1330*t3351;
  t3960 = t3907 + t3924;
  t2742 = t1330*t1394*t2614;
  t3364 = t2772*t3351;
  t3492 = t2742 + t3364;
  t4743 = -0.022225*t4734;
  t4798 = -0.086996*t2772;
  t4819 = 0. + t4743 + t4798;
  t4553 = t2780*t2891*t2995;
  t4555 = -1.*t1394*t3109;
  t4623 = t4553 + t4555;
  t4860 = -0.086996*t4734;
  t4865 = 0.022225*t2772;
  t4866 = 0. + t4860 + t4865;
  t4017 = -1.*t2614*t2772*t2891;
  t4050 = t1330*t3855;
  t4133 = t4017 + t4050;
  t3634 = t1330*t2614*t2891;
  t3872 = t2772*t3855;
  t3892 = t3634 + t3872;
  t4226 = t2772*t2995;
  t4339 = t1330*t2614*t3109;
  t4441 = t4226 + t4339;
  t3898 = -1.*t1330*t2995;
  t3901 = t2614*t2772*t3109;
  t3906 = t3898 + t3901;

  p_output1(0)=t3492;
  p_output1(1)=t3892;
  p_output1(2)=t3906;
  p_output1(3)=0.;
  p_output1(4)=t3960;
  p_output1(5)=t4133;
  p_output1(6)=t4441;
  p_output1(7)=0.;
  p_output1(8)=t4544;
  p_output1(9)=t4623;
  p_output1(10)=t2614*t2780;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t3492 - 0.086996*t3960 - 0.29508*t4544 + t1394*t2614*t4819 + t3351*t4866 + var1(0);
  p_output1(13)=0. - 0.022225*t3892 - 0.086996*t4133 - 0.29508*t4623 + t2614*t2891*t4819 + t3855*t4866 + var1(1);
  p_output1(14)=0. - 0.29508*t2614*t2780 - 0.022225*t3906 - 0.086996*t4441 - 1.*t2995*t4819 + t2614*t3109*t4866 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipYaw(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
