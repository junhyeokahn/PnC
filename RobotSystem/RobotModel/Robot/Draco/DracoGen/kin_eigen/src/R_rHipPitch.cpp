/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipPitch.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t3483;
  double t3823;
  double t4168;
  double t3956;
  double t4997;
  double t3513;
  double t3792;
  double t2879;
  double t3957;
  double t5048;
  double t5169;
  double t2375;
  double t5531;
  double t5571;
  double t5686;
  double t5898;
  double t5899;
  double t5901;
  double t5576;
  double t5620;
  double t5645;
  double t5719;
  double t5799;
  double t5863;
  double t5918;
  double t6015;
  double t6018;
  double t6029;
  double t6037;
  double t6039;
  double t6122;
  double t6132;
  double t6177;
  double t3514;
  double t5222;
  double t5339;
  double t5681;
  double t5872;
  double t5873;
  double t5896;
  double t5902;
  double t5914;
  double t6024;
  double t6041;
  double t6050;
  double t6061;
  double t6078;
  double t6093;
  double t6117;
  double t6192;
  double t6195;
  t3483 = Cos(var1[3]);
  t3823 = Cos(var1[5]);
  t4168 = Sin(var1[4]);
  t3956 = Sin(var1[3]);
  t4997 = Sin(var1[5]);
  t3513 = Cos(var1[4]);
  t3792 = Sin(var1[11]);
  t2879 = Cos(var1[11]);
  t3957 = -1.*t3823*t3956;
  t5048 = t3483*t4168*t4997;
  t5169 = t3957 + t5048;
  t2375 = Cos(var1[13]);
  t5531 = Sin(var1[13]);
  t5571 = Cos(var1[12]);
  t5686 = Sin(var1[12]);
  t5898 = t3483*t3823;
  t5899 = t3956*t4168*t4997;
  t5901 = t5898 + t5899;
  t5576 = t3483*t3823*t4168;
  t5620 = t3956*t4997;
  t5645 = t5576 + t5620;
  t5719 = -1.*t3483*t3513*t3792;
  t5799 = t2879*t5169;
  t5863 = t5719 + t5799;
  t5918 = t3823*t3956*t4168;
  t6015 = -1.*t3483*t4997;
  t6018 = t5918 + t6015;
  t6029 = -1.*t3513*t3792*t3956;
  t6037 = t2879*t5901;
  t6039 = t6029 + t6037;
  t6122 = t3792*t4168;
  t6132 = t2879*t3513*t4997;
  t6177 = t6122 + t6132;
  t3514 = t2879*t3483*t3513;
  t5222 = t3792*t5169;
  t5339 = t3514 + t5222;
  t5681 = t5571*t5645;
  t5872 = -1.*t5686*t5863;
  t5873 = t5681 + t5872;
  t5896 = t2879*t3513*t3956;
  t5902 = t3792*t5901;
  t5914 = t5896 + t5902;
  t6024 = t5571*t6018;
  t6041 = -1.*t5686*t6039;
  t6050 = t6024 + t6041;
  t6061 = -1.*t2879*t4168;
  t6078 = t3513*t3792*t4997;
  t6093 = t6061 + t6078;
  t6117 = t5571*t3513*t3823;
  t6192 = -1.*t5686*t6177;
  t6195 = t6117 + t6192;

  p_output1(0)=t2375*t5339 - 1.*t5531*t5873;
  p_output1(1)=t2375*t5914 - 1.*t5531*t6050;
  p_output1(2)=t2375*t6093 - 1.*t5531*t6195;
  p_output1(3)=t5645*t5686 + t5571*t5863;
  p_output1(4)=t5686*t6018 + t5571*t6039;
  p_output1(5)=t3513*t3823*t5686 + t5571*t6177;
  p_output1(6)=t5339*t5531 + t2375*t5873;
  p_output1(7)=t5531*t5914 + t2375*t6050;
  p_output1(8)=t5531*t6093 + t2375*t6195;
}


       
void R_rHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
