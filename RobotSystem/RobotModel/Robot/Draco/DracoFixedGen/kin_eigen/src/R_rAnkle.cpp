/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:15 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rAnkle.h"

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
  double t432;
  double t1055;
  double t1088;
  double t286;
  double t1277;
  double t183;
  double t2049;
  double t2084;
  double t2155;
  double t969;
  double t1594;
  double t1977;
  double t2588;
  double t2663;
  double t1987;
  double t2639;
  double t2642;
  double t116;
  double t2719;
  double t2756;
  double t2765;
  double t2890;
  double t2893;
  double t2894;
  double t2854;
  double t2867;
  double t2869;
  double t2884;
  double t2895;
  double t2897;
  double t2915;
  double t2917;
  double t2943;
  double t2971;
  double t2975;
  double t2999;
  double t3002;
  double t2977;
  double t2990;
  double t2996;
  double t3009;
  double t3011;
  double t3016;
  double t3018;
  double t3021;
  double t2647;
  double t2802;
  double t2910;
  double t2948;
  double t3013;
  double t3026;
  t432 = Cos(var1[7]);
  t1055 = Sin(var1[5]);
  t1088 = Sin(var1[6]);
  t286 = Cos(var1[5]);
  t1277 = Sin(var1[7]);
  t183 = Cos(var1[8]);
  t2049 = t432*t1055*t1088;
  t2084 = t286*t1277;
  t2155 = t2049 + t2084;
  t969 = t286*t432;
  t1594 = -1.*t1055*t1088*t1277;
  t1977 = t969 + t1594;
  t2588 = Sin(var1[8]);
  t2663 = Cos(var1[9]);
  t1987 = t183*t1977;
  t2639 = -1.*t2155*t2588;
  t2642 = t1987 + t2639;
  t116 = Sin(var1[9]);
  t2719 = t183*t2155;
  t2756 = t1977*t2588;
  t2765 = t2719 + t2756;
  t2890 = -1.*t286*t432*t1088;
  t2893 = t1055*t1277;
  t2894 = t2890 + t2893;
  t2854 = t432*t1055;
  t2867 = t286*t1088*t1277;
  t2869 = t2854 + t2867;
  t2884 = t183*t2869;
  t2895 = -1.*t2894*t2588;
  t2897 = t2884 + t2895;
  t2915 = t183*t2894;
  t2917 = t2869*t2588;
  t2943 = t2915 + t2917;
  t2971 = Cos(var1[6]);
  t2975 = 0. + t2971;
  t2999 = t2975*t432;
  t3002 = 0. + t2999;
  t2977 = -1.*t2975*t1277;
  t2990 = 0. + t2977;
  t2996 = t183*t2990;
  t3009 = -1.*t3002*t2588;
  t3011 = t2996 + t3009;
  t3016 = t3002*t183;
  t3018 = t2990*t2588;
  t3021 = t3016 + t3018;
  t2647 = t116*t2642;
  t2802 = t2663*t2765;
  t2910 = t116*t2897;
  t2948 = t2663*t2943;
  t3013 = t116*t3011;
  t3026 = t2663*t3021;

  p_output1(0)=t2647 + 0.000796*(t2642*t2663 - 1.*t116*t2765) + t2802;
  p_output1(1)=t2910 + 0.000796*(t2663*t2897 - 1.*t116*t2943) + t2948;
  p_output1(2)=t3013 + 0.000796*(t2663*t3011 - 1.*t116*t3021) + t3026;
  p_output1(3)=-1.*t1055*t2971;
  p_output1(4)=t286*t2971;
  p_output1(5)=0. + t1088;
  p_output1(6)=-1.*t2642*t2663 + t116*t2765 + 0.000796*(t2647 + t2802);
  p_output1(7)=-1.*t2663*t2897 + t116*t2943 + 0.000796*(t2910 + t2948);
  p_output1(8)=-1.*t2663*t3011 + t116*t3021 + 0.000796*(t3013 + t3026);
}


       
void R_rAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
