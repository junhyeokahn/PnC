/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:16 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootFront.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1977;
  double t2038;
  double t2521;
  double t969;
  double t2756;
  double t2820;
  double t2822;
  double t2867;
  double t3013;
  double t3018;
  double t3026;
  double t3034;
  double t2990;
  double t2996;
  double t2999;
  double t3055;
  double t3063;
  double t3065;
  double t3089;
  double t3090;
  double t3092;
  double t3095;
  double t3086;
  double t3087;
  double t3088;
  double t3108;
  double t3110;
  double t3116;
  double t1227;
  double t1237;
  double t2434;
  double t2513;
  double t2518;
  double t2598;
  double t2599;
  double t2844;
  double t2869;
  double t2884;
  double t2910;
  double t2915;
  double t2917;
  double t3032;
  double t3035;
  double t3038;
  double t3072;
  double t3076;
  double t3082;
  double t3190;
  double t3193;
  double t3194;
  double t3197;
  double t3198;
  double t3199;
  double t3093;
  double t3101;
  double t3104;
  double t3117;
  double t3118;
  double t3123;
  double t3202;
  double t3204;
  double t3210;
  double t3219;
  double t3225;
  double t3228;
  double t3268;
  double t3270;
  double t3273;
  double t3280;
  double t3281;
  double t3290;
  double t3293;
  double t3300;
  double t3302;
  double t3304;
  double t3311;
  t1977 = Sin(var1[0]);
  t2038 = Cos(var1[1]);
  t2521 = Sin(var1[1]);
  t969 = Cos(var1[0]);
  t2756 = Cos(var1[2]);
  t2820 = -1.*t2756;
  t2822 = 1. + t2820;
  t2867 = Sin(var1[2]);
  t3013 = Cos(var1[3]);
  t3018 = -1.*t3013;
  t3026 = 1. + t3018;
  t3034 = Sin(var1[3]);
  t2990 = t2756*t1977*t2521;
  t2996 = t969*t2867;
  t2999 = t2990 + t2996;
  t3055 = t969*t2756;
  t3063 = -1.*t1977*t2521*t2867;
  t3065 = t3055 + t3063;
  t3089 = Cos(var1[4]);
  t3090 = -1.*t3089;
  t3092 = 1. + t3090;
  t3095 = Sin(var1[4]);
  t3086 = t3013*t2999;
  t3087 = t3065*t3034;
  t3088 = t3086 + t3087;
  t3108 = t3013*t3065;
  t3110 = -1.*t2999*t3034;
  t3116 = t3108 + t3110;
  t1227 = -1.*t969;
  t1237 = 1. + t1227;
  t2434 = -1.*t2038;
  t2513 = 1. + t2434;
  t2518 = 0.331012*t2513;
  t2598 = -0.90524*t2521;
  t2599 = 0. + t2518 + t2598;
  t2844 = -0.97024*t2822;
  t2869 = -0.066675*t2867;
  t2884 = 0. + t2844 + t2869;
  t2910 = -0.066675*t2822;
  t2915 = 0.97024*t2867;
  t2917 = 0. + t2910 + t2915;
  t3032 = -1.45024*t3026;
  t3035 = -0.066675*t3034;
  t3038 = 0. + t3032 + t3035;
  t3072 = -0.066675*t3026;
  t3076 = 1.45024*t3034;
  t3082 = 0. + t3072 + t3076;
  t3190 = -1.*t969*t2756*t2521;
  t3193 = t1977*t2867;
  t3194 = t3190 + t3193;
  t3197 = t2756*t1977;
  t3198 = t969*t2521*t2867;
  t3199 = t3197 + t3198;
  t3093 = -1.93024*t3092;
  t3101 = -0.065597*t3095;
  t3104 = 0. + t3093 + t3101;
  t3117 = -0.065597*t3092;
  t3118 = 1.93024*t3095;
  t3123 = 0. + t3117 + t3118;
  t3202 = t3013*t3194;
  t3204 = t3199*t3034;
  t3210 = t3202 + t3204;
  t3219 = t3013*t3199;
  t3225 = -1.*t3194*t3034;
  t3228 = t3219 + t3225;
  t3268 = 0. + t2038;
  t3270 = t3268*t2756;
  t3273 = 0. + t3270;
  t3280 = -1.*t3268*t2867;
  t3281 = 0. + t3280;
  t3290 = t3273*t3013;
  t3293 = t3281*t3034;
  t3300 = t3290 + t3293;
  t3302 = t3013*t3281;
  t3304 = -1.*t3273*t3034;
  t3311 = t3302 + t3304;

  p_output1(0)=0. - 0.066675*t1237 + 0.261012*t1977 - 0.341012*t1977*t2038 - 1.*t1977*t2599 + t1977*t2521*t2884 + t2999*t3038 + t3065*t3082 + t3088*t3104 - 0.000525*(-1.*t3088*t3095 + t3089*t3116) - 1.840292*(t3088*t3089 + t3095*t3116) + t3116*t3123 + t2917*t969;
  p_output1(1)=0. + 0.261012*t1237 + 0.066675*t1977 + t1977*t2917 + t3038*t3194 + t3082*t3199 + t3104*t3210 + t3123*t3228 - 0.000525*(-1.*t3095*t3210 + t3089*t3228) - 1.840292*(t3089*t3210 + t3095*t3228) + 0.341012*t2038*t969 + t2599*t969 - 1.*t2521*t2884*t969;
  p_output1(2)=0. - 0.90524*t2513 - 0.331012*t2521 + 0.341012*(0. + t2521) + t2884*t3268 + t3038*t3273 + t3082*t3281 + t3104*t3300 + t3123*t3311 - 0.000525*(-1.*t3095*t3300 + t3089*t3311) - 1.840292*(t3089*t3300 + t3095*t3311);
}


       
void p_LeftFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
