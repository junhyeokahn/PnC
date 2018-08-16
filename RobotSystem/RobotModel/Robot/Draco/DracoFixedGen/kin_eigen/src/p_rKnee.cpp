/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:11 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rKnee.h"

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
  double t2082;
  double t2122;
  double t2161;
  double t12;
  double t2168;
  double t2170;
  double t2171;
  double t2181;
  double t2212;
  double t2213;
  double t2214;
  double t2216;
  double t2222;
  double t2225;
  double t2230;
  double t2207;
  double t2208;
  double t2210;
  double t938;
  double t992;
  double t2140;
  double t2146;
  double t2158;
  double t2164;
  double t2165;
  double t2172;
  double t2185;
  double t2186;
  double t2190;
  double t2204;
  double t2205;
  double t2215;
  double t2219;
  double t2220;
  double t2232;
  double t2234;
  double t2236;
  double t2294;
  double t2295;
  double t2296;
  double t2280;
  double t2282;
  double t2283;
  double t2335;
  double t2346;
  double t2347;
  double t2337;
  double t2339;
  t2082 = Sin(var1[5]);
  t2122 = Cos(var1[6]);
  t2161 = Sin(var1[6]);
  t12 = Cos(var1[5]);
  t2168 = Cos(var1[7]);
  t2170 = -1.*t2168;
  t2171 = 1. + t2170;
  t2181 = Sin(var1[7]);
  t2212 = Cos(var1[8]);
  t2213 = -1.*t2212;
  t2214 = 1. + t2213;
  t2216 = Sin(var1[8]);
  t2222 = t12*t2168;
  t2225 = -1.*t2082*t2161*t2181;
  t2230 = t2222 + t2225;
  t2207 = t2168*t2082*t2161;
  t2208 = t12*t2181;
  t2210 = t2207 + t2208;
  t938 = -1.*t12;
  t992 = 1. + t938;
  t2140 = -1.*t2122;
  t2146 = 1. + t2140;
  t2158 = -0.330988*t2146;
  t2164 = -0.90524*t2161;
  t2165 = 0. + t2158 + t2164;
  t2172 = -0.97024*t2171;
  t2185 = -0.066675*t2181;
  t2186 = 0. + t2172 + t2185;
  t2190 = -0.066675*t2171;
  t2204 = 0.97024*t2181;
  t2205 = 0. + t2190 + t2204;
  t2215 = -1.45024*t2214;
  t2219 = -0.066675*t2216;
  t2220 = 0. + t2215 + t2219;
  t2232 = -0.066675*t2214;
  t2234 = 1.45024*t2216;
  t2236 = 0. + t2232 + t2234;
  t2294 = t2168*t2082;
  t2295 = t12*t2161*t2181;
  t2296 = t2294 + t2295;
  t2280 = -1.*t12*t2168*t2161;
  t2282 = t2082*t2181;
  t2283 = t2280 + t2282;
  t2335 = 0. + t2122;
  t2346 = -1.*t2335*t2181;
  t2347 = 0. + t2346;
  t2337 = t2335*t2168;
  t2339 = 0. + t2337;

  p_output1(0)=0. - 0.260988*t2082 + 0.324238*t2082*t2122 - 1.*t2082*t2165 + t2082*t2161*t2186 + t12*t2205 + t2210*t2220 - 0.066675*(-1.*t2210*t2216 + t2212*t2230) - 1.45024*(t2210*t2212 + t2216*t2230) + t2230*t2236 - 0.066675*t992;
  p_output1(1)=0. + 0.066675*t2082 - 0.324238*t12*t2122 + t12*t2165 - 1.*t12*t2161*t2186 + t2082*t2205 + t2220*t2283 + t2236*t2296 - 0.066675*(-1.*t2216*t2283 + t2212*t2296) - 1.45024*(t2212*t2283 + t2216*t2296) - 0.260988*t992;
  p_output1(2)=0. - 0.90524*t2146 + 0.330988*t2161 - 0.324238*(0. + t2161) + t2186*t2335 + t2220*t2339 + t2236*t2347 - 0.066675*(-1.*t2216*t2339 + t2212*t2347) - 1.45024*(t2212*t2339 + t2216*t2347);
}


       
void p_rKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
