/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:53 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t1157;
  double t1845;
  double t1923;
  double t1894;
  double t1934;
  double t1424;
  double t1813;
  double t1122;
  double t1920;
  double t1938;
  double t1939;
  double t2118;
  double t1613;
  double t1975;
  double t2087;
  double t1102;
  double t2144;
  double t2222;
  double t2411;
  double t2443;
  double t2449;
  double t2503;
  double t2536;
  double t2574;
  double t2781;
  double t2867;
  double t2944;
  double t3007;
  double t2089;
  double t2973;
  double t2996;
  double t1024;
  double t3088;
  double t3103;
  double t3142;
  double t3238;
  double t3000;
  double t3172;
  double t3182;
  double t177;
  double t3258;
  double t3264;
  double t3280;
  double t3613;
  double t3679;
  double t3715;
  double t3585;
  double t3789;
  double t3797;
  double t3912;
  double t3955;
  double t3959;
  double t3986;
  double t3996;
  double t4010;
  double t4020;
  double t4033;
  double t4045;
  double t3857;
  double t4072;
  double t4217;
  double t4284;
  double t4293;
  double t4305;
  double t4278;
  double t4310;
  double t4333;
  double t4395;
  double t4411;
  double t4426;
  double t4626;
  double t4630;
  double t4674;
  double t4694;
  double t4696;
  double t4698;
  double t4768;
  double t4830;
  double t4857;
  double t4675;
  double t4861;
  double t4862;
  double t4883;
  double t4888;
  double t4890;
  double t4873;
  double t4905;
  double t4922;
  double t4978;
  double t4989;
  double t5031;
  double t3212;
  double t3311;
  double t4344;
  double t4427;
  double t4932;
  double t5034;
  t1157 = Cos(var1[3]);
  t1845 = Cos(var1[5]);
  t1923 = Sin(var1[4]);
  t1894 = Sin(var1[3]);
  t1934 = Sin(var1[5]);
  t1424 = Cos(var1[4]);
  t1813 = Sin(var1[11]);
  t1122 = Cos(var1[11]);
  t1920 = -1.*t1845*t1894;
  t1938 = t1157*t1923*t1934;
  t1939 = t1920 + t1938;
  t2118 = Cos(var1[13]);
  t1613 = t1122*t1157*t1424;
  t1975 = t1813*t1939;
  t2087 = t1613 + t1975;
  t1102 = Sin(var1[13]);
  t2144 = Cos(var1[12]);
  t2222 = t1157*t1845*t1923;
  t2411 = t1894*t1934;
  t2443 = t2222 + t2411;
  t2449 = t2144*t2443;
  t2503 = Sin(var1[12]);
  t2536 = -1.*t1157*t1424*t1813;
  t2574 = t1122*t1939;
  t2781 = t2536 + t2574;
  t2867 = -1.*t2503*t2781;
  t2944 = t2449 + t2867;
  t3007 = Cos(var1[14]);
  t2089 = t1102*t2087;
  t2973 = t2118*t2944;
  t2996 = t2089 + t2973;
  t1024 = Sin(var1[14]);
  t3088 = t2118*t2087;
  t3103 = -1.*t1102*t2944;
  t3142 = t3088 + t3103;
  t3238 = Cos(var1[15]);
  t3000 = -1.*t1024*t2996;
  t3172 = t3007*t3142;
  t3182 = t3000 + t3172;
  t177 = Sin(var1[15]);
  t3258 = t3007*t2996;
  t3264 = t1024*t3142;
  t3280 = t3258 + t3264;
  t3613 = t1157*t1845;
  t3679 = t1894*t1923*t1934;
  t3715 = t3613 + t3679;
  t3585 = t1122*t1424*t1894;
  t3789 = t1813*t3715;
  t3797 = t3585 + t3789;
  t3912 = t1845*t1894*t1923;
  t3955 = -1.*t1157*t1934;
  t3959 = t3912 + t3955;
  t3986 = t2144*t3959;
  t3996 = -1.*t1424*t1813*t1894;
  t4010 = t1122*t3715;
  t4020 = t3996 + t4010;
  t4033 = -1.*t2503*t4020;
  t4045 = t3986 + t4033;
  t3857 = t1102*t3797;
  t4072 = t2118*t4045;
  t4217 = t3857 + t4072;
  t4284 = t2118*t3797;
  t4293 = -1.*t1102*t4045;
  t4305 = t4284 + t4293;
  t4278 = -1.*t1024*t4217;
  t4310 = t3007*t4305;
  t4333 = t4278 + t4310;
  t4395 = t3007*t4217;
  t4411 = t1024*t4305;
  t4426 = t4395 + t4411;
  t4626 = -1.*t1122*t1923;
  t4630 = t1424*t1813*t1934;
  t4674 = t4626 + t4630;
  t4694 = t2144*t1424*t1845;
  t4696 = t1813*t1923;
  t4698 = t1122*t1424*t1934;
  t4768 = t4696 + t4698;
  t4830 = -1.*t2503*t4768;
  t4857 = t4694 + t4830;
  t4675 = t1102*t4674;
  t4861 = t2118*t4857;
  t4862 = t4675 + t4861;
  t4883 = t2118*t4674;
  t4888 = -1.*t1102*t4857;
  t4890 = t4883 + t4888;
  t4873 = -1.*t1024*t4862;
  t4905 = t3007*t4890;
  t4922 = t4873 + t4905;
  t4978 = t3007*t4862;
  t4989 = t1024*t4890;
  t5031 = t4978 + t4989;
  t3212 = t177*t3182;
  t3311 = t3238*t3280;
  t4344 = t177*t4333;
  t4427 = t3238*t4426;
  t4932 = t177*t4922;
  t5034 = t3238*t5031;

  p_output1(0)=t3212 + 0.000796*(t3182*t3238 - 1.*t177*t3280) + t3311;
  p_output1(1)=t4344 + 0.000796*(t3238*t4333 - 1.*t177*t4426) + t4427;
  p_output1(2)=t4932 + 0.000796*(t3238*t4922 - 1.*t177*t5031) + t5034;
  p_output1(3)=t2443*t2503 + t2144*t2781;
  p_output1(4)=t2503*t3959 + t2144*t4020;
  p_output1(5)=t1424*t1845*t2503 + t2144*t4768;
  p_output1(6)=-1.*t3182*t3238 + t177*t3280 + 0.000796*(t3212 + t3311);
  p_output1(7)=-1.*t3238*t4333 + t177*t4426 + 0.000796*(t4344 + t4427);
  p_output1(8)=-1.*t3238*t4922 + t177*t5031 + 0.000796*(t4932 + t5034);
}


       
void R_rAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
