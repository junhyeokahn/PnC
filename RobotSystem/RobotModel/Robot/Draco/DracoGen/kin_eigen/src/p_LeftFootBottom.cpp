/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:54 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootBottom.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t931;
  double t1144;
  double t1380;
  double t1613;
  double t1884;
  double t157;
  double t274;
  double t642;
  double t1082;
  double t1088;
  double t1091;
  double t1109;
  double t1975;
  double t2386;
  double t2400;
  double t2863;
  double t3088;
  double t2357;
  double t2373;
  double t2374;
  double t3258;
  double t3264;
  double t3311;
  double t3983;
  double t3986;
  double t4010;
  double t4118;
  double t3769;
  double t3912;
  double t3962;
  double t4310;
  double t4328;
  double t4344;
  double t4570;
  double t4630;
  double t4674;
  double t4696;
  double t4883;
  double t4905;
  double t4932;
  double t5134;
  double t5200;
  double t5211;
  double t5224;
  double t5227;
  double t5231;
  double t5252;
  double t5303;
  double t5329;
  double t5331;
  double t5408;
  double t5424;
  double t5442;
  double t1676;
  double t1909;
  double t1938;
  double t2089;
  double t2118;
  double t2222;
  double t5766;
  double t5773;
  double t5877;
  double t2973;
  double t3142;
  double t3172;
  double t3320;
  double t3337;
  double t3454;
  double t5936;
  double t5950;
  double t5972;
  double t6013;
  double t6025;
  double t6068;
  double t4072;
  double t4156;
  double t4284;
  double t4395;
  double t4411;
  double t4469;
  double t4694;
  double t4698;
  double t4861;
  double t6140;
  double t6164;
  double t6176;
  double t6214;
  double t6221;
  double t6256;
  double t5070;
  double t5089;
  double t5098;
  double t5251;
  double t5260;
  double t5292;
  double t6313;
  double t6386;
  double t6391;
  double t6421;
  double t6424;
  double t6429;
  double t5359;
  double t5375;
  double t5401;
  double t6466;
  double t6478;
  double t6529;
  double t6589;
  double t6619;
  double t6625;
  double t6731;
  double t6767;
  double t6780;
  double t6911;
  double t6926;
  double t6937;
  double t6944;
  double t6953;
  double t6954;
  double t6960;
  double t6977;
  double t6980;
  double t6983;
  double t7009;
  double t7036;
  double t7063;
  double t7069;
  double t7074;
  double t7094;
  double t7100;
  double t7108;
  t931 = Cos(var1[3]);
  t1144 = Cos(var1[6]);
  t1380 = -1.*t1144;
  t1613 = 1. + t1380;
  t1884 = Sin(var1[6]);
  t157 = Cos(var1[5]);
  t274 = Sin(var1[3]);
  t642 = -1.*t157*t274;
  t1082 = Sin(var1[4]);
  t1088 = Sin(var1[5]);
  t1091 = t931*t1082*t1088;
  t1109 = t642 + t1091;
  t1975 = Cos(var1[4]);
  t2386 = Cos(var1[7]);
  t2400 = -1.*t2386;
  t2863 = 1. + t2400;
  t3088 = Sin(var1[7]);
  t2357 = t1144*t1109;
  t2373 = -1.*t931*t1975*t1884;
  t2374 = t2357 + t2373;
  t3258 = t931*t157*t1082;
  t3264 = t274*t1088;
  t3311 = t3258 + t3264;
  t3983 = Cos(var1[8]);
  t3986 = -1.*t3983;
  t4010 = 1. + t3986;
  t4118 = Sin(var1[8]);
  t3769 = t2386*t3311;
  t3912 = -1.*t2374*t3088;
  t3962 = t3769 + t3912;
  t4310 = t931*t1975*t1144;
  t4328 = t1109*t1884;
  t4344 = t4310 + t4328;
  t4570 = Cos(var1[9]);
  t4630 = -1.*t4570;
  t4674 = 1. + t4630;
  t4696 = Sin(var1[9]);
  t4883 = t3983*t3962;
  t4905 = t4344*t4118;
  t4932 = t4883 + t4905;
  t5134 = t3983*t4344;
  t5200 = -1.*t3962*t4118;
  t5211 = t5134 + t5200;
  t5224 = Cos(var1[10]);
  t5227 = -1.*t5224;
  t5231 = 1. + t5227;
  t5252 = Sin(var1[10]);
  t5303 = -1.*t4696*t4932;
  t5329 = t4570*t5211;
  t5331 = t5303 + t5329;
  t5408 = t4570*t4932;
  t5424 = t4696*t5211;
  t5442 = t5408 + t5424;
  t1676 = 0.087004*t1613;
  t1909 = 0.022225*t1884;
  t1938 = 0. + t1676 + t1909;
  t2089 = -0.022225*t1613;
  t2118 = 0.087004*t1884;
  t2222 = 0. + t2089 + t2118;
  t5766 = t931*t157;
  t5773 = t274*t1082*t1088;
  t5877 = t5766 + t5773;
  t2973 = 0.157004*t2863;
  t3142 = -0.31508*t3088;
  t3172 = 0. + t2973 + t3142;
  t3320 = -0.31508*t2863;
  t3337 = -0.157004*t3088;
  t3454 = 0. + t3320 + t3337;
  t5936 = t1144*t5877;
  t5950 = -1.*t1975*t274*t1884;
  t5972 = t5936 + t5950;
  t6013 = t157*t274*t1082;
  t6025 = -1.*t931*t1088;
  t6068 = t6013 + t6025;
  t4072 = -0.38008*t4010;
  t4156 = -0.022225*t4118;
  t4284 = 0. + t4072 + t4156;
  t4395 = -0.022225*t4010;
  t4411 = 0.38008*t4118;
  t4469 = 0. + t4395 + t4411;
  t4694 = -0.86008*t4674;
  t4698 = -0.022225*t4696;
  t4861 = 0. + t4694 + t4698;
  t6140 = t2386*t6068;
  t6164 = -1.*t5972*t3088;
  t6176 = t6140 + t6164;
  t6214 = t1975*t1144*t274;
  t6221 = t5877*t1884;
  t6256 = t6214 + t6221;
  t5070 = -0.022225*t4674;
  t5089 = 0.86008*t4696;
  t5098 = 0. + t5070 + t5089;
  t5251 = -0.021147*t5231;
  t5260 = 1.34008*t5252;
  t5292 = 0. + t5251 + t5260;
  t6313 = t3983*t6176;
  t6386 = t6256*t4118;
  t6391 = t6313 + t6386;
  t6421 = t3983*t6256;
  t6424 = -1.*t6176*t4118;
  t6429 = t6421 + t6424;
  t5359 = -1.34008*t5231;
  t5375 = -0.021147*t5252;
  t5401 = 0. + t5359 + t5375;
  t6466 = -1.*t4696*t6391;
  t6478 = t4570*t6429;
  t6529 = t6466 + t6478;
  t6589 = t4570*t6391;
  t6619 = t4696*t6429;
  t6625 = t6589 + t6619;
  t6731 = t1975*t1144*t1088;
  t6767 = t1082*t1884;
  t6780 = t6731 + t6767;
  t6911 = t1975*t157*t2386;
  t6926 = -1.*t6780*t3088;
  t6937 = t6911 + t6926;
  t6944 = -1.*t1144*t1082;
  t6953 = t1975*t1088*t1884;
  t6954 = t6944 + t6953;
  t6960 = t3983*t6937;
  t6977 = t6954*t4118;
  t6980 = t6960 + t6977;
  t6983 = t3983*t6954;
  t7009 = -1.*t6937*t4118;
  t7036 = t6983 + t7009;
  t7063 = -1.*t4696*t6980;
  t7069 = t4570*t7036;
  t7074 = t7063 + t7069;
  t7094 = t4570*t6980;
  t7100 = t4696*t7036;
  t7108 = t7094 + t7100;

  p_output1(0)=0. + t1109*t1938 + t2374*t3172 + 0.167004*(t2374*t2386 + t3088*t3311) + t3311*t3454 + t3962*t4284 + t4344*t4469 + t4861*t4932 + t5098*t5211 + t5292*t5331 + t5401*t5442 - 1.325132*(t5252*t5331 + t5224*t5442) + 0.043865*(t5224*t5331 - 1.*t5252*t5442) + t1975*t2222*t931 + var1(0);
  p_output1(1)=0. + t1975*t2222*t274 + t1938*t5877 + t3172*t5972 + t3454*t6068 + 0.167004*(t2386*t5972 + t3088*t6068) + t4284*t6176 + t4469*t6256 + t4861*t6391 + t5098*t6429 + t5292*t6529 + t5401*t6625 - 1.325132*(t5252*t6529 + t5224*t6625) + 0.043865*(t5224*t6529 - 1.*t5252*t6625) + var1(1);
  p_output1(2)=0. + t1088*t1938*t1975 - 1.*t1082*t2222 + t157*t1975*t3454 + t3172*t6780 + 0.167004*(t157*t1975*t3088 + t2386*t6780) + t4284*t6937 + t4469*t6954 + t4861*t6980 + t5098*t7036 + t5292*t7074 + t5401*t7108 - 1.325132*(t5252*t7074 + t5224*t7108) + 0.043865*(t5224*t7074 - 1.*t5252*t7108) + var1(2);
}


       
void p_LeftFootBottom(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
