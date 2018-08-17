/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:46 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipRoll.h"

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
  double t784;
  double t67;
  double t1110;
  double t1137;
  double t1129;
  double t1130;
  double t1158;
  double t1159;
  double t1141;
  double t1162;
  double t1163;
  double t1192;
  double t1210;
  double t1173;
  double t1174;
  double t1175;
  double t1197;
  double t1199;
  double t1206;
  double t1211;
  double t1215;
  double t1216;
  double t1222;
  double t1225;
  double t1227;
  double t1312;
  double t1314;
  double t1413;
  double t1913;
  double t1917;
  double t1919;
  double t2775;
  double t2842;
  double t3090;
  double t3094;
  double t1121;
  double t1165;
  double t1166;
  double t1207;
  double t1217;
  double t1221;
  double t1981;
  double t2166;
  double t2168;
  double t2906;
  double t2967;
  double t3033;
  double t3237;
  double t3272;
  double t3361;
  double t3385;
  double t3393;
  double t3437;
  double t3477;
  double t3610;
  double t3696;
  double t1172;
  double t1181;
  double t1183;
  double t1231;
  double t1478;
  double t1611;
  double t2341;
  double t2344;
  double t2505;
  double t1184;
  double t1187;
  double t1191;
  double t1912;
  double t1921;
  double t1957;
  double t2527;
  double t2557;
  double t2618;
  t784 = Cos(var1[3]);
  t67 = Cos(var1[11]);
  t1110 = Cos(var1[4]);
  t1137 = Sin(var1[3]);
  t1129 = Sin(var1[11]);
  t1130 = Cos(var1[5]);
  t1158 = Sin(var1[4]);
  t1159 = Sin(var1[5]);
  t1141 = -1.*t1130*t1137;
  t1162 = t784*t1158*t1159;
  t1163 = t1141 + t1162;
  t1192 = Sin(var1[12]);
  t1210 = Cos(var1[12]);
  t1173 = t784*t1130;
  t1174 = t1137*t1158*t1159;
  t1175 = t1173 + t1174;
  t1197 = t784*t1130*t1158;
  t1199 = t1137*t1159;
  t1206 = t1197 + t1199;
  t1211 = -1.*t784*t1110*t1129;
  t1215 = t67*t1163;
  t1216 = t1211 + t1215;
  t1222 = t1130*t1137*t1158;
  t1225 = -1.*t784*t1159;
  t1227 = t1222 + t1225;
  t1312 = -1.*t1110*t1129*t1137;
  t1314 = t67*t1175;
  t1413 = t1312 + t1314;
  t1913 = t1129*t1158;
  t1917 = t67*t1110*t1159;
  t1919 = t1913 + t1917;
  t2775 = -1.*t67;
  t2842 = 1. + t2775;
  t3090 = -1.*t1210;
  t3094 = 1. + t3090;
  t1121 = t67*t784*t1110;
  t1165 = t1129*t1163;
  t1166 = t1121 + t1165;
  t1207 = t1192*t1206;
  t1217 = t1210*t1216;
  t1221 = t1207 + t1217;
  t1981 = t1210*t1206;
  t2166 = -1.*t1192*t1216;
  t2168 = t1981 + t2166;
  t2906 = -0.022225*t2842;
  t2967 = -0.086996*t1129;
  t3033 = 0. + t2906 + t2967;
  t3237 = -0.31508*t3094;
  t3272 = 0.156996*t1192;
  t3361 = 0. + t3237 + t3272;
  t3385 = -0.086996*t2842;
  t3393 = 0.022225*t1129;
  t3437 = 0. + t3385 + t3393;
  t3477 = -0.156996*t3094;
  t3610 = -0.31508*t1192;
  t3696 = 0. + t3477 + t3610;
  t1172 = t67*t1110*t1137;
  t1181 = t1129*t1175;
  t1183 = t1172 + t1181;
  t1231 = t1192*t1227;
  t1478 = t1210*t1413;
  t1611 = t1231 + t1478;
  t2341 = t1210*t1227;
  t2344 = -1.*t1192*t1413;
  t2505 = t2341 + t2344;
  t1184 = -1.*t67*t1158;
  t1187 = t1110*t1129*t1159;
  t1191 = t1184 + t1187;
  t1912 = t1110*t1130*t1192;
  t1921 = t1210*t1919;
  t1957 = t1912 + t1921;
  t2527 = t1210*t1110*t1130;
  t2557 = -1.*t1192*t1919;
  t2618 = t2527 + t2557;

  p_output1(0)=t1166;
  p_output1(1)=t1183;
  p_output1(2)=t1191;
  p_output1(3)=0.;
  p_output1(4)=t1221;
  p_output1(5)=t1611;
  p_output1(6)=t1957;
  p_output1(7)=0.;
  p_output1(8)=t2168;
  p_output1(9)=t2505;
  p_output1(10)=t2618;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t1166 - 0.156996*t1221 - 0.31508*t2168 + t1206*t3361 + t1163*t3437 + t1216*t3696 + t1110*t3033*t784 + var1(0);
  p_output1(13)=0. - 0.022225*t1183 - 0.156996*t1611 - 0.31508*t2505 + t1110*t1137*t3033 + t1227*t3361 + t1175*t3437 + t1413*t3696 + var1(1);
  p_output1(14)=0. - 0.022225*t1191 - 0.156996*t1957 - 0.31508*t2618 - 1.*t1158*t3033 + t1110*t1130*t3361 + t1110*t1159*t3437 + t1919*t3696 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
