/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:02 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lAnkle.h"

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
  double t947;
  double t971;
  double t1016;
  double t213;
  double t1025;
  double t1028;
  double t1034;
  double t1045;
  double t1077;
  double t1078;
  double t1079;
  double t1084;
  double t1061;
  double t1063;
  double t1073;
  double t1093;
  double t1094;
  double t1096;
  double t1118;
  double t1119;
  double t1121;
  double t1124;
  double t1110;
  double t1112;
  double t1117;
  double t1136;
  double t1140;
  double t1143;
  double t411;
  double t891;
  double t1005;
  double t1013;
  double t1015;
  double t1017;
  double t1020;
  double t1040;
  double t1048;
  double t1049;
  double t1056;
  double t1057;
  double t1058;
  double t1080;
  double t1087;
  double t1091;
  double t1100;
  double t1104;
  double t1107;
  double t1200;
  double t1204;
  double t1215;
  double t1218;
  double t1226;
  double t1231;
  double t1122;
  double t1126;
  double t1128;
  double t1146;
  double t1149;
  double t1153;
  double t1263;
  double t1265;
  double t1268;
  double t1275;
  double t1276;
  double t1281;
  double t1311;
  double t1315;
  double t1319;
  double t1321;
  double t1323;
  double t1326;
  double t1328;
  double t1330;
  double t1335;
  double t1338;
  double t1340;
  t947 = Sin(var1[0]);
  t971 = Cos(var1[1]);
  t1016 = Sin(var1[1]);
  t213 = Cos(var1[0]);
  t1025 = Cos(var1[2]);
  t1028 = -1.*t1025;
  t1034 = 1. + t1028;
  t1045 = Sin(var1[2]);
  t1077 = Cos(var1[3]);
  t1078 = -1.*t1077;
  t1079 = 1. + t1078;
  t1084 = Sin(var1[3]);
  t1061 = t1025*t947*t1016;
  t1063 = t213*t1045;
  t1073 = t1061 + t1063;
  t1093 = t213*t1025;
  t1094 = -1.*t947*t1016*t1045;
  t1096 = t1093 + t1094;
  t1118 = Cos(var1[4]);
  t1119 = -1.*t1118;
  t1121 = 1. + t1119;
  t1124 = Sin(var1[4]);
  t1110 = t1077*t1073;
  t1112 = t1096*t1084;
  t1117 = t1110 + t1112;
  t1136 = t1077*t1096;
  t1140 = -1.*t1073*t1084;
  t1143 = t1136 + t1140;
  t411 = -1.*t213;
  t891 = 1. + t411;
  t1005 = -1.*t971;
  t1013 = 1. + t1005;
  t1015 = 0.331012*t1013;
  t1017 = -0.90524*t1016;
  t1020 = 0. + t1015 + t1017;
  t1040 = -0.97024*t1034;
  t1048 = -0.066675*t1045;
  t1049 = 0. + t1040 + t1048;
  t1056 = -0.066675*t1034;
  t1057 = 0.97024*t1045;
  t1058 = 0. + t1056 + t1057;
  t1080 = -1.45024*t1079;
  t1087 = -0.066675*t1084;
  t1091 = 0. + t1080 + t1087;
  t1100 = -0.066675*t1079;
  t1104 = 1.45024*t1084;
  t1107 = 0. + t1100 + t1104;
  t1200 = -1.*t213*t1025*t1016;
  t1204 = t947*t1045;
  t1215 = t1200 + t1204;
  t1218 = t1025*t947;
  t1226 = t213*t1016*t1045;
  t1231 = t1218 + t1226;
  t1122 = -1.93024*t1121;
  t1126 = -0.065597*t1124;
  t1128 = 0. + t1122 + t1126;
  t1146 = -0.065597*t1121;
  t1149 = 1.93024*t1124;
  t1153 = 0. + t1146 + t1149;
  t1263 = t1077*t1215;
  t1265 = t1231*t1084;
  t1268 = t1263 + t1265;
  t1275 = t1077*t1231;
  t1276 = -1.*t1215*t1084;
  t1281 = t1275 + t1276;
  t1311 = 0. + t971;
  t1315 = t1311*t1025;
  t1319 = 0. + t1315;
  t1321 = -1.*t1311*t1045;
  t1323 = 0. + t1321;
  t1326 = t1319*t1077;
  t1328 = t1323*t1084;
  t1330 = t1326 + t1328;
  t1335 = t1077*t1323;
  t1338 = -1.*t1319*t1084;
  t1340 = t1335 + t1338;

  p_output1(0)=0. + t1073*t1091 + t1096*t1107 + t1117*t1128 - 0.065597*(-1.*t1117*t1124 + t1118*t1143) - 1.93024*(t1117*t1118 + t1124*t1143) + t1143*t1153 + t1058*t213 - 0.066675*t891 + 0.261012*t947 - 1.*t1020*t947 + t1016*t1049*t947 - 0.341012*t947*t971;
  p_output1(1)=0. + t1091*t1215 + t1107*t1231 + t1128*t1268 + t1153*t1281 - 0.065597*(-1.*t1124*t1268 + t1118*t1281) - 1.93024*(t1118*t1268 + t1124*t1281) + t1020*t213 - 1.*t1016*t1049*t213 + 0.261012*t891 + 0.066675*t947 + t1058*t947 + 0.341012*t213*t971;
  p_output1(2)=0. - 0.90524*t1013 - 0.331012*t1016 + 0.341012*(0. + t1016) + t1049*t1311 + t1091*t1319 + t1107*t1323 + t1128*t1330 + t1153*t1340 - 0.065597*(-1.*t1124*t1330 + t1118*t1340) - 1.93024*(t1118*t1330 + t1124*t1340);
}


       
void p_lAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
