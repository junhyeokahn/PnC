/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:21 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lAnkle.h"

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
  double t876;
  double t1079;
  double t858;
  double t893;
  double t1080;
  double t1172;
  double t1158;
  double t1162;
  double t1163;
  double t1165;
  double t1174;
  double t668;
  double t1192;
  double t1193;
  double t1197;
  double t789;
  double t1024;
  double t1083;
  double t1109;
  double t1129;
  double t1170;
  double t1175;
  double t1180;
  double t1181;
  double t1182;
  double t1183;
  double t1198;
  double t1215;
  double t1191;
  double t1199;
  double t1211;
  double t664;
  double t1216;
  double t1217;
  double t1223;
  double t1231;
  double t1212;
  double t1226;
  double t1227;
  double t615;
  double t1232;
  double t1234;
  double t1292;
  double t1608;
  double t1697;
  double t1713;
  double t1995;
  double t2019;
  double t2096;
  double t1567;
  double t1569;
  double t1576;
  double t1601;
  double t1759;
  double t1798;
  double t1802;
  double t1804;
  double t1812;
  double t1926;
  double t2121;
  double t2133;
  double t2225;
  double t2246;
  double t2273;
  double t2217;
  double t2305;
  double t2307;
  double t2310;
  double t2315;
  double t2375;
  double t2914;
  double t2944;
  double t2991;
  double t2750;
  double t2766;
  double t2812;
  double t2819;
  double t2823;
  double t2863;
  double t2867;
  double t3007;
  double t3018;
  double t3068;
  double t3085;
  double t3113;
  double t3053;
  double t3119;
  double t3121;
  double t3300;
  double t3313;
  double t3364;
  double t1230;
  double t1458;
  double t2308;
  double t2379;
  double t3299;
  double t3449;
  double t4199;
  double t4206;
  double t4378;
  double t4380;
  double t3809;
  double t3834;
  double t3875;
  double t4419;
  double t4428;
  double t4457;
  double t4458;
  double t4587;
  double t4594;
  double t3953;
  double t1471;
  double t1473;
  double t1489;
  double t4263;
  double t4283;
  double t4295;
  double t4344;
  double t4350;
  double t4364;
  double t4383;
  double t4392;
  double t4397;
  double t4405;
  double t4407;
  double t4411;
  double t3878;
  double t3893;
  double t3907;
  double t4431;
  double t4432;
  double t4434;
  double t4444;
  double t4449;
  double t4450;
  double t4460;
  double t4464;
  double t4466;
  double t4550;
  double t4553;
  double t4554;
  double t4602;
  double t4658;
  double t4705;
  double t4801;
  double t4812;
  double t4814;
  double t4003;
  double t2385;
  double t2516;
  double t2558;
  double t3908;
  double t3910;
  double t3921;
  double t4169;
  double t3511;
  double t3531;
  double t3670;
  t876 = Cos(var1[5]);
  t1079 = Sin(var1[3]);
  t858 = Cos(var1[3]);
  t893 = Sin(var1[4]);
  t1080 = Sin(var1[5]);
  t1172 = Cos(var1[4]);
  t1158 = Cos(var1[6]);
  t1162 = -1.*t876*t1079;
  t1163 = t858*t893*t1080;
  t1165 = t1162 + t1163;
  t1174 = Sin(var1[6]);
  t668 = Cos(var1[8]);
  t1192 = t858*t1172*t1158;
  t1193 = t1165*t1174;
  t1197 = t1192 + t1193;
  t789 = Cos(var1[7]);
  t1024 = t858*t876*t893;
  t1083 = t1079*t1080;
  t1109 = t1024 + t1083;
  t1129 = t789*t1109;
  t1170 = t1158*t1165;
  t1175 = -1.*t858*t1172*t1174;
  t1180 = t1170 + t1175;
  t1181 = Sin(var1[7]);
  t1182 = -1.*t1180*t1181;
  t1183 = t1129 + t1182;
  t1198 = Sin(var1[8]);
  t1215 = Cos(var1[9]);
  t1191 = t668*t1183;
  t1199 = t1197*t1198;
  t1211 = t1191 + t1199;
  t664 = Sin(var1[9]);
  t1216 = t668*t1197;
  t1217 = -1.*t1183*t1198;
  t1223 = t1216 + t1217;
  t1231 = Cos(var1[10]);
  t1212 = -1.*t664*t1211;
  t1226 = t1215*t1223;
  t1227 = t1212 + t1226;
  t615 = Sin(var1[10]);
  t1232 = t1215*t1211;
  t1234 = t664*t1223;
  t1292 = t1232 + t1234;
  t1608 = t858*t876;
  t1697 = t1079*t893*t1080;
  t1713 = t1608 + t1697;
  t1995 = t1172*t1158*t1079;
  t2019 = t1713*t1174;
  t2096 = t1995 + t2019;
  t1567 = t876*t1079*t893;
  t1569 = -1.*t858*t1080;
  t1576 = t1567 + t1569;
  t1601 = t789*t1576;
  t1759 = t1158*t1713;
  t1798 = -1.*t1172*t1079*t1174;
  t1802 = t1759 + t1798;
  t1804 = -1.*t1802*t1181;
  t1812 = t1601 + t1804;
  t1926 = t668*t1812;
  t2121 = t2096*t1198;
  t2133 = t1926 + t2121;
  t2225 = t668*t2096;
  t2246 = -1.*t1812*t1198;
  t2273 = t2225 + t2246;
  t2217 = -1.*t664*t2133;
  t2305 = t1215*t2273;
  t2307 = t2217 + t2305;
  t2310 = t1215*t2133;
  t2315 = t664*t2273;
  t2375 = t2310 + t2315;
  t2914 = -1.*t1158*t893;
  t2944 = t1172*t1080*t1174;
  t2991 = t2914 + t2944;
  t2750 = t1172*t876*t789;
  t2766 = t1172*t1158*t1080;
  t2812 = t893*t1174;
  t2819 = t2766 + t2812;
  t2823 = -1.*t2819*t1181;
  t2863 = t2750 + t2823;
  t2867 = t668*t2863;
  t3007 = t2991*t1198;
  t3018 = t2867 + t3007;
  t3068 = t668*t2991;
  t3085 = -1.*t2863*t1198;
  t3113 = t3068 + t3085;
  t3053 = -1.*t664*t3018;
  t3119 = t1215*t3113;
  t3121 = t3053 + t3119;
  t3300 = t1215*t3018;
  t3313 = t664*t3113;
  t3364 = t3300 + t3313;
  t1230 = t615*t1227;
  t1458 = t1231*t1292;
  t2308 = t615*t2307;
  t2379 = t1231*t2375;
  t3299 = t615*t3121;
  t3449 = t1231*t3364;
  t4199 = -1.*t1158;
  t4206 = 1. + t4199;
  t4378 = -1.*t789;
  t4380 = 1. + t4378;
  t3809 = t789*t1180;
  t3834 = t1109*t1181;
  t3875 = t3809 + t3834;
  t4419 = -1.*t668;
  t4428 = 1. + t4419;
  t4457 = -1.*t1215;
  t4458 = 1. + t4457;
  t4587 = -1.*t1231;
  t4594 = 1. + t4587;
  t3953 = t1230 + t1458;
  t1471 = t1231*t1227;
  t1473 = -1.*t615*t1292;
  t1489 = t1471 + t1473;
  t4263 = 0.087*t4206;
  t4283 = 0.0222*t1174;
  t4295 = 0. + t4263 + t4283;
  t4344 = -0.0222*t4206;
  t4350 = 0.087*t1174;
  t4364 = 0. + t4344 + t4350;
  t4383 = 0.157*t4380;
  t4392 = -0.3151*t1181;
  t4397 = 0. + t4383 + t4392;
  t4405 = -0.3151*t4380;
  t4407 = -0.157*t1181;
  t4411 = 0. + t4405 + t4407;
  t3878 = t789*t1802;
  t3893 = t1576*t1181;
  t3907 = t3878 + t3893;
  t4431 = -0.3801*t4428;
  t4432 = -0.0222*t1198;
  t4434 = 0. + t4431 + t4432;
  t4444 = -0.0222*t4428;
  t4449 = 0.3801*t1198;
  t4450 = 0. + t4444 + t4449;
  t4460 = -0.8601*t4458;
  t4464 = -0.0222*t664;
  t4466 = 0. + t4460 + t4464;
  t4550 = -0.0222*t4458;
  t4553 = 0.8601*t664;
  t4554 = 0. + t4550 + t4553;
  t4602 = -0.0211*t4594;
  t4658 = 1.3401*t615;
  t4705 = 0. + t4602 + t4658;
  t4801 = -1.3401*t4594;
  t4812 = -0.0211*t615;
  t4814 = 0. + t4801 + t4812;
  t4003 = t2308 + t2379;
  t2385 = t1231*t2307;
  t2516 = -1.*t615*t2375;
  t2558 = t2385 + t2516;
  t3908 = t789*t2819;
  t3910 = t1172*t876*t1181;
  t3921 = t3908 + t3910;
  t4169 = t3299 + t3449;
  t3511 = t1231*t3121;
  t3531 = -1.*t615*t3364;
  t3670 = t3511 + t3531;

  p_output1(0)=t1230 + t1458 + 0.000796*t1489;
  p_output1(1)=t2308 + t2379 + 0.000796*t2558;
  p_output1(2)=t3299 + t3449 + 0.000796*t3670;
  p_output1(3)=0.;
  p_output1(4)=t3875;
  p_output1(5)=t3907;
  p_output1(6)=t3921;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1227*t1231 + 0.000796*t3953 + t1292*t615;
  p_output1(9)=-1.*t1231*t2307 + 0.000796*t4003 + t2375*t615;
  p_output1(10)=-1.*t1231*t3121 + 0.000796*t4169 + t3364*t615;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0211*t1489 + 0.167*t3875 - 1.3401*t3953 + t1165*t4295 + t1180*t4397 + t1109*t4411 + t1183*t4434 + t1197*t4450 + t1211*t4466 + t1223*t4554 + t1227*t4705 + t1292*t4814 + t1172*t4364*t858 + var1(0);
  p_output1(13)=0. - 0.0211*t2558 + 0.167*t3907 - 1.3401*t4003 + t1713*t4295 + t1079*t1172*t4364 + t1802*t4397 + t1576*t4411 + t1812*t4434 + t2096*t4450 + t2133*t4466 + t2273*t4554 + t2307*t4705 + t2375*t4814 + var1(1);
  p_output1(14)=0. - 0.0211*t3670 + 0.167*t3921 - 1.3401*t4169 + t1080*t1172*t4295 + t2819*t4397 + t2863*t4434 + t2991*t4450 + t3018*t4466 + t3113*t4554 + t3121*t4705 + t3364*t4814 + t1172*t4411*t876 - 1.*t4364*t893 + var1(2);
  p_output1(15)=1.;
}


       
void H_lAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
