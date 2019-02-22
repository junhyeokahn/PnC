/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:30 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_LeftFootBottom.h"

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
  double t439;
  double t316;
  double t689;
  double t370;
  double t698;
  double t4;
  double t186;
  double t760;
  double t787;
  double t1056;
  double t894;
  double t953;
  double t1026;
  double t395;
  double t711;
  double t712;
  double t748;
  double t854;
  double t863;
  double t1817;
  double t1921;
  double t1924;
  double t1927;
  double t1818;
  double t1889;
  double t1893;
  double t1991;
  double t2072;
  double t1919;
  double t2037;
  double t2056;
  double t1789;
  double t2097;
  double t2106;
  double t2153;
  double t2253;
  double t2063;
  double t2162;
  double t2231;
  double t1761;
  double t2272;
  double t2287;
  double t2309;
  double t1476;
  double t1487;
  double t1596;
  double t1126;
  double t1132;
  double t1264;
  double t1299;
  double t1313;
  double t1382;
  double t2719;
  double t2728;
  double t2739;
  double t2596;
  double t2601;
  double t2699;
  double t2704;
  double t2805;
  double t2868;
  double t2901;
  double t2978;
  double t3000;
  double t2877;
  double t3010;
  double t3074;
  double t3116;
  double t3131;
  double t3132;
  double t1626;
  double t1635;
  double t1676;
  double t3401;
  double t3405;
  double t3424;
  double t3265;
  double t3274;
  double t3319;
  double t3346;
  double t3490;
  double t3549;
  double t3682;
  double t3735;
  double t3743;
  double t3554;
  double t3749;
  double t3750;
  double t3883;
  double t3918;
  double t3950;
  double t4474;
  double t4477;
  double t4623;
  double t4643;
  double t890;
  double t1091;
  double t1117;
  double t4865;
  double t4876;
  double t5013;
  double t5023;
  double t5251;
  double t5263;
  double t4166;
  double t4170;
  double t4178;
  double t2341;
  double t2402;
  double t2464;
  double t4500;
  double t4519;
  double t4523;
  double t4534;
  double t4586;
  double t4605;
  double t4655;
  double t4665;
  double t4670;
  double t4711;
  double t4759;
  double t4765;
  double t1435;
  double t1615;
  double t1621;
  double t4880;
  double t4889;
  double t4894;
  double t4900;
  double t4927;
  double t4978;
  double t5027;
  double t5046;
  double t5118;
  double t5191;
  double t5239;
  double t5243;
  double t5278;
  double t5280;
  double t5285;
  double t5321;
  double t5323;
  double t5337;
  double t4231;
  double t4261;
  double t4275;
  double t3151;
  double t3161;
  double t3181;
  double t1710;
  double t1712;
  double t1724;
  double t4319;
  double t4381;
  double t4388;
  double t3966;
  double t3970;
  double t4031;
  t439 = Cos(var1[3]);
  t316 = Cos(var1[5]);
  t689 = Sin(var1[4]);
  t370 = Sin(var1[3]);
  t698 = Sin(var1[5]);
  t4 = Cos(var1[7]);
  t186 = Cos(var1[6]);
  t760 = Cos(var1[4]);
  t787 = Sin(var1[6]);
  t1056 = Sin(var1[7]);
  t894 = t439*t316*t689;
  t953 = t370*t698;
  t1026 = t894 + t953;
  t395 = -1.*t316*t370;
  t711 = t439*t689*t698;
  t712 = t395 + t711;
  t748 = t186*t712;
  t854 = -1.*t439*t760*t787;
  t863 = t748 + t854;
  t1817 = Cos(var1[8]);
  t1921 = t439*t760*t186;
  t1924 = t712*t787;
  t1927 = t1921 + t1924;
  t1818 = t4*t1026;
  t1889 = -1.*t863*t1056;
  t1893 = t1818 + t1889;
  t1991 = Sin(var1[8]);
  t2072 = Cos(var1[9]);
  t1919 = t1817*t1893;
  t2037 = t1927*t1991;
  t2056 = t1919 + t2037;
  t1789 = Sin(var1[9]);
  t2097 = t1817*t1927;
  t2106 = -1.*t1893*t1991;
  t2153 = t2097 + t2106;
  t2253 = Cos(var1[10]);
  t2063 = -1.*t1789*t2056;
  t2162 = t2072*t2153;
  t2231 = t2063 + t2162;
  t1761 = Sin(var1[10]);
  t2272 = t2072*t2056;
  t2287 = t1789*t2153;
  t2309 = t2272 + t2287;
  t1476 = t316*t370*t689;
  t1487 = -1.*t439*t698;
  t1596 = t1476 + t1487;
  t1126 = t439*t316;
  t1132 = t370*t689*t698;
  t1264 = t1126 + t1132;
  t1299 = t186*t1264;
  t1313 = -1.*t760*t370*t787;
  t1382 = t1299 + t1313;
  t2719 = t760*t186*t370;
  t2728 = t1264*t787;
  t2739 = t2719 + t2728;
  t2596 = t4*t1596;
  t2601 = -1.*t1382*t1056;
  t2699 = t2596 + t2601;
  t2704 = t1817*t2699;
  t2805 = t2739*t1991;
  t2868 = t2704 + t2805;
  t2901 = t1817*t2739;
  t2978 = -1.*t2699*t1991;
  t3000 = t2901 + t2978;
  t2877 = -1.*t1789*t2868;
  t3010 = t2072*t3000;
  t3074 = t2877 + t3010;
  t3116 = t2072*t2868;
  t3131 = t1789*t3000;
  t3132 = t3116 + t3131;
  t1626 = t760*t186*t698;
  t1635 = t689*t787;
  t1676 = t1626 + t1635;
  t3401 = -1.*t186*t689;
  t3405 = t760*t698*t787;
  t3424 = t3401 + t3405;
  t3265 = t760*t316*t4;
  t3274 = -1.*t1676*t1056;
  t3319 = t3265 + t3274;
  t3346 = t1817*t3319;
  t3490 = t3424*t1991;
  t3549 = t3346 + t3490;
  t3682 = t1817*t3424;
  t3735 = -1.*t3319*t1991;
  t3743 = t3682 + t3735;
  t3554 = -1.*t1789*t3549;
  t3749 = t2072*t3743;
  t3750 = t3554 + t3749;
  t3883 = t2072*t3549;
  t3918 = t1789*t3743;
  t3950 = t3883 + t3918;
  t4474 = -1.*t186;
  t4477 = 1. + t4474;
  t4623 = -1.*t4;
  t4643 = 1. + t4623;
  t890 = t4*t863;
  t1091 = t1026*t1056;
  t1117 = t890 + t1091;
  t4865 = -1.*t1817;
  t4876 = 1. + t4865;
  t5013 = -1.*t2072;
  t5023 = 1. + t5013;
  t5251 = -1.*t2253;
  t5263 = 1. + t5251;
  t4166 = t1761*t2231;
  t4170 = t2253*t2309;
  t4178 = t4166 + t4170;
  t2341 = t2253*t2231;
  t2402 = -1.*t1761*t2309;
  t2464 = t2341 + t2402;
  t4500 = 0.087*t4477;
  t4519 = 0.0222*t787;
  t4523 = 0. + t4500 + t4519;
  t4534 = -0.0222*t4477;
  t4586 = 0.087*t787;
  t4605 = 0. + t4534 + t4586;
  t4655 = 0.157*t4643;
  t4665 = -0.3151*t1056;
  t4670 = 0. + t4655 + t4665;
  t4711 = -0.3151*t4643;
  t4759 = -0.157*t1056;
  t4765 = 0. + t4711 + t4759;
  t1435 = t4*t1382;
  t1615 = t1596*t1056;
  t1621 = t1435 + t1615;
  t4880 = -0.3801*t4876;
  t4889 = -0.0222*t1991;
  t4894 = 0. + t4880 + t4889;
  t4900 = -0.0222*t4876;
  t4927 = 0.3801*t1991;
  t4978 = 0. + t4900 + t4927;
  t5027 = -0.8601*t5023;
  t5046 = -0.0222*t1789;
  t5118 = 0. + t5027 + t5046;
  t5191 = -0.0222*t5023;
  t5239 = 0.8601*t1789;
  t5243 = 0. + t5191 + t5239;
  t5278 = -0.0211*t5263;
  t5280 = 1.3401*t1761;
  t5285 = 0. + t5278 + t5280;
  t5321 = -1.3401*t5263;
  t5323 = -0.0211*t1761;
  t5337 = 0. + t5321 + t5323;
  t4231 = t1761*t3074;
  t4261 = t2253*t3132;
  t4275 = t4231 + t4261;
  t3151 = t2253*t3074;
  t3161 = -1.*t1761*t3132;
  t3181 = t3151 + t3161;
  t1710 = t4*t1676;
  t1712 = t760*t316*t1056;
  t1724 = t1710 + t1712;
  t4319 = t1761*t3750;
  t4381 = t2253*t3950;
  t4388 = t4319 + t4381;
  t3966 = t2253*t3750;
  t3970 = -1.*t1761*t3950;
  t4031 = t3966 + t3970;

  p_output1(0)=t1117;
  p_output1(1)=t1621;
  p_output1(2)=t1724;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1761*t2231 - 1.*t2253*t2309 - 0.000796*t2464;
  p_output1(5)=-1.*t1761*t3074 - 1.*t2253*t3132 - 0.000796*t3181;
  p_output1(6)=-1.*t1761*t3750 - 1.*t2253*t3950 - 0.000796*t4031;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2231*t2253 + t1761*t2309 + 0.000796*t4178;
  p_output1(9)=-1.*t2253*t3074 + t1761*t3132 + 0.000796*t4275;
  p_output1(10)=-1.*t2253*t3750 + t1761*t3950 + 0.000796*t4388;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.167*t1117 + 0.043912*t2464 - 1.325152*t4178 + t1026*t4765 + t1893*t4894 + t1927*t4978 + t2056*t5118 + t2153*t5243 + t2231*t5285 + t2309*t5337 + t4523*t712 + t439*t4605*t760 + t4670*t863 + var1(0);
  p_output1(13)=0. + 0.167*t1621 + 0.043912*t3181 - 1.325152*t4275 + t1264*t4523 + t1382*t4670 + t1596*t4765 + t2699*t4894 + t2739*t4978 + t2868*t5118 + t3000*t5243 + t3074*t5285 + t3132*t5337 + t370*t4605*t760 + var1(1);
  p_output1(14)=0. + 0.167*t1724 + 0.043912*t4031 - 1.325152*t4388 + t1676*t4670 + t3319*t4894 + t3424*t4978 + t3549*t5118 + t3743*t5243 + t3750*t5285 + t3950*t5337 - 1.*t4605*t689 + t316*t4765*t760 + t4523*t698*t760 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootBottom(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
