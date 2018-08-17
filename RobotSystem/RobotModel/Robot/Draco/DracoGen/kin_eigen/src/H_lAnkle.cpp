/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:42 GMT-05:00
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
  double t343;
  double t546;
  double t214;
  double t360;
  double t556;
  double t955;
  double t799;
  double t804;
  double t805;
  double t824;
  double t968;
  double t117;
  double t1055;
  double t1063;
  double t1071;
  double t134;
  double t470;
  double t566;
  double t577;
  double t778;
  double t876;
  double t978;
  double t996;
  double t1015;
  double t1030;
  double t1044;
  double t1073;
  double t1099;
  double t1052;
  double t1074;
  double t1097;
  double t107;
  double t1100;
  double t1127;
  double t1131;
  double t1154;
  double t1098;
  double t1140;
  double t1145;
  double t106;
  double t1176;
  double t1213;
  double t1214;
  double t1532;
  double t1552;
  double t1568;
  double t1701;
  double t1753;
  double t1779;
  double t1474;
  double t1492;
  double t1505;
  double t1515;
  double t1599;
  double t1621;
  double t1642;
  double t1683;
  double t1694;
  double t1697;
  double t1780;
  double t1787;
  double t1882;
  double t1949;
  double t1958;
  double t1837;
  double t1961;
  double t1977;
  double t2016;
  double t2020;
  double t2129;
  double t2911;
  double t2918;
  double t2922;
  double t2538;
  double t2614;
  double t2729;
  double t2765;
  double t2804;
  double t2833;
  double t2840;
  double t2927;
  double t2932;
  double t3020;
  double t3022;
  double t3038;
  double t3008;
  double t3042;
  double t3073;
  double t3111;
  double t3112;
  double t3130;
  double t1146;
  double t1249;
  double t1984;
  double t2164;
  double t3080;
  double t3189;
  double t4192;
  double t4234;
  double t4576;
  double t4588;
  double t3364;
  double t3476;
  double t3492;
  double t4836;
  double t4846;
  double t4916;
  double t4929;
  double t5132;
  double t5136;
  double t3778;
  double t1331;
  double t1399;
  double t1443;
  double t4237;
  double t4244;
  double t4437;
  double t4452;
  double t4499;
  double t4503;
  double t4598;
  double t4639;
  double t4640;
  double t4734;
  double t4743;
  double t4798;
  double t3633;
  double t3634;
  double t3660;
  double t4847;
  double t4853;
  double t4860;
  double t4866;
  double t4881;
  double t4901;
  double t4933;
  double t4994;
  double t5004;
  double t5030;
  double t5039;
  double t5052;
  double t5156;
  double t5167;
  double t5203;
  double t5212;
  double t5213;
  double t5234;
  double t3942;
  double t2290;
  double t2326;
  double t2338;
  double t3661;
  double t3663;
  double t3666;
  double t4089;
  double t3223;
  double t3227;
  double t3239;
  t343 = Cos(var1[5]);
  t546 = Sin(var1[3]);
  t214 = Cos(var1[3]);
  t360 = Sin(var1[4]);
  t556 = Sin(var1[5]);
  t955 = Cos(var1[4]);
  t799 = Cos(var1[6]);
  t804 = -1.*t343*t546;
  t805 = t214*t360*t556;
  t824 = t804 + t805;
  t968 = Sin(var1[6]);
  t117 = Cos(var1[8]);
  t1055 = t214*t955*t799;
  t1063 = t824*t968;
  t1071 = t1055 + t1063;
  t134 = Cos(var1[7]);
  t470 = t214*t343*t360;
  t566 = t546*t556;
  t577 = t470 + t566;
  t778 = t134*t577;
  t876 = t799*t824;
  t978 = -1.*t214*t955*t968;
  t996 = t876 + t978;
  t1015 = Sin(var1[7]);
  t1030 = -1.*t996*t1015;
  t1044 = t778 + t1030;
  t1073 = Sin(var1[8]);
  t1099 = Cos(var1[9]);
  t1052 = t117*t1044;
  t1074 = t1071*t1073;
  t1097 = t1052 + t1074;
  t107 = Sin(var1[9]);
  t1100 = t117*t1071;
  t1127 = -1.*t1044*t1073;
  t1131 = t1100 + t1127;
  t1154 = Cos(var1[10]);
  t1098 = -1.*t107*t1097;
  t1140 = t1099*t1131;
  t1145 = t1098 + t1140;
  t106 = Sin(var1[10]);
  t1176 = t1099*t1097;
  t1213 = t107*t1131;
  t1214 = t1176 + t1213;
  t1532 = t214*t343;
  t1552 = t546*t360*t556;
  t1568 = t1532 + t1552;
  t1701 = t955*t799*t546;
  t1753 = t1568*t968;
  t1779 = t1701 + t1753;
  t1474 = t343*t546*t360;
  t1492 = -1.*t214*t556;
  t1505 = t1474 + t1492;
  t1515 = t134*t1505;
  t1599 = t799*t1568;
  t1621 = -1.*t955*t546*t968;
  t1642 = t1599 + t1621;
  t1683 = -1.*t1642*t1015;
  t1694 = t1515 + t1683;
  t1697 = t117*t1694;
  t1780 = t1779*t1073;
  t1787 = t1697 + t1780;
  t1882 = t117*t1779;
  t1949 = -1.*t1694*t1073;
  t1958 = t1882 + t1949;
  t1837 = -1.*t107*t1787;
  t1961 = t1099*t1958;
  t1977 = t1837 + t1961;
  t2016 = t1099*t1787;
  t2020 = t107*t1958;
  t2129 = t2016 + t2020;
  t2911 = -1.*t799*t360;
  t2918 = t955*t556*t968;
  t2922 = t2911 + t2918;
  t2538 = t955*t343*t134;
  t2614 = t955*t799*t556;
  t2729 = t360*t968;
  t2765 = t2614 + t2729;
  t2804 = -1.*t2765*t1015;
  t2833 = t2538 + t2804;
  t2840 = t117*t2833;
  t2927 = t2922*t1073;
  t2932 = t2840 + t2927;
  t3020 = t117*t2922;
  t3022 = -1.*t2833*t1073;
  t3038 = t3020 + t3022;
  t3008 = -1.*t107*t2932;
  t3042 = t1099*t3038;
  t3073 = t3008 + t3042;
  t3111 = t1099*t2932;
  t3112 = t107*t3038;
  t3130 = t3111 + t3112;
  t1146 = t106*t1145;
  t1249 = t1154*t1214;
  t1984 = t106*t1977;
  t2164 = t1154*t2129;
  t3080 = t106*t3073;
  t3189 = t1154*t3130;
  t4192 = -1.*t799;
  t4234 = 1. + t4192;
  t4576 = -1.*t134;
  t4588 = 1. + t4576;
  t3364 = t134*t996;
  t3476 = t577*t1015;
  t3492 = t3364 + t3476;
  t4836 = -1.*t117;
  t4846 = 1. + t4836;
  t4916 = -1.*t1099;
  t4929 = 1. + t4916;
  t5132 = -1.*t1154;
  t5136 = 1. + t5132;
  t3778 = t1146 + t1249;
  t1331 = t1154*t1145;
  t1399 = -1.*t106*t1214;
  t1443 = t1331 + t1399;
  t4237 = 0.087004*t4234;
  t4244 = 0.022225*t968;
  t4437 = 0. + t4237 + t4244;
  t4452 = -0.022225*t4234;
  t4499 = 0.087004*t968;
  t4503 = 0. + t4452 + t4499;
  t4598 = 0.157004*t4588;
  t4639 = -0.31508*t1015;
  t4640 = 0. + t4598 + t4639;
  t4734 = -0.31508*t4588;
  t4743 = -0.157004*t1015;
  t4798 = 0. + t4734 + t4743;
  t3633 = t134*t1642;
  t3634 = t1505*t1015;
  t3660 = t3633 + t3634;
  t4847 = -0.38008*t4846;
  t4853 = -0.022225*t1073;
  t4860 = 0. + t4847 + t4853;
  t4866 = -0.022225*t4846;
  t4881 = 0.38008*t1073;
  t4901 = 0. + t4866 + t4881;
  t4933 = -0.86008*t4929;
  t4994 = -0.022225*t107;
  t5004 = 0. + t4933 + t4994;
  t5030 = -0.022225*t4929;
  t5039 = 0.86008*t107;
  t5052 = 0. + t5030 + t5039;
  t5156 = -0.021147*t5136;
  t5167 = 1.34008*t106;
  t5203 = 0. + t5156 + t5167;
  t5212 = -1.34008*t5136;
  t5213 = -0.021147*t106;
  t5234 = 0. + t5212 + t5213;
  t3942 = t1984 + t2164;
  t2290 = t1154*t1977;
  t2326 = -1.*t106*t2129;
  t2338 = t2290 + t2326;
  t3661 = t134*t2765;
  t3663 = t955*t343*t1015;
  t3666 = t3661 + t3663;
  t4089 = t3080 + t3189;
  t3223 = t1154*t3073;
  t3227 = -1.*t106*t3130;
  t3239 = t3223 + t3227;

  p_output1(0)=t1146 + t1249 + 0.000796*t1443;
  p_output1(1)=t1984 + t2164 + 0.000796*t2338;
  p_output1(2)=t3080 + t3189 + 0.000796*t3239;
  p_output1(3)=0.;
  p_output1(4)=t3492;
  p_output1(5)=t3660;
  p_output1(6)=t3666;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1145*t1154 + t106*t1214 + 0.000796*t3778;
  p_output1(9)=-1.*t1154*t1977 + t106*t2129 + 0.000796*t3942;
  p_output1(10)=-1.*t1154*t3073 + t106*t3130 + 0.000796*t4089;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.021147*t1443 + 0.167004*t3492 - 1.34008*t3778 + t1044*t4860 + t1071*t4901 + t1097*t5004 + t1131*t5052 + t1145*t5203 + t1214*t5234 + t4798*t577 + t4437*t824 + t214*t4503*t955 + t4640*t996 + var1(0);
  p_output1(13)=0. - 0.021147*t2338 + 0.167004*t3660 - 1.34008*t3942 + t1568*t4437 + t1642*t4640 + t1505*t4798 + t1694*t4860 + t1779*t4901 + t1787*t5004 + t1958*t5052 + t1977*t5203 + t2129*t5234 + t4503*t546*t955 + var1(1);
  p_output1(14)=0. - 0.021147*t3239 + 0.167004*t3666 - 1.34008*t4089 - 1.*t360*t4503 + t2765*t4640 + t2833*t4860 + t2922*t4901 + t2932*t5004 + t3038*t5052 + t3073*t5203 + t3130*t5234 + t343*t4798*t955 + t4437*t556*t955 + var1(2);
  p_output1(15)=1.;
}


       
void H_lAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
