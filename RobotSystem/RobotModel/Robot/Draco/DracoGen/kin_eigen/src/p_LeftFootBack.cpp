/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:32 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_LeftFootBack.h"

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
  double t1156;
  double t1554;
  double t1638;
  double t1680;
  double t1920;
  double t458;
  double t990;
  double t1047;
  double t1212;
  double t1253;
  double t1426;
  double t1520;
  double t2149;
  double t2754;
  double t2765;
  double t2769;
  double t2828;
  double t2590;
  double t2621;
  double t2651;
  double t3053;
  double t3123;
  double t3131;
  double t3766;
  double t3787;
  double t3810;
  double t4027;
  double t3614;
  double t3649;
  double t3738;
  double t4159;
  double t4258;
  double t4299;
  double t4479;
  double t4523;
  double t4758;
  double t4843;
  double t4874;
  double t4910;
  double t4912;
  double t5007;
  double t5009;
  double t5020;
  double t5034;
  double t5070;
  double t5077;
  double t5125;
  double t5137;
  double t5139;
  double t5180;
  double t5243;
  double t5289;
  double t5299;
  double t1767;
  double t1980;
  double t1985;
  double t2159;
  double t2181;
  double t2406;
  double t5480;
  double t5490;
  double t5492;
  double t2824;
  double t2832;
  double t2918;
  double t3146;
  double t3179;
  double t3463;
  double t5529;
  double t5543;
  double t5548;
  double t5588;
  double t5614;
  double t5626;
  double t3833;
  double t4074;
  double t4142;
  double t4339;
  double t4366;
  double t4400;
  double t4820;
  double t4857;
  double t4863;
  double t5676;
  double t5681;
  double t5684;
  double t5702;
  double t5705;
  double t5712;
  double t4933;
  double t4957;
  double t4978;
  double t5124;
  double t5131;
  double t5135;
  double t5728;
  double t5729;
  double t5732;
  double t5774;
  double t5776;
  double t5809;
  double t5207;
  double t5216;
  double t5241;
  double t5844;
  double t5854;
  double t5863;
  double t5884;
  double t5896;
  double t5903;
  double t6087;
  double t6089;
  double t6090;
  double t6140;
  double t6158;
  double t6168;
  double t6209;
  double t6218;
  double t6238;
  double t6324;
  double t6326;
  double t6334;
  double t6344;
  double t6364;
  double t6376;
  double t6420;
  double t6428;
  double t6445;
  double t6492;
  double t6494;
  double t6504;
  t1156 = Cos(var1[3]);
  t1554 = Cos(var1[6]);
  t1638 = -1.*t1554;
  t1680 = 1. + t1638;
  t1920 = Sin(var1[6]);
  t458 = Cos(var1[5]);
  t990 = Sin(var1[3]);
  t1047 = -1.*t458*t990;
  t1212 = Sin(var1[4]);
  t1253 = Sin(var1[5]);
  t1426 = t1156*t1212*t1253;
  t1520 = t1047 + t1426;
  t2149 = Cos(var1[4]);
  t2754 = Cos(var1[7]);
  t2765 = -1.*t2754;
  t2769 = 1. + t2765;
  t2828 = Sin(var1[7]);
  t2590 = t1554*t1520;
  t2621 = -1.*t1156*t2149*t1920;
  t2651 = t2590 + t2621;
  t3053 = t1156*t458*t1212;
  t3123 = t990*t1253;
  t3131 = t3053 + t3123;
  t3766 = Cos(var1[8]);
  t3787 = -1.*t3766;
  t3810 = 1. + t3787;
  t4027 = Sin(var1[8]);
  t3614 = t2754*t3131;
  t3649 = -1.*t2651*t2828;
  t3738 = t3614 + t3649;
  t4159 = t1156*t2149*t1554;
  t4258 = t1520*t1920;
  t4299 = t4159 + t4258;
  t4479 = Cos(var1[9]);
  t4523 = -1.*t4479;
  t4758 = 1. + t4523;
  t4843 = Sin(var1[9]);
  t4874 = t3766*t3738;
  t4910 = t4299*t4027;
  t4912 = t4874 + t4910;
  t5007 = t3766*t4299;
  t5009 = -1.*t3738*t4027;
  t5020 = t5007 + t5009;
  t5034 = Cos(var1[10]);
  t5070 = -1.*t5034;
  t5077 = 1. + t5070;
  t5125 = Sin(var1[10]);
  t5137 = -1.*t4843*t4912;
  t5139 = t4479*t5020;
  t5180 = t5137 + t5139;
  t5243 = t4479*t4912;
  t5289 = t4843*t5020;
  t5299 = t5243 + t5289;
  t1767 = 0.087*t1680;
  t1980 = 0.0222*t1920;
  t1985 = 0. + t1767 + t1980;
  t2159 = -0.0222*t1680;
  t2181 = 0.087*t1920;
  t2406 = 0. + t2159 + t2181;
  t5480 = t1156*t458;
  t5490 = t990*t1212*t1253;
  t5492 = t5480 + t5490;
  t2824 = 0.157*t2769;
  t2832 = -0.3151*t2828;
  t2918 = 0. + t2824 + t2832;
  t3146 = -0.3151*t2769;
  t3179 = -0.157*t2828;
  t3463 = 0. + t3146 + t3179;
  t5529 = t1554*t5492;
  t5543 = -1.*t2149*t990*t1920;
  t5548 = t5529 + t5543;
  t5588 = t458*t990*t1212;
  t5614 = -1.*t1156*t1253;
  t5626 = t5588 + t5614;
  t3833 = -0.3801*t3810;
  t4074 = -0.0222*t4027;
  t4142 = 0. + t3833 + t4074;
  t4339 = -0.0222*t3810;
  t4366 = 0.3801*t4027;
  t4400 = 0. + t4339 + t4366;
  t4820 = -0.8601*t4758;
  t4857 = -0.0222*t4843;
  t4863 = 0. + t4820 + t4857;
  t5676 = t2754*t5626;
  t5681 = -1.*t5548*t2828;
  t5684 = t5676 + t5681;
  t5702 = t2149*t1554*t990;
  t5705 = t5492*t1920;
  t5712 = t5702 + t5705;
  t4933 = -0.0222*t4758;
  t4957 = 0.8601*t4843;
  t4978 = 0. + t4933 + t4957;
  t5124 = -0.0211*t5077;
  t5131 = 1.3401*t5125;
  t5135 = 0. + t5124 + t5131;
  t5728 = t3766*t5684;
  t5729 = t5712*t4027;
  t5732 = t5728 + t5729;
  t5774 = t3766*t5712;
  t5776 = -1.*t5684*t4027;
  t5809 = t5774 + t5776;
  t5207 = -1.3401*t5077;
  t5216 = -0.0211*t5125;
  t5241 = 0. + t5207 + t5216;
  t5844 = -1.*t4843*t5732;
  t5854 = t4479*t5809;
  t5863 = t5844 + t5854;
  t5884 = t4479*t5732;
  t5896 = t4843*t5809;
  t5903 = t5884 + t5896;
  t6087 = t2149*t1554*t1253;
  t6089 = t1212*t1920;
  t6090 = t6087 + t6089;
  t6140 = t2149*t458*t2754;
  t6158 = -1.*t6090*t2828;
  t6168 = t6140 + t6158;
  t6209 = -1.*t1554*t1212;
  t6218 = t2149*t1253*t1920;
  t6238 = t6209 + t6218;
  t6324 = t3766*t6168;
  t6326 = t6238*t4027;
  t6334 = t6324 + t6326;
  t6344 = t3766*t6238;
  t6364 = -1.*t6168*t4027;
  t6376 = t6344 + t6364;
  t6420 = -1.*t4843*t6334;
  t6428 = t4479*t6376;
  t6445 = t6420 + t6428;
  t6492 = t4479*t6334;
  t6494 = t4843*t6376;
  t6504 = t6492 + t6494;

  p_output1(0)=0. + t1520*t1985 + t1156*t2149*t2406 + t2651*t2918 + 0.092*(t2651*t2754 + t2828*t3131) + t3131*t3463 + t3738*t4142 + t4299*t4400 + t4863*t4912 + t4978*t5020 + t5135*t5180 + t5241*t5299 - 1.325152*(t5125*t5180 + t5034*t5299) + 0.043912*(t5034*t5180 - 1.*t5125*t5299) + var1(0);
  p_output1(1)=0. + t1985*t5492 + t2918*t5548 + t3463*t5626 + 0.092*(t2754*t5548 + t2828*t5626) + t4142*t5684 + t4400*t5712 + t4863*t5732 + t4978*t5809 + t5135*t5863 + t5241*t5903 - 1.325152*(t5125*t5863 + t5034*t5903) + 0.043912*(t5034*t5863 - 1.*t5125*t5903) + t2149*t2406*t990 + var1(1);
  p_output1(2)=0. + t1253*t1985*t2149 - 1.*t1212*t2406 + t2149*t3463*t458 + t2918*t6090 + 0.092*(t2149*t2828*t458 + t2754*t6090) + t4142*t6168 + t4400*t6238 + t4863*t6334 + t4978*t6376 + t5135*t6445 + t5241*t6504 - 1.325152*(t5125*t6445 + t5034*t6504) + 0.043912*(t5034*t6445 - 1.*t5125*t6504) + var1(2);
}


       
void p_LeftFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
