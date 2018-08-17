/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:01 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootBottom.h"

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
  double t1107;
  double t1422;
  double t1551;
  double t1483;
  double t1561;
  double t1142;
  double t1375;
  double t1006;
  double t1547;
  double t1572;
  double t1724;
  double t2126;
  double t1200;
  double t1947;
  double t2074;
  double t860;
  double t2194;
  double t2226;
  double t2232;
  double t2346;
  double t2383;
  double t2396;
  double t2485;
  double t2587;
  double t2595;
  double t2605;
  double t2640;
  double t2889;
  double t2105;
  double t2714;
  double t2816;
  double t770;
  double t2935;
  double t2959;
  double t2966;
  double t3118;
  double t2817;
  double t2983;
  double t2984;
  double t735;
  double t3138;
  double t3163;
  double t3181;
  double t3636;
  double t3669;
  double t3670;
  double t3618;
  double t3701;
  double t3743;
  double t3830;
  double t3856;
  double t3858;
  double t3874;
  double t3876;
  double t3888;
  double t3892;
  double t3918;
  double t3982;
  double t3749;
  double t4051;
  double t4208;
  double t4269;
  double t4321;
  double t4358;
  double t4253;
  double t4382;
  double t4389;
  double t4466;
  double t4494;
  double t4502;
  double t4635;
  double t4639;
  double t4647;
  double t4662;
  double t4666;
  double t4676;
  double t4682;
  double t4683;
  double t4725;
  double t4654;
  double t4734;
  double t4744;
  double t4789;
  double t4793;
  double t4797;
  double t4764;
  double t4808;
  double t4823;
  double t4915;
  double t4926;
  double t4996;
  double t3101;
  double t3305;
  double t4409;
  double t4545;
  double t4869;
  double t4999;
  t1107 = Cos(var1[3]);
  t1422 = Cos(var1[5]);
  t1551 = Sin(var1[4]);
  t1483 = Sin(var1[3]);
  t1561 = Sin(var1[5]);
  t1142 = Cos(var1[4]);
  t1375 = Sin(var1[11]);
  t1006 = Cos(var1[11]);
  t1547 = -1.*t1422*t1483;
  t1572 = t1107*t1551*t1561;
  t1724 = t1547 + t1572;
  t2126 = Cos(var1[13]);
  t1200 = t1006*t1107*t1142;
  t1947 = t1375*t1724;
  t2074 = t1200 + t1947;
  t860 = Sin(var1[13]);
  t2194 = Cos(var1[12]);
  t2226 = t1107*t1422*t1551;
  t2232 = t1483*t1561;
  t2346 = t2226 + t2232;
  t2383 = t2194*t2346;
  t2396 = Sin(var1[12]);
  t2485 = -1.*t1107*t1142*t1375;
  t2587 = t1006*t1724;
  t2595 = t2485 + t2587;
  t2605 = -1.*t2396*t2595;
  t2640 = t2383 + t2605;
  t2889 = Cos(var1[14]);
  t2105 = t860*t2074;
  t2714 = t2126*t2640;
  t2816 = t2105 + t2714;
  t770 = Sin(var1[14]);
  t2935 = t2126*t2074;
  t2959 = -1.*t860*t2640;
  t2966 = t2935 + t2959;
  t3118 = Cos(var1[15]);
  t2817 = -1.*t770*t2816;
  t2983 = t2889*t2966;
  t2984 = t2817 + t2983;
  t735 = Sin(var1[15]);
  t3138 = t2889*t2816;
  t3163 = t770*t2966;
  t3181 = t3138 + t3163;
  t3636 = t1107*t1422;
  t3669 = t1483*t1551*t1561;
  t3670 = t3636 + t3669;
  t3618 = t1006*t1142*t1483;
  t3701 = t1375*t3670;
  t3743 = t3618 + t3701;
  t3830 = t1422*t1483*t1551;
  t3856 = -1.*t1107*t1561;
  t3858 = t3830 + t3856;
  t3874 = t2194*t3858;
  t3876 = -1.*t1142*t1375*t1483;
  t3888 = t1006*t3670;
  t3892 = t3876 + t3888;
  t3918 = -1.*t2396*t3892;
  t3982 = t3874 + t3918;
  t3749 = t860*t3743;
  t4051 = t2126*t3982;
  t4208 = t3749 + t4051;
  t4269 = t2126*t3743;
  t4321 = -1.*t860*t3982;
  t4358 = t4269 + t4321;
  t4253 = -1.*t770*t4208;
  t4382 = t2889*t4358;
  t4389 = t4253 + t4382;
  t4466 = t2889*t4208;
  t4494 = t770*t4358;
  t4502 = t4466 + t4494;
  t4635 = -1.*t1006*t1551;
  t4639 = t1142*t1375*t1561;
  t4647 = t4635 + t4639;
  t4662 = t2194*t1142*t1422;
  t4666 = t1375*t1551;
  t4676 = t1006*t1142*t1561;
  t4682 = t4666 + t4676;
  t4683 = -1.*t2396*t4682;
  t4725 = t4662 + t4683;
  t4654 = t860*t4647;
  t4734 = t2126*t4725;
  t4744 = t4654 + t4734;
  t4789 = t2126*t4647;
  t4793 = -1.*t860*t4725;
  t4797 = t4789 + t4793;
  t4764 = -1.*t770*t4744;
  t4808 = t2889*t4797;
  t4823 = t4764 + t4808;
  t4915 = t2889*t4744;
  t4926 = t770*t4797;
  t4996 = t4915 + t4926;
  t3101 = t735*t2984;
  t3305 = t3118*t3181;
  t4409 = t735*t4389;
  t4545 = t3118*t4502;
  t4869 = t735*t4823;
  t4999 = t3118*t4996;

  p_output1(0)=t3101 + t3305 + 0.000796*(t2984*t3118 - 1.*t3181*t735);
  p_output1(1)=t4409 + t4545 + 0.000796*(t3118*t4389 - 1.*t4502*t735);
  p_output1(2)=t4869 + t4999 + 0.000796*(t3118*t4823 - 1.*t4996*t735);
  p_output1(3)=t2346*t2396 + t2194*t2595;
  p_output1(4)=t2396*t3858 + t2194*t3892;
  p_output1(5)=t1142*t1422*t2396 + t2194*t4682;
  p_output1(6)=-1.*t2984*t3118 + 0.000796*(t3101 + t3305) + t3181*t735;
  p_output1(7)=-1.*t3118*t4389 + 0.000796*(t4409 + t4545) + t4502*t735;
  p_output1(8)=-1.*t3118*t4823 + 0.000796*(t4869 + t4999) + t4996*t735;
}


       
void R_RightFootBottom(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
