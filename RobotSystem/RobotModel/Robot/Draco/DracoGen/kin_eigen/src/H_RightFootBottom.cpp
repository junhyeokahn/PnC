/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:01 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootBottom.h"

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
  double t333;
  double t430;
  double t536;
  double t431;
  double t570;
  double t389;
  double t425;
  double t330;
  double t474;
  double t735;
  double t753;
  double t872;
  double t411;
  double t762;
  double t783;
  double t299;
  double t928;
  double t937;
  double t1006;
  double t1107;
  double t1142;
  double t1147;
  double t1149;
  double t1200;
  double t1297;
  double t1337;
  double t1405;
  double t1452;
  double t803;
  double t1422;
  double t1432;
  double t202;
  double t1483;
  double t1518;
  double t1547;
  double t1586;
  double t1450;
  double t1551;
  double t1570;
  double t70;
  double t1608;
  double t1693;
  double t1721;
  double t2126;
  double t2194;
  double t2202;
  double t2117;
  double t2226;
  double t2338;
  double t2362;
  double t2371;
  double t2383;
  double t2396;
  double t2399;
  double t2417;
  double t2437;
  double t2448;
  double t2483;
  double t2346;
  double t2485;
  double t2486;
  double t2605;
  double t2611;
  double t2714;
  double t2547;
  double t2817;
  double t2882;
  double t2935;
  double t2966;
  double t2969;
  double t3153;
  double t3163;
  double t3179;
  double t3201;
  double t3305;
  double t3390;
  double t3394;
  double t3407;
  double t3413;
  double t3181;
  double t3436;
  double t3447;
  double t3455;
  double t3478;
  double t3493;
  double t3450;
  double t3528;
  double t3580;
  double t3602;
  double t3618;
  double t3629;
  double t1572;
  double t1724;
  double t2889;
  double t2983;
  double t3586;
  double t3632;
  double t4400;
  double t4435;
  double t4526;
  double t4540;
  double t3743;
  double t3749;
  double t3761;
  double t4687;
  double t4703;
  double t4872;
  double t4891;
  double t5130;
  double t5179;
  double t3976;
  double t1743;
  double t1758;
  double t1770;
  double t4466;
  double t4494;
  double t4497;
  double t4545;
  double t4552;
  double t4568;
  double t4634;
  double t4635;
  double t4647;
  double t4666;
  double t4676;
  double t4682;
  double t4725;
  double t4734;
  double t4744;
  double t3856;
  double t3858;
  double t3874;
  double t4797;
  double t4808;
  double t4823;
  double t4915;
  double t4926;
  double t4996;
  double t5061;
  double t5065;
  double t5117;
  double t5218;
  double t5255;
  double t5266;
  double t5284;
  double t5291;
  double t5299;
  double t4253;
  double t2984;
  double t3046;
  double t3101;
  double t3876;
  double t3888;
  double t3892;
  double t4382;
  double t3636;
  double t3651;
  double t3669;
  t333 = Cos(var1[3]);
  t430 = Cos(var1[5]);
  t536 = Sin(var1[4]);
  t431 = Sin(var1[3]);
  t570 = Sin(var1[5]);
  t389 = Cos(var1[4]);
  t425 = Sin(var1[11]);
  t330 = Cos(var1[11]);
  t474 = -1.*t430*t431;
  t735 = t333*t536*t570;
  t753 = t474 + t735;
  t872 = Cos(var1[13]);
  t411 = t330*t333*t389;
  t762 = t425*t753;
  t783 = t411 + t762;
  t299 = Sin(var1[13]);
  t928 = Cos(var1[12]);
  t937 = t333*t430*t536;
  t1006 = t431*t570;
  t1107 = t937 + t1006;
  t1142 = t928*t1107;
  t1147 = Sin(var1[12]);
  t1149 = -1.*t333*t389*t425;
  t1200 = t330*t753;
  t1297 = t1149 + t1200;
  t1337 = -1.*t1147*t1297;
  t1405 = t1142 + t1337;
  t1452 = Cos(var1[14]);
  t803 = t299*t783;
  t1422 = t872*t1405;
  t1432 = t803 + t1422;
  t202 = Sin(var1[14]);
  t1483 = t872*t783;
  t1518 = -1.*t299*t1405;
  t1547 = t1483 + t1518;
  t1586 = Cos(var1[15]);
  t1450 = -1.*t202*t1432;
  t1551 = t1452*t1547;
  t1570 = t1450 + t1551;
  t70 = Sin(var1[15]);
  t1608 = t1452*t1432;
  t1693 = t202*t1547;
  t1721 = t1608 + t1693;
  t2126 = t333*t430;
  t2194 = t431*t536*t570;
  t2202 = t2126 + t2194;
  t2117 = t330*t389*t431;
  t2226 = t425*t2202;
  t2338 = t2117 + t2226;
  t2362 = t430*t431*t536;
  t2371 = -1.*t333*t570;
  t2383 = t2362 + t2371;
  t2396 = t928*t2383;
  t2399 = -1.*t389*t425*t431;
  t2417 = t330*t2202;
  t2437 = t2399 + t2417;
  t2448 = -1.*t1147*t2437;
  t2483 = t2396 + t2448;
  t2346 = t299*t2338;
  t2485 = t872*t2483;
  t2486 = t2346 + t2485;
  t2605 = t872*t2338;
  t2611 = -1.*t299*t2483;
  t2714 = t2605 + t2611;
  t2547 = -1.*t202*t2486;
  t2817 = t1452*t2714;
  t2882 = t2547 + t2817;
  t2935 = t1452*t2486;
  t2966 = t202*t2714;
  t2969 = t2935 + t2966;
  t3153 = -1.*t330*t536;
  t3163 = t389*t425*t570;
  t3179 = t3153 + t3163;
  t3201 = t928*t389*t430;
  t3305 = t425*t536;
  t3390 = t330*t389*t570;
  t3394 = t3305 + t3390;
  t3407 = -1.*t1147*t3394;
  t3413 = t3201 + t3407;
  t3181 = t299*t3179;
  t3436 = t872*t3413;
  t3447 = t3181 + t3436;
  t3455 = t872*t3179;
  t3478 = -1.*t299*t3413;
  t3493 = t3455 + t3478;
  t3450 = -1.*t202*t3447;
  t3528 = t1452*t3493;
  t3580 = t3450 + t3528;
  t3602 = t1452*t3447;
  t3618 = t202*t3493;
  t3629 = t3602 + t3618;
  t1572 = t70*t1570;
  t1724 = t1586*t1721;
  t2889 = t70*t2882;
  t2983 = t1586*t2969;
  t3586 = t70*t3580;
  t3632 = t1586*t3629;
  t4400 = -1.*t330;
  t4435 = 1. + t4400;
  t4526 = -1.*t928;
  t4540 = 1. + t4526;
  t3743 = t1147*t1107;
  t3749 = t928*t1297;
  t3761 = t3743 + t3749;
  t4687 = -1.*t872;
  t4703 = 1. + t4687;
  t4872 = -1.*t1452;
  t4891 = 1. + t4872;
  t5130 = -1.*t1586;
  t5179 = 1. + t5130;
  t3976 = t1572 + t1724;
  t1743 = t1586*t1570;
  t1758 = -1.*t70*t1721;
  t1770 = t1743 + t1758;
  t4466 = -0.022225*t4435;
  t4494 = -0.086996*t425;
  t4497 = 0. + t4466 + t4494;
  t4545 = -0.31508*t4540;
  t4552 = 0.156996*t1147;
  t4568 = 0. + t4545 + t4552;
  t4634 = -0.086996*t4435;
  t4635 = 0.022225*t425;
  t4647 = 0. + t4634 + t4635;
  t4666 = -0.156996*t4540;
  t4676 = -0.31508*t1147;
  t4682 = 0. + t4666 + t4676;
  t4725 = -0.022225*t4703;
  t4734 = 0.38008*t299;
  t4744 = 0. + t4725 + t4734;
  t3856 = t1147*t2383;
  t3858 = t928*t2437;
  t3874 = t3856 + t3858;
  t4797 = -0.38008*t4703;
  t4808 = -0.022225*t299;
  t4823 = 0. + t4797 + t4808;
  t4915 = -0.86008*t4891;
  t4926 = -0.022225*t202;
  t4996 = 0. + t4915 + t4926;
  t5061 = -0.022225*t4891;
  t5065 = 0.86008*t202;
  t5117 = 0. + t5061 + t5065;
  t5218 = -0.021147*t5179;
  t5255 = 1.34008*t70;
  t5266 = 0. + t5218 + t5255;
  t5284 = -1.34008*t5179;
  t5291 = -0.021147*t70;
  t5299 = 0. + t5284 + t5291;
  t4253 = t2889 + t2983;
  t2984 = t1586*t2882;
  t3046 = -1.*t70*t2969;
  t3101 = t2984 + t3046;
  t3876 = t389*t430*t1147;
  t3888 = t928*t3394;
  t3892 = t3876 + t3888;
  t4382 = t3586 + t3632;
  t3636 = t1586*t3580;
  t3651 = -1.*t70*t3629;
  t3669 = t3636 + t3651;

  p_output1(0)=t1572 + t1724 + 0.000796*t1770;
  p_output1(1)=t2889 + t2983 + 0.000796*t3101;
  p_output1(2)=t3586 + t3632 + 0.000796*t3669;
  p_output1(3)=0.;
  p_output1(4)=t3761;
  p_output1(5)=t3874;
  p_output1(6)=t3892;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1570*t1586 + 0.000796*t3976 + t1721*t70;
  p_output1(9)=-1.*t1586*t2882 + 0.000796*t4253 + t2969*t70;
  p_output1(10)=-1.*t1586*t3580 + 0.000796*t4382 + t3629*t70;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043865*t1770 - 0.166996*t3761 - 1.325132*t3976 + t333*t389*t4497 + t1107*t4568 + t1297*t4682 + t1405*t4823 + t1432*t4996 + t1547*t5117 + t1570*t5266 + t1721*t5299 + t4647*t753 + t4744*t783 + var1(0);
  p_output1(13)=0. + 0.043865*t3101 - 0.166996*t3874 - 1.325132*t4253 + t389*t431*t4497 + t2383*t4568 + t2202*t4647 + t2437*t4682 + t2338*t4744 + t2483*t4823 + t2486*t4996 + t2714*t5117 + t2882*t5266 + t2969*t5299 + var1(1);
  p_output1(14)=0. + 0.043865*t3669 - 0.166996*t3892 - 1.325132*t4382 + t389*t430*t4568 + t3394*t4682 + t3179*t4744 + t3413*t4823 + t3447*t4996 + t3493*t5117 + t3580*t5266 + t3629*t5299 - 1.*t4497*t536 + t389*t4647*t570 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootBottom(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
