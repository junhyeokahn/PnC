/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootBack.h"

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
  double t480;
  double t602;
  double t457;
  double t593;
  double t610;
  double t790;
  double t667;
  double t668;
  double t687;
  double t703;
  double t823;
  double t207;
  double t1143;
  double t1259;
  double t1279;
  double t279;
  double t600;
  double t633;
  double t634;
  double t649;
  double t758;
  double t848;
  double t868;
  double t901;
  double t941;
  double t1030;
  double t1289;
  double t1427;
  double t1040;
  double t1296;
  double t1415;
  double t198;
  double t1434;
  double t1449;
  double t1504;
  double t2140;
  double t1425;
  double t1534;
  double t1649;
  double t145;
  double t2155;
  double t2191;
  double t2237;
  double t2506;
  double t2526;
  double t2540;
  double t2843;
  double t2859;
  double t2877;
  double t2436;
  double t2438;
  double t2445;
  double t2474;
  double t2638;
  double t2686;
  double t2698;
  double t2711;
  double t2739;
  double t2815;
  double t2908;
  double t2915;
  double t3086;
  double t3097;
  double t3099;
  double t3044;
  double t3105;
  double t3120;
  double t3139;
  double t3179;
  double t3283;
  double t3591;
  double t3623;
  double t3625;
  double t3462;
  double t3479;
  double t3549;
  double t3554;
  double t3557;
  double t3573;
  double t3584;
  double t3626;
  double t3638;
  double t3692;
  double t3699;
  double t3701;
  double t3691;
  double t3711;
  double t3762;
  double t3794;
  double t3837;
  double t3852;
  double t2136;
  double t2247;
  double t3134;
  double t3392;
  double t3785;
  double t3878;
  double t4726;
  double t4771;
  double t4878;
  double t4882;
  double t3961;
  double t4038;
  double t4130;
  double t5033;
  double t5057;
  double t5188;
  double t5195;
  double t5371;
  double t5382;
  double t4374;
  double t2306;
  double t2324;
  double t2370;
  double t4797;
  double t4801;
  double t4808;
  double t4832;
  double t4845;
  double t4864;
  double t4909;
  double t4928;
  double t4953;
  double t4993;
  double t5002;
  double t5006;
  double t4146;
  double t4158;
  double t4175;
  double t5069;
  double t5109;
  double t5119;
  double t5139;
  double t5142;
  double t5143;
  double t5198;
  double t5218;
  double t5253;
  double t5293;
  double t5333;
  double t5356;
  double t5397;
  double t5401;
  double t5418;
  double t5435;
  double t5441;
  double t5449;
  double t4487;
  double t3424;
  double t3425;
  double t3441;
  double t4186;
  double t4194;
  double t4201;
  double t4665;
  double t3905;
  double t3916;
  double t3917;
  t480 = Cos(var1[5]);
  t602 = Sin(var1[3]);
  t457 = Cos(var1[3]);
  t593 = Sin(var1[4]);
  t610 = Sin(var1[5]);
  t790 = Cos(var1[4]);
  t667 = Cos(var1[6]);
  t668 = -1.*t480*t602;
  t687 = t457*t593*t610;
  t703 = t668 + t687;
  t823 = Sin(var1[6]);
  t207 = Cos(var1[8]);
  t1143 = t457*t790*t667;
  t1259 = t703*t823;
  t1279 = t1143 + t1259;
  t279 = Cos(var1[7]);
  t600 = t457*t480*t593;
  t633 = t602*t610;
  t634 = t600 + t633;
  t649 = t279*t634;
  t758 = t667*t703;
  t848 = -1.*t457*t790*t823;
  t868 = t758 + t848;
  t901 = Sin(var1[7]);
  t941 = -1.*t868*t901;
  t1030 = t649 + t941;
  t1289 = Sin(var1[8]);
  t1427 = Cos(var1[9]);
  t1040 = t207*t1030;
  t1296 = t1279*t1289;
  t1415 = t1040 + t1296;
  t198 = Sin(var1[9]);
  t1434 = t207*t1279;
  t1449 = -1.*t1030*t1289;
  t1504 = t1434 + t1449;
  t2140 = Cos(var1[10]);
  t1425 = -1.*t198*t1415;
  t1534 = t1427*t1504;
  t1649 = t1425 + t1534;
  t145 = Sin(var1[10]);
  t2155 = t1427*t1415;
  t2191 = t198*t1504;
  t2237 = t2155 + t2191;
  t2506 = t457*t480;
  t2526 = t602*t593*t610;
  t2540 = t2506 + t2526;
  t2843 = t790*t667*t602;
  t2859 = t2540*t823;
  t2877 = t2843 + t2859;
  t2436 = t480*t602*t593;
  t2438 = -1.*t457*t610;
  t2445 = t2436 + t2438;
  t2474 = t279*t2445;
  t2638 = t667*t2540;
  t2686 = -1.*t790*t602*t823;
  t2698 = t2638 + t2686;
  t2711 = -1.*t2698*t901;
  t2739 = t2474 + t2711;
  t2815 = t207*t2739;
  t2908 = t2877*t1289;
  t2915 = t2815 + t2908;
  t3086 = t207*t2877;
  t3097 = -1.*t2739*t1289;
  t3099 = t3086 + t3097;
  t3044 = -1.*t198*t2915;
  t3105 = t1427*t3099;
  t3120 = t3044 + t3105;
  t3139 = t1427*t2915;
  t3179 = t198*t3099;
  t3283 = t3139 + t3179;
  t3591 = -1.*t667*t593;
  t3623 = t790*t610*t823;
  t3625 = t3591 + t3623;
  t3462 = t790*t480*t279;
  t3479 = t790*t667*t610;
  t3549 = t593*t823;
  t3554 = t3479 + t3549;
  t3557 = -1.*t3554*t901;
  t3573 = t3462 + t3557;
  t3584 = t207*t3573;
  t3626 = t3625*t1289;
  t3638 = t3584 + t3626;
  t3692 = t207*t3625;
  t3699 = -1.*t3573*t1289;
  t3701 = t3692 + t3699;
  t3691 = -1.*t198*t3638;
  t3711 = t1427*t3701;
  t3762 = t3691 + t3711;
  t3794 = t1427*t3638;
  t3837 = t198*t3701;
  t3852 = t3794 + t3837;
  t2136 = t145*t1649;
  t2247 = t2140*t2237;
  t3134 = t145*t3120;
  t3392 = t2140*t3283;
  t3785 = t145*t3762;
  t3878 = t2140*t3852;
  t4726 = -1.*t667;
  t4771 = 1. + t4726;
  t4878 = -1.*t279;
  t4882 = 1. + t4878;
  t3961 = t279*t868;
  t4038 = t634*t901;
  t4130 = t3961 + t4038;
  t5033 = -1.*t207;
  t5057 = 1. + t5033;
  t5188 = -1.*t1427;
  t5195 = 1. + t5188;
  t5371 = -1.*t2140;
  t5382 = 1. + t5371;
  t4374 = t2136 + t2247;
  t2306 = t2140*t1649;
  t2324 = -1.*t145*t2237;
  t2370 = t2306 + t2324;
  t4797 = 0.087004*t4771;
  t4801 = 0.022225*t823;
  t4808 = 0. + t4797 + t4801;
  t4832 = -0.022225*t4771;
  t4845 = 0.087004*t823;
  t4864 = 0. + t4832 + t4845;
  t4909 = 0.157004*t4882;
  t4928 = -0.31508*t901;
  t4953 = 0. + t4909 + t4928;
  t4993 = -0.31508*t4882;
  t5002 = -0.157004*t901;
  t5006 = 0. + t4993 + t5002;
  t4146 = t279*t2698;
  t4158 = t2445*t901;
  t4175 = t4146 + t4158;
  t5069 = -0.38008*t5057;
  t5109 = -0.022225*t1289;
  t5119 = 0. + t5069 + t5109;
  t5139 = -0.022225*t5057;
  t5142 = 0.38008*t1289;
  t5143 = 0. + t5139 + t5142;
  t5198 = -0.86008*t5195;
  t5218 = -0.022225*t198;
  t5253 = 0. + t5198 + t5218;
  t5293 = -0.022225*t5195;
  t5333 = 0.86008*t198;
  t5356 = 0. + t5293 + t5333;
  t5397 = -0.021147*t5382;
  t5401 = 1.34008*t145;
  t5418 = 0. + t5397 + t5401;
  t5435 = -1.34008*t5382;
  t5441 = -0.021147*t145;
  t5449 = 0. + t5435 + t5441;
  t4487 = t3134 + t3392;
  t3424 = t2140*t3120;
  t3425 = -1.*t145*t3283;
  t3441 = t3424 + t3425;
  t4186 = t279*t3554;
  t4194 = t790*t480*t901;
  t4201 = t4186 + t4194;
  t4665 = t3785 + t3878;
  t3905 = t2140*t3762;
  t3916 = -1.*t145*t3852;
  t3917 = t3905 + t3916;

  p_output1(0)=t2136 + t2247 + 0.000796*t2370;
  p_output1(1)=t3134 + t3392 + 0.000796*t3441;
  p_output1(2)=t3785 + t3878 + 0.000796*t3917;
  p_output1(3)=0.;
  p_output1(4)=t4130;
  p_output1(5)=t4175;
  p_output1(6)=t4201;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1649*t2140 + t145*t2237 + 0.000796*t4374;
  p_output1(9)=-1.*t2140*t3120 + t145*t3283 + 0.000796*t4487;
  p_output1(10)=-1.*t2140*t3762 + t145*t3852 + 0.000796*t4665;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043805*t2370 + 0.167004*t4130 - 1.400132*t4374 + t1030*t5119 + t1279*t5143 + t1415*t5253 + t1504*t5356 + t1649*t5418 + t2237*t5449 + t5006*t634 + t4808*t703 + t457*t4864*t790 + t4953*t868 + var1(0);
  p_output1(13)=0. + 0.043805*t3441 + 0.167004*t4175 - 1.400132*t4487 + t2540*t4808 + t2698*t4953 + t2445*t5006 + t2739*t5119 + t2877*t5143 + t2915*t5253 + t3099*t5356 + t3120*t5418 + t3283*t5449 + t4864*t602*t790 + var1(1);
  p_output1(14)=0. + 0.043805*t3917 + 0.167004*t4201 - 1.400132*t4665 + t3554*t4953 + t3573*t5119 + t3625*t5143 + t3638*t5253 + t3701*t5356 + t3762*t5418 + t3852*t5449 - 1.*t4864*t593 + t480*t5006*t790 + t4808*t610*t790 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
