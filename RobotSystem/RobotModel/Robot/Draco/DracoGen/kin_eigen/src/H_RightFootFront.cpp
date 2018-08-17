/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:03 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootFront.h"

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
  double t903;
  double t999;
  double t1159;
  double t1072;
  double t1225;
  double t952;
  double t970;
  double t891;
  double t1077;
  double t1309;
  double t1311;
  double t1406;
  double t966;
  double t1316;
  double t1343;
  double t429;
  double t1437;
  double t1472;
  double t1473;
  double t1594;
  double t1662;
  double t1670;
  double t1713;
  double t1793;
  double t1814;
  double t1825;
  double t1826;
  double t1931;
  double t1361;
  double t1850;
  double t1882;
  double t197;
  double t1959;
  double t1964;
  double t2015;
  double t2177;
  double t1900;
  double t2032;
  double t2134;
  double t88;
  double t2268;
  double t2351;
  double t2372;
  double t2520;
  double t2541;
  double t2629;
  double t2509;
  double t2641;
  double t2713;
  double t2784;
  double t2798;
  double t2814;
  double t2841;
  double t2851;
  double t2861;
  double t2878;
  double t2920;
  double t2928;
  double t2726;
  double t2968;
  double t2999;
  double t3023;
  double t3053;
  double t3174;
  double t3008;
  double t3335;
  double t3359;
  double t3388;
  double t3390;
  double t3440;
  double t3587;
  double t3588;
  double t3622;
  double t3668;
  double t3676;
  double t3688;
  double t3694;
  double t3710;
  double t3720;
  double t3636;
  double t3787;
  double t3815;
  double t3845;
  double t3846;
  double t3848;
  double t3843;
  double t3890;
  double t3915;
  double t4040;
  double t4103;
  double t4140;
  double t2139;
  double t2398;
  double t3377;
  double t3516;
  double t3963;
  double t4187;
  double t4781;
  double t4841;
  double t4982;
  double t5008;
  double t4235;
  double t4240;
  double t4257;
  double t5266;
  double t5299;
  double t5423;
  double t5436;
  double t5509;
  double t5512;
  double t4408;
  double t2403;
  double t2424;
  double t2431;
  double t4852;
  double t4876;
  double t4884;
  double t5009;
  double t5011;
  double t5031;
  double t5052;
  double t5101;
  double t5124;
  double t5135;
  double t5180;
  double t5234;
  double t5312;
  double t5328;
  double t5330;
  double t4269;
  double t4295;
  double t4314;
  double t5356;
  double t5357;
  double t5389;
  double t5451;
  double t5452;
  double t5475;
  double t5482;
  double t5484;
  double t5506;
  double t5517;
  double t5525;
  double t5527;
  double t5563;
  double t5566;
  double t5574;
  double t4537;
  double t3533;
  double t3543;
  double t3551;
  double t4354;
  double t4367;
  double t4377;
  double t4749;
  double t4210;
  double t4219;
  double t4223;
  t903 = Cos(var1[3]);
  t999 = Cos(var1[5]);
  t1159 = Sin(var1[4]);
  t1072 = Sin(var1[3]);
  t1225 = Sin(var1[5]);
  t952 = Cos(var1[4]);
  t970 = Sin(var1[11]);
  t891 = Cos(var1[11]);
  t1077 = -1.*t999*t1072;
  t1309 = t903*t1159*t1225;
  t1311 = t1077 + t1309;
  t1406 = Cos(var1[13]);
  t966 = t891*t903*t952;
  t1316 = t970*t1311;
  t1343 = t966 + t1316;
  t429 = Sin(var1[13]);
  t1437 = Cos(var1[12]);
  t1472 = t903*t999*t1159;
  t1473 = t1072*t1225;
  t1594 = t1472 + t1473;
  t1662 = t1437*t1594;
  t1670 = Sin(var1[12]);
  t1713 = -1.*t903*t952*t970;
  t1793 = t891*t1311;
  t1814 = t1713 + t1793;
  t1825 = -1.*t1670*t1814;
  t1826 = t1662 + t1825;
  t1931 = Cos(var1[14]);
  t1361 = t429*t1343;
  t1850 = t1406*t1826;
  t1882 = t1361 + t1850;
  t197 = Sin(var1[14]);
  t1959 = t1406*t1343;
  t1964 = -1.*t429*t1826;
  t2015 = t1959 + t1964;
  t2177 = Cos(var1[15]);
  t1900 = -1.*t197*t1882;
  t2032 = t1931*t2015;
  t2134 = t1900 + t2032;
  t88 = Sin(var1[15]);
  t2268 = t1931*t1882;
  t2351 = t197*t2015;
  t2372 = t2268 + t2351;
  t2520 = t903*t999;
  t2541 = t1072*t1159*t1225;
  t2629 = t2520 + t2541;
  t2509 = t891*t952*t1072;
  t2641 = t970*t2629;
  t2713 = t2509 + t2641;
  t2784 = t999*t1072*t1159;
  t2798 = -1.*t903*t1225;
  t2814 = t2784 + t2798;
  t2841 = t1437*t2814;
  t2851 = -1.*t952*t970*t1072;
  t2861 = t891*t2629;
  t2878 = t2851 + t2861;
  t2920 = -1.*t1670*t2878;
  t2928 = t2841 + t2920;
  t2726 = t429*t2713;
  t2968 = t1406*t2928;
  t2999 = t2726 + t2968;
  t3023 = t1406*t2713;
  t3053 = -1.*t429*t2928;
  t3174 = t3023 + t3053;
  t3008 = -1.*t197*t2999;
  t3335 = t1931*t3174;
  t3359 = t3008 + t3335;
  t3388 = t1931*t2999;
  t3390 = t197*t3174;
  t3440 = t3388 + t3390;
  t3587 = -1.*t891*t1159;
  t3588 = t952*t970*t1225;
  t3622 = t3587 + t3588;
  t3668 = t1437*t952*t999;
  t3676 = t970*t1159;
  t3688 = t891*t952*t1225;
  t3694 = t3676 + t3688;
  t3710 = -1.*t1670*t3694;
  t3720 = t3668 + t3710;
  t3636 = t429*t3622;
  t3787 = t1406*t3720;
  t3815 = t3636 + t3787;
  t3845 = t1406*t3622;
  t3846 = -1.*t429*t3720;
  t3848 = t3845 + t3846;
  t3843 = -1.*t197*t3815;
  t3890 = t1931*t3848;
  t3915 = t3843 + t3890;
  t4040 = t1931*t3815;
  t4103 = t197*t3848;
  t4140 = t4040 + t4103;
  t2139 = t88*t2134;
  t2398 = t2177*t2372;
  t3377 = t88*t3359;
  t3516 = t2177*t3440;
  t3963 = t88*t3915;
  t4187 = t2177*t4140;
  t4781 = -1.*t891;
  t4841 = 1. + t4781;
  t4982 = -1.*t1437;
  t5008 = 1. + t4982;
  t4235 = t1670*t1594;
  t4240 = t1437*t1814;
  t4257 = t4235 + t4240;
  t5266 = -1.*t1406;
  t5299 = 1. + t5266;
  t5423 = -1.*t1931;
  t5436 = 1. + t5423;
  t5509 = -1.*t2177;
  t5512 = 1. + t5509;
  t4408 = t2139 + t2398;
  t2403 = t2177*t2134;
  t2424 = -1.*t88*t2372;
  t2431 = t2403 + t2424;
  t4852 = -0.022225*t4841;
  t4876 = -0.086996*t970;
  t4884 = 0. + t4852 + t4876;
  t5009 = -0.31508*t5008;
  t5011 = 0.156996*t1670;
  t5031 = 0. + t5009 + t5011;
  t5052 = -0.086996*t4841;
  t5101 = 0.022225*t970;
  t5124 = 0. + t5052 + t5101;
  t5135 = -0.156996*t5008;
  t5180 = -0.31508*t1670;
  t5234 = 0. + t5135 + t5180;
  t5312 = -0.022225*t5299;
  t5328 = 0.38008*t429;
  t5330 = 0. + t5312 + t5328;
  t4269 = t1670*t2814;
  t4295 = t1437*t2878;
  t4314 = t4269 + t4295;
  t5356 = -0.38008*t5299;
  t5357 = -0.022225*t429;
  t5389 = 0. + t5356 + t5357;
  t5451 = -0.86008*t5436;
  t5452 = -0.022225*t197;
  t5475 = 0. + t5451 + t5452;
  t5482 = -0.022225*t5436;
  t5484 = 0.86008*t197;
  t5506 = 0. + t5482 + t5484;
  t5517 = -0.021147*t5512;
  t5525 = 1.34008*t88;
  t5527 = 0. + t5517 + t5525;
  t5563 = -1.34008*t5512;
  t5566 = -0.021147*t88;
  t5574 = 0. + t5563 + t5566;
  t4537 = t3377 + t3516;
  t3533 = t2177*t3359;
  t3543 = -1.*t88*t3440;
  t3551 = t3533 + t3543;
  t4354 = t952*t999*t1670;
  t4367 = t1437*t3694;
  t4377 = t4354 + t4367;
  t4749 = t3963 + t4187;
  t4210 = t2177*t3915;
  t4219 = -1.*t88*t4140;
  t4223 = t4210 + t4219;

  p_output1(0)=t2139 + t2398 + 0.000796*t2431;
  p_output1(1)=t3377 + t3516 + 0.000796*t3551;
  p_output1(2)=t3963 + t4187 + 0.000796*t4223;
  p_output1(3)=0.;
  p_output1(4)=t4257;
  p_output1(5)=t4314;
  p_output1(6)=t4377;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2134*t2177 + 0.000796*t4408 + t2372*t88;
  p_output1(9)=-1.*t2177*t3359 + 0.000796*t4537 + t3440*t88;
  p_output1(10)=-1.*t2177*t3915 + 0.000796*t4749 + t4140*t88;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.043925*t2431 - 0.166996*t4257 - 1.250132*t4408 + t1594*t5031 + t1311*t5124 + t1814*t5234 + t1343*t5330 + t1826*t5389 + t1882*t5475 + t2015*t5506 + t2134*t5527 + t2372*t5574 + t4884*t903*t952 + var1(0);
  p_output1(13)=0. + 0.043925*t3551 - 0.166996*t4314 - 1.250132*t4537 + t2814*t5031 + t2629*t5124 + t2878*t5234 + t2713*t5330 + t2928*t5389 + t2999*t5475 + t3174*t5506 + t3359*t5527 + t3440*t5574 + t1072*t4884*t952 + var1(1);
  p_output1(14)=0. + 0.043925*t4223 - 0.166996*t4377 - 1.250132*t4749 - 1.*t1159*t4884 + t3694*t5234 + t3622*t5330 + t3720*t5389 + t3815*t5475 + t3848*t5506 + t3915*t5527 + t4140*t5574 + t1225*t5124*t952 + t5031*t952*t999 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
