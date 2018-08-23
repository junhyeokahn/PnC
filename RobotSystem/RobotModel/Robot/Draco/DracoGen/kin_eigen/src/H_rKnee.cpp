/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:27 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rKnee.h"

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
  double t796;
  double t851;
  double t1176;
  double t959;
  double t1186;
  double t816;
  double t848;
  double t736;
  double t1045;
  double t1247;
  double t1284;
  double t1692;
  double t818;
  double t1301;
  double t1364;
  double t602;
  double t1717;
  double t1758;
  double t1807;
  double t1842;
  double t1844;
  double t1848;
  double t1849;
  double t1891;
  double t1909;
  double t1923;
  double t1937;
  double t254;
  double t2465;
  double t2573;
  double t2612;
  double t2065;
  double t2387;
  double t2630;
  double t2661;
  double t2762;
  double t2889;
  double t2909;
  double t3015;
  double t3058;
  double t3079;
  double t3096;
  double t3107;
  double t3145;
  double t3378;
  double t3384;
  double t3520;
  double t3596;
  double t3659;
  double t3664;
  double t3679;
  double t3690;
  double t3705;
  double t1551;
  double t1944;
  double t1975;
  double t2081;
  double t2087;
  double t2104;
  double t2747;
  double t3153;
  double t3174;
  double t3252;
  double t3276;
  double t3283;
  double t3589;
  double t3776;
  double t3935;
  double t4029;
  double t4087;
  double t4154;
  double t4890;
  double t4908;
  double t5056;
  double t5079;
  double t4227;
  double t4306;
  double t4360;
  double t5429;
  double t5491;
  double t5658;
  double t5664;
  double t1979;
  double t2298;
  double t2328;
  double t4652;
  double t4654;
  double t4727;
  double t4922;
  double t4948;
  double t5000;
  double t5080;
  double t5082;
  double t5100;
  double t5142;
  double t5143;
  double t5245;
  double t5277;
  double t5318;
  double t5322;
  double t5506;
  double t5541;
  double t5545;
  double t4375;
  double t4386;
  double t4389;
  double t5617;
  double t5625;
  double t5628;
  double t5731;
  double t5736;
  double t5741;
  double t5763;
  double t5766;
  double t5767;
  double t3177;
  double t3286;
  double t3354;
  double t4749;
  double t4764;
  double t4833;
  double t4476;
  double t4486;
  double t4612;
  double t3958;
  double t4180;
  double t4196;
  double t4850;
  double t4884;
  double t4886;
  t796 = Cos(var1[3]);
  t851 = Cos(var1[5]);
  t1176 = Sin(var1[4]);
  t959 = Sin(var1[3]);
  t1186 = Sin(var1[5]);
  t816 = Cos(var1[4]);
  t848 = Sin(var1[11]);
  t736 = Cos(var1[11]);
  t1045 = -1.*t851*t959;
  t1247 = t796*t1176*t1186;
  t1284 = t1045 + t1247;
  t1692 = Cos(var1[13]);
  t818 = t736*t796*t816;
  t1301 = t848*t1284;
  t1364 = t818 + t1301;
  t602 = Sin(var1[13]);
  t1717 = Cos(var1[12]);
  t1758 = t796*t851*t1176;
  t1807 = t959*t1186;
  t1842 = t1758 + t1807;
  t1844 = t1717*t1842;
  t1848 = Sin(var1[12]);
  t1849 = -1.*t796*t816*t848;
  t1891 = t736*t1284;
  t1909 = t1849 + t1891;
  t1923 = -1.*t1848*t1909;
  t1937 = t1844 + t1923;
  t254 = Sin(var1[14]);
  t2465 = t796*t851;
  t2573 = t959*t1176*t1186;
  t2612 = t2465 + t2573;
  t2065 = Cos(var1[14]);
  t2387 = t736*t816*t959;
  t2630 = t848*t2612;
  t2661 = t2387 + t2630;
  t2762 = t851*t959*t1176;
  t2889 = -1.*t796*t1186;
  t2909 = t2762 + t2889;
  t3015 = t1717*t2909;
  t3058 = -1.*t816*t848*t959;
  t3079 = t736*t2612;
  t3096 = t3058 + t3079;
  t3107 = -1.*t1848*t3096;
  t3145 = t3015 + t3107;
  t3378 = -1.*t736*t1176;
  t3384 = t816*t848*t1186;
  t3520 = t3378 + t3384;
  t3596 = t1717*t816*t851;
  t3659 = t848*t1176;
  t3664 = t736*t816*t1186;
  t3679 = t3659 + t3664;
  t3690 = -1.*t1848*t3679;
  t3705 = t3596 + t3690;
  t1551 = t602*t1364;
  t1944 = t1692*t1937;
  t1975 = t1551 + t1944;
  t2081 = t1692*t1364;
  t2087 = -1.*t602*t1937;
  t2104 = t2081 + t2087;
  t2747 = t602*t2661;
  t3153 = t1692*t3145;
  t3174 = t2747 + t3153;
  t3252 = t1692*t2661;
  t3276 = -1.*t602*t3145;
  t3283 = t3252 + t3276;
  t3589 = t602*t3520;
  t3776 = t1692*t3705;
  t3935 = t3589 + t3776;
  t4029 = t1692*t3520;
  t4087 = -1.*t602*t3705;
  t4154 = t4029 + t4087;
  t4890 = -1.*t736;
  t4908 = 1. + t4890;
  t5056 = -1.*t1717;
  t5079 = 1. + t5056;
  t4227 = t1848*t1842;
  t4306 = t1717*t1909;
  t4360 = t4227 + t4306;
  t5429 = -1.*t1692;
  t5491 = 1. + t5429;
  t5658 = -1.*t2065;
  t5664 = 1. + t5658;
  t1979 = -1.*t254*t1975;
  t2298 = t2065*t2104;
  t2328 = t1979 + t2298;
  t4652 = t2065*t1975;
  t4654 = t254*t2104;
  t4727 = t4652 + t4654;
  t4922 = -0.0222*t4908;
  t4948 = -0.087*t848;
  t5000 = 0. + t4922 + t4948;
  t5080 = -0.3151*t5079;
  t5082 = 0.157*t1848;
  t5100 = 0. + t5080 + t5082;
  t5142 = -0.087*t4908;
  t5143 = 0.0222*t848;
  t5245 = 0. + t5142 + t5143;
  t5277 = -0.157*t5079;
  t5318 = -0.3151*t1848;
  t5322 = 0. + t5277 + t5318;
  t5506 = -0.0222*t5491;
  t5541 = 0.3801*t602;
  t5545 = 0. + t5506 + t5541;
  t4375 = t1848*t2909;
  t4386 = t1717*t3096;
  t4389 = t4375 + t4386;
  t5617 = -0.3801*t5491;
  t5625 = -0.0222*t602;
  t5628 = 0. + t5617 + t5625;
  t5731 = -0.8601*t5664;
  t5736 = -0.0222*t254;
  t5741 = 0. + t5731 + t5736;
  t5763 = -0.0222*t5664;
  t5766 = 0.8601*t254;
  t5767 = 0. + t5763 + t5766;
  t3177 = -1.*t254*t3174;
  t3286 = t2065*t3283;
  t3354 = t3177 + t3286;
  t4749 = t2065*t3174;
  t4764 = t254*t3283;
  t4833 = t4749 + t4764;
  t4476 = t816*t851*t1848;
  t4486 = t1717*t3679;
  t4612 = t4476 + t4486;
  t3958 = -1.*t254*t3935;
  t4180 = t2065*t4154;
  t4196 = t3958 + t4180;
  t4850 = t2065*t3935;
  t4884 = t254*t4154;
  t4886 = t4850 + t4884;

  p_output1(0)=t2328;
  p_output1(1)=t3354;
  p_output1(2)=t4196;
  p_output1(3)=0.;
  p_output1(4)=t4360;
  p_output1(5)=t4389;
  p_output1(6)=t4612;
  p_output1(7)=0.;
  p_output1(8)=t4727;
  p_output1(9)=t4833;
  p_output1(10)=t4886;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t2328 - 0.15025*t4360 - 0.8601*t4727 + t1842*t5100 + t1284*t5245 + t1909*t5322 + t1364*t5545 + t1937*t5628 + t1975*t5741 + t2104*t5767 + t5000*t796*t816 + var1(0);
  p_output1(13)=0. - 0.0222*t3354 - 0.15025*t4389 - 0.8601*t4833 + t2909*t5100 + t2612*t5245 + t3096*t5322 + t2661*t5545 + t3145*t5628 + t3174*t5741 + t3283*t5767 + t5000*t816*t959 + var1(1);
  p_output1(14)=0. - 0.0222*t4196 - 0.15025*t4612 - 0.8601*t4886 - 1.*t1176*t5000 + t3679*t5322 + t3520*t5545 + t3705*t5628 + t3935*t5741 + t4154*t5767 + t1186*t5245*t816 + t5100*t816*t851 + var1(2);
  p_output1(15)=1.;
}


       
void H_rKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
