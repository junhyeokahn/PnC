/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:09 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBack.h"

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
  double t1012;
  double t1183;
  double t999;
  double t1124;
  double t1196;
  double t2227;
  double t1557;
  double t1637;
  double t1733;
  double t1854;
  double t2228;
  double t421;
  double t2564;
  double t2583;
  double t2603;
  double t802;
  double t1129;
  double t1439;
  double t1504;
  double t1507;
  double t2158;
  double t2254;
  double t2294;
  double t2338;
  double t2349;
  double t2439;
  double t2607;
  double t2800;
  double t2517;
  double t2631;
  double t2644;
  double t413;
  double t2809;
  double t2891;
  double t2907;
  double t3090;
  double t2671;
  double t2962;
  double t2980;
  double t59;
  double t3136;
  double t3159;
  double t3292;
  double t3649;
  double t3690;
  double t3707;
  double t3954;
  double t4002;
  double t4085;
  double t3490;
  double t3509;
  double t3626;
  double t3641;
  double t3775;
  double t3843;
  double t3858;
  double t3876;
  double t3889;
  double t3928;
  double t4094;
  double t4178;
  double t4231;
  double t4240;
  double t4295;
  double t4185;
  double t4301;
  double t4309;
  double t4351;
  double t4395;
  double t4412;
  double t4908;
  double t4915;
  double t4923;
  double t4568;
  double t4732;
  double t4735;
  double t4775;
  double t4837;
  double t4844;
  double t4899;
  double t4979;
  double t5040;
  double t5054;
  double t5064;
  double t5069;
  double t5049;
  double t5078;
  double t5081;
  double t5095;
  double t5098;
  double t5116;
  double t3069;
  double t3298;
  double t4328;
  double t4414;
  double t5085;
  double t5123;
  t1012 = Cos(var1[5]);
  t1183 = Sin(var1[3]);
  t999 = Cos(var1[3]);
  t1124 = Sin(var1[4]);
  t1196 = Sin(var1[5]);
  t2227 = Cos(var1[4]);
  t1557 = Cos(var1[6]);
  t1637 = -1.*t1012*t1183;
  t1733 = t999*t1124*t1196;
  t1854 = t1637 + t1733;
  t2228 = Sin(var1[6]);
  t421 = Cos(var1[8]);
  t2564 = t999*t2227*t1557;
  t2583 = t1854*t2228;
  t2603 = t2564 + t2583;
  t802 = Cos(var1[7]);
  t1129 = t999*t1012*t1124;
  t1439 = t1183*t1196;
  t1504 = t1129 + t1439;
  t1507 = t802*t1504;
  t2158 = t1557*t1854;
  t2254 = -1.*t999*t2227*t2228;
  t2294 = t2158 + t2254;
  t2338 = Sin(var1[7]);
  t2349 = -1.*t2294*t2338;
  t2439 = t1507 + t2349;
  t2607 = Sin(var1[8]);
  t2800 = Cos(var1[9]);
  t2517 = t421*t2439;
  t2631 = t2603*t2607;
  t2644 = t2517 + t2631;
  t413 = Sin(var1[9]);
  t2809 = t421*t2603;
  t2891 = -1.*t2439*t2607;
  t2907 = t2809 + t2891;
  t3090 = Cos(var1[10]);
  t2671 = -1.*t413*t2644;
  t2962 = t2800*t2907;
  t2980 = t2671 + t2962;
  t59 = Sin(var1[10]);
  t3136 = t2800*t2644;
  t3159 = t413*t2907;
  t3292 = t3136 + t3159;
  t3649 = t999*t1012;
  t3690 = t1183*t1124*t1196;
  t3707 = t3649 + t3690;
  t3954 = t2227*t1557*t1183;
  t4002 = t3707*t2228;
  t4085 = t3954 + t4002;
  t3490 = t1012*t1183*t1124;
  t3509 = -1.*t999*t1196;
  t3626 = t3490 + t3509;
  t3641 = t802*t3626;
  t3775 = t1557*t3707;
  t3843 = -1.*t2227*t1183*t2228;
  t3858 = t3775 + t3843;
  t3876 = -1.*t3858*t2338;
  t3889 = t3641 + t3876;
  t3928 = t421*t3889;
  t4094 = t4085*t2607;
  t4178 = t3928 + t4094;
  t4231 = t421*t4085;
  t4240 = -1.*t3889*t2607;
  t4295 = t4231 + t4240;
  t4185 = -1.*t413*t4178;
  t4301 = t2800*t4295;
  t4309 = t4185 + t4301;
  t4351 = t2800*t4178;
  t4395 = t413*t4295;
  t4412 = t4351 + t4395;
  t4908 = -1.*t1557*t1124;
  t4915 = t2227*t1196*t2228;
  t4923 = t4908 + t4915;
  t4568 = t2227*t1012*t802;
  t4732 = t2227*t1557*t1196;
  t4735 = t1124*t2228;
  t4775 = t4732 + t4735;
  t4837 = -1.*t4775*t2338;
  t4844 = t4568 + t4837;
  t4899 = t421*t4844;
  t4979 = t4923*t2607;
  t5040 = t4899 + t4979;
  t5054 = t421*t4923;
  t5064 = -1.*t4844*t2607;
  t5069 = t5054 + t5064;
  t5049 = -1.*t413*t5040;
  t5078 = t2800*t5069;
  t5081 = t5049 + t5078;
  t5095 = t2800*t5040;
  t5098 = t413*t5069;
  t5116 = t5095 + t5098;
  t3069 = t59*t2980;
  t3298 = t3090*t3292;
  t4328 = t59*t4309;
  t4414 = t3090*t4412;
  t5085 = t59*t5081;
  t5123 = t3090*t5116;

  p_output1(0)=t3069 + t3298 + 0.000796*(t2980*t3090 - 1.*t3292*t59);
  p_output1(1)=t4328 + t4414 + 0.000796*(t3090*t4309 - 1.*t4412*t59);
  p_output1(2)=t5085 + t5123 + 0.000796*(t3090*t5081 - 1.*t5116*t59);
  p_output1(3)=t1504*t2338 + t2294*t802;
  p_output1(4)=t2338*t3626 + t3858*t802;
  p_output1(5)=t1012*t2227*t2338 + t4775*t802;
  p_output1(6)=-1.*t2980*t3090 + 0.000796*(t3069 + t3298) + t3292*t59;
  p_output1(7)=-1.*t3090*t4309 + 0.000796*(t4328 + t4414) + t4412*t59;
  p_output1(8)=-1.*t3090*t5081 + 0.000796*(t5085 + t5123) + t5116*t59;
}


       
void R_LeftFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
