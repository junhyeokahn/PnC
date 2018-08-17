/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:09 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootFront.h"

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
  double t202;
  double t777;
  double t1118;
  double t1504;
  double t1674;
  double t2809;
  double t2907;
  double t2880;
  double t2962;
  double t2158;
  double t2297;
  double t2308;
  double t2517;
  double t347;
  double t3388;
  double t3469;
  double t3641;
  double t2883;
  double t3136;
  double t3159;
  double t3824;
  double t3928;
  double t3947;
  double t3981;
  double t4002;
  double t4011;
  double t4040;
  double t4094;
  double t4106;
  double t4109;
  double t4483;
  double t4531;
  double t4560;
  double t4732;
  double t4752;
  double t4899;
  double t4923;
  double t5054;
  double t5078;
  double t5085;
  double t5170;
  double t5183;
  double t5233;
  double t5237;
  double t5305;
  double t5310;
  double t5393;
  double t5421;
  double t5425;
  double t5439;
  double t5551;
  double t5553;
  double t5632;
  double t1556;
  double t1854;
  double t2076;
  double t2427;
  double t2583;
  double t2800;
  double t3298;
  double t3311;
  double t3364;
  double t3690;
  double t3707;
  double t3775;
  double t5820;
  double t5831;
  double t5834;
  double t4024;
  double t4084;
  double t4085;
  double t5743;
  double t5788;
  double t5794;
  double t5857;
  double t5860;
  double t5894;
  double t4357;
  double t4395;
  double t4403;
  double t4915;
  double t4979;
  double t5016;
  double t5920;
  double t5935;
  double t5972;
  double t6029;
  double t6034;
  double t6035;
  double t5123;
  double t5143;
  double t5145;
  double t5369;
  double t5406;
  double t5418;
  double t6044;
  double t6045;
  double t6046;
  double t6058;
  double t6063;
  double t6082;
  double t5466;
  double t5480;
  double t5528;
  double t6103;
  double t6110;
  double t6119;
  double t6140;
  double t6150;
  double t6153;
  double t6342;
  double t6357;
  double t6368;
  double t6384;
  double t6403;
  double t6418;
  double t6454;
  double t6463;
  double t6465;
  double t6480;
  double t6490;
  double t6508;
  double t6550;
  double t6554;
  double t6555;
  double t6568;
  double t6573;
  double t6575;
  double t6590;
  double t6592;
  double t6597;
  t202 = Cos(var1[3]);
  t777 = Cos(var1[11]);
  t1118 = -1.*t777;
  t1504 = 1. + t1118;
  t1674 = Sin(var1[11]);
  t2809 = Cos(var1[5]);
  t2907 = Sin(var1[3]);
  t2880 = Sin(var1[4]);
  t2962 = Sin(var1[5]);
  t2158 = Cos(var1[12]);
  t2297 = -1.*t2158;
  t2308 = 1. + t2297;
  t2517 = Sin(var1[12]);
  t347 = Cos(var1[4]);
  t3388 = -1.*t2809*t2907;
  t3469 = t202*t2880*t2962;
  t3641 = t3388 + t3469;
  t2883 = t202*t2809*t2880;
  t3136 = t2907*t2962;
  t3159 = t2883 + t3136;
  t3824 = -1.*t202*t347*t1674;
  t3928 = t777*t3641;
  t3947 = t3824 + t3928;
  t3981 = Cos(var1[13]);
  t4002 = -1.*t3981;
  t4011 = 1. + t4002;
  t4040 = Sin(var1[13]);
  t4094 = t777*t202*t347;
  t4106 = t1674*t3641;
  t4109 = t4094 + t4106;
  t4483 = t2158*t3159;
  t4531 = -1.*t2517*t3947;
  t4560 = t4483 + t4531;
  t4732 = Cos(var1[14]);
  t4752 = -1.*t4732;
  t4899 = 1. + t4752;
  t4923 = Sin(var1[14]);
  t5054 = t4040*t4109;
  t5078 = t3981*t4560;
  t5085 = t5054 + t5078;
  t5170 = t3981*t4109;
  t5183 = -1.*t4040*t4560;
  t5233 = t5170 + t5183;
  t5237 = Cos(var1[15]);
  t5305 = -1.*t5237;
  t5310 = 1. + t5305;
  t5393 = Sin(var1[15]);
  t5421 = -1.*t4923*t5085;
  t5425 = t4732*t5233;
  t5439 = t5421 + t5425;
  t5551 = t4732*t5085;
  t5553 = t4923*t5233;
  t5632 = t5551 + t5553;
  t1556 = -0.022225*t1504;
  t1854 = -0.086996*t1674;
  t2076 = 0. + t1556 + t1854;
  t2427 = -0.31508*t2308;
  t2583 = 0.156996*t2517;
  t2800 = 0. + t2427 + t2583;
  t3298 = -0.086996*t1504;
  t3311 = 0.022225*t1674;
  t3364 = 0. + t3298 + t3311;
  t3690 = -0.156996*t2308;
  t3707 = -0.31508*t2517;
  t3775 = 0. + t3690 + t3707;
  t5820 = t202*t2809;
  t5831 = t2907*t2880*t2962;
  t5834 = t5820 + t5831;
  t4024 = -0.022225*t4011;
  t4084 = 0.38008*t4040;
  t4085 = 0. + t4024 + t4084;
  t5743 = t2809*t2907*t2880;
  t5788 = -1.*t202*t2962;
  t5794 = t5743 + t5788;
  t5857 = -1.*t347*t1674*t2907;
  t5860 = t777*t5834;
  t5894 = t5857 + t5860;
  t4357 = -0.38008*t4011;
  t4395 = -0.022225*t4040;
  t4403 = 0. + t4357 + t4395;
  t4915 = -0.86008*t4899;
  t4979 = -0.022225*t4923;
  t5016 = 0. + t4915 + t4979;
  t5920 = t777*t347*t2907;
  t5935 = t1674*t5834;
  t5972 = t5920 + t5935;
  t6029 = t2158*t5794;
  t6034 = -1.*t2517*t5894;
  t6035 = t6029 + t6034;
  t5123 = -0.022225*t4899;
  t5143 = 0.86008*t4923;
  t5145 = 0. + t5123 + t5143;
  t5369 = -0.021147*t5310;
  t5406 = 1.34008*t5393;
  t5418 = 0. + t5369 + t5406;
  t6044 = t4040*t5972;
  t6045 = t3981*t6035;
  t6046 = t6044 + t6045;
  t6058 = t3981*t5972;
  t6063 = -1.*t4040*t6035;
  t6082 = t6058 + t6063;
  t5466 = -1.34008*t5310;
  t5480 = -0.021147*t5393;
  t5528 = 0. + t5466 + t5480;
  t6103 = -1.*t4923*t6046;
  t6110 = t4732*t6082;
  t6119 = t6103 + t6110;
  t6140 = t4732*t6046;
  t6150 = t4923*t6082;
  t6153 = t6140 + t6150;
  t6342 = t1674*t2880;
  t6357 = t777*t347*t2962;
  t6368 = t6342 + t6357;
  t6384 = -1.*t777*t2880;
  t6403 = t347*t1674*t2962;
  t6418 = t6384 + t6403;
  t6454 = t2158*t347*t2809;
  t6463 = -1.*t2517*t6368;
  t6465 = t6454 + t6463;
  t6480 = t4040*t6418;
  t6490 = t3981*t6465;
  t6508 = t6480 + t6490;
  t6550 = t3981*t6418;
  t6554 = -1.*t4040*t6465;
  t6555 = t6550 + t6554;
  t6568 = -1.*t4923*t6508;
  t6573 = t4732*t6555;
  t6575 = t6568 + t6573;
  t6590 = t4732*t6508;
  t6592 = t4923*t6555;
  t6597 = t6590 + t6592;

  p_output1(0)=0. + t2800*t3159 + t202*t2076*t347 + t3364*t3641 + t3775*t3947 - 0.166996*(t2517*t3159 + t2158*t3947) + t4085*t4109 + t4403*t4560 + t5016*t5085 + t5145*t5233 + t5418*t5439 + t5528*t5632 - 1.250132*(t5393*t5439 + t5237*t5632) + 0.043925*(t5237*t5439 - 1.*t5393*t5632) + var1(0);
  p_output1(1)=0. + t2076*t2907*t347 + t2800*t5794 + t3364*t5834 + t3775*t5894 - 0.166996*(t2517*t5794 + t2158*t5894) + t4085*t5972 + t4403*t6035 + t5016*t6046 + t5145*t6082 + t5418*t6119 + t5528*t6153 - 1.250132*(t5393*t6119 + t5237*t6153) + 0.043925*(t5237*t6119 - 1.*t5393*t6153) + var1(1);
  p_output1(2)=0. - 1.*t2076*t2880 + t2800*t2809*t347 + t2962*t3364*t347 + t3775*t6368 - 0.166996*(t2517*t2809*t347 + t2158*t6368) + t4085*t6418 + t4403*t6465 + t5016*t6508 + t5145*t6555 + t5418*t6575 + t5528*t6597 - 1.250132*(t5393*t6575 + t5237*t6597) + 0.043925*(t5237*t6575 - 1.*t5393*t6597) + var1(2);
}


       
void p_RightFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
