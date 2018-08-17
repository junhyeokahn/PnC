/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:05 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootBack.h"

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
  double t1319;
  double t1458;
  double t1529;
  double t1499;
  double t1543;
  double t1329;
  double t1410;
  double t1288;
  double t1522;
  double t1633;
  double t1634;
  double t1728;
  double t1402;
  double t1710;
  double t1711;
  double t1041;
  double t1924;
  double t2158;
  double t2250;
  double t2308;
  double t2313;
  double t2361;
  double t2375;
  double t2455;
  double t2468;
  double t2469;
  double t2532;
  double t2578;
  double t1712;
  double t2538;
  double t2548;
  double t897;
  double t2592;
  double t2597;
  double t2658;
  double t2810;
  double t2555;
  double t2667;
  double t2720;
  double t694;
  double t2862;
  double t2876;
  double t2885;
  double t3071;
  double t3117;
  double t3196;
  double t3061;
  double t3205;
  double t3252;
  double t3377;
  double t3390;
  double t3422;
  double t3469;
  double t3475;
  double t3510;
  double t3565;
  double t3570;
  double t3639;
  double t3362;
  double t3650;
  double t3652;
  double t3688;
  double t3712;
  double t3751;
  double t3655;
  double t3800;
  double t3812;
  double t3861;
  double t3873;
  double t3880;
  double t4204;
  double t4235;
  double t4278;
  double t4335;
  double t4340;
  double t4346;
  double t4372;
  double t4388;
  double t4440;
  double t4288;
  double t4449;
  double t4450;
  double t4474;
  double t4479;
  double t4489;
  double t4457;
  double t4506;
  double t4508;
  double t4524;
  double t4561;
  double t4572;
  double t2730;
  double t2894;
  double t3816;
  double t3964;
  double t4518;
  double t4575;
  t1319 = Cos(var1[3]);
  t1458 = Cos(var1[5]);
  t1529 = Sin(var1[4]);
  t1499 = Sin(var1[3]);
  t1543 = Sin(var1[5]);
  t1329 = Cos(var1[4]);
  t1410 = Sin(var1[11]);
  t1288 = Cos(var1[11]);
  t1522 = -1.*t1458*t1499;
  t1633 = t1319*t1529*t1543;
  t1634 = t1522 + t1633;
  t1728 = Cos(var1[13]);
  t1402 = t1288*t1319*t1329;
  t1710 = t1410*t1634;
  t1711 = t1402 + t1710;
  t1041 = Sin(var1[13]);
  t1924 = Cos(var1[12]);
  t2158 = t1319*t1458*t1529;
  t2250 = t1499*t1543;
  t2308 = t2158 + t2250;
  t2313 = t1924*t2308;
  t2361 = Sin(var1[12]);
  t2375 = -1.*t1319*t1329*t1410;
  t2455 = t1288*t1634;
  t2468 = t2375 + t2455;
  t2469 = -1.*t2361*t2468;
  t2532 = t2313 + t2469;
  t2578 = Cos(var1[14]);
  t1712 = t1041*t1711;
  t2538 = t1728*t2532;
  t2548 = t1712 + t2538;
  t897 = Sin(var1[14]);
  t2592 = t1728*t1711;
  t2597 = -1.*t1041*t2532;
  t2658 = t2592 + t2597;
  t2810 = Cos(var1[15]);
  t2555 = -1.*t897*t2548;
  t2667 = t2578*t2658;
  t2720 = t2555 + t2667;
  t694 = Sin(var1[15]);
  t2862 = t2578*t2548;
  t2876 = t897*t2658;
  t2885 = t2862 + t2876;
  t3071 = t1319*t1458;
  t3117 = t1499*t1529*t1543;
  t3196 = t3071 + t3117;
  t3061 = t1288*t1329*t1499;
  t3205 = t1410*t3196;
  t3252 = t3061 + t3205;
  t3377 = t1458*t1499*t1529;
  t3390 = -1.*t1319*t1543;
  t3422 = t3377 + t3390;
  t3469 = t1924*t3422;
  t3475 = -1.*t1329*t1410*t1499;
  t3510 = t1288*t3196;
  t3565 = t3475 + t3510;
  t3570 = -1.*t2361*t3565;
  t3639 = t3469 + t3570;
  t3362 = t1041*t3252;
  t3650 = t1728*t3639;
  t3652 = t3362 + t3650;
  t3688 = t1728*t3252;
  t3712 = -1.*t1041*t3639;
  t3751 = t3688 + t3712;
  t3655 = -1.*t897*t3652;
  t3800 = t2578*t3751;
  t3812 = t3655 + t3800;
  t3861 = t2578*t3652;
  t3873 = t897*t3751;
  t3880 = t3861 + t3873;
  t4204 = -1.*t1288*t1529;
  t4235 = t1329*t1410*t1543;
  t4278 = t4204 + t4235;
  t4335 = t1924*t1329*t1458;
  t4340 = t1410*t1529;
  t4346 = t1288*t1329*t1543;
  t4372 = t4340 + t4346;
  t4388 = -1.*t2361*t4372;
  t4440 = t4335 + t4388;
  t4288 = t1041*t4278;
  t4449 = t1728*t4440;
  t4450 = t4288 + t4449;
  t4474 = t1728*t4278;
  t4479 = -1.*t1041*t4440;
  t4489 = t4474 + t4479;
  t4457 = -1.*t897*t4450;
  t4506 = t2578*t4489;
  t4508 = t4457 + t4506;
  t4524 = t2578*t4450;
  t4561 = t897*t4489;
  t4572 = t4524 + t4561;
  t2730 = t694*t2720;
  t2894 = t2810*t2885;
  t3816 = t694*t3812;
  t3964 = t2810*t3880;
  t4518 = t694*t4508;
  t4575 = t2810*t4572;

  p_output1(0)=t2730 + t2894 + 0.000796*(t2720*t2810 - 1.*t2885*t694);
  p_output1(1)=t3816 + t3964 + 0.000796*(t2810*t3812 - 1.*t3880*t694);
  p_output1(2)=t4518 + t4575 + 0.000796*(t2810*t4508 - 1.*t4572*t694);
  p_output1(3)=t2308*t2361 + t1924*t2468;
  p_output1(4)=t2361*t3422 + t1924*t3565;
  p_output1(5)=t1329*t1458*t2361 + t1924*t4372;
  p_output1(6)=-1.*t2720*t2810 + 0.000796*(t2730 + t2894) + t2885*t694;
  p_output1(7)=-1.*t2810*t3812 + 0.000796*(t3816 + t3964) + t3880*t694;
  p_output1(8)=-1.*t2810*t4508 + 0.000796*(t4518 + t4575) + t4572*t694;
}


       
void R_RightFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
