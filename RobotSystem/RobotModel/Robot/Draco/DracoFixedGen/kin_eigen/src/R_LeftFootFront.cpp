/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootFront.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t2759;
  double t1242;
  double t1978;
  double t2108;
  double t3032;
  double t1101;
  double t3248;
  double t3263;
  double t3267;
  double t2650;
  double t3087;
  double t3187;
  double t3290;
  double t900;
  double t3339;
  double t3348;
  double t3354;
  double t3200;
  double t3301;
  double t3319;
  double t3375;
  double t3471;
  double t3473;
  double t3479;
  double t3432;
  double t3433;
  double t3438;
  double t3519;
  double t3528;
  double t3530;
  double t3462;
  double t3484;
  double t3488;
  double t3554;
  double t3556;
  double t3569;
  double t3570;
  double t3565;
  double t3567;
  double t3583;
  double t3584;
  double t3589;
  double t3568;
  double t3574;
  double t3576;
  double t3328;
  double t3381;
  double t3516;
  double t3532;
  double t3577;
  double t3598;
  t2759 = Cos(var1[0]);
  t1242 = Cos(var1[2]);
  t1978 = Sin(var1[0]);
  t2108 = Sin(var1[1]);
  t3032 = Sin(var1[2]);
  t1101 = Cos(var1[3]);
  t3248 = t2759*t1242;
  t3263 = -1.*t1978*t2108*t3032;
  t3267 = t3248 + t3263;
  t2650 = t1242*t1978*t2108;
  t3087 = t2759*t3032;
  t3187 = t2650 + t3087;
  t3290 = Sin(var1[3]);
  t900 = Cos(var1[4]);
  t3339 = t1101*t3267;
  t3348 = -1.*t3187*t3290;
  t3354 = t3339 + t3348;
  t3200 = t1101*t3187;
  t3301 = t3267*t3290;
  t3319 = t3200 + t3301;
  t3375 = Sin(var1[4]);
  t3471 = t1242*t1978;
  t3473 = t2759*t2108*t3032;
  t3479 = t3471 + t3473;
  t3432 = -1.*t2759*t1242*t2108;
  t3433 = t1978*t3032;
  t3438 = t3432 + t3433;
  t3519 = t1101*t3479;
  t3528 = -1.*t3438*t3290;
  t3530 = t3519 + t3528;
  t3462 = t1101*t3438;
  t3484 = t3479*t3290;
  t3488 = t3462 + t3484;
  t3554 = Cos(var1[1]);
  t3556 = 0. + t3554;
  t3569 = -1.*t3556*t3032;
  t3570 = 0. + t3569;
  t3565 = t3556*t1242;
  t3567 = 0. + t3565;
  t3583 = t1101*t3570;
  t3584 = -1.*t3567*t3290;
  t3589 = t3583 + t3584;
  t3568 = t3567*t1101;
  t3574 = t3570*t3290;
  t3576 = t3568 + t3574;
  t3328 = t900*t3319;
  t3381 = t3354*t3375;
  t3516 = t900*t3488;
  t3532 = t3530*t3375;
  t3577 = t900*t3576;
  t3598 = t3589*t3375;

  p_output1(0)=t3328 + t3381 + 0.000796*(-1.*t3319*t3375 + t3354*t900);
  p_output1(1)=t3516 + t3532 + 0.000796*(-1.*t3375*t3488 + t3530*t900);
  p_output1(2)=t3577 + t3598 + 0.000796*(-1.*t3375*t3576 + t3589*t900);
  p_output1(3)=-1.*t1978*t3554;
  p_output1(4)=t2759*t3554;
  p_output1(5)=0. + t2108;
  p_output1(6)=t3319*t3375 + 0.000796*(t3328 + t3381) - 1.*t3354*t900;
  p_output1(7)=t3375*t3488 + 0.000796*(t3516 + t3532) - 1.*t3530*t900;
  p_output1(8)=t3375*t3576 + 0.000796*(t3577 + t3598) - 1.*t3589*t900;
}


       
void R_LeftFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
