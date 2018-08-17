/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipPitch.h"

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
  double t1420;
  double t2804;
  double t3446;
  double t3032;
  double t3516;
  double t1692;
  double t2480;
  double t448;
  double t3406;
  double t3759;
  double t4012;
  double t124;
  double t5126;
  double t5261;
  double t5804;
  double t6048;
  double t6052;
  double t6061;
  double t5522;
  double t5689;
  double t5769;
  double t5815;
  double t5816;
  double t5823;
  double t6211;
  double t6216;
  double t6236;
  double t6382;
  double t6393;
  double t6430;
  double t6520;
  double t6546;
  double t6552;
  double t1992;
  double t4125;
  double t4137;
  double t5793;
  double t5914;
  double t5958;
  double t6037;
  double t6141;
  double t6191;
  double t6266;
  double t6433;
  double t6450;
  double t6462;
  double t6483;
  double t6487;
  double t6517;
  double t6574;
  double t6609;
  t1420 = Cos(var1[3]);
  t2804 = Cos(var1[5]);
  t3446 = Sin(var1[4]);
  t3032 = Sin(var1[3]);
  t3516 = Sin(var1[5]);
  t1692 = Cos(var1[4]);
  t2480 = Sin(var1[11]);
  t448 = Cos(var1[11]);
  t3406 = -1.*t2804*t3032;
  t3759 = t1420*t3446*t3516;
  t4012 = t3406 + t3759;
  t124 = Cos(var1[13]);
  t5126 = Sin(var1[13]);
  t5261 = Cos(var1[12]);
  t5804 = Sin(var1[12]);
  t6048 = t1420*t2804;
  t6052 = t3032*t3446*t3516;
  t6061 = t6048 + t6052;
  t5522 = t1420*t2804*t3446;
  t5689 = t3032*t3516;
  t5769 = t5522 + t5689;
  t5815 = -1.*t1420*t1692*t2480;
  t5816 = t448*t4012;
  t5823 = t5815 + t5816;
  t6211 = t2804*t3032*t3446;
  t6216 = -1.*t1420*t3516;
  t6236 = t6211 + t6216;
  t6382 = -1.*t1692*t2480*t3032;
  t6393 = t448*t6061;
  t6430 = t6382 + t6393;
  t6520 = t2480*t3446;
  t6546 = t448*t1692*t3516;
  t6552 = t6520 + t6546;
  t1992 = t448*t1420*t1692;
  t4125 = t2480*t4012;
  t4137 = t1992 + t4125;
  t5793 = t5261*t5769;
  t5914 = -1.*t5804*t5823;
  t5958 = t5793 + t5914;
  t6037 = t448*t1692*t3032;
  t6141 = t2480*t6061;
  t6191 = t6037 + t6141;
  t6266 = t5261*t6236;
  t6433 = -1.*t5804*t6430;
  t6450 = t6266 + t6433;
  t6462 = -1.*t448*t3446;
  t6483 = t1692*t2480*t3516;
  t6487 = t6462 + t6483;
  t6517 = t5261*t1692*t2804;
  t6574 = -1.*t5804*t6552;
  t6609 = t6517 + t6574;

  p_output1(0)=t124*t4137 - 1.*t5126*t5958;
  p_output1(1)=t124*t6191 - 1.*t5126*t6450;
  p_output1(2)=t124*t6487 - 1.*t5126*t6609;
  p_output1(3)=t5769*t5804 + t5261*t5823;
  p_output1(4)=t5804*t6236 + t5261*t6430;
  p_output1(5)=t1692*t2804*t5804 + t5261*t6552;
  p_output1(6)=t4137*t5126 + t124*t5958;
  p_output1(7)=t5126*t6191 + t124*t6450;
  p_output1(8)=t5126*t6487 + t124*t6609;
}


       
void R_rHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
