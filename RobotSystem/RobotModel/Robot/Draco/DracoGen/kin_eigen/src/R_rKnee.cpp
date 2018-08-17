/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:51 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rKnee.h"

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
  double t310;
  double t745;
  double t2740;
  double t1020;
  double t2746;
  double t311;
  double t732;
  double t245;
  double t2626;
  double t3213;
  double t3522;
  double t4151;
  double t723;
  double t3572;
  double t3598;
  double t236;
  double t4336;
  double t4338;
  double t4468;
  double t4504;
  double t4582;
  double t4644;
  double t4677;
  double t4684;
  double t4685;
  double t4800;
  double t4803;
  double t222;
  double t5298;
  double t5335;
  double t5342;
  double t5153;
  double t5240;
  double t5343;
  double t5460;
  double t5492;
  double t5499;
  double t5505;
  double t5507;
  double t5516;
  double t5520;
  double t5528;
  double t5544;
  double t5555;
  double t5847;
  double t5869;
  double t5882;
  double t6021;
  double t6033;
  double t6101;
  double t6114;
  double t6161;
  double t6174;
  double t3914;
  double t4906;
  double t5067;
  double t5154;
  double t5201;
  double t5205;
  double t5461;
  double t5562;
  double t5580;
  double t5705;
  double t5747;
  double t5751;
  double t5888;
  double t6186;
  double t6217;
  double t6292;
  double t6310;
  double t6320;
  t310 = Cos(var1[3]);
  t745 = Cos(var1[5]);
  t2740 = Sin(var1[4]);
  t1020 = Sin(var1[3]);
  t2746 = Sin(var1[5]);
  t311 = Cos(var1[4]);
  t732 = Sin(var1[11]);
  t245 = Cos(var1[11]);
  t2626 = -1.*t745*t1020;
  t3213 = t310*t2740*t2746;
  t3522 = t2626 + t3213;
  t4151 = Cos(var1[13]);
  t723 = t245*t310*t311;
  t3572 = t732*t3522;
  t3598 = t723 + t3572;
  t236 = Sin(var1[13]);
  t4336 = Cos(var1[12]);
  t4338 = t310*t745*t2740;
  t4468 = t1020*t2746;
  t4504 = t4338 + t4468;
  t4582 = t4336*t4504;
  t4644 = Sin(var1[12]);
  t4677 = -1.*t310*t311*t732;
  t4684 = t245*t3522;
  t4685 = t4677 + t4684;
  t4800 = -1.*t4644*t4685;
  t4803 = t4582 + t4800;
  t222 = Sin(var1[14]);
  t5298 = t310*t745;
  t5335 = t1020*t2740*t2746;
  t5342 = t5298 + t5335;
  t5153 = Cos(var1[14]);
  t5240 = t245*t311*t1020;
  t5343 = t732*t5342;
  t5460 = t5240 + t5343;
  t5492 = t745*t1020*t2740;
  t5499 = -1.*t310*t2746;
  t5505 = t5492 + t5499;
  t5507 = t4336*t5505;
  t5516 = -1.*t311*t732*t1020;
  t5520 = t245*t5342;
  t5528 = t5516 + t5520;
  t5544 = -1.*t4644*t5528;
  t5555 = t5507 + t5544;
  t5847 = -1.*t245*t2740;
  t5869 = t311*t732*t2746;
  t5882 = t5847 + t5869;
  t6021 = t4336*t311*t745;
  t6033 = t732*t2740;
  t6101 = t245*t311*t2746;
  t6114 = t6033 + t6101;
  t6161 = -1.*t4644*t6114;
  t6174 = t6021 + t6161;
  t3914 = t236*t3598;
  t4906 = t4151*t4803;
  t5067 = t3914 + t4906;
  t5154 = t4151*t3598;
  t5201 = -1.*t236*t4803;
  t5205 = t5154 + t5201;
  t5461 = t236*t5460;
  t5562 = t4151*t5555;
  t5580 = t5461 + t5562;
  t5705 = t4151*t5460;
  t5747 = -1.*t236*t5555;
  t5751 = t5705 + t5747;
  t5888 = t236*t5882;
  t6186 = t4151*t6174;
  t6217 = t5888 + t6186;
  t6292 = t4151*t5882;
  t6310 = -1.*t236*t6174;
  t6320 = t6292 + t6310;

  p_output1(0)=-1.*t222*t5067 + t5153*t5205;
  p_output1(1)=-1.*t222*t5580 + t5153*t5751;
  p_output1(2)=-1.*t222*t6217 + t5153*t6320;
  p_output1(3)=t4504*t4644 + t4336*t4685;
  p_output1(4)=t4644*t5505 + t4336*t5528;
  p_output1(5)=t4336*t6114 + t311*t4644*t745;
  p_output1(6)=t5067*t5153 + t222*t5205;
  p_output1(7)=t5153*t5580 + t222*t5751;
  p_output1(8)=t5153*t6217 + t222*t6320;
}


       
void R_rKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
