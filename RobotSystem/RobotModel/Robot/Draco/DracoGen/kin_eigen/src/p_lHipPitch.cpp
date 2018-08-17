/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:44 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lHipPitch.h"

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
  double t150;
  double t3277;
  double t3442;
  double t3444;
  double t4721;
  double t97;
  double t115;
  double t140;
  double t813;
  double t842;
  double t900;
  double t1512;
  double t5304;
  double t5397;
  double t5400;
  double t5404;
  double t5412;
  double t5387;
  double t5392;
  double t5395;
  double t5429;
  double t5430;
  double t5438;
  double t5549;
  double t5554;
  double t5562;
  double t5565;
  double t5505;
  double t5521;
  double t5538;
  double t5585;
  double t5607;
  double t5609;
  double t4713;
  double t4957;
  double t4993;
  double t5321;
  double t5326;
  double t5349;
  double t5718;
  double t5755;
  double t5766;
  double t5410;
  double t5420;
  double t5424;
  double t5456;
  double t5460;
  double t5486;
  double t5785;
  double t5796;
  double t5797;
  double t5809;
  double t5822;
  double t5830;
  double t5563;
  double t5568;
  double t5573;
  double t5631;
  double t5636;
  double t5638;
  double t5865;
  double t5867;
  double t5891;
  double t5900;
  double t5913;
  double t5921;
  double t6071;
  double t6074;
  double t6078;
  double t6117;
  double t6119;
  double t6133;
  double t6145;
  double t6147;
  double t6152;
  t150 = Cos(var1[3]);
  t3277 = Cos(var1[6]);
  t3442 = -1.*t3277;
  t3444 = 1. + t3442;
  t4721 = Sin(var1[6]);
  t97 = Cos(var1[5]);
  t115 = Sin(var1[3]);
  t140 = -1.*t97*t115;
  t813 = Sin(var1[4]);
  t842 = Sin(var1[5]);
  t900 = t150*t813*t842;
  t1512 = t140 + t900;
  t5304 = Cos(var1[4]);
  t5397 = Cos(var1[7]);
  t5400 = -1.*t5397;
  t5404 = 1. + t5400;
  t5412 = Sin(var1[7]);
  t5387 = t3277*t1512;
  t5392 = -1.*t150*t5304*t4721;
  t5395 = t5387 + t5392;
  t5429 = t150*t97*t813;
  t5430 = t115*t842;
  t5438 = t5429 + t5430;
  t5549 = Cos(var1[8]);
  t5554 = -1.*t5549;
  t5562 = 1. + t5554;
  t5565 = Sin(var1[8]);
  t5505 = t5397*t5438;
  t5521 = -1.*t5395*t5412;
  t5538 = t5505 + t5521;
  t5585 = t150*t5304*t3277;
  t5607 = t1512*t4721;
  t5609 = t5585 + t5607;
  t4713 = 0.087004*t3444;
  t4957 = 0.022225*t4721;
  t4993 = 0. + t4713 + t4957;
  t5321 = -0.022225*t3444;
  t5326 = 0.087004*t4721;
  t5349 = 0. + t5321 + t5326;
  t5718 = t150*t97;
  t5755 = t115*t813*t842;
  t5766 = t5718 + t5755;
  t5410 = 0.157004*t5404;
  t5420 = -0.31508*t5412;
  t5424 = 0. + t5410 + t5420;
  t5456 = -0.31508*t5404;
  t5460 = -0.157004*t5412;
  t5486 = 0. + t5456 + t5460;
  t5785 = t3277*t5766;
  t5796 = -1.*t5304*t115*t4721;
  t5797 = t5785 + t5796;
  t5809 = t97*t115*t813;
  t5822 = -1.*t150*t842;
  t5830 = t5809 + t5822;
  t5563 = -0.38008*t5562;
  t5568 = -0.022225*t5565;
  t5573 = 0. + t5563 + t5568;
  t5631 = -0.022225*t5562;
  t5636 = 0.38008*t5565;
  t5638 = 0. + t5631 + t5636;
  t5865 = t5397*t5830;
  t5867 = -1.*t5797*t5412;
  t5891 = t5865 + t5867;
  t5900 = t5304*t3277*t115;
  t5913 = t5766*t4721;
  t5921 = t5900 + t5913;
  t6071 = t5304*t3277*t842;
  t6074 = t813*t4721;
  t6078 = t6071 + t6074;
  t6117 = t5304*t97*t5397;
  t6119 = -1.*t6078*t5412;
  t6133 = t6117 + t6119;
  t6145 = -1.*t3277*t813;
  t6147 = t5304*t842*t4721;
  t6152 = t6145 + t6147;

  p_output1(0)=0. + t1512*t4993 + t150*t5304*t5349 + t5395*t5424 + 0.167004*(t5395*t5397 + t5412*t5438) + t5438*t5486 + t5538*t5573 - 0.022225*(-1.*t5538*t5565 + t5549*t5609) - 0.38008*(t5538*t5549 + t5565*t5609) + t5609*t5638 + var1(0);
  p_output1(1)=0. + t115*t5304*t5349 + t4993*t5766 + t5424*t5797 + t5486*t5830 + 0.167004*(t5397*t5797 + t5412*t5830) + t5573*t5891 + t5638*t5921 - 0.022225*(-1.*t5565*t5891 + t5549*t5921) - 0.38008*(t5549*t5891 + t5565*t5921) + var1(1);
  p_output1(2)=0. + t5424*t6078 + t5573*t6133 + t5638*t6152 - 0.022225*(-1.*t5565*t6133 + t5549*t6152) - 0.38008*(t5549*t6133 + t5565*t6152) - 1.*t5349*t813 + t4993*t5304*t842 + t5304*t5486*t97 + 0.167004*(t5397*t6078 + t5304*t5412*t97) + var1(2);
}


       
void p_lHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
