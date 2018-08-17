/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:36 GMT-05:00
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
  double t328;
  double t740;
  double t951;
  double t1018;
  double t1142;
  double t159;
  double t165;
  double t167;
  double t352;
  double t558;
  double t560;
  double t695;
  double t2417;
  double t4671;
  double t4741;
  double t4943;
  double t5310;
  double t4147;
  double t4637;
  double t4646;
  double t5651;
  double t5677;
  double t5687;
  double t5757;
  double t5761;
  double t5781;
  double t5791;
  double t5746;
  double t5749;
  double t5756;
  double t5820;
  double t5821;
  double t5822;
  double t1019;
  double t1560;
  double t1880;
  double t2518;
  double t2606;
  double t3788;
  double t5892;
  double t5894;
  double t5895;
  double t5202;
  double t5315;
  double t5494;
  double t5703;
  double t5713;
  double t5720;
  double t5926;
  double t5938;
  double t5939;
  double t5944;
  double t5947;
  double t5951;
  double t5788;
  double t5796;
  double t5802;
  double t5826;
  double t5828;
  double t5833;
  double t5992;
  double t6001;
  double t6009;
  double t6036;
  double t6047;
  double t6048;
  double t6134;
  double t6138;
  double t6143;
  double t6185;
  double t6207;
  double t6219;
  double t6226;
  double t6232;
  double t6234;
  t328 = Cos(var1[3]);
  t740 = Cos(var1[6]);
  t951 = -1.*t740;
  t1018 = 1. + t951;
  t1142 = Sin(var1[6]);
  t159 = Cos(var1[5]);
  t165 = Sin(var1[3]);
  t167 = -1.*t159*t165;
  t352 = Sin(var1[4]);
  t558 = Sin(var1[5]);
  t560 = t328*t352*t558;
  t695 = t167 + t560;
  t2417 = Cos(var1[4]);
  t4671 = Cos(var1[7]);
  t4741 = -1.*t4671;
  t4943 = 1. + t4741;
  t5310 = Sin(var1[7]);
  t4147 = t740*t695;
  t4637 = -1.*t328*t2417*t1142;
  t4646 = t4147 + t4637;
  t5651 = t328*t159*t352;
  t5677 = t165*t558;
  t5687 = t5651 + t5677;
  t5757 = Cos(var1[8]);
  t5761 = -1.*t5757;
  t5781 = 1. + t5761;
  t5791 = Sin(var1[8]);
  t5746 = t4671*t5687;
  t5749 = -1.*t4646*t5310;
  t5756 = t5746 + t5749;
  t5820 = t328*t2417*t740;
  t5821 = t695*t1142;
  t5822 = t5820 + t5821;
  t1019 = 0.087004*t1018;
  t1560 = 0.022225*t1142;
  t1880 = 0. + t1019 + t1560;
  t2518 = -0.022225*t1018;
  t2606 = 0.087004*t1142;
  t3788 = 0. + t2518 + t2606;
  t5892 = t328*t159;
  t5894 = t165*t352*t558;
  t5895 = t5892 + t5894;
  t5202 = 0.157004*t4943;
  t5315 = -0.31508*t5310;
  t5494 = 0. + t5202 + t5315;
  t5703 = -0.31508*t4943;
  t5713 = -0.157004*t5310;
  t5720 = 0. + t5703 + t5713;
  t5926 = t740*t5895;
  t5938 = -1.*t2417*t165*t1142;
  t5939 = t5926 + t5938;
  t5944 = t159*t165*t352;
  t5947 = -1.*t328*t558;
  t5951 = t5944 + t5947;
  t5788 = -0.38008*t5781;
  t5796 = -0.022225*t5791;
  t5802 = 0. + t5788 + t5796;
  t5826 = -0.022225*t5781;
  t5828 = 0.38008*t5791;
  t5833 = 0. + t5826 + t5828;
  t5992 = t4671*t5951;
  t6001 = -1.*t5939*t5310;
  t6009 = t5992 + t6001;
  t6036 = t2417*t740*t165;
  t6047 = t5895*t1142;
  t6048 = t6036 + t6047;
  t6134 = t2417*t740*t558;
  t6138 = t352*t1142;
  t6143 = t6134 + t6138;
  t6185 = t2417*t159*t4671;
  t6207 = -1.*t6143*t5310;
  t6219 = t6185 + t6207;
  t6226 = -1.*t740*t352;
  t6232 = t2417*t558*t1142;
  t6234 = t6226 + t6232;

  p_output1(0)=0. + t2417*t328*t3788 + t4646*t5494 + 0.167004*(t4646*t4671 + t5310*t5687) + t5687*t5720 + t5756*t5802 - 0.022225*(-1.*t5756*t5791 + t5757*t5822) - 0.38008*(t5756*t5757 + t5791*t5822) + t5822*t5833 + t1880*t695 + var1(0);
  p_output1(1)=0. + t165*t2417*t3788 + t1880*t5895 + t5494*t5939 + t5720*t5951 + 0.167004*(t4671*t5939 + t5310*t5951) + t5802*t6009 + t5833*t6048 - 0.022225*(-1.*t5791*t6009 + t5757*t6048) - 0.38008*(t5757*t6009 + t5791*t6048) + var1(1);
  p_output1(2)=0. - 1.*t352*t3788 + t1880*t2417*t558 + t159*t2417*t5720 + t5494*t6143 + 0.167004*(t159*t2417*t5310 + t4671*t6143) + t5802*t6219 + t5833*t6234 - 0.022225*(-1.*t5791*t6219 + t5757*t6234) - 0.38008*(t5757*t6219 + t5791*t6234) + var1(2);
}


       
void p_lHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
