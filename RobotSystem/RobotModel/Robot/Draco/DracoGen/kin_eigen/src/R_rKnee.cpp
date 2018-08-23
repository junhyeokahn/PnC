/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:27 GMT-05:00
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
  double t941;
  double t2077;
  double t3079;
  double t2783;
  double t3271;
  double t1475;
  double t2075;
  double t928;
  double t3015;
  double t3695;
  double t3817;
  double t4627;
  double t1778;
  double t4029;
  double t4196;
  double t892;
  double t4652;
  double t4764;
  double t4796;
  double t4833;
  double t4834;
  double t4850;
  double t4884;
  double t4886;
  double t4922;
  double t4948;
  double t5053;
  double t731;
  double t5322;
  double t5342;
  double t5416;
  double t5120;
  double t5318;
  double t5506;
  double t5541;
  double t5594;
  double t5595;
  double t5617;
  double t5625;
  double t5628;
  double t5652;
  double t5726;
  double t5731;
  double t5736;
  double t5791;
  double t5793;
  double t5802;
  double t5804;
  double t5805;
  double t5819;
  double t5822;
  double t5826;
  double t5841;
  double t4226;
  double t5080;
  double t5082;
  double t5142;
  double t5143;
  double t5245;
  double t5545;
  double t5739;
  double t5741;
  double t5763;
  double t5766;
  double t5767;
  double t5803;
  double t5842;
  double t5845;
  double t5878;
  double t5890;
  double t5892;
  t941 = Cos(var1[3]);
  t2077 = Cos(var1[5]);
  t3079 = Sin(var1[4]);
  t2783 = Sin(var1[3]);
  t3271 = Sin(var1[5]);
  t1475 = Cos(var1[4]);
  t2075 = Sin(var1[11]);
  t928 = Cos(var1[11]);
  t3015 = -1.*t2077*t2783;
  t3695 = t941*t3079*t3271;
  t3817 = t3015 + t3695;
  t4627 = Cos(var1[13]);
  t1778 = t928*t941*t1475;
  t4029 = t2075*t3817;
  t4196 = t1778 + t4029;
  t892 = Sin(var1[13]);
  t4652 = Cos(var1[12]);
  t4764 = t941*t2077*t3079;
  t4796 = t2783*t3271;
  t4833 = t4764 + t4796;
  t4834 = t4652*t4833;
  t4850 = Sin(var1[12]);
  t4884 = -1.*t941*t1475*t2075;
  t4886 = t928*t3817;
  t4922 = t4884 + t4886;
  t4948 = -1.*t4850*t4922;
  t5053 = t4834 + t4948;
  t731 = Sin(var1[14]);
  t5322 = t941*t2077;
  t5342 = t2783*t3079*t3271;
  t5416 = t5322 + t5342;
  t5120 = Cos(var1[14]);
  t5318 = t928*t1475*t2783;
  t5506 = t2075*t5416;
  t5541 = t5318 + t5506;
  t5594 = t2077*t2783*t3079;
  t5595 = -1.*t941*t3271;
  t5617 = t5594 + t5595;
  t5625 = t4652*t5617;
  t5628 = -1.*t1475*t2075*t2783;
  t5652 = t928*t5416;
  t5726 = t5628 + t5652;
  t5731 = -1.*t4850*t5726;
  t5736 = t5625 + t5731;
  t5791 = -1.*t928*t3079;
  t5793 = t1475*t2075*t3271;
  t5802 = t5791 + t5793;
  t5804 = t4652*t1475*t2077;
  t5805 = t2075*t3079;
  t5819 = t928*t1475*t3271;
  t5822 = t5805 + t5819;
  t5826 = -1.*t4850*t5822;
  t5841 = t5804 + t5826;
  t4226 = t892*t4196;
  t5080 = t4627*t5053;
  t5082 = t4226 + t5080;
  t5142 = t4627*t4196;
  t5143 = -1.*t892*t5053;
  t5245 = t5142 + t5143;
  t5545 = t892*t5541;
  t5739 = t4627*t5736;
  t5741 = t5545 + t5739;
  t5763 = t4627*t5541;
  t5766 = -1.*t892*t5736;
  t5767 = t5763 + t5766;
  t5803 = t892*t5802;
  t5842 = t4627*t5841;
  t5845 = t5803 + t5842;
  t5878 = t4627*t5802;
  t5890 = -1.*t892*t5841;
  t5892 = t5878 + t5890;

  p_output1(0)=t5120*t5245 - 1.*t5082*t731;
  p_output1(1)=t5120*t5767 - 1.*t5741*t731;
  p_output1(2)=t5120*t5892 - 1.*t5845*t731;
  p_output1(3)=t4833*t4850 + t4652*t4922;
  p_output1(4)=t4850*t5617 + t4652*t5726;
  p_output1(5)=t1475*t2077*t4850 + t4652*t5822;
  p_output1(6)=t5082*t5120 + t5245*t731;
  p_output1(7)=t5120*t5741 + t5767*t731;
  p_output1(8)=t5120*t5845 + t5892*t731;
}


       
void R_rKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
