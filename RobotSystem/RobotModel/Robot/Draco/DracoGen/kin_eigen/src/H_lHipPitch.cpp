/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:46 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipPitch.h"

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
  double t465;
  double t1017;
  double t1143;
  double t1048;
  double t1182;
  double t672;
  double t1055;
  double t1187;
  double t1237;
  double t646;
  double t1326;
  double t375;
  double t1523;
  double t3011;
  double t3022;
  double t3135;
  double t2579;
  double t2753;
  double t2026;
  double t2204;
  double t2519;
  double t1538;
  double t1560;
  double t1783;
  double t3447;
  double t3569;
  double t3603;
  double t3386;
  double t3398;
  double t3419;
  double t4156;
  double t4178;
  double t4187;
  double t1902;
  double t2725;
  double t2750;
  double t787;
  double t1373;
  double t1444;
  double t3431;
  double t3615;
  double t3636;
  double t2848;
  double t3246;
  double t3323;
  double t4152;
  double t4194;
  double t4358;
  double t3709;
  double t3825;
  double t3830;
  double t5460;
  double t5462;
  double t5638;
  double t5654;
  double t4609;
  double t4624;
  double t4730;
  double t5767;
  double t5785;
  double t5147;
  double t5188;
  double t5321;
  double t1454;
  double t2771;
  double t2817;
  double t5474;
  double t5489;
  double t5490;
  double t5573;
  double t5585;
  double t5594;
  double t5671;
  double t5673;
  double t5692;
  double t5714;
  double t5717;
  double t5718;
  double t4772;
  double t4790;
  double t4853;
  double t5809;
  double t5830;
  double t5832;
  double t5858;
  double t5865;
  double t5900;
  double t5354;
  double t5387;
  double t5410;
  double t3366;
  double t3651;
  double t3697;
  double t4934;
  double t5028;
  double t5088;
  double t5429;
  double t5430;
  double t5454;
  double t4023;
  double t4448;
  double t4490;
  t465 = Cos(var1[3]);
  t1017 = Cos(var1[5]);
  t1143 = Sin(var1[4]);
  t1048 = Sin(var1[3]);
  t1182 = Sin(var1[5]);
  t672 = Cos(var1[6]);
  t1055 = -1.*t1017*t1048;
  t1187 = t465*t1143*t1182;
  t1237 = t1055 + t1187;
  t646 = Cos(var1[4]);
  t1326 = Sin(var1[6]);
  t375 = Cos(var1[8]);
  t1523 = Cos(var1[7]);
  t3011 = t465*t1017;
  t3022 = t1048*t1143*t1182;
  t3135 = t3011 + t3022;
  t2579 = Sin(var1[7]);
  t2753 = Sin(var1[8]);
  t2026 = t672*t1237;
  t2204 = -1.*t465*t646*t1326;
  t2519 = t2026 + t2204;
  t1538 = t465*t1017*t1143;
  t1560 = t1048*t1182;
  t1783 = t1538 + t1560;
  t3447 = t672*t3135;
  t3569 = -1.*t646*t1048*t1326;
  t3603 = t3447 + t3569;
  t3386 = t1017*t1048*t1143;
  t3398 = -1.*t465*t1182;
  t3419 = t3386 + t3398;
  t4156 = t646*t672*t1182;
  t4178 = t1143*t1326;
  t4187 = t4156 + t4178;
  t1902 = t1523*t1783;
  t2725 = -1.*t2519*t2579;
  t2750 = t1902 + t2725;
  t787 = t465*t646*t672;
  t1373 = t1237*t1326;
  t1444 = t787 + t1373;
  t3431 = t1523*t3419;
  t3615 = -1.*t3603*t2579;
  t3636 = t3431 + t3615;
  t2848 = t646*t672*t1048;
  t3246 = t3135*t1326;
  t3323 = t2848 + t3246;
  t4152 = t646*t1017*t1523;
  t4194 = -1.*t4187*t2579;
  t4358 = t4152 + t4194;
  t3709 = -1.*t672*t1143;
  t3825 = t646*t1182*t1326;
  t3830 = t3709 + t3825;
  t5460 = -1.*t672;
  t5462 = 1. + t5460;
  t5638 = -1.*t1523;
  t5654 = 1. + t5638;
  t4609 = t1523*t2519;
  t4624 = t1783*t2579;
  t4730 = t4609 + t4624;
  t5767 = -1.*t375;
  t5785 = 1. + t5767;
  t5147 = t375*t2750;
  t5188 = t1444*t2753;
  t5321 = t5147 + t5188;
  t1454 = t375*t1444;
  t2771 = -1.*t2750*t2753;
  t2817 = t1454 + t2771;
  t5474 = 0.087004*t5462;
  t5489 = 0.022225*t1326;
  t5490 = 0. + t5474 + t5489;
  t5573 = -0.022225*t5462;
  t5585 = 0.087004*t1326;
  t5594 = 0. + t5573 + t5585;
  t5671 = 0.157004*t5654;
  t5673 = -0.31508*t2579;
  t5692 = 0. + t5671 + t5673;
  t5714 = -0.31508*t5654;
  t5717 = -0.157004*t2579;
  t5718 = 0. + t5714 + t5717;
  t4772 = t1523*t3603;
  t4790 = t3419*t2579;
  t4853 = t4772 + t4790;
  t5809 = -0.38008*t5785;
  t5830 = -0.022225*t2753;
  t5832 = 0. + t5809 + t5830;
  t5858 = -0.022225*t5785;
  t5865 = 0.38008*t2753;
  t5900 = 0. + t5858 + t5865;
  t5354 = t375*t3636;
  t5387 = t3323*t2753;
  t5410 = t5354 + t5387;
  t3366 = t375*t3323;
  t3651 = -1.*t3636*t2753;
  t3697 = t3366 + t3651;
  t4934 = t1523*t4187;
  t5028 = t646*t1017*t2579;
  t5088 = t4934 + t5028;
  t5429 = t375*t4358;
  t5430 = t3830*t2753;
  t5454 = t5429 + t5430;
  t4023 = t375*t3830;
  t4448 = -1.*t4358*t2753;
  t4490 = t4023 + t4448;

  p_output1(0)=t2817;
  p_output1(1)=t3697;
  p_output1(2)=t4490;
  p_output1(3)=0.;
  p_output1(4)=t4730;
  p_output1(5)=t4853;
  p_output1(6)=t5088;
  p_output1(7)=0.;
  p_output1(8)=t5321;
  p_output1(9)=t5410;
  p_output1(10)=t5454;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t2817 + 0.167004*t4730 - 0.38008*t5321 + t1237*t5490 + t2519*t5692 + t1783*t5718 + t2750*t5832 + t1444*t5900 + t465*t5594*t646 + var1(0);
  p_output1(13)=0. - 0.022225*t3697 + 0.167004*t4853 - 0.38008*t5410 + t3135*t5490 + t3603*t5692 + t3419*t5718 + t3636*t5832 + t3323*t5900 + t1048*t5594*t646 + var1(1);
  p_output1(14)=0. - 0.022225*t4490 + 0.167004*t5088 - 0.38008*t5454 - 1.*t1143*t5594 + t4187*t5692 + t4358*t5832 + t3830*t5900 + t1182*t5490*t646 + t1017*t5718*t646 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
