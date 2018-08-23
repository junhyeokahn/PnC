/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:23 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t509;
  double t43;
  double t47;
  double t50;
  double t54;
  double t629;
  double t622;
  double t623;
  double t631;
  double t553;
  double t604;
  double t607;
  double t611;
  double t41;
  double t1725;
  double t1855;
  double t1885;
  double t624;
  double t798;
  double t843;
  double t2266;
  double t2487;
  double t2496;
  double t52;
  double t64;
  double t473;
  double t608;
  double t618;
  double t619;
  double t969;
  double t1288;
  double t1469;
  double t1949;
  double t1999;
  double t2011;
  double t3524;
  double t3537;
  double t3538;
  double t3281;
  double t3299;
  double t3389;
  double t3712;
  double t3740;
  double t3748;
  double t3857;
  double t3858;
  double t3860;
  double t4167;
  double t4172;
  double t4197;
  double t4485;
  double t4512;
  double t4545;
  double t4871;
  double t4875;
  double t4878;
  double t4952;
  double t4955;
  double t4962;
  double t5102;
  double t5104;
  double t5105;
  double t5078;
  double t5088;
  double t5090;
  double t5094;
  double t5095;
  double t5126;
  double t5127;
  double t5128;
  double t2531;
  double t5153;
  double t5156;
  double t5186;
  double t5203;
  double t5212;
  double t3843;
  double t3844;
  double t3845;
  double t5242;
  double t5250;
  double t5258;
  double t5259;
  double t5283;
  double t5133;
  double t5134;
  double t5138;
  double t5172;
  double t5174;
  double t5176;
  t509 = Sin(var1[3]);
  t43 = Cos(var1[11]);
  t47 = -1.*t43;
  t50 = 1. + t47;
  t54 = Sin(var1[11]);
  t629 = Cos(var1[3]);
  t622 = Cos(var1[5]);
  t623 = Sin(var1[4]);
  t631 = Sin(var1[5]);
  t553 = Cos(var1[12]);
  t604 = -1.*t553;
  t607 = 1. + t604;
  t611 = Sin(var1[12]);
  t41 = Cos(var1[4]);
  t1725 = -1.*t629*t622;
  t1855 = -1.*t509*t623*t631;
  t1885 = t1725 + t1855;
  t624 = -1.*t622*t509*t623;
  t798 = t629*t631;
  t843 = t624 + t798;
  t2266 = t41*t54*t509;
  t2487 = t43*t1885;
  t2496 = t2266 + t2487;
  t52 = -0.0222*t50;
  t64 = -0.087*t54;
  t473 = 0. + t52 + t64;
  t608 = -0.3151*t607;
  t618 = 0.157*t611;
  t619 = 0. + t608 + t618;
  t969 = -0.087*t50;
  t1288 = 0.0222*t54;
  t1469 = 0. + t969 + t1288;
  t1949 = -0.157*t607;
  t1999 = -0.3151*t611;
  t2011 = 0. + t1949 + t1999;
  t3524 = -1.*t622*t509;
  t3537 = t629*t623*t631;
  t3538 = t3524 + t3537;
  t3281 = t629*t622*t623;
  t3299 = t509*t631;
  t3389 = t3281 + t3299;
  t3712 = -1.*t629*t41*t54;
  t3740 = t43*t3538;
  t3748 = t3712 + t3740;
  t3857 = t629*t54*t623;
  t3858 = t43*t629*t41*t631;
  t3860 = t3857 + t3858;
  t4167 = t54*t509*t623;
  t4172 = t43*t41*t509*t631;
  t4197 = t4167 + t4172;
  t4485 = t41*t54;
  t4512 = -1.*t43*t623*t631;
  t4545 = t4485 + t4512;
  t4871 = t622*t509;
  t4875 = -1.*t629*t623*t631;
  t4878 = t4871 + t4875;
  t4952 = t622*t509*t623;
  t4955 = -1.*t629*t631;
  t4962 = t4952 + t4955;
  t5102 = -1.*t43*t629*t41;
  t5104 = -1.*t54*t3538;
  t5105 = t5102 + t5104;
  t5078 = -0.087*t43;
  t5088 = -0.0222*t54;
  t5090 = t5078 + t5088;
  t5094 = 0.0222*t43;
  t5095 = t5094 + t64;
  t5126 = t629*t622;
  t5127 = t509*t623*t631;
  t5128 = t5126 + t5127;
  t2531 = -1.*t43*t41*t509;
  t5153 = -1.*t54*t5128;
  t5156 = t2531 + t5153;
  t5186 = t43*t623;
  t5203 = -1.*t41*t54*t631;
  t5212 = t5186 + t5203;
  t3843 = t553*t3389;
  t3844 = -1.*t611*t3748;
  t3845 = t3843 + t3844;
  t5242 = 0.157*t553;
  t5250 = t5242 + t1999;
  t5258 = -0.3151*t553;
  t5259 = -0.157*t611;
  t5283 = t5258 + t5259;
  t5133 = -1.*t41*t54*t509;
  t5134 = t43*t5128;
  t5138 = t5133 + t5134;
  t5172 = t54*t623;
  t5174 = t43*t41*t631;
  t5176 = t5172 + t5174;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1469*t1885 + t2011*t2496 - 1.*t41*t473*t509 - 0.0222*(t2531 + t1885*t54) + t619*t843 - 0.3151*(-1.*t2496*t611 + t553*t843) - 0.157*(t2496*t553 + t611*t843);
  p_output1(10)=t1469*t3538 + t2011*t3748 - 0.3151*t3845 - 0.157*(t3748*t553 + t3389*t611) + t3389*t619 + t41*t473*t629 - 0.0222*(t3538*t54 + t41*t43*t629);
  p_output1(11)=0;
  p_output1(12)=t2011*t3860 + t41*t619*t622*t629 - 1.*t473*t623*t629 - 0.3151*(-1.*t3860*t611 + t41*t553*t622*t629) - 0.157*(t3860*t553 + t41*t611*t622*t629) + t1469*t41*t629*t631 - 0.0222*(-1.*t43*t623*t629 + t41*t54*t629*t631);
  p_output1(13)=t2011*t4197 + t41*t509*t619*t622 - 0.3151*(-1.*t4197*t611 + t41*t509*t553*t622) - 0.157*(t4197*t553 + t41*t509*t611*t622) - 1.*t473*t509*t623 + t1469*t41*t509*t631 - 0.0222*(-1.*t43*t509*t623 + t41*t509*t54*t631);
  p_output1(14)=t2011*t4545 - 1.*t41*t473 - 1.*t619*t622*t623 - 0.3151*(-1.*t4545*t611 - 1.*t553*t622*t623) - 0.157*(t4545*t553 - 1.*t611*t622*t623) - 1.*t1469*t623*t631 - 0.0222*(-1.*t41*t43 - 1.*t54*t623*t631);
  p_output1(15)=t1469*t3389 + t2011*t3389*t43 - 0.0222*t3389*t54 - 0.3151*(t4878*t553 - 1.*t3389*t43*t611) - 0.157*(t3389*t43*t553 + t4878*t611) + t4878*t619;
  p_output1(16)=t1469*t4962 + t2011*t43*t4962 - 0.0222*t4962*t54 - 0.157*(t43*t4962*t553 + t1885*t611) - 0.3151*(t1885*t553 - 1.*t43*t4962*t611) + t1885*t619;
  p_output1(17)=t1469*t41*t622 + t2011*t41*t43*t622 - 0.0222*t41*t54*t622 - 1.*t41*t619*t631 - 0.3151*(-1.*t41*t43*t611*t622 - 1.*t41*t553*t631) - 0.157*(t41*t43*t553*t622 - 1.*t41*t611*t631);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=-0.0222*t3748 + t3538*t5095 + t2011*t5105 - 0.157*t5105*t553 + 0.3151*t5105*t611 + t41*t5090*t629;
  p_output1(34)=t41*t509*t5090 + t5095*t5128 - 0.0222*t5138 + t2011*t5156 - 0.157*t5156*t553 + 0.3151*t5156*t611;
  p_output1(35)=-0.0222*t5176 + t2011*t5212 - 0.157*t5212*t553 + 0.3151*t5212*t611 - 1.*t5090*t623 + t41*t5095*t631;
  p_output1(36)=-0.157*t3845 + t3389*t5250 + t3748*t5283 - 0.3151*(-1.*t3748*t553 - 1.*t3389*t611);
  p_output1(37)=t4962*t5250 + t5138*t5283 - 0.3151*(-1.*t5138*t553 - 1.*t4962*t611) - 0.157*(t4962*t553 - 1.*t5138*t611);
  p_output1(38)=t5176*t5283 + t41*t5250*t622 - 0.157*(-1.*t5176*t611 + t41*t553*t622) - 0.3151*(-1.*t5176*t553 - 1.*t41*t611*t622);
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rHipRoll(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
