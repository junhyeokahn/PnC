/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:19:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lKnee.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t543;
  double t558;
  double t588;
  double t496;
  double t600;
  double t608;
  double t611;
  double t615;
  double t637;
  double t641;
  double t643;
  double t648;
  double t658;
  double t661;
  double t663;
  double t630;
  double t634;
  double t635;
  double t516;
  double t520;
  double t576;
  double t585;
  double t586;
  double t591;
  double t594;
  double t614;
  double t616;
  double t620;
  double t624;
  double t625;
  double t627;
  double t646;
  double t653;
  double t655;
  double t667;
  double t668;
  double t669;
  double t731;
  double t733;
  double t735;
  double t706;
  double t719;
  double t722;
  double t764;
  double t774;
  double t775;
  double t769;
  double t770;
  t543 = Sin(var1[0]);
  t558 = Cos(var1[1]);
  t588 = Sin(var1[1]);
  t496 = Cos(var1[0]);
  t600 = Cos(var1[2]);
  t608 = -1.*t600;
  t611 = 1. + t608;
  t615 = Sin(var1[2]);
  t637 = Cos(var1[3]);
  t641 = -1.*t637;
  t643 = 1. + t641;
  t648 = Sin(var1[3]);
  t658 = t496*t600;
  t661 = -1.*t543*t588*t615;
  t663 = t658 + t661;
  t630 = t600*t543*t588;
  t634 = t496*t615;
  t635 = t630 + t634;
  t516 = -1.*t496;
  t520 = 1. + t516;
  t576 = -1.*t558;
  t585 = 1. + t576;
  t586 = 0.331012*t585;
  t591 = -0.90524*t588;
  t594 = 0. + t586 + t591;
  t614 = -0.97024*t611;
  t616 = -0.066675*t615;
  t620 = 0. + t614 + t616;
  t624 = -0.066675*t611;
  t625 = 0.97024*t615;
  t627 = 0. + t624 + t625;
  t646 = -1.45024*t643;
  t653 = -0.066675*t648;
  t655 = 0. + t646 + t653;
  t667 = -0.066675*t643;
  t668 = 1.45024*t648;
  t669 = 0. + t667 + t668;
  t731 = t600*t543;
  t733 = t496*t588*t615;
  t735 = t731 + t733;
  t706 = -1.*t496*t600*t588;
  t719 = t543*t615;
  t722 = t706 + t719;
  t764 = 0. + t558;
  t774 = -1.*t764*t615;
  t775 = 0. + t774;
  t769 = t764*t600;
  t770 = 0. + t769;

  p_output1(0)=0. - 0.066675*t520 + 0.261012*t543 - 0.324262*t543*t558 - 1.*t543*t594 + t543*t588*t620 + t496*t627 + t635*t655 - 0.066675*(-1.*t635*t648 + t637*t663) - 1.45024*(t635*t637 + t648*t663) + t663*t669;
  p_output1(1)=0. + 0.261012*t520 + 0.066675*t543 + 0.324262*t496*t558 + t496*t594 - 1.*t496*t588*t620 + t543*t627 + t655*t722 + t669*t735 - 0.066675*(-1.*t648*t722 + t637*t735) - 1.45024*(t637*t722 + t648*t735);
  p_output1(2)=0. - 0.90524*t585 - 0.331012*t588 + 0.324262*(0. + t588) + t620*t764 + t655*t770 + t669*t775 - 0.066675*(-1.*t648*t770 + t637*t775) - 1.45024*(t637*t770 + t648*t775);
}


       
void p_lKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
