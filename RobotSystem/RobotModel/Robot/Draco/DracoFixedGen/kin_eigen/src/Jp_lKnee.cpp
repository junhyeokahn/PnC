/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lKnee.h"

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
static void output1(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t11;
  double t149;
  double t629;
  double t202;
  double t653;
  double t656;
  double t658;
  double t675;
  double t739;
  double t743;
  double t744;
  double t750;
  double t754;
  double t756;
  double t757;
  double t726;
  double t731;
  double t733;
  double t591;
  double t614;
  double t625;
  double t634;
  double t646;
  double t673;
  double t683;
  double t692;
  double t698;
  double t700;
  double t704;
  double t749;
  double t751;
  double t752;
  double t762;
  double t763;
  double t771;
  double t836;
  double t837;
  double t843;
  double t819;
  double t822;
  double t829;
  double t863;
  double t867;
  double t874;
  double t845;
  double t991;
  double t993;
  double t995;
  double t970;
  double t971;
  double t974;
  double t978;
  double t979;
  double t1017;
  double t1020;
  double t1022;
  double t794;
  double t1035;
  double t846;
  double t847;
  double t1008;
  double t1063;
  double t1065;
  double t1068;
  double t1072;
  double t1073;
  double t1027;
  double t1083;
  double t1084;
  double t1085;
  double t1031;
  double t1113;
  double t1115;
  double t1110;
  double t1111;
  t11 = Cos(var1[0]);
  t149 = Cos(var1[1]);
  t629 = Sin(var1[1]);
  t202 = Sin(var1[0]);
  t653 = Cos(var1[2]);
  t656 = -1.*t653;
  t658 = 1. + t656;
  t675 = Sin(var1[2]);
  t739 = Cos(var1[3]);
  t743 = -1.*t739;
  t744 = 1. + t743;
  t750 = Sin(var1[3]);
  t754 = -1.*t653*t202;
  t756 = -1.*t11*t629*t675;
  t757 = t754 + t756;
  t726 = t11*t653*t629;
  t731 = -1.*t202*t675;
  t733 = t726 + t731;
  t591 = -1.*t149;
  t614 = 1. + t591;
  t625 = 0.331012*t614;
  t634 = -0.90524*t629;
  t646 = 0. + t625 + t634;
  t673 = -0.97024*t658;
  t683 = -0.066675*t675;
  t692 = 0. + t673 + t683;
  t698 = -0.066675*t658;
  t700 = 0.97024*t675;
  t704 = 0. + t698 + t700;
  t749 = -1.45024*t744;
  t751 = -0.066675*t750;
  t752 = 0. + t749 + t751;
  t762 = -0.066675*t744;
  t763 = 1.45024*t750;
  t771 = 0. + t762 + t763;
  t836 = t11*t653;
  t837 = -1.*t202*t629*t675;
  t843 = t836 + t837;
  t819 = t653*t202*t629;
  t822 = t11*t675;
  t829 = t819 + t822;
  t863 = -0.90524*t149;
  t867 = 0.331012*t629;
  t874 = t863 + t867;
  t845 = t739*t843;
  t991 = -1.*t653*t202*t629;
  t993 = -1.*t11*t675;
  t995 = t991 + t993;
  t970 = -0.066675*t653;
  t971 = -0.97024*t675;
  t974 = t970 + t971;
  t978 = 0.97024*t653;
  t979 = t978 + t683;
  t1017 = t653*t202;
  t1020 = t11*t629*t675;
  t1022 = t1017 + t1020;
  t794 = t739*t733;
  t1035 = 0. + t149;
  t846 = -1.*t829*t750;
  t847 = t845 + t846;
  t1008 = -1.*t843*t750;
  t1063 = -0.066675*t739;
  t1065 = -1.45024*t750;
  t1068 = t1063 + t1065;
  t1072 = 1.45024*t739;
  t1073 = t1072 + t751;
  t1027 = t739*t1022;
  t1083 = -1.*t11*t653*t629;
  t1084 = t202*t675;
  t1085 = t1083 + t1084;
  t1031 = -1.*t1022*t750;
  t1113 = -1.*t1035*t675;
  t1115 = 0. + t1113;
  t1110 = t1035*t653;
  t1111 = 0. + t1110;

  p_output1(0)=0.261012*t11 - 0.324262*t11*t149 - 0.066675*t202 - 1.*t11*t646 + t11*t629*t692 - 1.*t202*t704 + t733*t752 - 0.066675*(-1.*t733*t750 + t739*t757) + t757*t771 - 1.45024*(t750*t757 + t794);
  p_output1(1)=0.066675*t11 + 0.261012*t202 - 0.324262*t149*t202 - 1.*t202*t646 + t202*t629*t692 + t11*t704 + t752*t829 + t771*t843 - 1.45024*(t739*t829 + t750*t843) - 0.066675*t847;
  p_output1(2)=0;
  p_output1(3)=0.324262*t202*t629 + t149*t202*t692 - 0.066675*(-1.*t149*t202*t675*t739 - 1.*t149*t202*t653*t750) - 1.45024*(t149*t202*t653*t739 - 1.*t149*t202*t675*t750) + t149*t202*t653*t752 - 1.*t149*t202*t675*t771 - 1.*t202*t874;
  p_output1(4)=-0.324262*t11*t629 - 1.*t11*t149*t692 - 0.066675*(t11*t149*t675*t739 + t11*t149*t653*t750) - 1.45024*(-1.*t11*t149*t653*t739 + t11*t149*t675*t750) - 1.*t11*t149*t653*t752 + t11*t149*t675*t771 + t11*t874;
  p_output1(5)=-0.006749999999999978*t149 + t634 - 1.*t629*t692 - 0.066675*(t629*t675*t739 + t629*t653*t750) - 1.45024*(-1.*t629*t653*t739 + t629*t675*t750) - 1.*t629*t653*t752 + t629*t675*t771;
  p_output1(6)=t752*t843 + t202*t629*t974 + t11*t979 + t771*t995 - 0.066675*(t1008 + t739*t995) - 1.45024*(t845 + t750*t995);
  p_output1(7)=-1.45024*(t1027 + t733*t750) + t1022*t752 + t733*t771 - 0.066675*(t1031 + t794) - 1.*t11*t629*t974 + t202*t979;
  p_output1(8)=-1.45024*(-1.*t1035*t675*t739 - 1.*t1035*t653*t750) - 0.066675*(-1.*t1035*t653*t739 + t1035*t675*t750) - 1.*t1035*t675*t752 - 1.*t1035*t653*t771 + t1035*t974;
  p_output1(9)=t1068*t829 - 0.066675*(t1008 - 1.*t739*t829) + t1073*t843 - 1.45024*t847;
  p_output1(10)=t1022*t1073 + t1068*t1085 - 0.066675*(t1031 - 1.*t1085*t739) - 1.45024*(t1027 - 1.*t1085*t750);
  p_output1(11)=t1068*t1111 + t1073*t1115 - 1.45024*(t1115*t739 - 1.*t1111*t750) - 0.066675*(-1.*t1111*t739 - 1.*t1115*t750);
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0;
  p_output1(17)=0;
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
}


       
void Jp_lKnee(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
