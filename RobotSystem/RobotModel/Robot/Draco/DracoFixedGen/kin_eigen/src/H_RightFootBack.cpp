/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:24 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootBack.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1337;
  double t1404;
  double t1503;
  double t1225;
  double t1513;
  double t748;
  double t2344;
  double t2362;
  double t2412;
  double t1401;
  double t1523;
  double t1544;
  double t2573;
  double t2825;
  double t1845;
  double t2652;
  double t2678;
  double t69;
  double t2913;
  double t3402;
  double t3446;
  double t4467;
  double t4499;
  double t4512;
  double t4106;
  double t4199;
  double t4364;
  double t4377;
  double t4519;
  double t4529;
  double t4566;
  double t4571;
  double t4573;
  double t4636;
  double t4646;
  double t4660;
  double t4661;
  double t4649;
  double t4655;
  double t4658;
  double t4665;
  double t4666;
  double t4670;
  double t4673;
  double t4677;
  double t2819;
  double t3491;
  double t4549;
  double t4603;
  double t4668;
  double t4678;
  double t4786;
  double t4790;
  double t4811;
  double t4815;
  double t4839;
  double t4841;
  double t4711;
  double t3560;
  double t3716;
  double t3873;
  double t4752;
  double t4753;
  double t4770;
  double t4772;
  double t4775;
  double t4777;
  double t4784;
  double t4791;
  double t4800;
  double t4802;
  double t4805;
  double t4807;
  double t4808;
  double t4819;
  double t4823;
  double t4826;
  double t4828;
  double t4829;
  double t4832;
  double t4842;
  double t4843;
  double t4845;
  double t4848;
  double t4849;
  double t4851;
  double t4736;
  double t4607;
  double t4614;
  double t4628;
  double t4704;
  double t4747;
  double t4686;
  double t4689;
  double t4690;
  t1337 = Cos(var1[7]);
  t1404 = Sin(var1[5]);
  t1503 = Sin(var1[6]);
  t1225 = Cos(var1[5]);
  t1513 = Sin(var1[7]);
  t748 = Cos(var1[8]);
  t2344 = t1337*t1404*t1503;
  t2362 = t1225*t1513;
  t2412 = t2344 + t2362;
  t1401 = t1225*t1337;
  t1523 = -1.*t1404*t1503*t1513;
  t1544 = t1401 + t1523;
  t2573 = Sin(var1[8]);
  t2825 = Cos(var1[9]);
  t1845 = t748*t1544;
  t2652 = -1.*t2412*t2573;
  t2678 = t1845 + t2652;
  t69 = Sin(var1[9]);
  t2913 = t748*t2412;
  t3402 = t1544*t2573;
  t3446 = t2913 + t3402;
  t4467 = -1.*t1225*t1337*t1503;
  t4499 = t1404*t1513;
  t4512 = t4467 + t4499;
  t4106 = t1337*t1404;
  t4199 = t1225*t1503*t1513;
  t4364 = t4106 + t4199;
  t4377 = t748*t4364;
  t4519 = -1.*t4512*t2573;
  t4529 = t4377 + t4519;
  t4566 = t748*t4512;
  t4571 = t4364*t2573;
  t4573 = t4566 + t4571;
  t4636 = Cos(var1[6]);
  t4646 = 0. + t4636;
  t4660 = t4646*t1337;
  t4661 = 0. + t4660;
  t4649 = -1.*t4646*t1513;
  t4655 = 0. + t4649;
  t4658 = t748*t4655;
  t4665 = -1.*t4661*t2573;
  t4666 = t4658 + t4665;
  t4670 = t4661*t748;
  t4673 = t4655*t2573;
  t4677 = t4670 + t4673;
  t2819 = t69*t2678;
  t3491 = t2825*t3446;
  t4549 = t69*t4529;
  t4603 = t2825*t4573;
  t4668 = t69*t4666;
  t4678 = t2825*t4677;
  t4786 = -1.*t1337;
  t4790 = 1. + t4786;
  t4811 = -1.*t748;
  t4815 = 1. + t4811;
  t4839 = -1.*t2825;
  t4841 = 1. + t4839;
  t4711 = t2819 + t3491;
  t3560 = t2825*t2678;
  t3716 = -1.*t69*t3446;
  t3873 = t3560 + t3716;
  t4752 = -1.*t1225;
  t4753 = 1. + t4752;
  t4770 = -1.*t4636;
  t4772 = 1. + t4770;
  t4775 = -0.330988*t4772;
  t4777 = -0.90524*t1503;
  t4784 = 0. + t4775 + t4777;
  t4791 = -0.97024*t4790;
  t4800 = -0.066675*t1513;
  t4802 = 0. + t4791 + t4800;
  t4805 = -0.066675*t4790;
  t4807 = 0.97024*t1513;
  t4808 = 0. + t4805 + t4807;
  t4819 = -1.45024*t4815;
  t4823 = -0.066675*t2573;
  t4826 = 0. + t4819 + t4823;
  t4828 = -0.066675*t4815;
  t4829 = 1.45024*t2573;
  t4832 = 0. + t4828 + t4829;
  t4842 = -0.065597*t4841;
  t4843 = 1.93024*t69;
  t4845 = 0. + t4842 + t4843;
  t4848 = -1.93024*t4841;
  t4849 = -0.065597*t69;
  t4851 = 0. + t4848 + t4849;
  t4736 = t4549 + t4603;
  t4607 = t2825*t4529;
  t4614 = -1.*t69*t4573;
  t4628 = t4607 + t4614;
  t4704 = 0. + t1503;
  t4747 = t4668 + t4678;
  t4686 = t2825*t4666;
  t4689 = -1.*t69*t4677;
  t4690 = t4686 + t4689;

  p_output1(0)=t2819 + t3491 + 0.000796*t3873;
  p_output1(1)=t4549 + t4603 + 0.000796*t4628;
  p_output1(2)=t4668 + t4678 + 0.000796*t4690;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1404*t4636;
  p_output1(5)=t1225*t4636;
  p_output1(6)=t4704;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2678*t2825 + 0.000796*t4711 + t3446*t69;
  p_output1(9)=-1.*t2825*t4529 + 0.000796*t4736 + t4573*t69;
  p_output1(10)=-1.*t2825*t4666 + 0.000796*t4747 + t4677*t69;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.260988*t1404 - 0.000645*t3873 + 0.340988*t1404*t4636 - 1.990292*t4711 - 0.066675*t4753 - 1.*t1404*t4784 + t1404*t1503*t4802 + t1225*t4808 + t2412*t4826 + t1544*t4832 + t2678*t4845 + t3446*t4851;
  p_output1(13)=0. + 0.066675*t1404 - 0.000645*t4628 - 0.340988*t1225*t4636 - 1.990292*t4736 - 0.260988*t4753 + t1225*t4784 - 1.*t1225*t1503*t4802 + t1404*t4808 + t4512*t4826 + t4364*t4832 + t4529*t4845 + t4573*t4851;
  p_output1(14)=0. + 0.330988*t1503 - 0.000645*t4690 - 0.340988*t4704 - 1.990292*t4747 - 0.90524*t4772 + t4646*t4802 + t4661*t4826 + t4655*t4832 + t4666*t4845 + t4677*t4851;
  p_output1(15)=1.;
}


       
void H_RightFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
