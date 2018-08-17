/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:46 GMT-05:00
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
  double t516;
  double t87;
  double t88;
  double t197;
  double t337;
  double t2384;
  double t2053;
  double t2055;
  double t2451;
  double t714;
  double t843;
  double t934;
  double t1659;
  double t53;
  double t4098;
  double t4101;
  double t4127;
  double t2333;
  double t2505;
  double t3410;
  double t4912;
  double t4952;
  double t5020;
  double t280;
  double t436;
  double t507;
  double t1121;
  double t1896;
  double t1930;
  double t3610;
  double t3739;
  double t4097;
  double t4180;
  double t4214;
  double t4261;
  double t5631;
  double t5633;
  double t5666;
  double t5576;
  double t5596;
  double t5623;
  double t5714;
  double t5727;
  double t5731;
  double t5931;
  double t5949;
  double t5957;
  double t6050;
  double t6058;
  double t6064;
  double t6196;
  double t6197;
  double t6199;
  double t6331;
  double t6341;
  double t6348;
  double t6463;
  double t6469;
  double t6471;
  double t6721;
  double t6722;
  double t6725;
  double t6688;
  double t6696;
  double t6699;
  double t6707;
  double t6717;
  double t6796;
  double t6798;
  double t6799;
  double t5320;
  double t6833;
  double t6838;
  double t6899;
  double t6901;
  double t6911;
  double t5876;
  double t5878;
  double t5885;
  double t6923;
  double t6931;
  double t6939;
  double t6959;
  double t6963;
  double t6810;
  double t6813;
  double t6822;
  double t6878;
  double t6883;
  double t6891;
  t516 = Sin(var1[3]);
  t87 = Cos(var1[11]);
  t88 = -1.*t87;
  t197 = 1. + t88;
  t337 = Sin(var1[11]);
  t2384 = Cos(var1[3]);
  t2053 = Cos(var1[5]);
  t2055 = Sin(var1[4]);
  t2451 = Sin(var1[5]);
  t714 = Cos(var1[12]);
  t843 = -1.*t714;
  t934 = 1. + t843;
  t1659 = Sin(var1[12]);
  t53 = Cos(var1[4]);
  t4098 = -1.*t2384*t2053;
  t4101 = -1.*t516*t2055*t2451;
  t4127 = t4098 + t4101;
  t2333 = -1.*t2053*t516*t2055;
  t2505 = t2384*t2451;
  t3410 = t2333 + t2505;
  t4912 = t53*t337*t516;
  t4952 = t87*t4127;
  t5020 = t4912 + t4952;
  t280 = -0.022225*t197;
  t436 = -0.086996*t337;
  t507 = 0. + t280 + t436;
  t1121 = -0.31508*t934;
  t1896 = 0.156996*t1659;
  t1930 = 0. + t1121 + t1896;
  t3610 = -0.086996*t197;
  t3739 = 0.022225*t337;
  t4097 = 0. + t3610 + t3739;
  t4180 = -0.156996*t934;
  t4214 = -0.31508*t1659;
  t4261 = 0. + t4180 + t4214;
  t5631 = -1.*t2053*t516;
  t5633 = t2384*t2055*t2451;
  t5666 = t5631 + t5633;
  t5576 = t2384*t2053*t2055;
  t5596 = t516*t2451;
  t5623 = t5576 + t5596;
  t5714 = -1.*t2384*t53*t337;
  t5727 = t87*t5666;
  t5731 = t5714 + t5727;
  t5931 = t2384*t337*t2055;
  t5949 = t87*t2384*t53*t2451;
  t5957 = t5931 + t5949;
  t6050 = t337*t516*t2055;
  t6058 = t87*t53*t516*t2451;
  t6064 = t6050 + t6058;
  t6196 = t53*t337;
  t6197 = -1.*t87*t2055*t2451;
  t6199 = t6196 + t6197;
  t6331 = t2053*t516;
  t6341 = -1.*t2384*t2055*t2451;
  t6348 = t6331 + t6341;
  t6463 = t2053*t516*t2055;
  t6469 = -1.*t2384*t2451;
  t6471 = t6463 + t6469;
  t6721 = -1.*t87*t2384*t53;
  t6722 = -1.*t337*t5666;
  t6725 = t6721 + t6722;
  t6688 = -0.086996*t87;
  t6696 = -0.022225*t337;
  t6699 = t6688 + t6696;
  t6707 = 0.022225*t87;
  t6717 = t6707 + t436;
  t6796 = t2384*t2053;
  t6798 = t516*t2055*t2451;
  t6799 = t6796 + t6798;
  t5320 = -1.*t87*t53*t516;
  t6833 = -1.*t337*t6799;
  t6838 = t5320 + t6833;
  t6899 = t87*t2055;
  t6901 = -1.*t53*t337*t2451;
  t6911 = t6899 + t6901;
  t5876 = t714*t5623;
  t5878 = -1.*t1659*t5731;
  t5885 = t5876 + t5878;
  t6923 = 0.156996*t714;
  t6931 = t6923 + t4214;
  t6939 = -0.31508*t714;
  t6959 = -0.156996*t1659;
  t6963 = t6939 + t6959;
  t6810 = -1.*t53*t337*t516;
  t6813 = t87*t6799;
  t6822 = t6810 + t6813;
  t6878 = t337*t2055;
  t6883 = t87*t53*t2451;
  t6891 = t6878 + t6883;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1930*t3410 + t4097*t4127 + t4261*t5020 - 1.*t507*t516*t53 - 0.022225*(t337*t4127 + t5320) - 0.31508*(-1.*t1659*t5020 + t3410*t714) - 0.156996*(t1659*t3410 + t5020*t714);
  p_output1(10)=t2384*t507*t53 + t1930*t5623 + t4097*t5666 + t4261*t5731 - 0.31508*t5885 - 0.156996*(t1659*t5623 + t5731*t714) - 0.022225*(t337*t5666 + t2384*t53*t87);
  p_output1(11)=0;
  p_output1(12)=-1.*t2055*t2384*t507 + t1930*t2053*t2384*t53 + t2384*t2451*t4097*t53 + t4261*t5957 - 0.31508*(-1.*t1659*t5957 + t2053*t2384*t53*t714) - 0.156996*(t1659*t2053*t2384*t53 + t5957*t714) - 0.022225*(t2384*t2451*t337*t53 - 1.*t2055*t2384*t87);
  p_output1(13)=-1.*t2055*t507*t516 + t1930*t2053*t516*t53 + t2451*t4097*t516*t53 + t4261*t6064 - 0.31508*(-1.*t1659*t6064 + t2053*t516*t53*t714) - 0.156996*(t1659*t2053*t516*t53 + t6064*t714) - 0.022225*(t2451*t337*t516*t53 - 1.*t2055*t516*t87);
  p_output1(14)=-1.*t1930*t2053*t2055 - 1.*t2055*t2451*t4097 - 1.*t507*t53 + t4261*t6199 - 0.31508*(-1.*t1659*t6199 - 1.*t2053*t2055*t714) - 0.156996*(-1.*t1659*t2053*t2055 + t6199*t714) - 0.022225*(-1.*t2055*t2451*t337 - 1.*t53*t87);
  p_output1(15)=-0.022225*t337*t5623 + t4097*t5623 + t1930*t6348 + t4261*t5623*t87 - 0.31508*(t6348*t714 - 1.*t1659*t5623*t87) - 0.156996*(t1659*t6348 + t5623*t714*t87);
  p_output1(16)=t1930*t4127 - 0.022225*t337*t6471 + t4097*t6471 + t4261*t6471*t87 - 0.31508*(t4127*t714 - 1.*t1659*t6471*t87) - 0.156996*(t1659*t4127 + t6471*t714*t87);
  p_output1(17)=-1.*t1930*t2451*t53 - 0.022225*t2053*t337*t53 + t2053*t4097*t53 + t2053*t4261*t53*t87 - 0.31508*(-1.*t2451*t53*t714 - 1.*t1659*t2053*t53*t87) - 0.156996*(-1.*t1659*t2451*t53 + t2053*t53*t714*t87);
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
  p_output1(33)=-0.022225*t5731 + t2384*t53*t6699 + t5666*t6717 + 0.31508*t1659*t6725 + t4261*t6725 - 0.156996*t6725*t714;
  p_output1(34)=t516*t53*t6699 + t6717*t6799 - 0.022225*t6822 + 0.31508*t1659*t6838 + t4261*t6838 - 0.156996*t6838*t714;
  p_output1(35)=-1.*t2055*t6699 + t2451*t53*t6717 - 0.022225*t6891 + 0.31508*t1659*t6911 + t4261*t6911 - 0.156996*t6911*t714;
  p_output1(36)=-0.156996*t5885 + t5623*t6931 + t5731*t6963 - 0.31508*(-1.*t1659*t5623 - 1.*t5731*t714);
  p_output1(37)=t6471*t6931 + t6822*t6963 - 0.156996*(-1.*t1659*t6822 + t6471*t714) - 0.31508*(-1.*t1659*t6471 - 1.*t6822*t714);
  p_output1(38)=t2053*t53*t6931 + t6891*t6963 - 0.156996*(-1.*t1659*t6891 + t2053*t53*t714) - 0.31508*(-1.*t1659*t2053*t53 - 1.*t6891*t714);
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
