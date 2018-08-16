/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:19 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_LeftFootBack.h"

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
  double t1392;
  double t922;
  double t931;
  double t1249;
  double t1448;
  double t873;
  double t2378;
  double t2673;
  double t2696;
  double t1280;
  double t1573;
  double t1592;
  double t2705;
  double t511;
  double t3614;
  double t3624;
  double t3676;
  double t2376;
  double t2951;
  double t3478;
  double t3706;
  double t3791;
  double t3797;
  double t3807;
  double t3762;
  double t3775;
  double t3776;
  double t3857;
  double t3858;
  double t3866;
  double t3785;
  double t3820;
  double t3840;
  double t3906;
  double t3913;
  double t3918;
  double t3919;
  double t3914;
  double t3916;
  double t3929;
  double t3931;
  double t3936;
  double t3917;
  double t3923;
  double t3925;
  double t3484;
  double t3726;
  double t3844;
  double t3878;
  double t3928;
  double t3937;
  double t4034;
  double t4035;
  double t4053;
  double t4055;
  double t4078;
  double t4083;
  double t3976;
  double t3728;
  double t3729;
  double t3738;
  double t4012;
  double t4014;
  double t4023;
  double t4025;
  double t4026;
  double t4028;
  double t4029;
  double t4036;
  double t4037;
  double t4040;
  double t4044;
  double t4050;
  double t4051;
  double t4060;
  double t4062;
  double t4063;
  double t4066;
  double t4070;
  double t4071;
  double t4088;
  double t4090;
  double t4091;
  double t4095;
  double t4096;
  double t4098;
  double t3990;
  double t3891;
  double t3892;
  double t3893;
  double t3963;
  double t4002;
  double t3939;
  double t3941;
  double t3946;
  t1392 = Cos(var1[0]);
  t922 = Cos(var1[2]);
  t931 = Sin(var1[0]);
  t1249 = Sin(var1[1]);
  t1448 = Sin(var1[2]);
  t873 = Cos(var1[3]);
  t2378 = t1392*t922;
  t2673 = -1.*t931*t1249*t1448;
  t2696 = t2378 + t2673;
  t1280 = t922*t931*t1249;
  t1573 = t1392*t1448;
  t1592 = t1280 + t1573;
  t2705 = Sin(var1[3]);
  t511 = Cos(var1[4]);
  t3614 = t873*t2696;
  t3624 = -1.*t1592*t2705;
  t3676 = t3614 + t3624;
  t2376 = t873*t1592;
  t2951 = t2696*t2705;
  t3478 = t2376 + t2951;
  t3706 = Sin(var1[4]);
  t3791 = t922*t931;
  t3797 = t1392*t1249*t1448;
  t3807 = t3791 + t3797;
  t3762 = -1.*t1392*t922*t1249;
  t3775 = t931*t1448;
  t3776 = t3762 + t3775;
  t3857 = t873*t3807;
  t3858 = -1.*t3776*t2705;
  t3866 = t3857 + t3858;
  t3785 = t873*t3776;
  t3820 = t3807*t2705;
  t3840 = t3785 + t3820;
  t3906 = Cos(var1[1]);
  t3913 = 0. + t3906;
  t3918 = -1.*t3913*t1448;
  t3919 = 0. + t3918;
  t3914 = t3913*t922;
  t3916 = 0. + t3914;
  t3929 = t873*t3919;
  t3931 = -1.*t3916*t2705;
  t3936 = t3929 + t3931;
  t3917 = t3916*t873;
  t3923 = t3919*t2705;
  t3925 = t3917 + t3923;
  t3484 = t511*t3478;
  t3726 = t3676*t3706;
  t3844 = t511*t3840;
  t3878 = t3866*t3706;
  t3928 = t511*t3925;
  t3937 = t3936*t3706;
  t4034 = -1.*t922;
  t4035 = 1. + t4034;
  t4053 = -1.*t873;
  t4055 = 1. + t4053;
  t4078 = -1.*t511;
  t4083 = 1. + t4078;
  t3976 = t3484 + t3726;
  t3728 = t511*t3676;
  t3729 = -1.*t3478*t3706;
  t3738 = t3728 + t3729;
  t4012 = -1.*t1392;
  t4014 = 1. + t4012;
  t4023 = -1.*t3906;
  t4025 = 1. + t4023;
  t4026 = 0.331012*t4025;
  t4028 = -0.90524*t1249;
  t4029 = 0. + t4026 + t4028;
  t4036 = -0.97024*t4035;
  t4037 = -0.066675*t1448;
  t4040 = 0. + t4036 + t4037;
  t4044 = -0.066675*t4035;
  t4050 = 0.97024*t1448;
  t4051 = 0. + t4044 + t4050;
  t4060 = -1.45024*t4055;
  t4062 = -0.066675*t2705;
  t4063 = 0. + t4060 + t4062;
  t4066 = -0.066675*t4055;
  t4070 = 1.45024*t2705;
  t4071 = 0. + t4066 + t4070;
  t4088 = -1.93024*t4083;
  t4090 = -0.065597*t3706;
  t4091 = 0. + t4088 + t4090;
  t4095 = -0.065597*t4083;
  t4096 = 1.93024*t3706;
  t4098 = 0. + t4095 + t4096;
  t3990 = t3844 + t3878;
  t3891 = t511*t3866;
  t3892 = -1.*t3840*t3706;
  t3893 = t3891 + t3892;
  t3963 = 0. + t1249;
  t4002 = t3928 + t3937;
  t3939 = t511*t3936;
  t3941 = -1.*t3925*t3706;
  t3946 = t3939 + t3941;

  p_output1(0)=t3484 + t3726 + 0.000796*t3738;
  p_output1(1)=t3844 + t3878 + 0.000796*t3893;
  p_output1(2)=t3928 + t3937 + 0.000796*t3946;
  p_output1(3)=0.;
  p_output1(4)=-1.*t3906*t931;
  p_output1(5)=t1392*t3906;
  p_output1(6)=t3963;
  p_output1(7)=0.;
  p_output1(8)=t3478*t3706 + 0.000796*t3976 - 1.*t3676*t511;
  p_output1(9)=t3706*t3840 + 0.000796*t3990 - 1.*t3866*t511;
  p_output1(10)=t3706*t3925 + 0.000796*t4002 - 1.*t3936*t511;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.000645*t3738 - 1.990292*t3976 - 0.066675*t4014 + t1392*t4051 + t1592*t4063 + t2696*t4071 + t3478*t4091 + t3676*t4098 + 0.261012*t931 - 0.341012*t3906*t931 - 1.*t4029*t931 + t1249*t4040*t931;
  p_output1(13)=0. - 0.000645*t3893 + 0.341012*t1392*t3906 - 1.990292*t3990 + 0.261012*t4014 + t1392*t4029 - 1.*t1249*t1392*t4040 + t3776*t4063 + t3807*t4071 + t3840*t4091 + t3866*t4098 + 0.066675*t931 + t4051*t931;
  p_output1(14)=0. - 0.331012*t1249 - 0.000645*t3946 + 0.341012*t3963 - 1.990292*t4002 - 0.90524*t4025 + t3913*t4040 + t3916*t4063 + t3919*t4071 + t3925*t4091 + t3936*t4098;
  p_output1(15)=1.;
}


       
void H_LeftFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
