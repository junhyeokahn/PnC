/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:21 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootFront.h"

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
  double t2368;
  double t2755;
  double t3857;
  double t862;
  double t3915;
  double t3917;
  double t3928;
  double t3948;
  double t4066;
  double t4071;
  double t4073;
  double t4089;
  double t4098;
  double t4100;
  double t4102;
  double t4043;
  double t4044;
  double t4051;
  double t4115;
  double t4116;
  double t4119;
  double t4123;
  double t4131;
  double t4132;
  double t4136;
  double t4149;
  double t4153;
  double t4154;
  double t972;
  double t1161;
  double t3233;
  double t3298;
  double t3738;
  double t3891;
  double t3900;
  double t3946;
  double t3979;
  double t3996;
  double t4005;
  double t4016;
  double t4028;
  double t4088;
  double t4092;
  double t4095;
  double t4104;
  double t4107;
  double t4111;
  double t4120;
  double t4126;
  double t4130;
  double t4219;
  double t4225;
  double t4228;
  double t4203;
  double t4205;
  double t4210;
  double t4139;
  double t4143;
  double t4147;
  double t4236;
  double t4238;
  double t4240;
  double t4250;
  double t4251;
  double t4254;
  double t4291;
  double t4297;
  double t4299;
  double t4293;
  double t4295;
  double t4301;
  double t4302;
  double t4304;
  double t4307;
  double t4308;
  double t4309;
  t2368 = Sin(var1[5]);
  t2755 = Cos(var1[6]);
  t3857 = Sin(var1[6]);
  t862 = Cos(var1[5]);
  t3915 = Cos(var1[7]);
  t3917 = -1.*t3915;
  t3928 = 1. + t3917;
  t3948 = Sin(var1[7]);
  t4066 = Cos(var1[8]);
  t4071 = -1.*t4066;
  t4073 = 1. + t4071;
  t4089 = Sin(var1[8]);
  t4098 = t862*t3915;
  t4100 = -1.*t2368*t3857*t3948;
  t4102 = t4098 + t4100;
  t4043 = t3915*t2368*t3857;
  t4044 = t862*t3948;
  t4051 = t4043 + t4044;
  t4115 = Cos(var1[9]);
  t4116 = -1.*t4115;
  t4119 = 1. + t4116;
  t4123 = Sin(var1[9]);
  t4131 = t4066*t4102;
  t4132 = -1.*t4051*t4089;
  t4136 = t4131 + t4132;
  t4149 = t4066*t4051;
  t4153 = t4102*t4089;
  t4154 = t4149 + t4153;
  t972 = -1.*t862;
  t1161 = 1. + t972;
  t3233 = -1.*t2755;
  t3298 = 1. + t3233;
  t3738 = -0.330988*t3298;
  t3891 = -0.90524*t3857;
  t3900 = 0. + t3738 + t3891;
  t3946 = -0.97024*t3928;
  t3979 = -0.066675*t3948;
  t3996 = 0. + t3946 + t3979;
  t4005 = -0.066675*t3928;
  t4016 = 0.97024*t3948;
  t4028 = 0. + t4005 + t4016;
  t4088 = -1.45024*t4073;
  t4092 = -0.066675*t4089;
  t4095 = 0. + t4088 + t4092;
  t4104 = -0.066675*t4073;
  t4107 = 1.45024*t4089;
  t4111 = 0. + t4104 + t4107;
  t4120 = -0.065597*t4119;
  t4126 = 1.93024*t4123;
  t4130 = 0. + t4120 + t4126;
  t4219 = t3915*t2368;
  t4225 = t862*t3857*t3948;
  t4228 = t4219 + t4225;
  t4203 = -1.*t862*t3915*t3857;
  t4205 = t2368*t3948;
  t4210 = t4203 + t4205;
  t4139 = -1.93024*t4119;
  t4143 = -0.065597*t4123;
  t4147 = 0. + t4139 + t4143;
  t4236 = t4066*t4228;
  t4238 = -1.*t4210*t4089;
  t4240 = t4236 + t4238;
  t4250 = t4066*t4210;
  t4251 = t4228*t4089;
  t4254 = t4250 + t4251;
  t4291 = 0. + t2755;
  t4297 = -1.*t4291*t3948;
  t4299 = 0. + t4297;
  t4293 = t4291*t3915;
  t4295 = 0. + t4293;
  t4301 = t4066*t4299;
  t4302 = -1.*t4295*t4089;
  t4304 = t4301 + t4302;
  t4307 = t4295*t4066;
  t4308 = t4299*t4089;
  t4309 = t4307 + t4308;

  p_output1(0)=0. - 0.066675*t1161 - 0.260988*t2368 + 0.340988*t2368*t2755 - 1.*t2368*t3900 + t2368*t3857*t3996 + t4051*t4095 + t4102*t4111 + t4130*t4136 + t4147*t4154 - 1.840292*(t4123*t4136 + t4115*t4154) - 0.000525*(t4115*t4136 - 1.*t4123*t4154) + t4028*t862;
  p_output1(1)=0. - 0.260988*t1161 + 0.066675*t2368 + t2368*t4028 + t4095*t4210 + t4111*t4228 + t4130*t4240 + t4147*t4254 - 1.840292*(t4123*t4240 + t4115*t4254) - 0.000525*(t4115*t4240 - 1.*t4123*t4254) - 0.340988*t2755*t862 + t3900*t862 - 1.*t3857*t3996*t862;
  p_output1(2)=0. - 0.90524*t3298 + 0.330988*t3857 - 0.340988*(0. + t3857) + t3996*t4291 + t4095*t4295 + t4111*t4299 + t4130*t4304 + t4147*t4309 - 1.840292*(t4123*t4304 + t4115*t4309) - 0.000525*(t4115*t4304 - 1.*t4123*t4309);
}


       
void p_RightFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
