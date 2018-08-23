/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:37 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootBack.h"

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
  double t57;
  double t810;
  double t1015;
  double t1256;
  double t1302;
  double t2094;
  double t2230;
  double t2172;
  double t2455;
  double t1494;
  double t1533;
  double t1915;
  double t1941;
  double t356;
  double t2947;
  double t2956;
  double t3018;
  double t2203;
  double t2473;
  double t2482;
  double t3156;
  double t3163;
  double t3232;
  double t3417;
  double t3451;
  double t3498;
  double t3570;
  double t3714;
  double t3741;
  double t3812;
  double t4048;
  double t4099;
  double t4100;
  double t4123;
  double t4201;
  double t4263;
  double t4455;
  double t4574;
  double t4611;
  double t4624;
  double t4756;
  double t4812;
  double t4829;
  double t4847;
  double t4930;
  double t4950;
  double t5145;
  double t5177;
  double t5179;
  double t5203;
  double t5286;
  double t5312;
  double t5325;
  double t1266;
  double t1327;
  double t1363;
  double t1932;
  double t1963;
  double t1986;
  double t2700;
  double t2722;
  double t2814;
  double t3029;
  double t3111;
  double t3139;
  double t5444;
  double t5445;
  double t5450;
  double t3527;
  double t3701;
  double t3711;
  double t5407;
  double t5411;
  double t5414;
  double t5471;
  double t5480;
  double t5484;
  double t4032;
  double t4034;
  double t4046;
  double t4316;
  double t4463;
  double t4572;
  double t5515;
  double t5522;
  double t5526;
  double t5635;
  double t5645;
  double t5676;
  double t4675;
  double t4709;
  double t4724;
  double t5104;
  double t5148;
  double t5165;
  double t5700;
  double t5706;
  double t5709;
  double t5720;
  double t5721;
  double t5734;
  double t5214;
  double t5215;
  double t5230;
  double t5751;
  double t5756;
  double t5764;
  double t5830;
  double t5849;
  double t5868;
  double t5981;
  double t5985;
  double t5989;
  double t6004;
  double t6029;
  double t6054;
  double t6150;
  double t6179;
  double t6188;
  double t6197;
  double t6214;
  double t6220;
  double t6229;
  double t6239;
  double t6243;
  double t6253;
  double t6254;
  double t6263;
  double t6268;
  double t6280;
  double t6284;
  t57 = Cos(var1[3]);
  t810 = Cos(var1[11]);
  t1015 = -1.*t810;
  t1256 = 1. + t1015;
  t1302 = Sin(var1[11]);
  t2094 = Cos(var1[5]);
  t2230 = Sin(var1[3]);
  t2172 = Sin(var1[4]);
  t2455 = Sin(var1[5]);
  t1494 = Cos(var1[12]);
  t1533 = -1.*t1494;
  t1915 = 1. + t1533;
  t1941 = Sin(var1[12]);
  t356 = Cos(var1[4]);
  t2947 = -1.*t2094*t2230;
  t2956 = t57*t2172*t2455;
  t3018 = t2947 + t2956;
  t2203 = t57*t2094*t2172;
  t2473 = t2230*t2455;
  t2482 = t2203 + t2473;
  t3156 = -1.*t57*t356*t1302;
  t3163 = t810*t3018;
  t3232 = t3156 + t3163;
  t3417 = Cos(var1[13]);
  t3451 = -1.*t3417;
  t3498 = 1. + t3451;
  t3570 = Sin(var1[13]);
  t3714 = t810*t57*t356;
  t3741 = t1302*t3018;
  t3812 = t3714 + t3741;
  t4048 = t1494*t2482;
  t4099 = -1.*t1941*t3232;
  t4100 = t4048 + t4099;
  t4123 = Cos(var1[14]);
  t4201 = -1.*t4123;
  t4263 = 1. + t4201;
  t4455 = Sin(var1[14]);
  t4574 = t3570*t3812;
  t4611 = t3417*t4100;
  t4624 = t4574 + t4611;
  t4756 = t3417*t3812;
  t4812 = -1.*t3570*t4100;
  t4829 = t4756 + t4812;
  t4847 = Cos(var1[15]);
  t4930 = -1.*t4847;
  t4950 = 1. + t4930;
  t5145 = Sin(var1[15]);
  t5177 = -1.*t4455*t4624;
  t5179 = t4123*t4829;
  t5203 = t5177 + t5179;
  t5286 = t4123*t4624;
  t5312 = t4455*t4829;
  t5325 = t5286 + t5312;
  t1266 = -0.0222*t1256;
  t1327 = -0.087*t1302;
  t1363 = 0. + t1266 + t1327;
  t1932 = -0.3151*t1915;
  t1963 = 0.157*t1941;
  t1986 = 0. + t1932 + t1963;
  t2700 = -0.087*t1256;
  t2722 = 0.0222*t1302;
  t2814 = 0. + t2700 + t2722;
  t3029 = -0.157*t1915;
  t3111 = -0.3151*t1941;
  t3139 = 0. + t3029 + t3111;
  t5444 = t57*t2094;
  t5445 = t2230*t2172*t2455;
  t5450 = t5444 + t5445;
  t3527 = -0.0222*t3498;
  t3701 = 0.3801*t3570;
  t3711 = 0. + t3527 + t3701;
  t5407 = t2094*t2230*t2172;
  t5411 = -1.*t57*t2455;
  t5414 = t5407 + t5411;
  t5471 = -1.*t356*t1302*t2230;
  t5480 = t810*t5450;
  t5484 = t5471 + t5480;
  t4032 = -0.3801*t3498;
  t4034 = -0.0222*t3570;
  t4046 = 0. + t4032 + t4034;
  t4316 = -0.8601*t4263;
  t4463 = -0.0222*t4455;
  t4572 = 0. + t4316 + t4463;
  t5515 = t810*t356*t2230;
  t5522 = t1302*t5450;
  t5526 = t5515 + t5522;
  t5635 = t1494*t5414;
  t5645 = -1.*t1941*t5484;
  t5676 = t5635 + t5645;
  t4675 = -0.0222*t4263;
  t4709 = 0.8601*t4455;
  t4724 = 0. + t4675 + t4709;
  t5104 = -0.0211*t4950;
  t5148 = 1.3401*t5145;
  t5165 = 0. + t5104 + t5148;
  t5700 = t3570*t5526;
  t5706 = t3417*t5676;
  t5709 = t5700 + t5706;
  t5720 = t3417*t5526;
  t5721 = -1.*t3570*t5676;
  t5734 = t5720 + t5721;
  t5214 = -1.3401*t4950;
  t5215 = -0.0211*t5145;
  t5230 = 0. + t5214 + t5215;
  t5751 = -1.*t4455*t5709;
  t5756 = t4123*t5734;
  t5764 = t5751 + t5756;
  t5830 = t4123*t5709;
  t5849 = t4455*t5734;
  t5868 = t5830 + t5849;
  t5981 = t1302*t2172;
  t5985 = t810*t356*t2455;
  t5989 = t5981 + t5985;
  t6004 = -1.*t810*t2172;
  t6029 = t356*t1302*t2455;
  t6054 = t6004 + t6029;
  t6150 = t1494*t356*t2094;
  t6179 = -1.*t1941*t5989;
  t6188 = t6150 + t6179;
  t6197 = t3570*t6054;
  t6214 = t3417*t6188;
  t6220 = t6197 + t6214;
  t6229 = t3417*t6054;
  t6239 = -1.*t3570*t6188;
  t6243 = t6229 + t6239;
  t6253 = -1.*t4455*t6220;
  t6254 = t4123*t6243;
  t6263 = t6253 + t6254;
  t6268 = t4123*t6220;
  t6280 = t4455*t6243;
  t6284 = t6268 + t6280;

  p_output1(0)=0. + t1986*t2482 + t2814*t3018 + t3139*t3232 - 0.24205*(t1941*t2482 + t1494*t3232) + t3711*t3812 + t4046*t4100 + t4572*t4624 + t4724*t4829 + t5165*t5203 + t5230*t5325 - 1.325152*(t5145*t5203 + t4847*t5325) + 0.043912*(t4847*t5203 - 1.*t5145*t5325) + t1363*t356*t57 + var1(0);
  p_output1(1)=0. + t1363*t2230*t356 + t1986*t5414 + t2814*t5450 + t3139*t5484 - 0.24205*(t1941*t5414 + t1494*t5484) + t3711*t5526 + t4046*t5676 + t4572*t5709 + t4724*t5734 + t5165*t5764 + t5230*t5868 - 1.325152*(t5145*t5764 + t4847*t5868) + 0.043912*(t4847*t5764 - 1.*t5145*t5868) + var1(1);
  p_output1(2)=0. - 1.*t1363*t2172 + t1986*t2094*t356 + t2455*t2814*t356 + t3139*t5989 - 0.24205*(t1941*t2094*t356 + t1494*t5989) + t3711*t6054 + t4046*t6188 + t4572*t6220 + t4724*t6243 + t5165*t6263 + t5230*t6284 - 1.325152*(t5145*t6263 + t4847*t6284) + 0.043912*(t4847*t6263 - 1.*t5145*t6284) + var1(2);
}


       
void p_RightFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
