/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:03 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootFront.h"

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
  double t1628;
  double t1878;
  double t2032;
  double t1959;
  double t2139;
  double t1662;
  double t1850;
  double t1594;
  double t2015;
  double t2268;
  double t2403;
  double t2629;
  double t1793;
  double t2463;
  double t2520;
  double t1309;
  double t2641;
  double t2713;
  double t2784;
  double t2814;
  double t2861;
  double t2948;
  double t2968;
  double t2993;
  double t3023;
  double t3174;
  double t3333;
  double t3377;
  double t2541;
  double t3335;
  double t3370;
  double t895;
  double t3379;
  double t3388;
  double t3390;
  double t3514;
  double t3374;
  double t3417;
  double t3443;
  double t278;
  double t3533;
  double t3551;
  double t3569;
  double t3890;
  double t3963;
  double t4032;
  double t3848;
  double t4069;
  double t4103;
  double t4172;
  double t4187;
  double t4210;
  double t4224;
  double t4232;
  double t4235;
  double t4240;
  double t4257;
  double t4269;
  double t4162;
  double t4367;
  double t4382;
  double t4474;
  double t4495;
  double t4517;
  double t4445;
  double t4524;
  double t4537;
  double t4580;
  double t4602;
  double t4615;
  double t4777;
  double t4809;
  double t4818;
  double t4852;
  double t4876;
  double t4968;
  double t4988;
  double t5009;
  double t5011;
  double t4827;
  double t5031;
  double t5044;
  double t5052;
  double t5081;
  double t5101;
  double t5048;
  double t5124;
  double t5127;
  double t5158;
  double t5164;
  double t5169;
  double t3480;
  double t3668;
  double t4571;
  double t4628;
  double t5135;
  double t5180;
  t1628 = Cos(var1[3]);
  t1878 = Cos(var1[5]);
  t2032 = Sin(var1[4]);
  t1959 = Sin(var1[3]);
  t2139 = Sin(var1[5]);
  t1662 = Cos(var1[4]);
  t1850 = Sin(var1[11]);
  t1594 = Cos(var1[11]);
  t2015 = -1.*t1878*t1959;
  t2268 = t1628*t2032*t2139;
  t2403 = t2015 + t2268;
  t2629 = Cos(var1[13]);
  t1793 = t1594*t1628*t1662;
  t2463 = t1850*t2403;
  t2520 = t1793 + t2463;
  t1309 = Sin(var1[13]);
  t2641 = Cos(var1[12]);
  t2713 = t1628*t1878*t2032;
  t2784 = t1959*t2139;
  t2814 = t2713 + t2784;
  t2861 = t2641*t2814;
  t2948 = Sin(var1[12]);
  t2968 = -1.*t1628*t1662*t1850;
  t2993 = t1594*t2403;
  t3023 = t2968 + t2993;
  t3174 = -1.*t2948*t3023;
  t3333 = t2861 + t3174;
  t3377 = Cos(var1[14]);
  t2541 = t1309*t2520;
  t3335 = t2629*t3333;
  t3370 = t2541 + t3335;
  t895 = Sin(var1[14]);
  t3379 = t2629*t2520;
  t3388 = -1.*t1309*t3333;
  t3390 = t3379 + t3388;
  t3514 = Cos(var1[15]);
  t3374 = -1.*t895*t3370;
  t3417 = t3377*t3390;
  t3443 = t3374 + t3417;
  t278 = Sin(var1[15]);
  t3533 = t3377*t3370;
  t3551 = t895*t3390;
  t3569 = t3533 + t3551;
  t3890 = t1628*t1878;
  t3963 = t1959*t2032*t2139;
  t4032 = t3890 + t3963;
  t3848 = t1594*t1662*t1959;
  t4069 = t1850*t4032;
  t4103 = t3848 + t4069;
  t4172 = t1878*t1959*t2032;
  t4187 = -1.*t1628*t2139;
  t4210 = t4172 + t4187;
  t4224 = t2641*t4210;
  t4232 = -1.*t1662*t1850*t1959;
  t4235 = t1594*t4032;
  t4240 = t4232 + t4235;
  t4257 = -1.*t2948*t4240;
  t4269 = t4224 + t4257;
  t4162 = t1309*t4103;
  t4367 = t2629*t4269;
  t4382 = t4162 + t4367;
  t4474 = t2629*t4103;
  t4495 = -1.*t1309*t4269;
  t4517 = t4474 + t4495;
  t4445 = -1.*t895*t4382;
  t4524 = t3377*t4517;
  t4537 = t4445 + t4524;
  t4580 = t3377*t4382;
  t4602 = t895*t4517;
  t4615 = t4580 + t4602;
  t4777 = -1.*t1594*t2032;
  t4809 = t1662*t1850*t2139;
  t4818 = t4777 + t4809;
  t4852 = t2641*t1662*t1878;
  t4876 = t1850*t2032;
  t4968 = t1594*t1662*t2139;
  t4988 = t4876 + t4968;
  t5009 = -1.*t2948*t4988;
  t5011 = t4852 + t5009;
  t4827 = t1309*t4818;
  t5031 = t2629*t5011;
  t5044 = t4827 + t5031;
  t5052 = t2629*t4818;
  t5081 = -1.*t1309*t5011;
  t5101 = t5052 + t5081;
  t5048 = -1.*t895*t5044;
  t5124 = t3377*t5101;
  t5127 = t5048 + t5124;
  t5158 = t3377*t5044;
  t5164 = t895*t5101;
  t5169 = t5158 + t5164;
  t3480 = t278*t3443;
  t3668 = t3514*t3569;
  t4571 = t278*t4537;
  t4628 = t3514*t4615;
  t5135 = t278*t5127;
  t5180 = t3514*t5169;

  p_output1(0)=t3480 + 0.000796*(t3443*t3514 - 1.*t278*t3569) + t3668;
  p_output1(1)=t4571 + 0.000796*(t3514*t4537 - 1.*t278*t4615) + t4628;
  p_output1(2)=t5135 + 0.000796*(t3514*t5127 - 1.*t278*t5169) + t5180;
  p_output1(3)=t2814*t2948 + t2641*t3023;
  p_output1(4)=t2948*t4210 + t2641*t4240;
  p_output1(5)=t1662*t1878*t2948 + t2641*t4988;
  p_output1(6)=-1.*t3443*t3514 + t278*t3569 + 0.000796*(t3480 + t3668);
  p_output1(7)=-1.*t3514*t4537 + t278*t4615 + 0.000796*(t4571 + t4628);
  p_output1(8)=-1.*t3514*t5127 + t278*t5169 + 0.000796*(t5135 + t5180);
}


       
void R_RightFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
