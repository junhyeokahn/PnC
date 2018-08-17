/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:04 GMT-05:00
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
  double t568;
  double t1196;
  double t1537;
  double t1544;
  double t1668;
  double t2784;
  double t3009;
  double t2861;
  double t3125;
  double t2170;
  double t2268;
  double t2403;
  double t2520;
  double t938;
  double t3417;
  double t3480;
  double t3533;
  double t2896;
  double t3149;
  double t3172;
  double t3792;
  double t3848;
  double t3963;
  double t4069;
  double t4103;
  double t4162;
  double t4210;
  double t4367;
  double t4471;
  double t4474;
  double t4827;
  double t4852;
  double t4876;
  double t5031;
  double t5101;
  double t5124;
  double t5222;
  double t5312;
  double t5330;
  double t5334;
  double t5389;
  double t5410;
  double t5451;
  double t5466;
  double t5475;
  double t5476;
  double t5482;
  double t5508;
  double t5513;
  double t5521;
  double t5563;
  double t5574;
  double t5593;
  double t1632;
  double t1988;
  double t1992;
  double t2409;
  double t2676;
  double t2779;
  double t3373;
  double t3377;
  double t3390;
  double t3611;
  double t3643;
  double t3688;
  double t5842;
  double t5849;
  double t5864;
  double t4172;
  double t4224;
  double t4235;
  double t5729;
  double t5760;
  double t5768;
  double t5885;
  double t5898;
  double t5899;
  double t4809;
  double t4811;
  double t4812;
  double t5180;
  double t5234;
  double t5278;
  double t5902;
  double t5909;
  double t5918;
  double t5991;
  double t5998;
  double t6024;
  double t5350;
  double t5356;
  double t5357;
  double t5480;
  double t5484;
  double t5506;
  double t6041;
  double t6073;
  double t6076;
  double t6092;
  double t6095;
  double t6099;
  double t5527;
  double t5543;
  double t5561;
  double t6102;
  double t6117;
  double t6122;
  double t6137;
  double t6142;
  double t6160;
  double t6324;
  double t6333;
  double t6334;
  double t6352;
  double t6363;
  double t6368;
  double t6431;
  double t6449;
  double t6463;
  double t6474;
  double t6504;
  double t6514;
  double t6559;
  double t6582;
  double t6588;
  double t6603;
  double t6606;
  double t6617;
  double t6624;
  double t6627;
  double t6663;
  t568 = Cos(var1[3]);
  t1196 = Cos(var1[11]);
  t1537 = -1.*t1196;
  t1544 = 1. + t1537;
  t1668 = Sin(var1[11]);
  t2784 = Cos(var1[5]);
  t3009 = Sin(var1[3]);
  t2861 = Sin(var1[4]);
  t3125 = Sin(var1[5]);
  t2170 = Cos(var1[12]);
  t2268 = -1.*t2170;
  t2403 = 1. + t2268;
  t2520 = Sin(var1[12]);
  t938 = Cos(var1[4]);
  t3417 = -1.*t2784*t3009;
  t3480 = t568*t2861*t3125;
  t3533 = t3417 + t3480;
  t2896 = t568*t2784*t2861;
  t3149 = t3009*t3125;
  t3172 = t2896 + t3149;
  t3792 = -1.*t568*t938*t1668;
  t3848 = t1196*t3533;
  t3963 = t3792 + t3848;
  t4069 = Cos(var1[13]);
  t4103 = -1.*t4069;
  t4162 = 1. + t4103;
  t4210 = Sin(var1[13]);
  t4367 = t1196*t568*t938;
  t4471 = t1668*t3533;
  t4474 = t4367 + t4471;
  t4827 = t2170*t3172;
  t4852 = -1.*t2520*t3963;
  t4876 = t4827 + t4852;
  t5031 = Cos(var1[14]);
  t5101 = -1.*t5031;
  t5124 = 1. + t5101;
  t5222 = Sin(var1[14]);
  t5312 = t4210*t4474;
  t5330 = t4069*t4876;
  t5334 = t5312 + t5330;
  t5389 = t4069*t4474;
  t5410 = -1.*t4210*t4876;
  t5451 = t5389 + t5410;
  t5466 = Cos(var1[15]);
  t5475 = -1.*t5466;
  t5476 = 1. + t5475;
  t5482 = Sin(var1[15]);
  t5508 = -1.*t5222*t5334;
  t5513 = t5031*t5451;
  t5521 = t5508 + t5513;
  t5563 = t5031*t5334;
  t5574 = t5222*t5451;
  t5593 = t5563 + t5574;
  t1632 = -0.022225*t1544;
  t1988 = -0.086996*t1668;
  t1992 = 0. + t1632 + t1988;
  t2409 = -0.31508*t2403;
  t2676 = 0.156996*t2520;
  t2779 = 0. + t2409 + t2676;
  t3373 = -0.086996*t1544;
  t3377 = 0.022225*t1668;
  t3390 = 0. + t3373 + t3377;
  t3611 = -0.156996*t2403;
  t3643 = -0.31508*t2520;
  t3688 = 0. + t3611 + t3643;
  t5842 = t568*t2784;
  t5849 = t3009*t2861*t3125;
  t5864 = t5842 + t5849;
  t4172 = -0.022225*t4162;
  t4224 = 0.38008*t4210;
  t4235 = 0. + t4172 + t4224;
  t5729 = t2784*t3009*t2861;
  t5760 = -1.*t568*t3125;
  t5768 = t5729 + t5760;
  t5885 = -1.*t938*t1668*t3009;
  t5898 = t1196*t5864;
  t5899 = t5885 + t5898;
  t4809 = -0.38008*t4162;
  t4811 = -0.022225*t4210;
  t4812 = 0. + t4809 + t4811;
  t5180 = -0.86008*t5124;
  t5234 = -0.022225*t5222;
  t5278 = 0. + t5180 + t5234;
  t5902 = t1196*t938*t3009;
  t5909 = t1668*t5864;
  t5918 = t5902 + t5909;
  t5991 = t2170*t5768;
  t5998 = -1.*t2520*t5899;
  t6024 = t5991 + t5998;
  t5350 = -0.022225*t5124;
  t5356 = 0.86008*t5222;
  t5357 = 0. + t5350 + t5356;
  t5480 = -0.021147*t5476;
  t5484 = 1.34008*t5482;
  t5506 = 0. + t5480 + t5484;
  t6041 = t4210*t5918;
  t6073 = t4069*t6024;
  t6076 = t6041 + t6073;
  t6092 = t4069*t5918;
  t6095 = -1.*t4210*t6024;
  t6099 = t6092 + t6095;
  t5527 = -1.34008*t5476;
  t5543 = -0.021147*t5482;
  t5561 = 0. + t5527 + t5543;
  t6102 = -1.*t5222*t6076;
  t6117 = t5031*t6099;
  t6122 = t6102 + t6117;
  t6137 = t5031*t6076;
  t6142 = t5222*t6099;
  t6160 = t6137 + t6142;
  t6324 = t1668*t2861;
  t6333 = t1196*t938*t3125;
  t6334 = t6324 + t6333;
  t6352 = -1.*t1196*t2861;
  t6363 = t938*t1668*t3125;
  t6368 = t6352 + t6363;
  t6431 = t2170*t938*t2784;
  t6449 = -1.*t2520*t6334;
  t6463 = t6431 + t6449;
  t6474 = t4210*t6368;
  t6504 = t4069*t6463;
  t6514 = t6474 + t6504;
  t6559 = t4069*t6368;
  t6582 = -1.*t4210*t6463;
  t6588 = t6559 + t6582;
  t6603 = -1.*t5222*t6514;
  t6606 = t5031*t6588;
  t6617 = t6603 + t6606;
  t6624 = t5031*t6514;
  t6627 = t5222*t6588;
  t6663 = t6624 + t6627;

  p_output1(0)=0. + t2779*t3172 + t3390*t3533 + t3688*t3963 - 0.166996*(t2520*t3172 + t2170*t3963) + t4235*t4474 + t4812*t4876 + t5278*t5334 + t5357*t5451 + t5506*t5521 + t5561*t5593 - 1.400132*(t5482*t5521 + t5466*t5593) + 0.043805*(t5466*t5521 - 1.*t5482*t5593) + t1992*t568*t938 + var1(0);
  p_output1(1)=0. + t2779*t5768 + t3390*t5864 + t3688*t5899 - 0.166996*(t2520*t5768 + t2170*t5899) + t4235*t5918 + t4812*t6024 + t5278*t6076 + t5357*t6099 + t5506*t6122 + t5561*t6160 - 1.400132*(t5482*t6122 + t5466*t6160) + 0.043805*(t5466*t6122 - 1.*t5482*t6160) + t1992*t3009*t938 + var1(1);
  p_output1(2)=0. - 1.*t1992*t2861 + t3688*t6334 + t4235*t6368 + t4812*t6463 + t5278*t6514 + t5357*t6588 + t5506*t6617 + t5561*t6663 - 1.400132*(t5482*t6617 + t5466*t6663) + 0.043805*(t5466*t6617 - 1.*t5482*t6663) + t2779*t2784*t938 + t3125*t3390*t938 - 0.166996*(t2170*t6334 + t2520*t2784*t938) + var1(2);
}


       
void p_RightFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
