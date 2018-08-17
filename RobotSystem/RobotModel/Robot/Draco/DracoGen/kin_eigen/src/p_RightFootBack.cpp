/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:12 GMT-05:00
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
  double t224;
  double t306;
  double t332;
  double t333;
  double t689;
  double t1618;
  double t1767;
  double t1656;
  double t1884;
  double t851;
  double t852;
  double t950;
  double t1058;
  double t287;
  double t2343;
  double t2411;
  double t2476;
  double t1724;
  double t2103;
  double t2209;
  double t2675;
  double t2708;
  double t2722;
  double t2998;
  double t3020;
  double t3058;
  double t3143;
  double t3259;
  double t3260;
  double t3275;
  double t3380;
  double t3439;
  double t3523;
  double t3609;
  double t3636;
  double t3650;
  double t3668;
  double t3697;
  double t3718;
  double t3719;
  double t4085;
  double t4102;
  double t4106;
  double t4128;
  double t4157;
  double t4169;
  double t4188;
  double t4267;
  double t4286;
  double t4295;
  double t4398;
  double t4527;
  double t4555;
  double t619;
  double t763;
  double t819;
  double t1034;
  double t1094;
  double t1313;
  double t2286;
  double t2305;
  double t2307;
  double t2538;
  double t2582;
  double t2673;
  double t4679;
  double t4702;
  double t4726;
  double t3076;
  double t3157;
  double t3237;
  double t4663;
  double t4668;
  double t4673;
  double t4765;
  double t4768;
  double t4770;
  double t3345;
  double t3370;
  double t3375;
  double t3651;
  double t3682;
  double t3690;
  double t4807;
  double t4852;
  double t4853;
  double t4901;
  double t4907;
  double t4915;
  double t3846;
  double t3916;
  double t4047;
  double t4182;
  double t4231;
  double t4257;
  double t4962;
  double t4963;
  double t4967;
  double t4986;
  double t4999;
  double t5016;
  double t4367;
  double t4377;
  double t4380;
  double t5054;
  double t5076;
  double t5078;
  double t5093;
  double t5118;
  double t5119;
  double t5278;
  double t5313;
  double t5315;
  double t5354;
  double t5358;
  double t5365;
  double t5456;
  double t5466;
  double t5480;
  double t5489;
  double t5490;
  double t5506;
  double t5519;
  double t5544;
  double t5551;
  double t5576;
  double t5585;
  double t5643;
  double t5656;
  double t5666;
  double t5671;
  t224 = Cos(var1[3]);
  t306 = Cos(var1[11]);
  t332 = -1.*t306;
  t333 = 1. + t332;
  t689 = Sin(var1[11]);
  t1618 = Cos(var1[5]);
  t1767 = Sin(var1[3]);
  t1656 = Sin(var1[4]);
  t1884 = Sin(var1[5]);
  t851 = Cos(var1[12]);
  t852 = -1.*t851;
  t950 = 1. + t852;
  t1058 = Sin(var1[12]);
  t287 = Cos(var1[4]);
  t2343 = -1.*t1618*t1767;
  t2411 = t224*t1656*t1884;
  t2476 = t2343 + t2411;
  t1724 = t224*t1618*t1656;
  t2103 = t1767*t1884;
  t2209 = t1724 + t2103;
  t2675 = -1.*t224*t287*t689;
  t2708 = t306*t2476;
  t2722 = t2675 + t2708;
  t2998 = Cos(var1[13]);
  t3020 = -1.*t2998;
  t3058 = 1. + t3020;
  t3143 = Sin(var1[13]);
  t3259 = t306*t224*t287;
  t3260 = t689*t2476;
  t3275 = t3259 + t3260;
  t3380 = t851*t2209;
  t3439 = -1.*t1058*t2722;
  t3523 = t3380 + t3439;
  t3609 = Cos(var1[14]);
  t3636 = -1.*t3609;
  t3650 = 1. + t3636;
  t3668 = Sin(var1[14]);
  t3697 = t3143*t3275;
  t3718 = t2998*t3523;
  t3719 = t3697 + t3718;
  t4085 = t2998*t3275;
  t4102 = -1.*t3143*t3523;
  t4106 = t4085 + t4102;
  t4128 = Cos(var1[15]);
  t4157 = -1.*t4128;
  t4169 = 1. + t4157;
  t4188 = Sin(var1[15]);
  t4267 = -1.*t3668*t3719;
  t4286 = t3609*t4106;
  t4295 = t4267 + t4286;
  t4398 = t3609*t3719;
  t4527 = t3668*t4106;
  t4555 = t4398 + t4527;
  t619 = -0.022225*t333;
  t763 = -0.086996*t689;
  t819 = 0. + t619 + t763;
  t1034 = -0.31508*t950;
  t1094 = 0.156996*t1058;
  t1313 = 0. + t1034 + t1094;
  t2286 = -0.086996*t333;
  t2305 = 0.022225*t689;
  t2307 = 0. + t2286 + t2305;
  t2538 = -0.156996*t950;
  t2582 = -0.31508*t1058;
  t2673 = 0. + t2538 + t2582;
  t4679 = t224*t1618;
  t4702 = t1767*t1656*t1884;
  t4726 = t4679 + t4702;
  t3076 = -0.022225*t3058;
  t3157 = 0.38008*t3143;
  t3237 = 0. + t3076 + t3157;
  t4663 = t1618*t1767*t1656;
  t4668 = -1.*t224*t1884;
  t4673 = t4663 + t4668;
  t4765 = -1.*t287*t689*t1767;
  t4768 = t306*t4726;
  t4770 = t4765 + t4768;
  t3345 = -0.38008*t3058;
  t3370 = -0.022225*t3143;
  t3375 = 0. + t3345 + t3370;
  t3651 = -0.86008*t3650;
  t3682 = -0.022225*t3668;
  t3690 = 0. + t3651 + t3682;
  t4807 = t306*t287*t1767;
  t4852 = t689*t4726;
  t4853 = t4807 + t4852;
  t4901 = t851*t4673;
  t4907 = -1.*t1058*t4770;
  t4915 = t4901 + t4907;
  t3846 = -0.022225*t3650;
  t3916 = 0.86008*t3668;
  t4047 = 0. + t3846 + t3916;
  t4182 = -0.021147*t4169;
  t4231 = 1.34008*t4188;
  t4257 = 0. + t4182 + t4231;
  t4962 = t3143*t4853;
  t4963 = t2998*t4915;
  t4967 = t4962 + t4963;
  t4986 = t2998*t4853;
  t4999 = -1.*t3143*t4915;
  t5016 = t4986 + t4999;
  t4367 = -1.34008*t4169;
  t4377 = -0.021147*t4188;
  t4380 = 0. + t4367 + t4377;
  t5054 = -1.*t3668*t4967;
  t5076 = t3609*t5016;
  t5078 = t5054 + t5076;
  t5093 = t3609*t4967;
  t5118 = t3668*t5016;
  t5119 = t5093 + t5118;
  t5278 = t689*t1656;
  t5313 = t306*t287*t1884;
  t5315 = t5278 + t5313;
  t5354 = -1.*t306*t1656;
  t5358 = t287*t689*t1884;
  t5365 = t5354 + t5358;
  t5456 = t851*t287*t1618;
  t5466 = -1.*t1058*t5315;
  t5480 = t5456 + t5466;
  t5489 = t3143*t5365;
  t5490 = t2998*t5480;
  t5506 = t5489 + t5490;
  t5519 = t2998*t5365;
  t5544 = -1.*t3143*t5480;
  t5551 = t5519 + t5544;
  t5576 = -1.*t3668*t5506;
  t5585 = t3609*t5551;
  t5643 = t5576 + t5585;
  t5656 = t3609*t5506;
  t5666 = t3668*t5551;
  t5671 = t5656 + t5666;

  p_output1(0)=0. + t1313*t2209 + t2307*t2476 + t2673*t2722 + t3237*t3275 + t3375*t3523 + t3690*t3719 + t4047*t4106 + t4257*t4295 + t4380*t4555 - 1.400132*(t4188*t4295 + t4128*t4555) + 0.043805*(t4128*t4295 - 1.*t4188*t4555) + t224*t287*t819 - 0.166996*(t1058*t2209 + t2722*t851) + var1(0);
  p_output1(1)=0. + t1313*t4673 + t2307*t4726 + t2673*t4770 + t3237*t4853 + t3375*t4915 + t3690*t4967 + t4047*t5016 + t4257*t5078 + t4380*t5119 - 1.400132*(t4188*t5078 + t4128*t5119) + 0.043805*(t4128*t5078 - 1.*t4188*t5119) + t1767*t287*t819 - 0.166996*(t1058*t4673 + t4770*t851) + var1(1);
  p_output1(2)=0. + t1313*t1618*t287 + t1884*t2307*t287 + t2673*t5315 + t3237*t5365 + t3375*t5480 + t3690*t5506 + t4047*t5551 + t4257*t5643 + t4380*t5671 - 1.400132*(t4188*t5643 + t4128*t5671) + 0.043805*(t4128*t5643 - 1.*t4188*t5671) - 1.*t1656*t819 - 0.166996*(t1058*t1618*t287 + t5315*t851) + var1(2);
}


       
void p_RightFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
