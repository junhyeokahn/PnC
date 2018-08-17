/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:37 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipPitch.h"

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
  double t2516;
  double t3429;
  double t3800;
  double t3667;
  double t3812;
  double t3165;
  double t3699;
  double t4067;
  double t4186;
  double t3152;
  double t4330;
  double t1691;
  double t4726;
  double t5189;
  double t5202;
  double t5253;
  double t4941;
  double t5012;
  double t4879;
  double t4902;
  double t4903;
  double t4765;
  double t4771;
  double t4864;
  double t5577;
  double t5583;
  double t5617;
  double t5449;
  double t5453;
  double t5466;
  double t5802;
  double t5820;
  double t5826;
  double t4878;
  double t4954;
  double t5005;
  double t3284;
  double t4375;
  double t4433;
  double t5500;
  double t5643;
  double t5651;
  double t5181;
  double t5326;
  double t5397;
  double t5788;
  double t5828;
  double t5835;
  double t5741;
  double t5743;
  double t5772;
  t2516 = Cos(var1[3]);
  t3429 = Cos(var1[5]);
  t3800 = Sin(var1[4]);
  t3667 = Sin(var1[3]);
  t3812 = Sin(var1[5]);
  t3165 = Cos(var1[6]);
  t3699 = -1.*t3429*t3667;
  t4067 = t2516*t3800*t3812;
  t4186 = t3699 + t4067;
  t3152 = Cos(var1[4]);
  t4330 = Sin(var1[6]);
  t1691 = Cos(var1[8]);
  t4726 = Cos(var1[7]);
  t5189 = t2516*t3429;
  t5202 = t3667*t3800*t3812;
  t5253 = t5189 + t5202;
  t4941 = Sin(var1[7]);
  t5012 = Sin(var1[8]);
  t4879 = t3165*t4186;
  t4902 = -1.*t2516*t3152*t4330;
  t4903 = t4879 + t4902;
  t4765 = t2516*t3429*t3800;
  t4771 = t3667*t3812;
  t4864 = t4765 + t4771;
  t5577 = t3165*t5253;
  t5583 = -1.*t3152*t3667*t4330;
  t5617 = t5577 + t5583;
  t5449 = t3429*t3667*t3800;
  t5453 = -1.*t2516*t3812;
  t5466 = t5449 + t5453;
  t5802 = t3152*t3165*t3812;
  t5820 = t3800*t4330;
  t5826 = t5802 + t5820;
  t4878 = t4726*t4864;
  t4954 = -1.*t4903*t4941;
  t5005 = t4878 + t4954;
  t3284 = t2516*t3152*t3165;
  t4375 = t4186*t4330;
  t4433 = t3284 + t4375;
  t5500 = t4726*t5466;
  t5643 = -1.*t5617*t4941;
  t5651 = t5500 + t5643;
  t5181 = t3152*t3165*t3667;
  t5326 = t5253*t4330;
  t5397 = t5181 + t5326;
  t5788 = t3152*t3429*t4726;
  t5828 = -1.*t5826*t4941;
  t5835 = t5788 + t5828;
  t5741 = -1.*t3165*t3800;
  t5743 = t3152*t3812*t4330;
  t5772 = t5741 + t5743;

  p_output1(0)=t1691*t4433 - 1.*t5005*t5012;
  p_output1(1)=t1691*t5397 - 1.*t5012*t5651;
  p_output1(2)=t1691*t5772 - 1.*t5012*t5835;
  p_output1(3)=t4726*t4903 + t4864*t4941;
  p_output1(4)=t4941*t5466 + t4726*t5617;
  p_output1(5)=t3152*t3429*t4941 + t4726*t5826;
  p_output1(6)=t1691*t5005 + t4433*t5012;
  p_output1(7)=t5012*t5397 + t1691*t5651;
  p_output1(8)=t5012*t5772 + t1691*t5835;
}


       
void R_lHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
