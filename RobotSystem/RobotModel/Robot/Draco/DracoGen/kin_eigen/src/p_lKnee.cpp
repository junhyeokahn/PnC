/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:38 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lKnee.h"

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
  double t1466;
  double t2993;
  double t4204;
  double t4725;
  double t4821;
  double t348;
  double t358;
  double t580;
  double t1729;
  double t2881;
  double t2915;
  double t2989;
  double t5713;
  double t5856;
  double t5862;
  double t5909;
  double t5930;
  double t5840;
  double t5841;
  double t5844;
  double t6003;
  double t6010;
  double t6014;
  double t6146;
  double t6149;
  double t6151;
  double t6170;
  double t6100;
  double t6102;
  double t6138;
  double t6235;
  double t6247;
  double t6262;
  double t6301;
  double t6303;
  double t6304;
  double t6318;
  double t6337;
  double t6346;
  double t6347;
  double t6369;
  double t6379;
  double t6393;
  double t4815;
  double t4825;
  double t5369;
  double t5722;
  double t5763;
  double t5775;
  double t6453;
  double t6456;
  double t6458;
  double t5926;
  double t5970;
  double t5982;
  double t6020;
  double t6027;
  double t6030;
  double t6486;
  double t6487;
  double t6493;
  double t6503;
  double t6509;
  double t6511;
  double t6158;
  double t6173;
  double t6220;
  double t6265;
  double t6267;
  double t6281;
  double t6308;
  double t6326;
  double t6331;
  double t6536;
  double t6541;
  double t6554;
  double t6557;
  double t6558;
  double t6565;
  double t6350;
  double t6353;
  double t6357;
  double t6580;
  double t6594;
  double t6606;
  double t6628;
  double t6630;
  double t6638;
  double t6715;
  double t6726;
  double t6737;
  double t6773;
  double t6777;
  double t6781;
  double t6791;
  double t6792;
  double t6793;
  double t6800;
  double t6807;
  double t6809;
  double t6821;
  double t6824;
  double t6826;
  t1466 = Cos(var1[3]);
  t2993 = Cos(var1[6]);
  t4204 = -1.*t2993;
  t4725 = 1. + t4204;
  t4821 = Sin(var1[6]);
  t348 = Cos(var1[5]);
  t358 = Sin(var1[3]);
  t580 = -1.*t348*t358;
  t1729 = Sin(var1[4]);
  t2881 = Sin(var1[5]);
  t2915 = t1466*t1729*t2881;
  t2989 = t580 + t2915;
  t5713 = Cos(var1[4]);
  t5856 = Cos(var1[7]);
  t5862 = -1.*t5856;
  t5909 = 1. + t5862;
  t5930 = Sin(var1[7]);
  t5840 = t2993*t2989;
  t5841 = -1.*t1466*t5713*t4821;
  t5844 = t5840 + t5841;
  t6003 = t1466*t348*t1729;
  t6010 = t358*t2881;
  t6014 = t6003 + t6010;
  t6146 = Cos(var1[8]);
  t6149 = -1.*t6146;
  t6151 = 1. + t6149;
  t6170 = Sin(var1[8]);
  t6100 = t5856*t6014;
  t6102 = -1.*t5844*t5930;
  t6138 = t6100 + t6102;
  t6235 = t1466*t5713*t2993;
  t6247 = t2989*t4821;
  t6262 = t6235 + t6247;
  t6301 = Cos(var1[9]);
  t6303 = -1.*t6301;
  t6304 = 1. + t6303;
  t6318 = Sin(var1[9]);
  t6337 = t6146*t6138;
  t6346 = t6262*t6170;
  t6347 = t6337 + t6346;
  t6369 = t6146*t6262;
  t6379 = -1.*t6138*t6170;
  t6393 = t6369 + t6379;
  t4815 = 0.087004*t4725;
  t4825 = 0.022225*t4821;
  t5369 = 0. + t4815 + t4825;
  t5722 = -0.022225*t4725;
  t5763 = 0.087004*t4821;
  t5775 = 0. + t5722 + t5763;
  t6453 = t1466*t348;
  t6456 = t358*t1729*t2881;
  t6458 = t6453 + t6456;
  t5926 = 0.157004*t5909;
  t5970 = -0.31508*t5930;
  t5982 = 0. + t5926 + t5970;
  t6020 = -0.31508*t5909;
  t6027 = -0.157004*t5930;
  t6030 = 0. + t6020 + t6027;
  t6486 = t2993*t6458;
  t6487 = -1.*t5713*t358*t4821;
  t6493 = t6486 + t6487;
  t6503 = t348*t358*t1729;
  t6509 = -1.*t1466*t2881;
  t6511 = t6503 + t6509;
  t6158 = -0.38008*t6151;
  t6173 = -0.022225*t6170;
  t6220 = 0. + t6158 + t6173;
  t6265 = -0.022225*t6151;
  t6267 = 0.38008*t6170;
  t6281 = 0. + t6265 + t6267;
  t6308 = -0.86008*t6304;
  t6326 = -0.022225*t6318;
  t6331 = 0. + t6308 + t6326;
  t6536 = t5856*t6511;
  t6541 = -1.*t6493*t5930;
  t6554 = t6536 + t6541;
  t6557 = t5713*t2993*t358;
  t6558 = t6458*t4821;
  t6565 = t6557 + t6558;
  t6350 = -0.022225*t6304;
  t6353 = 0.86008*t6318;
  t6357 = 0. + t6350 + t6353;
  t6580 = t6146*t6554;
  t6594 = t6565*t6170;
  t6606 = t6580 + t6594;
  t6628 = t6146*t6565;
  t6630 = -1.*t6554*t6170;
  t6638 = t6628 + t6630;
  t6715 = t5713*t2993*t2881;
  t6726 = t1729*t4821;
  t6737 = t6715 + t6726;
  t6773 = t5713*t348*t5856;
  t6777 = -1.*t6737*t5930;
  t6781 = t6773 + t6777;
  t6791 = -1.*t2993*t1729;
  t6792 = t5713*t2881*t4821;
  t6793 = t6791 + t6792;
  t6800 = t6146*t6781;
  t6807 = t6793*t6170;
  t6809 = t6800 + t6807;
  t6821 = t6146*t6793;
  t6824 = -1.*t6781*t6170;
  t6826 = t6821 + t6824;

  p_output1(0)=0. + t2989*t5369 + t1466*t5713*t5775 + t5844*t5982 + 0.150254*(t5844*t5856 + t5930*t6014) + t6014*t6030 + t6138*t6220 + t6262*t6281 + t6331*t6347 + t6357*t6393 - 0.022225*(-1.*t6318*t6347 + t6301*t6393) - 0.86008*(t6301*t6347 + t6318*t6393) + var1(0);
  p_output1(1)=0. + t358*t5713*t5775 + t5369*t6458 + t5982*t6493 + t6030*t6511 + 0.150254*(t5856*t6493 + t5930*t6511) + t6220*t6554 + t6281*t6565 + t6331*t6606 + t6357*t6638 - 0.022225*(-1.*t6318*t6606 + t6301*t6638) - 0.86008*(t6301*t6606 + t6318*t6638) + var1(1);
  p_output1(2)=0. + t2881*t5369*t5713 - 1.*t1729*t5775 + t348*t5713*t6030 + t5982*t6737 + 0.150254*(t348*t5713*t5930 + t5856*t6737) + t6220*t6781 + t6281*t6793 + t6331*t6809 + t6357*t6826 - 0.022225*(-1.*t6318*t6809 + t6301*t6826) - 0.86008*(t6301*t6809 + t6318*t6826) + var1(2);
}


       
void p_lKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
