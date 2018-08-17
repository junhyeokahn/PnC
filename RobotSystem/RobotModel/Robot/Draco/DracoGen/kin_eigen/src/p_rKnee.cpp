/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rKnee.h"

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
  double t343;
  double t616;
  double t1617;
  double t2115;
  double t2736;
  double t6336;
  double t6501;
  double t6381;
  double t6630;
  double t5298;
  double t5374;
  double t5468;
  double t5932;
  double t611;
  double t6695;
  double t6722;
  double t6754;
  double t6451;
  double t6641;
  double t6654;
  double t6802;
  double t6807;
  double t6831;
  double t6889;
  double t6894;
  double t6895;
  double t6909;
  double t6918;
  double t6924;
  double t6956;
  double t7120;
  double t7122;
  double t7123;
  double t7176;
  double t7183;
  double t7184;
  double t7200;
  double t7218;
  double t7222;
  double t7231;
  double t7273;
  double t7276;
  double t7287;
  double t2645;
  double t3545;
  double t4700;
  double t5485;
  double t6089;
  double t6205;
  double t6669;
  double t6679;
  double t6681;
  double t6777;
  double t6780;
  double t6790;
  double t7432;
  double t7448;
  double t7449;
  double t6900;
  double t6912;
  double t6916;
  double t7399;
  double t7402;
  double t7418;
  double t7469;
  double t7473;
  double t7476;
  double t7032;
  double t7071;
  double t7119;
  double t7187;
  double t7205;
  double t7207;
  double t7480;
  double t7482;
  double t7485;
  double t7495;
  double t7511;
  double t7524;
  double t7266;
  double t7271;
  double t7272;
  double t7534;
  double t7545;
  double t7564;
  double t7566;
  double t7572;
  double t7578;
  double t7660;
  double t7661;
  double t7664;
  double t7666;
  double t7668;
  double t7670;
  double t7704;
  double t7705;
  double t7708;
  double t7711;
  double t7715;
  double t7721;
  double t7747;
  double t7756;
  double t7760;
  t343 = Cos(var1[3]);
  t616 = Cos(var1[11]);
  t1617 = -1.*t616;
  t2115 = 1. + t1617;
  t2736 = Sin(var1[11]);
  t6336 = Cos(var1[5]);
  t6501 = Sin(var1[3]);
  t6381 = Sin(var1[4]);
  t6630 = Sin(var1[5]);
  t5298 = Cos(var1[12]);
  t5374 = -1.*t5298;
  t5468 = 1. + t5374;
  t5932 = Sin(var1[12]);
  t611 = Cos(var1[4]);
  t6695 = -1.*t6336*t6501;
  t6722 = t343*t6381*t6630;
  t6754 = t6695 + t6722;
  t6451 = t343*t6336*t6381;
  t6641 = t6501*t6630;
  t6654 = t6451 + t6641;
  t6802 = -1.*t343*t611*t2736;
  t6807 = t616*t6754;
  t6831 = t6802 + t6807;
  t6889 = Cos(var1[13]);
  t6894 = -1.*t6889;
  t6895 = 1. + t6894;
  t6909 = Sin(var1[13]);
  t6918 = t616*t343*t611;
  t6924 = t2736*t6754;
  t6956 = t6918 + t6924;
  t7120 = t5298*t6654;
  t7122 = -1.*t5932*t6831;
  t7123 = t7120 + t7122;
  t7176 = Cos(var1[14]);
  t7183 = -1.*t7176;
  t7184 = 1. + t7183;
  t7200 = Sin(var1[14]);
  t7218 = t6909*t6956;
  t7222 = t6889*t7123;
  t7231 = t7218 + t7222;
  t7273 = t6889*t6956;
  t7276 = -1.*t6909*t7123;
  t7287 = t7273 + t7276;
  t2645 = -0.022225*t2115;
  t3545 = -0.086996*t2736;
  t4700 = 0. + t2645 + t3545;
  t5485 = -0.31508*t5468;
  t6089 = 0.156996*t5932;
  t6205 = 0. + t5485 + t6089;
  t6669 = -0.086996*t2115;
  t6679 = 0.022225*t2736;
  t6681 = 0. + t6669 + t6679;
  t6777 = -0.156996*t5468;
  t6780 = -0.31508*t5932;
  t6790 = 0. + t6777 + t6780;
  t7432 = t343*t6336;
  t7448 = t6501*t6381*t6630;
  t7449 = t7432 + t7448;
  t6900 = -0.022225*t6895;
  t6912 = 0.38008*t6909;
  t6916 = 0. + t6900 + t6912;
  t7399 = t6336*t6501*t6381;
  t7402 = -1.*t343*t6630;
  t7418 = t7399 + t7402;
  t7469 = -1.*t611*t2736*t6501;
  t7473 = t616*t7449;
  t7476 = t7469 + t7473;
  t7032 = -0.38008*t6895;
  t7071 = -0.022225*t6909;
  t7119 = 0. + t7032 + t7071;
  t7187 = -0.86008*t7184;
  t7205 = -0.022225*t7200;
  t7207 = 0. + t7187 + t7205;
  t7480 = t616*t611*t6501;
  t7482 = t2736*t7449;
  t7485 = t7480 + t7482;
  t7495 = t5298*t7418;
  t7511 = -1.*t5932*t7476;
  t7524 = t7495 + t7511;
  t7266 = -0.022225*t7184;
  t7271 = 0.86008*t7200;
  t7272 = 0. + t7266 + t7271;
  t7534 = t6909*t7485;
  t7545 = t6889*t7524;
  t7564 = t7534 + t7545;
  t7566 = t6889*t7485;
  t7572 = -1.*t6909*t7524;
  t7578 = t7566 + t7572;
  t7660 = t2736*t6381;
  t7661 = t616*t611*t6630;
  t7664 = t7660 + t7661;
  t7666 = -1.*t616*t6381;
  t7668 = t611*t2736*t6630;
  t7670 = t7666 + t7668;
  t7704 = t5298*t611*t6336;
  t7705 = -1.*t5932*t7664;
  t7708 = t7704 + t7705;
  t7711 = t6909*t7670;
  t7715 = t6889*t7708;
  t7721 = t7711 + t7715;
  t7747 = t6889*t7670;
  t7756 = -1.*t6909*t7708;
  t7760 = t7747 + t7756;

  p_output1(0)=0. + t343*t4700*t611 + t6205*t6654 + t6681*t6754 + t6790*t6831 - 0.150246*(t5932*t6654 + t5298*t6831) + t6916*t6956 + t7119*t7123 + t7207*t7231 + t7272*t7287 - 0.022225*(-1.*t7200*t7231 + t7176*t7287) - 0.86008*(t7176*t7231 + t7200*t7287) + var1(0);
  p_output1(1)=0. + t4700*t611*t6501 + t6205*t7418 + t6681*t7449 + t6790*t7476 - 0.150246*(t5932*t7418 + t5298*t7476) + t6916*t7485 + t7119*t7524 + t7207*t7564 + t7272*t7578 - 0.022225*(-1.*t7200*t7564 + t7176*t7578) - 0.86008*(t7176*t7564 + t7200*t7578) + var1(1);
  p_output1(2)=0. + t611*t6205*t6336 - 1.*t4700*t6381 + t611*t6630*t6681 + t6790*t7664 - 0.150246*(t5932*t611*t6336 + t5298*t7664) + t6916*t7670 + t7119*t7708 + t7207*t7721 + t7272*t7760 - 0.022225*(-1.*t7200*t7721 + t7176*t7760) - 0.86008*(t7176*t7721 + t7200*t7760) + var1(2);
}


       
void p_rKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
