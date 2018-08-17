/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:57 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipPitch.h"

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
  double t1432;
  double t1613;
  double t1644;
  double t1788;
  double t1967;
  double t5302;
  double t5674;
  double t5331;
  double t5751;
  double t3162;
  double t3273;
  double t4301;
  double t4553;
  double t1564;
  double t6134;
  double t6199;
  double t6200;
  double t5575;
  double t5958;
  double t6056;
  double t6267;
  double t6280;
  double t6294;
  double t6310;
  double t6326;
  double t6332;
  double t6369;
  double t6392;
  double t6393;
  double t6428;
  double t6520;
  double t6525;
  double t6526;
  double t1815;
  double t2383;
  double t2588;
  double t4336;
  double t5048;
  double t5173;
  double t6104;
  double t6118;
  double t6127;
  double t6227;
  double t6236;
  double t6237;
  double t6679;
  double t6680;
  double t6681;
  double t6360;
  double t6387;
  double t6391;
  double t6659;
  double t6666;
  double t6669;
  double t6698;
  double t6722;
  double t6738;
  double t6511;
  double t6513;
  double t6517;
  double t6769;
  double t6772;
  double t6776;
  double t6823;
  double t6826;
  double t6830;
  double t6916;
  double t6918;
  double t6945;
  double t6963;
  double t6969;
  double t6974;
  double t7004;
  double t7006;
  double t7018;
  t1432 = Cos(var1[3]);
  t1613 = Cos(var1[11]);
  t1644 = -1.*t1613;
  t1788 = 1. + t1644;
  t1967 = Sin(var1[11]);
  t5302 = Cos(var1[5]);
  t5674 = Sin(var1[3]);
  t5331 = Sin(var1[4]);
  t5751 = Sin(var1[5]);
  t3162 = Cos(var1[12]);
  t3273 = -1.*t3162;
  t4301 = 1. + t3273;
  t4553 = Sin(var1[12]);
  t1564 = Cos(var1[4]);
  t6134 = -1.*t5302*t5674;
  t6199 = t1432*t5331*t5751;
  t6200 = t6134 + t6199;
  t5575 = t1432*t5302*t5331;
  t5958 = t5674*t5751;
  t6056 = t5575 + t5958;
  t6267 = -1.*t1432*t1564*t1967;
  t6280 = t1613*t6200;
  t6294 = t6267 + t6280;
  t6310 = Cos(var1[13]);
  t6326 = -1.*t6310;
  t6332 = 1. + t6326;
  t6369 = Sin(var1[13]);
  t6392 = t1613*t1432*t1564;
  t6393 = t1967*t6200;
  t6428 = t6392 + t6393;
  t6520 = t3162*t6056;
  t6525 = -1.*t4553*t6294;
  t6526 = t6520 + t6525;
  t1815 = -0.022225*t1788;
  t2383 = -0.086996*t1967;
  t2588 = 0. + t1815 + t2383;
  t4336 = -0.31508*t4301;
  t5048 = 0.156996*t4553;
  t5173 = 0. + t4336 + t5048;
  t6104 = -0.086996*t1788;
  t6118 = 0.022225*t1967;
  t6127 = 0. + t6104 + t6118;
  t6227 = -0.156996*t4301;
  t6236 = -0.31508*t4553;
  t6237 = 0. + t6227 + t6236;
  t6679 = t1432*t5302;
  t6680 = t5674*t5331*t5751;
  t6681 = t6679 + t6680;
  t6360 = -0.022225*t6332;
  t6387 = 0.38008*t6369;
  t6391 = 0. + t6360 + t6387;
  t6659 = t5302*t5674*t5331;
  t6666 = -1.*t1432*t5751;
  t6669 = t6659 + t6666;
  t6698 = -1.*t1564*t1967*t5674;
  t6722 = t1613*t6681;
  t6738 = t6698 + t6722;
  t6511 = -0.38008*t6332;
  t6513 = -0.022225*t6369;
  t6517 = 0. + t6511 + t6513;
  t6769 = t1613*t1564*t5674;
  t6772 = t1967*t6681;
  t6776 = t6769 + t6772;
  t6823 = t3162*t6669;
  t6826 = -1.*t4553*t6738;
  t6830 = t6823 + t6826;
  t6916 = t1967*t5331;
  t6918 = t1613*t1564*t5751;
  t6945 = t6916 + t6918;
  t6963 = -1.*t1613*t5331;
  t6969 = t1564*t1967*t5751;
  t6974 = t6963 + t6969;
  t7004 = t3162*t1564*t5302;
  t7006 = -1.*t4553*t6945;
  t7018 = t7004 + t7006;

  p_output1(0)=0. + t1432*t1564*t2588 + t5173*t6056 + t6127*t6200 + t6237*t6294 - 0.166996*(t4553*t6056 + t3162*t6294) + t6391*t6428 + t6517*t6526 - 0.38008*(t6369*t6428 + t6310*t6526) - 0.022225*(t6310*t6428 - 1.*t6369*t6526) + var1(0);
  p_output1(1)=0. + t1564*t2588*t5674 + t5173*t6669 + t6127*t6681 + t6237*t6738 - 0.166996*(t4553*t6669 + t3162*t6738) + t6391*t6776 + t6517*t6830 - 0.38008*(t6369*t6776 + t6310*t6830) - 0.022225*(t6310*t6776 - 1.*t6369*t6830) + var1(1);
  p_output1(2)=0. + t1564*t5173*t5302 - 1.*t2588*t5331 + t1564*t5751*t6127 + t6237*t6945 - 0.166996*(t1564*t4553*t5302 + t3162*t6945) + t6391*t6974 + t6517*t7018 - 0.38008*(t6369*t6974 + t6310*t7018) - 0.022225*(t6310*t6974 - 1.*t6369*t7018) + var1(2);
}


       
void p_rHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
