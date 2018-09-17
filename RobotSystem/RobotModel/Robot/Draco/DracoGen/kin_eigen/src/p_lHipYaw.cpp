/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:14 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_lHipYaw.h"

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
  double t1381;
  double t1559;
  double t1366;
  double t1507;
  double t1632;
  double t3340;
  double t3352;
  double t3581;
  double t3621;
  double t3214;
  double t3270;
  double t3312;
  double t3763;
  double t3610;
  double t3630;
  double t3662;
  double t3768;
  double t3813;
  double t3998;
  double t4215;
  double t4216;
  double t4222;
  t1381 = Cos(var1[5]);
  t1559 = Sin(var1[3]);
  t1366 = Cos(var1[3]);
  t1507 = Sin(var1[4]);
  t1632 = Sin(var1[5]);
  t3340 = Cos(var1[6]);
  t3352 = -1.*t3340;
  t3581 = 1. + t3352;
  t3621 = Sin(var1[6]);
  t3214 = -1.*t1381*t1559;
  t3270 = t1366*t1507*t1632;
  t3312 = t3214 + t3270;
  t3763 = Cos(var1[4]);
  t3610 = 0.087*t3581;
  t3630 = 0.0222*t3621;
  t3662 = 0. + t3610 + t3630;
  t3768 = -0.0222*t3581;
  t3813 = 0.087*t3621;
  t3998 = 0. + t3768 + t3813;
  t4215 = t1366*t1381;
  t4216 = t1559*t1507*t1632;
  t4222 = t4215 + t4216;

  p_output1(0)=0. - 0.2951*(t1366*t1381*t1507 + t1559*t1632) + t3312*t3662 - 0.0222*(t3312*t3621 + t1366*t3340*t3763) + 0.087*(t3312*t3340 - 1.*t1366*t3621*t3763) + t1366*t3763*t3998 + var1(0);
  p_output1(1)=0. - 0.2951*(t1381*t1507*t1559 - 1.*t1366*t1632) + t1559*t3763*t3998 + t3662*t4222 + 0.087*(-1.*t1559*t3621*t3763 + t3340*t4222) - 0.0222*(t1559*t3340*t3763 + t3621*t4222) + var1(1);
  p_output1(2)=0. - 0.2951*t1381*t3763 + t1632*t3662*t3763 + 0.087*(t1507*t3621 + t1632*t3340*t3763) - 0.0222*(-1.*t1507*t3340 + t1632*t3621*t3763) - 1.*t1507*t3998 + var1(2);
}


       
void p_lHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
