/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:22 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_rHipYaw.h"

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
  double t1165;
  double t1562;
  double t1608;
  double t2084;
  double t2115;
  double t2136;
  double t2143;
  double t2140;
  double t2144;
  double t1223;
  double t2154;
  double t2155;
  double t2156;
  double t2096;
  double t2125;
  double t2126;
  double t2148;
  double t2151;
  double t2152;
  double t2195;
  double t2196;
  double t2197;
  t1165 = Cos(var1[3]);
  t1562 = Cos(var1[11]);
  t1608 = -1.*t1562;
  t2084 = 1. + t1608;
  t2115 = Sin(var1[11]);
  t2136 = Cos(var1[5]);
  t2143 = Sin(var1[3]);
  t2140 = Sin(var1[4]);
  t2144 = Sin(var1[5]);
  t1223 = Cos(var1[4]);
  t2154 = -1.*t2136*t2143;
  t2155 = t1165*t2140*t2144;
  t2156 = t2154 + t2155;
  t2096 = -0.0222*t2084;
  t2125 = -0.087*t2115;
  t2126 = 0. + t2096 + t2125;
  t2148 = -0.087*t2084;
  t2151 = 0.0222*t2115;
  t2152 = 0. + t2148 + t2151;
  t2195 = t1165*t2136;
  t2196 = t2143*t2140*t2144;
  t2197 = t2195 + t2196;

  p_output1(0)=0. + t1165*t1223*t2126 - 0.2951*(t1165*t2136*t2140 + t2143*t2144) + t2152*t2156 - 0.087*(-1.*t1165*t1223*t2115 + t1562*t2156) - 0.0222*(t1165*t1223*t1562 + t2115*t2156) + var1(0);
  p_output1(1)=0. + t1223*t2126*t2143 - 0.2951*(t2136*t2140*t2143 - 1.*t1165*t2144) + t2152*t2197 - 0.087*(-1.*t1223*t2115*t2143 + t1562*t2197) - 0.0222*(t1223*t1562*t2143 + t2115*t2197) + var1(1);
  p_output1(2)=0. - 0.2951*t1223*t2136 - 1.*t2126*t2140 - 0.087*(t2115*t2140 + t1223*t1562*t2144) - 0.0222*(-1.*t1562*t2140 + t1223*t2115*t2144) + t1223*t2144*t2152 + var1(2);
}


       
void p_rHipYaw(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
