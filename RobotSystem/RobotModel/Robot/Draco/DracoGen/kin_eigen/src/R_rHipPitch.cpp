/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:26 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipPitch.h"

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
  double t1354;
  double t2990;
  double t3494;
  double t3175;
  double t3536;
  double t1425;
  double t2927;
  double t1333;
  double t3266;
  double t3642;
  double t4143;
  double t577;
  double t4714;
  double t4729;
  double t5048;
  double t5126;
  double t5127;
  double t5141;
  double t4806;
  double t4810;
  double t5040;
  double t5049;
  double t5061;
  double t5064;
  double t5210;
  double t5211;
  double t5217;
  double t5258;
  double t5259;
  double t5283;
  double t5454;
  double t5455;
  double t5457;
  double t2477;
  double t4145;
  double t4152;
  double t5044;
  double t5087;
  double t5089;
  double t5123;
  double t5160;
  double t5163;
  double t5242;
  double t5291;
  double t5306;
  double t5336;
  double t5379;
  double t5426;
  double t5451;
  double t5482;
  double t5485;
  t1354 = Cos(var1[3]);
  t2990 = Cos(var1[5]);
  t3494 = Sin(var1[4]);
  t3175 = Sin(var1[3]);
  t3536 = Sin(var1[5]);
  t1425 = Cos(var1[4]);
  t2927 = Sin(var1[11]);
  t1333 = Cos(var1[11]);
  t3266 = -1.*t2990*t3175;
  t3642 = t1354*t3494*t3536;
  t4143 = t3266 + t3642;
  t577 = Cos(var1[13]);
  t4714 = Sin(var1[13]);
  t4729 = Cos(var1[12]);
  t5048 = Sin(var1[12]);
  t5126 = t1354*t2990;
  t5127 = t3175*t3494*t3536;
  t5141 = t5126 + t5127;
  t4806 = t1354*t2990*t3494;
  t4810 = t3175*t3536;
  t5040 = t4806 + t4810;
  t5049 = -1.*t1354*t1425*t2927;
  t5061 = t1333*t4143;
  t5064 = t5049 + t5061;
  t5210 = t2990*t3175*t3494;
  t5211 = -1.*t1354*t3536;
  t5217 = t5210 + t5211;
  t5258 = -1.*t1425*t2927*t3175;
  t5259 = t1333*t5141;
  t5283 = t5258 + t5259;
  t5454 = t2927*t3494;
  t5455 = t1333*t1425*t3536;
  t5457 = t5454 + t5455;
  t2477 = t1333*t1354*t1425;
  t4145 = t2927*t4143;
  t4152 = t2477 + t4145;
  t5044 = t4729*t5040;
  t5087 = -1.*t5048*t5064;
  t5089 = t5044 + t5087;
  t5123 = t1333*t1425*t3175;
  t5160 = t2927*t5141;
  t5163 = t5123 + t5160;
  t5242 = t4729*t5217;
  t5291 = -1.*t5048*t5283;
  t5306 = t5242 + t5291;
  t5336 = -1.*t1333*t3494;
  t5379 = t1425*t2927*t3536;
  t5426 = t5336 + t5379;
  t5451 = t4729*t1425*t2990;
  t5482 = -1.*t5048*t5457;
  t5485 = t5451 + t5482;

  p_output1(0)=-1.*t4714*t5089 + t4152*t577;
  p_output1(1)=-1.*t4714*t5306 + t5163*t577;
  p_output1(2)=-1.*t4714*t5485 + t5426*t577;
  p_output1(3)=t5040*t5048 + t4729*t5064;
  p_output1(4)=t5048*t5217 + t4729*t5283;
  p_output1(5)=t1425*t2990*t5048 + t4729*t5457;
  p_output1(6)=t4152*t4714 + t5089*t577;
  p_output1(7)=t4714*t5163 + t5306*t577;
  p_output1(8)=t4714*t5426 + t5485*t577;
}


       
void R_rHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
