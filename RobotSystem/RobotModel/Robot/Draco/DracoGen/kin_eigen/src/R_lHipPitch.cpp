/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:46 GMT-05:00
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
  double t1456;
  double t3323;
  double t3697;
  double t3366;
  double t3815;
  double t2250;
  double t3656;
  double t3880;
  double t4277;
  double t2176;
  double t4624;
  double t132;
  double t5207;
  double t5628;
  double t5671;
  double t5673;
  double t5474;
  double t5493;
  double t5429;
  double t5430;
  double t5454;
  double t5321;
  double t5354;
  double t5387;
  double t5809;
  double t5830;
  double t5832;
  double t5717;
  double t5718;
  double t5755;
  double t5954;
  double t5975;
  double t5979;
  double t5410;
  double t5489;
  double t5490;
  double t2363;
  double t4625;
  double t4853;
  double t5766;
  double t5854;
  double t5858;
  double t5585;
  double t5692;
  double t5693;
  double t5939;
  double t5982;
  double t5993;
  double t5908;
  double t5913;
  double t5922;
  t1456 = Cos(var1[3]);
  t3323 = Cos(var1[5]);
  t3697 = Sin(var1[4]);
  t3366 = Sin(var1[3]);
  t3815 = Sin(var1[5]);
  t2250 = Cos(var1[6]);
  t3656 = -1.*t3323*t3366;
  t3880 = t1456*t3697*t3815;
  t4277 = t3656 + t3880;
  t2176 = Cos(var1[4]);
  t4624 = Sin(var1[6]);
  t132 = Cos(var1[8]);
  t5207 = Cos(var1[7]);
  t5628 = t1456*t3323;
  t5671 = t3366*t3697*t3815;
  t5673 = t5628 + t5671;
  t5474 = Sin(var1[7]);
  t5493 = Sin(var1[8]);
  t5429 = t2250*t4277;
  t5430 = -1.*t1456*t2176*t4624;
  t5454 = t5429 + t5430;
  t5321 = t1456*t3323*t3697;
  t5354 = t3366*t3815;
  t5387 = t5321 + t5354;
  t5809 = t2250*t5673;
  t5830 = -1.*t2176*t3366*t4624;
  t5832 = t5809 + t5830;
  t5717 = t3323*t3366*t3697;
  t5718 = -1.*t1456*t3815;
  t5755 = t5717 + t5718;
  t5954 = t2176*t2250*t3815;
  t5975 = t3697*t4624;
  t5979 = t5954 + t5975;
  t5410 = t5207*t5387;
  t5489 = -1.*t5454*t5474;
  t5490 = t5410 + t5489;
  t2363 = t1456*t2176*t2250;
  t4625 = t4277*t4624;
  t4853 = t2363 + t4625;
  t5766 = t5207*t5755;
  t5854 = -1.*t5832*t5474;
  t5858 = t5766 + t5854;
  t5585 = t2176*t2250*t3366;
  t5692 = t5673*t4624;
  t5693 = t5585 + t5692;
  t5939 = t2176*t3323*t5207;
  t5982 = -1.*t5979*t5474;
  t5993 = t5939 + t5982;
  t5908 = -1.*t2250*t3697;
  t5913 = t2176*t3815*t4624;
  t5922 = t5908 + t5913;

  p_output1(0)=t132*t4853 - 1.*t5490*t5493;
  p_output1(1)=t132*t5693 - 1.*t5493*t5858;
  p_output1(2)=t132*t5922 - 1.*t5493*t5993;
  p_output1(3)=t5207*t5454 + t5387*t5474;
  p_output1(4)=t5474*t5755 + t5207*t5832;
  p_output1(5)=t2176*t3323*t5474 + t5207*t5979;
  p_output1(6)=t132*t5490 + t4853*t5493;
  p_output1(7)=t5493*t5693 + t132*t5858;
  p_output1(8)=t5493*t5922 + t132*t5993;
}


       
void R_lHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
