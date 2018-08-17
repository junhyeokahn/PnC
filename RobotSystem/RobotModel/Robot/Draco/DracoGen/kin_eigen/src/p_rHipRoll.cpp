/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:54 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rHipRoll.h"

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
  double t171;
  double t722;
  double t730;
  double t741;
  double t971;
  double t4472;
  double t4797;
  double t4511;
  double t4808;
  double t1869;
  double t2835;
  double t4061;
  double t4162;
  double t435;
  double t5105;
  double t5133;
  double t5135;
  double t4760;
  double t4861;
  double t4990;
  double t5201;
  double t5228;
  double t5250;
  double t908;
  double t1223;
  double t1666;
  double t4079;
  double t4218;
  double t4321;
  double t5089;
  double t5092;
  double t5100;
  double t5155;
  double t5170;
  double t5196;
  double t5589;
  double t5596;
  double t5627;
  double t5537;
  double t5555;
  double t5564;
  double t5660;
  double t5663;
  double t5665;
  double t5884;
  double t5900;
  double t5924;
  t171 = Cos(var1[3]);
  t722 = Cos(var1[11]);
  t730 = -1.*t722;
  t741 = 1. + t730;
  t971 = Sin(var1[11]);
  t4472 = Cos(var1[5]);
  t4797 = Sin(var1[3]);
  t4511 = Sin(var1[4]);
  t4808 = Sin(var1[5]);
  t1869 = Cos(var1[12]);
  t2835 = -1.*t1869;
  t4061 = 1. + t2835;
  t4162 = Sin(var1[12]);
  t435 = Cos(var1[4]);
  t5105 = -1.*t4472*t4797;
  t5133 = t171*t4511*t4808;
  t5135 = t5105 + t5133;
  t4760 = t171*t4472*t4511;
  t4861 = t4797*t4808;
  t4990 = t4760 + t4861;
  t5201 = -1.*t171*t435*t971;
  t5228 = t722*t5135;
  t5250 = t5201 + t5228;
  t908 = -0.022225*t741;
  t1223 = -0.086996*t971;
  t1666 = 0. + t908 + t1223;
  t4079 = -0.31508*t4061;
  t4218 = 0.156996*t4162;
  t4321 = 0. + t4079 + t4218;
  t5089 = -0.086996*t741;
  t5092 = 0.022225*t971;
  t5100 = 0. + t5089 + t5092;
  t5155 = -0.156996*t4061;
  t5170 = -0.31508*t4162;
  t5196 = 0. + t5155 + t5170;
  t5589 = t171*t4472;
  t5596 = t4797*t4511*t4808;
  t5627 = t5589 + t5596;
  t5537 = t4472*t4797*t4511;
  t5555 = -1.*t171*t4808;
  t5564 = t5537 + t5555;
  t5660 = -1.*t435*t971*t4797;
  t5663 = t722*t5627;
  t5665 = t5660 + t5663;
  t5884 = t971*t4511;
  t5900 = t722*t435*t4808;
  t5924 = t5884 + t5900;

  p_output1(0)=0. + t1666*t171*t435 + t4321*t4990 + t5100*t5135 + t5196*t5250 - 0.156996*(t4162*t4990 + t1869*t5250) - 0.31508*(t1869*t4990 - 1.*t4162*t5250) - 0.022225*(t171*t435*t722 + t5135*t971) + var1(0);
  p_output1(1)=0. + t1666*t435*t4797 + t4321*t5564 + t5100*t5627 + t5196*t5665 - 0.156996*(t4162*t5564 + t1869*t5665) - 0.31508*(t1869*t5564 - 1.*t4162*t5665) - 0.022225*(t435*t4797*t722 + t5627*t971) + var1(1);
  p_output1(2)=0. + t4321*t435*t4472 - 1.*t1666*t4511 + t435*t4808*t5100 + t5196*t5924 - 0.156996*(t4162*t435*t4472 + t1869*t5924) - 0.31508*(t1869*t435*t4472 - 1.*t4162*t5924) - 0.022225*(-1.*t4511*t722 + t435*t4808*t971) + var1(2);
}


       
void p_rHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
