/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lKnee.h"

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
  double t2336;
  double t2485;
  double t1764;
  double t2341;
  double t2524;
  double t3776;
  double t3335;
  double t3365;
  double t3450;
  double t3471;
  double t3864;
  double t254;
  double t3970;
  double t3989;
  double t3997;
  double t412;
  double t2463;
  double t2623;
  double t3126;
  double t3243;
  double t3760;
  double t3878;
  double t3906;
  double t3922;
  double t3946;
  double t3960;
  double t4032;
  double t42;
  double t4242;
  double t4245;
  double t4251;
  double t4043;
  double t4458;
  double t4487;
  double t4513;
  double t4226;
  double t4229;
  double t4233;
  double t4238;
  double t4272;
  double t4274;
  double t4284;
  double t4313;
  double t4315;
  double t5255;
  double t5259;
  double t5336;
  double t4851;
  double t4881;
  double t4988;
  double t4995;
  double t5198;
  double t5206;
  double t3968;
  double t4033;
  double t4035;
  double t4072;
  double t4076;
  double t4147;
  double t4316;
  double t4525;
  double t4598;
  double t4659;
  double t4671;
  double t4755;
  double t5241;
  double t5381;
  double t5419;
  double t5482;
  double t5496;
  double t5501;
  t2336 = Cos(var1[5]);
  t2485 = Sin(var1[3]);
  t1764 = Cos(var1[3]);
  t2341 = Sin(var1[4]);
  t2524 = Sin(var1[5]);
  t3776 = Cos(var1[4]);
  t3335 = Cos(var1[6]);
  t3365 = -1.*t2336*t2485;
  t3450 = t1764*t2341*t2524;
  t3471 = t3365 + t3450;
  t3864 = Sin(var1[6]);
  t254 = Cos(var1[8]);
  t3970 = t1764*t3776*t3335;
  t3989 = t3471*t3864;
  t3997 = t3970 + t3989;
  t412 = Cos(var1[7]);
  t2463 = t1764*t2336*t2341;
  t2623 = t2485*t2524;
  t3126 = t2463 + t2623;
  t3243 = t412*t3126;
  t3760 = t3335*t3471;
  t3878 = -1.*t1764*t3776*t3864;
  t3906 = t3760 + t3878;
  t3922 = Sin(var1[7]);
  t3946 = -1.*t3906*t3922;
  t3960 = t3243 + t3946;
  t4032 = Sin(var1[8]);
  t42 = Sin(var1[9]);
  t4242 = t1764*t2336;
  t4245 = t2485*t2341*t2524;
  t4251 = t4242 + t4245;
  t4043 = Cos(var1[9]);
  t4458 = t3776*t3335*t2485;
  t4487 = t4251*t3864;
  t4513 = t4458 + t4487;
  t4226 = t2336*t2485*t2341;
  t4229 = -1.*t1764*t2524;
  t4233 = t4226 + t4229;
  t4238 = t412*t4233;
  t4272 = t3335*t4251;
  t4274 = -1.*t3776*t2485*t3864;
  t4284 = t4272 + t4274;
  t4313 = -1.*t4284*t3922;
  t4315 = t4238 + t4313;
  t5255 = -1.*t3335*t2341;
  t5259 = t3776*t2524*t3864;
  t5336 = t5255 + t5259;
  t4851 = t3776*t2336*t412;
  t4881 = t3776*t3335*t2524;
  t4988 = t2341*t3864;
  t4995 = t4881 + t4988;
  t5198 = -1.*t4995*t3922;
  t5206 = t4851 + t5198;
  t3968 = t254*t3960;
  t4033 = t3997*t4032;
  t4035 = t3968 + t4033;
  t4072 = t254*t3997;
  t4076 = -1.*t3960*t4032;
  t4147 = t4072 + t4076;
  t4316 = t254*t4315;
  t4525 = t4513*t4032;
  t4598 = t4316 + t4525;
  t4659 = t254*t4513;
  t4671 = -1.*t4315*t4032;
  t4755 = t4659 + t4671;
  t5241 = t254*t5206;
  t5381 = t5336*t4032;
  t5419 = t5241 + t5381;
  t5482 = t254*t5336;
  t5496 = -1.*t5206*t4032;
  t5501 = t5482 + t5496;

  p_output1(0)=t4043*t4147 - 1.*t4035*t42;
  p_output1(1)=-1.*t42*t4598 + t4043*t4755;
  p_output1(2)=-1.*t42*t5419 + t4043*t5501;
  p_output1(3)=t3126*t3922 + t3906*t412;
  p_output1(4)=t3922*t4233 + t412*t4284;
  p_output1(5)=t2336*t3776*t3922 + t412*t4995;
  p_output1(6)=t4035*t4043 + t4147*t42;
  p_output1(7)=t4043*t4598 + t42*t4755;
  p_output1(8)=t4043*t5419 + t42*t5501;
}


       
void R_lKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
