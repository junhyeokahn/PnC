/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:23 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootBack.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1853;
  double t2141;
  double t3663;
  double t908;
  double t4236;
  double t4285;
  double t4292;
  double t4325;
  double t4429;
  double t4434;
  double t4436;
  double t4440;
  double t4451;
  double t4456;
  double t4457;
  double t4415;
  double t4422;
  double t4425;
  double t4469;
  double t4470;
  double t4472;
  double t4474;
  double t4479;
  double t4481;
  double t4488;
  double t4499;
  double t4501;
  double t4503;
  double t1187;
  double t1188;
  double t2764;
  double t3222;
  double t3522;
  double t3704;
  double t4147;
  double t4317;
  double t4362;
  double t4363;
  double t4377;
  double t4378;
  double t4403;
  double t4437;
  double t4445;
  double t4447;
  double t4461;
  double t4465;
  double t4466;
  double t4473;
  double t4475;
  double t4478;
  double t4566;
  double t4571;
  double t4573;
  double t4561;
  double t4562;
  double t4563;
  double t4491;
  double t4493;
  double t4495;
  double t4581;
  double t4589;
  double t4594;
  double t4598;
  double t4603;
  double t4606;
  double t4642;
  double t4657;
  double t4658;
  double t4649;
  double t4652;
  double t4661;
  double t4662;
  double t4663;
  double t4665;
  double t4666;
  double t4667;
  t1853 = Sin(var1[5]);
  t2141 = Cos(var1[6]);
  t3663 = Sin(var1[6]);
  t908 = Cos(var1[5]);
  t4236 = Cos(var1[7]);
  t4285 = -1.*t4236;
  t4292 = 1. + t4285;
  t4325 = Sin(var1[7]);
  t4429 = Cos(var1[8]);
  t4434 = -1.*t4429;
  t4436 = 1. + t4434;
  t4440 = Sin(var1[8]);
  t4451 = t908*t4236;
  t4456 = -1.*t1853*t3663*t4325;
  t4457 = t4451 + t4456;
  t4415 = t4236*t1853*t3663;
  t4422 = t908*t4325;
  t4425 = t4415 + t4422;
  t4469 = Cos(var1[9]);
  t4470 = -1.*t4469;
  t4472 = 1. + t4470;
  t4474 = Sin(var1[9]);
  t4479 = t4429*t4457;
  t4481 = -1.*t4425*t4440;
  t4488 = t4479 + t4481;
  t4499 = t4429*t4425;
  t4501 = t4457*t4440;
  t4503 = t4499 + t4501;
  t1187 = -1.*t908;
  t1188 = 1. + t1187;
  t2764 = -1.*t2141;
  t3222 = 1. + t2764;
  t3522 = -0.330988*t3222;
  t3704 = -0.90524*t3663;
  t4147 = 0. + t3522 + t3704;
  t4317 = -0.97024*t4292;
  t4362 = -0.066675*t4325;
  t4363 = 0. + t4317 + t4362;
  t4377 = -0.066675*t4292;
  t4378 = 0.97024*t4325;
  t4403 = 0. + t4377 + t4378;
  t4437 = -1.45024*t4436;
  t4445 = -0.066675*t4440;
  t4447 = 0. + t4437 + t4445;
  t4461 = -0.066675*t4436;
  t4465 = 1.45024*t4440;
  t4466 = 0. + t4461 + t4465;
  t4473 = -0.065597*t4472;
  t4475 = 1.93024*t4474;
  t4478 = 0. + t4473 + t4475;
  t4566 = t4236*t1853;
  t4571 = t908*t3663*t4325;
  t4573 = t4566 + t4571;
  t4561 = -1.*t908*t4236*t3663;
  t4562 = t1853*t4325;
  t4563 = t4561 + t4562;
  t4491 = -1.93024*t4472;
  t4493 = -0.065597*t4474;
  t4495 = 0. + t4491 + t4493;
  t4581 = t4429*t4573;
  t4589 = -1.*t4563*t4440;
  t4594 = t4581 + t4589;
  t4598 = t4429*t4563;
  t4603 = t4573*t4440;
  t4606 = t4598 + t4603;
  t4642 = 0. + t2141;
  t4657 = -1.*t4642*t4325;
  t4658 = 0. + t4657;
  t4649 = t4642*t4236;
  t4652 = 0. + t4649;
  t4661 = t4429*t4658;
  t4662 = -1.*t4652*t4440;
  t4663 = t4661 + t4662;
  t4665 = t4652*t4429;
  t4666 = t4658*t4440;
  t4667 = t4665 + t4666;

  p_output1(0)=0. - 0.066675*t1188 - 0.260988*t1853 + 0.340988*t1853*t2141 - 1.*t1853*t4147 + t1853*t3663*t4363 + t4425*t4447 + t4457*t4466 + t4478*t4488 + t4495*t4503 - 1.990292*(t4474*t4488 + t4469*t4503) - 0.000645*(t4469*t4488 - 1.*t4474*t4503) + t4403*t908;
  p_output1(1)=0. - 0.260988*t1188 + 0.066675*t1853 + t1853*t4403 + t4447*t4563 + t4466*t4573 + t4478*t4594 + t4495*t4606 - 1.990292*(t4474*t4594 + t4469*t4606) - 0.000645*(t4469*t4594 - 1.*t4474*t4606) - 0.340988*t2141*t908 + t4147*t908 - 1.*t3663*t4363*t908;
  p_output1(2)=0. - 0.90524*t3222 + 0.330988*t3663 - 0.340988*(0. + t3663) + t4363*t4642 + t4447*t4652 + t4466*t4658 + t4478*t4663 + t4495*t4667 - 1.990292*(t4474*t4663 + t4469*t4667) - 0.000645*(t4469*t4663 - 1.*t4474*t4667);
}


       
void p_RightFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
