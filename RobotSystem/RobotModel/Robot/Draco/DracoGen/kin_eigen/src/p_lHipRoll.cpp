/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:16 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_lHipRoll.h"

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
  double t3254;
  double t4440;
  double t4490;
  double t4542;
  double t4573;
  double t4600;
  double t1564;
  double t1565;
  double t1566;
  double t3710;
  double t3912;
  double t4264;
  double t4331;
  double t4743;
  double t4744;
  double t4749;
  double t4756;
  double t4722;
  double t4728;
  double t4738;
  double t4770;
  double t4771;
  double t4772;
  double t4558;
  double t4574;
  double t4593;
  double t4601;
  double t4610;
  double t4621;
  double t4935;
  double t4945;
  double t4953;
  double t4753;
  double t4759;
  double t4768;
  double t4778;
  double t4784;
  double t4786;
  double t5007;
  double t5019;
  double t5024;
  double t5043;
  double t5050;
  double t5054;
  double t5193;
  double t5202;
  double t5206;
  t3254 = Cos(var1[3]);
  t4440 = Cos(var1[6]);
  t4490 = -1.*t4440;
  t4542 = 1. + t4490;
  t4573 = Sin(var1[6]);
  t4600 = Cos(var1[4]);
  t1564 = Cos(var1[5]);
  t1565 = Sin(var1[3]);
  t1566 = -1.*t1564*t1565;
  t3710 = Sin(var1[4]);
  t3912 = Sin(var1[5]);
  t4264 = t3254*t3710*t3912;
  t4331 = t1566 + t4264;
  t4743 = Cos(var1[7]);
  t4744 = -1.*t4743;
  t4749 = 1. + t4744;
  t4756 = Sin(var1[7]);
  t4722 = t4440*t4331;
  t4728 = -1.*t3254*t4600*t4573;
  t4738 = t4722 + t4728;
  t4770 = t3254*t1564*t3710;
  t4771 = t1565*t3912;
  t4772 = t4770 + t4771;
  t4558 = 0.087*t4542;
  t4574 = 0.0222*t4573;
  t4593 = 0. + t4558 + t4574;
  t4601 = -0.0222*t4542;
  t4610 = 0.087*t4573;
  t4621 = 0. + t4601 + t4610;
  t4935 = t3254*t1564;
  t4945 = t1565*t3710*t3912;
  t4953 = t4935 + t4945;
  t4753 = 0.157*t4749;
  t4759 = -0.3151*t4756;
  t4768 = 0. + t4753 + t4759;
  t4778 = -0.3151*t4749;
  t4784 = -0.157*t4756;
  t4786 = 0. + t4778 + t4784;
  t5007 = t4440*t4953;
  t5019 = -1.*t4600*t1565*t4573;
  t5024 = t5007 + t5019;
  t5043 = t1564*t1565*t3710;
  t5050 = -1.*t3254*t3912;
  t5054 = t5043 + t5050;
  t5193 = t4600*t4440*t3912;
  t5202 = t3710*t4573;
  t5206 = t5193 + t5202;

  p_output1(0)=0. + t4331*t4593 - 0.0222*(t4331*t4573 + t3254*t4440*t4600) + t3254*t4600*t4621 + t4738*t4768 - 0.3151*(-1.*t4738*t4756 + t4743*t4772) + 0.157*(t4738*t4743 + t4756*t4772) + t4772*t4786 + var1(0);
  p_output1(1)=0. + t1565*t4600*t4621 + t4593*t4953 - 0.0222*(t1565*t4440*t4600 + t4573*t4953) + t4768*t5024 + t4786*t5054 - 0.3151*(-1.*t4756*t5024 + t4743*t5054) + 0.157*(t4743*t5024 + t4756*t5054) + var1(1);
  p_output1(2)=0. + t3912*t4593*t4600 - 0.0222*(-1.*t3710*t4440 + t3912*t4573*t4600) - 1.*t3710*t4621 + t1564*t4600*t4786 + t4768*t5206 + 0.157*(t1564*t4600*t4756 + t4743*t5206) - 0.3151*(t1564*t4600*t4743 - 1.*t4756*t5206) + var1(2);
}


       
void p_lHipRoll(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
