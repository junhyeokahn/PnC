/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:17 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lHipRoll.h"

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
  double t1599;
  double t1681;
  double t1779;
  double t2435;
  double t1783;
  double t2549;
  double t2735;
  double t2967;
  double t2456;
  double t2743;
  double t2806;
  double t4130;
  double t3115;
  double t3201;
  double t3391;
  double t4851;
  double t4598;
  double t4722;
  double t4775;
  double t4164;
  double t4175;
  double t4182;
  double t4959;
  double t4975;
  double t4976;
  double t4935;
  double t4941;
  double t4946;
  double t5029;
  double t5030;
  double t5036;
  t1599 = Cos(var1[3]);
  t1681 = Cos(var1[4]);
  t1779 = Cos(var1[6]);
  t2435 = Sin(var1[3]);
  t1783 = Cos(var1[5]);
  t2549 = Sin(var1[4]);
  t2735 = Sin(var1[5]);
  t2967 = Sin(var1[6]);
  t2456 = -1.*t1783*t2435;
  t2743 = t1599*t2549*t2735;
  t2806 = t2456 + t2743;
  t4130 = Cos(var1[7]);
  t3115 = t1599*t1783;
  t3201 = t2435*t2549*t2735;
  t3391 = t3115 + t3201;
  t4851 = Sin(var1[7]);
  t4598 = t1599*t1783*t2549;
  t4722 = t2435*t2735;
  t4775 = t4598 + t4722;
  t4164 = t1779*t2806;
  t4175 = -1.*t1599*t1681*t2967;
  t4182 = t4164 + t4175;
  t4959 = t1783*t2435*t2549;
  t4975 = -1.*t1599*t2735;
  t4976 = t4959 + t4975;
  t4935 = t1779*t3391;
  t4941 = -1.*t1681*t2435*t2967;
  t4946 = t4935 + t4941;
  t5029 = t1681*t1779*t2735;
  t5030 = t2549*t2967;
  t5036 = t5029 + t5030;

  p_output1(0)=t1599*t1681*t1779 + t2806*t2967;
  p_output1(1)=t1681*t1779*t2435 + t2967*t3391;
  p_output1(2)=-1.*t1779*t2549 + t1681*t2735*t2967;
  p_output1(3)=t4130*t4182 + t4775*t4851;
  p_output1(4)=t4130*t4946 + t4851*t4976;
  p_output1(5)=t1681*t1783*t4851 + t4130*t5036;
  p_output1(6)=t4130*t4775 - 1.*t4182*t4851;
  p_output1(7)=-1.*t4851*t4946 + t4130*t4976;
  p_output1(8)=t1681*t1783*t4130 - 1.*t4851*t5036;
}


       
void R_lHipRoll(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
