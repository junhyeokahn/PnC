/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:17 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_lHipRoll.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t502;
  double t516;
  double t551;
  double t688;
  double t684;
  double t927;
  double t1278;
  double t1745;
  double t761;
  double t1320;
  double t1501;
  double t2601;
  double t2037;
  double t2238;
  double t2340;
  double t3732;
  double t3081;
  double t3674;
  double t3700;
  double t2633;
  double t2731;
  double t2992;
  double t4315;
  double t4321;
  double t4453;
  double t3970;
  double t4142;
  double t4159;
  double t4568;
  double t4598;
  double t4631;
  double t4852;
  double t4869;
  double t639;
  double t1818;
  double t1834;
  double t4973;
  double t4974;
  double t3077;
  double t3877;
  double t3883;
  double t4722;
  double t4745;
  double t4759;
  double t4889;
  double t4896;
  double t4906;
  double t4935;
  double t4941;
  double t4953;
  double t1958;
  double t2435;
  double t2456;
  double t4975;
  double t4976;
  double t5001;
  double t5029;
  double t5030;
  double t5036;
  double t4182;
  double t4523;
  double t4534;
  double t4768;
  double t4769;
  double t4771;
  double t2491;
  double t2549;
  double t2596;
  double t4655;
  double t4670;
  double t4707;
  double t4775;
  double t4841;
  double t4851;
  t502 = Cos(var1[3]);
  t516 = Cos(var1[4]);
  t551 = Cos(var1[6]);
  t688 = Sin(var1[3]);
  t684 = Cos(var1[5]);
  t927 = Sin(var1[4]);
  t1278 = Sin(var1[5]);
  t1745 = Sin(var1[6]);
  t761 = -1.*t684*t688;
  t1320 = t502*t927*t1278;
  t1501 = t761 + t1320;
  t2601 = Cos(var1[7]);
  t2037 = t502*t684;
  t2238 = t688*t927*t1278;
  t2340 = t2037 + t2238;
  t3732 = Sin(var1[7]);
  t3081 = t502*t684*t927;
  t3674 = t688*t1278;
  t3700 = t3081 + t3674;
  t2633 = t551*t1501;
  t2731 = -1.*t502*t516*t1745;
  t2992 = t2633 + t2731;
  t4315 = t684*t688*t927;
  t4321 = -1.*t502*t1278;
  t4453 = t4315 + t4321;
  t3970 = t551*t2340;
  t4142 = -1.*t516*t688*t1745;
  t4159 = t3970 + t4142;
  t4568 = t516*t551*t1278;
  t4598 = t927*t1745;
  t4631 = t4568 + t4598;
  t4852 = -1.*t551;
  t4869 = 1. + t4852;
  t639 = t502*t516*t551;
  t1818 = t1501*t1745;
  t1834 = t639 + t1818;
  t4973 = -1.*t2601;
  t4974 = 1. + t4973;
  t3077 = t2601*t2992;
  t3877 = t3700*t3732;
  t3883 = t3077 + t3877;
  t4722 = t2601*t3700;
  t4745 = -1.*t2992*t3732;
  t4759 = t4722 + t4745;
  t4889 = 0.087*t4869;
  t4896 = 0.0222*t1745;
  t4906 = 0. + t4889 + t4896;
  t4935 = -0.0222*t4869;
  t4941 = 0.087*t1745;
  t4953 = 0. + t4935 + t4941;
  t1958 = t516*t551*t688;
  t2435 = t2340*t1745;
  t2456 = t1958 + t2435;
  t4975 = 0.157*t4974;
  t4976 = -0.3151*t3732;
  t5001 = 0. + t4975 + t4976;
  t5029 = -0.3151*t4974;
  t5030 = -0.157*t3732;
  t5036 = 0. + t5029 + t5030;
  t4182 = t2601*t4159;
  t4523 = t4453*t3732;
  t4534 = t4182 + t4523;
  t4768 = t2601*t4453;
  t4769 = -1.*t4159*t3732;
  t4771 = t4768 + t4769;
  t2491 = -1.*t551*t927;
  t2549 = t516*t1278*t1745;
  t2596 = t2491 + t2549;
  t4655 = t2601*t4631;
  t4670 = t516*t684*t3732;
  t4707 = t4655 + t4670;
  t4775 = t516*t684*t2601;
  t4841 = -1.*t4631*t3732;
  t4851 = t4775 + t4841;

  p_output1(0)=t1834;
  p_output1(1)=t2456;
  p_output1(2)=t2596;
  p_output1(3)=0.;
  p_output1(4)=t3883;
  p_output1(5)=t4534;
  p_output1(6)=t4707;
  p_output1(7)=0.;
  p_output1(8)=t4759;
  p_output1(9)=t4771;
  p_output1(10)=t4851;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t1834 + 0.157*t3883 - 0.3151*t4759 + t1501*t4906 + t2992*t5001 + t3700*t5036 + t4953*t502*t516 + var1(0);
  p_output1(13)=0. - 0.0222*t2456 + 0.157*t4534 - 0.3151*t4771 + t2340*t4906 + t4159*t5001 + t4453*t5036 + t4953*t516*t688 + var1(1);
  p_output1(14)=0. - 0.0222*t2596 + 0.157*t4707 - 0.3151*t4851 + t4631*t5001 + t1278*t4906*t516 + t5036*t516*t684 - 1.*t4953*t927 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
