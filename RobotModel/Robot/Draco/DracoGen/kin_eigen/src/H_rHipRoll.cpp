/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:24 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_rHipRoll.h"

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
  double t1067;
  double t537;
  double t1070;
  double t1342;
  double t1305;
  double t1319;
  double t1548;
  double t1637;
  double t1465;
  double t1651;
  double t1687;
  double t2485;
  double t2635;
  double t1835;
  double t1890;
  double t1965;
  double t2506;
  double t2508;
  double t2542;
  double t2637;
  double t2670;
  double t2723;
  double t2945;
  double t2984;
  double t3124;
  double t3173;
  double t3280;
  double t3314;
  double t3696;
  double t3765;
  double t3770;
  double t3852;
  double t3853;
  double t3863;
  double t3869;
  double t1288;
  double t1695;
  double t1788;
  double t2607;
  double t2933;
  double t2941;
  double t3809;
  double t3828;
  double t3834;
  double t3856;
  double t3857;
  double t3858;
  double t3870;
  double t3871;
  double t3907;
  double t3910;
  double t3921;
  double t3925;
  double t3960;
  double t4003;
  double t4058;
  double t1808;
  double t1968;
  double t1999;
  double t3162;
  double t3436;
  double t3537;
  double t3835;
  double t3837;
  double t3843;
  double t2090;
  double t2232;
  double t2266;
  double t3601;
  double t3794;
  double t3803;
  double t3845;
  double t3847;
  double t3850;
  t1067 = Cos(var1[3]);
  t537 = Cos(var1[11]);
  t1070 = Cos(var1[4]);
  t1342 = Sin(var1[3]);
  t1305 = Sin(var1[11]);
  t1319 = Cos(var1[5]);
  t1548 = Sin(var1[4]);
  t1637 = Sin(var1[5]);
  t1465 = -1.*t1319*t1342;
  t1651 = t1067*t1548*t1637;
  t1687 = t1465 + t1651;
  t2485 = Sin(var1[12]);
  t2635 = Cos(var1[12]);
  t1835 = t1067*t1319;
  t1890 = t1342*t1548*t1637;
  t1965 = t1835 + t1890;
  t2506 = t1067*t1319*t1548;
  t2508 = t1342*t1637;
  t2542 = t2506 + t2508;
  t2637 = -1.*t1067*t1070*t1305;
  t2670 = t537*t1687;
  t2723 = t2637 + t2670;
  t2945 = t1319*t1342*t1548;
  t2984 = -1.*t1067*t1637;
  t3124 = t2945 + t2984;
  t3173 = -1.*t1070*t1305*t1342;
  t3280 = t537*t1965;
  t3314 = t3173 + t3280;
  t3696 = t1305*t1548;
  t3765 = t537*t1070*t1637;
  t3770 = t3696 + t3765;
  t3852 = -1.*t537;
  t3853 = 1. + t3852;
  t3863 = -1.*t2635;
  t3869 = 1. + t3863;
  t1288 = t537*t1067*t1070;
  t1695 = t1305*t1687;
  t1788 = t1288 + t1695;
  t2607 = t2485*t2542;
  t2933 = t2635*t2723;
  t2941 = t2607 + t2933;
  t3809 = t2635*t2542;
  t3828 = -1.*t2485*t2723;
  t3834 = t3809 + t3828;
  t3856 = -0.0222*t3853;
  t3857 = -0.087*t1305;
  t3858 = 0. + t3856 + t3857;
  t3870 = -0.3151*t3869;
  t3871 = 0.157*t2485;
  t3907 = 0. + t3870 + t3871;
  t3910 = -0.087*t3853;
  t3921 = 0.0222*t1305;
  t3925 = 0. + t3910 + t3921;
  t3960 = -0.157*t3869;
  t4003 = -0.3151*t2485;
  t4058 = 0. + t3960 + t4003;
  t1808 = t537*t1070*t1342;
  t1968 = t1305*t1965;
  t1999 = t1808 + t1968;
  t3162 = t2485*t3124;
  t3436 = t2635*t3314;
  t3537 = t3162 + t3436;
  t3835 = t2635*t3124;
  t3837 = -1.*t2485*t3314;
  t3843 = t3835 + t3837;
  t2090 = -1.*t537*t1548;
  t2232 = t1070*t1305*t1637;
  t2266 = t2090 + t2232;
  t3601 = t1070*t1319*t2485;
  t3794 = t2635*t3770;
  t3803 = t3601 + t3794;
  t3845 = t2635*t1070*t1319;
  t3847 = -1.*t2485*t3770;
  t3850 = t3845 + t3847;

  p_output1(0)=t1788;
  p_output1(1)=t1999;
  p_output1(2)=t2266;
  p_output1(3)=0.;
  p_output1(4)=t2941;
  p_output1(5)=t3537;
  p_output1(6)=t3803;
  p_output1(7)=0.;
  p_output1(8)=t3834;
  p_output1(9)=t3843;
  p_output1(10)=t3850;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t1788 - 0.157*t2941 - 0.3151*t3834 + t1067*t1070*t3858 + t2542*t3907 + t1687*t3925 + t2723*t4058 + var1(0);
  p_output1(13)=0. - 0.0222*t1999 - 0.157*t3537 - 0.3151*t3843 + t1070*t1342*t3858 + t3124*t3907 + t1965*t3925 + t3314*t4058 + var1(1);
  p_output1(14)=0. - 0.0222*t2266 - 0.157*t3803 - 0.3151*t3850 - 1.*t1548*t3858 + t1070*t1319*t3907 + t1070*t1637*t3925 + t3770*t4058 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipRoll(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
