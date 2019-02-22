/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:32 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/R_LeftFootFront.h"

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
  double t873;
  double t410;
  double t878;
  double t627;
  double t934;
  double t120;
  double t389;
  double t1057;
  double t1094;
  double t1459;
  double t1348;
  double t1386;
  double t1429;
  double t867;
  double t983;
  double t1009;
  double t1047;
  double t1099;
  double t1103;
  double t2765;
  double t2828;
  double t2832;
  double t2918;
  double t2769;
  double t2788;
  double t2791;
  double t2966;
  double t3123;
  double t2824;
  double t2969;
  double t2971;
  double t2733;
  double t3131;
  double t3209;
  double t3211;
  double t3363;
  double t3087;
  double t3227;
  double t3334;
  double t2642;
  double t3441;
  double t3466;
  double t3482;
  double t2070;
  double t2204;
  double t2269;
  double t1554;
  double t1638;
  double t1767;
  double t1849;
  double t1852;
  double t1908;
  double t3810;
  double t3833;
  double t3922;
  double t3649;
  double t3660;
  double t3684;
  double t3766;
  double t3963;
  double t3977;
  double t4027;
  double t4031;
  double t4047;
  double t4018;
  double t4074;
  double t4139;
  double t4144;
  double t4159;
  double t4192;
  double t2406;
  double t2523;
  double t2539;
  double t4536;
  double t4544;
  double t4571;
  double t4479;
  double t4483;
  double t4515;
  double t4523;
  double t4645;
  double t4742;
  double t4769;
  double t4788;
  double t4843;
  double t4763;
  double t4846;
  double t4856;
  double t4858;
  double t4863;
  double t4869;
  t873 = Cos(var1[3]);
  t410 = Cos(var1[5]);
  t878 = Sin(var1[4]);
  t627 = Sin(var1[3]);
  t934 = Sin(var1[5]);
  t120 = Cos(var1[7]);
  t389 = Cos(var1[6]);
  t1057 = Cos(var1[4]);
  t1094 = Sin(var1[6]);
  t1459 = Sin(var1[7]);
  t1348 = t873*t410*t878;
  t1386 = t627*t934;
  t1429 = t1348 + t1386;
  t867 = -1.*t410*t627;
  t983 = t873*t878*t934;
  t1009 = t867 + t983;
  t1047 = t389*t1009;
  t1099 = -1.*t873*t1057*t1094;
  t1103 = t1047 + t1099;
  t2765 = Cos(var1[8]);
  t2828 = t873*t1057*t389;
  t2832 = t1009*t1094;
  t2918 = t2828 + t2832;
  t2769 = t120*t1429;
  t2788 = -1.*t1103*t1459;
  t2791 = t2769 + t2788;
  t2966 = Sin(var1[8]);
  t3123 = Cos(var1[9]);
  t2824 = t2765*t2791;
  t2969 = t2918*t2966;
  t2971 = t2824 + t2969;
  t2733 = Sin(var1[9]);
  t3131 = t2765*t2918;
  t3209 = -1.*t2791*t2966;
  t3211 = t3131 + t3209;
  t3363 = Cos(var1[10]);
  t3087 = -1.*t2733*t2971;
  t3227 = t3123*t3211;
  t3334 = t3087 + t3227;
  t2642 = Sin(var1[10]);
  t3441 = t3123*t2971;
  t3466 = t2733*t3211;
  t3482 = t3441 + t3466;
  t2070 = t410*t627*t878;
  t2204 = -1.*t873*t934;
  t2269 = t2070 + t2204;
  t1554 = t873*t410;
  t1638 = t627*t878*t934;
  t1767 = t1554 + t1638;
  t1849 = t389*t1767;
  t1852 = -1.*t1057*t627*t1094;
  t1908 = t1849 + t1852;
  t3810 = t1057*t389*t627;
  t3833 = t1767*t1094;
  t3922 = t3810 + t3833;
  t3649 = t120*t2269;
  t3660 = -1.*t1908*t1459;
  t3684 = t3649 + t3660;
  t3766 = t2765*t3684;
  t3963 = t3922*t2966;
  t3977 = t3766 + t3963;
  t4027 = t2765*t3922;
  t4031 = -1.*t3684*t2966;
  t4047 = t4027 + t4031;
  t4018 = -1.*t2733*t3977;
  t4074 = t3123*t4047;
  t4139 = t4018 + t4074;
  t4144 = t3123*t3977;
  t4159 = t2733*t4047;
  t4192 = t4144 + t4159;
  t2406 = t1057*t389*t934;
  t2523 = t878*t1094;
  t2539 = t2406 + t2523;
  t4536 = -1.*t389*t878;
  t4544 = t1057*t934*t1094;
  t4571 = t4536 + t4544;
  t4479 = t1057*t410*t120;
  t4483 = -1.*t2539*t1459;
  t4515 = t4479 + t4483;
  t4523 = t2765*t4515;
  t4645 = t4571*t2966;
  t4742 = t4523 + t4645;
  t4769 = t2765*t4571;
  t4788 = -1.*t4515*t2966;
  t4843 = t4769 + t4788;
  t4763 = -1.*t2733*t4742;
  t4846 = t3123*t4843;
  t4856 = t4763 + t4846;
  t4858 = t3123*t4742;
  t4863 = t2733*t4843;
  t4869 = t4858 + t4863;

  p_output1(0)=t1103*t120 + t1429*t1459;
  p_output1(1)=t120*t1908 + t1459*t2269;
  p_output1(2)=t120*t2539 + t1057*t1459*t410;
  p_output1(3)=-1.*t2642*t3334 - 1.*t3363*t3482 - 0.000796*(t3334*t3363 - 1.*t2642*t3482);
  p_output1(4)=-1.*t2642*t4139 - 1.*t3363*t4192 - 0.000796*(t3363*t4139 - 1.*t2642*t4192);
  p_output1(5)=-1.*t2642*t4856 - 1.*t3363*t4869 - 0.000796*(t3363*t4856 - 1.*t2642*t4869);
  p_output1(6)=-1.*t3334*t3363 + t2642*t3482 + 0.000796*(t2642*t3334 + t3363*t3482);
  p_output1(7)=-1.*t3363*t4139 + t2642*t4192 + 0.000796*(t2642*t4139 + t3363*t4192);
  p_output1(8)=-1.*t3363*t4856 + t2642*t4869 + 0.000796*(t2642*t4856 + t3363*t4869);
}


       
void R_LeftFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
