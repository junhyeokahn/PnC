/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:36 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/R_RightFootFront.h"

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
  double t274;
  double t416;
  double t933;
  double t888;
  double t1069;
  double t210;
  double t1468;
  double t1503;
  double t1517;
  double t1646;
  double t1821;
  double t1886;
  double t1915;
  double t896;
  double t1302;
  double t1329;
  double t1613;
  double t1963;
  double t1971;
  double t3478;
  double t3405;
  double t3427;
  double t3451;
  double t3361;
  double t3488;
  double t3505;
  double t3507;
  double t3711;
  double t3456;
  double t3618;
  double t3699;
  double t3270;
  double t3714;
  double t3788;
  double t3930;
  double t4026;
  double t3707;
  double t3996;
  double t4014;
  double t3251;
  double t4032;
  double t4034;
  double t4039;
  double t2455;
  double t2473;
  double t2674;
  double t2167;
  double t2176;
  double t2182;
  double t2429;
  double t2722;
  double t2937;
  double t4123;
  double t4165;
  double t4169;
  double t4263;
  double t4264;
  double t4302;
  double t4201;
  double t4316;
  double t4320;
  double t4455;
  double t4460;
  double t4463;
  double t4433;
  double t4538;
  double t4558;
  double t4611;
  double t4624;
  double t4634;
  double t3025;
  double t3029;
  double t3083;
  double t4802;
  double t4812;
  double t4819;
  double t4839;
  double t4877;
  double t4920;
  double t4829;
  double t4923;
  double t4924;
  double t4930;
  double t4993;
  double t5074;
  double t4929;
  double t5104;
  double t5108;
  double t5148;
  double t5165;
  double t5173;
  t274 = Cos(var1[3]);
  t416 = Cos(var1[5]);
  t933 = Sin(var1[3]);
  t888 = Sin(var1[4]);
  t1069 = Sin(var1[5]);
  t210 = Sin(var1[12]);
  t1468 = Cos(var1[12]);
  t1503 = Cos(var1[4]);
  t1517 = Sin(var1[11]);
  t1646 = Cos(var1[11]);
  t1821 = -1.*t416*t933;
  t1886 = t274*t888*t1069;
  t1915 = t1821 + t1886;
  t896 = t274*t416*t888;
  t1302 = t933*t1069;
  t1329 = t896 + t1302;
  t1613 = -1.*t274*t1503*t1517;
  t1963 = t1646*t1915;
  t1971 = t1613 + t1963;
  t3478 = Cos(var1[13]);
  t3405 = t1646*t274*t1503;
  t3427 = t1517*t1915;
  t3451 = t3405 + t3427;
  t3361 = Sin(var1[13]);
  t3488 = t1468*t1329;
  t3505 = -1.*t210*t1971;
  t3507 = t3488 + t3505;
  t3711 = Cos(var1[14]);
  t3456 = t3361*t3451;
  t3618 = t3478*t3507;
  t3699 = t3456 + t3618;
  t3270 = Sin(var1[14]);
  t3714 = t3478*t3451;
  t3788 = -1.*t3361*t3507;
  t3930 = t3714 + t3788;
  t4026 = Cos(var1[15]);
  t3707 = -1.*t3270*t3699;
  t3996 = t3711*t3930;
  t4014 = t3707 + t3996;
  t3251 = Sin(var1[15]);
  t4032 = t3711*t3699;
  t4034 = t3270*t3930;
  t4039 = t4032 + t4034;
  t2455 = t274*t416;
  t2473 = t933*t888*t1069;
  t2674 = t2455 + t2473;
  t2167 = t416*t933*t888;
  t2176 = -1.*t274*t1069;
  t2182 = t2167 + t2176;
  t2429 = -1.*t1503*t1517*t933;
  t2722 = t1646*t2674;
  t2937 = t2429 + t2722;
  t4123 = t1646*t1503*t933;
  t4165 = t1517*t2674;
  t4169 = t4123 + t4165;
  t4263 = t1468*t2182;
  t4264 = -1.*t210*t2937;
  t4302 = t4263 + t4264;
  t4201 = t3361*t4169;
  t4316 = t3478*t4302;
  t4320 = t4201 + t4316;
  t4455 = t3478*t4169;
  t4460 = -1.*t3361*t4302;
  t4463 = t4455 + t4460;
  t4433 = -1.*t3270*t4320;
  t4538 = t3711*t4463;
  t4558 = t4433 + t4538;
  t4611 = t3711*t4320;
  t4624 = t3270*t4463;
  t4634 = t4611 + t4624;
  t3025 = t1517*t888;
  t3029 = t1646*t1503*t1069;
  t3083 = t3025 + t3029;
  t4802 = -1.*t1646*t888;
  t4812 = t1503*t1517*t1069;
  t4819 = t4802 + t4812;
  t4839 = t1468*t1503*t416;
  t4877 = -1.*t210*t3083;
  t4920 = t4839 + t4877;
  t4829 = t3361*t4819;
  t4923 = t3478*t4920;
  t4924 = t4829 + t4923;
  t4930 = t3478*t4819;
  t4993 = -1.*t3361*t4920;
  t5074 = t4930 + t4993;
  t4929 = -1.*t3270*t4924;
  t5104 = t3711*t5074;
  t5108 = t4929 + t5104;
  t5148 = t3711*t4924;
  t5165 = t3270*t5074;
  t5173 = t5148 + t5165;

  p_output1(0)=t1468*t1971 + t1329*t210;
  p_output1(1)=t210*t2182 + t1468*t2937;
  p_output1(2)=t1468*t3083 + t1503*t210*t416;
  p_output1(3)=-1.*t3251*t4014 - 1.*t4026*t4039 - 0.000796*(t4014*t4026 - 1.*t3251*t4039);
  p_output1(4)=-1.*t3251*t4558 - 1.*t4026*t4634 - 0.000796*(t4026*t4558 - 1.*t3251*t4634);
  p_output1(5)=-1.*t3251*t5108 - 1.*t4026*t5173 - 0.000796*(t4026*t5108 - 1.*t3251*t5173);
  p_output1(6)=-1.*t4014*t4026 + t3251*t4039 + 0.000796*(t3251*t4014 + t4026*t4039);
  p_output1(7)=-1.*t4026*t4558 + t3251*t4634 + 0.000796*(t3251*t4558 + t4026*t4634);
  p_output1(8)=-1.*t4026*t5108 + t3251*t5173 + 0.000796*(t3251*t5108 + t4026*t5173);
}


       
void R_RightFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
