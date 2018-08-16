/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:20 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBack.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t1573;
  double t415;
  double t416;
  double t566;
  double t1815;
  double t409;
  double t2951;
  double t3029;
  double t3614;
  double t1521;
  double t2376;
  double t2696;
  double t3726;
  double t200;
  double t3797;
  double t3807;
  double t3820;
  double t2755;
  double t3738;
  double t3748;
  double t3844;
  double t3948;
  double t3953;
  double t3960;
  double t3919;
  double t3928;
  double t3929;
  double t3979;
  double t3982;
  double t3990;
  double t3946;
  double t3961;
  double t3963;
  double t4019;
  double t4026;
  double t4037;
  double t4043;
  double t4028;
  double t4030;
  double t4052;
  double t4060;
  double t4062;
  double t4036;
  double t4044;
  double t4050;
  double t3791;
  double t3857;
  double t3976;
  double t3995;
  double t4051;
  double t4063;
  t1573 = Cos(var1[0]);
  t415 = Cos(var1[2]);
  t416 = Sin(var1[0]);
  t566 = Sin(var1[1]);
  t1815 = Sin(var1[2]);
  t409 = Cos(var1[3]);
  t2951 = t1573*t415;
  t3029 = -1.*t416*t566*t1815;
  t3614 = t2951 + t3029;
  t1521 = t415*t416*t566;
  t2376 = t1573*t1815;
  t2696 = t1521 + t2376;
  t3726 = Sin(var1[3]);
  t200 = Cos(var1[4]);
  t3797 = t409*t3614;
  t3807 = -1.*t2696*t3726;
  t3820 = t3797 + t3807;
  t2755 = t409*t2696;
  t3738 = t3614*t3726;
  t3748 = t2755 + t3738;
  t3844 = Sin(var1[4]);
  t3948 = t415*t416;
  t3953 = t1573*t566*t1815;
  t3960 = t3948 + t3953;
  t3919 = -1.*t1573*t415*t566;
  t3928 = t416*t1815;
  t3929 = t3919 + t3928;
  t3979 = t409*t3960;
  t3982 = -1.*t3929*t3726;
  t3990 = t3979 + t3982;
  t3946 = t409*t3929;
  t3961 = t3960*t3726;
  t3963 = t3946 + t3961;
  t4019 = Cos(var1[1]);
  t4026 = 0. + t4019;
  t4037 = -1.*t4026*t1815;
  t4043 = 0. + t4037;
  t4028 = t4026*t415;
  t4030 = 0. + t4028;
  t4052 = t409*t4043;
  t4060 = -1.*t4030*t3726;
  t4062 = t4052 + t4060;
  t4036 = t4030*t409;
  t4044 = t4043*t3726;
  t4050 = t4036 + t4044;
  t3791 = t200*t3748;
  t3857 = t3820*t3844;
  t3976 = t200*t3963;
  t3995 = t3990*t3844;
  t4051 = t200*t4050;
  t4063 = t4062*t3844;

  p_output1(0)=t3791 + 0.000796*(t200*t3820 - 1.*t3748*t3844) + t3857;
  p_output1(1)=t3976 + 0.000796*(-1.*t3844*t3963 + t200*t3990) + t3995;
  p_output1(2)=t4051 + 0.000796*(-1.*t3844*t4050 + t200*t4062) + t4063;
  p_output1(3)=-1.*t4019*t416;
  p_output1(4)=t1573*t4019;
  p_output1(5)=0. + t566;
  p_output1(6)=-1.*t200*t3820 + t3748*t3844 + 0.000796*(t3791 + t3857);
  p_output1(7)=t3844*t3963 - 1.*t200*t3990 + 0.000796*(t3976 + t3995);
  p_output1(8)=t3844*t4050 - 1.*t200*t4062 + 0.000796*(t4051 + t4063);
}


       
void R_LeftFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
