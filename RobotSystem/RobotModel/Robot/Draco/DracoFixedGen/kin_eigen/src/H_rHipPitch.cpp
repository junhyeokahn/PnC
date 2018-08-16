/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:10 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipPitch.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t454;
  double t1957;
  double t236;
  double t1963;
  double t1983;
  double t2044;
  double t2051;
  double t2136;
  double t2137;
  double t2063;
  double t2070;
  double t2072;
  double t1531;
  double t1996;
  double t2015;
  double t2106;
  double t2112;
  double t2123;
  double t2130;
  double t2131;
  double t2132;
  double t2133;
  double t2138;
  double t2140;
  double t2143;
  double t2146;
  double t2148;
  double t2154;
  double t2073;
  double t2082;
  double t2083;
  double t2035;
  double t2039;
  double t2040;
  double t2095;
  double t2105;
  double t2061;
  double t2052;
  double t2053;
  t454 = Cos(var1[7]);
  t1957 = Sin(var1[5]);
  t236 = Cos(var1[5]);
  t1963 = Sin(var1[6]);
  t1983 = Sin(var1[7]);
  t2044 = Cos(var1[6]);
  t2051 = 0. + t2044;
  t2136 = -1.*t454;
  t2137 = 1. + t2136;
  t2063 = t454*t1957*t1963;
  t2070 = t236*t1983;
  t2072 = t2063 + t2070;
  t1531 = t236*t454;
  t1996 = -1.*t1957*t1963*t1983;
  t2015 = t1531 + t1996;
  t2106 = -1.*t236;
  t2112 = 1. + t2106;
  t2123 = -1.*t2044;
  t2130 = 1. + t2123;
  t2131 = -0.330988*t2130;
  t2132 = -0.90524*t1963;
  t2133 = 0. + t2131 + t2132;
  t2138 = -0.97024*t2137;
  t2140 = -0.066675*t1983;
  t2143 = 0. + t2138 + t2140;
  t2146 = -0.066675*t2137;
  t2148 = 0.97024*t1983;
  t2154 = 0. + t2146 + t2148;
  t2073 = -1.*t236*t454*t1963;
  t2082 = t1957*t1983;
  t2083 = t2073 + t2082;
  t2035 = t454*t1957;
  t2039 = t236*t1963*t1983;
  t2040 = t2035 + t2039;
  t2095 = t2051*t454;
  t2105 = 0. + t2095;
  t2061 = 0. + t1963;
  t2052 = -1.*t2051*t1983;
  t2053 = 0. + t2052;

  p_output1(0)=t2015;
  p_output1(1)=t2040;
  p_output1(2)=t2053;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1957*t2044;
  p_output1(5)=t2044*t236;
  p_output1(6)=t2061;
  p_output1(7)=0.;
  p_output1(8)=t2072;
  p_output1(9)=t2083;
  p_output1(10)=t2105;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.260988*t1957 - 0.066675*t2015 + 0.340988*t1957*t2044 - 0.97024*t2072 - 0.066675*t2112 - 1.*t1957*t2133 + t1957*t1963*t2143 + t2154*t236;
  p_output1(13)=0. + 0.066675*t1957 - 0.066675*t2040 - 0.97024*t2083 - 0.260988*t2112 + t1957*t2154 - 0.340988*t2044*t236 + t2133*t236 - 1.*t1963*t2143*t236;
  p_output1(14)=0. + 0.330988*t1963 - 0.066675*t2053 - 0.340988*t2061 - 0.97024*t2105 - 0.90524*t2130 + t2051*t2143;
  p_output1(15)=1.;
}


       
void H_rHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
