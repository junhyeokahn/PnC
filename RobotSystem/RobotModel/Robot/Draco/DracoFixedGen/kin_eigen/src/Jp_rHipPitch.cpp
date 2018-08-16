/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:09 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipPitch.h"

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
static void output1(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t660;
  double t1650;
  double t1970;
  double t1890;
  double t1997;
  double t1998;
  double t2008;
  double t2024;
  double t1935;
  double t1954;
  double t1963;
  double t1983;
  double t1992;
  double t2015;
  double t2025;
  double t2028;
  double t2033;
  double t2035;
  double t2037;
  double t2105;
  double t2106;
  double t2109;
  double t2083;
  double t2088;
  double t2091;
  double t2146;
  double t2148;
  double t2150;
  double t2156;
  double t2158;
  double t2040;
  double t2042;
  double t2044;
  double t2184;
  t660 = Cos(var1[5]);
  t1650 = Cos(var1[6]);
  t1970 = Sin(var1[6]);
  t1890 = Sin(var1[5]);
  t1997 = Cos(var1[7]);
  t1998 = -1.*t1997;
  t2008 = 1. + t1998;
  t2024 = Sin(var1[7]);
  t1935 = -1.*t1650;
  t1954 = 1. + t1935;
  t1963 = -0.330988*t1954;
  t1983 = -0.90524*t1970;
  t1992 = 0. + t1963 + t1983;
  t2015 = -0.97024*t2008;
  t2025 = -0.066675*t2024;
  t2028 = 0. + t2015 + t2025;
  t2033 = -0.066675*t2008;
  t2035 = 0.97024*t2024;
  t2037 = 0. + t2033 + t2035;
  t2105 = -0.90524*t1650;
  t2106 = -0.330988*t1970;
  t2109 = t2105 + t2106;
  t2083 = t660*t1997;
  t2088 = -1.*t1890*t1970*t2024;
  t2091 = t2083 + t2088;
  t2146 = -0.066675*t1997;
  t2148 = -0.97024*t2024;
  t2150 = t2146 + t2148;
  t2156 = 0.97024*t1997;
  t2158 = t2156 + t2025;
  t2040 = t660*t1997*t1970;
  t2042 = -1.*t1890*t2024;
  t2044 = t2040 + t2042;
  t2184 = 0. + t1650;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=-0.066675*t1890 - 1.*t1890*t2037 - 0.97024*t2044 - 0.260988*t660 + 0.340988*t1650*t660 - 1.*t1992*t660 + t1970*t2028*t660 - 0.066675*(-1.*t1890*t1997 - 1.*t1970*t2024*t660);
  p_output1(16)=-0.260988*t1890 + 0.340988*t1650*t1890 - 1.*t1890*t1992 + t1890*t1970*t2028 - 0.066675*t2091 + 0.066675*t660 + t2037*t660 - 0.97024*(t1890*t1970*t1997 + t2024*t660);
  p_output1(17)=0;
  p_output1(18)=-0.340988*t1890*t1970 - 0.97024*t1650*t1890*t1997 + 0.066675*t1650*t1890*t2024 + t1650*t1890*t2028 - 1.*t1890*t2109;
  p_output1(19)=0.340988*t1970*t660 + 0.97024*t1650*t1997*t660 - 0.066675*t1650*t2024*t660 - 1.*t1650*t2028*t660 + t2109*t660;
  p_output1(20)=-0.010000000000000009*t1650 + t1983 + 0.97024*t1970*t1997 - 0.066675*t1970*t2024 - 1.*t1970*t2028;
  p_output1(21)=-0.97024*t2091 + t1890*t1970*t2150 + t2158*t660 - 0.066675*(-1.*t1890*t1970*t1997 - 1.*t2024*t660);
  p_output1(22)=-0.066675*t2044 + t1890*t2158 - 1.*t1970*t2150*t660 - 0.97024*(t1890*t1997 + t1970*t2024*t660);
  p_output1(23)=0.066675*t1997*t2184 + 0.97024*t2024*t2184 + t2150*t2184;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
}


       
void Jp_rHipPitch(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
