/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:14 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootBack.h"

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
  double t750;
  double t1134;
  double t1188;
  double t1169;
  double t1192;
  double t1015;
  double t1075;
  double t741;
  double t1170;
  double t1228;
  double t1250;
  double t1412;
  double t1044;
  double t1307;
  double t1337;
  double t730;
  double t1487;
  double t1621;
  double t1716;
  double t1717;
  double t1779;
  double t1866;
  double t1917;
  double t1954;
  double t1961;
  double t1972;
  double t1999;
  double t2133;
  double t1355;
  double t2061;
  double t2074;
  double t632;
  double t2134;
  double t2146;
  double t2192;
  double t2255;
  double t2117;
  double t2200;
  double t2214;
  double t294;
  double t2257;
  double t2260;
  double t2265;
  double t2436;
  double t2584;
  double t2589;
  double t2407;
  double t2621;
  double t2681;
  double t2700;
  double t2701;
  double t2703;
  double t2708;
  double t2723;
  double t2750;
  double t2763;
  double t2771;
  double t2837;
  double t2688;
  double t2862;
  double t2863;
  double t2873;
  double t2916;
  double t2978;
  double t2871;
  double t3030;
  double t3033;
  double t3088;
  double t3113;
  double t3148;
  double t3333;
  double t3338;
  double t3354;
  double t3371;
  double t3373;
  double t3375;
  double t3380;
  double t3383;
  double t3389;
  double t3370;
  double t3394;
  double t3410;
  double t3428;
  double t3474;
  double t3480;
  double t3412;
  double t3508;
  double t3567;
  double t3578;
  double t3581;
  double t3589;
  double t2222;
  double t2268;
  double t3081;
  double t3201;
  double t3575;
  double t3591;
  t750 = Cos(var1[3]);
  t1134 = Cos(var1[5]);
  t1188 = Sin(var1[4]);
  t1169 = Sin(var1[3]);
  t1192 = Sin(var1[5]);
  t1015 = Cos(var1[4]);
  t1075 = Sin(var1[11]);
  t741 = Cos(var1[11]);
  t1170 = -1.*t1134*t1169;
  t1228 = t750*t1188*t1192;
  t1250 = t1170 + t1228;
  t1412 = Cos(var1[13]);
  t1044 = t741*t750*t1015;
  t1307 = t1075*t1250;
  t1337 = t1044 + t1307;
  t730 = Sin(var1[13]);
  t1487 = Cos(var1[12]);
  t1621 = t750*t1134*t1188;
  t1716 = t1169*t1192;
  t1717 = t1621 + t1716;
  t1779 = t1487*t1717;
  t1866 = Sin(var1[12]);
  t1917 = -1.*t750*t1015*t1075;
  t1954 = t741*t1250;
  t1961 = t1917 + t1954;
  t1972 = -1.*t1866*t1961;
  t1999 = t1779 + t1972;
  t2133 = Cos(var1[14]);
  t1355 = t730*t1337;
  t2061 = t1412*t1999;
  t2074 = t1355 + t2061;
  t632 = Sin(var1[14]);
  t2134 = t1412*t1337;
  t2146 = -1.*t730*t1999;
  t2192 = t2134 + t2146;
  t2255 = Cos(var1[15]);
  t2117 = -1.*t632*t2074;
  t2200 = t2133*t2192;
  t2214 = t2117 + t2200;
  t294 = Sin(var1[15]);
  t2257 = t2133*t2074;
  t2260 = t632*t2192;
  t2265 = t2257 + t2260;
  t2436 = t750*t1134;
  t2584 = t1169*t1188*t1192;
  t2589 = t2436 + t2584;
  t2407 = t741*t1015*t1169;
  t2621 = t1075*t2589;
  t2681 = t2407 + t2621;
  t2700 = t1134*t1169*t1188;
  t2701 = -1.*t750*t1192;
  t2703 = t2700 + t2701;
  t2708 = t1487*t2703;
  t2723 = -1.*t1015*t1075*t1169;
  t2750 = t741*t2589;
  t2763 = t2723 + t2750;
  t2771 = -1.*t1866*t2763;
  t2837 = t2708 + t2771;
  t2688 = t730*t2681;
  t2862 = t1412*t2837;
  t2863 = t2688 + t2862;
  t2873 = t1412*t2681;
  t2916 = -1.*t730*t2837;
  t2978 = t2873 + t2916;
  t2871 = -1.*t632*t2863;
  t3030 = t2133*t2978;
  t3033 = t2871 + t3030;
  t3088 = t2133*t2863;
  t3113 = t632*t2978;
  t3148 = t3088 + t3113;
  t3333 = -1.*t741*t1188;
  t3338 = t1015*t1075*t1192;
  t3354 = t3333 + t3338;
  t3371 = t1487*t1015*t1134;
  t3373 = t1075*t1188;
  t3375 = t741*t1015*t1192;
  t3380 = t3373 + t3375;
  t3383 = -1.*t1866*t3380;
  t3389 = t3371 + t3383;
  t3370 = t730*t3354;
  t3394 = t1412*t3389;
  t3410 = t3370 + t3394;
  t3428 = t1412*t3354;
  t3474 = -1.*t730*t3389;
  t3480 = t3428 + t3474;
  t3412 = -1.*t632*t3410;
  t3508 = t2133*t3480;
  t3567 = t3412 + t3508;
  t3578 = t2133*t3410;
  t3581 = t632*t3480;
  t3589 = t3578 + t3581;
  t2222 = t294*t2214;
  t2268 = t2255*t2265;
  t3081 = t294*t3033;
  t3201 = t2255*t3148;
  t3575 = t294*t3567;
  t3591 = t2255*t3589;

  p_output1(0)=t2222 + t2268 + 0.000796*(t2214*t2255 - 1.*t2265*t294);
  p_output1(1)=t3081 + 0.000796*(t2255*t3033 - 1.*t294*t3148) + t3201;
  p_output1(2)=t3575 + 0.000796*(t2255*t3567 - 1.*t294*t3589) + t3591;
  p_output1(3)=t1717*t1866 + t1487*t1961;
  p_output1(4)=t1866*t2703 + t1487*t2763;
  p_output1(5)=t1015*t1134*t1866 + t1487*t3380;
  p_output1(6)=-1.*t2214*t2255 + 0.000796*(t2222 + t2268) + t2265*t294;
  p_output1(7)=-1.*t2255*t3033 + t294*t3148 + 0.000796*(t3081 + t3201);
  p_output1(8)=-1.*t2255*t3567 + t294*t3589 + 0.000796*(t3575 + t3591);
}


       
void R_RightFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
