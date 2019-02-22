/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:19 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_lKnee.h"

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
  double t216;
  double t232;
  double t215;
  double t221;
  double t315;
  double t812;
  double t571;
  double t693;
  double t729;
  double t807;
  double t835;
  double t202;
  double t1209;
  double t1220;
  double t1224;
  double t210;
  double t227;
  double t521;
  double t528;
  double t556;
  double t811;
  double t860;
  double t869;
  double t905;
  double t938;
  double t981;
  double t1237;
  double t61;
  double t1291;
  double t1295;
  double t1296;
  double t1248;
  double t1387;
  double t1399;
  double t1406;
  double t1273;
  double t1274;
  double t1275;
  double t1286;
  double t1297;
  double t1328;
  double t1353;
  double t1356;
  double t1357;
  double t1734;
  double t1838;
  double t1841;
  double t1691;
  double t1698;
  double t1699;
  double t1703;
  double t1707;
  double t1708;
  double t984;
  double t1239;
  double t1242;
  double t1250;
  double t1251;
  double t1254;
  double t1367;
  double t1423;
  double t1447;
  double t1505;
  double t1579;
  double t1582;
  double t1732;
  double t1873;
  double t1886;
  double t1896;
  double t1907;
  double t1987;
  double t3004;
  double t3051;
  double t3287;
  double t3292;
  double t2080;
  double t2089;
  double t2261;
  double t3392;
  double t3409;
  double t3476;
  double t3478;
  double t1246;
  double t1261;
  double t1270;
  double t2785;
  double t2820;
  double t2829;
  double t3122;
  double t3141;
  double t3143;
  double t3192;
  double t3197;
  double t3228;
  double t3302;
  double t3315;
  double t3337;
  double t3357;
  double t3365;
  double t3371;
  double t2282;
  double t2284;
  double t2437;
  double t3414;
  double t3431;
  double t3437;
  double t3465;
  double t3472;
  double t3473;
  double t3481;
  double t3483;
  double t3485;
  double t3491;
  double t3492;
  double t3495;
  double t1486;
  double t1628;
  double t1678;
  double t2835;
  double t2836;
  double t2847;
  double t2562;
  double t2761;
  double t2779;
  double t1895;
  double t2033;
  double t2061;
  double t2864;
  double t2913;
  double t2975;
  t216 = Cos(var1[5]);
  t232 = Sin(var1[3]);
  t215 = Cos(var1[3]);
  t221 = Sin(var1[4]);
  t315 = Sin(var1[5]);
  t812 = Cos(var1[4]);
  t571 = Cos(var1[6]);
  t693 = -1.*t216*t232;
  t729 = t215*t221*t315;
  t807 = t693 + t729;
  t835 = Sin(var1[6]);
  t202 = Cos(var1[8]);
  t1209 = t215*t812*t571;
  t1220 = t807*t835;
  t1224 = t1209 + t1220;
  t210 = Cos(var1[7]);
  t227 = t215*t216*t221;
  t521 = t232*t315;
  t528 = t227 + t521;
  t556 = t210*t528;
  t811 = t571*t807;
  t860 = -1.*t215*t812*t835;
  t869 = t811 + t860;
  t905 = Sin(var1[7]);
  t938 = -1.*t869*t905;
  t981 = t556 + t938;
  t1237 = Sin(var1[8]);
  t61 = Sin(var1[9]);
  t1291 = t215*t216;
  t1295 = t232*t221*t315;
  t1296 = t1291 + t1295;
  t1248 = Cos(var1[9]);
  t1387 = t812*t571*t232;
  t1399 = t1296*t835;
  t1406 = t1387 + t1399;
  t1273 = t216*t232*t221;
  t1274 = -1.*t215*t315;
  t1275 = t1273 + t1274;
  t1286 = t210*t1275;
  t1297 = t571*t1296;
  t1328 = -1.*t812*t232*t835;
  t1353 = t1297 + t1328;
  t1356 = -1.*t1353*t905;
  t1357 = t1286 + t1356;
  t1734 = -1.*t571*t221;
  t1838 = t812*t315*t835;
  t1841 = t1734 + t1838;
  t1691 = t812*t216*t210;
  t1698 = t812*t571*t315;
  t1699 = t221*t835;
  t1703 = t1698 + t1699;
  t1707 = -1.*t1703*t905;
  t1708 = t1691 + t1707;
  t984 = t202*t981;
  t1239 = t1224*t1237;
  t1242 = t984 + t1239;
  t1250 = t202*t1224;
  t1251 = -1.*t981*t1237;
  t1254 = t1250 + t1251;
  t1367 = t202*t1357;
  t1423 = t1406*t1237;
  t1447 = t1367 + t1423;
  t1505 = t202*t1406;
  t1579 = -1.*t1357*t1237;
  t1582 = t1505 + t1579;
  t1732 = t202*t1708;
  t1873 = t1841*t1237;
  t1886 = t1732 + t1873;
  t1896 = t202*t1841;
  t1907 = -1.*t1708*t1237;
  t1987 = t1896 + t1907;
  t3004 = -1.*t571;
  t3051 = 1. + t3004;
  t3287 = -1.*t210;
  t3292 = 1. + t3287;
  t2080 = t210*t869;
  t2089 = t528*t905;
  t2261 = t2080 + t2089;
  t3392 = -1.*t202;
  t3409 = 1. + t3392;
  t3476 = -1.*t1248;
  t3478 = 1. + t3476;
  t1246 = -1.*t61*t1242;
  t1261 = t1248*t1254;
  t1270 = t1246 + t1261;
  t2785 = t1248*t1242;
  t2820 = t61*t1254;
  t2829 = t2785 + t2820;
  t3122 = 0.087*t3051;
  t3141 = 0.0222*t835;
  t3143 = 0. + t3122 + t3141;
  t3192 = -0.0222*t3051;
  t3197 = 0.087*t835;
  t3228 = 0. + t3192 + t3197;
  t3302 = 0.157*t3292;
  t3315 = -0.3151*t905;
  t3337 = 0. + t3302 + t3315;
  t3357 = -0.3151*t3292;
  t3365 = -0.157*t905;
  t3371 = 0. + t3357 + t3365;
  t2282 = t210*t1353;
  t2284 = t1275*t905;
  t2437 = t2282 + t2284;
  t3414 = -0.3801*t3409;
  t3431 = -0.0222*t1237;
  t3437 = 0. + t3414 + t3431;
  t3465 = -0.0222*t3409;
  t3472 = 0.3801*t1237;
  t3473 = 0. + t3465 + t3472;
  t3481 = -0.8601*t3478;
  t3483 = -0.0222*t61;
  t3485 = 0. + t3481 + t3483;
  t3491 = -0.0222*t3478;
  t3492 = 0.8601*t61;
  t3495 = 0. + t3491 + t3492;
  t1486 = -1.*t61*t1447;
  t1628 = t1248*t1582;
  t1678 = t1486 + t1628;
  t2835 = t1248*t1447;
  t2836 = t61*t1582;
  t2847 = t2835 + t2836;
  t2562 = t210*t1703;
  t2761 = t812*t216*t905;
  t2779 = t2562 + t2761;
  t1895 = -1.*t61*t1886;
  t2033 = t1248*t1987;
  t2061 = t1895 + t2033;
  t2864 = t1248*t1886;
  t2913 = t61*t1987;
  t2975 = t2864 + t2913;

  p_output1(0)=t1270;
  p_output1(1)=t1678;
  p_output1(2)=t2061;
  p_output1(3)=0.;
  p_output1(4)=t2261;
  p_output1(5)=t2437;
  p_output1(6)=t2779;
  p_output1(7)=0.;
  p_output1(8)=t2829;
  p_output1(9)=t2847;
  p_output1(10)=t2975;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t1270 + 0.1502*t2261 - 0.8601*t2829 + t1224*t3473 + t1242*t3485 + t1254*t3495 + t3371*t528 + t3143*t807 + t215*t3228*t812 + t3337*t869 + t3437*t981 + var1(0);
  p_output1(13)=0. - 0.0222*t1678 + 0.1502*t2437 - 0.8601*t2847 + t1296*t3143 + t1353*t3337 + t1275*t3371 + t1357*t3437 + t1406*t3473 + t1447*t3485 + t1582*t3495 + t232*t3228*t812 + var1(1);
  p_output1(14)=0. - 0.0222*t2061 + 0.1502*t2779 - 0.8601*t2975 - 1.*t221*t3228 + t1703*t3337 + t1708*t3437 + t1841*t3473 + t1886*t3485 + t1987*t3495 + t3143*t315*t812 + t216*t3371*t812 + var1(2);
  p_output1(15)=1.;
}


       
void H_lKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
