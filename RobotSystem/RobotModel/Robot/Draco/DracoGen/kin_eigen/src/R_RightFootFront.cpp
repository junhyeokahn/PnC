/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:11 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootFront.h"

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
  double t1324;
  double t1542;
  double t1685;
  double t1623;
  double t1686;
  double t1346;
  double t1457;
  double t1323;
  double t1639;
  double t1767;
  double t1854;
  double t2365;
  double t1413;
  double t2209;
  double t2271;
  double t1097;
  double t2411;
  double t2445;
  double t2476;
  double t2487;
  double t2491;
  double t2495;
  double t2521;
  double t2546;
  double t2576;
  double t2612;
  double t2617;
  double t2903;
  double t2361;
  double t2708;
  double t2789;
  double t808;
  double t2945;
  double t2967;
  double t2992;
  double t3137;
  double t2900;
  double t3020;
  double t3048;
  double t20;
  double t3143;
  double t3157;
  double t3184;
  double t3307;
  double t3341;
  double t3342;
  double t3302;
  double t3345;
  double t3353;
  double t3367;
  double t3377;
  double t3380;
  double t3396;
  double t3417;
  double t3439;
  double t3452;
  double t3454;
  double t3503;
  double t3362;
  double t3523;
  double t3548;
  double t3596;
  double t3631;
  double t3650;
  double t3571;
  double t3668;
  double t3680;
  double t3690;
  double t3697;
  double t3713;
  double t4005;
  double t4014;
  double t4047;
  double t4085;
  double t4106;
  double t4109;
  double t4113;
  double t4122;
  double t4123;
  double t4056;
  double t4128;
  double t4133;
  double t4141;
  double t4146;
  double t4157;
  double t4138;
  double t4169;
  double t4204;
  double t4257;
  double t4267;
  double t4280;
  double t3076;
  double t3237;
  double t3682;
  double t3718;
  double t4231;
  double t4285;
  t1324 = Cos(var1[3]);
  t1542 = Cos(var1[5]);
  t1685 = Sin(var1[4]);
  t1623 = Sin(var1[3]);
  t1686 = Sin(var1[5]);
  t1346 = Cos(var1[4]);
  t1457 = Sin(var1[11]);
  t1323 = Cos(var1[11]);
  t1639 = -1.*t1542*t1623;
  t1767 = t1324*t1685*t1686;
  t1854 = t1639 + t1767;
  t2365 = Cos(var1[13]);
  t1413 = t1323*t1324*t1346;
  t2209 = t1457*t1854;
  t2271 = t1413 + t2209;
  t1097 = Sin(var1[13]);
  t2411 = Cos(var1[12]);
  t2445 = t1324*t1542*t1685;
  t2476 = t1623*t1686;
  t2487 = t2445 + t2476;
  t2491 = t2411*t2487;
  t2495 = Sin(var1[12]);
  t2521 = -1.*t1324*t1346*t1457;
  t2546 = t1323*t1854;
  t2576 = t2521 + t2546;
  t2612 = -1.*t2495*t2576;
  t2617 = t2491 + t2612;
  t2903 = Cos(var1[14]);
  t2361 = t1097*t2271;
  t2708 = t2365*t2617;
  t2789 = t2361 + t2708;
  t808 = Sin(var1[14]);
  t2945 = t2365*t2271;
  t2967 = -1.*t1097*t2617;
  t2992 = t2945 + t2967;
  t3137 = Cos(var1[15]);
  t2900 = -1.*t808*t2789;
  t3020 = t2903*t2992;
  t3048 = t2900 + t3020;
  t20 = Sin(var1[15]);
  t3143 = t2903*t2789;
  t3157 = t808*t2992;
  t3184 = t3143 + t3157;
  t3307 = t1324*t1542;
  t3341 = t1623*t1685*t1686;
  t3342 = t3307 + t3341;
  t3302 = t1323*t1346*t1623;
  t3345 = t1457*t3342;
  t3353 = t3302 + t3345;
  t3367 = t1542*t1623*t1685;
  t3377 = -1.*t1324*t1686;
  t3380 = t3367 + t3377;
  t3396 = t2411*t3380;
  t3417 = -1.*t1346*t1457*t1623;
  t3439 = t1323*t3342;
  t3452 = t3417 + t3439;
  t3454 = -1.*t2495*t3452;
  t3503 = t3396 + t3454;
  t3362 = t1097*t3353;
  t3523 = t2365*t3503;
  t3548 = t3362 + t3523;
  t3596 = t2365*t3353;
  t3631 = -1.*t1097*t3503;
  t3650 = t3596 + t3631;
  t3571 = -1.*t808*t3548;
  t3668 = t2903*t3650;
  t3680 = t3571 + t3668;
  t3690 = t2903*t3548;
  t3697 = t808*t3650;
  t3713 = t3690 + t3697;
  t4005 = -1.*t1323*t1685;
  t4014 = t1346*t1457*t1686;
  t4047 = t4005 + t4014;
  t4085 = t2411*t1346*t1542;
  t4106 = t1457*t1685;
  t4109 = t1323*t1346*t1686;
  t4113 = t4106 + t4109;
  t4122 = -1.*t2495*t4113;
  t4123 = t4085 + t4122;
  t4056 = t1097*t4047;
  t4128 = t2365*t4123;
  t4133 = t4056 + t4128;
  t4141 = t2365*t4047;
  t4146 = -1.*t1097*t4123;
  t4157 = t4141 + t4146;
  t4138 = -1.*t808*t4133;
  t4169 = t2903*t4157;
  t4204 = t4138 + t4169;
  t4257 = t2903*t4133;
  t4267 = t808*t4157;
  t4280 = t4257 + t4267;
  t3076 = t20*t3048;
  t3237 = t3137*t3184;
  t3682 = t20*t3680;
  t3718 = t3137*t3713;
  t4231 = t20*t4204;
  t4285 = t3137*t4280;

  p_output1(0)=t3076 + 0.000796*(t3048*t3137 - 1.*t20*t3184) + t3237;
  p_output1(1)=t3682 + 0.000796*(t3137*t3680 - 1.*t20*t3713) + t3718;
  p_output1(2)=t4231 + 0.000796*(t3137*t4204 - 1.*t20*t4280) + t4285;
  p_output1(3)=t2487*t2495 + t2411*t2576;
  p_output1(4)=t2495*t3380 + t2411*t3452;
  p_output1(5)=t1346*t1542*t2495 + t2411*t4113;
  p_output1(6)=-1.*t3048*t3137 + t20*t3184 + 0.000796*(t3076 + t3237);
  p_output1(7)=-1.*t3137*t3680 + t20*t3713 + 0.000796*(t3682 + t3718);
  p_output1(8)=-1.*t3137*t4204 + t20*t4280 + 0.000796*(t4231 + t4285);
}


       
void R_RightFootFront(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
