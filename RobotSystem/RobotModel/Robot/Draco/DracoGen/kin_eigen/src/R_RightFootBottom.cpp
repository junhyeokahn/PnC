/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:35 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RightFootBottom.h"

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
  double t624;
  double t734;
  double t1064;
  double t887;
  double t1241;
  double t456;
  double t1393;
  double t1404;
  double t1446;
  double t1485;
  double t1534;
  double t1929;
  double t2048;
  double t986;
  double t1277;
  double t1369;
  double t1457;
  double t2190;
  double t2197;
  double t3421;
  double t3361;
  double t3386;
  double t3400;
  double t3327;
  double t3438;
  double t3439;
  double t3449;
  double t3504;
  double t3405;
  double t3454;
  double t3477;
  double t3219;
  double t3508;
  double t3573;
  double t3643;
  double t3751;
  double t3480;
  double t3668;
  double t3671;
  double t3092;
  double t3757;
  double t3823;
  double t3857;
  double t2594;
  double t2622;
  double t2639;
  double t2278;
  double t2417;
  double t2418;
  double t2587;
  double t2641;
  double t2643;
  double t4014;
  double t4025;
  double t4048;
  double t4098;
  double t4185;
  double t4203;
  double t4095;
  double t4247;
  double t4258;
  double t4335;
  double t4337;
  double t4338;
  double t4318;
  double t4368;
  double t4370;
  double t4546;
  double t4555;
  double t4579;
  double t2890;
  double t2939;
  double t2992;
  double t4701;
  double t4738;
  double t4745;
  double t4794;
  double t4805;
  double t4807;
  double t4772;
  double t4812;
  double t4855;
  double t4877;
  double t4902;
  double t4917;
  double t4857;
  double t4924;
  double t4937;
  double t4952;
  double t4954;
  double t4958;
  t624 = Cos(var1[3]);
  t734 = Cos(var1[5]);
  t1064 = Sin(var1[3]);
  t887 = Sin(var1[4]);
  t1241 = Sin(var1[5]);
  t456 = Sin(var1[12]);
  t1393 = Cos(var1[12]);
  t1404 = Cos(var1[4]);
  t1446 = Sin(var1[11]);
  t1485 = Cos(var1[11]);
  t1534 = -1.*t734*t1064;
  t1929 = t624*t887*t1241;
  t2048 = t1534 + t1929;
  t986 = t624*t734*t887;
  t1277 = t1064*t1241;
  t1369 = t986 + t1277;
  t1457 = -1.*t624*t1404*t1446;
  t2190 = t1485*t2048;
  t2197 = t1457 + t2190;
  t3421 = Cos(var1[13]);
  t3361 = t1485*t624*t1404;
  t3386 = t1446*t2048;
  t3400 = t3361 + t3386;
  t3327 = Sin(var1[13]);
  t3438 = t1393*t1369;
  t3439 = -1.*t456*t2197;
  t3449 = t3438 + t3439;
  t3504 = Cos(var1[14]);
  t3405 = t3327*t3400;
  t3454 = t3421*t3449;
  t3477 = t3405 + t3454;
  t3219 = Sin(var1[14]);
  t3508 = t3421*t3400;
  t3573 = -1.*t3327*t3449;
  t3643 = t3508 + t3573;
  t3751 = Cos(var1[15]);
  t3480 = -1.*t3219*t3477;
  t3668 = t3504*t3643;
  t3671 = t3480 + t3668;
  t3092 = Sin(var1[15]);
  t3757 = t3504*t3477;
  t3823 = t3219*t3643;
  t3857 = t3757 + t3823;
  t2594 = t624*t734;
  t2622 = t1064*t887*t1241;
  t2639 = t2594 + t2622;
  t2278 = t734*t1064*t887;
  t2417 = -1.*t624*t1241;
  t2418 = t2278 + t2417;
  t2587 = -1.*t1404*t1446*t1064;
  t2641 = t1485*t2639;
  t2643 = t2587 + t2641;
  t4014 = t1485*t1404*t1064;
  t4025 = t1446*t2639;
  t4048 = t4014 + t4025;
  t4098 = t1393*t2418;
  t4185 = -1.*t456*t2643;
  t4203 = t4098 + t4185;
  t4095 = t3327*t4048;
  t4247 = t3421*t4203;
  t4258 = t4095 + t4247;
  t4335 = t3421*t4048;
  t4337 = -1.*t3327*t4203;
  t4338 = t4335 + t4337;
  t4318 = -1.*t3219*t4258;
  t4368 = t3504*t4338;
  t4370 = t4318 + t4368;
  t4546 = t3504*t4258;
  t4555 = t3219*t4338;
  t4579 = t4546 + t4555;
  t2890 = t1446*t887;
  t2939 = t1485*t1404*t1241;
  t2992 = t2890 + t2939;
  t4701 = -1.*t1485*t887;
  t4738 = t1404*t1446*t1241;
  t4745 = t4701 + t4738;
  t4794 = t1393*t1404*t734;
  t4805 = -1.*t456*t2992;
  t4807 = t4794 + t4805;
  t4772 = t3327*t4745;
  t4812 = t3421*t4807;
  t4855 = t4772 + t4812;
  t4877 = t3421*t4745;
  t4902 = -1.*t3327*t4807;
  t4917 = t4877 + t4902;
  t4857 = -1.*t3219*t4855;
  t4924 = t3504*t4917;
  t4937 = t4857 + t4924;
  t4952 = t3504*t4855;
  t4954 = t3219*t4917;
  t4958 = t4952 + t4954;

  p_output1(0)=t1393*t2197 + t1369*t456;
  p_output1(1)=t1393*t2643 + t2418*t456;
  p_output1(2)=t1393*t2992 + t1404*t456*t734;
  p_output1(3)=-1.*t3092*t3671 - 1.*t3751*t3857 - 0.000796*(t3671*t3751 - 1.*t3092*t3857);
  p_output1(4)=-1.*t3092*t4370 - 1.*t3751*t4579 - 0.000796*(t3751*t4370 - 1.*t3092*t4579);
  p_output1(5)=-1.*t3092*t4937 - 1.*t3751*t4958 - 0.000796*(t3751*t4937 - 1.*t3092*t4958);
  p_output1(6)=-1.*t3671*t3751 + t3092*t3857 + 0.000796*(t3092*t3671 + t3751*t3857);
  p_output1(7)=-1.*t3751*t4370 + t3092*t4579 + 0.000796*(t3092*t4370 + t3751*t4579);
  p_output1(8)=-1.*t3751*t4937 + t3092*t4958 + 0.000796*(t3092*t4937 + t3751*t4958);
}


       
void R_RightFootBottom(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
