/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:30 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBottom.h"

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
  double t1032;
  double t916;
  double t1091;
  double t982;
  double t1263;
  double t296;
  double t655;
  double t1476;
  double t1596;
  double t1712;
  double t1621;
  double t1626;
  double t1642;
  double t1026;
  double t1264;
  double t1299;
  double t1402;
  double t1615;
  double t1618;
  double t2719;
  double t3000;
  double t3010;
  double t3075;
  double t2739;
  double t2834;
  double t2901;
  double t3116;
  double t3216;
  double t2943;
  double t3131;
  double t3147;
  double t2704;
  double t3265;
  double t3273;
  double t3321;
  double t3424;
  double t3194;
  double t3325;
  double t3362;
  double t2648;
  double t3490;
  double t3574;
  double t3682;
  double t1992;
  double t2006;
  double t2029;
  double t1879;
  double t1910;
  double t1919;
  double t1921;
  double t1924;
  double t1983;
  double t4031;
  double t4044;
  double t4146;
  double t3883;
  double t3918;
  double t3965;
  double t3966;
  double t4170;
  double t4178;
  double t4220;
  double t4261;
  double t4275;
  double t4184;
  double t4303;
  double t4308;
  double t4319;
  double t4381;
  double t4388;
  double t2287;
  double t2319;
  double t2341;
  double t4626;
  double t4630;
  double t4655;
  double t4532;
  double t4534;
  double t4586;
  double t4609;
  double t4665;
  double t4670;
  double t4694;
  double t4698;
  double t4707;
  double t4678;
  double t4711;
  double t4759;
  double t4768;
  double t4771;
  double t4866;
  t1032 = Cos(var1[3]);
  t916 = Cos(var1[5]);
  t1091 = Sin(var1[4]);
  t982 = Sin(var1[3]);
  t1263 = Sin(var1[5]);
  t296 = Cos(var1[7]);
  t655 = Cos(var1[6]);
  t1476 = Cos(var1[4]);
  t1596 = Sin(var1[6]);
  t1712 = Sin(var1[7]);
  t1621 = t1032*t916*t1091;
  t1626 = t982*t1263;
  t1642 = t1621 + t1626;
  t1026 = -1.*t916*t982;
  t1264 = t1032*t1091*t1263;
  t1299 = t1026 + t1264;
  t1402 = t655*t1299;
  t1615 = -1.*t1032*t1476*t1596;
  t1618 = t1402 + t1615;
  t2719 = Cos(var1[8]);
  t3000 = t1032*t1476*t655;
  t3010 = t1299*t1596;
  t3075 = t3000 + t3010;
  t2739 = t296*t1642;
  t2834 = -1.*t1618*t1712;
  t2901 = t2739 + t2834;
  t3116 = Sin(var1[8]);
  t3216 = Cos(var1[9]);
  t2943 = t2719*t2901;
  t3131 = t3075*t3116;
  t3147 = t2943 + t3131;
  t2704 = Sin(var1[9]);
  t3265 = t2719*t3075;
  t3273 = -1.*t2901*t3116;
  t3321 = t3265 + t3273;
  t3424 = Cos(var1[10]);
  t3194 = -1.*t2704*t3147;
  t3325 = t3216*t3321;
  t3362 = t3194 + t3325;
  t2648 = Sin(var1[10]);
  t3490 = t3216*t3147;
  t3574 = t2704*t3321;
  t3682 = t3490 + t3574;
  t1992 = t916*t982*t1091;
  t2006 = -1.*t1032*t1263;
  t2029 = t1992 + t2006;
  t1879 = t1032*t916;
  t1910 = t982*t1091*t1263;
  t1919 = t1879 + t1910;
  t1921 = t655*t1919;
  t1924 = -1.*t1476*t982*t1596;
  t1983 = t1921 + t1924;
  t4031 = t1476*t655*t982;
  t4044 = t1919*t1596;
  t4146 = t4031 + t4044;
  t3883 = t296*t2029;
  t3918 = -1.*t1983*t1712;
  t3965 = t3883 + t3918;
  t3966 = t2719*t3965;
  t4170 = t4146*t3116;
  t4178 = t3966 + t4170;
  t4220 = t2719*t4146;
  t4261 = -1.*t3965*t3116;
  t4275 = t4220 + t4261;
  t4184 = -1.*t2704*t4178;
  t4303 = t3216*t4275;
  t4308 = t4184 + t4303;
  t4319 = t3216*t4178;
  t4381 = t2704*t4275;
  t4388 = t4319 + t4381;
  t2287 = t1476*t655*t1263;
  t2319 = t1091*t1596;
  t2341 = t2287 + t2319;
  t4626 = -1.*t655*t1091;
  t4630 = t1476*t1263*t1596;
  t4655 = t4626 + t4630;
  t4532 = t1476*t916*t296;
  t4534 = -1.*t2341*t1712;
  t4586 = t4532 + t4534;
  t4609 = t2719*t4586;
  t4665 = t4655*t3116;
  t4670 = t4609 + t4665;
  t4694 = t2719*t4655;
  t4698 = -1.*t4586*t3116;
  t4707 = t4694 + t4698;
  t4678 = -1.*t2704*t4670;
  t4711 = t3216*t4707;
  t4759 = t4678 + t4711;
  t4768 = t3216*t4670;
  t4771 = t2704*t4707;
  t4866 = t4768 + t4771;

  p_output1(0)=t1642*t1712 + t1618*t296;
  p_output1(1)=t1712*t2029 + t1983*t296;
  p_output1(2)=t2341*t296 + t1476*t1712*t916;
  p_output1(3)=-1.*t2648*t3362 - 1.*t3424*t3682 - 0.000796*(t3362*t3424 - 1.*t2648*t3682);
  p_output1(4)=-1.*t2648*t4308 - 1.*t3424*t4388 - 0.000796*(t3424*t4308 - 1.*t2648*t4388);
  p_output1(5)=-1.*t2648*t4759 - 1.*t3424*t4866 - 0.000796*(t3424*t4759 - 1.*t2648*t4866);
  p_output1(6)=-1.*t3362*t3424 + t2648*t3682 + 0.000796*(t2648*t3362 + t3424*t3682);
  p_output1(7)=-1.*t3424*t4308 + t2648*t4388 + 0.000796*(t2648*t4308 + t3424*t4388);
  p_output1(8)=-1.*t3424*t4759 + t2648*t4866 + 0.000796*(t2648*t4759 + t3424*t4866);
}


       
void R_LeftFootBottom(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
