/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:30 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_LeftFootFront.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t817;
  double t1299;
  double t1626;
  double t1654;
  double t1726;
  double t123;
  double t329;
  double t749;
  double t823;
  double t931;
  double t975;
  double t1264;
  double t1879;
  double t2319;
  double t2462;
  double t2464;
  double t2537;
  double t2244;
  double t2256;
  double t2287;
  double t2719;
  double t2738;
  double t2739;
  double t3731;
  double t3743;
  double t3753;
  double t3859;
  double t3574;
  double t3584;
  double t3698;
  double t4044;
  double t4047;
  double t4105;
  double t4275;
  double t4303;
  double t4315;
  double t4445;
  double t4465;
  double t4467;
  double t4519;
  double t4694;
  double t4707;
  double t4765;
  double t4880;
  double t4889;
  double t4891;
  double t4896;
  double t4965;
  double t4978;
  double t4983;
  double t5075;
  double t5091;
  double t5118;
  double t1724;
  double t1767;
  double t1780;
  double t1901;
  double t1910;
  double t2049;
  double t5337;
  double t5350;
  double t5357;
  double t2525;
  double t2596;
  double t2610;
  double t2921;
  double t2943;
  double t3000;
  double t5419;
  double t5427;
  double t5428;
  double t5478;
  double t5481;
  double t5492;
  double t3755;
  double t3883;
  double t3966;
  double t4111;
  double t4170;
  double t4183;
  double t4319;
  double t4453;
  double t4461;
  double t5588;
  double t5589;
  double t5608;
  double t5626;
  double t5637;
  double t5643;
  double t4645;
  double t4647;
  double t4655;
  double t4894;
  double t4927;
  double t4939;
  double t5673;
  double t5684;
  double t5750;
  double t5776;
  double t5799;
  double t5809;
  double t5009;
  double t5027;
  double t5046;
  double t5836;
  double t5856;
  double t5904;
  double t5992;
  double t5993;
  double t6016;
  double t6270;
  double t6335;
  double t6348;
  double t6511;
  double t6547;
  double t6556;
  double t6589;
  double t6594;
  double t6599;
  double t6602;
  double t6603;
  double t6614;
  double t6635;
  double t6637;
  double t6638;
  double t6643;
  double t6644;
  double t6645;
  double t6664;
  double t6686;
  double t6689;
  t817 = Cos(var1[3]);
  t1299 = Cos(var1[6]);
  t1626 = -1.*t1299;
  t1654 = 1. + t1626;
  t1726 = Sin(var1[6]);
  t123 = Cos(var1[5]);
  t329 = Sin(var1[3]);
  t749 = -1.*t123*t329;
  t823 = Sin(var1[4]);
  t931 = Sin(var1[5]);
  t975 = t817*t823*t931;
  t1264 = t749 + t975;
  t1879 = Cos(var1[4]);
  t2319 = Cos(var1[7]);
  t2462 = -1.*t2319;
  t2464 = 1. + t2462;
  t2537 = Sin(var1[7]);
  t2244 = t1299*t1264;
  t2256 = -1.*t817*t1879*t1726;
  t2287 = t2244 + t2256;
  t2719 = t817*t123*t823;
  t2738 = t329*t931;
  t2739 = t2719 + t2738;
  t3731 = Cos(var1[8]);
  t3743 = -1.*t3731;
  t3753 = 1. + t3743;
  t3859 = Sin(var1[8]);
  t3574 = t2319*t2739;
  t3584 = -1.*t2287*t2537;
  t3698 = t3574 + t3584;
  t4044 = t817*t1879*t1299;
  t4047 = t1264*t1726;
  t4105 = t4044 + t4047;
  t4275 = Cos(var1[9]);
  t4303 = -1.*t4275;
  t4315 = 1. + t4303;
  t4445 = Sin(var1[9]);
  t4465 = t3731*t3698;
  t4467 = t4105*t3859;
  t4519 = t4465 + t4467;
  t4694 = t3731*t4105;
  t4707 = -1.*t3698*t3859;
  t4765 = t4694 + t4707;
  t4880 = Cos(var1[10]);
  t4889 = -1.*t4880;
  t4891 = 1. + t4889;
  t4896 = Sin(var1[10]);
  t4965 = -1.*t4445*t4519;
  t4978 = t4275*t4765;
  t4983 = t4965 + t4978;
  t5075 = t4275*t4519;
  t5091 = t4445*t4765;
  t5118 = t5075 + t5091;
  t1724 = 0.087*t1654;
  t1767 = 0.0222*t1726;
  t1780 = 0. + t1724 + t1767;
  t1901 = -0.0222*t1654;
  t1910 = 0.087*t1726;
  t2049 = 0. + t1901 + t1910;
  t5337 = t817*t123;
  t5350 = t329*t823*t931;
  t5357 = t5337 + t5350;
  t2525 = 0.157*t2464;
  t2596 = -0.3151*t2537;
  t2610 = 0. + t2525 + t2596;
  t2921 = -0.3151*t2464;
  t2943 = -0.157*t2537;
  t3000 = 0. + t2921 + t2943;
  t5419 = t1299*t5357;
  t5427 = -1.*t1879*t329*t1726;
  t5428 = t5419 + t5427;
  t5478 = t123*t329*t823;
  t5481 = -1.*t817*t931;
  t5492 = t5478 + t5481;
  t3755 = -0.3801*t3753;
  t3883 = -0.0222*t3859;
  t3966 = 0. + t3755 + t3883;
  t4111 = -0.0222*t3753;
  t4170 = 0.3801*t3859;
  t4183 = 0. + t4111 + t4170;
  t4319 = -0.8601*t4315;
  t4453 = -0.0222*t4445;
  t4461 = 0. + t4319 + t4453;
  t5588 = t2319*t5492;
  t5589 = -1.*t5428*t2537;
  t5608 = t5588 + t5589;
  t5626 = t1879*t1299*t329;
  t5637 = t5357*t1726;
  t5643 = t5626 + t5637;
  t4645 = -0.0222*t4315;
  t4647 = 0.8601*t4445;
  t4655 = 0. + t4645 + t4647;
  t4894 = -0.0211*t4891;
  t4927 = 1.3401*t4896;
  t4939 = 0. + t4894 + t4927;
  t5673 = t3731*t5608;
  t5684 = t5643*t3859;
  t5750 = t5673 + t5684;
  t5776 = t3731*t5643;
  t5799 = -1.*t5608*t3859;
  t5809 = t5776 + t5799;
  t5009 = -1.3401*t4891;
  t5027 = -0.0211*t4896;
  t5046 = 0. + t5009 + t5027;
  t5836 = -1.*t4445*t5750;
  t5856 = t4275*t5809;
  t5904 = t5836 + t5856;
  t5992 = t4275*t5750;
  t5993 = t4445*t5809;
  t6016 = t5992 + t5993;
  t6270 = t1879*t1299*t931;
  t6335 = t823*t1726;
  t6348 = t6270 + t6335;
  t6511 = t1879*t123*t2319;
  t6547 = -1.*t6348*t2537;
  t6556 = t6511 + t6547;
  t6589 = -1.*t1299*t823;
  t6594 = t1879*t931*t1726;
  t6599 = t6589 + t6594;
  t6602 = t3731*t6556;
  t6603 = t6599*t3859;
  t6614 = t6602 + t6603;
  t6635 = t3731*t6599;
  t6637 = -1.*t6556*t3859;
  t6638 = t6635 + t6637;
  t6643 = -1.*t4445*t6614;
  t6644 = t4275*t6638;
  t6645 = t6643 + t6644;
  t6664 = t4275*t6614;
  t6686 = t4445*t6638;
  t6689 = t6664 + t6686;

  p_output1(0)=0. + t1264*t1780 + t2287*t2610 + 0.242*(t2287*t2319 + t2537*t2739) + t2739*t3000 + t3698*t3966 + t4105*t4183 + t4461*t4519 + t4655*t4765 + t4939*t4983 + t5046*t5118 - 1.325152*(t4896*t4983 + t4880*t5118) + 0.043912*(t4880*t4983 - 1.*t4896*t5118) + t1879*t2049*t817 + var1(0);
  p_output1(1)=0. + t1879*t2049*t329 + t1780*t5357 + t2610*t5428 + t3000*t5492 + 0.242*(t2319*t5428 + t2537*t5492) + t3966*t5608 + t4183*t5643 + t4461*t5750 + t4655*t5809 + t4939*t5904 + t5046*t6016 - 1.325152*(t4896*t5904 + t4880*t6016) + 0.043912*(t4880*t5904 - 1.*t4896*t6016) + var1(1);
  p_output1(2)=0. + t123*t1879*t3000 + t2610*t6348 + 0.242*(t123*t1879*t2537 + t2319*t6348) + t3966*t6556 + t4183*t6599 + t4461*t6614 + t4655*t6638 + t4939*t6645 + t5046*t6689 - 1.325152*(t4896*t6645 + t4880*t6689) + 0.043912*(t4880*t6645 - 1.*t4896*t6689) - 1.*t2049*t823 + t1780*t1879*t931 + var1(2);
}


       
void p_LeftFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
