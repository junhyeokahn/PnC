/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:39 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lKnee.h"

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
  double t306;
  double t509;
  double t115;
  double t346;
  double t529;
  double t617;
  double t565;
  double t595;
  double t597;
  double t613;
  double t618;
  double t91;
  double t946;
  double t1139;
  double t1202;
  double t95;
  double t468;
  double t537;
  double t540;
  double t559;
  double t614;
  double t622;
  double t629;
  double t631;
  double t726;
  double t777;
  double t1205;
  double t79;
  double t2225;
  double t2242;
  double t2252;
  double t1726;
  double t2510;
  double t2569;
  double t2581;
  double t1973;
  double t2116;
  double t2130;
  double t2183;
  double t2256;
  double t2285;
  double t2308;
  double t2361;
  double t2455;
  double t3384;
  double t3399;
  double t3494;
  double t2847;
  double t2874;
  double t2949;
  double t3119;
  double t3287;
  double t3312;
  double t866;
  double t1293;
  double t1470;
  double t1765;
  double t1789;
  double t1815;
  double t2487;
  double t2624;
  double t2650;
  double t2673;
  double t2773;
  double t2783;
  double t3352;
  double t3517;
  double t3524;
  double t3695;
  double t3706;
  double t3732;
  double t4567;
  double t4585;
  double t4721;
  double t4746;
  double t3799;
  double t3868;
  double t3909;
  double t4889;
  double t4896;
  double t5368;
  double t5419;
  double t1524;
  double t1820;
  double t1834;
  double t4135;
  double t4160;
  double t4249;
  double t4596;
  double t4597;
  double t4605;
  double t4631;
  double t4660;
  double t4669;
  double t4748;
  double t4752;
  double t4759;
  double t4785;
  double t4791;
  double t4805;
  double t3984;
  double t3988;
  double t4104;
  double t4944;
  double t5022;
  double t5049;
  double t5246;
  double t5287;
  double t5340;
  double t5436;
  double t5437;
  double t5457;
  double t5548;
  double t5559;
  double t5560;
  double t2665;
  double t2829;
  double t2830;
  double t4337;
  double t4480;
  double t4498;
  double t4109;
  double t4113;
  double t4115;
  double t3635;
  double t3784;
  double t3795;
  double t4525;
  double t4559;
  double t4562;
  t306 = Cos(var1[5]);
  t509 = Sin(var1[3]);
  t115 = Cos(var1[3]);
  t346 = Sin(var1[4]);
  t529 = Sin(var1[5]);
  t617 = Cos(var1[4]);
  t565 = Cos(var1[6]);
  t595 = -1.*t306*t509;
  t597 = t115*t346*t529;
  t613 = t595 + t597;
  t618 = Sin(var1[6]);
  t91 = Cos(var1[8]);
  t946 = t115*t617*t565;
  t1139 = t613*t618;
  t1202 = t946 + t1139;
  t95 = Cos(var1[7]);
  t468 = t115*t306*t346;
  t537 = t509*t529;
  t540 = t468 + t537;
  t559 = t95*t540;
  t614 = t565*t613;
  t622 = -1.*t115*t617*t618;
  t629 = t614 + t622;
  t631 = Sin(var1[7]);
  t726 = -1.*t629*t631;
  t777 = t559 + t726;
  t1205 = Sin(var1[8]);
  t79 = Sin(var1[9]);
  t2225 = t115*t306;
  t2242 = t509*t346*t529;
  t2252 = t2225 + t2242;
  t1726 = Cos(var1[9]);
  t2510 = t617*t565*t509;
  t2569 = t2252*t618;
  t2581 = t2510 + t2569;
  t1973 = t306*t509*t346;
  t2116 = -1.*t115*t529;
  t2130 = t1973 + t2116;
  t2183 = t95*t2130;
  t2256 = t565*t2252;
  t2285 = -1.*t617*t509*t618;
  t2308 = t2256 + t2285;
  t2361 = -1.*t2308*t631;
  t2455 = t2183 + t2361;
  t3384 = -1.*t565*t346;
  t3399 = t617*t529*t618;
  t3494 = t3384 + t3399;
  t2847 = t617*t306*t95;
  t2874 = t617*t565*t529;
  t2949 = t346*t618;
  t3119 = t2874 + t2949;
  t3287 = -1.*t3119*t631;
  t3312 = t2847 + t3287;
  t866 = t91*t777;
  t1293 = t1202*t1205;
  t1470 = t866 + t1293;
  t1765 = t91*t1202;
  t1789 = -1.*t777*t1205;
  t1815 = t1765 + t1789;
  t2487 = t91*t2455;
  t2624 = t2581*t1205;
  t2650 = t2487 + t2624;
  t2673 = t91*t2581;
  t2773 = -1.*t2455*t1205;
  t2783 = t2673 + t2773;
  t3352 = t91*t3312;
  t3517 = t3494*t1205;
  t3524 = t3352 + t3517;
  t3695 = t91*t3494;
  t3706 = -1.*t3312*t1205;
  t3732 = t3695 + t3706;
  t4567 = -1.*t565;
  t4585 = 1. + t4567;
  t4721 = -1.*t95;
  t4746 = 1. + t4721;
  t3799 = t95*t629;
  t3868 = t540*t631;
  t3909 = t3799 + t3868;
  t4889 = -1.*t91;
  t4896 = 1. + t4889;
  t5368 = -1.*t1726;
  t5419 = 1. + t5368;
  t1524 = -1.*t79*t1470;
  t1820 = t1726*t1815;
  t1834 = t1524 + t1820;
  t4135 = t1726*t1470;
  t4160 = t79*t1815;
  t4249 = t4135 + t4160;
  t4596 = 0.087004*t4585;
  t4597 = 0.022225*t618;
  t4605 = 0. + t4596 + t4597;
  t4631 = -0.022225*t4585;
  t4660 = 0.087004*t618;
  t4669 = 0. + t4631 + t4660;
  t4748 = 0.157004*t4746;
  t4752 = -0.31508*t631;
  t4759 = 0. + t4748 + t4752;
  t4785 = -0.31508*t4746;
  t4791 = -0.157004*t631;
  t4805 = 0. + t4785 + t4791;
  t3984 = t95*t2308;
  t3988 = t2130*t631;
  t4104 = t3984 + t3988;
  t4944 = -0.38008*t4896;
  t5022 = -0.022225*t1205;
  t5049 = 0. + t4944 + t5022;
  t5246 = -0.022225*t4896;
  t5287 = 0.38008*t1205;
  t5340 = 0. + t5246 + t5287;
  t5436 = -0.86008*t5419;
  t5437 = -0.022225*t79;
  t5457 = 0. + t5436 + t5437;
  t5548 = -0.022225*t5419;
  t5559 = 0.86008*t79;
  t5560 = 0. + t5548 + t5559;
  t2665 = -1.*t79*t2650;
  t2829 = t1726*t2783;
  t2830 = t2665 + t2829;
  t4337 = t1726*t2650;
  t4480 = t79*t2783;
  t4498 = t4337 + t4480;
  t4109 = t95*t3119;
  t4113 = t617*t306*t631;
  t4115 = t4109 + t4113;
  t3635 = -1.*t79*t3524;
  t3784 = t1726*t3732;
  t3795 = t3635 + t3784;
  t4525 = t1726*t3524;
  t4559 = t79*t3732;
  t4562 = t4525 + t4559;

  p_output1(0)=t1834;
  p_output1(1)=t2830;
  p_output1(2)=t3795;
  p_output1(3)=0.;
  p_output1(4)=t3909;
  p_output1(5)=t4104;
  p_output1(6)=t4115;
  p_output1(7)=0.;
  p_output1(8)=t4249;
  p_output1(9)=t4498;
  p_output1(10)=t4562;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.022225*t1834 + 0.150254*t3909 - 0.86008*t4249 + t1202*t5340 + t4805*t540 + t1470*t5457 + t1815*t5560 + t4605*t613 + t115*t4669*t617 + t4759*t629 + t5049*t777 + var1(0);
  p_output1(13)=0. - 0.022225*t2830 + 0.150254*t4104 - 0.86008*t4498 + t2252*t4605 + t2308*t4759 + t2130*t4805 + t2455*t5049 + t2581*t5340 + t2650*t5457 + t2783*t5560 + t4669*t509*t617 + var1(1);
  p_output1(14)=0. - 0.022225*t3795 + 0.150254*t4115 - 0.86008*t4562 - 1.*t346*t4669 + t3119*t4759 + t3312*t5049 + t3494*t5340 + t3524*t5457 + t3732*t5560 + t306*t4805*t617 + t4605*t529*t617 + var1(2);
  p_output1(15)=1.;
}


       
void H_lKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
