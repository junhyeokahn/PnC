/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:16 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/Jp_lHipRoll.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t1309;
  double t1636;
  double t1645;
  double t1662;
  double t1750;
  double t2178;
  double t1021;
  double t1028;
  double t1113;
  double t1311;
  double t1416;
  double t1437;
  double t1467;
  double t3102;
  double t3348;
  double t3384;
  double t3676;
  double t2584;
  double t2633;
  double t2910;
  double t4314;
  double t4321;
  double t4593;
  double t1745;
  double t1834;
  double t1943;
  double t2238;
  double t2340;
  double t2351;
  double t4805;
  double t4820;
  double t4831;
  double t3668;
  double t3877;
  double t3883;
  double t4601;
  double t4631;
  double t4673;
  double t4959;
  double t4963;
  double t4967;
  double t4974;
  double t4975;
  double t4980;
  double t5180;
  double t5181;
  double t5189;
  double t5304;
  double t5316;
  double t5317;
  double t5375;
  double t5377;
  double t5378;
  double t5444;
  double t5449;
  double t5459;
  double t5509;
  double t5510;
  double t5516;
  double t5706;
  double t5708;
  double t5709;
  double t5625;
  double t5627;
  double t5646;
  double t5666;
  double t5682;
  double t5744;
  double t5748;
  double t5759;
  double t2449;
  double t5798;
  double t5803;
  double t5869;
  double t5870;
  double t5872;
  double t5043;
  double t5081;
  double t5108;
  double t5935;
  double t5937;
  double t5765;
  double t5779;
  double t5784;
  double t5939;
  double t5941;
  double t5942;
  double t5848;
  double t5858;
  double t5863;
  t1309 = Sin(var1[3]);
  t1636 = Cos(var1[6]);
  t1645 = -1.*t1636;
  t1662 = 1. + t1645;
  t1750 = Sin(var1[6]);
  t2178 = Cos(var1[4]);
  t1021 = Cos(var1[3]);
  t1028 = Cos(var1[5]);
  t1113 = -1.*t1021*t1028;
  t1311 = Sin(var1[4]);
  t1416 = Sin(var1[5]);
  t1437 = -1.*t1309*t1311*t1416;
  t1467 = t1113 + t1437;
  t3102 = Cos(var1[7]);
  t3348 = -1.*t3102;
  t3384 = 1. + t3348;
  t3676 = Sin(var1[7]);
  t2584 = t1636*t1467;
  t2633 = t2178*t1309*t1750;
  t2910 = t2584 + t2633;
  t4314 = -1.*t1028*t1309*t1311;
  t4321 = t1021*t1416;
  t4593 = t4314 + t4321;
  t1745 = 0.087*t1662;
  t1834 = 0.0222*t1750;
  t1943 = 0. + t1745 + t1834;
  t2238 = -0.0222*t1662;
  t2340 = 0.087*t1750;
  t2351 = 0. + t2238 + t2340;
  t4805 = -1.*t1028*t1309;
  t4820 = t1021*t1311*t1416;
  t4831 = t4805 + t4820;
  t3668 = 0.157*t3384;
  t3877 = -0.3151*t3676;
  t3883 = 0. + t3668 + t3877;
  t4601 = -0.3151*t3384;
  t4631 = -0.157*t3676;
  t4673 = 0. + t4601 + t4631;
  t4959 = t1636*t4831;
  t4963 = -1.*t1021*t2178*t1750;
  t4967 = t4959 + t4963;
  t4974 = t1021*t1028*t1311;
  t4975 = t1309*t1416;
  t4980 = t4974 + t4975;
  t5180 = t1021*t2178*t1636*t1416;
  t5181 = t1021*t1311*t1750;
  t5189 = t5180 + t5181;
  t5304 = t2178*t1636*t1309*t1416;
  t5316 = t1309*t1311*t1750;
  t5317 = t5304 + t5316;
  t5375 = -1.*t1636*t1311*t1416;
  t5377 = t2178*t1750;
  t5378 = t5375 + t5377;
  t5444 = t1028*t1309;
  t5449 = -1.*t1021*t1311*t1416;
  t5459 = t5444 + t5449;
  t5509 = t1028*t1309*t1311;
  t5510 = -1.*t1021*t1416;
  t5516 = t5509 + t5510;
  t5706 = -1.*t1021*t2178*t1636;
  t5708 = -1.*t4831*t1750;
  t5709 = t5706 + t5708;
  t5625 = 0.087*t1636;
  t5627 = -0.0222*t1750;
  t5646 = t5625 + t5627;
  t5666 = 0.0222*t1636;
  t5682 = t5666 + t2340;
  t5744 = t1021*t1028;
  t5748 = t1309*t1311*t1416;
  t5759 = t5744 + t5748;
  t2449 = -1.*t2178*t1636*t1309;
  t5798 = -1.*t5759*t1750;
  t5803 = t2449 + t5798;
  t5869 = t1636*t1311;
  t5870 = -1.*t2178*t1416*t1750;
  t5872 = t5869 + t5870;
  t5043 = t3102*t4980;
  t5081 = -1.*t4967*t3676;
  t5108 = t5043 + t5081;
  t5935 = -0.157*t3102;
  t5937 = t5935 + t3877;
  t5765 = t1636*t5759;
  t5779 = -1.*t2178*t1309*t1750;
  t5784 = t5765 + t5779;
  t5939 = -0.3151*t3102;
  t5941 = 0.157*t3676;
  t5942 = t5939 + t5941;
  t5848 = t2178*t1636*t1416;
  t5858 = t1311*t1750;
  t5863 = t5848 + t5858;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1467*t1943 - 1.*t1309*t2178*t2351 - 0.0222*(t1467*t1750 + t2449) + t2910*t3883 - 0.3151*(-1.*t2910*t3676 + t3102*t4593) + 0.157*(t2910*t3102 + t3676*t4593) + t4593*t4673;
  p_output1(10)=t1021*t2178*t2351 + t1943*t4831 - 0.0222*(t1021*t1636*t2178 + t1750*t4831) + t3883*t4967 + t4673*t4980 + 0.157*(t3102*t4967 + t3676*t4980) - 0.3151*t5108;
  p_output1(11)=0;
  p_output1(12)=t1021*t1416*t1943*t2178 - 0.0222*(-1.*t1021*t1311*t1636 + t1021*t1416*t1750*t2178) - 1.*t1021*t1311*t2351 + t1021*t1028*t2178*t4673 + t3883*t5189 + 0.157*(t1021*t1028*t2178*t3676 + t3102*t5189) - 0.3151*(t1021*t1028*t2178*t3102 - 1.*t3676*t5189);
  p_output1(13)=t1309*t1416*t1943*t2178 - 0.0222*(-1.*t1309*t1311*t1636 + t1309*t1416*t1750*t2178) - 1.*t1309*t1311*t2351 + t1028*t1309*t2178*t4673 + t3883*t5317 + 0.157*(t1028*t1309*t2178*t3676 + t3102*t5317) - 0.3151*(t1028*t1309*t2178*t3102 - 1.*t3676*t5317);
  p_output1(14)=-1.*t1311*t1416*t1943 - 0.0222*(-1.*t1311*t1416*t1750 - 1.*t1636*t2178) - 1.*t2178*t2351 - 1.*t1028*t1311*t4673 + t3883*t5378 + 0.157*(-1.*t1028*t1311*t3676 + t3102*t5378) - 0.3151*(-1.*t1028*t1311*t3102 - 1.*t3676*t5378);
  p_output1(15)=-0.0222*t1750*t4980 + t1943*t4980 + t1636*t3883*t4980 + t4673*t5459 - 0.3151*(-1.*t1636*t3676*t4980 + t3102*t5459) + 0.157*(t1636*t3102*t4980 + t3676*t5459);
  p_output1(16)=t1467*t4673 - 0.0222*t1750*t5516 + t1943*t5516 + t1636*t3883*t5516 + 0.157*(t1467*t3676 + t1636*t3102*t5516) - 0.3151*(t1467*t3102 - 1.*t1636*t3676*t5516);
  p_output1(17)=-0.0222*t1028*t1750*t2178 + t1028*t1943*t2178 + 0.157*(t1028*t1636*t2178*t3102 - 1.*t1416*t2178*t3676) - 0.3151*(-1.*t1416*t2178*t3102 - 1.*t1028*t1636*t2178*t3676) + t1028*t1636*t2178*t3883 - 1.*t1416*t2178*t4673;
  p_output1(18)=-0.0222*t4967 + t1021*t2178*t5646 + t4831*t5682 + 0.157*t3102*t5709 + 0.3151*t3676*t5709 + t3883*t5709;
  p_output1(19)=t1309*t2178*t5646 + t5682*t5759 - 0.0222*t5784 + 0.157*t3102*t5803 + 0.3151*t3676*t5803 + t3883*t5803;
  p_output1(20)=-1.*t1311*t5646 + t1416*t2178*t5682 - 0.0222*t5863 + 0.157*t3102*t5872 + 0.3151*t3676*t5872 + t3883*t5872;
  p_output1(21)=-0.3151*(-1.*t3102*t4967 - 1.*t3676*t4980) + 0.157*t5108 + t4980*t5937 + t4967*t5942;
  p_output1(22)=-0.3151*(-1.*t3676*t5516 - 1.*t3102*t5784) + 0.157*(t3102*t5516 - 1.*t3676*t5784) + t5516*t5937 + t5784*t5942;
  p_output1(23)=-0.3151*(-1.*t1028*t2178*t3676 - 1.*t3102*t5863) + 0.157*(t1028*t2178*t3102 - 1.*t3676*t5863) + t1028*t2178*t5937 + t5863*t5942;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_lHipRoll(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
