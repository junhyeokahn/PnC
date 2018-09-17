/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:34 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_RightFootBottom.h"

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
  double t241;
  double t307;
  double t408;
  double t396;
  double t612;
  double t206;
  double t723;
  double t763;
  double t767;
  double t806;
  double t917;
  double t986;
  double t1064;
  double t397;
  double t624;
  double t639;
  double t779;
  double t1066;
  double t1072;
  double t2252;
  double t2116;
  double t2197;
  double t2225;
  double t2113;
  double t2278;
  double t2285;
  double t2306;
  double t2370;
  double t2250;
  double t2311;
  double t2335;
  double t2107;
  double t2418;
  double t2422;
  double t2424;
  double t2519;
  double t2337;
  double t2444;
  double t2450;
  double t2082;
  double t2555;
  double t2587;
  double t2631;
  double t1369;
  double t1372;
  double t1375;
  double t1146;
  double t1149;
  double t1241;
  double t1338;
  double t1393;
  double t1409;
  double t2771;
  double t2810;
  double t2890;
  double t3033;
  double t3042;
  double t3059;
  double t2924;
  double t3092;
  double t3105;
  double t3219;
  double t3308;
  double t3327;
  double t3210;
  double t3341;
  double t3351;
  double t3386;
  double t3400;
  double t3404;
  double t1485;
  double t1534;
  double t1733;
  double t3452;
  double t3454;
  double t3504;
  double t3515;
  double t3531;
  double t3533;
  double t3508;
  double t3538;
  double t3550;
  double t3573;
  double t3580;
  double t3591;
  double t3557;
  double t3599;
  double t3609;
  double t3668;
  double t3671;
  double t3673;
  double t4495;
  double t4505;
  double t4608;
  double t4615;
  double t675;
  double t1082;
  double t1122;
  double t4870;
  double t4871;
  double t4996;
  double t5007;
  double t5088;
  double t5103;
  double t4014;
  double t4023;
  double t4025;
  double t2641;
  double t2653;
  double t2721;
  double t4555;
  double t4579;
  double t4588;
  double t4617;
  double t4618;
  double t4624;
  double t4693;
  double t4701;
  double t4794;
  double t4807;
  double t4812;
  double t4855;
  double t4877;
  double t4902;
  double t4917;
  double t1277;
  double t1446;
  double t1457;
  double t4952;
  double t4954;
  double t4958;
  double t5011;
  double t5015;
  double t5019;
  double t5069;
  double t5074;
  double t5078;
  double t5104;
  double t5107;
  double t5108;
  double t5124;
  double t5149;
  double t5173;
  double t4249;
  double t4258;
  double t4298;
  double t3421;
  double t3433;
  double t3438;
  double t1479;
  double t1929;
  double t2048;
  double t4342;
  double t4368;
  double t4370;
  double t3727;
  double t3745;
  double t3805;
  t241 = Cos(var1[3]);
  t307 = Cos(var1[5]);
  t408 = Sin(var1[3]);
  t396 = Sin(var1[4]);
  t612 = Sin(var1[5]);
  t206 = Sin(var1[12]);
  t723 = Cos(var1[12]);
  t763 = Cos(var1[4]);
  t767 = Sin(var1[11]);
  t806 = Cos(var1[11]);
  t917 = -1.*t307*t408;
  t986 = t241*t396*t612;
  t1064 = t917 + t986;
  t397 = t241*t307*t396;
  t624 = t408*t612;
  t639 = t397 + t624;
  t779 = -1.*t241*t763*t767;
  t1066 = t806*t1064;
  t1072 = t779 + t1066;
  t2252 = Cos(var1[13]);
  t2116 = t806*t241*t763;
  t2197 = t767*t1064;
  t2225 = t2116 + t2197;
  t2113 = Sin(var1[13]);
  t2278 = t723*t639;
  t2285 = -1.*t206*t1072;
  t2306 = t2278 + t2285;
  t2370 = Cos(var1[14]);
  t2250 = t2113*t2225;
  t2311 = t2252*t2306;
  t2335 = t2250 + t2311;
  t2107 = Sin(var1[14]);
  t2418 = t2252*t2225;
  t2422 = -1.*t2113*t2306;
  t2424 = t2418 + t2422;
  t2519 = Cos(var1[15]);
  t2337 = -1.*t2107*t2335;
  t2444 = t2370*t2424;
  t2450 = t2337 + t2444;
  t2082 = Sin(var1[15]);
  t2555 = t2370*t2335;
  t2587 = t2107*t2424;
  t2631 = t2555 + t2587;
  t1369 = t241*t307;
  t1372 = t408*t396*t612;
  t1375 = t1369 + t1372;
  t1146 = t307*t408*t396;
  t1149 = -1.*t241*t612;
  t1241 = t1146 + t1149;
  t1338 = -1.*t763*t767*t408;
  t1393 = t806*t1375;
  t1409 = t1338 + t1393;
  t2771 = t806*t763*t408;
  t2810 = t767*t1375;
  t2890 = t2771 + t2810;
  t3033 = t723*t1241;
  t3042 = -1.*t206*t1409;
  t3059 = t3033 + t3042;
  t2924 = t2113*t2890;
  t3092 = t2252*t3059;
  t3105 = t2924 + t3092;
  t3219 = t2252*t2890;
  t3308 = -1.*t2113*t3059;
  t3327 = t3219 + t3308;
  t3210 = -1.*t2107*t3105;
  t3341 = t2370*t3327;
  t3351 = t3210 + t3341;
  t3386 = t2370*t3105;
  t3400 = t2107*t3327;
  t3404 = t3386 + t3400;
  t1485 = t767*t396;
  t1534 = t806*t763*t612;
  t1733 = t1485 + t1534;
  t3452 = -1.*t806*t396;
  t3454 = t763*t767*t612;
  t3504 = t3452 + t3454;
  t3515 = t723*t763*t307;
  t3531 = -1.*t206*t1733;
  t3533 = t3515 + t3531;
  t3508 = t2113*t3504;
  t3538 = t2252*t3533;
  t3550 = t3508 + t3538;
  t3573 = t2252*t3504;
  t3580 = -1.*t2113*t3533;
  t3591 = t3573 + t3580;
  t3557 = -1.*t2107*t3550;
  t3599 = t2370*t3591;
  t3609 = t3557 + t3599;
  t3668 = t2370*t3550;
  t3671 = t2107*t3591;
  t3673 = t3668 + t3671;
  t4495 = -1.*t806;
  t4505 = 1. + t4495;
  t4608 = -1.*t723;
  t4615 = 1. + t4608;
  t675 = t206*t639;
  t1082 = t723*t1072;
  t1122 = t675 + t1082;
  t4870 = -1.*t2252;
  t4871 = 1. + t4870;
  t4996 = -1.*t2370;
  t5007 = 1. + t4996;
  t5088 = -1.*t2519;
  t5103 = 1. + t5088;
  t4014 = t2082*t2450;
  t4023 = t2519*t2631;
  t4025 = t4014 + t4023;
  t2641 = t2519*t2450;
  t2653 = -1.*t2082*t2631;
  t2721 = t2641 + t2653;
  t4555 = -0.0222*t4505;
  t4579 = -0.087*t767;
  t4588 = 0. + t4555 + t4579;
  t4617 = -0.3151*t4615;
  t4618 = 0.157*t206;
  t4624 = 0. + t4617 + t4618;
  t4693 = -0.087*t4505;
  t4701 = 0.0222*t767;
  t4794 = 0. + t4693 + t4701;
  t4807 = -0.157*t4615;
  t4812 = -0.3151*t206;
  t4855 = 0. + t4807 + t4812;
  t4877 = -0.0222*t4871;
  t4902 = 0.3801*t2113;
  t4917 = 0. + t4877 + t4902;
  t1277 = t206*t1241;
  t1446 = t723*t1409;
  t1457 = t1277 + t1446;
  t4952 = -0.3801*t4871;
  t4954 = -0.0222*t2113;
  t4958 = 0. + t4952 + t4954;
  t5011 = -0.8601*t5007;
  t5015 = -0.0222*t2107;
  t5019 = 0. + t5011 + t5015;
  t5069 = -0.0222*t5007;
  t5074 = 0.8601*t2107;
  t5078 = 0. + t5069 + t5074;
  t5104 = -0.0211*t5103;
  t5107 = 1.3401*t2082;
  t5108 = 0. + t5104 + t5107;
  t5124 = -1.3401*t5103;
  t5149 = -0.0211*t2082;
  t5173 = 0. + t5124 + t5149;
  t4249 = t2082*t3351;
  t4258 = t2519*t3404;
  t4298 = t4249 + t4258;
  t3421 = t2519*t3351;
  t3433 = -1.*t2082*t3404;
  t3438 = t3421 + t3433;
  t1479 = t763*t307*t206;
  t1929 = t723*t1733;
  t2048 = t1479 + t1929;
  t4342 = t2082*t3609;
  t4368 = t2519*t3673;
  t4370 = t4342 + t4368;
  t3727 = t2519*t3609;
  t3745 = -1.*t2082*t3673;
  t3805 = t3727 + t3745;

  p_output1(0)=t1122;
  p_output1(1)=t1457;
  p_output1(2)=t2048;
  p_output1(3)=0.;
  p_output1(4)=-1.*t2082*t2450 - 1.*t2519*t2631 - 0.000796*t2721;
  p_output1(5)=-1.*t2082*t3351 - 1.*t2519*t3404 - 0.000796*t3438;
  p_output1(6)=-1.*t2082*t3609 - 1.*t2519*t3673 - 0.000796*t3805;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2450*t2519 + t2082*t2631 + 0.000796*t4025;
  p_output1(9)=-1.*t2519*t3351 + t2082*t3404 + 0.000796*t4298;
  p_output1(10)=-1.*t2519*t3609 + t2082*t3673 + 0.000796*t4370;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.16705*t1122 + 0.043912*t2721 - 1.325152*t4025 + t1064*t4794 + t1072*t4855 + t2225*t4917 + t2306*t4958 + t2335*t5019 + t2424*t5078 + t2450*t5108 + t2631*t5173 + t4624*t639 + t241*t4588*t763 + var1(0);
  p_output1(13)=0. - 0.16705*t1457 + 0.043912*t3438 - 1.325152*t4298 + t1241*t4624 + t1375*t4794 + t1409*t4855 + t2890*t4917 + t3059*t4958 + t3105*t5019 + t3327*t5078 + t3351*t5108 + t3404*t5173 + t408*t4588*t763 + var1(1);
  p_output1(14)=0. - 0.16705*t2048 + 0.043912*t3805 - 1.325152*t4370 - 1.*t396*t4588 + t1733*t4855 + t3504*t4917 + t3533*t4958 + t3550*t5019 + t3591*t5078 + t3609*t5108 + t3673*t5173 + t307*t4624*t763 + t4794*t612*t763 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootBottom(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
