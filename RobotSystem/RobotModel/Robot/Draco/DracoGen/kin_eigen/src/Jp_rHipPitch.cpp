/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:25 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/Jp_rHipPitch.h"

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
  double t1343;
  double t154;
  double t191;
  double t212;
  double t735;
  double t2431;
  double t2224;
  double t2338;
  double t2460;
  double t1571;
  double t1643;
  double t1781;
  double t2044;
  double t121;
  double t2989;
  double t2993;
  double t2996;
  double t2353;
  double t2499;
  double t2507;
  double t3460;
  double t3546;
  double t3628;
  double t3652;
  double t3861;
  double t3938;
  double t4109;
  double t4444;
  double t4563;
  double t4591;
  double t4913;
  double t4918;
  double t4920;
  double t494;
  double t872;
  double t1136;
  double t1887;
  double t2098;
  double t2199;
  double t2644;
  double t2665;
  double t2676;
  double t3258;
  double t3333;
  double t3347;
  double t5063;
  double t5064;
  double t5065;
  double t3940;
  double t4346;
  double t4407;
  double t5048;
  double t5049;
  double t5055;
  double t5096;
  double t5098;
  double t5105;
  double t4883;
  double t4888;
  double t4904;
  double t5112;
  double t5120;
  double t5121;
  double t5168;
  double t5172;
  double t5186;
  double t5329;
  double t5336;
  double t5358;
  double t5434;
  double t5441;
  double t5445;
  double t5457;
  double t5460;
  double t5470;
  double t5621;
  double t5625;
  double t5639;
  double t5642;
  double t5652;
  double t5655;
  double t5677;
  double t5701;
  double t5705;
  double t5797;
  double t5800;
  double t5801;
  double t5824;
  double t5827;
  double t5828;
  double t5905;
  double t5924;
  double t5941;
  double t6037;
  double t6044;
  double t6051;
  double t6072;
  double t6082;
  double t6084;
  double t6144;
  double t6149;
  double t6167;
  double t6194;
  double t6200;
  double t6207;
  double t6289;
  double t6294;
  double t6296;
  double t6440;
  double t6450;
  double t6455;
  double t6403;
  double t6414;
  double t6415;
  double t6431;
  double t6432;
  double t6536;
  double t6538;
  double t6554;
  double t6572;
  double t6575;
  double t6562;
  double t6567;
  double t6568;
  double t6625;
  double t6629;
  double t6633;
  double t6617;
  double t6619;
  double t6620;
  double t6757;
  double t6761;
  double t6765;
  double t6729;
  double t6732;
  double t6741;
  double t6746;
  double t6751;
  double t6795;
  double t6802;
  double t6805;
  double t6839;
  double t6843;
  double t6846;
  double t5242;
  double t5250;
  double t5258;
  double t6882;
  double t6891;
  double t6896;
  double t6897;
  double t6899;
  double t6818;
  double t6820;
  double t6828;
  double t6923;
  double t6926;
  double t6929;
  double t6864;
  double t6871;
  double t6872;
  double t6964;
  double t6965;
  double t6967;
  t1343 = Sin(var1[3]);
  t154 = Cos(var1[11]);
  t191 = -1.*t154;
  t212 = 1. + t191;
  t735 = Sin(var1[11]);
  t2431 = Cos(var1[3]);
  t2224 = Cos(var1[5]);
  t2338 = Sin(var1[4]);
  t2460 = Sin(var1[5]);
  t1571 = Cos(var1[12]);
  t1643 = -1.*t1571;
  t1781 = 1. + t1643;
  t2044 = Sin(var1[12]);
  t121 = Cos(var1[4]);
  t2989 = -1.*t2431*t2224;
  t2993 = -1.*t1343*t2338*t2460;
  t2996 = t2989 + t2993;
  t2353 = -1.*t2224*t1343*t2338;
  t2499 = t2431*t2460;
  t2507 = t2353 + t2499;
  t3460 = t121*t735*t1343;
  t3546 = t154*t2996;
  t3628 = t3460 + t3546;
  t3652 = Cos(var1[13]);
  t3861 = -1.*t3652;
  t3938 = 1. + t3861;
  t4109 = Sin(var1[13]);
  t4444 = -1.*t154*t121*t1343;
  t4563 = t735*t2996;
  t4591 = t4444 + t4563;
  t4913 = t1571*t2507;
  t4918 = -1.*t2044*t3628;
  t4920 = t4913 + t4918;
  t494 = -0.0222*t212;
  t872 = -0.087*t735;
  t1136 = 0. + t494 + t872;
  t1887 = -0.3151*t1781;
  t2098 = 0.157*t2044;
  t2199 = 0. + t1887 + t2098;
  t2644 = -0.087*t212;
  t2665 = 0.0222*t735;
  t2676 = 0. + t2644 + t2665;
  t3258 = -0.157*t1781;
  t3333 = -0.3151*t2044;
  t3347 = 0. + t3258 + t3333;
  t5063 = -1.*t2224*t1343;
  t5064 = t2431*t2338*t2460;
  t5065 = t5063 + t5064;
  t3940 = -0.0222*t3938;
  t4346 = 0.3801*t4109;
  t4407 = 0. + t3940 + t4346;
  t5048 = t2431*t2224*t2338;
  t5049 = t1343*t2460;
  t5055 = t5048 + t5049;
  t5096 = -1.*t2431*t121*t735;
  t5098 = t154*t5065;
  t5105 = t5096 + t5098;
  t4883 = -0.3801*t3938;
  t4888 = -0.0222*t4109;
  t4904 = 0. + t4883 + t4888;
  t5112 = t154*t2431*t121;
  t5120 = t735*t5065;
  t5121 = t5112 + t5120;
  t5168 = t1571*t5055;
  t5172 = -1.*t2044*t5105;
  t5186 = t5168 + t5172;
  t5329 = t2431*t735*t2338;
  t5336 = t154*t2431*t121*t2460;
  t5358 = t5329 + t5336;
  t5434 = -1.*t154*t2431*t2338;
  t5441 = t2431*t121*t735*t2460;
  t5445 = t5434 + t5441;
  t5457 = t1571*t2431*t121*t2224;
  t5460 = -1.*t2044*t5358;
  t5470 = t5457 + t5460;
  t5621 = t735*t1343*t2338;
  t5625 = t154*t121*t1343*t2460;
  t5639 = t5621 + t5625;
  t5642 = -1.*t154*t1343*t2338;
  t5652 = t121*t735*t1343*t2460;
  t5655 = t5642 + t5652;
  t5677 = t1571*t121*t2224*t1343;
  t5701 = -1.*t2044*t5639;
  t5705 = t5677 + t5701;
  t5797 = t121*t735;
  t5800 = -1.*t154*t2338*t2460;
  t5801 = t5797 + t5800;
  t5824 = -1.*t154*t121;
  t5827 = -1.*t735*t2338*t2460;
  t5828 = t5824 + t5827;
  t5905 = -1.*t1571*t2224*t2338;
  t5924 = -1.*t2044*t5801;
  t5941 = t5905 + t5924;
  t6037 = t2224*t1343;
  t6044 = -1.*t2431*t2338*t2460;
  t6051 = t6037 + t6044;
  t6072 = -1.*t154*t2044*t5055;
  t6082 = t1571*t6051;
  t6084 = t6072 + t6082;
  t6144 = t2224*t1343*t2338;
  t6149 = -1.*t2431*t2460;
  t6167 = t6144 + t6149;
  t6194 = -1.*t154*t2044*t6167;
  t6200 = t1571*t2996;
  t6207 = t6194 + t6200;
  t6289 = -1.*t154*t121*t2224*t2044;
  t6294 = -1.*t1571*t121*t2460;
  t6296 = t6289 + t6294;
  t6440 = -1.*t154*t2431*t121;
  t6450 = -1.*t735*t5065;
  t6455 = t6440 + t6450;
  t6403 = -0.087*t154;
  t6414 = -0.0222*t735;
  t6415 = t6403 + t6414;
  t6431 = 0.0222*t154;
  t6432 = t6431 + t872;
  t6536 = t2431*t2224;
  t6538 = t1343*t2338*t2460;
  t6554 = t6536 + t6538;
  t6572 = -1.*t735*t6554;
  t6575 = t4444 + t6572;
  t6562 = -1.*t121*t735*t1343;
  t6567 = t154*t6554;
  t6568 = t6562 + t6567;
  t6625 = t154*t2338;
  t6629 = -1.*t121*t735*t2460;
  t6633 = t6625 + t6629;
  t6617 = t735*t2338;
  t6619 = t154*t121*t2460;
  t6620 = t6617 + t6619;
  t6757 = -1.*t2044*t5055;
  t6761 = -1.*t1571*t5105;
  t6765 = t6757 + t6761;
  t6729 = 0.157*t1571;
  t6732 = t6729 + t3333;
  t6741 = -0.3151*t1571;
  t6746 = -0.157*t2044;
  t6751 = t6741 + t6746;
  t6795 = -1.*t2044*t6167;
  t6802 = -1.*t1571*t6568;
  t6805 = t6795 + t6802;
  t6839 = -1.*t121*t2224*t2044;
  t6843 = -1.*t1571*t6620;
  t6846 = t6839 + t6843;
  t5242 = t3652*t5121;
  t5250 = -1.*t4109*t5186;
  t5258 = t5242 + t5250;
  t6882 = 0.3801*t3652;
  t6891 = t6882 + t4888;
  t6896 = -0.0222*t3652;
  t6897 = -0.3801*t4109;
  t6899 = t6896 + t6897;
  t6818 = t1571*t6167;
  t6820 = -1.*t2044*t6568;
  t6828 = t6818 + t6820;
  t6923 = t154*t121*t1343;
  t6926 = t735*t6554;
  t6929 = t6923 + t6926;
  t6864 = t1571*t121*t2224;
  t6871 = -1.*t2044*t6620;
  t6872 = t6864 + t6871;
  t6964 = -1.*t154*t2338;
  t6965 = t121*t735*t2460;
  t6967 = t6964 + t6965;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t1136*t121*t1343 + t2199*t2507 + t2676*t2996 + t3347*t3628 - 0.167*(t2044*t2507 + t1571*t3628) + t4407*t4591 + t4904*t4920 - 0.3801*(t4109*t4591 + t3652*t4920) - 0.0222*(t3652*t4591 - 1.*t4109*t4920);
  p_output1(10)=t1136*t121*t2431 + t2199*t5055 + t2676*t5065 + t3347*t5105 - 0.167*(t2044*t5055 + t1571*t5105) + t4407*t5121 + t4904*t5186 - 0.3801*(t4109*t5121 + t3652*t5186) - 0.0222*t5258;
  p_output1(11)=0;
  p_output1(12)=t121*t2199*t2224*t2431 - 1.*t1136*t2338*t2431 + t121*t2431*t2460*t2676 + t3347*t5358 - 0.167*(t121*t2044*t2224*t2431 + t1571*t5358) + t4407*t5445 + t4904*t5470 - 0.3801*(t4109*t5445 + t3652*t5470) - 0.0222*(t3652*t5445 - 1.*t4109*t5470);
  p_output1(13)=t121*t1343*t2199*t2224 - 1.*t1136*t1343*t2338 + t121*t1343*t2460*t2676 + t3347*t5639 - 0.167*(t121*t1343*t2044*t2224 + t1571*t5639) + t4407*t5655 + t4904*t5705 - 0.3801*(t4109*t5655 + t3652*t5705) - 0.0222*(t3652*t5655 - 1.*t4109*t5705);
  p_output1(14)=-1.*t1136*t121 - 1.*t2199*t2224*t2338 - 1.*t2338*t2460*t2676 + t3347*t5801 - 0.167*(-1.*t2044*t2224*t2338 + t1571*t5801) + t4407*t5828 + t4904*t5941 - 0.3801*(t4109*t5828 + t3652*t5941) - 0.0222*(t3652*t5828 - 1.*t4109*t5941);
  p_output1(15)=t2676*t5055 + t154*t3347*t5055 + t2199*t6051 - 0.167*(t154*t1571*t5055 + t2044*t6051) + t4904*t6084 + t4407*t5055*t735 - 0.0222*(-1.*t4109*t6084 + t3652*t5055*t735) - 0.3801*(t3652*t6084 + t4109*t5055*t735);
  p_output1(16)=t2199*t2996 + t2676*t6167 + t154*t3347*t6167 - 0.167*(t2044*t2996 + t154*t1571*t6167) + t4904*t6207 + t4407*t6167*t735 - 0.0222*(-1.*t4109*t6207 + t3652*t6167*t735) - 0.3801*(t3652*t6207 + t4109*t6167*t735);
  p_output1(17)=-1.*t121*t2199*t2460 - 0.167*(t121*t154*t1571*t2224 - 1.*t121*t2044*t2460) + t121*t2224*t2676 + t121*t154*t2224*t3347 + t4904*t6296 + t121*t2224*t4407*t735 - 0.0222*(-1.*t4109*t6296 + t121*t2224*t3652*t735) - 0.3801*(t3652*t6296 + t121*t2224*t4109*t735);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t4407*t5105 + t121*t2431*t6415 + t5065*t6432 - 0.167*t1571*t6455 + t3347*t6455 - 1.*t2044*t4904*t6455 - 0.3801*(t4109*t5105 - 1.*t2044*t3652*t6455) - 0.0222*(t3652*t5105 + t2044*t4109*t6455);
  p_output1(34)=t121*t1343*t6415 + t6432*t6554 + t4407*t6568 - 0.167*t1571*t6575 + t3347*t6575 - 1.*t2044*t4904*t6575 - 0.3801*(t4109*t6568 - 1.*t2044*t3652*t6575) - 0.0222*(t3652*t6568 + t2044*t4109*t6575);
  p_output1(35)=-1.*t2338*t6415 + t121*t2460*t6432 + t4407*t6620 - 0.167*t1571*t6633 + t3347*t6633 - 1.*t2044*t4904*t6633 - 0.3801*(t4109*t6620 - 1.*t2044*t3652*t6633) - 0.0222*(t3652*t6620 + t2044*t4109*t6633);
  p_output1(36)=-0.167*t5186 + t5055*t6732 + t5105*t6751 - 0.3801*t3652*t6765 + 0.0222*t4109*t6765 + t4904*t6765;
  p_output1(37)=t6167*t6732 + t6568*t6751 - 0.3801*t3652*t6805 + 0.0222*t4109*t6805 + t4904*t6805 - 0.167*t6828;
  p_output1(38)=t121*t2224*t6732 + t6620*t6751 - 0.3801*t3652*t6846 + 0.0222*t4109*t6846 + t4904*t6846 - 0.167*t6872;
  p_output1(39)=-0.0222*(-1.*t4109*t5121 - 1.*t3652*t5186) - 0.3801*t5258 + t5121*t6891 + t5186*t6899;
  p_output1(40)=t6828*t6899 + t6891*t6929 - 0.3801*(-1.*t4109*t6828 + t3652*t6929) - 0.0222*(-1.*t3652*t6828 - 1.*t4109*t6929);
  p_output1(41)=t6872*t6899 + t6891*t6967 - 0.3801*(-1.*t4109*t6872 + t3652*t6967) - 0.0222*(-1.*t3652*t6872 - 1.*t4109*t6967);
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
