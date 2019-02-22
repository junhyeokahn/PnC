/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:27 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_rAnkle.h"

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
  double t1850;
  double t2322;
  double t2378;
  double t2436;
  double t2790;
  double t4854;
  double t5743;
  double t5277;
  double t5777;
  double t3249;
  double t3275;
  double t3322;
  double t4502;
  double t1853;
  double t5936;
  double t5940;
  double t5945;
  double t5727;
  double t5810;
  double t5817;
  double t6119;
  double t6121;
  double t6141;
  double t6171;
  double t6204;
  double t6216;
  double t6223;
  double t6236;
  double t6239;
  double t6244;
  double t6371;
  double t6374;
  double t6392;
  double t6422;
  double t6423;
  double t6432;
  double t6457;
  double t6536;
  double t6556;
  double t6574;
  double t6608;
  double t6610;
  double t6611;
  double t6621;
  double t6628;
  double t6647;
  double t6653;
  double t6663;
  double t6668;
  double t6671;
  double t6715;
  double t6741;
  double t6746;
  double t2493;
  double t2963;
  double t3101;
  double t4498;
  double t4524;
  double t4803;
  double t5895;
  double t5917;
  double t5921;
  double t6024;
  double t6103;
  double t6116;
  double t6857;
  double t6859;
  double t6875;
  double t6217;
  double t6224;
  double t6227;
  double t6818;
  double t6830;
  double t6831;
  double t6897;
  double t6902;
  double t6909;
  double t6314;
  double t6336;
  double t6365;
  double t6444;
  double t6464;
  double t6470;
  double t6917;
  double t6923;
  double t6927;
  double t6954;
  double t6961;
  double t6965;
  double t6589;
  double t6591;
  double t6598;
  double t6651;
  double t6654;
  double t6655;
  double t6974;
  double t6981;
  double t6984;
  double t6995;
  double t6997;
  double t7007;
  double t6685;
  double t6691;
  double t6714;
  double t7022;
  double t7023;
  double t7024;
  double t7031;
  double t7033;
  double t7036;
  double t7111;
  double t7115;
  double t7116;
  double t7122;
  double t7145;
  double t7154;
  double t7172;
  double t7175;
  double t7184;
  double t7189;
  double t7195;
  double t7203;
  double t7211;
  double t7219;
  double t7228;
  double t7231;
  double t7239;
  double t7245;
  double t7257;
  double t7258;
  double t7261;
  t1850 = Cos(var1[3]);
  t2322 = Cos(var1[11]);
  t2378 = -1.*t2322;
  t2436 = 1. + t2378;
  t2790 = Sin(var1[11]);
  t4854 = Cos(var1[5]);
  t5743 = Sin(var1[3]);
  t5277 = Sin(var1[4]);
  t5777 = Sin(var1[5]);
  t3249 = Cos(var1[12]);
  t3275 = -1.*t3249;
  t3322 = 1. + t3275;
  t4502 = Sin(var1[12]);
  t1853 = Cos(var1[4]);
  t5936 = -1.*t4854*t5743;
  t5940 = t1850*t5277*t5777;
  t5945 = t5936 + t5940;
  t5727 = t1850*t4854*t5277;
  t5810 = t5743*t5777;
  t5817 = t5727 + t5810;
  t6119 = -1.*t1850*t1853*t2790;
  t6121 = t2322*t5945;
  t6141 = t6119 + t6121;
  t6171 = Cos(var1[13]);
  t6204 = -1.*t6171;
  t6216 = 1. + t6204;
  t6223 = Sin(var1[13]);
  t6236 = t2322*t1850*t1853;
  t6239 = t2790*t5945;
  t6244 = t6236 + t6239;
  t6371 = t3249*t5817;
  t6374 = -1.*t4502*t6141;
  t6392 = t6371 + t6374;
  t6422 = Cos(var1[14]);
  t6423 = -1.*t6422;
  t6432 = 1. + t6423;
  t6457 = Sin(var1[14]);
  t6536 = t6223*t6244;
  t6556 = t6171*t6392;
  t6574 = t6536 + t6556;
  t6608 = t6171*t6244;
  t6610 = -1.*t6223*t6392;
  t6611 = t6608 + t6610;
  t6621 = Cos(var1[15]);
  t6628 = -1.*t6621;
  t6647 = 1. + t6628;
  t6653 = Sin(var1[15]);
  t6663 = -1.*t6457*t6574;
  t6668 = t6422*t6611;
  t6671 = t6663 + t6668;
  t6715 = t6422*t6574;
  t6741 = t6457*t6611;
  t6746 = t6715 + t6741;
  t2493 = -0.0222*t2436;
  t2963 = -0.087*t2790;
  t3101 = 0. + t2493 + t2963;
  t4498 = -0.3151*t3322;
  t4524 = 0.157*t4502;
  t4803 = 0. + t4498 + t4524;
  t5895 = -0.087*t2436;
  t5917 = 0.0222*t2790;
  t5921 = 0. + t5895 + t5917;
  t6024 = -0.157*t3322;
  t6103 = -0.3151*t4502;
  t6116 = 0. + t6024 + t6103;
  t6857 = t1850*t4854;
  t6859 = t5743*t5277*t5777;
  t6875 = t6857 + t6859;
  t6217 = -0.0222*t6216;
  t6224 = 0.3801*t6223;
  t6227 = 0. + t6217 + t6224;
  t6818 = t4854*t5743*t5277;
  t6830 = -1.*t1850*t5777;
  t6831 = t6818 + t6830;
  t6897 = -1.*t1853*t2790*t5743;
  t6902 = t2322*t6875;
  t6909 = t6897 + t6902;
  t6314 = -0.3801*t6216;
  t6336 = -0.0222*t6223;
  t6365 = 0. + t6314 + t6336;
  t6444 = -0.8601*t6432;
  t6464 = -0.0222*t6457;
  t6470 = 0. + t6444 + t6464;
  t6917 = t2322*t1853*t5743;
  t6923 = t2790*t6875;
  t6927 = t6917 + t6923;
  t6954 = t3249*t6831;
  t6961 = -1.*t4502*t6909;
  t6965 = t6954 + t6961;
  t6589 = -0.0222*t6432;
  t6591 = 0.8601*t6457;
  t6598 = 0. + t6589 + t6591;
  t6651 = -0.0211*t6647;
  t6654 = 1.3401*t6653;
  t6655 = 0. + t6651 + t6654;
  t6974 = t6223*t6927;
  t6981 = t6171*t6965;
  t6984 = t6974 + t6981;
  t6995 = t6171*t6927;
  t6997 = -1.*t6223*t6965;
  t7007 = t6995 + t6997;
  t6685 = -1.3401*t6647;
  t6691 = -0.0211*t6653;
  t6714 = 0. + t6685 + t6691;
  t7022 = -1.*t6457*t6984;
  t7023 = t6422*t7007;
  t7024 = t7022 + t7023;
  t7031 = t6422*t6984;
  t7033 = t6457*t7007;
  t7036 = t7031 + t7033;
  t7111 = t2790*t5277;
  t7115 = t2322*t1853*t5777;
  t7116 = t7111 + t7115;
  t7122 = -1.*t2322*t5277;
  t7145 = t1853*t2790*t5777;
  t7154 = t7122 + t7145;
  t7172 = t3249*t1853*t4854;
  t7175 = -1.*t4502*t7116;
  t7184 = t7172 + t7175;
  t7189 = t6223*t7154;
  t7195 = t6171*t7184;
  t7203 = t7189 + t7195;
  t7211 = t6171*t7154;
  t7219 = -1.*t6223*t7184;
  t7228 = t7211 + t7219;
  t7231 = -1.*t6457*t7203;
  t7239 = t6422*t7228;
  t7245 = t7231 + t7239;
  t7257 = t6422*t7203;
  t7258 = t6457*t7228;
  t7261 = t7257 + t7258;

  p_output1(0)=0. + t1850*t1853*t3101 + t4803*t5817 + t5921*t5945 + t6116*t6141 - 0.16705*(t4502*t5817 + t3249*t6141) + t6227*t6244 + t6365*t6392 + t6470*t6574 + t6598*t6611 + t6655*t6671 + t6714*t6746 - 1.3401*(t6653*t6671 + t6621*t6746) - 0.0211*(t6621*t6671 - 1.*t6653*t6746) + var1(0);
  p_output1(1)=0. + t1853*t3101*t5743 + t4803*t6831 + t5921*t6875 + t6116*t6909 - 0.16705*(t4502*t6831 + t3249*t6909) + t6227*t6927 + t6365*t6965 + t6470*t6984 + t6598*t7007 + t6655*t7024 + t6714*t7036 - 1.3401*(t6653*t7024 + t6621*t7036) - 0.0211*(t6621*t7024 - 1.*t6653*t7036) + var1(1);
  p_output1(2)=0. + t1853*t4803*t4854 - 1.*t3101*t5277 + t1853*t5777*t5921 + t6116*t7116 - 0.16705*(t1853*t4502*t4854 + t3249*t7116) + t6227*t7154 + t6365*t7184 + t6470*t7203 + t6598*t7228 + t6655*t7245 + t6714*t7261 - 1.3401*(t6653*t7245 + t6621*t7261) - 0.0211*(t6621*t7245 - 1.*t6653*t7261) + var1(2);
}


       
void p_rAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
