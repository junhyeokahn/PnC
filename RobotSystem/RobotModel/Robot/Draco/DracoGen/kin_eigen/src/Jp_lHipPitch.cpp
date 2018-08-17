/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:45 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipPitch.h"

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
  double t297;
  double t384;
  double t432;
  double t872;
  double t1164;
  double t193;
  double t234;
  double t286;
  double t318;
  double t353;
  double t362;
  double t375;
  double t1361;
  double t2465;
  double t2486;
  double t2567;
  double t2731;
  double t1645;
  double t2026;
  double t2411;
  double t2878;
  double t3124;
  double t3135;
  double t3945;
  double t3974;
  double t4161;
  double t4179;
  double t3697;
  double t3846;
  double t3934;
  double t4485;
  double t4609;
  double t4713;
  double t1143;
  double t1187;
  double t1303;
  double t1538;
  double t1560;
  double t1562;
  double t5456;
  double t5460;
  double t5486;
  double t2725;
  double t2769;
  double t2771;
  double t3238;
  double t3246;
  double t3249;
  double t5493;
  double t5509;
  double t5527;
  double t5585;
  double t5594;
  double t5631;
  double t4178;
  double t4187;
  double t4354;
  double t4790;
  double t4934;
  double t4957;
  double t5693;
  double t5705;
  double t5713;
  double t5717;
  double t5718;
  double t5721;
  double t5922;
  double t5933;
  double t5936;
  double t6033;
  double t6034;
  double t6071;
  double t6079;
  double t6081;
  double t6087;
  double t6168;
  double t6171;
  double t6204;
  double t6262;
  double t6269;
  double t6278;
  double t6290;
  double t6296;
  double t6297;
  double t6350;
  double t6352;
  double t6354;
  double t6373;
  double t6375;
  double t6395;
  double t6404;
  double t6405;
  double t6406;
  double t6488;
  double t6496;
  double t6498;
  double t6517;
  double t6521;
  double t6527;
  double t6579;
  double t6581;
  double t6584;
  double t6659;
  double t6661;
  double t6663;
  double t6753;
  double t6755;
  double t6757;
  double t6843;
  double t6849;
  double t6854;
  double t6801;
  double t6810;
  double t6817;
  double t6832;
  double t6833;
  double t6934;
  double t6954;
  double t6955;
  double t6958;
  double t6973;
  double t6988;
  double t6991;
  double t6998;
  double t7046;
  double t7048;
  double t7051;
  double t7076;
  double t7086;
  double t7096;
  double t7220;
  double t7239;
  double t7241;
  double t7161;
  double t7165;
  double t7200;
  double t7204;
  double t7210;
  double t7307;
  double t7310;
  double t7317;
  double t7356;
  double t7374;
  double t7377;
  double t5830;
  double t5833;
  double t5854;
  double t7335;
  double t7336;
  double t7337;
  double t7442;
  double t7443;
  double t7447;
  double t7469;
  double t7473;
  double t7506;
  double t7512;
  double t7516;
  double t7391;
  double t7393;
  double t7398;
  double t7588;
  double t7589;
  double t7596;
  t297 = Sin(var1[3]);
  t384 = Cos(var1[6]);
  t432 = -1.*t384;
  t872 = 1. + t432;
  t1164 = Sin(var1[6]);
  t193 = Cos(var1[3]);
  t234 = Cos(var1[5]);
  t286 = -1.*t193*t234;
  t318 = Sin(var1[4]);
  t353 = Sin(var1[5]);
  t362 = -1.*t297*t318*t353;
  t375 = t286 + t362;
  t1361 = Cos(var1[4]);
  t2465 = Cos(var1[7]);
  t2486 = -1.*t2465;
  t2567 = 1. + t2486;
  t2731 = Sin(var1[7]);
  t1645 = t384*t375;
  t2026 = t1361*t297*t1164;
  t2411 = t1645 + t2026;
  t2878 = -1.*t234*t297*t318;
  t3124 = t193*t353;
  t3135 = t2878 + t3124;
  t3945 = Cos(var1[8]);
  t3974 = -1.*t3945;
  t4161 = 1. + t3974;
  t4179 = Sin(var1[8]);
  t3697 = t2465*t3135;
  t3846 = -1.*t2411*t2731;
  t3934 = t3697 + t3846;
  t4485 = -1.*t1361*t384*t297;
  t4609 = t375*t1164;
  t4713 = t4485 + t4609;
  t1143 = 0.087004*t872;
  t1187 = 0.022225*t1164;
  t1303 = 0. + t1143 + t1187;
  t1538 = -0.022225*t872;
  t1560 = 0.087004*t1164;
  t1562 = 0. + t1538 + t1560;
  t5456 = -1.*t234*t297;
  t5460 = t193*t318*t353;
  t5486 = t5456 + t5460;
  t2725 = 0.157004*t2567;
  t2769 = -0.31508*t2731;
  t2771 = 0. + t2725 + t2769;
  t3238 = -0.31508*t2567;
  t3246 = -0.157004*t2731;
  t3249 = 0. + t3238 + t3246;
  t5493 = t384*t5486;
  t5509 = -1.*t193*t1361*t1164;
  t5527 = t5493 + t5509;
  t5585 = t193*t234*t318;
  t5594 = t297*t353;
  t5631 = t5585 + t5594;
  t4178 = -0.38008*t4161;
  t4187 = -0.022225*t4179;
  t4354 = 0. + t4178 + t4187;
  t4790 = -0.022225*t4161;
  t4934 = 0.38008*t4179;
  t4957 = 0. + t4790 + t4934;
  t5693 = t2465*t5631;
  t5705 = -1.*t5527*t2731;
  t5713 = t5693 + t5705;
  t5717 = t193*t1361*t384;
  t5718 = t5486*t1164;
  t5721 = t5717 + t5718;
  t5922 = t193*t1361*t384*t353;
  t5933 = t193*t318*t1164;
  t5936 = t5922 + t5933;
  t6033 = t193*t1361*t234*t2465;
  t6034 = -1.*t5936*t2731;
  t6071 = t6033 + t6034;
  t6079 = -1.*t193*t384*t318;
  t6081 = t193*t1361*t353*t1164;
  t6087 = t6079 + t6081;
  t6168 = t1361*t384*t297*t353;
  t6171 = t297*t318*t1164;
  t6204 = t6168 + t6171;
  t6262 = t1361*t234*t2465*t297;
  t6269 = -1.*t6204*t2731;
  t6278 = t6262 + t6269;
  t6290 = -1.*t384*t297*t318;
  t6296 = t1361*t297*t353*t1164;
  t6297 = t6290 + t6296;
  t6350 = -1.*t384*t318*t353;
  t6352 = t1361*t1164;
  t6354 = t6350 + t6352;
  t6373 = -1.*t234*t2465*t318;
  t6375 = -1.*t6354*t2731;
  t6395 = t6373 + t6375;
  t6404 = -1.*t1361*t384;
  t6405 = -1.*t318*t353*t1164;
  t6406 = t6404 + t6405;
  t6488 = t234*t297;
  t6496 = -1.*t193*t318*t353;
  t6498 = t6488 + t6496;
  t6517 = t2465*t6498;
  t6521 = -1.*t384*t5631*t2731;
  t6527 = t6517 + t6521;
  t6579 = t234*t297*t318;
  t6581 = -1.*t193*t353;
  t6584 = t6579 + t6581;
  t6659 = t2465*t375;
  t6661 = -1.*t384*t6584*t2731;
  t6663 = t6659 + t6661;
  t6753 = -1.*t1361*t2465*t353;
  t6755 = -1.*t1361*t234*t384*t2731;
  t6757 = t6753 + t6755;
  t6843 = -1.*t193*t1361*t384;
  t6849 = -1.*t5486*t1164;
  t6854 = t6843 + t6849;
  t6801 = 0.087004*t384;
  t6810 = -0.022225*t1164;
  t6817 = t6801 + t6810;
  t6832 = 0.022225*t384;
  t6833 = t6832 + t1560;
  t6934 = t193*t234;
  t6954 = t297*t318*t353;
  t6955 = t6934 + t6954;
  t6958 = -1.*t6955*t1164;
  t6973 = t4485 + t6958;
  t6988 = t384*t6955;
  t6991 = -1.*t1361*t297*t1164;
  t6998 = t6988 + t6991;
  t7046 = t384*t318;
  t7048 = -1.*t1361*t353*t1164;
  t7051 = t7046 + t7048;
  t7076 = t1361*t384*t353;
  t7086 = t318*t1164;
  t7096 = t7076 + t7086;
  t7220 = -1.*t2465*t5527;
  t7239 = -1.*t5631*t2731;
  t7241 = t7220 + t7239;
  t7161 = -0.157004*t2465;
  t7165 = t7161 + t2769;
  t7200 = -0.31508*t2465;
  t7204 = 0.157004*t2731;
  t7210 = t7200 + t7204;
  t7307 = -1.*t2465*t6998;
  t7310 = -1.*t6584*t2731;
  t7317 = t7307 + t7310;
  t7356 = -1.*t2465*t7096;
  t7374 = -1.*t1361*t234*t2731;
  t7377 = t7356 + t7374;
  t5830 = t3945*t5721;
  t5833 = -1.*t5713*t4179;
  t5854 = t5830 + t5833;
  t7335 = t2465*t6584;
  t7336 = -1.*t6998*t2731;
  t7337 = t7335 + t7336;
  t7442 = -0.022225*t3945;
  t7443 = -0.38008*t4179;
  t7447 = t7442 + t7443;
  t7469 = 0.38008*t3945;
  t7473 = t7469 + t4187;
  t7506 = t1361*t384*t297;
  t7512 = t6955*t1164;
  t7516 = t7506 + t7512;
  t7391 = t1361*t234*t2465;
  t7393 = -1.*t7096*t2731;
  t7398 = t7391 + t7393;
  t7588 = -1.*t384*t318;
  t7589 = t1361*t353*t1164;
  t7596 = t7588 + t7589;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t2411*t2771 - 1.*t1361*t1562*t297 + 0.167004*(t2411*t2465 + t2731*t3135) + t3135*t3249 + t1303*t375 + t3934*t4354 - 0.022225*(-1.*t3934*t4179 + t3945*t4713) - 0.38008*(t3934*t3945 + t4179*t4713) + t4713*t4957;
  p_output1(10)=t1361*t1562*t193 + t1303*t5486 + t2771*t5527 + t3249*t5631 + 0.167004*(t2465*t5527 + t2731*t5631) + t4354*t5713 + t4957*t5721 - 0.38008*(t3945*t5713 + t4179*t5721) - 0.022225*t5854;
  p_output1(11)=0;
  p_output1(12)=-1.*t1562*t193*t318 + t1361*t193*t234*t3249 + t1303*t1361*t193*t353 + t2771*t5936 + 0.167004*(t1361*t193*t234*t2731 + t2465*t5936) + t4354*t6071 + t4957*t6087 - 0.022225*(-1.*t4179*t6071 + t3945*t6087) - 0.38008*(t3945*t6071 + t4179*t6087);
  p_output1(13)=-1.*t1562*t297*t318 + t1361*t234*t297*t3249 + t1303*t1361*t297*t353 + t2771*t6204 + 0.167004*(t1361*t234*t2731*t297 + t2465*t6204) + t4354*t6278 + t4957*t6297 - 0.022225*(-1.*t4179*t6278 + t3945*t6297) - 0.38008*(t3945*t6278 + t4179*t6297);
  p_output1(14)=-1.*t1361*t1562 - 1.*t234*t318*t3249 - 1.*t1303*t318*t353 + t2771*t6354 + 0.167004*(-1.*t234*t2731*t318 + t2465*t6354) + t4354*t6395 + t4957*t6406 - 0.022225*(-1.*t4179*t6395 + t3945*t6406) - 0.38008*(t3945*t6395 + t4179*t6406);
  p_output1(15)=t1303*t5631 + t2771*t384*t5631 + t1164*t4957*t5631 + t3249*t6498 + 0.167004*(t2465*t384*t5631 + t2731*t6498) + t4354*t6527 - 0.38008*(t1164*t4179*t5631 + t3945*t6527) - 0.022225*(t1164*t3945*t5631 - 1.*t4179*t6527);
  p_output1(16)=t3249*t375 + t1303*t6584 + t2771*t384*t6584 + t1164*t4957*t6584 + 0.167004*(t2731*t375 + t2465*t384*t6584) + t4354*t6663 - 0.38008*(t1164*t4179*t6584 + t3945*t6663) - 0.022225*(t1164*t3945*t6584 - 1.*t4179*t6663);
  p_output1(17)=t1303*t1361*t234 - 1.*t1361*t3249*t353 + t1361*t234*t2771*t384 + 0.167004*(-1.*t1361*t2731*t353 + t1361*t234*t2465*t384) + t1164*t1361*t234*t4957 + t4354*t6757 - 0.38008*(t1164*t1361*t234*t4179 + t3945*t6757) - 0.022225*(t1164*t1361*t234*t3945 - 1.*t4179*t6757);
  p_output1(18)=t4957*t5527 + t1361*t193*t6817 + t5486*t6833 + 0.167004*t2465*t6854 + t2771*t6854 - 1.*t2731*t4354*t6854 - 0.38008*(t4179*t5527 - 1.*t2731*t3945*t6854) - 0.022225*(t3945*t5527 + t2731*t4179*t6854);
  p_output1(19)=t1361*t297*t6817 + t6833*t6955 + 0.167004*t2465*t6973 + t2771*t6973 - 1.*t2731*t4354*t6973 + t4957*t6998 - 0.022225*(t2731*t4179*t6973 + t3945*t6998) - 0.38008*(-1.*t2731*t3945*t6973 + t4179*t6998);
  p_output1(20)=-1.*t318*t6817 + t1361*t353*t6833 + 0.167004*t2465*t7051 + t2771*t7051 - 1.*t2731*t4354*t7051 + t4957*t7096 - 0.022225*(t2731*t4179*t7051 + t3945*t7096) - 0.38008*(-1.*t2731*t3945*t7051 + t4179*t7096);
  p_output1(21)=0.167004*t5713 + t5631*t7165 + t5527*t7210 - 0.38008*t3945*t7241 + 0.022225*t4179*t7241 + t4354*t7241;
  p_output1(22)=t6584*t7165 + t6998*t7210 - 0.38008*t3945*t7317 + 0.022225*t4179*t7317 + t4354*t7317 + 0.167004*t7337;
  p_output1(23)=t1361*t234*t7165 + t7096*t7210 - 0.38008*t3945*t7377 + 0.022225*t4179*t7377 + t4354*t7377 + 0.167004*t7398;
  p_output1(24)=-0.022225*(-1.*t3945*t5713 - 1.*t4179*t5721) - 0.38008*t5854 + t5713*t7447 + t5721*t7473;
  p_output1(25)=t7337*t7447 + t7473*t7516 - 0.38008*(-1.*t4179*t7337 + t3945*t7516) - 0.022225*(-1.*t3945*t7337 - 1.*t4179*t7516);
  p_output1(26)=t7398*t7447 + t7473*t7596 - 0.38008*(-1.*t4179*t7398 + t3945*t7596) - 0.022225*(-1.*t3945*t7398 - 1.*t4179*t7596);
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


       
void Jp_lHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
