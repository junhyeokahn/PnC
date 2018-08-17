/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:50 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rKnee.h"

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
  double t801;
  double t1059;
  double t1523;
  double t1622;
  double t1679;
  double t3484;
  double t4774;
  double t3599;
  double t4913;
  double t2651;
  double t2784;
  double t2814;
  double t2946;
  double t1028;
  double t5916;
  double t6114;
  double t6213;
  double t4677;
  double t5068;
  double t5074;
  double t6302;
  double t6320;
  double t6323;
  double t6351;
  double t6361;
  double t6364;
  double t6385;
  double t6433;
  double t6460;
  double t6467;
  double t6532;
  double t6543;
  double t6555;
  double t6567;
  double t6585;
  double t6595;
  double t6600;
  double t6681;
  double t6683;
  double t6684;
  double t6740;
  double t6742;
  double t6752;
  double t1640;
  double t1749;
  double t2445;
  double t2943;
  double t3160;
  double t3247;
  double t5533;
  double t5665;
  double t5888;
  double t6222;
  double t6272;
  double t6295;
  double t6937;
  double t6939;
  double t6943;
  double t6377;
  double t6408;
  double t6431;
  double t6912;
  double t6917;
  double t6923;
  double t6959;
  double t6963;
  double t6973;
  double t6520;
  double t6523;
  double t6526;
  double t6599;
  double t6604;
  double t6678;
  double t6986;
  double t6998;
  double t7008;
  double t7046;
  double t7054;
  double t7057;
  double t6720;
  double t6725;
  double t6731;
  double t7062;
  double t7070;
  double t7075;
  double t7077;
  double t7083;
  double t7095;
  double t7188;
  double t7190;
  double t7192;
  double t7199;
  double t7212;
  double t7214;
  double t7252;
  double t7257;
  double t7264;
  double t7273;
  double t7276;
  double t7278;
  double t7293;
  double t7300;
  double t7303;
  t801 = Cos(var1[3]);
  t1059 = Cos(var1[11]);
  t1523 = -1.*t1059;
  t1622 = 1. + t1523;
  t1679 = Sin(var1[11]);
  t3484 = Cos(var1[5]);
  t4774 = Sin(var1[3]);
  t3599 = Sin(var1[4]);
  t4913 = Sin(var1[5]);
  t2651 = Cos(var1[12]);
  t2784 = -1.*t2651;
  t2814 = 1. + t2784;
  t2946 = Sin(var1[12]);
  t1028 = Cos(var1[4]);
  t5916 = -1.*t3484*t4774;
  t6114 = t801*t3599*t4913;
  t6213 = t5916 + t6114;
  t4677 = t801*t3484*t3599;
  t5068 = t4774*t4913;
  t5074 = t4677 + t5068;
  t6302 = -1.*t801*t1028*t1679;
  t6320 = t1059*t6213;
  t6323 = t6302 + t6320;
  t6351 = Cos(var1[13]);
  t6361 = -1.*t6351;
  t6364 = 1. + t6361;
  t6385 = Sin(var1[13]);
  t6433 = t1059*t801*t1028;
  t6460 = t1679*t6213;
  t6467 = t6433 + t6460;
  t6532 = t2651*t5074;
  t6543 = -1.*t2946*t6323;
  t6555 = t6532 + t6543;
  t6567 = Cos(var1[14]);
  t6585 = -1.*t6567;
  t6595 = 1. + t6585;
  t6600 = Sin(var1[14]);
  t6681 = t6385*t6467;
  t6683 = t6351*t6555;
  t6684 = t6681 + t6683;
  t6740 = t6351*t6467;
  t6742 = -1.*t6385*t6555;
  t6752 = t6740 + t6742;
  t1640 = -0.022225*t1622;
  t1749 = -0.086996*t1679;
  t2445 = 0. + t1640 + t1749;
  t2943 = -0.31508*t2814;
  t3160 = 0.156996*t2946;
  t3247 = 0. + t2943 + t3160;
  t5533 = -0.086996*t1622;
  t5665 = 0.022225*t1679;
  t5888 = 0. + t5533 + t5665;
  t6222 = -0.156996*t2814;
  t6272 = -0.31508*t2946;
  t6295 = 0. + t6222 + t6272;
  t6937 = t801*t3484;
  t6939 = t4774*t3599*t4913;
  t6943 = t6937 + t6939;
  t6377 = -0.022225*t6364;
  t6408 = 0.38008*t6385;
  t6431 = 0. + t6377 + t6408;
  t6912 = t3484*t4774*t3599;
  t6917 = -1.*t801*t4913;
  t6923 = t6912 + t6917;
  t6959 = -1.*t1028*t1679*t4774;
  t6963 = t1059*t6943;
  t6973 = t6959 + t6963;
  t6520 = -0.38008*t6364;
  t6523 = -0.022225*t6385;
  t6526 = 0. + t6520 + t6523;
  t6599 = -0.86008*t6595;
  t6604 = -0.022225*t6600;
  t6678 = 0. + t6599 + t6604;
  t6986 = t1059*t1028*t4774;
  t6998 = t1679*t6943;
  t7008 = t6986 + t6998;
  t7046 = t2651*t6923;
  t7054 = -1.*t2946*t6973;
  t7057 = t7046 + t7054;
  t6720 = -0.022225*t6595;
  t6725 = 0.86008*t6600;
  t6731 = 0. + t6720 + t6725;
  t7062 = t6385*t7008;
  t7070 = t6351*t7057;
  t7075 = t7062 + t7070;
  t7077 = t6351*t7008;
  t7083 = -1.*t6385*t7057;
  t7095 = t7077 + t7083;
  t7188 = t1679*t3599;
  t7190 = t1059*t1028*t4913;
  t7192 = t7188 + t7190;
  t7199 = -1.*t1059*t3599;
  t7212 = t1028*t1679*t4913;
  t7214 = t7199 + t7212;
  t7252 = t2651*t1028*t3484;
  t7257 = -1.*t2946*t7192;
  t7264 = t7252 + t7257;
  t7273 = t6385*t7214;
  t7276 = t6351*t7264;
  t7278 = t7273 + t7276;
  t7293 = t6351*t7214;
  t7300 = -1.*t6385*t7264;
  t7303 = t7293 + t7300;

  p_output1(0)=0. + t3247*t5074 + t5888*t6213 + t6295*t6323 - 0.150246*(t2946*t5074 + t2651*t6323) + t6431*t6467 + t6526*t6555 + t6678*t6684 + t6731*t6752 - 0.022225*(-1.*t6600*t6684 + t6567*t6752) - 0.86008*(t6567*t6684 + t6600*t6752) + t1028*t2445*t801 + var1(0);
  p_output1(1)=0. + t1028*t2445*t4774 + t3247*t6923 + t5888*t6943 + t6295*t6973 - 0.150246*(t2946*t6923 + t2651*t6973) + t6431*t7008 + t6526*t7057 + t6678*t7075 + t6731*t7095 - 0.022225*(-1.*t6600*t7075 + t6567*t7095) - 0.86008*(t6567*t7075 + t6600*t7095) + var1(1);
  p_output1(2)=0. + t1028*t3247*t3484 - 1.*t2445*t3599 + t1028*t4913*t5888 + t6295*t7192 - 0.150246*(t1028*t2946*t3484 + t2651*t7192) + t6431*t7214 + t6526*t7264 + t6678*t7278 + t6731*t7303 - 0.022225*(-1.*t6600*t7278 + t6567*t7303) - 0.86008*(t6567*t7278 + t6600*t7303) + var1(2);
}


       
void p_rKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
