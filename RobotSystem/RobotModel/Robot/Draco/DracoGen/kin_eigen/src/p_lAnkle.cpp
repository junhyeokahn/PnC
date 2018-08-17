/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lAnkle.h"

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
  double t479;
  double t731;
  double t1013;
  double t1014;
  double t1352;
  double t37;
  double t278;
  double t310;
  double t500;
  double t578;
  double t688;
  double t699;
  double t2059;
  double t3138;
  double t3547;
  double t3787;
  double t4158;
  double t2686;
  double t2984;
  double t2986;
  double t4605;
  double t4796;
  double t4818;
  double t5772;
  double t5821;
  double t5886;
  double t5946;
  double t5691;
  double t5702;
  double t5749;
  double t6106;
  double t6161;
  double t6178;
  double t6254;
  double t6255;
  double t6263;
  double t6304;
  double t6360;
  double t6361;
  double t6370;
  double t6412;
  double t6426;
  double t6458;
  double t6485;
  double t6499;
  double t6507;
  double t6541;
  double t6578;
  double t6591;
  double t6604;
  double t6667;
  double t6669;
  double t6672;
  double t1255;
  double t1559;
  double t1761;
  double t2121;
  double t2235;
  double t2515;
  double t6797;
  double t6798;
  double t6804;
  double t4038;
  double t4215;
  double t4287;
  double t5447;
  double t5587;
  double t5598;
  double t6821;
  double t6833;
  double t6855;
  double t6865;
  double t6868;
  double t6870;
  double t5900;
  double t5983;
  double t6012;
  double t6185;
  double t6195;
  double t6207;
  double t6264;
  double t6310;
  double t6353;
  double t6900;
  double t6913;
  double t6930;
  double t6947;
  double t6954;
  double t6957;
  double t6393;
  double t6402;
  double t6407;
  double t6517;
  double t6566;
  double t6577;
  double t6969;
  double t6975;
  double t6987;
  double t7006;
  double t7009;
  double t7013;
  double t6646;
  double t6655;
  double t6659;
  double t7019;
  double t7024;
  double t7031;
  double t7043;
  double t7058;
  double t7065;
  double t7133;
  double t7145;
  double t7155;
  double t7214;
  double t7232;
  double t7234;
  double t7241;
  double t7245;
  double t7246;
  double t7273;
  double t7278;
  double t7285;
  double t7289;
  double t7311;
  double t7315;
  double t7319;
  double t7338;
  double t7341;
  double t7348;
  double t7354;
  double t7355;
  t479 = Cos(var1[3]);
  t731 = Cos(var1[6]);
  t1013 = -1.*t731;
  t1014 = 1. + t1013;
  t1352 = Sin(var1[6]);
  t37 = Cos(var1[5]);
  t278 = Sin(var1[3]);
  t310 = -1.*t37*t278;
  t500 = Sin(var1[4]);
  t578 = Sin(var1[5]);
  t688 = t479*t500*t578;
  t699 = t310 + t688;
  t2059 = Cos(var1[4]);
  t3138 = Cos(var1[7]);
  t3547 = -1.*t3138;
  t3787 = 1. + t3547;
  t4158 = Sin(var1[7]);
  t2686 = t731*t699;
  t2984 = -1.*t479*t2059*t1352;
  t2986 = t2686 + t2984;
  t4605 = t479*t37*t500;
  t4796 = t278*t578;
  t4818 = t4605 + t4796;
  t5772 = Cos(var1[8]);
  t5821 = -1.*t5772;
  t5886 = 1. + t5821;
  t5946 = Sin(var1[8]);
  t5691 = t3138*t4818;
  t5702 = -1.*t2986*t4158;
  t5749 = t5691 + t5702;
  t6106 = t479*t2059*t731;
  t6161 = t699*t1352;
  t6178 = t6106 + t6161;
  t6254 = Cos(var1[9]);
  t6255 = -1.*t6254;
  t6263 = 1. + t6255;
  t6304 = Sin(var1[9]);
  t6360 = t5772*t5749;
  t6361 = t6178*t5946;
  t6370 = t6360 + t6361;
  t6412 = t5772*t6178;
  t6426 = -1.*t5749*t5946;
  t6458 = t6412 + t6426;
  t6485 = Cos(var1[10]);
  t6499 = -1.*t6485;
  t6507 = 1. + t6499;
  t6541 = Sin(var1[10]);
  t6578 = -1.*t6304*t6370;
  t6591 = t6254*t6458;
  t6604 = t6578 + t6591;
  t6667 = t6254*t6370;
  t6669 = t6304*t6458;
  t6672 = t6667 + t6669;
  t1255 = 0.087004*t1014;
  t1559 = 0.022225*t1352;
  t1761 = 0. + t1255 + t1559;
  t2121 = -0.022225*t1014;
  t2235 = 0.087004*t1352;
  t2515 = 0. + t2121 + t2235;
  t6797 = t479*t37;
  t6798 = t278*t500*t578;
  t6804 = t6797 + t6798;
  t4038 = 0.157004*t3787;
  t4215 = -0.31508*t4158;
  t4287 = 0. + t4038 + t4215;
  t5447 = -0.31508*t3787;
  t5587 = -0.157004*t4158;
  t5598 = 0. + t5447 + t5587;
  t6821 = t731*t6804;
  t6833 = -1.*t2059*t278*t1352;
  t6855 = t6821 + t6833;
  t6865 = t37*t278*t500;
  t6868 = -1.*t479*t578;
  t6870 = t6865 + t6868;
  t5900 = -0.38008*t5886;
  t5983 = -0.022225*t5946;
  t6012 = 0. + t5900 + t5983;
  t6185 = -0.022225*t5886;
  t6195 = 0.38008*t5946;
  t6207 = 0. + t6185 + t6195;
  t6264 = -0.86008*t6263;
  t6310 = -0.022225*t6304;
  t6353 = 0. + t6264 + t6310;
  t6900 = t3138*t6870;
  t6913 = -1.*t6855*t4158;
  t6930 = t6900 + t6913;
  t6947 = t2059*t731*t278;
  t6954 = t6804*t1352;
  t6957 = t6947 + t6954;
  t6393 = -0.022225*t6263;
  t6402 = 0.86008*t6304;
  t6407 = 0. + t6393 + t6402;
  t6517 = -0.021147*t6507;
  t6566 = 1.34008*t6541;
  t6577 = 0. + t6517 + t6566;
  t6969 = t5772*t6930;
  t6975 = t6957*t5946;
  t6987 = t6969 + t6975;
  t7006 = t5772*t6957;
  t7009 = -1.*t6930*t5946;
  t7013 = t7006 + t7009;
  t6646 = -1.34008*t6507;
  t6655 = -0.021147*t6541;
  t6659 = 0. + t6646 + t6655;
  t7019 = -1.*t6304*t6987;
  t7024 = t6254*t7013;
  t7031 = t7019 + t7024;
  t7043 = t6254*t6987;
  t7058 = t6304*t7013;
  t7065 = t7043 + t7058;
  t7133 = t2059*t731*t578;
  t7145 = t500*t1352;
  t7155 = t7133 + t7145;
  t7214 = t2059*t37*t3138;
  t7232 = -1.*t7155*t4158;
  t7234 = t7214 + t7232;
  t7241 = -1.*t731*t500;
  t7245 = t2059*t578*t1352;
  t7246 = t7241 + t7245;
  t7273 = t5772*t7234;
  t7278 = t7246*t5946;
  t7285 = t7273 + t7278;
  t7289 = t5772*t7246;
  t7311 = -1.*t7234*t5946;
  t7315 = t7289 + t7311;
  t7319 = -1.*t6304*t7285;
  t7338 = t6254*t7315;
  t7341 = t7319 + t7338;
  t7348 = t6254*t7285;
  t7354 = t6304*t7315;
  t7355 = t7348 + t7354;

  p_output1(0)=0. + t2986*t4287 + t2059*t2515*t479 + 0.167004*(t2986*t3138 + t4158*t4818) + t4818*t5598 + t5749*t6012 + t6178*t6207 + t6353*t6370 + t6407*t6458 + t6577*t6604 + t6659*t6672 - 1.34008*(t6541*t6604 + t6485*t6672) - 0.021147*(t6485*t6604 - 1.*t6541*t6672) + t1761*t699 + var1(0);
  p_output1(1)=0. + t2059*t2515*t278 + t1761*t6804 + t4287*t6855 + t5598*t6870 + 0.167004*(t3138*t6855 + t4158*t6870) + t6012*t6930 + t6207*t6957 + t6353*t6987 + t6407*t7013 + t6577*t7031 + t6659*t7065 - 1.34008*(t6541*t7031 + t6485*t7065) - 0.021147*(t6485*t7031 - 1.*t6541*t7065) + var1(1);
  p_output1(2)=0. - 1.*t2515*t500 + t2059*t37*t5598 + t1761*t2059*t578 + t4287*t7155 + 0.167004*(t2059*t37*t4158 + t3138*t7155) + t6012*t7234 + t6207*t7246 + t6353*t7285 + t6407*t7315 + t6577*t7341 + t6659*t7355 - 1.34008*(t6541*t7341 + t6485*t7355) - 0.021147*(t6485*t7341 - 1.*t6541*t7355) + var1(2);
}


       
void p_lAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
