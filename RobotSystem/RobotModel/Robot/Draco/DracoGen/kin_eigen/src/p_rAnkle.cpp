/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:52 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rAnkle.h"

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
  double t211;
  double t1491;
  double t1592;
  double t1840;
  double t1853;
  double t4161;
  double t4271;
  double t4173;
  double t4323;
  double t2264;
  double t2693;
  double t3907;
  double t3990;
  double t878;
  double t4394;
  double t4395;
  double t4396;
  double t4235;
  double t4324;
  double t4344;
  double t4407;
  double t4411;
  double t4412;
  double t4414;
  double t4415;
  double t4416;
  double t4418;
  double t4421;
  double t4426;
  double t4427;
  double t4447;
  double t4448;
  double t4449;
  double t4451;
  double t4456;
  double t4457;
  double t4460;
  double t4466;
  double t4469;
  double t4470;
  double t4924;
  double t5223;
  double t5401;
  double t5628;
  double t5650;
  double t5917;
  double t6194;
  double t6322;
  double t6329;
  double t6334;
  double t6467;
  double t6473;
  double t6485;
  double t1852;
  double t1873;
  double t1906;
  double t3986;
  double t4076;
  double t4084;
  double t4365;
  double t4380;
  double t4392;
  double t4399;
  double t4404;
  double t4405;
  double t6785;
  double t6786;
  double t6790;
  double t4417;
  double t4419;
  double t4420;
  double t6724;
  double t6725;
  double t6731;
  double t6816;
  double t6854;
  double t6880;
  double t4442;
  double t4443;
  double t4444;
  double t4458;
  double t4462;
  double t4464;
  double t6937;
  double t6939;
  double t6943;
  double t7021;
  double t7025;
  double t7045;
  double t4570;
  double t4892;
  double t4921;
  double t6167;
  double t6205;
  double t6245;
  double t7061;
  double t7074;
  double t7107;
  double t7125;
  double t7137;
  double t7151;
  double t6377;
  double t6386;
  double t6443;
  double t7160;
  double t7164;
  double t7170;
  double t7195;
  double t7208;
  double t7214;
  double t7311;
  double t7314;
  double t7317;
  double t7331;
  double t7338;
  double t7341;
  double t7385;
  double t7389;
  double t7404;
  double t7409;
  double t7438;
  double t7443;
  double t7455;
  double t7461;
  double t7474;
  double t7486;
  double t7489;
  double t7490;
  double t7500;
  double t7504;
  double t7510;
  t211 = Cos(var1[3]);
  t1491 = Cos(var1[11]);
  t1592 = -1.*t1491;
  t1840 = 1. + t1592;
  t1853 = Sin(var1[11]);
  t4161 = Cos(var1[5]);
  t4271 = Sin(var1[3]);
  t4173 = Sin(var1[4]);
  t4323 = Sin(var1[5]);
  t2264 = Cos(var1[12]);
  t2693 = -1.*t2264;
  t3907 = 1. + t2693;
  t3990 = Sin(var1[12]);
  t878 = Cos(var1[4]);
  t4394 = -1.*t4161*t4271;
  t4395 = t211*t4173*t4323;
  t4396 = t4394 + t4395;
  t4235 = t211*t4161*t4173;
  t4324 = t4271*t4323;
  t4344 = t4235 + t4324;
  t4407 = -1.*t211*t878*t1853;
  t4411 = t1491*t4396;
  t4412 = t4407 + t4411;
  t4414 = Cos(var1[13]);
  t4415 = -1.*t4414;
  t4416 = 1. + t4415;
  t4418 = Sin(var1[13]);
  t4421 = t1491*t211*t878;
  t4426 = t1853*t4396;
  t4427 = t4421 + t4426;
  t4447 = t2264*t4344;
  t4448 = -1.*t3990*t4412;
  t4449 = t4447 + t4448;
  t4451 = Cos(var1[14]);
  t4456 = -1.*t4451;
  t4457 = 1. + t4456;
  t4460 = Sin(var1[14]);
  t4466 = t4418*t4427;
  t4469 = t4414*t4449;
  t4470 = t4466 + t4469;
  t4924 = t4414*t4427;
  t5223 = -1.*t4418*t4449;
  t5401 = t4924 + t5223;
  t5628 = Cos(var1[15]);
  t5650 = -1.*t5628;
  t5917 = 1. + t5650;
  t6194 = Sin(var1[15]);
  t6322 = -1.*t4460*t4470;
  t6329 = t4451*t5401;
  t6334 = t6322 + t6329;
  t6467 = t4451*t4470;
  t6473 = t4460*t5401;
  t6485 = t6467 + t6473;
  t1852 = -0.022225*t1840;
  t1873 = -0.086996*t1853;
  t1906 = 0. + t1852 + t1873;
  t3986 = -0.31508*t3907;
  t4076 = 0.156996*t3990;
  t4084 = 0. + t3986 + t4076;
  t4365 = -0.086996*t1840;
  t4380 = 0.022225*t1853;
  t4392 = 0. + t4365 + t4380;
  t4399 = -0.156996*t3907;
  t4404 = -0.31508*t3990;
  t4405 = 0. + t4399 + t4404;
  t6785 = t211*t4161;
  t6786 = t4271*t4173*t4323;
  t6790 = t6785 + t6786;
  t4417 = -0.022225*t4416;
  t4419 = 0.38008*t4418;
  t4420 = 0. + t4417 + t4419;
  t6724 = t4161*t4271*t4173;
  t6725 = -1.*t211*t4323;
  t6731 = t6724 + t6725;
  t6816 = -1.*t878*t1853*t4271;
  t6854 = t1491*t6790;
  t6880 = t6816 + t6854;
  t4442 = -0.38008*t4416;
  t4443 = -0.022225*t4418;
  t4444 = 0. + t4442 + t4443;
  t4458 = -0.86008*t4457;
  t4462 = -0.022225*t4460;
  t4464 = 0. + t4458 + t4462;
  t6937 = t1491*t878*t4271;
  t6939 = t1853*t6790;
  t6943 = t6937 + t6939;
  t7021 = t2264*t6731;
  t7025 = -1.*t3990*t6880;
  t7045 = t7021 + t7025;
  t4570 = -0.022225*t4457;
  t4892 = 0.86008*t4460;
  t4921 = 0. + t4570 + t4892;
  t6167 = -0.021147*t5917;
  t6205 = 1.34008*t6194;
  t6245 = 0. + t6167 + t6205;
  t7061 = t4418*t6943;
  t7074 = t4414*t7045;
  t7107 = t7061 + t7074;
  t7125 = t4414*t6943;
  t7137 = -1.*t4418*t7045;
  t7151 = t7125 + t7137;
  t6377 = -1.34008*t5917;
  t6386 = -0.021147*t6194;
  t6443 = 0. + t6377 + t6386;
  t7160 = -1.*t4460*t7107;
  t7164 = t4451*t7151;
  t7170 = t7160 + t7164;
  t7195 = t4451*t7107;
  t7208 = t4460*t7151;
  t7214 = t7195 + t7208;
  t7311 = t1853*t4173;
  t7314 = t1491*t878*t4323;
  t7317 = t7311 + t7314;
  t7331 = -1.*t1491*t4173;
  t7338 = t878*t1853*t4323;
  t7341 = t7331 + t7338;
  t7385 = t2264*t878*t4161;
  t7389 = -1.*t3990*t7317;
  t7404 = t7385 + t7389;
  t7409 = t4418*t7341;
  t7438 = t4414*t7404;
  t7443 = t7409 + t7438;
  t7455 = t4414*t7341;
  t7461 = -1.*t4418*t7404;
  t7474 = t7455 + t7461;
  t7486 = -1.*t4460*t7443;
  t7489 = t4451*t7474;
  t7490 = t7486 + t7489;
  t7500 = t4451*t7443;
  t7504 = t4460*t7474;
  t7510 = t7500 + t7504;

  p_output1(0)=0. + t4084*t4344 + t4392*t4396 + t4405*t4412 - 0.166996*(t3990*t4344 + t2264*t4412) + t4420*t4427 + t4444*t4449 + t4464*t4470 + t4921*t5401 + t6245*t6334 + t6443*t6485 - 1.34008*(t6194*t6334 + t5628*t6485) - 0.021147*(t5628*t6334 - 1.*t6194*t6485) + t1906*t211*t878 + var1(0);
  p_output1(1)=0. + t4084*t6731 + t4392*t6790 + t4405*t6880 - 0.166996*(t3990*t6731 + t2264*t6880) + t4420*t6943 + t4444*t7045 + t4464*t7107 + t4921*t7151 + t6245*t7170 + t6443*t7214 - 1.34008*(t6194*t7170 + t5628*t7214) - 0.021147*(t5628*t7170 - 1.*t6194*t7214) + t1906*t4271*t878 + var1(1);
  p_output1(2)=0. - 1.*t1906*t4173 + t4405*t7317 + t4420*t7341 + t4444*t7404 + t4464*t7443 + t4921*t7474 + t6245*t7490 + t6443*t7510 - 1.34008*(t6194*t7490 + t5628*t7510) - 0.021147*(t5628*t7490 - 1.*t6194*t7510) + t4084*t4161*t878 + t4323*t4392*t878 - 0.166996*(t2264*t7317 + t3990*t4161*t878) + var1(2);
}


       
void p_rAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
