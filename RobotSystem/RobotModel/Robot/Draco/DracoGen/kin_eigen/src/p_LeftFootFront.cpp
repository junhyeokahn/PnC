/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:56 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootFront.h"

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
  double t700;
  double t1208;
  double t1264;
  double t1304;
  double t1430;
  double t61;
  double t188;
  double t340;
  double t710;
  double t880;
  double t893;
  double t1132;
  double t1867;
  double t2560;
  double t2764;
  double t2779;
  double t2831;
  double t2276;
  double t2299;
  double t2433;
  double t3087;
  double t3136;
  double t3146;
  double t3534;
  double t3558;
  double t3575;
  double t3627;
  double t3455;
  double t3464;
  double t3509;
  double t3939;
  double t3958;
  double t3960;
  double t4164;
  double t4284;
  double t4483;
  double t4566;
  double t4664;
  double t4680;
  double t4698;
  double t4934;
  double t4990;
  double t5063;
  double t5123;
  double t5186;
  double t5192;
  double t5206;
  double t5220;
  double t5250;
  double t5259;
  double t5384;
  double t5393;
  double t5396;
  double t1307;
  double t1719;
  double t1722;
  double t1954;
  double t2124;
  double t2178;
  double t5522;
  double t5526;
  double t5563;
  double t2825;
  double t2864;
  double t3049;
  double t3154;
  double t3197;
  double t3226;
  double t5613;
  double t5621;
  double t5637;
  double t5649;
  double t5695;
  double t5707;
  double t3581;
  double t3747;
  double t3861;
  double t4001;
  double t4058;
  double t4116;
  double t4514;
  double t4578;
  double t4608;
  double t5778;
  double t5781;
  double t5803;
  double t5836;
  double t5877;
  double t5889;
  double t4777;
  double t4810;
  double t4928;
  double t5198;
  double t5216;
  double t5219;
  double t5921;
  double t5956;
  double t5962;
  double t5981;
  double t5983;
  double t6004;
  double t5299;
  double t5332;
  double t5374;
  double t6028;
  double t6036;
  double t6053;
  double t6068;
  double t6072;
  double t6074;
  double t6256;
  double t6263;
  double t6284;
  double t6402;
  double t6416;
  double t6434;
  double t6443;
  double t6464;
  double t6472;
  double t6481;
  double t6494;
  double t6499;
  double t6560;
  double t6566;
  double t6568;
  double t6576;
  double t6579;
  double t6589;
  double t6615;
  double t6623;
  double t6626;
  t700 = Cos(var1[3]);
  t1208 = Cos(var1[6]);
  t1264 = -1.*t1208;
  t1304 = 1. + t1264;
  t1430 = Sin(var1[6]);
  t61 = Cos(var1[5]);
  t188 = Sin(var1[3]);
  t340 = -1.*t61*t188;
  t710 = Sin(var1[4]);
  t880 = Sin(var1[5]);
  t893 = t700*t710*t880;
  t1132 = t340 + t893;
  t1867 = Cos(var1[4]);
  t2560 = Cos(var1[7]);
  t2764 = -1.*t2560;
  t2779 = 1. + t2764;
  t2831 = Sin(var1[7]);
  t2276 = t1208*t1132;
  t2299 = -1.*t700*t1867*t1430;
  t2433 = t2276 + t2299;
  t3087 = t700*t61*t710;
  t3136 = t188*t880;
  t3146 = t3087 + t3136;
  t3534 = Cos(var1[8]);
  t3558 = -1.*t3534;
  t3575 = 1. + t3558;
  t3627 = Sin(var1[8]);
  t3455 = t2560*t3146;
  t3464 = -1.*t2433*t2831;
  t3509 = t3455 + t3464;
  t3939 = t700*t1867*t1208;
  t3958 = t1132*t1430;
  t3960 = t3939 + t3958;
  t4164 = Cos(var1[9]);
  t4284 = -1.*t4164;
  t4483 = 1. + t4284;
  t4566 = Sin(var1[9]);
  t4664 = t3534*t3509;
  t4680 = t3960*t3627;
  t4698 = t4664 + t4680;
  t4934 = t3534*t3960;
  t4990 = -1.*t3509*t3627;
  t5063 = t4934 + t4990;
  t5123 = Cos(var1[10]);
  t5186 = -1.*t5123;
  t5192 = 1. + t5186;
  t5206 = Sin(var1[10]);
  t5220 = -1.*t4566*t4698;
  t5250 = t4164*t5063;
  t5259 = t5220 + t5250;
  t5384 = t4164*t4698;
  t5393 = t4566*t5063;
  t5396 = t5384 + t5393;
  t1307 = 0.087004*t1304;
  t1719 = 0.022225*t1430;
  t1722 = 0. + t1307 + t1719;
  t1954 = -0.022225*t1304;
  t2124 = 0.087004*t1430;
  t2178 = 0. + t1954 + t2124;
  t5522 = t700*t61;
  t5526 = t188*t710*t880;
  t5563 = t5522 + t5526;
  t2825 = 0.157004*t2779;
  t2864 = -0.31508*t2831;
  t3049 = 0. + t2825 + t2864;
  t3154 = -0.31508*t2779;
  t3197 = -0.157004*t2831;
  t3226 = 0. + t3154 + t3197;
  t5613 = t1208*t5563;
  t5621 = -1.*t1867*t188*t1430;
  t5637 = t5613 + t5621;
  t5649 = t61*t188*t710;
  t5695 = -1.*t700*t880;
  t5707 = t5649 + t5695;
  t3581 = -0.38008*t3575;
  t3747 = -0.022225*t3627;
  t3861 = 0. + t3581 + t3747;
  t4001 = -0.022225*t3575;
  t4058 = 0.38008*t3627;
  t4116 = 0. + t4001 + t4058;
  t4514 = -0.86008*t4483;
  t4578 = -0.022225*t4566;
  t4608 = 0. + t4514 + t4578;
  t5778 = t2560*t5707;
  t5781 = -1.*t5637*t2831;
  t5803 = t5778 + t5781;
  t5836 = t1867*t1208*t188;
  t5877 = t5563*t1430;
  t5889 = t5836 + t5877;
  t4777 = -0.022225*t4483;
  t4810 = 0.86008*t4566;
  t4928 = 0. + t4777 + t4810;
  t5198 = -0.021147*t5192;
  t5216 = 1.34008*t5206;
  t5219 = 0. + t5198 + t5216;
  t5921 = t3534*t5803;
  t5956 = t5889*t3627;
  t5962 = t5921 + t5956;
  t5981 = t3534*t5889;
  t5983 = -1.*t5803*t3627;
  t6004 = t5981 + t5983;
  t5299 = -1.34008*t5192;
  t5332 = -0.021147*t5206;
  t5374 = 0. + t5299 + t5332;
  t6028 = -1.*t4566*t5962;
  t6036 = t4164*t6004;
  t6053 = t6028 + t6036;
  t6068 = t4164*t5962;
  t6072 = t4566*t6004;
  t6074 = t6068 + t6072;
  t6256 = t1867*t1208*t880;
  t6263 = t710*t1430;
  t6284 = t6256 + t6263;
  t6402 = t1867*t61*t2560;
  t6416 = -1.*t6284*t2831;
  t6434 = t6402 + t6416;
  t6443 = -1.*t1208*t710;
  t6464 = t1867*t880*t1430;
  t6472 = t6443 + t6464;
  t6481 = t3534*t6434;
  t6494 = t6472*t3627;
  t6499 = t6481 + t6494;
  t6560 = t3534*t6472;
  t6566 = -1.*t6434*t3627;
  t6568 = t6560 + t6566;
  t6576 = -1.*t4566*t6499;
  t6579 = t4164*t6568;
  t6589 = t6576 + t6579;
  t6615 = t4164*t6499;
  t6623 = t4566*t6568;
  t6626 = t6615 + t6623;

  p_output1(0)=0. + t1132*t1722 + t2433*t3049 + 0.167004*(t2433*t2560 + t2831*t3146) + t3146*t3226 + t3509*t3861 + t3960*t4116 + t4608*t4698 + t4928*t5063 + t5219*t5259 + t5374*t5396 - 1.250132*(t5206*t5259 + t5123*t5396) + 0.043925*(t5123*t5259 - 1.*t5206*t5396) + t1867*t2178*t700 + var1(0);
  p_output1(1)=0. + t1867*t188*t2178 + t1722*t5563 + t3049*t5637 + t3226*t5707 + 0.167004*(t2560*t5637 + t2831*t5707) + t3861*t5803 + t4116*t5889 + t4608*t5962 + t4928*t6004 + t5219*t6053 + t5374*t6074 - 1.250132*(t5206*t6053 + t5123*t6074) + 0.043925*(t5123*t6053 - 1.*t5206*t6074) + var1(1);
  p_output1(2)=0. + t1867*t3226*t61 + t3049*t6284 + 0.167004*(t1867*t2831*t61 + t2560*t6284) + t3861*t6434 + t4116*t6472 + t4608*t6499 + t4928*t6568 + t5219*t6589 + t5374*t6626 - 1.250132*(t5206*t6589 + t5123*t6626) + 0.043925*(t5123*t6589 - 1.*t5206*t6626) - 1.*t2178*t710 + t1722*t1867*t880 + var1(2);
}


       
void p_LeftFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
