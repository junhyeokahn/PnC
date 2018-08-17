/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:53 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rAnkle.h"

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
  double t417;
  double t995;
  double t1090;
  double t1002;
  double t1224;
  double t538;
  double t864;
  double t351;
  double t1011;
  double t1235;
  double t1251;
  double t1433;
  double t851;
  double t1302;
  double t1362;
  double t331;
  double t1521;
  double t1539;
  double t1550;
  double t1585;
  double t1603;
  double t1665;
  double t1752;
  double t1759;
  double t1769;
  double t1776;
  double t1797;
  double t1829;
  double t1426;
  double t1813;
  double t1824;
  double t292;
  double t1845;
  double t1851;
  double t1894;
  double t2058;
  double t1828;
  double t1923;
  double t1974;
  double t90;
  double t2089;
  double t2118;
  double t2143;
  double t2423;
  double t2439;
  double t2443;
  double t2411;
  double t2449;
  double t2502;
  double t2536;
  double t2546;
  double t2553;
  double t2574;
  double t2577;
  double t2593;
  double t2646;
  double t2649;
  double t2719;
  double t2503;
  double t2781;
  double t2828;
  double t2867;
  double t2924;
  double t2930;
  double t2848;
  double t2973;
  double t2987;
  double t3000;
  double t3007;
  double t3034;
  double t3229;
  double t3238;
  double t3258;
  double t3279;
  double t3280;
  double t3322;
  double t3323;
  double t3343;
  double t3381;
  double t3264;
  double t3418;
  double t3430;
  double t3496;
  double t3498;
  double t3552;
  double t3489;
  double t3555;
  double t3567;
  double t3585;
  double t3613;
  double t3664;
  double t1975;
  double t2144;
  double t2996;
  double t3103;
  double t3578;
  double t3713;
  double t4417;
  double t4419;
  double t4444;
  double t4458;
  double t3955;
  double t3959;
  double t3968;
  double t4935;
  double t4946;
  double t5144;
  double t5187;
  double t5288;
  double t5301;
  double t4185;
  double t2160;
  double t2227;
  double t2309;
  double t4426;
  double t4427;
  double t4428;
  double t4466;
  double t4469;
  double t4550;
  double t4674;
  double t4675;
  double t4696;
  double t4888;
  double t4905;
  double t4922;
  double t4978;
  double t4989;
  double t5031;
  double t3986;
  double t3996;
  double t4010;
  double t5047;
  double t5089;
  double t5125;
  double t5190;
  double t5200;
  double t5211;
  double t5227;
  double t5231;
  double t5251;
  double t5329;
  double t5331;
  double t5332;
  double t5375;
  double t5401;
  double t5408;
  double t4282;
  double t3142;
  double t3151;
  double t3172;
  double t4020;
  double t4033;
  double t4045;
  double t4344;
  double t3715;
  double t3721;
  double t3789;
  t417 = Cos(var1[3]);
  t995 = Cos(var1[5]);
  t1090 = Sin(var1[4]);
  t1002 = Sin(var1[3]);
  t1224 = Sin(var1[5]);
  t538 = Cos(var1[4]);
  t864 = Sin(var1[11]);
  t351 = Cos(var1[11]);
  t1011 = -1.*t995*t1002;
  t1235 = t417*t1090*t1224;
  t1251 = t1011 + t1235;
  t1433 = Cos(var1[13]);
  t851 = t351*t417*t538;
  t1302 = t864*t1251;
  t1362 = t851 + t1302;
  t331 = Sin(var1[13]);
  t1521 = Cos(var1[12]);
  t1539 = t417*t995*t1090;
  t1550 = t1002*t1224;
  t1585 = t1539 + t1550;
  t1603 = t1521*t1585;
  t1665 = Sin(var1[12]);
  t1752 = -1.*t417*t538*t864;
  t1759 = t351*t1251;
  t1769 = t1752 + t1759;
  t1776 = -1.*t1665*t1769;
  t1797 = t1603 + t1776;
  t1829 = Cos(var1[14]);
  t1426 = t331*t1362;
  t1813 = t1433*t1797;
  t1824 = t1426 + t1813;
  t292 = Sin(var1[14]);
  t1845 = t1433*t1362;
  t1851 = -1.*t331*t1797;
  t1894 = t1845 + t1851;
  t2058 = Cos(var1[15]);
  t1828 = -1.*t292*t1824;
  t1923 = t1829*t1894;
  t1974 = t1828 + t1923;
  t90 = Sin(var1[15]);
  t2089 = t1829*t1824;
  t2118 = t292*t1894;
  t2143 = t2089 + t2118;
  t2423 = t417*t995;
  t2439 = t1002*t1090*t1224;
  t2443 = t2423 + t2439;
  t2411 = t351*t538*t1002;
  t2449 = t864*t2443;
  t2502 = t2411 + t2449;
  t2536 = t995*t1002*t1090;
  t2546 = -1.*t417*t1224;
  t2553 = t2536 + t2546;
  t2574 = t1521*t2553;
  t2577 = -1.*t538*t864*t1002;
  t2593 = t351*t2443;
  t2646 = t2577 + t2593;
  t2649 = -1.*t1665*t2646;
  t2719 = t2574 + t2649;
  t2503 = t331*t2502;
  t2781 = t1433*t2719;
  t2828 = t2503 + t2781;
  t2867 = t1433*t2502;
  t2924 = -1.*t331*t2719;
  t2930 = t2867 + t2924;
  t2848 = -1.*t292*t2828;
  t2973 = t1829*t2930;
  t2987 = t2848 + t2973;
  t3000 = t1829*t2828;
  t3007 = t292*t2930;
  t3034 = t3000 + t3007;
  t3229 = -1.*t351*t1090;
  t3238 = t538*t864*t1224;
  t3258 = t3229 + t3238;
  t3279 = t1521*t538*t995;
  t3280 = t864*t1090;
  t3322 = t351*t538*t1224;
  t3323 = t3280 + t3322;
  t3343 = -1.*t1665*t3323;
  t3381 = t3279 + t3343;
  t3264 = t331*t3258;
  t3418 = t1433*t3381;
  t3430 = t3264 + t3418;
  t3496 = t1433*t3258;
  t3498 = -1.*t331*t3381;
  t3552 = t3496 + t3498;
  t3489 = -1.*t292*t3430;
  t3555 = t1829*t3552;
  t3567 = t3489 + t3555;
  t3585 = t1829*t3430;
  t3613 = t292*t3552;
  t3664 = t3585 + t3613;
  t1975 = t90*t1974;
  t2144 = t2058*t2143;
  t2996 = t90*t2987;
  t3103 = t2058*t3034;
  t3578 = t90*t3567;
  t3713 = t2058*t3664;
  t4417 = -1.*t351;
  t4419 = 1. + t4417;
  t4444 = -1.*t1521;
  t4458 = 1. + t4444;
  t3955 = t1665*t1585;
  t3959 = t1521*t1769;
  t3968 = t3955 + t3959;
  t4935 = -1.*t1433;
  t4946 = 1. + t4935;
  t5144 = -1.*t1829;
  t5187 = 1. + t5144;
  t5288 = -1.*t2058;
  t5301 = 1. + t5288;
  t4185 = t1975 + t2144;
  t2160 = t2058*t1974;
  t2227 = -1.*t90*t2143;
  t2309 = t2160 + t2227;
  t4426 = -0.022225*t4419;
  t4427 = -0.086996*t864;
  t4428 = 0. + t4426 + t4427;
  t4466 = -0.31508*t4458;
  t4469 = 0.156996*t1665;
  t4550 = 0. + t4466 + t4469;
  t4674 = -0.086996*t4419;
  t4675 = 0.022225*t864;
  t4696 = 0. + t4674 + t4675;
  t4888 = -0.156996*t4458;
  t4905 = -0.31508*t1665;
  t4922 = 0. + t4888 + t4905;
  t4978 = -0.022225*t4946;
  t4989 = 0.38008*t331;
  t5031 = 0. + t4978 + t4989;
  t3986 = t1665*t2553;
  t3996 = t1521*t2646;
  t4010 = t3986 + t3996;
  t5047 = -0.38008*t4946;
  t5089 = -0.022225*t331;
  t5125 = 0. + t5047 + t5089;
  t5190 = -0.86008*t5187;
  t5200 = -0.022225*t292;
  t5211 = 0. + t5190 + t5200;
  t5227 = -0.022225*t5187;
  t5231 = 0.86008*t292;
  t5251 = 0. + t5227 + t5231;
  t5329 = -0.021147*t5301;
  t5331 = 1.34008*t90;
  t5332 = 0. + t5329 + t5331;
  t5375 = -1.34008*t5301;
  t5401 = -0.021147*t90;
  t5408 = 0. + t5375 + t5401;
  t4282 = t2996 + t3103;
  t3142 = t2058*t2987;
  t3151 = -1.*t90*t3034;
  t3172 = t3142 + t3151;
  t4020 = t538*t995*t1665;
  t4033 = t1521*t3323;
  t4045 = t4020 + t4033;
  t4344 = t3578 + t3713;
  t3715 = t2058*t3567;
  t3721 = -1.*t90*t3664;
  t3789 = t3715 + t3721;

  p_output1(0)=t1975 + t2144 + 0.000796*t2309;
  p_output1(1)=t2996 + t3103 + 0.000796*t3172;
  p_output1(2)=t3578 + t3713 + 0.000796*t3789;
  p_output1(3)=0.;
  p_output1(4)=t3968;
  p_output1(5)=t4010;
  p_output1(6)=t4045;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1974*t2058 + 0.000796*t4185 + t2143*t90;
  p_output1(9)=-1.*t2058*t2987 + 0.000796*t4282 + t3034*t90;
  p_output1(10)=-1.*t2058*t3567 + 0.000796*t4344 + t3664*t90;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.021147*t2309 - 0.166996*t3968 - 1.34008*t4185 + t1585*t4550 + t1251*t4696 + t1769*t4922 + t1362*t5031 + t1797*t5125 + t1824*t5211 + t1894*t5251 + t1974*t5332 + t417*t4428*t538 + t2143*t5408 + var1(0);
  p_output1(13)=0. - 0.021147*t3172 - 0.166996*t4010 - 1.34008*t4282 + t2553*t4550 + t2443*t4696 + t2646*t4922 + t2502*t5031 + t2719*t5125 + t2828*t5211 + t2930*t5251 + t2987*t5332 + t1002*t4428*t538 + t3034*t5408 + var1(1);
  p_output1(14)=0. - 0.021147*t3789 - 0.166996*t4045 - 1.34008*t4344 - 1.*t1090*t4428 + t3323*t4922 + t3258*t5031 + t3381*t5125 + t3430*t5211 + t3552*t5251 + t3567*t5332 + t1224*t4696*t538 + t3664*t5408 + t4550*t538*t995 + var1(2);
  p_output1(15)=1.;
}


       
void H_rAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
