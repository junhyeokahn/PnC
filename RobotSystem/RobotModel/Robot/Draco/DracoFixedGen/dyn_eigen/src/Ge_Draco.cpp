/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:32:13 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Ge_Draco.h"

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
static void output1(Eigen::Matrix<double,10,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t10;
  double t40;
  double t165;
  double t247;
  double t110;
  double t229;
  double t232;
  double t234;
  double t264;
  double t273;
  double t309;
  double t374;
  double t396;
  double t403;
  double t417;
  double t405;
  double t448;
  double t469;
  double t474;
  double t480;
  double t577;
  double t580;
  double t582;
  double t633;
  double t646;
  double t657;
  double t610;
  double t620;
  double t621;
  double t735;
  double t740;
  double t742;
  double t786;
  double t1131;
  double t1161;
  double t1184;
  double t1186;
  double t1233;
  double t1249;
  double t1256;
  double t1273;
  double t1274;
  double t775;
  double t788;
  double t789;
  double t1279;
  double t1306;
  double t1352;
  double t823;
  double t841;
  double t858;
  double t1277;
  double t1356;
  double t1525;
  double t1555;
  double t1592;
  double t1600;
  double t1618;
  double t1620;
  double t1644;
  double t1377;
  double t1380;
  double t1386;
  double t1392;
  double t1413;
  double t1443;
  double t1444;
  double t1476;
  double t1488;
  double t1495;
  double t1728;
  double t1730;
  double t1780;
  double t1970;
  double t1987;
  double t2037;
  double t2094;
  double t2032;
  double t2040;
  double t2047;
  double t2056;
  double t2117;
  double t2119;
  double t2128;
  double t2212;
  double t2215;
  double t2219;
  double t2270;
  double t2260;
  double t2274;
  double t2282;
  double t2289;
  double t2290;
  double t2304;
  double t2321;
  double t2322;
  double t2324;
  double t2346;
  double t2348;
  double t2412;
  double t2415;
  double t2430;
  double t2433;
  double t2350;
  double t2356;
  double t2357;
  double t2538;
  double t2539;
  double t2540;
  double t2548;
  double t2559;
  double t2561;
  double t2476;
  double t2481;
  double t2482;
  double t2566;
  double t2572;
  double t2578;
  double t2431;
  double t2434;
  double t2440;
  double t2580;
  double t2585;
  double t2587;
  double t2579;
  double t2588;
  double t2672;
  double t2674;
  double t2676;
  double t2679;
  double t2683;
  double t2688;
  double t2702;
  double t2599;
  double t2602;
  double t2605;
  double t2608;
  double t2609;
  double t2613;
  double t2624;
  double t2636;
  double t2653;
  double t2662;
  double t2792;
  double t2797;
  double t2799;
  t10 = Cos(var1[1]);
  t40 = Sin(var1[1]);
  t165 = Cos(var1[2]);
  t247 = Sin(var1[2]);
  t110 = -0.90524*t40;
  t229 = -1.*t165;
  t232 = 1. + t229;
  t234 = -0.97024*t232;
  t264 = -0.066675*t247;
  t273 = t234 + t264;
  t309 = -1.*t40*t273;
  t374 = Cos(var1[3]);
  t396 = -1.*t374;
  t403 = 1. + t396;
  t417 = Sin(var1[3]);
  t405 = -1.45024*t403;
  t448 = -0.066675*t417;
  t469 = t405 + t448;
  t474 = -1.*t165*t40*t469;
  t480 = -0.066675*t403;
  t577 = 1.45024*t417;
  t580 = t480 + t577;
  t582 = t40*t247*t580;
  t633 = -1.*t165*t374*t40;
  t646 = t40*t247*t417;
  t657 = t633 + t646;
  t610 = t374*t40*t247;
  t620 = t165*t40*t417;
  t621 = t610 + t620;
  t735 = Cos(var1[4]);
  t740 = -1.*t735;
  t742 = 1. + t740;
  t786 = Sin(var1[4]);
  t1131 = -0.066675*t165;
  t1161 = -0.97024*t247;
  t1184 = t1131 + t1161;
  t1186 = t10*t1184;
  t1233 = -1.*t10*t247*t469;
  t1249 = -1.*t10*t165*t580;
  t1256 = -1.*t10*t374*t247;
  t1273 = -1.*t10*t165*t417;
  t1274 = t1256 + t1273;
  t775 = -1.93024*t742;
  t788 = -0.065597*t786;
  t789 = t775 + t788;
  t1279 = -1.*t10*t165*t374;
  t1306 = t10*t247*t417;
  t1352 = t1279 + t1306;
  t823 = -0.065597*t742;
  t841 = 1.93024*t786;
  t858 = t823 + t841;
  t1277 = -1.698062*t1274;
  t1356 = -0.082744*t1352;
  t1525 = -0.066675*t374;
  t1555 = -1.45024*t417;
  t1592 = t1525 + t1555;
  t1600 = t10*t165*t1592;
  t1618 = 1.45024*t374;
  t1620 = t1618 + t448;
  t1644 = -1.*t10*t247*t1620;
  t1377 = t1274*t789;
  t1380 = t1352*t858;
  t1386 = t735*t1352;
  t1392 = -1.*t1274*t786;
  t1413 = t1386 + t1392;
  t1443 = -0.021314*t1413;
  t1444 = t735*t1274;
  t1476 = t1352*t786;
  t1488 = t1444 + t1476;
  t1495 = -1.919616*t1488;
  t1728 = t10*t165*t374;
  t1730 = -1.*t10*t247*t417;
  t1780 = t1728 + t1730;
  t1970 = Cos(var1[6]);
  t1987 = Sin(var1[6]);
  t2037 = Cos(var1[7]);
  t2094 = Sin(var1[7]);
  t2032 = -0.90524*t1987;
  t2040 = -1.*t2037;
  t2047 = 1. + t2040;
  t2056 = -0.97024*t2047;
  t2117 = -0.066675*t2094;
  t2119 = t2056 + t2117;
  t2128 = -1.*t1987*t2119;
  t2212 = Cos(var1[8]);
  t2215 = -1.*t2212;
  t2219 = 1. + t2215;
  t2270 = Sin(var1[8]);
  t2260 = -1.45024*t2219;
  t2274 = -0.066675*t2270;
  t2282 = t2260 + t2274;
  t2289 = -1.*t2037*t1987*t2282;
  t2290 = -0.066675*t2219;
  t2304 = 1.45024*t2270;
  t2321 = t2290 + t2304;
  t2322 = t1987*t2094*t2321;
  t2324 = t2212*t1987*t2094;
  t2346 = t2037*t1987*t2270;
  t2348 = t2324 + t2346;
  t2412 = Cos(var1[9]);
  t2415 = -1.*t2412;
  t2430 = 1. + t2415;
  t2433 = Sin(var1[9]);
  t2350 = -1.*t2037*t2212*t1987;
  t2356 = t1987*t2094*t2270;
  t2357 = t2350 + t2356;
  t2538 = -0.066675*t2037;
  t2539 = -0.97024*t2094;
  t2540 = t2538 + t2539;
  t2548 = t1970*t2540;
  t2559 = -1.*t1970*t2094*t2282;
  t2561 = -1.*t1970*t2037*t2321;
  t2476 = -1.93024*t2430;
  t2481 = -0.065597*t2433;
  t2482 = t2476 + t2481;
  t2566 = -1.*t1970*t2212*t2094;
  t2572 = -1.*t1970*t2037*t2270;
  t2578 = t2566 + t2572;
  t2431 = -0.065597*t2430;
  t2434 = 1.93024*t2433;
  t2440 = t2431 + t2434;
  t2580 = -1.*t1970*t2037*t2212;
  t2585 = t1970*t2094*t2270;
  t2587 = t2580 + t2585;
  t2579 = -1.698062*t2578;
  t2588 = -0.082744*t2587;
  t2672 = -0.066675*t2212;
  t2674 = -1.45024*t2270;
  t2676 = t2672 + t2674;
  t2679 = t1970*t2037*t2676;
  t2683 = 1.45024*t2212;
  t2688 = t2683 + t2274;
  t2702 = -1.*t1970*t2094*t2688;
  t2599 = t2482*t2578;
  t2602 = t2440*t2587;
  t2605 = -1.*t2433*t2578;
  t2608 = t2412*t2587;
  t2609 = t2605 + t2608;
  t2613 = -0.021314*t2609;
  t2624 = t2412*t2578;
  t2636 = t2433*t2587;
  t2653 = t2624 + t2636;
  t2662 = -1.919616*t2653;
  t2792 = t1970*t2037*t2212;
  t2797 = -1.*t1970*t2094*t2270;
  t2799 = t2792 + t2797;

  p_output1(0)=0;
  p_output1(1)=4.905*(0.02066100000000004*t10 + 0.025729000000000002*t40) + 58.86*(0.01665*t10 + t110 + t309 + 1.210348*t165*t40 - 0.065782*t247*t40) + 37.278*(0.009942*t10 + t110 + t309 + t474 + t582 - 0.082744*t621 - 1.698062*t657) + 4.905*(0.009995000000000032*t10 + t110 + t309 + t474 + t582 - 1.919616*(t657*t735 + t621*t786) - 0.021314*(t621*t735 - 1.*t657*t786) + t657*t789 + t621*t858);
  p_output1(2)=37.278*(t1186 + t1233 + t1249 + t1277 + t1356) + 4.905*(t1186 + t1233 + t1249 + t1377 + t1380 + t1443 + t1495) + 58.86*(t1186 + 0.065782*t10*t165 + 1.210348*t10*t247);
  p_output1(3)=37.278*(t1277 + t1356 + t1600 + t1644) + 4.905*(t1377 + t1380 + t1443 + t1495 + t1600 + t1644);
  p_output1(4)=4.905*(-0.021314*(t1392 - 1.*t1780*t735) + t1780*(-0.065597*t735 - 1.93024*t786) - 1.919616*(t1444 - 1.*t1780*t786) + t1274*(1.93024*t735 + t788));
  p_output1(5)=0;
  p_output1(6)=4.905*(-0.020660999999999985*t1970 + 0.025729000000000002*t1987) + 58.86*(-0.00335*t1970 + t2032 + 1.210348*t1987*t2037 - 0.065782*t1987*t2094 + t2128) + 37.278*(-0.010058*t1970 + t2032 + t2128 + t2289 + t2322 - 0.082744*t2348 - 1.698062*t2357) + 4.905*(-0.010004999999999986*t1970 + t2032 + t2128 + t2289 + t2322 - 1.919616*(t2357*t2412 + t2348*t2433) - 0.021314*(t2348*t2412 - 1.*t2357*t2433) + t2348*t2440 + t2357*t2482);
  p_output1(7)=58.86*(0.065782*t1970*t2037 + 1.210348*t1970*t2094 + t2548) + 37.278*(t2548 + t2559 + t2561 + t2579 + t2588) + 4.905*(t2548 + t2559 + t2561 + t2599 + t2602 + t2613 + t2662);
  p_output1(8)=37.278*(t2579 + t2588 + t2679 + t2702) + 4.905*(t2599 + t2602 + t2613 + t2662 + t2679 + t2702);
  p_output1(9)=4.905*((1.93024*t2412 + t2481)*t2578 + (-0.065597*t2412 - 1.93024*t2433)*t2799 - 0.021314*(t2605 - 1.*t2412*t2799) - 1.919616*(t2624 - 1.*t2433*t2799));
}


       
void Ge_Draco(Eigen::Matrix<double,10,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
