/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:07 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_LeftFootBack.h"

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
  double t1477;
  double t2041;
  double t2406;
  double t2441;
  double t2641;
  double t456;
  double t1260;
  double t1337;
  double t1549;
  double t1628;
  double t1771;
  double t1897;
  double t3122;
  double t3558;
  double t3676;
  double t3727;
  double t3763;
  double t3481;
  double t3497;
  double t3543;
  double t3839;
  double t3863;
  double t4020;
  double t4793;
  double t4828;
  double t4873;
  double t4902;
  double t4735;
  double t4738;
  double t4792;
  double t5038;
  double t5078;
  double t5090;
  double t5265;
  double t5303;
  double t5311;
  double t5344;
  double t5378;
  double t5393;
  double t5401;
  double t5469;
  double t5508;
  double t5512;
  double t5539;
  double t5546;
  double t5559;
  double t5606;
  double t5645;
  double t5648;
  double t5655;
  double t5733;
  double t5746;
  double t5824;
  double t2452;
  double t2698;
  double t2778;
  double t3154;
  double t3340;
  double t3390;
  double t5938;
  double t5952;
  double t5962;
  double t3752;
  double t3775;
  double t3784;
  double t4042;
  double t4351;
  double t4482;
  double t5980;
  double t5981;
  double t5987;
  double t6003;
  double t6005;
  double t6020;
  double t4899;
  double t4919;
  double t4968;
  double t5143;
  double t5166;
  double t5167;
  double t5334;
  double t5350;
  double t5369;
  double t6063;
  double t6095;
  double t6099;
  double t6116;
  double t6122;
  double t6125;
  double t5421;
  double t5453;
  double t5466;
  double t5569;
  double t5620;
  double t5623;
  double t6149;
  double t6153;
  double t6187;
  double t6201;
  double t6209;
  double t6222;
  double t5700;
  double t5706;
  double t5710;
  double t6242;
  double t6290;
  double t6313;
  double t6333;
  double t6342;
  double t6356;
  double t6467;
  double t6480;
  double t6491;
  double t6539;
  double t6542;
  double t6543;
  double t6553;
  double t6555;
  double t6556;
  double t6590;
  double t6607;
  double t6615;
  double t6620;
  double t6622;
  double t6639;
  double t6647;
  double t6657;
  double t6661;
  double t6687;
  double t6703;
  double t6720;
  t1477 = Cos(var1[3]);
  t2041 = Cos(var1[6]);
  t2406 = -1.*t2041;
  t2441 = 1. + t2406;
  t2641 = Sin(var1[6]);
  t456 = Cos(var1[5]);
  t1260 = Sin(var1[3]);
  t1337 = -1.*t456*t1260;
  t1549 = Sin(var1[4]);
  t1628 = Sin(var1[5]);
  t1771 = t1477*t1549*t1628;
  t1897 = t1337 + t1771;
  t3122 = Cos(var1[4]);
  t3558 = Cos(var1[7]);
  t3676 = -1.*t3558;
  t3727 = 1. + t3676;
  t3763 = Sin(var1[7]);
  t3481 = t2041*t1897;
  t3497 = -1.*t1477*t3122*t2641;
  t3543 = t3481 + t3497;
  t3839 = t1477*t456*t1549;
  t3863 = t1260*t1628;
  t4020 = t3839 + t3863;
  t4793 = Cos(var1[8]);
  t4828 = -1.*t4793;
  t4873 = 1. + t4828;
  t4902 = Sin(var1[8]);
  t4735 = t3558*t4020;
  t4738 = -1.*t3543*t3763;
  t4792 = t4735 + t4738;
  t5038 = t1477*t3122*t2041;
  t5078 = t1897*t2641;
  t5090 = t5038 + t5078;
  t5265 = Cos(var1[9]);
  t5303 = -1.*t5265;
  t5311 = 1. + t5303;
  t5344 = Sin(var1[9]);
  t5378 = t4793*t4792;
  t5393 = t5090*t4902;
  t5401 = t5378 + t5393;
  t5469 = t4793*t5090;
  t5508 = -1.*t4792*t4902;
  t5512 = t5469 + t5508;
  t5539 = Cos(var1[10]);
  t5546 = -1.*t5539;
  t5559 = 1. + t5546;
  t5606 = Sin(var1[10]);
  t5645 = -1.*t5344*t5401;
  t5648 = t5265*t5512;
  t5655 = t5645 + t5648;
  t5733 = t5265*t5401;
  t5746 = t5344*t5512;
  t5824 = t5733 + t5746;
  t2452 = 0.087004*t2441;
  t2698 = 0.022225*t2641;
  t2778 = 0. + t2452 + t2698;
  t3154 = -0.022225*t2441;
  t3340 = 0.087004*t2641;
  t3390 = 0. + t3154 + t3340;
  t5938 = t1477*t456;
  t5952 = t1260*t1549*t1628;
  t5962 = t5938 + t5952;
  t3752 = 0.157004*t3727;
  t3775 = -0.31508*t3763;
  t3784 = 0. + t3752 + t3775;
  t4042 = -0.31508*t3727;
  t4351 = -0.157004*t3763;
  t4482 = 0. + t4042 + t4351;
  t5980 = t2041*t5962;
  t5981 = -1.*t3122*t1260*t2641;
  t5987 = t5980 + t5981;
  t6003 = t456*t1260*t1549;
  t6005 = -1.*t1477*t1628;
  t6020 = t6003 + t6005;
  t4899 = -0.38008*t4873;
  t4919 = -0.022225*t4902;
  t4968 = 0. + t4899 + t4919;
  t5143 = -0.022225*t4873;
  t5166 = 0.38008*t4902;
  t5167 = 0. + t5143 + t5166;
  t5334 = -0.86008*t5311;
  t5350 = -0.022225*t5344;
  t5369 = 0. + t5334 + t5350;
  t6063 = t3558*t6020;
  t6095 = -1.*t5987*t3763;
  t6099 = t6063 + t6095;
  t6116 = t3122*t2041*t1260;
  t6122 = t5962*t2641;
  t6125 = t6116 + t6122;
  t5421 = -0.022225*t5311;
  t5453 = 0.86008*t5344;
  t5466 = 0. + t5421 + t5453;
  t5569 = -0.021147*t5559;
  t5620 = 1.34008*t5606;
  t5623 = 0. + t5569 + t5620;
  t6149 = t4793*t6099;
  t6153 = t6125*t4902;
  t6187 = t6149 + t6153;
  t6201 = t4793*t6125;
  t6209 = -1.*t6099*t4902;
  t6222 = t6201 + t6209;
  t5700 = -1.34008*t5559;
  t5706 = -0.021147*t5606;
  t5710 = 0. + t5700 + t5706;
  t6242 = -1.*t5344*t6187;
  t6290 = t5265*t6222;
  t6313 = t6242 + t6290;
  t6333 = t5265*t6187;
  t6342 = t5344*t6222;
  t6356 = t6333 + t6342;
  t6467 = t3122*t2041*t1628;
  t6480 = t1549*t2641;
  t6491 = t6467 + t6480;
  t6539 = t3122*t456*t3558;
  t6542 = -1.*t6491*t3763;
  t6543 = t6539 + t6542;
  t6553 = -1.*t2041*t1549;
  t6555 = t3122*t1628*t2641;
  t6556 = t6553 + t6555;
  t6590 = t4793*t6543;
  t6607 = t6556*t4902;
  t6615 = t6590 + t6607;
  t6620 = t4793*t6556;
  t6622 = -1.*t6543*t4902;
  t6639 = t6620 + t6622;
  t6647 = -1.*t5344*t6615;
  t6657 = t5265*t6639;
  t6661 = t6647 + t6657;
  t6687 = t5265*t6615;
  t6703 = t5344*t6639;
  t6720 = t6687 + t6703;

  p_output1(0)=0. + t1897*t2778 + t1477*t3122*t3390 + t3543*t3784 + 0.167004*(t3543*t3558 + t3763*t4020) + t4020*t4482 + t4792*t4968 + t5090*t5167 + t5369*t5401 + t5466*t5512 + t5623*t5655 + t5710*t5824 - 1.400132*(t5606*t5655 + t5539*t5824) + 0.043805*(t5539*t5655 - 1.*t5606*t5824) + var1(0);
  p_output1(1)=0. + t1260*t3122*t3390 + t2778*t5962 + t3784*t5987 + t4482*t6020 + 0.167004*(t3558*t5987 + t3763*t6020) + t4968*t6099 + t5167*t6125 + t5369*t6187 + t5466*t6222 + t5623*t6313 + t5710*t6356 - 1.400132*(t5606*t6313 + t5539*t6356) + 0.043805*(t5539*t6313 - 1.*t5606*t6356) + var1(1);
  p_output1(2)=0. + t1628*t2778*t3122 - 1.*t1549*t3390 + t3122*t4482*t456 + t3784*t6491 + 0.167004*(t3122*t3763*t456 + t3558*t6491) + t4968*t6543 + t5167*t6556 + t5369*t6615 + t5466*t6639 + t5623*t6661 + t5710*t6720 - 1.400132*(t5606*t6661 + t5539*t6720) + 0.043805*(t5539*t6661 - 1.*t5606*t6720) + var1(2);
}


       
void p_LeftFootBack(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
