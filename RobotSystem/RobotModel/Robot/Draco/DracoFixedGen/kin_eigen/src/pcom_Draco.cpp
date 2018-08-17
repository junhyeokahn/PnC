/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:29:53 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pcom_Draco.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t42;
  double t135;
  double t86;
  double t102;
  double t107;
  double t179;
  double t199;
  double t224;
  double t195;
  double t201;
  double t208;
  double t222;
  double t239;
  double t243;
  double t253;
  double t262;
  double t268;
  double t281;
  double t287;
  double t282;
  double t290;
  double t291;
  double t293;
  double t308;
  double t312;
  double t314;
  double t315;
  double t317;
  double t320;
  double t326;
  double t331;
  double t333;
  double t337;
  double t353;
  double t357;
  double t384;
  double t392;
  double t391;
  double t395;
  double t396;
  double t409;
  double t413;
  double t414;
  double t422;
  double t424;
  double t462;
  double t475;
  double t482;
  double t446;
  double t450;
  double t459;
  double t497;
  double t518;
  double t519;
  double t530;
  double t136;
  double t137;
  double t138;
  double t569;
  double t584;
  double t600;
  double t581;
  double t595;
  double t597;
  double t599;
  double t604;
  double t606;
  double t607;
  double t631;
  double t635;
  double t637;
  double t649;
  double t648;
  double t650;
  double t652;
  double t655;
  double t656;
  double t658;
  double t659;
  double t667;
  double t675;
  double t681;
  double t682;
  double t686;
  double t687;
  double t691;
  double t702;
  double t704;
  double t706;
  double t716;
  double t713;
  double t721;
  double t723;
  double t728;
  double t729;
  double t732;
  double t733;
  double t734;
  double t735;
  double t737;
  double t745;
  double t760;
  double t761;
  double t763;
  double t765;
  double t747;
  double t751;
  double t752;
  double t830;
  double t849;
  double t862;
  double t864;
  double t866;
  double t868;
  double t870;
  double t875;
  double t877;
  double t880;
  double t881;
  double t898;
  double t900;
  double t926;
  double t933;
  double t934;
  double t522;
  double t533;
  double t534;
  double t901;
  double t915;
  double t919;
  double t537;
  double t539;
  double t540;
  double t836;
  double t977;
  double t985;
  double t987;
  double t994;
  double t996;
  double t999;
  double t1001;
  double t1013;
  double t1014;
  double t1016;
  double t1025;
  double t1028;
  double t764;
  double t769;
  double t770;
  double t1029;
  double t1031;
  double t1032;
  double t776;
  double t777;
  double t779;
  double t1035;
  double t1036;
  double t1037;
  double t1088;
  double t1116;
  double t1125;
  double t1127;
  double t1138;
  double t1139;
  double t1144;
  double t1131;
  double t1134;
  double t1135;
  double t1183;
  double t1202;
  double t1214;
  double t1223;
  double t1225;
  double t1226;
  double t1227;
  double t1237;
  double t1238;
  double t1241;
  t42 = Cos(var1[0]);
  t135 = Cos(var1[5]);
  t86 = -1.*t42;
  t102 = 1. + t86;
  t107 = -0.066675*t102;
  t179 = Sin(var1[0]);
  t199 = Cos(var1[1]);
  t224 = Sin(var1[1]);
  t195 = 0.261012*t179;
  t201 = -1.*t199;
  t208 = 1. + t201;
  t222 = 0.331012*t208;
  t239 = -0.90524*t224;
  t243 = t222 + t239;
  t253 = -1.*t179*t243;
  t262 = Cos(var1[2]);
  t268 = -1.*t262;
  t281 = 1. + t268;
  t287 = Sin(var1[2]);
  t282 = -0.97024*t281;
  t290 = -0.066675*t287;
  t291 = t282 + t290;
  t293 = t179*t224*t291;
  t308 = -0.066675*t281;
  t312 = 0.97024*t287;
  t314 = t308 + t312;
  t315 = t42*t314;
  t317 = t262*t179*t224;
  t320 = t42*t287;
  t326 = t317 + t320;
  t331 = t42*t262;
  t333 = -1.*t179*t224*t287;
  t337 = t331 + t333;
  t353 = Cos(var1[3]);
  t357 = -1.*t353;
  t384 = 1. + t357;
  t392 = Sin(var1[3]);
  t391 = -1.45024*t384;
  t395 = -0.066675*t392;
  t396 = t391 + t395;
  t409 = t326*t396;
  t413 = -0.066675*t384;
  t414 = 1.45024*t392;
  t422 = t413 + t414;
  t424 = t337*t422;
  t462 = t353*t326;
  t475 = t337*t392;
  t482 = t462 + t475;
  t446 = t353*t337;
  t450 = -1.*t326*t392;
  t459 = t446 + t450;
  t497 = Cos(var1[4]);
  t518 = -1.*t497;
  t519 = 1. + t518;
  t530 = Sin(var1[4]);
  t136 = -1.*t135;
  t137 = 1. + t136;
  t138 = -0.066675*t137;
  t569 = Sin(var1[5]);
  t584 = Cos(var1[6]);
  t600 = Sin(var1[6]);
  t581 = -0.260988*t569;
  t595 = -1.*t584;
  t597 = 1. + t595;
  t599 = -0.330988*t597;
  t604 = -0.90524*t600;
  t606 = t599 + t604;
  t607 = -1.*t569*t606;
  t631 = Cos(var1[7]);
  t635 = -1.*t631;
  t637 = 1. + t635;
  t649 = Sin(var1[7]);
  t648 = -0.97024*t637;
  t650 = -0.066675*t649;
  t652 = t648 + t650;
  t655 = t569*t600*t652;
  t656 = -0.066675*t637;
  t658 = 0.97024*t649;
  t659 = t656 + t658;
  t667 = t135*t659;
  t675 = t631*t569*t600;
  t681 = t135*t649;
  t682 = t675 + t681;
  t686 = t135*t631;
  t687 = -1.*t569*t600*t649;
  t691 = t686 + t687;
  t702 = Cos(var1[8]);
  t704 = -1.*t702;
  t706 = 1. + t704;
  t716 = Sin(var1[8]);
  t713 = -1.45024*t706;
  t721 = -0.066675*t716;
  t723 = t713 + t721;
  t728 = t682*t723;
  t729 = -0.066675*t706;
  t732 = 1.45024*t716;
  t733 = t729 + t732;
  t734 = t691*t733;
  t735 = t702*t691;
  t737 = -1.*t682*t716;
  t745 = t735 + t737;
  t760 = Cos(var1[9]);
  t761 = -1.*t760;
  t763 = 1. + t761;
  t765 = Sin(var1[9]);
  t747 = t702*t682;
  t751 = t691*t716;
  t752 = t747 + t751;
  t830 = 0.261012*t102;
  t849 = t42*t243;
  t862 = 0.066675*t179;
  t864 = -1.*t42*t224*t291;
  t866 = t179*t314;
  t868 = -1.*t42*t262*t224;
  t870 = t179*t287;
  t875 = t868 + t870;
  t877 = t262*t179;
  t880 = t42*t224*t287;
  t881 = t877 + t880;
  t898 = t875*t396;
  t900 = t881*t422;
  t926 = t353*t875;
  t933 = t881*t392;
  t934 = t926 + t933;
  t522 = -1.93024*t519;
  t533 = -0.065597*t530;
  t534 = t522 + t533;
  t901 = t353*t881;
  t915 = -1.*t875*t392;
  t919 = t901 + t915;
  t537 = -0.065597*t519;
  t539 = 1.93024*t530;
  t540 = t537 + t539;
  t836 = -0.260988*t137;
  t977 = t135*t606;
  t985 = 0.066675*t569;
  t987 = -1.*t135*t600*t652;
  t994 = t569*t659;
  t996 = -1.*t135*t631*t600;
  t999 = t569*t649;
  t1001 = t996 + t999;
  t1013 = t631*t569;
  t1014 = t135*t600*t649;
  t1016 = t1013 + t1014;
  t1025 = t1001*t723;
  t1028 = t1016*t733;
  t764 = -0.065597*t763;
  t769 = 1.93024*t765;
  t770 = t764 + t769;
  t1029 = t702*t1016;
  t1031 = -1.*t1001*t716;
  t1032 = t1029 + t1031;
  t776 = -1.93024*t763;
  t777 = -0.065597*t765;
  t779 = t776 + t777;
  t1035 = t702*t1001;
  t1036 = t1016*t716;
  t1037 = t1035 + t1036;
  t1088 = -0.90524*t208;
  t1116 = t199*t291;
  t1125 = t199*t262*t396;
  t1127 = -1.*t199*t287*t422;
  t1138 = t199*t262*t353;
  t1139 = -1.*t199*t287*t392;
  t1144 = t1138 + t1139;
  t1131 = -1.*t199*t353*t287;
  t1134 = -1.*t199*t262*t392;
  t1135 = t1131 + t1134;
  t1183 = -0.90524*t597;
  t1202 = t584*t652;
  t1214 = t584*t631*t723;
  t1223 = -1.*t584*t649*t733;
  t1225 = -1.*t584*t702*t649;
  t1226 = -1.*t584*t631*t716;
  t1227 = t1225 + t1226;
  t1237 = t584*t631*t702;
  t1238 = -1.*t584*t649*t716;
  t1241 = t1237 + t1238;

  p_output1(0)=0.0423728813559322*(t107 - 0.065344*t135 + t138 - 0.076726*t179 + 6.*(t107 + t195 - 0.347662*t179*t199 + t253 + t293 + t315 - 1.210348*t326 - 0.065782*t337) + 0.5*(t107 + t195 - 0.351673*t179*t199 - 0.930969*t179*t224 + t253 - 0.063284*t42) - 0.068006*t42 + 3.8*(t107 + t195 - 0.340954*t179*t199 + t253 + t293 + t315 + t409 + t424 - 0.082744*t459 - 1.698062*t482) + 0.5*(t107 + t195 - 0.341007*t179*t199 + t253 + t293 + t315 + t409 + t424 - 1.919616*(t482*t497 + t459*t530) - 0.021314*(t459*t497 - 1.*t482*t530) + t482*t534 + t459*t540) + 0.076726*t569 + 0.5*(-0.063284*t135 + t138 + t581 + 0.351649*t569*t584 - 0.930969*t569*t600 + t607) + 6.*(t138 + t581 + 0.334338*t569*t584 + t607 + t655 + t667 - 1.210348*t682 - 0.065782*t691) + 3.8*(t138 + t581 + 0.341046*t569*t584 + t607 + t655 + t667 + t728 + t734 - 0.082744*t745 - 1.698062*t752) + 0.5*(t138 + t581 + 0.340993*t569*t584 + t607 + t655 + t667 + t728 + t734 - 1.919616*(t752*t760 + t745*t765) - 0.021314*(t745*t760 - 1.*t752*t765) + t745*t770 + t752*t779));
  p_output1(1)=0.0423728813559322*(-0.337714*t135 - 0.001331*t179 + 0.337738*t42 + 0.001331*t569 + t830 + t836 + 0.5*(0.003391*t179 + 0.351673*t199*t42 + 0.930969*t224*t42 + t830 + t849) + 6.*(0.347662*t199*t42 + t830 + t849 + t862 + t864 + t866 - 1.210348*t875 - 0.065782*t881) + 3.8*(0.340954*t199*t42 + t830 + t849 + t862 + t864 + t866 + t898 + t900 - 0.082744*t919 - 1.698062*t934) + 0.5*(0.341007*t199*t42 + t830 + t849 + t862 + t864 + t866 + t898 + t900 + t540*t919 + t534*t934 - 1.919616*(t530*t919 + t497*t934) - 0.021314*(t497*t919 - 1.*t530*t934)) + 0.5*(0.003391*t569 - 0.351649*t135*t584 + 0.930969*t135*t600 + t836 + t977) + 3.8*(t1025 + t1028 - 0.082744*t1032 - 1.698062*t1037 - 0.341046*t135*t584 + t836 + t977 + t985 + t987 + t994) + 6.*(-1.210348*t1001 - 0.065782*t1016 - 0.334338*t135*t584 + t836 + t977 + t985 + t987 + t994) + 0.5*(t1025 + t1028 - 0.340993*t135*t584 - 1.919616*(t1037*t760 + t1032*t765) - 0.021314*(t1032*t760 - 1.*t1037*t765) + t1032*t770 + t1037*t779 + t836 + t977 + t985 + t987 + t994));
  p_output1(2)=0.0423728813559322*(-1.704188 + 3.8*(t1088 + t1116 + t1125 + t1127 - 0.082744*t1135 - 1.698062*t1144 + 0.009942*t224) + 0.5*(t1088 - 0.930969*t199 + 0.02066100000000004*t224) + 6.*(t1088 + t1116 + 0.01665*t224 - 1.210348*t199*t262 + 0.065782*t199*t287) + 0.5*(t1088 + t1116 + t1125 + t1127 + 0.009995000000000032*t224 - 1.919616*(t1144*t497 + t1135*t530) - 0.021314*(t1135*t497 - 1.*t1144*t530) + t1144*t534 + t1135*t540) + 0.5*(t1183 - 0.930969*t584 - 0.020660999999999985*t600) + 3.8*(t1183 + t1202 + t1214 + t1223 - 0.082744*t1227 - 1.698062*t1241 - 0.010058*t600) + 6.*(t1183 + t1202 - 0.00335*t600 - 1.210348*t584*t631 + 0.065782*t584*t649) + 0.5*(t1183 + t1202 + t1214 + t1223 - 0.010004999999999986*t600 - 1.919616*(t1241*t760 + t1227*t765) - 0.021314*(t1227*t760 - 1.*t1241*t765) + t1227*t770 + t1241*t779));
}


       
void pcom_Draco(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
