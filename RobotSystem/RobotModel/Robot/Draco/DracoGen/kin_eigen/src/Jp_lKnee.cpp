/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:48 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lKnee.h"

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
  double t331;
  double t573;
  double t630;
  double t648;
  double t713;
  double t151;
  double t160;
  double t289;
  double t477;
  double t522;
  double t528;
  double t546;
  double t1072;
  double t1317;
  double t1339;
  double t1342;
  double t1385;
  double t1238;
  double t1244;
  double t1293;
  double t1518;
  double t1635;
  double t1675;
  double t2157;
  double t2215;
  double t2221;
  double t2244;
  double t2062;
  double t2076;
  double t2079;
  double t2434;
  double t2439;
  double t2442;
  double t3013;
  double t3027;
  double t3081;
  double t3207;
  double t3312;
  double t3332;
  double t3391;
  double t3712;
  double t3748;
  double t3771;
  double t692;
  double t794;
  double t852;
  double t1145;
  double t1176;
  double t1184;
  double t4325;
  double t4515;
  double t4520;
  double t1349;
  double t1473;
  double t1492;
  double t1870;
  double t1916;
  double t1920;
  double t4659;
  double t4754;
  double t4771;
  double t4818;
  double t4851;
  double t5030;
  double t2225;
  double t2336;
  double t2372;
  double t2637;
  double t2749;
  double t2831;
  double t3125;
  double t3309;
  double t3310;
  double t5501;
  double t5632;
  double t5635;
  double t5665;
  double t5900;
  double t5933;
  double t3572;
  double t3611;
  double t3662;
  double t6012;
  double t6015;
  double t6033;
  double t6106;
  double t6114;
  double t6115;
  double t6360;
  double t6361;
  double t6365;
  double t6412;
  double t6414;
  double t6438;
  double t6447;
  double t6449;
  double t6458;
  double t6475;
  double t6485;
  double t6488;
  double t6504;
  double t6514;
  double t6516;
  double t6669;
  double t6672;
  double t6676;
  double t6726;
  double t6730;
  double t6758;
  double t6783;
  double t6797;
  double t6798;
  double t6810;
  double t6821;
  double t6832;
  double t6834;
  double t6837;
  double t6841;
  double t6915;
  double t6930;
  double t6941;
  double t6979;
  double t6982;
  double t6983;
  double t6990;
  double t7005;
  double t7006;
  double t7013;
  double t7019;
  double t7030;
  double t7032;
  double t7036;
  double t7038;
  double t7145;
  double t7148;
  double t7155;
  double t7200;
  double t7204;
  double t7210;
  double t7237;
  double t7241;
  double t7242;
  double t7246;
  double t7247;
  double t7248;
  double t7348;
  double t7351;
  double t7353;
  double t7407;
  double t7411;
  double t7412;
  double t7431;
  double t7432;
  double t7433;
  double t7442;
  double t7443;
  double t7447;
  double t7563;
  double t7567;
  double t7570;
  double t7581;
  double t7586;
  double t7589;
  double t7600;
  double t7614;
  double t7615;
  double t7684;
  double t7693;
  double t7694;
  double t7710;
  double t7711;
  double t7712;
  double t7724;
  double t7729;
  double t7731;
  double t7651;
  double t7652;
  double t7654;
  double t7661;
  double t7664;
  double t7806;
  double t7810;
  double t7813;
  double t7819;
  double t7825;
  double t7837;
  double t7840;
  double t7850;
  double t7857;
  double t7863;
  double t7864;
  double t7890;
  double t7894;
  double t7897;
  double t7935;
  double t7936;
  double t7939;
  double t7970;
  double t7980;
  double t7981;
  double t7997;
  double t8005;
  double t8014;
  double t8028;
  double t8030;
  double t8037;
  double t8121;
  double t8123;
  double t8124;
  double t8081;
  double t8083;
  double t8090;
  double t8093;
  double t8099;
  double t8219;
  double t8221;
  double t8225;
  double t8359;
  double t8366;
  double t8368;
  double t8522;
  double t8523;
  double t8524;
  double t6161;
  double t8230;
  double t8232;
  double t8233;
  double t8463;
  double t8483;
  double t8492;
  double t8506;
  double t8509;
  double t8567;
  double t8586;
  double t8587;
  double t8597;
  double t8648;
  double t8650;
  double t8671;
  double t8673;
  double t8675;
  double t8381;
  double t8395;
  double t8404;
  double t8771;
  double t8778;
  double t8779;
  double t8819;
  double t8824;
  double t8827;
  double t8852;
  double t8853;
  double t8855;
  double t6142;
  double t6185;
  double t8546;
  double t8932;
  double t8941;
  double t8944;
  double t8960;
  double t8966;
  double t9026;
  double t9028;
  double t9040;
  double t8692;
  double t8712;
  double t9072;
  double t9073;
  double t9074;
  double t8871;
  double t8917;
  t331 = Sin(var1[3]);
  t573 = Cos(var1[6]);
  t630 = -1.*t573;
  t648 = 1. + t630;
  t713 = Sin(var1[6]);
  t151 = Cos(var1[3]);
  t160 = Cos(var1[5]);
  t289 = -1.*t151*t160;
  t477 = Sin(var1[4]);
  t522 = Sin(var1[5]);
  t528 = -1.*t331*t477*t522;
  t546 = t289 + t528;
  t1072 = Cos(var1[4]);
  t1317 = Cos(var1[7]);
  t1339 = -1.*t1317;
  t1342 = 1. + t1339;
  t1385 = Sin(var1[7]);
  t1238 = t573*t546;
  t1244 = t1072*t331*t713;
  t1293 = t1238 + t1244;
  t1518 = -1.*t160*t331*t477;
  t1635 = t151*t522;
  t1675 = t1518 + t1635;
  t2157 = Cos(var1[8]);
  t2215 = -1.*t2157;
  t2221 = 1. + t2215;
  t2244 = Sin(var1[8]);
  t2062 = t1317*t1675;
  t2076 = -1.*t1293*t1385;
  t2079 = t2062 + t2076;
  t2434 = -1.*t1072*t573*t331;
  t2439 = t546*t713;
  t2442 = t2434 + t2439;
  t3013 = Cos(var1[9]);
  t3027 = -1.*t3013;
  t3081 = 1. + t3027;
  t3207 = Sin(var1[9]);
  t3312 = t2157*t2079;
  t3332 = t2442*t2244;
  t3391 = t3312 + t3332;
  t3712 = t2157*t2442;
  t3748 = -1.*t2079*t2244;
  t3771 = t3712 + t3748;
  t692 = 0.087004*t648;
  t794 = 0.022225*t713;
  t852 = 0. + t692 + t794;
  t1145 = -0.022225*t648;
  t1176 = 0.087004*t713;
  t1184 = 0. + t1145 + t1176;
  t4325 = -1.*t160*t331;
  t4515 = t151*t477*t522;
  t4520 = t4325 + t4515;
  t1349 = 0.157004*t1342;
  t1473 = -0.31508*t1385;
  t1492 = 0. + t1349 + t1473;
  t1870 = -0.31508*t1342;
  t1916 = -0.157004*t1385;
  t1920 = 0. + t1870 + t1916;
  t4659 = t573*t4520;
  t4754 = -1.*t151*t1072*t713;
  t4771 = t4659 + t4754;
  t4818 = t151*t160*t477;
  t4851 = t331*t522;
  t5030 = t4818 + t4851;
  t2225 = -0.38008*t2221;
  t2336 = -0.022225*t2244;
  t2372 = 0. + t2225 + t2336;
  t2637 = -0.022225*t2221;
  t2749 = 0.38008*t2244;
  t2831 = 0. + t2637 + t2749;
  t3125 = -0.86008*t3081;
  t3309 = -0.022225*t3207;
  t3310 = 0. + t3125 + t3309;
  t5501 = t1317*t5030;
  t5632 = -1.*t4771*t1385;
  t5635 = t5501 + t5632;
  t5665 = t151*t1072*t573;
  t5900 = t4520*t713;
  t5933 = t5665 + t5900;
  t3572 = -0.022225*t3081;
  t3611 = 0.86008*t3207;
  t3662 = 0. + t3572 + t3611;
  t6012 = t2157*t5635;
  t6015 = t5933*t2244;
  t6033 = t6012 + t6015;
  t6106 = t2157*t5933;
  t6114 = -1.*t5635*t2244;
  t6115 = t6106 + t6114;
  t6360 = t151*t1072*t573*t522;
  t6361 = t151*t477*t713;
  t6365 = t6360 + t6361;
  t6412 = t151*t1072*t160*t1317;
  t6414 = -1.*t6365*t1385;
  t6438 = t6412 + t6414;
  t6447 = -1.*t151*t573*t477;
  t6449 = t151*t1072*t522*t713;
  t6458 = t6447 + t6449;
  t6475 = t2157*t6438;
  t6485 = t6458*t2244;
  t6488 = t6475 + t6485;
  t6504 = t2157*t6458;
  t6514 = -1.*t6438*t2244;
  t6516 = t6504 + t6514;
  t6669 = t1072*t573*t331*t522;
  t6672 = t331*t477*t713;
  t6676 = t6669 + t6672;
  t6726 = t1072*t160*t1317*t331;
  t6730 = -1.*t6676*t1385;
  t6758 = t6726 + t6730;
  t6783 = -1.*t573*t331*t477;
  t6797 = t1072*t331*t522*t713;
  t6798 = t6783 + t6797;
  t6810 = t2157*t6758;
  t6821 = t6798*t2244;
  t6832 = t6810 + t6821;
  t6834 = t2157*t6798;
  t6837 = -1.*t6758*t2244;
  t6841 = t6834 + t6837;
  t6915 = -1.*t573*t477*t522;
  t6930 = t1072*t713;
  t6941 = t6915 + t6930;
  t6979 = -1.*t160*t1317*t477;
  t6982 = -1.*t6941*t1385;
  t6983 = t6979 + t6982;
  t6990 = -1.*t1072*t573;
  t7005 = -1.*t477*t522*t713;
  t7006 = t6990 + t7005;
  t7013 = t2157*t6983;
  t7019 = t7006*t2244;
  t7030 = t7013 + t7019;
  t7032 = t2157*t7006;
  t7036 = -1.*t6983*t2244;
  t7038 = t7032 + t7036;
  t7145 = t160*t331;
  t7148 = -1.*t151*t477*t522;
  t7155 = t7145 + t7148;
  t7200 = t1317*t7155;
  t7204 = -1.*t573*t5030*t1385;
  t7210 = t7200 + t7204;
  t7237 = t2157*t7210;
  t7241 = t5030*t713*t2244;
  t7242 = t7237 + t7241;
  t7246 = t2157*t5030*t713;
  t7247 = -1.*t7210*t2244;
  t7248 = t7246 + t7247;
  t7348 = t160*t331*t477;
  t7351 = -1.*t151*t522;
  t7353 = t7348 + t7351;
  t7407 = t1317*t546;
  t7411 = -1.*t573*t7353*t1385;
  t7412 = t7407 + t7411;
  t7431 = t2157*t7412;
  t7432 = t7353*t713*t2244;
  t7433 = t7431 + t7432;
  t7442 = t2157*t7353*t713;
  t7443 = -1.*t7412*t2244;
  t7447 = t7442 + t7443;
  t7563 = -1.*t1072*t1317*t522;
  t7567 = -1.*t1072*t160*t573*t1385;
  t7570 = t7563 + t7567;
  t7581 = t2157*t7570;
  t7586 = t1072*t160*t713*t2244;
  t7589 = t7581 + t7586;
  t7600 = t1072*t160*t2157*t713;
  t7614 = -1.*t7570*t2244;
  t7615 = t7600 + t7614;
  t7684 = -1.*t151*t1072*t573;
  t7693 = -1.*t4520*t713;
  t7694 = t7684 + t7693;
  t7710 = -1.*t2157*t7694*t1385;
  t7711 = t4771*t2244;
  t7712 = t7710 + t7711;
  t7724 = t2157*t4771;
  t7729 = t7694*t1385*t2244;
  t7731 = t7724 + t7729;
  t7651 = 0.087004*t573;
  t7652 = -0.022225*t713;
  t7654 = t7651 + t7652;
  t7661 = 0.022225*t573;
  t7664 = t7661 + t1176;
  t7806 = t151*t160;
  t7810 = t331*t477*t522;
  t7813 = t7806 + t7810;
  t7819 = -1.*t7813*t713;
  t7825 = t2434 + t7819;
  t7837 = t573*t7813;
  t7840 = -1.*t1072*t331*t713;
  t7850 = t7837 + t7840;
  t7857 = -1.*t2157*t7825*t1385;
  t7863 = t7850*t2244;
  t7864 = t7857 + t7863;
  t7890 = t2157*t7850;
  t7894 = t7825*t1385*t2244;
  t7897 = t7890 + t7894;
  t7935 = t573*t477;
  t7936 = -1.*t1072*t522*t713;
  t7939 = t7935 + t7936;
  t7970 = t1072*t573*t522;
  t7980 = t477*t713;
  t7981 = t7970 + t7980;
  t7997 = -1.*t2157*t7939*t1385;
  t8005 = t7981*t2244;
  t8014 = t7997 + t8005;
  t8028 = t2157*t7981;
  t8030 = t7939*t1385*t2244;
  t8037 = t8028 + t8030;
  t8121 = -1.*t1317*t4771;
  t8123 = -1.*t5030*t1385;
  t8124 = t8121 + t8123;
  t8081 = -0.157004*t1317;
  t8083 = t8081 + t1473;
  t8090 = -0.31508*t1317;
  t8093 = 0.157004*t1385;
  t8099 = t8090 + t8093;
  t8219 = -1.*t1317*t7850;
  t8221 = -1.*t7353*t1385;
  t8225 = t8219 + t8221;
  t8359 = -1.*t1317*t7981;
  t8366 = -1.*t1072*t160*t1385;
  t8368 = t8359 + t8366;
  t8522 = -1.*t2157*t5635;
  t8523 = -1.*t5933*t2244;
  t8524 = t8522 + t8523;
  t6161 = t3013*t6115;
  t8230 = t1317*t7353;
  t8232 = -1.*t7850*t1385;
  t8233 = t8230 + t8232;
  t8463 = -0.022225*t2157;
  t8483 = -0.38008*t2244;
  t8492 = t8463 + t8483;
  t8506 = 0.38008*t2157;
  t8509 = t8506 + t2336;
  t8567 = t1072*t573*t331;
  t8586 = t7813*t713;
  t8587 = t8567 + t8586;
  t8597 = -1.*t2157*t8233;
  t8648 = -1.*t8587*t2244;
  t8650 = t8597 + t8648;
  t8671 = t2157*t8587;
  t8673 = -1.*t8233*t2244;
  t8675 = t8671 + t8673;
  t8381 = t1072*t160*t1317;
  t8395 = -1.*t7981*t1385;
  t8404 = t8381 + t8395;
  t8771 = -1.*t573*t477;
  t8778 = t1072*t522*t713;
  t8779 = t8771 + t8778;
  t8819 = -1.*t2157*t8404;
  t8824 = -1.*t8779*t2244;
  t8827 = t8819 + t8824;
  t8852 = t2157*t8779;
  t8853 = -1.*t8404*t2244;
  t8855 = t8852 + t8853;
  t6142 = -1.*t3207*t6033;
  t6185 = t6142 + t6161;
  t8546 = -1.*t3207*t6115;
  t8932 = -0.022225*t3013;
  t8941 = -0.86008*t3207;
  t8944 = t8932 + t8941;
  t8960 = 0.86008*t3013;
  t8966 = t8960 + t3309;
  t9026 = t2157*t8233;
  t9028 = t8587*t2244;
  t9040 = t9026 + t9028;
  t8692 = t3013*t8675;
  t8712 = -1.*t3207*t8675;
  t9072 = t2157*t8404;
  t9073 = t8779*t2244;
  t9074 = t9072 + t9073;
  t8871 = t3013*t8855;
  t8917 = -1.*t3207*t8855;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1293*t1492 + 0.150254*(t1293*t1317 + t1385*t1675) + t1675*t1920 + t2079*t2372 + t2442*t2831 - 1.*t1072*t1184*t331 + t3310*t3391 + t3662*t3771 - 0.022225*(-1.*t3207*t3391 + t3013*t3771) - 0.86008*(t3013*t3391 + t3207*t3771) + t546*t852;
  p_output1(10)=t1072*t1184*t151 + t1492*t4771 + t1920*t5030 + 0.150254*(t1317*t4771 + t1385*t5030) + t2372*t5635 + t2831*t5933 + t3310*t6033 + t3662*t6115 - 0.86008*(t3013*t6033 + t3207*t6115) - 0.022225*t6185 + t4520*t852;
  p_output1(11)=0;
  p_output1(12)=t1072*t151*t160*t1920 - 1.*t1184*t151*t477 + t1492*t6365 + 0.150254*(t1072*t1385*t151*t160 + t1317*t6365) + t2372*t6438 + t2831*t6458 + t3310*t6488 + t3662*t6516 - 0.022225*(-1.*t3207*t6488 + t3013*t6516) - 0.86008*(t3013*t6488 + t3207*t6516) + t1072*t151*t522*t852;
  p_output1(13)=t1072*t160*t1920*t331 - 1.*t1184*t331*t477 + t1492*t6676 + 0.150254*(t1072*t1385*t160*t331 + t1317*t6676) + t2372*t6758 + t2831*t6798 + t3310*t6832 + t3662*t6841 - 0.022225*(-1.*t3207*t6832 + t3013*t6841) - 0.86008*(t3013*t6832 + t3207*t6841) + t1072*t331*t522*t852;
  p_output1(14)=-1.*t1072*t1184 - 1.*t160*t1920*t477 + t1492*t6941 + 0.150254*(-1.*t1385*t160*t477 + t1317*t6941) + t2372*t6983 + t2831*t7006 + t3310*t7030 + t3662*t7038 - 0.022225*(-1.*t3207*t7030 + t3013*t7038) - 0.86008*(t3013*t7030 + t3207*t7038) - 1.*t477*t522*t852;
  p_output1(15)=t1492*t5030*t573 + t2831*t5030*t713 + t1920*t7155 + 0.150254*(t1317*t5030*t573 + t1385*t7155) + t2372*t7210 + t3310*t7242 + t3662*t7248 - 0.022225*(-1.*t3207*t7242 + t3013*t7248) - 0.86008*(t3013*t7242 + t3207*t7248) + t5030*t852;
  p_output1(16)=t1920*t546 + t1492*t573*t7353 + t2831*t713*t7353 + 0.150254*(t1385*t546 + t1317*t573*t7353) + t2372*t7412 + t3310*t7433 + t3662*t7447 - 0.022225*(-1.*t3207*t7433 + t3013*t7447) - 0.86008*(t3013*t7433 + t3207*t7447) + t7353*t852;
  p_output1(17)=-1.*t1072*t1920*t522 + t1072*t1492*t160*t573 + 0.150254*(-1.*t1072*t1385*t522 + t1072*t1317*t160*t573) + t1072*t160*t2831*t713 + t2372*t7570 + t3310*t7589 + t3662*t7615 - 0.022225*(-1.*t3207*t7589 + t3013*t7615) - 0.86008*(t3013*t7589 + t3207*t7615) + t1072*t160*t852;
  p_output1(18)=t2831*t4771 + t1072*t151*t7654 + t4520*t7664 + 0.150254*t1317*t7694 + t1492*t7694 - 1.*t1385*t2372*t7694 + t3310*t7712 + t3662*t7731 - 0.022225*(-1.*t3207*t7712 + t3013*t7731) - 0.86008*(t3013*t7712 + t3207*t7731);
  p_output1(19)=t1072*t331*t7654 + t7664*t7813 + 0.150254*t1317*t7825 + t1492*t7825 - 1.*t1385*t2372*t7825 + t2831*t7850 + t3310*t7864 + t3662*t7897 - 0.022225*(-1.*t3207*t7864 + t3013*t7897) - 0.86008*(t3013*t7864 + t3207*t7897);
  p_output1(20)=-1.*t477*t7654 + t1072*t522*t7664 + 0.150254*t1317*t7939 + t1492*t7939 - 1.*t1385*t2372*t7939 + t2831*t7981 + t3310*t8014 + t3662*t8037 - 0.022225*(-1.*t3207*t8014 + t3013*t8037) - 0.86008*(t3013*t8014 + t3207*t8037);
  p_output1(21)=0.150254*t5635 + t5030*t8083 + t4771*t8099 + t2372*t8124 + t2157*t3310*t8124 - 1.*t2244*t3662*t8124 - 0.022225*(-1.*t2244*t3013*t8124 - 1.*t2157*t3207*t8124) - 0.86008*(t2157*t3013*t8124 - 1.*t2244*t3207*t8124);
  p_output1(22)=t7353*t8083 + t7850*t8099 + t2372*t8225 + t2157*t3310*t8225 - 1.*t2244*t3662*t8225 - 0.022225*(-1.*t2244*t3013*t8225 - 1.*t2157*t3207*t8225) - 0.86008*(t2157*t3013*t8225 - 1.*t2244*t3207*t8225) + 0.150254*t8233;
  p_output1(23)=t1072*t160*t8083 + t7981*t8099 + t2372*t8368 + t2157*t3310*t8368 - 1.*t2244*t3662*t8368 - 0.022225*(-1.*t2244*t3013*t8368 - 1.*t2157*t3207*t8368) - 0.86008*(t2157*t3013*t8368 - 1.*t2244*t3207*t8368) + 0.150254*t8404;
  p_output1(24)=t3310*t6115 + t5635*t8492 + t5933*t8509 + t3662*t8524 - 0.86008*(t6161 + t3207*t8524) - 0.022225*(t3013*t8524 + t8546);
  p_output1(25)=t8233*t8492 + t8509*t8587 + t3662*t8650 + t3310*t8675 - 0.86008*(t3207*t8650 + t8692) - 0.022225*(t3013*t8650 + t8712);
  p_output1(26)=t8404*t8492 + t8509*t8779 + t3662*t8827 + t3310*t8855 - 0.86008*(t3207*t8827 + t8871) - 0.022225*(t3013*t8827 + t8917);
  p_output1(27)=-0.86008*t6185 - 0.022225*(-1.*t3013*t6033 + t8546) + t6033*t8944 + t6115*t8966;
  p_output1(28)=t8675*t8966 + t8944*t9040 - 0.022225*(t8712 - 1.*t3013*t9040) - 0.86008*(t8692 - 1.*t3207*t9040);
  p_output1(29)=t8855*t8966 + t8944*t9074 - 0.022225*(t8917 - 1.*t3013*t9074) - 0.86008*(t8871 - 1.*t3207*t9074);
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


       
void Jp_lKnee(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
