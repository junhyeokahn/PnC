/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:05 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_LeftFootFront.h"

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
  double t326;
  double t752;
  double t801;
  double t818;
  double t870;
  double t62;
  double t109;
  double t175;
  double t498;
  double t501;
  double t503;
  double t599;
  double t1011;
  double t1327;
  double t1328;
  double t1357;
  double t1444;
  double t1260;
  double t1261;
  double t1297;
  double t1565;
  double t1571;
  double t1572;
  double t1758;
  double t1765;
  double t1766;
  double t1926;
  double t1720;
  double t1730;
  double t1738;
  double t2131;
  double t2159;
  double t2171;
  double t2243;
  double t2259;
  double t2313;
  double t2366;
  double t2435;
  double t2441;
  double t2457;
  double t2812;
  double t2842;
  double t2887;
  double t2993;
  double t3038;
  double t3042;
  double t3144;
  double t3255;
  double t3263;
  double t3267;
  double t3515;
  double t3616;
  double t3647;
  double t859;
  double t944;
  double t974;
  double t1027;
  double t1064;
  double t1195;
  double t3868;
  double t3909;
  double t3923;
  double t1409;
  double t1466;
  double t1526;
  double t1640;
  double t1647;
  double t1649;
  double t4003;
  double t4008;
  double t4090;
  double t4258;
  double t4270;
  double t4402;
  double t1859;
  double t1935;
  double t2054;
  double t2181;
  double t2212;
  double t2220;
  double t2323;
  double t2403;
  double t2404;
  double t4569;
  double t4642;
  double t4666;
  double t4738;
  double t4773;
  double t4787;
  double t2622;
  double t2737;
  double t2744;
  double t3139;
  double t3169;
  double t3220;
  double t4793;
  double t4828;
  double t4862;
  double t4868;
  double t4872;
  double t4879;
  double t3449;
  double t3472;
  double t3504;
  double t4921;
  double t4922;
  double t4935;
  double t5015;
  double t5022;
  double t5073;
  double t5308;
  double t5311;
  double t5335;
  double t5464;
  double t5476;
  double t5540;
  double t5577;
  double t5620;
  double t5623;
  double t5653;
  double t5655;
  double t5657;
  double t5700;
  double t5716;
  double t5733;
  double t5786;
  double t5824;
  double t5827;
  double t5856;
  double t5866;
  double t5870;
  double t6003;
  double t6005;
  double t6006;
  double t6099;
  double t6109;
  double t6120;
  double t6128;
  double t6135;
  double t6143;
  double t6151;
  double t6153;
  double t6183;
  double t6196;
  double t6207;
  double t6222;
  double t6235;
  double t6242;
  double t6257;
  double t6313;
  double t6324;
  double t6331;
  double t6518;
  double t6522;
  double t6524;
  double t6559;
  double t6561;
  double t6572;
  double t6582;
  double t6596;
  double t6615;
  double t6620;
  double t6622;
  double t6634;
  double t6647;
  double t6653;
  double t6657;
  double t6670;
  double t6703;
  double t6704;
  double t6739;
  double t6741;
  double t6742;
  double t6815;
  double t6816;
  double t6821;
  double t6892;
  double t6917;
  double t6918;
  double t6938;
  double t6940;
  double t6942;
  double t6962;
  double t6967;
  double t6976;
  double t7012;
  double t7025;
  double t7041;
  double t7053;
  double t7116;
  double t7146;
  double t7238;
  double t7264;
  double t7283;
  double t7338;
  double t7343;
  double t7344;
  double t7371;
  double t7373;
  double t7378;
  double t7390;
  double t7392;
  double t7405;
  double t7415;
  double t7419;
  double t7422;
  double t7439;
  double t7444;
  double t7456;
  double t7565;
  double t7566;
  double t7568;
  double t7584;
  double t7592;
  double t7595;
  double t7612;
  double t7621;
  double t7622;
  double t7652;
  double t7657;
  double t7671;
  double t7680;
  double t7683;
  double t7685;
  double t7836;
  double t7845;
  double t7847;
  double t7879;
  double t7881;
  double t7895;
  double t7913;
  double t7932;
  double t7934;
  double t7951;
  double t7953;
  double t7983;
  double t7995;
  double t8000;
  double t8007;
  double t7789;
  double t7798;
  double t7802;
  double t7818;
  double t7824;
  double t8066;
  double t8067;
  double t8071;
  double t8077;
  double t8078;
  double t8101;
  double t8102;
  double t8103;
  double t8127;
  double t8128;
  double t8135;
  double t8156;
  double t8159;
  double t8170;
  double t8175;
  double t8183;
  double t8194;
  double t8213;
  double t8214;
  double t8231;
  double t8306;
  double t8308;
  double t8311;
  double t8326;
  double t8333;
  double t8338;
  double t8352;
  double t8366;
  double t8374;
  double t8381;
  double t8383;
  double t8391;
  double t8394;
  double t8397;
  double t8402;
  double t8414;
  double t8415;
  double t8420;
  double t8506;
  double t8509;
  double t8518;
  double t8549;
  double t8551;
  double t8569;
  double t8574;
  double t8577;
  double t8589;
  double t8471;
  double t8477;
  double t8488;
  double t8494;
  double t8495;
  double t8679;
  double t8681;
  double t8684;
  double t8719;
  double t8723;
  double t8725;
  double t8747;
  double t8750;
  double t8753;
  double t8836;
  double t8838;
  double t8839;
  double t8875;
  double t8886;
  double t8916;
  double t8924;
  double t8936;
  double t8940;
  double t9052;
  double t9056;
  double t9057;
  double t9094;
  double t9103;
  double t9112;
  double t9116;
  double t9119;
  double t8699;
  double t8704;
  double t8706;
  double t9036;
  double t9042;
  double t9043;
  double t9046;
  double t9047;
  double t9183;
  double t9191;
  double t9197;
  double t9201;
  double t9202;
  double t9211;
  double t9222;
  double t9223;
  double t9229;
  double t9236;
  double t9244;
  double t9245;
  double t9271;
  double t9275;
  double t9283;
  double t8844;
  double t8845;
  double t8847;
  double t9362;
  double t9371;
  double t9379;
  double t9389;
  double t9390;
  double t9414;
  double t9434;
  double t9448;
  double t9458;
  double t9479;
  double t9486;
  double t9493;
  double t9501;
  double t9505;
  double t9506;
  double t9625;
  double t9638;
  double t5171;
  double t9534;
  double t9536;
  double t9542;
  double t9555;
  double t9585;
  double t9715;
  double t9719;
  double t9720;
  double t9738;
  double t9742;
  double t9746;
  double t9752;
  double t9799;
  double t9803;
  double t9805;
  double t9812;
  double t9816;
  double t9821;
  double t9824;
  double t9649;
  double t5179;
  double t5191;
  double t9842;
  double t9846;
  double t9849;
  double t9851;
  double t9852;
  double t9761;
  double t9870;
  double t9873;
  double t9874;
  double t9789;
  double t9826;
  double t9890;
  double t9892;
  double t9893;
  double t9832;
  t326 = Sin(var1[3]);
  t752 = Cos(var1[6]);
  t801 = -1.*t752;
  t818 = 1. + t801;
  t870 = Sin(var1[6]);
  t62 = Cos(var1[3]);
  t109 = Cos(var1[5]);
  t175 = -1.*t62*t109;
  t498 = Sin(var1[4]);
  t501 = Sin(var1[5]);
  t503 = -1.*t326*t498*t501;
  t599 = t175 + t503;
  t1011 = Cos(var1[4]);
  t1327 = Cos(var1[7]);
  t1328 = -1.*t1327;
  t1357 = 1. + t1328;
  t1444 = Sin(var1[7]);
  t1260 = t752*t599;
  t1261 = t1011*t326*t870;
  t1297 = t1260 + t1261;
  t1565 = -1.*t109*t326*t498;
  t1571 = t62*t501;
  t1572 = t1565 + t1571;
  t1758 = Cos(var1[8]);
  t1765 = -1.*t1758;
  t1766 = 1. + t1765;
  t1926 = Sin(var1[8]);
  t1720 = t1327*t1572;
  t1730 = -1.*t1297*t1444;
  t1738 = t1720 + t1730;
  t2131 = -1.*t1011*t752*t326;
  t2159 = t599*t870;
  t2171 = t2131 + t2159;
  t2243 = Cos(var1[9]);
  t2259 = -1.*t2243;
  t2313 = 1. + t2259;
  t2366 = Sin(var1[9]);
  t2435 = t1758*t1738;
  t2441 = t2171*t1926;
  t2457 = t2435 + t2441;
  t2812 = t1758*t2171;
  t2842 = -1.*t1738*t1926;
  t2887 = t2812 + t2842;
  t2993 = Cos(var1[10]);
  t3038 = -1.*t2993;
  t3042 = 1. + t3038;
  t3144 = Sin(var1[10]);
  t3255 = -1.*t2366*t2457;
  t3263 = t2243*t2887;
  t3267 = t3255 + t3263;
  t3515 = t2243*t2457;
  t3616 = t2366*t2887;
  t3647 = t3515 + t3616;
  t859 = 0.087004*t818;
  t944 = 0.022225*t870;
  t974 = 0. + t859 + t944;
  t1027 = -0.022225*t818;
  t1064 = 0.087004*t870;
  t1195 = 0. + t1027 + t1064;
  t3868 = -1.*t109*t326;
  t3909 = t62*t498*t501;
  t3923 = t3868 + t3909;
  t1409 = 0.157004*t1357;
  t1466 = -0.31508*t1444;
  t1526 = 0. + t1409 + t1466;
  t1640 = -0.31508*t1357;
  t1647 = -0.157004*t1444;
  t1649 = 0. + t1640 + t1647;
  t4003 = t752*t3923;
  t4008 = -1.*t62*t1011*t870;
  t4090 = t4003 + t4008;
  t4258 = t62*t109*t498;
  t4270 = t326*t501;
  t4402 = t4258 + t4270;
  t1859 = -0.38008*t1766;
  t1935 = -0.022225*t1926;
  t2054 = 0. + t1859 + t1935;
  t2181 = -0.022225*t1766;
  t2212 = 0.38008*t1926;
  t2220 = 0. + t2181 + t2212;
  t2323 = -0.86008*t2313;
  t2403 = -0.022225*t2366;
  t2404 = 0. + t2323 + t2403;
  t4569 = t1327*t4402;
  t4642 = -1.*t4090*t1444;
  t4666 = t4569 + t4642;
  t4738 = t62*t1011*t752;
  t4773 = t3923*t870;
  t4787 = t4738 + t4773;
  t2622 = -0.022225*t2313;
  t2737 = 0.86008*t2366;
  t2744 = 0. + t2622 + t2737;
  t3139 = -0.021147*t3042;
  t3169 = 1.34008*t3144;
  t3220 = 0. + t3139 + t3169;
  t4793 = t1758*t4666;
  t4828 = t4787*t1926;
  t4862 = t4793 + t4828;
  t4868 = t1758*t4787;
  t4872 = -1.*t4666*t1926;
  t4879 = t4868 + t4872;
  t3449 = -1.34008*t3042;
  t3472 = -0.021147*t3144;
  t3504 = 0. + t3449 + t3472;
  t4921 = -1.*t2366*t4862;
  t4922 = t2243*t4879;
  t4935 = t4921 + t4922;
  t5015 = t2243*t4862;
  t5022 = t2366*t4879;
  t5073 = t5015 + t5022;
  t5308 = t62*t1011*t752*t501;
  t5311 = t62*t498*t870;
  t5335 = t5308 + t5311;
  t5464 = t62*t1011*t109*t1327;
  t5476 = -1.*t5335*t1444;
  t5540 = t5464 + t5476;
  t5577 = -1.*t62*t752*t498;
  t5620 = t62*t1011*t501*t870;
  t5623 = t5577 + t5620;
  t5653 = t1758*t5540;
  t5655 = t5623*t1926;
  t5657 = t5653 + t5655;
  t5700 = t1758*t5623;
  t5716 = -1.*t5540*t1926;
  t5733 = t5700 + t5716;
  t5786 = -1.*t2366*t5657;
  t5824 = t2243*t5733;
  t5827 = t5786 + t5824;
  t5856 = t2243*t5657;
  t5866 = t2366*t5733;
  t5870 = t5856 + t5866;
  t6003 = t1011*t752*t326*t501;
  t6005 = t326*t498*t870;
  t6006 = t6003 + t6005;
  t6099 = t1011*t109*t1327*t326;
  t6109 = -1.*t6006*t1444;
  t6120 = t6099 + t6109;
  t6128 = -1.*t752*t326*t498;
  t6135 = t1011*t326*t501*t870;
  t6143 = t6128 + t6135;
  t6151 = t1758*t6120;
  t6153 = t6143*t1926;
  t6183 = t6151 + t6153;
  t6196 = t1758*t6143;
  t6207 = -1.*t6120*t1926;
  t6222 = t6196 + t6207;
  t6235 = -1.*t2366*t6183;
  t6242 = t2243*t6222;
  t6257 = t6235 + t6242;
  t6313 = t2243*t6183;
  t6324 = t2366*t6222;
  t6331 = t6313 + t6324;
  t6518 = -1.*t752*t498*t501;
  t6522 = t1011*t870;
  t6524 = t6518 + t6522;
  t6559 = -1.*t109*t1327*t498;
  t6561 = -1.*t6524*t1444;
  t6572 = t6559 + t6561;
  t6582 = -1.*t1011*t752;
  t6596 = -1.*t498*t501*t870;
  t6615 = t6582 + t6596;
  t6620 = t1758*t6572;
  t6622 = t6615*t1926;
  t6634 = t6620 + t6622;
  t6647 = t1758*t6615;
  t6653 = -1.*t6572*t1926;
  t6657 = t6647 + t6653;
  t6670 = -1.*t2366*t6634;
  t6703 = t2243*t6657;
  t6704 = t6670 + t6703;
  t6739 = t2243*t6634;
  t6741 = t2366*t6657;
  t6742 = t6739 + t6741;
  t6815 = t109*t326;
  t6816 = -1.*t62*t498*t501;
  t6821 = t6815 + t6816;
  t6892 = t1327*t6821;
  t6917 = -1.*t752*t4402*t1444;
  t6918 = t6892 + t6917;
  t6938 = t1758*t6918;
  t6940 = t4402*t870*t1926;
  t6942 = t6938 + t6940;
  t6962 = t1758*t4402*t870;
  t6967 = -1.*t6918*t1926;
  t6976 = t6962 + t6967;
  t7012 = -1.*t2366*t6942;
  t7025 = t2243*t6976;
  t7041 = t7012 + t7025;
  t7053 = t2243*t6942;
  t7116 = t2366*t6976;
  t7146 = t7053 + t7116;
  t7238 = t109*t326*t498;
  t7264 = -1.*t62*t501;
  t7283 = t7238 + t7264;
  t7338 = t1327*t599;
  t7343 = -1.*t752*t7283*t1444;
  t7344 = t7338 + t7343;
  t7371 = t1758*t7344;
  t7373 = t7283*t870*t1926;
  t7378 = t7371 + t7373;
  t7390 = t1758*t7283*t870;
  t7392 = -1.*t7344*t1926;
  t7405 = t7390 + t7392;
  t7415 = -1.*t2366*t7378;
  t7419 = t2243*t7405;
  t7422 = t7415 + t7419;
  t7439 = t2243*t7378;
  t7444 = t2366*t7405;
  t7456 = t7439 + t7444;
  t7565 = -1.*t1011*t1327*t501;
  t7566 = -1.*t1011*t109*t752*t1444;
  t7568 = t7565 + t7566;
  t7584 = t1758*t7568;
  t7592 = t1011*t109*t870*t1926;
  t7595 = t7584 + t7592;
  t7612 = t1011*t109*t1758*t870;
  t7621 = -1.*t7568*t1926;
  t7622 = t7612 + t7621;
  t7652 = -1.*t2366*t7595;
  t7657 = t2243*t7622;
  t7671 = t7652 + t7657;
  t7680 = t2243*t7595;
  t7683 = t2366*t7622;
  t7685 = t7680 + t7683;
  t7836 = -1.*t62*t1011*t752;
  t7845 = -1.*t3923*t870;
  t7847 = t7836 + t7845;
  t7879 = -1.*t1758*t7847*t1444;
  t7881 = t4090*t1926;
  t7895 = t7879 + t7881;
  t7913 = t1758*t4090;
  t7932 = t7847*t1444*t1926;
  t7934 = t7913 + t7932;
  t7951 = -1.*t2366*t7895;
  t7953 = t2243*t7934;
  t7983 = t7951 + t7953;
  t7995 = t2243*t7895;
  t8000 = t2366*t7934;
  t8007 = t7995 + t8000;
  t7789 = 0.087004*t752;
  t7798 = -0.022225*t870;
  t7802 = t7789 + t7798;
  t7818 = 0.022225*t752;
  t7824 = t7818 + t1064;
  t8066 = t62*t109;
  t8067 = t326*t498*t501;
  t8071 = t8066 + t8067;
  t8077 = -1.*t8071*t870;
  t8078 = t2131 + t8077;
  t8101 = t752*t8071;
  t8102 = -1.*t1011*t326*t870;
  t8103 = t8101 + t8102;
  t8127 = -1.*t1758*t8078*t1444;
  t8128 = t8103*t1926;
  t8135 = t8127 + t8128;
  t8156 = t1758*t8103;
  t8159 = t8078*t1444*t1926;
  t8170 = t8156 + t8159;
  t8175 = -1.*t2366*t8135;
  t8183 = t2243*t8170;
  t8194 = t8175 + t8183;
  t8213 = t2243*t8135;
  t8214 = t2366*t8170;
  t8231 = t8213 + t8214;
  t8306 = t752*t498;
  t8308 = -1.*t1011*t501*t870;
  t8311 = t8306 + t8308;
  t8326 = t1011*t752*t501;
  t8333 = t498*t870;
  t8338 = t8326 + t8333;
  t8352 = -1.*t1758*t8311*t1444;
  t8366 = t8338*t1926;
  t8374 = t8352 + t8366;
  t8381 = t1758*t8338;
  t8383 = t8311*t1444*t1926;
  t8391 = t8381 + t8383;
  t8394 = -1.*t2366*t8374;
  t8397 = t2243*t8391;
  t8402 = t8394 + t8397;
  t8414 = t2243*t8374;
  t8415 = t2366*t8391;
  t8420 = t8414 + t8415;
  t8506 = -1.*t1327*t4090;
  t8509 = -1.*t4402*t1444;
  t8518 = t8506 + t8509;
  t8549 = -1.*t1758*t2366*t8518;
  t8551 = -1.*t2243*t8518*t1926;
  t8569 = t8549 + t8551;
  t8574 = t2243*t1758*t8518;
  t8577 = -1.*t2366*t8518*t1926;
  t8589 = t8574 + t8577;
  t8471 = -0.157004*t1327;
  t8477 = t8471 + t1466;
  t8488 = -0.31508*t1327;
  t8494 = 0.157004*t1444;
  t8495 = t8488 + t8494;
  t8679 = -1.*t1327*t8103;
  t8681 = -1.*t7283*t1444;
  t8684 = t8679 + t8681;
  t8719 = -1.*t1758*t2366*t8684;
  t8723 = -1.*t2243*t8684*t1926;
  t8725 = t8719 + t8723;
  t8747 = t2243*t1758*t8684;
  t8750 = -1.*t2366*t8684*t1926;
  t8753 = t8747 + t8750;
  t8836 = -1.*t1327*t8338;
  t8838 = -1.*t1011*t109*t1444;
  t8839 = t8836 + t8838;
  t8875 = -1.*t1758*t2366*t8839;
  t8886 = -1.*t2243*t8839*t1926;
  t8916 = t8875 + t8886;
  t8924 = t2243*t1758*t8839;
  t8936 = -1.*t2366*t8839*t1926;
  t8940 = t8924 + t8936;
  t9052 = -1.*t1758*t4666;
  t9056 = -1.*t4787*t1926;
  t9057 = t9052 + t9056;
  t9094 = t2366*t9057;
  t9103 = t9094 + t4922;
  t9112 = t2243*t9057;
  t9116 = -1.*t2366*t4879;
  t9119 = t9112 + t9116;
  t8699 = t1327*t7283;
  t8704 = -1.*t8103*t1444;
  t8706 = t8699 + t8704;
  t9036 = -0.022225*t1758;
  t9042 = -0.38008*t1926;
  t9043 = t9036 + t9042;
  t9046 = 0.38008*t1758;
  t9047 = t9046 + t1935;
  t9183 = t1011*t752*t326;
  t9191 = t8071*t870;
  t9197 = t9183 + t9191;
  t9201 = -1.*t1758*t8706;
  t9202 = -1.*t9197*t1926;
  t9211 = t9201 + t9202;
  t9222 = t1758*t9197;
  t9223 = -1.*t8706*t1926;
  t9229 = t9222 + t9223;
  t9236 = t2366*t9211;
  t9244 = t2243*t9229;
  t9245 = t9236 + t9244;
  t9271 = t2243*t9211;
  t9275 = -1.*t2366*t9229;
  t9283 = t9271 + t9275;
  t8844 = t1011*t109*t1327;
  t8845 = -1.*t8338*t1444;
  t8847 = t8844 + t8845;
  t9362 = -1.*t752*t498;
  t9371 = t1011*t501*t870;
  t9379 = t9362 + t9371;
  t9389 = -1.*t1758*t8847;
  t9390 = -1.*t9379*t1926;
  t9414 = t9389 + t9390;
  t9434 = t1758*t9379;
  t9448 = -1.*t8847*t1926;
  t9458 = t9434 + t9448;
  t9479 = t2366*t9414;
  t9486 = t2243*t9458;
  t9493 = t9479 + t9486;
  t9501 = t2243*t9414;
  t9505 = -1.*t2366*t9458;
  t9506 = t9501 + t9505;
  t9625 = -1.*t2243*t4862;
  t9638 = t9625 + t9116;
  t5171 = t2993*t4935;
  t9534 = -0.022225*t2243;
  t9536 = -0.86008*t2366;
  t9542 = t9534 + t9536;
  t9555 = 0.86008*t2243;
  t9585 = t9555 + t2403;
  t9715 = t1758*t8706;
  t9719 = t9197*t1926;
  t9720 = t9715 + t9719;
  t9738 = -1.*t2366*t9720;
  t9742 = t9738 + t9244;
  t9746 = -1.*t2243*t9720;
  t9752 = t9746 + t9275;
  t9799 = t1758*t8847;
  t9803 = t9379*t1926;
  t9805 = t9799 + t9803;
  t9812 = -1.*t2366*t9805;
  t9816 = t9812 + t9486;
  t9821 = -1.*t2243*t9805;
  t9824 = t9821 + t9505;
  t9649 = -1.*t3144*t4935;
  t5179 = -1.*t3144*t5073;
  t5191 = t5171 + t5179;
  t9842 = 1.34008*t2993;
  t9846 = t9842 + t3472;
  t9849 = -0.021147*t2993;
  t9851 = -1.34008*t3144;
  t9852 = t9849 + t9851;
  t9761 = -1.*t3144*t9742;
  t9870 = t2243*t9720;
  t9873 = t2366*t9229;
  t9874 = t9870 + t9873;
  t9789 = t2993*t9742;
  t9826 = -1.*t3144*t9816;
  t9890 = t2243*t9805;
  t9892 = t2366*t9458;
  t9893 = t9890 + t9892;
  t9832 = t2993*t9816;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1297*t1526 + 0.167004*(t1297*t1327 + t1444*t1572) + t1572*t1649 + t1738*t2054 + t2171*t2220 + t2404*t2457 + t2744*t2887 - 1.*t1011*t1195*t326 + t3220*t3267 + t3504*t3647 - 1.250132*(t3144*t3267 + t2993*t3647) + 0.043925*(t2993*t3267 - 1.*t3144*t3647) + t599*t974;
  p_output1(10)=t1526*t4090 + t1649*t4402 + 0.167004*(t1327*t4090 + t1444*t4402) + t2054*t4666 + t2220*t4787 + t2404*t4862 + t2744*t4879 + t3220*t4935 + t3504*t5073 - 1.250132*(t3144*t4935 + t2993*t5073) + 0.043925*t5191 + t1011*t1195*t62 + t3923*t974;
  p_output1(11)=0;
  p_output1(12)=t1526*t5335 + t2054*t5540 + t2220*t5623 + t2404*t5657 + t2744*t5733 + t3220*t5827 + t3504*t5870 - 1.250132*(t3144*t5827 + t2993*t5870) + 0.043925*(t2993*t5827 - 1.*t3144*t5870) + t1011*t109*t1649*t62 - 1.*t1195*t498*t62 + 0.167004*(t1327*t5335 + t1011*t109*t1444*t62) + t1011*t501*t62*t974;
  p_output1(13)=t1011*t109*t1649*t326 - 1.*t1195*t326*t498 + t1526*t6006 + 0.167004*(t1011*t109*t1444*t326 + t1327*t6006) + t2054*t6120 + t2220*t6143 + t2404*t6183 + t2744*t6222 + t3220*t6257 + t3504*t6331 - 1.250132*(t3144*t6257 + t2993*t6331) + 0.043925*(t2993*t6257 - 1.*t3144*t6331) + t1011*t326*t501*t974;
  p_output1(14)=-1.*t1011*t1195 - 1.*t109*t1649*t498 + t1526*t6524 + 0.167004*(-1.*t109*t1444*t498 + t1327*t6524) + t2054*t6572 + t2220*t6615 + t2404*t6634 + t2744*t6657 + t3220*t6704 + t3504*t6742 - 1.250132*(t3144*t6704 + t2993*t6742) + 0.043925*(t2993*t6704 - 1.*t3144*t6742) - 1.*t498*t501*t974;
  p_output1(15)=t1649*t6821 + t2054*t6918 + t2404*t6942 + t2744*t6976 + t3220*t7041 + t3504*t7146 - 1.250132*(t3144*t7041 + t2993*t7146) + 0.043925*(t2993*t7041 - 1.*t3144*t7146) + t1526*t4402*t752 + 0.167004*(t1444*t6821 + t1327*t4402*t752) + t2220*t4402*t870 + t4402*t974;
  p_output1(16)=t1649*t599 + t2054*t7344 + t2404*t7378 + t2744*t7405 + t3220*t7422 + t3504*t7456 - 1.250132*(t3144*t7422 + t2993*t7456) + 0.043925*(t2993*t7422 - 1.*t3144*t7456) + t1526*t7283*t752 + 0.167004*(t1444*t599 + t1327*t7283*t752) + t2220*t7283*t870 + t7283*t974;
  p_output1(17)=-1.*t1011*t1649*t501 + t1011*t109*t1526*t752 + 0.167004*(-1.*t1011*t1444*t501 + t1011*t109*t1327*t752) + t2054*t7568 + t2404*t7595 + t2744*t7622 + t3220*t7671 + t3504*t7685 - 1.250132*(t3144*t7671 + t2993*t7685) + 0.043925*(t2993*t7671 - 1.*t3144*t7685) + t1011*t109*t2220*t870 + t1011*t109*t974;
  p_output1(18)=t2220*t4090 + t1011*t62*t7802 + t3923*t7824 + 0.167004*t1327*t7847 + t1526*t7847 - 1.*t1444*t2054*t7847 + t2404*t7895 + t2744*t7934 + t3220*t7983 + t3504*t8007 - 1.250132*(t3144*t7983 + t2993*t8007) + 0.043925*(t2993*t7983 - 1.*t3144*t8007);
  p_output1(19)=t1011*t326*t7802 + t7824*t8071 + 0.167004*t1327*t8078 + t1526*t8078 - 1.*t1444*t2054*t8078 + t2220*t8103 + t2404*t8135 + t2744*t8170 + t3220*t8194 + t3504*t8231 - 1.250132*(t3144*t8194 + t2993*t8231) + 0.043925*(t2993*t8194 - 1.*t3144*t8231);
  p_output1(20)=-1.*t498*t7802 + t1011*t501*t7824 + 0.167004*t1327*t8311 + t1526*t8311 - 1.*t1444*t2054*t8311 + t2220*t8338 + t2404*t8374 + t2744*t8391 + t3220*t8402 + t3504*t8420 - 1.250132*(t3144*t8402 + t2993*t8420) + 0.043925*(t2993*t8402 - 1.*t3144*t8420);
  p_output1(21)=0.167004*t4666 + t4402*t8477 + t4090*t8495 + t2054*t8518 + t1758*t2404*t8518 - 1.*t1926*t2744*t8518 + t3220*t8569 + t3504*t8589 - 1.250132*(t3144*t8569 + t2993*t8589) + 0.043925*(t2993*t8569 - 1.*t3144*t8589);
  p_output1(22)=t7283*t8477 + t8103*t8495 + t2054*t8684 + t1758*t2404*t8684 - 1.*t1926*t2744*t8684 + 0.167004*t8706 + t3220*t8725 + t3504*t8753 - 1.250132*(t3144*t8725 + t2993*t8753) + 0.043925*(t2993*t8725 - 1.*t3144*t8753);
  p_output1(23)=t1011*t109*t8477 + t8338*t8495 + t2054*t8839 + t1758*t2404*t8839 - 1.*t1926*t2744*t8839 + 0.167004*t8847 + t3220*t8916 + t3504*t8940 - 1.250132*(t3144*t8916 + t2993*t8940) + 0.043925*(t2993*t8916 - 1.*t3144*t8940);
  p_output1(24)=t2404*t4879 + t4666*t9043 + t4787*t9047 + t2744*t9057 + t3504*t9103 + t3220*t9119 + 0.043925*(-1.*t3144*t9103 + t2993*t9119) - 1.250132*(t2993*t9103 + t3144*t9119);
  p_output1(25)=t8706*t9043 + t9047*t9197 + t2744*t9211 + t2404*t9229 + t3504*t9245 + t3220*t9283 + 0.043925*(-1.*t3144*t9245 + t2993*t9283) - 1.250132*(t2993*t9245 + t3144*t9283);
  p_output1(26)=t8847*t9043 + t9047*t9379 + t2744*t9414 + t2404*t9458 + t3504*t9493 + t3220*t9506 + 0.043925*(-1.*t3144*t9493 + t2993*t9506) - 1.250132*(t2993*t9493 + t3144*t9506);
  p_output1(27)=t3504*t4935 + t4862*t9542 + t4879*t9585 + t3220*t9638 - 1.250132*(t5171 + t3144*t9638) + 0.043925*(t2993*t9638 + t9649);
  p_output1(28)=t9229*t9585 + t9542*t9720 + t3504*t9742 + t3220*t9752 + 0.043925*(t2993*t9752 + t9761) - 1.250132*(t3144*t9752 + t9789);
  p_output1(29)=t9458*t9585 + t9542*t9805 + t3504*t9816 + t3220*t9824 + 0.043925*(t2993*t9824 + t9826) - 1.250132*(t3144*t9824 + t9832);
  p_output1(30)=-1.250132*t5191 + 0.043925*(-1.*t2993*t5073 + t9649) + t4935*t9846 + t5073*t9852;
  p_output1(31)=t9742*t9846 + t9852*t9874 + 0.043925*(t9761 - 1.*t2993*t9874) - 1.250132*(t9789 - 1.*t3144*t9874);
  p_output1(32)=t9816*t9846 + t9852*t9893 + 0.043925*(t9826 - 1.*t2993*t9893) - 1.250132*(t9832 - 1.*t3144*t9893);
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


       
void Jp_LeftFootFront(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}