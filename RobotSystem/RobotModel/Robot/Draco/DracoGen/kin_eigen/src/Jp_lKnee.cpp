/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:19 GMT-05:00
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
  double t279;
  double t385;
  double t444;
  double t455;
  double t472;
  double t185;
  double t224;
  double t276;
  double t294;
  double t304;
  double t355;
  double t376;
  double t530;
  double t1189;
  double t1190;
  double t1202;
  double t1219;
  double t986;
  double t1101;
  double t1171;
  double t1238;
  double t1239;
  double t1240;
  double t1257;
  double t1258;
  double t1260;
  double t1262;
  double t1254;
  double t1255;
  double t1256;
  double t1268;
  double t1269;
  double t1270;
  double t1276;
  double t1279;
  double t1280;
  double t1282;
  double t1289;
  double t1291;
  double t1294;
  double t1387;
  double t1417;
  double t1424;
  double t467;
  double t496;
  double t520;
  double t650;
  double t812;
  double t904;
  double t1851;
  double t1896;
  double t1931;
  double t1209;
  double t1220;
  double t1224;
  double t1241;
  double t1242;
  double t1243;
  double t2089;
  double t2320;
  double t2425;
  double t2562;
  double t2659;
  double t2708;
  double t1261;
  double t1265;
  double t1266;
  double t1272;
  double t1273;
  double t1274;
  double t1281;
  double t1285;
  double t1286;
  double t2847;
  double t2866;
  double t2875;
  double t2969;
  double t2975;
  double t3014;
  double t1296;
  double t1297;
  double t1298;
  double t3122;
  double t3141;
  double t3150;
  double t3192;
  double t3195;
  double t3199;
  double t3815;
  double t3824;
  double t3838;
  double t3855;
  double t3865;
  double t3866;
  double t3873;
  double t3882;
  double t3884;
  double t3886;
  double t3888;
  double t3889;
  double t3891;
  double t3895;
  double t3897;
  double t4065;
  double t4089;
  double t4095;
  double t4565;
  double t4585;
  double t4758;
  double t4797;
  double t4901;
  double t4951;
  double t4961;
  double t4962;
  double t4965;
  double t5010;
  double t5020;
  double t5125;
  double t5641;
  double t5649;
  double t5680;
  double t5813;
  double t5820;
  double t5833;
  double t5896;
  double t5899;
  double t5905;
  double t5927;
  double t5930;
  double t5932;
  double t5948;
  double t5954;
  double t5956;
  double t6037;
  double t6045;
  double t6047;
  double t6096;
  double t6111;
  double t6143;
  double t6152;
  double t6155;
  double t6163;
  double t6184;
  double t6195;
  double t6199;
  double t6255;
  double t6258;
  double t6260;
  double t6340;
  double t6363;
  double t6367;
  double t6386;
  double t6401;
  double t6402;
  double t6406;
  double t6410;
  double t6426;
  double t6493;
  double t6495;
  double t6496;
  double t6514;
  double t6524;
  double t6539;
  double t6561;
  double t6563;
  double t6566;
  double t6684;
  double t6698;
  double t6700;
  double t6722;
  double t6726;
  double t6735;
  double t6747;
  double t6749;
  double t6762;
  double t6661;
  double t6663;
  double t6665;
  double t6670;
  double t6671;
  double t6835;
  double t6843;
  double t6845;
  double t6849;
  double t6854;
  double t6871;
  double t6873;
  double t6876;
  double t6892;
  double t6896;
  double t6898;
  double t6915;
  double t6921;
  double t6928;
  double t7000;
  double t7004;
  double t7008;
  double t7033;
  double t7041;
  double t7050;
  double t7059;
  double t7070;
  double t7077;
  double t7088;
  double t7100;
  double t7105;
  double t7176;
  double t7187;
  double t7194;
  double t7159;
  double t7165;
  double t7169;
  double t7171;
  double t7173;
  double t7278;
  double t7280;
  double t7285;
  double t7399;
  double t7400;
  double t7405;
  double t7559;
  double t7566;
  double t7567;
  double t3302;
  double t7290;
  double t7295;
  double t7304;
  double t7527;
  double t7528;
  double t7535;
  double t7539;
  double t7542;
  double t7628;
  double t7639;
  double t7649;
  double t7662;
  double t7676;
  double t7682;
  double t7695;
  double t7718;
  double t7719;
  double t7410;
  double t7411;
  double t7425;
  double t7799;
  double t7820;
  double t7825;
  double t7835;
  double t7840;
  double t7851;
  double t7854;
  double t7867;
  double t7877;
  double t3297;
  double t3357;
  double t7604;
  double t7945;
  double t7953;
  double t7960;
  double t7978;
  double t7985;
  double t8078;
  double t8089;
  double t8095;
  double t7747;
  double t7763;
  double t8182;
  double t8193;
  double t8206;
  double t7891;
  double t7914;
  t279 = Sin(var1[3]);
  t385 = Cos(var1[6]);
  t444 = -1.*t385;
  t455 = 1. + t444;
  t472 = Sin(var1[6]);
  t185 = Cos(var1[3]);
  t224 = Cos(var1[5]);
  t276 = -1.*t185*t224;
  t294 = Sin(var1[4]);
  t304 = Sin(var1[5]);
  t355 = -1.*t279*t294*t304;
  t376 = t276 + t355;
  t530 = Cos(var1[4]);
  t1189 = Cos(var1[7]);
  t1190 = -1.*t1189;
  t1202 = 1. + t1190;
  t1219 = Sin(var1[7]);
  t986 = t385*t376;
  t1101 = t530*t279*t472;
  t1171 = t986 + t1101;
  t1238 = -1.*t224*t279*t294;
  t1239 = t185*t304;
  t1240 = t1238 + t1239;
  t1257 = Cos(var1[8]);
  t1258 = -1.*t1257;
  t1260 = 1. + t1258;
  t1262 = Sin(var1[8]);
  t1254 = t1189*t1240;
  t1255 = -1.*t1171*t1219;
  t1256 = t1254 + t1255;
  t1268 = -1.*t530*t385*t279;
  t1269 = t376*t472;
  t1270 = t1268 + t1269;
  t1276 = Cos(var1[9]);
  t1279 = -1.*t1276;
  t1280 = 1. + t1279;
  t1282 = Sin(var1[9]);
  t1289 = t1257*t1256;
  t1291 = t1270*t1262;
  t1294 = t1289 + t1291;
  t1387 = t1257*t1270;
  t1417 = -1.*t1256*t1262;
  t1424 = t1387 + t1417;
  t467 = 0.087*t455;
  t496 = 0.0222*t472;
  t520 = 0. + t467 + t496;
  t650 = -0.0222*t455;
  t812 = 0.087*t472;
  t904 = 0. + t650 + t812;
  t1851 = -1.*t224*t279;
  t1896 = t185*t294*t304;
  t1931 = t1851 + t1896;
  t1209 = 0.157*t1202;
  t1220 = -0.3151*t1219;
  t1224 = 0. + t1209 + t1220;
  t1241 = -0.3151*t1202;
  t1242 = -0.157*t1219;
  t1243 = 0. + t1241 + t1242;
  t2089 = t385*t1931;
  t2320 = -1.*t185*t530*t472;
  t2425 = t2089 + t2320;
  t2562 = t185*t224*t294;
  t2659 = t279*t304;
  t2708 = t2562 + t2659;
  t1261 = -0.3801*t1260;
  t1265 = -0.0222*t1262;
  t1266 = 0. + t1261 + t1265;
  t1272 = -0.0222*t1260;
  t1273 = 0.3801*t1262;
  t1274 = 0. + t1272 + t1273;
  t1281 = -0.8601*t1280;
  t1285 = -0.0222*t1282;
  t1286 = 0. + t1281 + t1285;
  t2847 = t1189*t2708;
  t2866 = -1.*t2425*t1219;
  t2875 = t2847 + t2866;
  t2969 = t185*t530*t385;
  t2975 = t1931*t472;
  t3014 = t2969 + t2975;
  t1296 = -0.0222*t1280;
  t1297 = 0.8601*t1282;
  t1298 = 0. + t1296 + t1297;
  t3122 = t1257*t2875;
  t3141 = t3014*t1262;
  t3150 = t3122 + t3141;
  t3192 = t1257*t3014;
  t3195 = -1.*t2875*t1262;
  t3199 = t3192 + t3195;
  t3815 = t185*t530*t385*t304;
  t3824 = t185*t294*t472;
  t3838 = t3815 + t3824;
  t3855 = t185*t530*t224*t1189;
  t3865 = -1.*t3838*t1219;
  t3866 = t3855 + t3865;
  t3873 = -1.*t185*t385*t294;
  t3882 = t185*t530*t304*t472;
  t3884 = t3873 + t3882;
  t3886 = t1257*t3866;
  t3888 = t3884*t1262;
  t3889 = t3886 + t3888;
  t3891 = t1257*t3884;
  t3895 = -1.*t3866*t1262;
  t3897 = t3891 + t3895;
  t4065 = t530*t385*t279*t304;
  t4089 = t279*t294*t472;
  t4095 = t4065 + t4089;
  t4565 = t530*t224*t1189*t279;
  t4585 = -1.*t4095*t1219;
  t4758 = t4565 + t4585;
  t4797 = -1.*t385*t279*t294;
  t4901 = t530*t279*t304*t472;
  t4951 = t4797 + t4901;
  t4961 = t1257*t4758;
  t4962 = t4951*t1262;
  t4965 = t4961 + t4962;
  t5010 = t1257*t4951;
  t5020 = -1.*t4758*t1262;
  t5125 = t5010 + t5020;
  t5641 = -1.*t385*t294*t304;
  t5649 = t530*t472;
  t5680 = t5641 + t5649;
  t5813 = -1.*t224*t1189*t294;
  t5820 = -1.*t5680*t1219;
  t5833 = t5813 + t5820;
  t5896 = -1.*t530*t385;
  t5899 = -1.*t294*t304*t472;
  t5905 = t5896 + t5899;
  t5927 = t1257*t5833;
  t5930 = t5905*t1262;
  t5932 = t5927 + t5930;
  t5948 = t1257*t5905;
  t5954 = -1.*t5833*t1262;
  t5956 = t5948 + t5954;
  t6037 = t224*t279;
  t6045 = -1.*t185*t294*t304;
  t6047 = t6037 + t6045;
  t6096 = t1189*t6047;
  t6111 = -1.*t385*t2708*t1219;
  t6143 = t6096 + t6111;
  t6152 = t1257*t6143;
  t6155 = t2708*t472*t1262;
  t6163 = t6152 + t6155;
  t6184 = t1257*t2708*t472;
  t6195 = -1.*t6143*t1262;
  t6199 = t6184 + t6195;
  t6255 = t224*t279*t294;
  t6258 = -1.*t185*t304;
  t6260 = t6255 + t6258;
  t6340 = t1189*t376;
  t6363 = -1.*t385*t6260*t1219;
  t6367 = t6340 + t6363;
  t6386 = t1257*t6367;
  t6401 = t6260*t472*t1262;
  t6402 = t6386 + t6401;
  t6406 = t1257*t6260*t472;
  t6410 = -1.*t6367*t1262;
  t6426 = t6406 + t6410;
  t6493 = -1.*t530*t1189*t304;
  t6495 = -1.*t530*t224*t385*t1219;
  t6496 = t6493 + t6495;
  t6514 = t1257*t6496;
  t6524 = t530*t224*t472*t1262;
  t6539 = t6514 + t6524;
  t6561 = t530*t224*t1257*t472;
  t6563 = -1.*t6496*t1262;
  t6566 = t6561 + t6563;
  t6684 = -1.*t185*t530*t385;
  t6698 = -1.*t1931*t472;
  t6700 = t6684 + t6698;
  t6722 = -1.*t1257*t6700*t1219;
  t6726 = t2425*t1262;
  t6735 = t6722 + t6726;
  t6747 = t1257*t2425;
  t6749 = t6700*t1219*t1262;
  t6762 = t6747 + t6749;
  t6661 = 0.087*t385;
  t6663 = -0.0222*t472;
  t6665 = t6661 + t6663;
  t6670 = 0.0222*t385;
  t6671 = t6670 + t812;
  t6835 = t185*t224;
  t6843 = t279*t294*t304;
  t6845 = t6835 + t6843;
  t6849 = -1.*t6845*t472;
  t6854 = t1268 + t6849;
  t6871 = t385*t6845;
  t6873 = -1.*t530*t279*t472;
  t6876 = t6871 + t6873;
  t6892 = -1.*t1257*t6854*t1219;
  t6896 = t6876*t1262;
  t6898 = t6892 + t6896;
  t6915 = t1257*t6876;
  t6921 = t6854*t1219*t1262;
  t6928 = t6915 + t6921;
  t7000 = t385*t294;
  t7004 = -1.*t530*t304*t472;
  t7008 = t7000 + t7004;
  t7033 = t530*t385*t304;
  t7041 = t294*t472;
  t7050 = t7033 + t7041;
  t7059 = -1.*t1257*t7008*t1219;
  t7070 = t7050*t1262;
  t7077 = t7059 + t7070;
  t7088 = t1257*t7050;
  t7100 = t7008*t1219*t1262;
  t7105 = t7088 + t7100;
  t7176 = -1.*t1189*t2425;
  t7187 = -1.*t2708*t1219;
  t7194 = t7176 + t7187;
  t7159 = -0.157*t1189;
  t7165 = t7159 + t1220;
  t7169 = -0.3151*t1189;
  t7171 = 0.157*t1219;
  t7173 = t7169 + t7171;
  t7278 = -1.*t1189*t6876;
  t7280 = -1.*t6260*t1219;
  t7285 = t7278 + t7280;
  t7399 = -1.*t1189*t7050;
  t7400 = -1.*t530*t224*t1219;
  t7405 = t7399 + t7400;
  t7559 = -1.*t1257*t2875;
  t7566 = -1.*t3014*t1262;
  t7567 = t7559 + t7566;
  t3302 = t1276*t3199;
  t7290 = t1189*t6260;
  t7295 = -1.*t6876*t1219;
  t7304 = t7290 + t7295;
  t7527 = -0.0222*t1257;
  t7528 = -0.3801*t1262;
  t7535 = t7527 + t7528;
  t7539 = 0.3801*t1257;
  t7542 = t7539 + t1265;
  t7628 = t530*t385*t279;
  t7639 = t6845*t472;
  t7649 = t7628 + t7639;
  t7662 = -1.*t1257*t7304;
  t7676 = -1.*t7649*t1262;
  t7682 = t7662 + t7676;
  t7695 = t1257*t7649;
  t7718 = -1.*t7304*t1262;
  t7719 = t7695 + t7718;
  t7410 = t530*t224*t1189;
  t7411 = -1.*t7050*t1219;
  t7425 = t7410 + t7411;
  t7799 = -1.*t385*t294;
  t7820 = t530*t304*t472;
  t7825 = t7799 + t7820;
  t7835 = -1.*t1257*t7425;
  t7840 = -1.*t7825*t1262;
  t7851 = t7835 + t7840;
  t7854 = t1257*t7825;
  t7867 = -1.*t7425*t1262;
  t7877 = t7854 + t7867;
  t3297 = -1.*t1282*t3150;
  t3357 = t3297 + t3302;
  t7604 = -1.*t1282*t3199;
  t7945 = -0.0222*t1276;
  t7953 = -0.8601*t1282;
  t7960 = t7945 + t7953;
  t7978 = 0.8601*t1276;
  t7985 = t7978 + t1285;
  t8078 = t1257*t7304;
  t8089 = t7649*t1262;
  t8095 = t8078 + t8089;
  t7747 = t1276*t7719;
  t7763 = -1.*t1282*t7719;
  t8182 = t1257*t7425;
  t8193 = t7825*t1262;
  t8206 = t8182 + t8193;
  t7891 = t1276*t7877;
  t7914 = -1.*t1282*t7877;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1171*t1224 + 0.1502*(t1171*t1189 + t1219*t1240) + t1240*t1243 + t1256*t1266 + t1270*t1274 + t1286*t1294 + t1298*t1424 - 0.0222*(-1.*t1282*t1294 + t1276*t1424) - 0.8601*(t1276*t1294 + t1282*t1424) + t376*t520 - 1.*t279*t530*t904;
  p_output1(10)=t1224*t2425 + t1243*t2708 + 0.1502*(t1189*t2425 + t1219*t2708) + t1266*t2875 + t1274*t3014 + t1286*t3150 + t1298*t3199 - 0.8601*(t1276*t3150 + t1282*t3199) - 0.0222*t3357 + t1931*t520 + t185*t530*t904;
  p_output1(11)=0;
  p_output1(12)=t1224*t3838 + t1266*t3866 + t1274*t3884 + t1286*t3889 + t1298*t3897 - 0.0222*(-1.*t1282*t3889 + t1276*t3897) - 0.8601*(t1276*t3889 + t1282*t3897) + t1243*t185*t224*t530 + t185*t304*t520*t530 + 0.1502*(t1189*t3838 + t1219*t185*t224*t530) - 1.*t185*t294*t904;
  p_output1(13)=t1224*t4095 + t1266*t4758 + t1274*t4951 + t1286*t4965 + t1298*t5125 - 0.0222*(-1.*t1282*t4965 + t1276*t5125) - 0.8601*(t1276*t4965 + t1282*t5125) + t1243*t224*t279*t530 + t279*t304*t520*t530 + 0.1502*(t1189*t4095 + t1219*t224*t279*t530) - 1.*t279*t294*t904;
  p_output1(14)=-1.*t1243*t224*t294 - 1.*t294*t304*t520 + t1224*t5680 + 0.1502*(-1.*t1219*t224*t294 + t1189*t5680) + t1266*t5833 + t1274*t5905 + t1286*t5932 + t1298*t5956 - 0.0222*(-1.*t1282*t5932 + t1276*t5956) - 0.8601*(t1276*t5932 + t1282*t5956) - 1.*t530*t904;
  p_output1(15)=t1224*t2708*t385 + t1274*t2708*t472 + t2708*t520 + t1243*t6047 + 0.1502*(t1189*t2708*t385 + t1219*t6047) + t1266*t6143 + t1286*t6163 + t1298*t6199 - 0.0222*(-1.*t1282*t6163 + t1276*t6199) - 0.8601*(t1276*t6163 + t1282*t6199);
  p_output1(16)=t1243*t376 + t1224*t385*t6260 + t1274*t472*t6260 + t520*t6260 + 0.1502*(t1219*t376 + t1189*t385*t6260) + t1266*t6367 + t1286*t6402 + t1298*t6426 - 0.0222*(-1.*t1282*t6402 + t1276*t6426) - 0.8601*(t1276*t6402 + t1282*t6426);
  p_output1(17)=-1.*t1243*t304*t530 + t1224*t224*t385*t530 + t1274*t224*t472*t530 + t224*t520*t530 + 0.1502*(-1.*t1219*t304*t530 + t1189*t224*t385*t530) + t1266*t6496 + t1286*t6539 + t1298*t6566 - 0.0222*(-1.*t1282*t6539 + t1276*t6566) - 0.8601*(t1276*t6539 + t1282*t6566);
  p_output1(18)=t1274*t2425 + t185*t530*t6665 + t1931*t6671 + 0.1502*t1189*t6700 + t1224*t6700 - 1.*t1219*t1266*t6700 + t1286*t6735 + t1298*t6762 - 0.0222*(-1.*t1282*t6735 + t1276*t6762) - 0.8601*(t1276*t6735 + t1282*t6762);
  p_output1(19)=t279*t530*t6665 + t6671*t6845 + 0.1502*t1189*t6854 + t1224*t6854 - 1.*t1219*t1266*t6854 + t1274*t6876 + t1286*t6898 + t1298*t6928 - 0.0222*(-1.*t1282*t6898 + t1276*t6928) - 0.8601*(t1276*t6898 + t1282*t6928);
  p_output1(20)=-1.*t294*t6665 + t304*t530*t6671 + 0.1502*t1189*t7008 + t1224*t7008 - 1.*t1219*t1266*t7008 + t1274*t7050 + t1286*t7077 + t1298*t7105 - 0.0222*(-1.*t1282*t7077 + t1276*t7105) - 0.8601*(t1276*t7077 + t1282*t7105);
  p_output1(21)=0.1502*t2875 + t2708*t7165 + t2425*t7173 + t1266*t7194 + t1257*t1286*t7194 - 1.*t1262*t1298*t7194 - 0.0222*(-1.*t1262*t1276*t7194 - 1.*t1257*t1282*t7194) - 0.8601*(t1257*t1276*t7194 - 1.*t1262*t1282*t7194);
  p_output1(22)=t6260*t7165 + t6876*t7173 + t1266*t7285 + t1257*t1286*t7285 - 1.*t1262*t1298*t7285 - 0.0222*(-1.*t1262*t1276*t7285 - 1.*t1257*t1282*t7285) - 0.8601*(t1257*t1276*t7285 - 1.*t1262*t1282*t7285) + 0.1502*t7304;
  p_output1(23)=t224*t530*t7165 + t7050*t7173 + t1266*t7405 + t1257*t1286*t7405 - 1.*t1262*t1298*t7405 - 0.0222*(-1.*t1262*t1276*t7405 - 1.*t1257*t1282*t7405) - 0.8601*(t1257*t1276*t7405 - 1.*t1262*t1282*t7405) + 0.1502*t7425;
  p_output1(24)=t1286*t3199 + t2875*t7535 + t3014*t7542 + t1298*t7567 - 0.8601*(t3302 + t1282*t7567) - 0.0222*(t1276*t7567 + t7604);
  p_output1(25)=t7304*t7535 + t7542*t7649 + t1298*t7682 + t1286*t7719 - 0.8601*(t1282*t7682 + t7747) - 0.0222*(t1276*t7682 + t7763);
  p_output1(26)=t7425*t7535 + t7542*t7825 + t1298*t7851 + t1286*t7877 - 0.8601*(t1282*t7851 + t7891) - 0.0222*(t1276*t7851 + t7914);
  p_output1(27)=-0.8601*t3357 - 0.0222*(-1.*t1276*t3150 + t7604) + t3150*t7960 + t3199*t7985;
  p_output1(28)=t7719*t7985 + t7960*t8095 - 0.0222*(t7763 - 1.*t1276*t8095) - 0.8601*(t7747 - 1.*t1282*t8095);
  p_output1(29)=t7877*t7985 + t7960*t8206 - 0.0222*(t7914 - 1.*t1276*t8206) - 0.8601*(t7891 - 1.*t1282*t8206);
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
