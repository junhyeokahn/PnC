/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:05:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rKnee.h"

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
  double t835;
  double t473;
  double t519;
  double t533;
  double t587;
  double t1605;
  double t1398;
  double t1410;
  double t1619;
  double t1110;
  double t1113;
  double t1115;
  double t1162;
  double t63;
  double t2090;
  double t2241;
  double t2252;
  double t1515;
  double t1833;
  double t1839;
  double t2459;
  double t2489;
  double t2494;
  double t2634;
  double t2657;
  double t2679;
  double t2806;
  double t2950;
  double t2977;
  double t3053;
  double t3545;
  double t3637;
  double t3645;
  double t3736;
  double t3841;
  double t3975;
  double t4071;
  double t4163;
  double t4269;
  double t4271;
  double t4500;
  double t4616;
  double t4632;
  double t558;
  double t600;
  double t826;
  double t1156;
  double t1278;
  double t1376;
  double t1875;
  double t2028;
  double t2042;
  double t2342;
  double t2390;
  double t2414;
  double t5583;
  double t5649;
  double t5715;
  double t2743;
  double t2889;
  double t2912;
  double t5503;
  double t5546;
  double t5550;
  double t5813;
  double t5841;
  double t5929;
  double t3378;
  double t3424;
  double t3440;
  double t4062;
  double t4116;
  double t4129;
  double t6088;
  double t6089;
  double t6098;
  double t6260;
  double t6292;
  double t6302;
  double t4460;
  double t4466;
  double t4467;
  double t6451;
  double t6472;
  double t6473;
  double t6641;
  double t6654;
  double t6659;
  double t6956;
  double t6961;
  double t6969;
  double t6992;
  double t7105;
  double t7110;
  double t7236;
  double t7246;
  double t7260;
  double t7271;
  double t7272;
  double t7273;
  double t7360;
  double t7367;
  double t7377;
  double t7488;
  double t7489;
  double t7493;
  double t7495;
  double t7534;
  double t7545;
  double t7612;
  double t7616;
  double t7624;
  double t7648;
  double t7652;
  double t7656;
  double t7665;
  double t7668;
  double t7670;
  double t7763;
  double t7764;
  double t7768;
  double t7772;
  double t7781;
  double t7785;
  double t7835;
  double t7843;
  double t7863;
  double t7867;
  double t7869;
  double t7873;
  double t7884;
  double t7886;
  double t7888;
  double t7957;
  double t7969;
  double t7983;
  double t7988;
  double t7992;
  double t8010;
  double t8034;
  double t8036;
  double t8047;
  double t8051;
  double t8063;
  double t8079;
  double t8142;
  double t8149;
  double t8151;
  double t8211;
  double t8214;
  double t8236;
  double t8257;
  double t8258;
  double t8261;
  double t8266;
  double t8269;
  double t8273;
  double t8354;
  double t8355;
  double t8371;
  double t8382;
  double t8383;
  double t8386;
  double t8394;
  double t8395;
  double t8403;
  double t8465;
  double t8466;
  double t8475;
  double t8488;
  double t8492;
  double t8496;
  double t8508;
  double t8509;
  double t8511;
  double t8435;
  double t8437;
  double t8441;
  double t8443;
  double t8450;
  double t8567;
  double t8573;
  double t8578;
  double t8614;
  double t8620;
  double t8582;
  double t8603;
  double t8606;
  double t8632;
  double t8638;
  double t8639;
  double t8656;
  double t8662;
  double t8663;
  double t8725;
  double t8728;
  double t8731;
  double t8717;
  double t8720;
  double t8721;
  double t8757;
  double t8765;
  double t8769;
  double t8781;
  double t8782;
  double t8786;
  double t8868;
  double t8872;
  double t8873;
  double t8827;
  double t8828;
  double t8836;
  double t8845;
  double t8847;
  double t8970;
  double t8975;
  double t8977;
  double t9061;
  double t9062;
  double t9063;
  double t9210;
  double t9215;
  double t9222;
  double t6681;
  double t9150;
  double t9163;
  double t9172;
  double t9174;
  double t9201;
  double t8988;
  double t9004;
  double t9007;
  double t9250;
  double t9255;
  double t9264;
  double t9281;
  double t9282;
  double t9287;
  double t9292;
  double t9298;
  double t9303;
  double t9081;
  double t9087;
  double t9100;
  double t9347;
  double t9350;
  double t9352;
  double t9365;
  double t9369;
  double t9376;
  double t9389;
  double t9393;
  double t9410;
  double t6679;
  double t6754;
  double t9243;
  double t9461;
  double t9462;
  double t9463;
  double t9466;
  double t9470;
  double t9501;
  double t9510;
  double t9514;
  double t9316;
  double t9332;
  double t9606;
  double t9609;
  double t9618;
  double t9431;
  double t9450;
  t835 = Sin(var1[3]);
  t473 = Cos(var1[11]);
  t519 = -1.*t473;
  t533 = 1. + t519;
  t587 = Sin(var1[11]);
  t1605 = Cos(var1[3]);
  t1398 = Cos(var1[5]);
  t1410 = Sin(var1[4]);
  t1619 = Sin(var1[5]);
  t1110 = Cos(var1[12]);
  t1113 = -1.*t1110;
  t1115 = 1. + t1113;
  t1162 = Sin(var1[12]);
  t63 = Cos(var1[4]);
  t2090 = -1.*t1605*t1398;
  t2241 = -1.*t835*t1410*t1619;
  t2252 = t2090 + t2241;
  t1515 = -1.*t1398*t835*t1410;
  t1833 = t1605*t1619;
  t1839 = t1515 + t1833;
  t2459 = t63*t587*t835;
  t2489 = t473*t2252;
  t2494 = t2459 + t2489;
  t2634 = Cos(var1[13]);
  t2657 = -1.*t2634;
  t2679 = 1. + t2657;
  t2806 = Sin(var1[13]);
  t2950 = -1.*t473*t63*t835;
  t2977 = t587*t2252;
  t3053 = t2950 + t2977;
  t3545 = t1110*t1839;
  t3637 = -1.*t1162*t2494;
  t3645 = t3545 + t3637;
  t3736 = Cos(var1[14]);
  t3841 = -1.*t3736;
  t3975 = 1. + t3841;
  t4071 = Sin(var1[14]);
  t4163 = t2806*t3053;
  t4269 = t2634*t3645;
  t4271 = t4163 + t4269;
  t4500 = t2634*t3053;
  t4616 = -1.*t2806*t3645;
  t4632 = t4500 + t4616;
  t558 = -0.022225*t533;
  t600 = -0.086996*t587;
  t826 = 0. + t558 + t600;
  t1156 = -0.31508*t1115;
  t1278 = 0.156996*t1162;
  t1376 = 0. + t1156 + t1278;
  t1875 = -0.086996*t533;
  t2028 = 0.022225*t587;
  t2042 = 0. + t1875 + t2028;
  t2342 = -0.156996*t1115;
  t2390 = -0.31508*t1162;
  t2414 = 0. + t2342 + t2390;
  t5583 = -1.*t1398*t835;
  t5649 = t1605*t1410*t1619;
  t5715 = t5583 + t5649;
  t2743 = -0.022225*t2679;
  t2889 = 0.38008*t2806;
  t2912 = 0. + t2743 + t2889;
  t5503 = t1605*t1398*t1410;
  t5546 = t835*t1619;
  t5550 = t5503 + t5546;
  t5813 = -1.*t1605*t63*t587;
  t5841 = t473*t5715;
  t5929 = t5813 + t5841;
  t3378 = -0.38008*t2679;
  t3424 = -0.022225*t2806;
  t3440 = 0. + t3378 + t3424;
  t4062 = -0.86008*t3975;
  t4116 = -0.022225*t4071;
  t4129 = 0. + t4062 + t4116;
  t6088 = t473*t1605*t63;
  t6089 = t587*t5715;
  t6098 = t6088 + t6089;
  t6260 = t1110*t5550;
  t6292 = -1.*t1162*t5929;
  t6302 = t6260 + t6292;
  t4460 = -0.022225*t3975;
  t4466 = 0.86008*t4071;
  t4467 = 0. + t4460 + t4466;
  t6451 = t2806*t6098;
  t6472 = t2634*t6302;
  t6473 = t6451 + t6472;
  t6641 = t2634*t6098;
  t6654 = -1.*t2806*t6302;
  t6659 = t6641 + t6654;
  t6956 = t1605*t587*t1410;
  t6961 = t473*t1605*t63*t1619;
  t6969 = t6956 + t6961;
  t6992 = -1.*t473*t1605*t1410;
  t7105 = t1605*t63*t587*t1619;
  t7110 = t6992 + t7105;
  t7236 = t1110*t1605*t63*t1398;
  t7246 = -1.*t1162*t6969;
  t7260 = t7236 + t7246;
  t7271 = t2806*t7110;
  t7272 = t2634*t7260;
  t7273 = t7271 + t7272;
  t7360 = t2634*t7110;
  t7367 = -1.*t2806*t7260;
  t7377 = t7360 + t7367;
  t7488 = t587*t835*t1410;
  t7489 = t473*t63*t835*t1619;
  t7493 = t7488 + t7489;
  t7495 = -1.*t473*t835*t1410;
  t7534 = t63*t587*t835*t1619;
  t7545 = t7495 + t7534;
  t7612 = t1110*t63*t1398*t835;
  t7616 = -1.*t1162*t7493;
  t7624 = t7612 + t7616;
  t7648 = t2806*t7545;
  t7652 = t2634*t7624;
  t7656 = t7648 + t7652;
  t7665 = t2634*t7545;
  t7668 = -1.*t2806*t7624;
  t7670 = t7665 + t7668;
  t7763 = t63*t587;
  t7764 = -1.*t473*t1410*t1619;
  t7768 = t7763 + t7764;
  t7772 = -1.*t473*t63;
  t7781 = -1.*t587*t1410*t1619;
  t7785 = t7772 + t7781;
  t7835 = -1.*t1110*t1398*t1410;
  t7843 = -1.*t1162*t7768;
  t7863 = t7835 + t7843;
  t7867 = t2806*t7785;
  t7869 = t2634*t7863;
  t7873 = t7867 + t7869;
  t7884 = t2634*t7785;
  t7886 = -1.*t2806*t7863;
  t7888 = t7884 + t7886;
  t7957 = t1398*t835;
  t7969 = -1.*t1605*t1410*t1619;
  t7983 = t7957 + t7969;
  t7988 = -1.*t473*t1162*t5550;
  t7992 = t1110*t7983;
  t8010 = t7988 + t7992;
  t8034 = t587*t2806*t5550;
  t8036 = t2634*t8010;
  t8047 = t8034 + t8036;
  t8051 = t2634*t587*t5550;
  t8063 = -1.*t2806*t8010;
  t8079 = t8051 + t8063;
  t8142 = t1398*t835*t1410;
  t8149 = -1.*t1605*t1619;
  t8151 = t8142 + t8149;
  t8211 = -1.*t473*t1162*t8151;
  t8214 = t1110*t2252;
  t8236 = t8211 + t8214;
  t8257 = t587*t2806*t8151;
  t8258 = t2634*t8236;
  t8261 = t8257 + t8258;
  t8266 = t2634*t587*t8151;
  t8269 = -1.*t2806*t8236;
  t8273 = t8266 + t8269;
  t8354 = -1.*t473*t63*t1398*t1162;
  t8355 = -1.*t1110*t63*t1619;
  t8371 = t8354 + t8355;
  t8382 = t63*t1398*t587*t2806;
  t8383 = t2634*t8371;
  t8386 = t8382 + t8383;
  t8394 = t2634*t63*t1398*t587;
  t8395 = -1.*t2806*t8371;
  t8403 = t8394 + t8395;
  t8465 = -1.*t473*t1605*t63;
  t8466 = -1.*t587*t5715;
  t8475 = t8465 + t8466;
  t8488 = t2806*t5929;
  t8492 = -1.*t2634*t1162*t8475;
  t8496 = t8488 + t8492;
  t8508 = t2634*t5929;
  t8509 = t1162*t2806*t8475;
  t8511 = t8508 + t8509;
  t8435 = -0.086996*t473;
  t8437 = -0.022225*t587;
  t8441 = t8435 + t8437;
  t8443 = 0.022225*t473;
  t8450 = t8443 + t600;
  t8567 = t1605*t1398;
  t8573 = t835*t1410*t1619;
  t8578 = t8567 + t8573;
  t8614 = -1.*t587*t8578;
  t8620 = t2950 + t8614;
  t8582 = -1.*t63*t587*t835;
  t8603 = t473*t8578;
  t8606 = t8582 + t8603;
  t8632 = t2806*t8606;
  t8638 = -1.*t2634*t1162*t8620;
  t8639 = t8632 + t8638;
  t8656 = t2634*t8606;
  t8662 = t1162*t2806*t8620;
  t8663 = t8656 + t8662;
  t8725 = t473*t1410;
  t8728 = -1.*t63*t587*t1619;
  t8731 = t8725 + t8728;
  t8717 = t587*t1410;
  t8720 = t473*t63*t1619;
  t8721 = t8717 + t8720;
  t8757 = t2806*t8721;
  t8765 = -1.*t2634*t1162*t8731;
  t8769 = t8757 + t8765;
  t8781 = t2634*t8721;
  t8782 = t1162*t2806*t8731;
  t8786 = t8781 + t8782;
  t8868 = -1.*t1162*t5550;
  t8872 = -1.*t1110*t5929;
  t8873 = t8868 + t8872;
  t8827 = 0.156996*t1110;
  t8828 = t8827 + t2390;
  t8836 = -0.31508*t1110;
  t8845 = -0.156996*t1162;
  t8847 = t8836 + t8845;
  t8970 = -1.*t1162*t8151;
  t8975 = -1.*t1110*t8606;
  t8977 = t8970 + t8975;
  t9061 = -1.*t63*t1398*t1162;
  t9062 = -1.*t1110*t8721;
  t9063 = t9061 + t9062;
  t9210 = -1.*t2806*t6098;
  t9215 = -1.*t2634*t6302;
  t9222 = t9210 + t9215;
  t6681 = t3736*t6659;
  t9150 = 0.38008*t2634;
  t9163 = t9150 + t3424;
  t9172 = -0.022225*t2634;
  t9174 = -0.38008*t2806;
  t9201 = t9172 + t9174;
  t8988 = t1110*t8151;
  t9004 = -1.*t1162*t8606;
  t9007 = t8988 + t9004;
  t9250 = t473*t63*t835;
  t9255 = t587*t8578;
  t9264 = t9250 + t9255;
  t9281 = -1.*t2806*t9264;
  t9282 = -1.*t2634*t9007;
  t9287 = t9281 + t9282;
  t9292 = t2634*t9264;
  t9298 = -1.*t2806*t9007;
  t9303 = t9292 + t9298;
  t9081 = t1110*t63*t1398;
  t9087 = -1.*t1162*t8721;
  t9100 = t9081 + t9087;
  t9347 = -1.*t473*t1410;
  t9350 = t63*t587*t1619;
  t9352 = t9347 + t9350;
  t9365 = -1.*t2806*t9352;
  t9369 = -1.*t2634*t9100;
  t9376 = t9365 + t9369;
  t9389 = t2634*t9352;
  t9393 = -1.*t2806*t9100;
  t9410 = t9389 + t9393;
  t6679 = -1.*t4071*t6473;
  t6754 = t6679 + t6681;
  t9243 = -1.*t4071*t6659;
  t9461 = -0.022225*t3736;
  t9462 = -0.86008*t4071;
  t9463 = t9461 + t9462;
  t9466 = 0.86008*t3736;
  t9470 = t9466 + t4116;
  t9501 = t2806*t9264;
  t9510 = t2634*t9007;
  t9514 = t9501 + t9510;
  t9316 = t3736*t9303;
  t9332 = -1.*t4071*t9303;
  t9606 = t2806*t9352;
  t9609 = t2634*t9100;
  t9618 = t9606 + t9609;
  t9431 = t3736*t9410;
  t9450 = -1.*t4071*t9410;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1376*t1839 + t2042*t2252 + t2414*t2494 - 0.150246*(t1162*t1839 + t1110*t2494) + t2912*t3053 + t3440*t3645 + t4129*t4271 + t4467*t4632 - 0.022225*(-1.*t4071*t4271 + t3736*t4632) - 0.86008*(t3736*t4271 + t4071*t4632) - 1.*t63*t826*t835;
  p_output1(10)=t1376*t5550 + t2042*t5715 + t2414*t5929 - 0.150246*(t1162*t5550 + t1110*t5929) + t2912*t6098 + t3440*t6302 + t4129*t6473 + t4467*t6659 - 0.86008*(t3736*t6473 + t4071*t6659) - 0.022225*t6754 + t1605*t63*t826;
  p_output1(11)=0;
  p_output1(12)=t1376*t1398*t1605*t63 + t1605*t1619*t2042*t63 + t2414*t6969 - 0.150246*(t1162*t1398*t1605*t63 + t1110*t6969) + t2912*t7110 + t3440*t7260 + t4129*t7273 + t4467*t7377 - 0.022225*(-1.*t4071*t7273 + t3736*t7377) - 0.86008*(t3736*t7273 + t4071*t7377) - 1.*t1410*t1605*t826;
  p_output1(13)=t2414*t7493 + t2912*t7545 + t3440*t7624 + t4129*t7656 + t4467*t7670 - 0.022225*(-1.*t4071*t7656 + t3736*t7670) - 0.86008*(t3736*t7656 + t4071*t7670) + t1376*t1398*t63*t835 + t1619*t2042*t63*t835 - 1.*t1410*t826*t835 - 0.150246*(t1110*t7493 + t1162*t1398*t63*t835);
  p_output1(14)=-1.*t1376*t1398*t1410 - 1.*t1410*t1619*t2042 + t2414*t7768 - 0.150246*(-1.*t1162*t1398*t1410 + t1110*t7768) + t2912*t7785 + t3440*t7863 + t4129*t7873 + t4467*t7888 - 0.022225*(-1.*t4071*t7873 + t3736*t7888) - 0.86008*(t3736*t7873 + t4071*t7888) - 1.*t63*t826;
  p_output1(15)=t2042*t5550 + t2414*t473*t5550 + t2912*t5550*t587 + t1376*t7983 - 0.150246*(t1110*t473*t5550 + t1162*t7983) + t3440*t8010 + t4129*t8047 + t4467*t8079 - 0.022225*(-1.*t4071*t8047 + t3736*t8079) - 0.86008*(t3736*t8047 + t4071*t8079);
  p_output1(16)=t1376*t2252 + t2042*t8151 + t2414*t473*t8151 + t2912*t587*t8151 - 0.150246*(t1162*t2252 + t1110*t473*t8151) + t3440*t8236 + t4129*t8261 + t4467*t8273 - 0.022225*(-1.*t4071*t8261 + t3736*t8273) - 0.86008*(t3736*t8261 + t4071*t8273);
  p_output1(17)=-1.*t1376*t1619*t63 + t1398*t2042*t63 + t1398*t2414*t473*t63 + t1398*t2912*t587*t63 - 0.150246*(-1.*t1162*t1619*t63 + t1110*t1398*t473*t63) + t3440*t8371 + t4129*t8386 + t4467*t8403 - 0.022225*(-1.*t4071*t8386 + t3736*t8403) - 0.86008*(t3736*t8386 + t4071*t8403);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t2912*t5929 + t1605*t63*t8441 + t5715*t8450 - 0.150246*t1110*t8475 + t2414*t8475 - 1.*t1162*t3440*t8475 + t4129*t8496 + t4467*t8511 - 0.022225*(-1.*t4071*t8496 + t3736*t8511) - 0.86008*(t3736*t8496 + t4071*t8511);
  p_output1(34)=t63*t835*t8441 + t8450*t8578 + t2912*t8606 - 0.150246*t1110*t8620 + t2414*t8620 - 1.*t1162*t3440*t8620 + t4129*t8639 + t4467*t8663 - 0.022225*(-1.*t4071*t8639 + t3736*t8663) - 0.86008*(t3736*t8639 + t4071*t8663);
  p_output1(35)=-1.*t1410*t8441 + t1619*t63*t8450 + t2912*t8721 - 0.150246*t1110*t8731 + t2414*t8731 - 1.*t1162*t3440*t8731 + t4129*t8769 + t4467*t8786 - 0.022225*(-1.*t4071*t8769 + t3736*t8786) - 0.86008*(t3736*t8769 + t4071*t8786);
  p_output1(36)=-0.150246*t6302 + t5550*t8828 + t5929*t8847 + t3440*t8873 + t2634*t4129*t8873 - 1.*t2806*t4467*t8873 - 0.022225*(-1.*t2806*t3736*t8873 - 1.*t2634*t4071*t8873) - 0.86008*(t2634*t3736*t8873 - 1.*t2806*t4071*t8873);
  p_output1(37)=t8151*t8828 + t8606*t8847 + t3440*t8977 + t2634*t4129*t8977 - 1.*t2806*t4467*t8977 - 0.022225*(-1.*t2806*t3736*t8977 - 1.*t2634*t4071*t8977) - 0.86008*(t2634*t3736*t8977 - 1.*t2806*t4071*t8977) - 0.150246*t9007;
  p_output1(38)=t1398*t63*t8828 + t8721*t8847 + t3440*t9063 + t2634*t4129*t9063 - 1.*t2806*t4467*t9063 - 0.022225*(-1.*t2806*t3736*t9063 - 1.*t2634*t4071*t9063) - 0.86008*(t2634*t3736*t9063 - 1.*t2806*t4071*t9063) - 0.150246*t9100;
  p_output1(39)=t4129*t6659 + t6098*t9163 + t6302*t9201 + t4467*t9222 - 0.86008*(t6681 + t4071*t9222) - 0.022225*(t3736*t9222 + t9243);
  p_output1(40)=t9007*t9201 + t9163*t9264 + t4467*t9287 + t4129*t9303 - 0.86008*(t4071*t9287 + t9316) - 0.022225*(t3736*t9287 + t9332);
  p_output1(41)=t9100*t9201 + t9163*t9352 + t4467*t9376 + t4129*t9410 - 0.86008*(t4071*t9376 + t9431) - 0.022225*(t3736*t9376 + t9450);
  p_output1(42)=-0.86008*t6754 - 0.022225*(-1.*t3736*t6473 + t9243) + t6473*t9463 + t6659*t9470;
  p_output1(43)=t9303*t9470 + t9463*t9514 - 0.022225*(t9332 - 1.*t3736*t9514) - 0.86008*(t9316 - 1.*t4071*t9514);
  p_output1(44)=t9410*t9470 + t9463*t9618 - 0.022225*(t9450 - 1.*t3736*t9618) - 0.86008*(t9431 - 1.*t4071*t9618);
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rKnee(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
