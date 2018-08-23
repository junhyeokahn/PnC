/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipPitch.h"

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
  double t245;
  double t259;
  double t260;
  double t261;
  double t264;
  double t222;
  double t226;
  double t236;
  double t246;
  double t250;
  double t256;
  double t258;
  double t272;
  double t434;
  double t449;
  double t495;
  double t531;
  double t348;
  double t369;
  double t421;
  double t821;
  double t936;
  double t1064;
  double t2490;
  double t2595;
  double t2758;
  double t2840;
  double t1989;
  double t2380;
  double t2461;
  double t3306;
  double t3527;
  double t3532;
  double t262;
  double t265;
  double t269;
  double t301;
  double t325;
  double t326;
  double t4122;
  double t4260;
  double t4568;
  double t498;
  double t573;
  double t575;
  double t1066;
  double t1201;
  double t1235;
  double t5070;
  double t5155;
  double t5164;
  double t5175;
  double t5180;
  double t5184;
  double t2833;
  double t3077;
  double t3169;
  double t3534;
  double t3552;
  double t3571;
  double t5287;
  double t5316;
  double t5330;
  double t5338;
  double t5345;
  double t5348;
  double t5493;
  double t5494;
  double t5495;
  double t5550;
  double t5554;
  double t5556;
  double t5579;
  double t5580;
  double t5584;
  double t5721;
  double t5725;
  double t5742;
  double t5803;
  double t5808;
  double t5811;
  double t5814;
  double t5820;
  double t5821;
  double t5920;
  double t5926;
  double t5928;
  double t5942;
  double t5943;
  double t5944;
  double t5950;
  double t5952;
  double t5953;
  double t6018;
  double t6019;
  double t6025;
  double t6052;
  double t6053;
  double t6054;
  double t6128;
  double t6129;
  double t6133;
  double t6170;
  double t6176;
  double t6178;
  double t6282;
  double t6283;
  double t6290;
  double t6379;
  double t6397;
  double t6399;
  double t6345;
  double t6349;
  double t6362;
  double t6368;
  double t6369;
  double t6468;
  double t6500;
  double t6506;
  double t6522;
  double t6523;
  double t6539;
  double t6546;
  double t6549;
  double t6646;
  double t6649;
  double t6658;
  double t6670;
  double t6671;
  double t6676;
  double t6758;
  double t6759;
  double t6762;
  double t6727;
  double t6731;
  double t6740;
  double t6744;
  double t6747;
  double t6786;
  double t6793;
  double t6794;
  double t6858;
  double t6860;
  double t6866;
  double t5392;
  double t5395;
  double t5404;
  double t6803;
  double t6810;
  double t6826;
  double t6915;
  double t6921;
  double t6932;
  double t6936;
  double t6944;
  double t7000;
  double t7004;
  double t7011;
  double t6873;
  double t6877;
  double t6880;
  double t7112;
  double t7114;
  double t7119;
  t245 = Sin(var1[3]);
  t259 = Cos(var1[6]);
  t260 = -1.*t259;
  t261 = 1. + t260;
  t264 = Sin(var1[6]);
  t222 = Cos(var1[3]);
  t226 = Cos(var1[5]);
  t236 = -1.*t222*t226;
  t246 = Sin(var1[4]);
  t250 = Sin(var1[5]);
  t256 = -1.*t245*t246*t250;
  t258 = t236 + t256;
  t272 = Cos(var1[4]);
  t434 = Cos(var1[7]);
  t449 = -1.*t434;
  t495 = 1. + t449;
  t531 = Sin(var1[7]);
  t348 = t259*t258;
  t369 = t272*t245*t264;
  t421 = t348 + t369;
  t821 = -1.*t226*t245*t246;
  t936 = t222*t250;
  t1064 = t821 + t936;
  t2490 = Cos(var1[8]);
  t2595 = -1.*t2490;
  t2758 = 1. + t2595;
  t2840 = Sin(var1[8]);
  t1989 = t434*t1064;
  t2380 = -1.*t421*t531;
  t2461 = t1989 + t2380;
  t3306 = -1.*t272*t259*t245;
  t3527 = t258*t264;
  t3532 = t3306 + t3527;
  t262 = 0.087*t261;
  t265 = 0.0222*t264;
  t269 = 0. + t262 + t265;
  t301 = -0.0222*t261;
  t325 = 0.087*t264;
  t326 = 0. + t301 + t325;
  t4122 = -1.*t226*t245;
  t4260 = t222*t246*t250;
  t4568 = t4122 + t4260;
  t498 = 0.157*t495;
  t573 = -0.3151*t531;
  t575 = 0. + t498 + t573;
  t1066 = -0.3151*t495;
  t1201 = -0.157*t531;
  t1235 = 0. + t1066 + t1201;
  t5070 = t259*t4568;
  t5155 = -1.*t222*t272*t264;
  t5164 = t5070 + t5155;
  t5175 = t222*t226*t246;
  t5180 = t245*t250;
  t5184 = t5175 + t5180;
  t2833 = -0.3801*t2758;
  t3077 = -0.0222*t2840;
  t3169 = 0. + t2833 + t3077;
  t3534 = -0.0222*t2758;
  t3552 = 0.3801*t2840;
  t3571 = 0. + t3534 + t3552;
  t5287 = t434*t5184;
  t5316 = -1.*t5164*t531;
  t5330 = t5287 + t5316;
  t5338 = t222*t272*t259;
  t5345 = t4568*t264;
  t5348 = t5338 + t5345;
  t5493 = t222*t272*t259*t250;
  t5494 = t222*t246*t264;
  t5495 = t5493 + t5494;
  t5550 = t222*t272*t226*t434;
  t5554 = -1.*t5495*t531;
  t5556 = t5550 + t5554;
  t5579 = -1.*t222*t259*t246;
  t5580 = t222*t272*t250*t264;
  t5584 = t5579 + t5580;
  t5721 = t272*t259*t245*t250;
  t5725 = t245*t246*t264;
  t5742 = t5721 + t5725;
  t5803 = t272*t226*t434*t245;
  t5808 = -1.*t5742*t531;
  t5811 = t5803 + t5808;
  t5814 = -1.*t259*t245*t246;
  t5820 = t272*t245*t250*t264;
  t5821 = t5814 + t5820;
  t5920 = -1.*t259*t246*t250;
  t5926 = t272*t264;
  t5928 = t5920 + t5926;
  t5942 = -1.*t226*t434*t246;
  t5943 = -1.*t5928*t531;
  t5944 = t5942 + t5943;
  t5950 = -1.*t272*t259;
  t5952 = -1.*t246*t250*t264;
  t5953 = t5950 + t5952;
  t6018 = t226*t245;
  t6019 = -1.*t222*t246*t250;
  t6025 = t6018 + t6019;
  t6052 = t434*t6025;
  t6053 = -1.*t259*t5184*t531;
  t6054 = t6052 + t6053;
  t6128 = t226*t245*t246;
  t6129 = -1.*t222*t250;
  t6133 = t6128 + t6129;
  t6170 = t434*t258;
  t6176 = -1.*t259*t6133*t531;
  t6178 = t6170 + t6176;
  t6282 = -1.*t272*t434*t250;
  t6283 = -1.*t272*t226*t259*t531;
  t6290 = t6282 + t6283;
  t6379 = -1.*t222*t272*t259;
  t6397 = -1.*t4568*t264;
  t6399 = t6379 + t6397;
  t6345 = 0.087*t259;
  t6349 = -0.0222*t264;
  t6362 = t6345 + t6349;
  t6368 = 0.0222*t259;
  t6369 = t6368 + t325;
  t6468 = t222*t226;
  t6500 = t245*t246*t250;
  t6506 = t6468 + t6500;
  t6522 = -1.*t6506*t264;
  t6523 = t3306 + t6522;
  t6539 = t259*t6506;
  t6546 = -1.*t272*t245*t264;
  t6549 = t6539 + t6546;
  t6646 = t259*t246;
  t6649 = -1.*t272*t250*t264;
  t6658 = t6646 + t6649;
  t6670 = t272*t259*t250;
  t6671 = t246*t264;
  t6676 = t6670 + t6671;
  t6758 = -1.*t434*t5164;
  t6759 = -1.*t5184*t531;
  t6762 = t6758 + t6759;
  t6727 = -0.157*t434;
  t6731 = t6727 + t573;
  t6740 = -0.3151*t434;
  t6744 = 0.157*t531;
  t6747 = t6740 + t6744;
  t6786 = -1.*t434*t6549;
  t6793 = -1.*t6133*t531;
  t6794 = t6786 + t6793;
  t6858 = -1.*t434*t6676;
  t6860 = -1.*t272*t226*t531;
  t6866 = t6858 + t6860;
  t5392 = t2490*t5348;
  t5395 = -1.*t5330*t2840;
  t5404 = t5392 + t5395;
  t6803 = t434*t6133;
  t6810 = -1.*t6549*t531;
  t6826 = t6803 + t6810;
  t6915 = -0.0222*t2490;
  t6921 = -0.3801*t2840;
  t6932 = t6915 + t6921;
  t6936 = 0.3801*t2490;
  t6944 = t6936 + t3077;
  t7000 = t272*t259*t245;
  t7004 = t6506*t264;
  t7011 = t7000 + t7004;
  t6873 = t272*t226*t434;
  t6877 = -1.*t6676*t531;
  t6880 = t6873 + t6877;
  t7112 = -1.*t259*t246;
  t7114 = t272*t250*t264;
  t7119 = t7112 + t7114;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1064*t1235 + t258*t269 + t2461*t3169 - 1.*t245*t272*t326 - 0.0222*(-1.*t2461*t2840 + t2490*t3532) - 0.3801*(t2461*t2490 + t2840*t3532) + t3532*t3571 + 0.167*(t421*t434 + t1064*t531) + t421*t575;
  p_output1(10)=t222*t272*t326 + t269*t4568 + t1235*t5184 + 0.167*(t434*t5164 + t5184*t531) + t3169*t5330 + t3571*t5348 - 0.3801*(t2490*t5330 + t2840*t5348) - 0.0222*t5404 + t5164*t575;
  p_output1(11)=0;
  p_output1(12)=t1235*t222*t226*t272 + t222*t250*t269*t272 - 1.*t222*t246*t326 + 0.167*(t222*t226*t272*t531 + t434*t5495) + t3169*t5556 + t3571*t5584 - 0.0222*(-1.*t2840*t5556 + t2490*t5584) - 0.3801*(t2490*t5556 + t2840*t5584) + t5495*t575;
  p_output1(13)=t1235*t226*t245*t272 + t245*t250*t269*t272 - 1.*t245*t246*t326 + 0.167*(t226*t245*t272*t531 + t434*t5742) + t5742*t575 + t3169*t5811 + t3571*t5821 - 0.0222*(-1.*t2840*t5811 + t2490*t5821) - 0.3801*(t2490*t5811 + t2840*t5821);
  p_output1(14)=-1.*t1235*t226*t246 - 1.*t246*t250*t269 - 1.*t272*t326 + t575*t5928 + 0.167*(-1.*t226*t246*t531 + t434*t5928) + t3169*t5944 + t3571*t5953 - 0.0222*(-1.*t2840*t5944 + t2490*t5953) - 0.3801*(t2490*t5944 + t2840*t5953);
  p_output1(15)=t269*t5184 + t264*t3571*t5184 + t259*t5184*t575 + t1235*t6025 + 0.167*(t259*t434*t5184 + t531*t6025) + t3169*t6054 - 0.3801*(t264*t2840*t5184 + t2490*t6054) - 0.0222*(t2490*t264*t5184 - 1.*t2840*t6054);
  p_output1(16)=t1235*t258 + t269*t6133 + t264*t3571*t6133 + t259*t575*t6133 + 0.167*(t258*t531 + t259*t434*t6133) + t3169*t6178 - 0.3801*(t264*t2840*t6133 + t2490*t6178) - 0.0222*(t2490*t264*t6133 - 1.*t2840*t6178);
  p_output1(17)=-1.*t1235*t250*t272 + t226*t269*t272 + t226*t264*t272*t3571 + 0.167*(t226*t259*t272*t434 - 1.*t250*t272*t531) + t226*t259*t272*t575 + t3169*t6290 - 0.3801*(t226*t264*t272*t2840 + t2490*t6290) - 0.0222*(t226*t2490*t264*t272 - 1.*t2840*t6290);
  p_output1(18)=t3571*t5164 + t222*t272*t6362 + t4568*t6369 + 0.167*t434*t6399 - 1.*t3169*t531*t6399 + t575*t6399 - 0.3801*(t2840*t5164 - 1.*t2490*t531*t6399) - 0.0222*(t2490*t5164 + t2840*t531*t6399);
  p_output1(19)=t245*t272*t6362 + t6369*t6506 + 0.167*t434*t6523 - 1.*t3169*t531*t6523 + t575*t6523 + t3571*t6549 - 0.0222*(t2840*t531*t6523 + t2490*t6549) - 0.3801*(-1.*t2490*t531*t6523 + t2840*t6549);
  p_output1(20)=-1.*t246*t6362 + t250*t272*t6369 + 0.167*t434*t6658 - 1.*t3169*t531*t6658 + t575*t6658 + t3571*t6676 - 0.0222*(t2840*t531*t6658 + t2490*t6676) - 0.3801*(-1.*t2490*t531*t6658 + t2840*t6676);
  p_output1(21)=0.167*t5330 + t5184*t6731 + t5164*t6747 - 0.3801*t2490*t6762 + 0.0222*t2840*t6762 + t3169*t6762;
  p_output1(22)=t6133*t6731 + t6549*t6747 - 0.3801*t2490*t6794 + 0.0222*t2840*t6794 + t3169*t6794 + 0.167*t6826;
  p_output1(23)=t226*t272*t6731 + t6676*t6747 - 0.3801*t2490*t6866 + 0.0222*t2840*t6866 + t3169*t6866 + 0.167*t6880;
  p_output1(24)=-0.0222*(-1.*t2490*t5330 - 1.*t2840*t5348) - 0.3801*t5404 + t5330*t6932 + t5348*t6944;
  p_output1(25)=t6826*t6932 + t6944*t7011 - 0.3801*(-1.*t2840*t6826 + t2490*t7011) - 0.0222*(-1.*t2490*t6826 - 1.*t2840*t7011);
  p_output1(26)=t6880*t6932 + t6944*t7119 - 0.3801*(-1.*t2840*t6880 + t2490*t7119) - 0.0222*(-1.*t2490*t6880 - 1.*t2840*t7119);
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
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


       
void Jp_lHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
