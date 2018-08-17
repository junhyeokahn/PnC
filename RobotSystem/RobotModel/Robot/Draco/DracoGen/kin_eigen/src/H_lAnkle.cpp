/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:51 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lAnkle.h"

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
  double t527;
  double t701;
  double t512;
  double t673;
  double t707;
  double t1254;
  double t951;
  double t957;
  double t1023;
  double t1099;
  double t1259;
  double t401;
  double t1488;
  double t1534;
  double t1567;
  double t444;
  double t690;
  double t769;
  double t830;
  double t869;
  double t1107;
  double t1275;
  double t1281;
  double t1294;
  double t1379;
  double t1418;
  double t1573;
  double t1709;
  double t1430;
  double t1606;
  double t1679;
  double t275;
  double t1727;
  double t1748;
  double t1798;
  double t2047;
  double t1702;
  double t1994;
  double t1998;
  double t252;
  double t2087;
  double t2113;
  double t2138;
  double t2529;
  double t2548;
  double t2563;
  double t2729;
  double t2738;
  double t2815;
  double t2458;
  double t2460;
  double t2483;
  double t2510;
  double t2565;
  double t2660;
  double t2673;
  double t2675;
  double t2719;
  double t2722;
  double t2836;
  double t2839;
  double t2944;
  double t3104;
  double t3132;
  double t2856;
  double t3147;
  double t3212;
  double t3467;
  double t3477;
  double t3487;
  double t3722;
  double t3735;
  double t3780;
  double t3614;
  double t3621;
  double t3648;
  double t3658;
  double t3686;
  double t3701;
  double t3706;
  double t3799;
  double t3800;
  double t3848;
  double t3849;
  double t3871;
  double t3808;
  double t3943;
  double t3966;
  double t4007;
  double t4038;
  double t4041;
  double t2023;
  double t2188;
  double t3437;
  double t3494;
  double t4001;
  double t4057;
  double t4644;
  double t4645;
  double t4904;
  double t4920;
  double t4210;
  double t4219;
  double t4221;
  double t5061;
  double t5072;
  double t5266;
  double t5300;
  double t5587;
  double t5603;
  double t4443;
  double t2195;
  double t2318;
  double t2353;
  double t4657;
  double t4664;
  double t4682;
  double t4708;
  double t4723;
  double t4749;
  double t4946;
  double t4947;
  double t4949;
  double t4990;
  double t5033;
  double t5037;
  double t4224;
  double t4235;
  double t4282;
  double t5074;
  double t5092;
  double t5141;
  double t5228;
  double t5245;
  double t5250;
  double t5316;
  double t5319;
  double t5340;
  double t5356;
  double t5394;
  double t5555;
  double t5627;
  double t5646;
  double t5665;
  double t5686;
  double t5696;
  double t5697;
  double t4530;
  double t3506;
  double t3534;
  double t3546;
  double t4287;
  double t4318;
  double t4323;
  double t4585;
  double t4100;
  double t4142;
  double t4159;
  t527 = Cos(var1[5]);
  t701 = Sin(var1[3]);
  t512 = Cos(var1[3]);
  t673 = Sin(var1[4]);
  t707 = Sin(var1[5]);
  t1254 = Cos(var1[4]);
  t951 = Cos(var1[6]);
  t957 = -1.*t527*t701;
  t1023 = t512*t673*t707;
  t1099 = t957 + t1023;
  t1259 = Sin(var1[6]);
  t401 = Cos(var1[8]);
  t1488 = t512*t1254*t951;
  t1534 = t1099*t1259;
  t1567 = t1488 + t1534;
  t444 = Cos(var1[7]);
  t690 = t512*t527*t673;
  t769 = t701*t707;
  t830 = t690 + t769;
  t869 = t444*t830;
  t1107 = t951*t1099;
  t1275 = -1.*t512*t1254*t1259;
  t1281 = t1107 + t1275;
  t1294 = Sin(var1[7]);
  t1379 = -1.*t1281*t1294;
  t1418 = t869 + t1379;
  t1573 = Sin(var1[8]);
  t1709 = Cos(var1[9]);
  t1430 = t401*t1418;
  t1606 = t1567*t1573;
  t1679 = t1430 + t1606;
  t275 = Sin(var1[9]);
  t1727 = t401*t1567;
  t1748 = -1.*t1418*t1573;
  t1798 = t1727 + t1748;
  t2047 = Cos(var1[10]);
  t1702 = -1.*t275*t1679;
  t1994 = t1709*t1798;
  t1998 = t1702 + t1994;
  t252 = Sin(var1[10]);
  t2087 = t1709*t1679;
  t2113 = t275*t1798;
  t2138 = t2087 + t2113;
  t2529 = t512*t527;
  t2548 = t701*t673*t707;
  t2563 = t2529 + t2548;
  t2729 = t1254*t951*t701;
  t2738 = t2563*t1259;
  t2815 = t2729 + t2738;
  t2458 = t527*t701*t673;
  t2460 = -1.*t512*t707;
  t2483 = t2458 + t2460;
  t2510 = t444*t2483;
  t2565 = t951*t2563;
  t2660 = -1.*t1254*t701*t1259;
  t2673 = t2565 + t2660;
  t2675 = -1.*t2673*t1294;
  t2719 = t2510 + t2675;
  t2722 = t401*t2719;
  t2836 = t2815*t1573;
  t2839 = t2722 + t2836;
  t2944 = t401*t2815;
  t3104 = -1.*t2719*t1573;
  t3132 = t2944 + t3104;
  t2856 = -1.*t275*t2839;
  t3147 = t1709*t3132;
  t3212 = t2856 + t3147;
  t3467 = t1709*t2839;
  t3477 = t275*t3132;
  t3487 = t3467 + t3477;
  t3722 = -1.*t951*t673;
  t3735 = t1254*t707*t1259;
  t3780 = t3722 + t3735;
  t3614 = t1254*t527*t444;
  t3621 = t1254*t951*t707;
  t3648 = t673*t1259;
  t3658 = t3621 + t3648;
  t3686 = -1.*t3658*t1294;
  t3701 = t3614 + t3686;
  t3706 = t401*t3701;
  t3799 = t3780*t1573;
  t3800 = t3706 + t3799;
  t3848 = t401*t3780;
  t3849 = -1.*t3701*t1573;
  t3871 = t3848 + t3849;
  t3808 = -1.*t275*t3800;
  t3943 = t1709*t3871;
  t3966 = t3808 + t3943;
  t4007 = t1709*t3800;
  t4038 = t275*t3871;
  t4041 = t4007 + t4038;
  t2023 = t252*t1998;
  t2188 = t2047*t2138;
  t3437 = t252*t3212;
  t3494 = t2047*t3487;
  t4001 = t252*t3966;
  t4057 = t2047*t4041;
  t4644 = -1.*t951;
  t4645 = 1. + t4644;
  t4904 = -1.*t444;
  t4920 = 1. + t4904;
  t4210 = t444*t1281;
  t4219 = t830*t1294;
  t4221 = t4210 + t4219;
  t5061 = -1.*t401;
  t5072 = 1. + t5061;
  t5266 = -1.*t1709;
  t5300 = 1. + t5266;
  t5587 = -1.*t2047;
  t5603 = 1. + t5587;
  t4443 = t2023 + t2188;
  t2195 = t2047*t1998;
  t2318 = -1.*t252*t2138;
  t2353 = t2195 + t2318;
  t4657 = 0.087004*t4645;
  t4664 = 0.022225*t1259;
  t4682 = 0. + t4657 + t4664;
  t4708 = -0.022225*t4645;
  t4723 = 0.087004*t1259;
  t4749 = 0. + t4708 + t4723;
  t4946 = 0.157004*t4920;
  t4947 = -0.31508*t1294;
  t4949 = 0. + t4946 + t4947;
  t4990 = -0.31508*t4920;
  t5033 = -0.157004*t1294;
  t5037 = 0. + t4990 + t5033;
  t4224 = t444*t2673;
  t4235 = t2483*t1294;
  t4282 = t4224 + t4235;
  t5074 = -0.38008*t5072;
  t5092 = -0.022225*t1573;
  t5141 = 0. + t5074 + t5092;
  t5228 = -0.022225*t5072;
  t5245 = 0.38008*t1573;
  t5250 = 0. + t5228 + t5245;
  t5316 = -0.86008*t5300;
  t5319 = -0.022225*t275;
  t5340 = 0. + t5316 + t5319;
  t5356 = -0.022225*t5300;
  t5394 = 0.86008*t275;
  t5555 = 0. + t5356 + t5394;
  t5627 = -0.021147*t5603;
  t5646 = 1.34008*t252;
  t5665 = 0. + t5627 + t5646;
  t5686 = -1.34008*t5603;
  t5696 = -0.021147*t252;
  t5697 = 0. + t5686 + t5696;
  t4530 = t3437 + t3494;
  t3506 = t2047*t3212;
  t3534 = -1.*t252*t3487;
  t3546 = t3506 + t3534;
  t4287 = t444*t3658;
  t4318 = t1254*t527*t1294;
  t4323 = t4287 + t4318;
  t4585 = t4001 + t4057;
  t4100 = t2047*t3966;
  t4142 = -1.*t252*t4041;
  t4159 = t4100 + t4142;

  p_output1(0)=t2023 + t2188 + 0.000796*t2353;
  p_output1(1)=t3437 + t3494 + 0.000796*t3546;
  p_output1(2)=t4001 + t4057 + 0.000796*t4159;
  p_output1(3)=0.;
  p_output1(4)=t4221;
  p_output1(5)=t4282;
  p_output1(6)=t4323;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1998*t2047 + t2138*t252 + 0.000796*t4443;
  p_output1(9)=-1.*t2047*t3212 + t252*t3487 + 0.000796*t4530;
  p_output1(10)=-1.*t2047*t3966 + t252*t4041 + 0.000796*t4585;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.021147*t2353 + 0.167004*t4221 - 1.34008*t4443 + t1099*t4682 + t1281*t4949 + t1254*t4749*t512 + t1418*t5141 + t1567*t5250 + t1679*t5340 + t1798*t5555 + t1998*t5665 + t2138*t5697 + t5037*t830 + var1(0);
  p_output1(13)=0. - 0.021147*t3546 + 0.167004*t4282 - 1.34008*t4530 + t2563*t4682 + t2673*t4949 + t2483*t5037 + t2719*t5141 + t2815*t5250 + t2839*t5340 + t3132*t5555 + t3212*t5665 + t3487*t5697 + t1254*t4749*t701 + var1(1);
  p_output1(14)=0. - 0.021147*t4159 + 0.167004*t4323 - 1.34008*t4585 + t3658*t4949 + t3701*t5141 + t3780*t5250 + t1254*t5037*t527 + t3800*t5340 + t3871*t5555 + t3966*t5665 + t4041*t5697 - 1.*t4749*t673 + t1254*t4682*t707 + var1(2);
  p_output1(15)=1.;
}


       
void H_lAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
