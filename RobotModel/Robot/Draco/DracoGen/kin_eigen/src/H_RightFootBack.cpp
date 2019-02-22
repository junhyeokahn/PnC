/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:37 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_RightFootBack.h"

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
  double t233;
  double t306;
  double t409;
  double t314;
  double t440;
  double t140;
  double t575;
  double t589;
  double t595;
  double t616;
  double t658;
  double t661;
  double t745;
  double t346;
  double t448;
  double t532;
  double t605;
  double t827;
  double t848;
  double t1925;
  double t1764;
  double t1814;
  double t1825;
  double t1669;
  double t1942;
  double t1986;
  double t1993;
  double t2229;
  double t1901;
  double t2018;
  double t2124;
  double t1630;
  double t2283;
  double t2310;
  double t2312;
  double t2433;
  double t2203;
  double t2329;
  double t2351;
  double t1591;
  double t2439;
  double t2440;
  double t2481;
  double t1188;
  double t1227;
  double t1242;
  double t1075;
  double t1095;
  double t1098;
  double t1182;
  double t1371;
  double t1428;
  double t2768;
  double t2814;
  double t2850;
  double t2869;
  double t2893;
  double t2932;
  double t2858;
  double t2935;
  double t2944;
  double t2958;
  double t2974;
  double t2983;
  double t2956;
  double t3027;
  double t3036;
  double t3060;
  double t3064;
  double t3072;
  double t1492;
  double t1493;
  double t1502;
  double t3277;
  double t3307;
  double t3310;
  double t3320;
  double t3339;
  double t3352;
  double t3311;
  double t3375;
  double t3510;
  double t3559;
  double t3567;
  double t3597;
  double t3527;
  double t3598;
  double t3623;
  double t3626;
  double t3701;
  double t3706;
  double t4199;
  double t4253;
  double t4457;
  double t4458;
  double t573;
  double t972;
  double t1068;
  double t4699;
  double t4735;
  double t4921;
  double t4926;
  double t5200;
  double t5219;
  double t3895;
  double t3930;
  double t3934;
  double t2517;
  double t2540;
  double t2609;
  double t4280;
  double t4305;
  double t4416;
  double t4469;
  double t4472;
  double t4497;
  double t4567;
  double t4574;
  double t4584;
  double t4622;
  double t4668;
  double t4675;
  double t4760;
  double t4778;
  double t4827;
  double t1173;
  double t1434;
  double t1439;
  double t4848;
  double t4898;
  double t4901;
  double t4994;
  double t5051;
  double t5081;
  double t5113;
  double t5151;
  double t5155;
  double t5224;
  double t5225;
  double t5230;
  double t5255;
  double t5286;
  double t5300;
  double t4007;
  double t4008;
  double t4009;
  double t3106;
  double t3163;
  double t3242;
  double t1450;
  double t1509;
  double t1580;
  double t4101;
  double t4102;
  double t4118;
  double t3714;
  double t3719;
  double t3741;
  t233 = Cos(var1[3]);
  t306 = Cos(var1[5]);
  t409 = Sin(var1[3]);
  t314 = Sin(var1[4]);
  t440 = Sin(var1[5]);
  t140 = Sin(var1[12]);
  t575 = Cos(var1[12]);
  t589 = Cos(var1[4]);
  t595 = Sin(var1[11]);
  t616 = Cos(var1[11]);
  t658 = -1.*t306*t409;
  t661 = t233*t314*t440;
  t745 = t658 + t661;
  t346 = t233*t306*t314;
  t448 = t409*t440;
  t532 = t346 + t448;
  t605 = -1.*t233*t589*t595;
  t827 = t616*t745;
  t848 = t605 + t827;
  t1925 = Cos(var1[13]);
  t1764 = t616*t233*t589;
  t1814 = t595*t745;
  t1825 = t1764 + t1814;
  t1669 = Sin(var1[13]);
  t1942 = t575*t532;
  t1986 = -1.*t140*t848;
  t1993 = t1942 + t1986;
  t2229 = Cos(var1[14]);
  t1901 = t1669*t1825;
  t2018 = t1925*t1993;
  t2124 = t1901 + t2018;
  t1630 = Sin(var1[14]);
  t2283 = t1925*t1825;
  t2310 = -1.*t1669*t1993;
  t2312 = t2283 + t2310;
  t2433 = Cos(var1[15]);
  t2203 = -1.*t1630*t2124;
  t2329 = t2229*t2312;
  t2351 = t2203 + t2329;
  t1591 = Sin(var1[15]);
  t2439 = t2229*t2124;
  t2440 = t1630*t2312;
  t2481 = t2439 + t2440;
  t1188 = t233*t306;
  t1227 = t409*t314*t440;
  t1242 = t1188 + t1227;
  t1075 = t306*t409*t314;
  t1095 = -1.*t233*t440;
  t1098 = t1075 + t1095;
  t1182 = -1.*t589*t595*t409;
  t1371 = t616*t1242;
  t1428 = t1182 + t1371;
  t2768 = t616*t589*t409;
  t2814 = t595*t1242;
  t2850 = t2768 + t2814;
  t2869 = t575*t1098;
  t2893 = -1.*t140*t1428;
  t2932 = t2869 + t2893;
  t2858 = t1669*t2850;
  t2935 = t1925*t2932;
  t2944 = t2858 + t2935;
  t2958 = t1925*t2850;
  t2974 = -1.*t1669*t2932;
  t2983 = t2958 + t2974;
  t2956 = -1.*t1630*t2944;
  t3027 = t2229*t2983;
  t3036 = t2956 + t3027;
  t3060 = t2229*t2944;
  t3064 = t1630*t2983;
  t3072 = t3060 + t3064;
  t1492 = t595*t314;
  t1493 = t616*t589*t440;
  t1502 = t1492 + t1493;
  t3277 = -1.*t616*t314;
  t3307 = t589*t595*t440;
  t3310 = t3277 + t3307;
  t3320 = t575*t589*t306;
  t3339 = -1.*t140*t1502;
  t3352 = t3320 + t3339;
  t3311 = t1669*t3310;
  t3375 = t1925*t3352;
  t3510 = t3311 + t3375;
  t3559 = t1925*t3310;
  t3567 = -1.*t1669*t3352;
  t3597 = t3559 + t3567;
  t3527 = -1.*t1630*t3510;
  t3598 = t2229*t3597;
  t3623 = t3527 + t3598;
  t3626 = t2229*t3510;
  t3701 = t1630*t3597;
  t3706 = t3626 + t3701;
  t4199 = -1.*t616;
  t4253 = 1. + t4199;
  t4457 = -1.*t575;
  t4458 = 1. + t4457;
  t573 = t140*t532;
  t972 = t575*t848;
  t1068 = t573 + t972;
  t4699 = -1.*t1925;
  t4735 = 1. + t4699;
  t4921 = -1.*t2229;
  t4926 = 1. + t4921;
  t5200 = -1.*t2433;
  t5219 = 1. + t5200;
  t3895 = t1591*t2351;
  t3930 = t2433*t2481;
  t3934 = t3895 + t3930;
  t2517 = t2433*t2351;
  t2540 = -1.*t1591*t2481;
  t2609 = t2517 + t2540;
  t4280 = -0.0222*t4253;
  t4305 = -0.087*t595;
  t4416 = 0. + t4280 + t4305;
  t4469 = -0.3151*t4458;
  t4472 = 0.157*t140;
  t4497 = 0. + t4469 + t4472;
  t4567 = -0.087*t4253;
  t4574 = 0.0222*t595;
  t4584 = 0. + t4567 + t4574;
  t4622 = -0.157*t4458;
  t4668 = -0.3151*t140;
  t4675 = 0. + t4622 + t4668;
  t4760 = -0.0222*t4735;
  t4778 = 0.3801*t1669;
  t4827 = 0. + t4760 + t4778;
  t1173 = t140*t1098;
  t1434 = t575*t1428;
  t1439 = t1173 + t1434;
  t4848 = -0.3801*t4735;
  t4898 = -0.0222*t1669;
  t4901 = 0. + t4848 + t4898;
  t4994 = -0.8601*t4926;
  t5051 = -0.0222*t1630;
  t5081 = 0. + t4994 + t5051;
  t5113 = -0.0222*t4926;
  t5151 = 0.8601*t1630;
  t5155 = 0. + t5113 + t5151;
  t5224 = -0.0211*t5219;
  t5225 = 1.3401*t1591;
  t5230 = 0. + t5224 + t5225;
  t5255 = -1.3401*t5219;
  t5286 = -0.0211*t1591;
  t5300 = 0. + t5255 + t5286;
  t4007 = t1591*t3036;
  t4008 = t2433*t3072;
  t4009 = t4007 + t4008;
  t3106 = t2433*t3036;
  t3163 = -1.*t1591*t3072;
  t3242 = t3106 + t3163;
  t1450 = t589*t306*t140;
  t1509 = t575*t1502;
  t1580 = t1450 + t1509;
  t4101 = t1591*t3623;
  t4102 = t2433*t3706;
  t4118 = t4101 + t4102;
  t3714 = t2433*t3623;
  t3719 = -1.*t1591*t3706;
  t3741 = t3714 + t3719;

  p_output1(0)=t1068;
  p_output1(1)=t1439;
  p_output1(2)=t1580;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1591*t2351 - 1.*t2433*t2481 - 0.000796*t2609;
  p_output1(5)=-1.*t1591*t3036 - 1.*t2433*t3072 - 0.000796*t3242;
  p_output1(6)=-1.*t1591*t3623 - 1.*t2433*t3706 - 0.000796*t3741;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2351*t2433 + t1591*t2481 + 0.000796*t3934;
  p_output1(9)=-1.*t2433*t3036 + t1591*t3072 + 0.000796*t4009;
  p_output1(10)=-1.*t2433*t3623 + t1591*t3706 + 0.000796*t4118;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.24205*t1068 + 0.043912*t2609 - 1.325152*t3934 + t1825*t4827 + t1993*t4901 + t2124*t5081 + t2312*t5155 + t2351*t5230 + t2481*t5300 + t4497*t532 + t233*t4416*t589 + t4584*t745 + t4675*t848 + var1(0);
  p_output1(13)=0. - 0.24205*t1439 + 0.043912*t3242 - 1.325152*t4009 + t1098*t4497 + t1242*t4584 + t1428*t4675 + t2850*t4827 + t2932*t4901 + t2944*t5081 + t2983*t5155 + t3036*t5230 + t3072*t5300 + t409*t4416*t589 + var1(1);
  p_output1(14)=0. - 0.24205*t1580 + 0.043912*t3741 - 1.325152*t4118 - 1.*t314*t4416 + t1502*t4675 + t3310*t4827 + t3352*t4901 + t3510*t5081 + t3597*t5155 + t3623*t5230 + t3706*t5300 + t306*t4497*t589 + t440*t4584*t589 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootBack(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
