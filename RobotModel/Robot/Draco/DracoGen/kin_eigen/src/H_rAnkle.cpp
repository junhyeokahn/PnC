/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:28 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_rAnkle.h"

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
  double t646;
  double t1043;
  double t1178;
  double t1049;
  double t1188;
  double t720;
  double t1039;
  double t637;
  double t1084;
  double t1218;
  double t1350;
  double t1577;
  double t768;
  double t1440;
  double t1466;
  double t593;
  double t1585;
  double t1597;
  double t1623;
  double t1629;
  double t1631;
  double t1660;
  double t1771;
  double t1774;
  double t1799;
  double t1813;
  double t1857;
  double t2012;
  double t1498;
  double t1867;
  double t1895;
  double t566;
  double t2013;
  double t2024;
  double t2028;
  double t2348;
  double t2002;
  double t2060;
  double t2076;
  double t287;
  double t2410;
  double t2443;
  double t2467;
  double t2784;
  double t2805;
  double t2865;
  double t2767;
  double t2870;
  double t2871;
  double t2950;
  double t2958;
  double t3037;
  double t3049;
  double t3090;
  double t3094;
  double t3095;
  double t3160;
  double t3171;
  double t2911;
  double t3194;
  double t3208;
  double t3262;
  double t3379;
  double t3419;
  double t3261;
  double t3434;
  double t3587;
  double t3634;
  double t3781;
  double t3955;
  double t4115;
  double t4121;
  double t4179;
  double t4193;
  double t4204;
  double t4212;
  double t4228;
  double t4240;
  double t4242;
  double t4190;
  double t4322;
  double t4327;
  double t4372;
  double t4377;
  double t4385;
  double t4333;
  double t4409;
  double t4441;
  double t4524;
  double t4535;
  double t4641;
  double t2294;
  double t2479;
  double t3606;
  double t3956;
  double t4498;
  double t4649;
  double t5683;
  double t5699;
  double t5918;
  double t5929;
  double t4737;
  double t4750;
  double t4865;
  double t6118;
  double t6131;
  double t6236;
  double t6239;
  double t6295;
  double t6302;
  double t5285;
  double t2480;
  double t2617;
  double t2619;
  double t5710;
  double t5727;
  double t5768;
  double t5940;
  double t5993;
  double t6003;
  double t6010;
  double t6015;
  double t6016;
  double t6098;
  double t6103;
  double t6105;
  double t6156;
  double t6158;
  double t6166;
  double t4905;
  double t4979;
  double t4988;
  double t6202;
  double t6211;
  double t6213;
  double t6241;
  double t6244;
  double t6255;
  double t6270;
  double t6272;
  double t6284;
  double t6371;
  double t6377;
  double t6378;
  double t6391;
  double t6425;
  double t6436;
  double t5380;
  double t4019;
  double t4045;
  double t4094;
  double t5002;
  double t5059;
  double t5068;
  double t5596;
  double t4659;
  double t4696;
  double t4712;
  t646 = Cos(var1[3]);
  t1043 = Cos(var1[5]);
  t1178 = Sin(var1[4]);
  t1049 = Sin(var1[3]);
  t1188 = Sin(var1[5]);
  t720 = Cos(var1[4]);
  t1039 = Sin(var1[11]);
  t637 = Cos(var1[11]);
  t1084 = -1.*t1043*t1049;
  t1218 = t646*t1178*t1188;
  t1350 = t1084 + t1218;
  t1577 = Cos(var1[13]);
  t768 = t637*t646*t720;
  t1440 = t1039*t1350;
  t1466 = t768 + t1440;
  t593 = Sin(var1[13]);
  t1585 = Cos(var1[12]);
  t1597 = t646*t1043*t1178;
  t1623 = t1049*t1188;
  t1629 = t1597 + t1623;
  t1631 = t1585*t1629;
  t1660 = Sin(var1[12]);
  t1771 = -1.*t646*t720*t1039;
  t1774 = t637*t1350;
  t1799 = t1771 + t1774;
  t1813 = -1.*t1660*t1799;
  t1857 = t1631 + t1813;
  t2012 = Cos(var1[14]);
  t1498 = t593*t1466;
  t1867 = t1577*t1857;
  t1895 = t1498 + t1867;
  t566 = Sin(var1[14]);
  t2013 = t1577*t1466;
  t2024 = -1.*t593*t1857;
  t2028 = t2013 + t2024;
  t2348 = Cos(var1[15]);
  t2002 = -1.*t566*t1895;
  t2060 = t2012*t2028;
  t2076 = t2002 + t2060;
  t287 = Sin(var1[15]);
  t2410 = t2012*t1895;
  t2443 = t566*t2028;
  t2467 = t2410 + t2443;
  t2784 = t646*t1043;
  t2805 = t1049*t1178*t1188;
  t2865 = t2784 + t2805;
  t2767 = t637*t720*t1049;
  t2870 = t1039*t2865;
  t2871 = t2767 + t2870;
  t2950 = t1043*t1049*t1178;
  t2958 = -1.*t646*t1188;
  t3037 = t2950 + t2958;
  t3049 = t1585*t3037;
  t3090 = -1.*t720*t1039*t1049;
  t3094 = t637*t2865;
  t3095 = t3090 + t3094;
  t3160 = -1.*t1660*t3095;
  t3171 = t3049 + t3160;
  t2911 = t593*t2871;
  t3194 = t1577*t3171;
  t3208 = t2911 + t3194;
  t3262 = t1577*t2871;
  t3379 = -1.*t593*t3171;
  t3419 = t3262 + t3379;
  t3261 = -1.*t566*t3208;
  t3434 = t2012*t3419;
  t3587 = t3261 + t3434;
  t3634 = t2012*t3208;
  t3781 = t566*t3419;
  t3955 = t3634 + t3781;
  t4115 = -1.*t637*t1178;
  t4121 = t720*t1039*t1188;
  t4179 = t4115 + t4121;
  t4193 = t1585*t720*t1043;
  t4204 = t1039*t1178;
  t4212 = t637*t720*t1188;
  t4228 = t4204 + t4212;
  t4240 = -1.*t1660*t4228;
  t4242 = t4193 + t4240;
  t4190 = t593*t4179;
  t4322 = t1577*t4242;
  t4327 = t4190 + t4322;
  t4372 = t1577*t4179;
  t4377 = -1.*t593*t4242;
  t4385 = t4372 + t4377;
  t4333 = -1.*t566*t4327;
  t4409 = t2012*t4385;
  t4441 = t4333 + t4409;
  t4524 = t2012*t4327;
  t4535 = t566*t4385;
  t4641 = t4524 + t4535;
  t2294 = t287*t2076;
  t2479 = t2348*t2467;
  t3606 = t287*t3587;
  t3956 = t2348*t3955;
  t4498 = t287*t4441;
  t4649 = t2348*t4641;
  t5683 = -1.*t637;
  t5699 = 1. + t5683;
  t5918 = -1.*t1585;
  t5929 = 1. + t5918;
  t4737 = t1660*t1629;
  t4750 = t1585*t1799;
  t4865 = t4737 + t4750;
  t6118 = -1.*t1577;
  t6131 = 1. + t6118;
  t6236 = -1.*t2012;
  t6239 = 1. + t6236;
  t6295 = -1.*t2348;
  t6302 = 1. + t6295;
  t5285 = t2294 + t2479;
  t2480 = t2348*t2076;
  t2617 = -1.*t287*t2467;
  t2619 = t2480 + t2617;
  t5710 = -0.0222*t5699;
  t5727 = -0.087*t1039;
  t5768 = 0. + t5710 + t5727;
  t5940 = -0.3151*t5929;
  t5993 = 0.157*t1660;
  t6003 = 0. + t5940 + t5993;
  t6010 = -0.087*t5699;
  t6015 = 0.0222*t1039;
  t6016 = 0. + t6010 + t6015;
  t6098 = -0.157*t5929;
  t6103 = -0.3151*t1660;
  t6105 = 0. + t6098 + t6103;
  t6156 = -0.0222*t6131;
  t6158 = 0.3801*t593;
  t6166 = 0. + t6156 + t6158;
  t4905 = t1660*t3037;
  t4979 = t1585*t3095;
  t4988 = t4905 + t4979;
  t6202 = -0.3801*t6131;
  t6211 = -0.0222*t593;
  t6213 = 0. + t6202 + t6211;
  t6241 = -0.8601*t6239;
  t6244 = -0.0222*t566;
  t6255 = 0. + t6241 + t6244;
  t6270 = -0.0222*t6239;
  t6272 = 0.8601*t566;
  t6284 = 0. + t6270 + t6272;
  t6371 = -0.0211*t6302;
  t6377 = 1.3401*t287;
  t6378 = 0. + t6371 + t6377;
  t6391 = -1.3401*t6302;
  t6425 = -0.0211*t287;
  t6436 = 0. + t6391 + t6425;
  t5380 = t3606 + t3956;
  t4019 = t2348*t3587;
  t4045 = -1.*t287*t3955;
  t4094 = t4019 + t4045;
  t5002 = t720*t1043*t1660;
  t5059 = t1585*t4228;
  t5068 = t5002 + t5059;
  t5596 = t4498 + t4649;
  t4659 = t2348*t4441;
  t4696 = -1.*t287*t4641;
  t4712 = t4659 + t4696;

  p_output1(0)=t2294 + t2479 + 0.000796*t2619;
  p_output1(1)=t3606 + t3956 + 0.000796*t4094;
  p_output1(2)=t4498 + t4649 + 0.000796*t4712;
  p_output1(3)=0.;
  p_output1(4)=t4865;
  p_output1(5)=t4988;
  p_output1(6)=t5068;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2076*t2348 + t2467*t287 + 0.000796*t5285;
  p_output1(9)=-1.*t2348*t3587 + t287*t3955 + 0.000796*t5380;
  p_output1(10)=-1.*t2348*t4441 + t287*t4641 + 0.000796*t5596;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0211*t2619 - 0.16705*t4865 - 1.3401*t5285 + t1629*t6003 + t1350*t6016 + t1799*t6105 + t1466*t6166 + t1857*t6213 + t1895*t6255 + t2028*t6284 + t2076*t6378 + t2467*t6436 + t5768*t646*t720 + var1(0);
  p_output1(13)=0. - 0.0211*t4094 - 0.16705*t4988 - 1.3401*t5380 + t3037*t6003 + t2865*t6016 + t3095*t6105 + t2871*t6166 + t3171*t6213 + t3208*t6255 + t3419*t6284 + t3587*t6378 + t3955*t6436 + t1049*t5768*t720 + var1(1);
  p_output1(14)=0. - 0.0211*t4712 - 0.16705*t5068 - 1.3401*t5596 - 1.*t1178*t5768 + t4228*t6105 + t4179*t6166 + t4242*t6213 + t4327*t6255 + t4385*t6284 + t4441*t6378 + t4641*t6436 + t1043*t6003*t720 + t1188*t6016*t720 + var1(2);
  p_output1(15)=1.;
}


       
void H_rAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
