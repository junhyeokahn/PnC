/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:17 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_lHipPitch.h"

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
  double t1038;
  double t2471;
  double t2906;
  double t2972;
  double t3048;
  double t247;
  double t248;
  double t284;
  double t1570;
  double t1714;
  double t1716;
  double t2445;
  double t3700;
  double t5135;
  double t5145;
  double t5146;
  double t5161;
  double t4906;
  double t4954;
  double t5001;
  double t5180;
  double t5181;
  double t5193;
  double t5304;
  double t5310;
  double t5315;
  double t5327;
  double t5287;
  double t5295;
  double t5299;
  double t5338;
  double t5345;
  double t5347;
  double t3033;
  double t3077;
  double t3372;
  double t3820;
  double t4125;
  double t4328;
  double t5392;
  double t5394;
  double t5395;
  double t5155;
  double t5164;
  double t5165;
  double t5202;
  double t5205;
  double t5216;
  double t5413;
  double t5425;
  double t5444;
  double t5469;
  double t5477;
  double t5483;
  double t5316;
  double t5330;
  double t5333;
  double t5348;
  double t5349;
  double t5353;
  double t5508;
  double t5509;
  double t5518;
  double t5528;
  double t5530;
  double t5538;
  double t5610;
  double t5611;
  double t5618;
  double t5693;
  double t5695;
  double t5698;
  double t5709;
  double t5715;
  double t5717;
  t1038 = Cos(var1[3]);
  t2471 = Cos(var1[6]);
  t2906 = -1.*t2471;
  t2972 = 1. + t2906;
  t3048 = Sin(var1[6]);
  t247 = Cos(var1[5]);
  t248 = Sin(var1[3]);
  t284 = -1.*t247*t248;
  t1570 = Sin(var1[4]);
  t1714 = Sin(var1[5]);
  t1716 = t1038*t1570*t1714;
  t2445 = t284 + t1716;
  t3700 = Cos(var1[4]);
  t5135 = Cos(var1[7]);
  t5145 = -1.*t5135;
  t5146 = 1. + t5145;
  t5161 = Sin(var1[7]);
  t4906 = t2471*t2445;
  t4954 = -1.*t1038*t3700*t3048;
  t5001 = t4906 + t4954;
  t5180 = t1038*t247*t1570;
  t5181 = t248*t1714;
  t5193 = t5180 + t5181;
  t5304 = Cos(var1[8]);
  t5310 = -1.*t5304;
  t5315 = 1. + t5310;
  t5327 = Sin(var1[8]);
  t5287 = t5135*t5193;
  t5295 = -1.*t5001*t5161;
  t5299 = t5287 + t5295;
  t5338 = t1038*t3700*t2471;
  t5345 = t2445*t3048;
  t5347 = t5338 + t5345;
  t3033 = 0.087*t2972;
  t3077 = 0.0222*t3048;
  t3372 = 0. + t3033 + t3077;
  t3820 = -0.0222*t2972;
  t4125 = 0.087*t3048;
  t4328 = 0. + t3820 + t4125;
  t5392 = t1038*t247;
  t5394 = t248*t1570*t1714;
  t5395 = t5392 + t5394;
  t5155 = 0.157*t5146;
  t5164 = -0.3151*t5161;
  t5165 = 0. + t5155 + t5164;
  t5202 = -0.3151*t5146;
  t5205 = -0.157*t5161;
  t5216 = 0. + t5202 + t5205;
  t5413 = t2471*t5395;
  t5425 = -1.*t3700*t248*t3048;
  t5444 = t5413 + t5425;
  t5469 = t247*t248*t1570;
  t5477 = -1.*t1038*t1714;
  t5483 = t5469 + t5477;
  t5316 = -0.3801*t5315;
  t5330 = -0.0222*t5327;
  t5333 = 0. + t5316 + t5330;
  t5348 = -0.0222*t5315;
  t5349 = 0.3801*t5327;
  t5353 = 0. + t5348 + t5349;
  t5508 = t5135*t5483;
  t5509 = -1.*t5444*t5161;
  t5518 = t5508 + t5509;
  t5528 = t3700*t2471*t248;
  t5530 = t5395*t3048;
  t5538 = t5528 + t5530;
  t5610 = t3700*t2471*t1714;
  t5611 = t1570*t3048;
  t5618 = t5610 + t5611;
  t5693 = t3700*t247*t5135;
  t5695 = -1.*t5618*t5161;
  t5698 = t5693 + t5695;
  t5709 = -1.*t2471*t1570;
  t5715 = t3700*t1714*t3048;
  t5717 = t5709 + t5715;

  p_output1(0)=0. + t2445*t3372 + t1038*t3700*t4328 + t5001*t5165 + 0.167*(t5001*t5135 + t5161*t5193) + t5193*t5216 + t5299*t5333 - 0.0222*(-1.*t5299*t5327 + t5304*t5347) - 0.3801*(t5299*t5304 + t5327*t5347) + t5347*t5353 + var1(0);
  p_output1(1)=0. + t248*t3700*t4328 + t3372*t5395 + t5165*t5444 + t5216*t5483 + 0.167*(t5135*t5444 + t5161*t5483) + t5333*t5518 + t5353*t5538 - 0.0222*(-1.*t5327*t5518 + t5304*t5538) - 0.3801*(t5304*t5518 + t5327*t5538) + var1(1);
  p_output1(2)=0. + t1714*t3372*t3700 - 1.*t1570*t4328 + t247*t3700*t5216 + t5165*t5618 + 0.167*(t247*t3700*t5161 + t5135*t5618) + t5333*t5698 + t5353*t5717 - 0.0222*(-1.*t5327*t5698 + t5304*t5717) - 0.3801*(t5304*t5698 + t5327*t5717) + var1(2);
}


       
void p_lHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
