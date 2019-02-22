/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:26 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_rKnee.h"

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
  double t324;
  double t690;
  double t751;
  double t827;
  double t2109;
  double t5417;
  double t5546;
  double t5505;
  double t5562;
  double t3050;
  double t3112;
  double t4290;
  double t4439;
  double t549;
  double t5633;
  double t5652;
  double t5657;
  double t5541;
  double t5597;
  double t5607;
  double t5677;
  double t5733;
  double t5752;
  double t5781;
  double t5785;
  double t5788;
  double t5792;
  double t5796;
  double t5797;
  double t5803;
  double t6009;
  double t6013;
  double t6014;
  double t6028;
  double t6035;
  double t6037;
  double t6059;
  double t6103;
  double t6104;
  double t6110;
  double t6123;
  double t6125;
  double t6126;
  double t2103;
  double t2312;
  double t2395;
  double t4375;
  double t5120;
  double t5318;
  double t5617;
  double t5621;
  double t5625;
  double t5660;
  double t5664;
  double t5670;
  double t6242;
  double t6246;
  double t6255;
  double t5791;
  double t5793;
  double t5795;
  double t6227;
  double t6228;
  double t6230;
  double t6267;
  double t6272;
  double t6273;
  double t5879;
  double t5945;
  double t5996;
  double t6051;
  double t6097;
  double t6099;
  double t6311;
  double t6314;
  double t6316;
  double t6338;
  double t6339;
  double t6340;
  double t6119;
  double t6121;
  double t6122;
  double t6400;
  double t6403;
  double t6414;
  double t6432;
  double t6437;
  double t6448;
  double t6555;
  double t6567;
  double t6570;
  double t6580;
  double t6584;
  double t6585;
  double t6595;
  double t6604;
  double t6606;
  double t6615;
  double t6616;
  double t6617;
  double t6621;
  double t6625;
  double t6634;
  t324 = Cos(var1[3]);
  t690 = Cos(var1[11]);
  t751 = -1.*t690;
  t827 = 1. + t751;
  t2109 = Sin(var1[11]);
  t5417 = Cos(var1[5]);
  t5546 = Sin(var1[3]);
  t5505 = Sin(var1[4]);
  t5562 = Sin(var1[5]);
  t3050 = Cos(var1[12]);
  t3112 = -1.*t3050;
  t4290 = 1. + t3112;
  t4439 = Sin(var1[12]);
  t549 = Cos(var1[4]);
  t5633 = -1.*t5417*t5546;
  t5652 = t324*t5505*t5562;
  t5657 = t5633 + t5652;
  t5541 = t324*t5417*t5505;
  t5597 = t5546*t5562;
  t5607 = t5541 + t5597;
  t5677 = -1.*t324*t549*t2109;
  t5733 = t690*t5657;
  t5752 = t5677 + t5733;
  t5781 = Cos(var1[13]);
  t5785 = -1.*t5781;
  t5788 = 1. + t5785;
  t5792 = Sin(var1[13]);
  t5796 = t690*t324*t549;
  t5797 = t2109*t5657;
  t5803 = t5796 + t5797;
  t6009 = t3050*t5607;
  t6013 = -1.*t4439*t5752;
  t6014 = t6009 + t6013;
  t6028 = Cos(var1[14]);
  t6035 = -1.*t6028;
  t6037 = 1. + t6035;
  t6059 = Sin(var1[14]);
  t6103 = t5792*t5803;
  t6104 = t5781*t6014;
  t6110 = t6103 + t6104;
  t6123 = t5781*t5803;
  t6125 = -1.*t5792*t6014;
  t6126 = t6123 + t6125;
  t2103 = -0.0222*t827;
  t2312 = -0.087*t2109;
  t2395 = 0. + t2103 + t2312;
  t4375 = -0.3151*t4290;
  t5120 = 0.157*t4439;
  t5318 = 0. + t4375 + t5120;
  t5617 = -0.087*t827;
  t5621 = 0.0222*t2109;
  t5625 = 0. + t5617 + t5621;
  t5660 = -0.157*t4290;
  t5664 = -0.3151*t4439;
  t5670 = 0. + t5660 + t5664;
  t6242 = t324*t5417;
  t6246 = t5546*t5505*t5562;
  t6255 = t6242 + t6246;
  t5791 = -0.0222*t5788;
  t5793 = 0.3801*t5792;
  t5795 = 0. + t5791 + t5793;
  t6227 = t5417*t5546*t5505;
  t6228 = -1.*t324*t5562;
  t6230 = t6227 + t6228;
  t6267 = -1.*t549*t2109*t5546;
  t6272 = t690*t6255;
  t6273 = t6267 + t6272;
  t5879 = -0.3801*t5788;
  t5945 = -0.0222*t5792;
  t5996 = 0. + t5879 + t5945;
  t6051 = -0.8601*t6037;
  t6097 = -0.0222*t6059;
  t6099 = 0. + t6051 + t6097;
  t6311 = t690*t549*t5546;
  t6314 = t2109*t6255;
  t6316 = t6311 + t6314;
  t6338 = t3050*t6230;
  t6339 = -1.*t4439*t6273;
  t6340 = t6338 + t6339;
  t6119 = -0.0222*t6037;
  t6121 = 0.8601*t6059;
  t6122 = 0. + t6119 + t6121;
  t6400 = t5792*t6316;
  t6403 = t5781*t6340;
  t6414 = t6400 + t6403;
  t6432 = t5781*t6316;
  t6437 = -1.*t5792*t6340;
  t6448 = t6432 + t6437;
  t6555 = t2109*t5505;
  t6567 = t690*t549*t5562;
  t6570 = t6555 + t6567;
  t6580 = -1.*t690*t5505;
  t6584 = t549*t2109*t5562;
  t6585 = t6580 + t6584;
  t6595 = t3050*t549*t5417;
  t6604 = -1.*t4439*t6570;
  t6606 = t6595 + t6604;
  t6615 = t5792*t6585;
  t6616 = t5781*t6606;
  t6617 = t6615 + t6616;
  t6621 = t5781*t6585;
  t6625 = -1.*t5792*t6606;
  t6634 = t6621 + t6625;

  p_output1(0)=0. + t2395*t324*t549 + t5318*t5607 + t5625*t5657 + t5670*t5752 - 0.15025*(t4439*t5607 + t3050*t5752) + t5795*t5803 + t5996*t6014 + t6099*t6110 + t6122*t6126 - 0.0222*(-1.*t6059*t6110 + t6028*t6126) - 0.8601*(t6028*t6110 + t6059*t6126) + var1(0);
  p_output1(1)=0. + t2395*t549*t5546 + t5318*t6230 + t5625*t6255 + t5670*t6273 - 0.15025*(t4439*t6230 + t3050*t6273) + t5795*t6316 + t5996*t6340 + t6099*t6414 + t6122*t6448 - 0.0222*(-1.*t6059*t6414 + t6028*t6448) - 0.8601*(t6028*t6414 + t6059*t6448) + var1(1);
  p_output1(2)=0. + t5318*t5417*t549 - 1.*t2395*t5505 + t549*t5562*t5625 + t5670*t6570 - 0.15025*(t4439*t5417*t549 + t3050*t6570) + t5795*t6585 + t5996*t6606 + t6099*t6617 + t6122*t6634 - 0.0222*(-1.*t6059*t6617 + t6028*t6634) - 0.8601*(t6028*t6617 + t6059*t6634) + var1(2);
}


       
void p_rKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
