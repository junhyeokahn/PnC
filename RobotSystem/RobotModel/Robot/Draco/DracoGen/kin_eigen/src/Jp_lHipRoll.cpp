/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:43 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lHipRoll.h"

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
  double t416;
  double t853;
  double t1421;
  double t1603;
  double t1622;
  double t1770;
  double t6;
  double t126;
  double t366;
  double t446;
  double t620;
  double t652;
  double t695;
  double t3232;
  double t3239;
  double t3297;
  double t3559;
  double t3222;
  double t3224;
  double t3227;
  double t4137;
  double t4183;
  double t4211;
  double t1616;
  double t1656;
  double t1677;
  double t2043;
  double t2340;
  double t2762;
  double t4903;
  double t4917;
  double t4924;
  double t3480;
  double t3582;
  double t3619;
  double t4369;
  double t4370;
  double t4621;
  double t4981;
  double t5012;
  double t5013;
  double t5020;
  double t5044;
  double t5055;
  double t5249;
  double t5263;
  double t5266;
  double t5376;
  double t5378;
  double t5383;
  double t5448;
  double t5456;
  double t5457;
  double t5505;
  double t5506;
  double t5513;
  double t5585;
  double t5590;
  double t5591;
  double t5843;
  double t5846;
  double t5848;
  double t5796;
  double t5797;
  double t5800;
  double t5809;
  double t5822;
  double t5891;
  double t5913;
  double t5915;
  double t3015;
  double t5971;
  double t5979;
  double t6034;
  double t6060;
  double t6071;
  double t5142;
  double t5152;
  double t5180;
  double t6087;
  double t6101;
  double t5933;
  double t5949;
  double t5951;
  double t6117;
  double t6119;
  double t6133;
  double t6017;
  double t6019;
  double t6026;
  t416 = Sin(var1[3]);
  t853 = Cos(var1[6]);
  t1421 = -1.*t853;
  t1603 = 1. + t1421;
  t1622 = Sin(var1[6]);
  t1770 = Cos(var1[4]);
  t6 = Cos(var1[3]);
  t126 = Cos(var1[5]);
  t366 = -1.*t6*t126;
  t446 = Sin(var1[4]);
  t620 = Sin(var1[5]);
  t652 = -1.*t416*t446*t620;
  t695 = t366 + t652;
  t3232 = Cos(var1[7]);
  t3239 = -1.*t3232;
  t3297 = 1. + t3239;
  t3559 = Sin(var1[7]);
  t3222 = t853*t695;
  t3224 = t1770*t416*t1622;
  t3227 = t3222 + t3224;
  t4137 = -1.*t126*t416*t446;
  t4183 = t6*t620;
  t4211 = t4137 + t4183;
  t1616 = 0.087004*t1603;
  t1656 = 0.022225*t1622;
  t1677 = 0. + t1616 + t1656;
  t2043 = -0.022225*t1603;
  t2340 = 0.087004*t1622;
  t2762 = 0. + t2043 + t2340;
  t4903 = -1.*t126*t416;
  t4917 = t6*t446*t620;
  t4924 = t4903 + t4917;
  t3480 = 0.157004*t3297;
  t3582 = -0.31508*t3559;
  t3619 = 0. + t3480 + t3582;
  t4369 = -0.31508*t3297;
  t4370 = -0.157004*t3559;
  t4621 = 0. + t4369 + t4370;
  t4981 = t853*t4924;
  t5012 = -1.*t6*t1770*t1622;
  t5013 = t4981 + t5012;
  t5020 = t6*t126*t446;
  t5044 = t416*t620;
  t5055 = t5020 + t5044;
  t5249 = t6*t1770*t853*t620;
  t5263 = t6*t446*t1622;
  t5266 = t5249 + t5263;
  t5376 = t1770*t853*t416*t620;
  t5378 = t416*t446*t1622;
  t5383 = t5376 + t5378;
  t5448 = -1.*t853*t446*t620;
  t5456 = t1770*t1622;
  t5457 = t5448 + t5456;
  t5505 = t126*t416;
  t5506 = -1.*t6*t446*t620;
  t5513 = t5505 + t5506;
  t5585 = t126*t416*t446;
  t5590 = -1.*t6*t620;
  t5591 = t5585 + t5590;
  t5843 = -1.*t6*t1770*t853;
  t5846 = -1.*t4924*t1622;
  t5848 = t5843 + t5846;
  t5796 = 0.087004*t853;
  t5797 = -0.022225*t1622;
  t5800 = t5796 + t5797;
  t5809 = 0.022225*t853;
  t5822 = t5809 + t2340;
  t5891 = t6*t126;
  t5913 = t416*t446*t620;
  t5915 = t5891 + t5913;
  t3015 = -1.*t1770*t853*t416;
  t5971 = -1.*t5915*t1622;
  t5979 = t3015 + t5971;
  t6034 = t853*t446;
  t6060 = -1.*t1770*t620*t1622;
  t6071 = t6034 + t6060;
  t5142 = t3232*t5055;
  t5152 = -1.*t5013*t3559;
  t5180 = t5142 + t5152;
  t6087 = -0.157004*t3232;
  t6101 = t6087 + t3582;
  t5933 = t853*t5915;
  t5949 = -1.*t1770*t416*t1622;
  t5951 = t5933 + t5949;
  t6117 = -0.31508*t3232;
  t6119 = 0.157004*t3559;
  t6133 = t6117 + t6119;
  t6017 = t1770*t853*t620;
  t6019 = t446*t1622;
  t6026 = t6017 + t6019;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t3227*t3619 - 1.*t1770*t2762*t416 - 0.31508*(-1.*t3227*t3559 + t3232*t4211) + 0.157004*(t3227*t3232 + t3559*t4211) + t4211*t4621 + t1677*t695 - 0.022225*(t3015 + t1622*t695);
  p_output1(10)=t1677*t4924 + t3619*t5013 + t4621*t5055 + 0.157004*(t3232*t5013 + t3559*t5055) - 0.31508*t5180 + t1770*t2762*t6 - 0.022225*(t1622*t4924 + t1770*t6*t853);
  p_output1(11)=0;
  p_output1(12)=t3619*t5266 - 1.*t2762*t446*t6 + t126*t1770*t4621*t6 - 0.31508*(-1.*t3559*t5266 + t126*t1770*t3232*t6) + 0.157004*(t3232*t5266 + t126*t1770*t3559*t6) + t1677*t1770*t6*t620 - 0.022225*(t1622*t1770*t6*t620 - 1.*t446*t6*t853);
  p_output1(13)=-1.*t2762*t416*t446 + t126*t1770*t416*t4621 + t3619*t5383 + 0.157004*(t126*t1770*t3559*t416 + t3232*t5383) - 0.31508*(t126*t1770*t3232*t416 - 1.*t3559*t5383) + t1677*t1770*t416*t620 - 0.022225*(t1622*t1770*t416*t620 - 1.*t416*t446*t853);
  p_output1(14)=-1.*t1770*t2762 - 1.*t126*t446*t4621 + t3619*t5457 + 0.157004*(-1.*t126*t3559*t446 + t3232*t5457) - 0.31508*(-1.*t126*t3232*t446 - 1.*t3559*t5457) - 1.*t1677*t446*t620 - 0.022225*(-1.*t1622*t446*t620 - 1.*t1770*t853);
  p_output1(15)=-0.022225*t1622*t5055 + t1677*t5055 + t4621*t5513 + t3619*t5055*t853 + 0.157004*(t3559*t5513 + t3232*t5055*t853) - 0.31508*(t3232*t5513 - 1.*t3559*t5055*t853);
  p_output1(16)=-0.022225*t1622*t5591 + t1677*t5591 + t4621*t695 + t3619*t5591*t853 + 0.157004*(t3559*t695 + t3232*t5591*t853) - 0.31508*(t3232*t695 - 1.*t3559*t5591*t853);
  p_output1(17)=-0.022225*t126*t1622*t1770 + t126*t1677*t1770 - 1.*t1770*t4621*t620 + t126*t1770*t3619*t853 + 0.157004*(-1.*t1770*t3559*t620 + t126*t1770*t3232*t853) - 0.31508*(-1.*t1770*t3232*t620 - 1.*t126*t1770*t3559*t853);
  p_output1(18)=-0.022225*t5013 + t4924*t5822 + 0.157004*t3232*t5848 + 0.31508*t3559*t5848 + t3619*t5848 + t1770*t5800*t6;
  p_output1(19)=t1770*t416*t5800 + t5822*t5915 - 0.022225*t5951 + 0.157004*t3232*t5979 + 0.31508*t3559*t5979 + t3619*t5979;
  p_output1(20)=-1.*t446*t5800 - 0.022225*t6026 + 0.157004*t3232*t6071 + 0.31508*t3559*t6071 + t3619*t6071 + t1770*t5822*t620;
  p_output1(21)=-0.31508*(-1.*t3232*t5013 - 1.*t3559*t5055) + 0.157004*t5180 + t5055*t6101 + t5013*t6133;
  p_output1(22)=-0.31508*(-1.*t3559*t5591 - 1.*t3232*t5951) + 0.157004*(t3232*t5591 - 1.*t3559*t5951) + t5591*t6101 + t5951*t6133;
  p_output1(23)=-0.31508*(-1.*t126*t1770*t3559 - 1.*t3232*t6026) + 0.157004*(t126*t1770*t3232 - 1.*t3559*t6026) + t126*t1770*t6101 + t6026*t6133;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
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


       
void Jp_lHipRoll(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
