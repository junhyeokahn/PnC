/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:22:02 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RightFootFront.h"

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
  double t130;
  double t300;
  double t405;
  double t898;
  double t935;
  double t2771;
  double t2983;
  double t2889;
  double t3101;
  double t2031;
  double t2074;
  double t2194;
  double t2383;
  double t144;
  double t3428;
  double t3493;
  double t3636;
  double t2966;
  double t3138;
  double t3144;
  double t3743;
  double t3749;
  double t3828;
  double t3858;
  double t3874;
  double t3888;
  double t4018;
  double t4108;
  double t4144;
  double t4200;
  double t4634;
  double t4639;
  double t4647;
  double t4666;
  double t4676;
  double t4734;
  double t4797;
  double t4915;
  double t5002;
  double t5061;
  double t5127;
  double t5138;
  double t5180;
  double t5218;
  double t5228;
  double t5255;
  double t5268;
  double t5307;
  double t5312;
  double t5313;
  double t5355;
  double t5356;
  double t5373;
  double t900;
  double t1106;
  double t1724;
  double t2346;
  double t2587;
  double t2714;
  double t3163;
  double t3191;
  double t3390;
  double t3670;
  double t3682;
  double t3701;
  double t5527;
  double t5561;
  double t5563;
  double t3907;
  double t4051;
  double t4103;
  double t5480;
  double t5490;
  double t5506;
  double t5601;
  double t5603;
  double t5635;
  double t4445;
  double t4494;
  double t4594;
  double t4789;
  double t4869;
  double t4903;
  double t5717;
  double t5721;
  double t5760;
  double t5909;
  double t5915;
  double t5934;
  double t5090;
  double t5101;
  double t5122;
  double t5266;
  double t5291;
  double t5299;
  double t5965;
  double t5968;
  double t5981;
  double t5991;
  double t5996;
  double t5998;
  double t5321;
  double t5352;
  double t5354;
  double t6040;
  double t6042;
  double t6046;
  double t6118;
  double t6120;
  double t6127;
  double t6333;
  double t6359;
  double t6370;
  double t6413;
  double t6417;
  double t6419;
  double t6529;
  double t6540;
  double t6543;
  double t6559;
  double t6582;
  double t6584;
  double t6603;
  double t6607;
  double t6617;
  double t6635;
  double t6637;
  double t6640;
  double t6654;
  double t6663;
  double t6665;
  t130 = Cos(var1[3]);
  t300 = Cos(var1[11]);
  t405 = -1.*t300;
  t898 = 1. + t405;
  t935 = Sin(var1[11]);
  t2771 = Cos(var1[5]);
  t2983 = Sin(var1[3]);
  t2889 = Sin(var1[4]);
  t3101 = Sin(var1[5]);
  t2031 = Cos(var1[12]);
  t2074 = -1.*t2031;
  t2194 = 1. + t2074;
  t2383 = Sin(var1[12]);
  t144 = Cos(var1[4]);
  t3428 = -1.*t2771*t2983;
  t3493 = t130*t2889*t3101;
  t3636 = t3428 + t3493;
  t2966 = t130*t2771*t2889;
  t3138 = t2983*t3101;
  t3144 = t2966 + t3138;
  t3743 = -1.*t130*t144*t935;
  t3749 = t300*t3636;
  t3828 = t3743 + t3749;
  t3858 = Cos(var1[13]);
  t3874 = -1.*t3858;
  t3888 = 1. + t3874;
  t4018 = Sin(var1[13]);
  t4108 = t300*t130*t144;
  t4144 = t935*t3636;
  t4200 = t4108 + t4144;
  t4634 = t2031*t3144;
  t4639 = -1.*t2383*t3828;
  t4647 = t4634 + t4639;
  t4666 = Cos(var1[14]);
  t4676 = -1.*t4666;
  t4734 = 1. + t4676;
  t4797 = Sin(var1[14]);
  t4915 = t4018*t4200;
  t5002 = t3858*t4647;
  t5061 = t4915 + t5002;
  t5127 = t3858*t4200;
  t5138 = -1.*t4018*t4647;
  t5180 = t5127 + t5138;
  t5218 = Cos(var1[15]);
  t5228 = -1.*t5218;
  t5255 = 1. + t5228;
  t5268 = Sin(var1[15]);
  t5307 = -1.*t4797*t5061;
  t5312 = t4666*t5180;
  t5313 = t5307 + t5312;
  t5355 = t4666*t5061;
  t5356 = t4797*t5180;
  t5373 = t5355 + t5356;
  t900 = -0.022225*t898;
  t1106 = -0.086996*t935;
  t1724 = 0. + t900 + t1106;
  t2346 = -0.31508*t2194;
  t2587 = 0.156996*t2383;
  t2714 = 0. + t2346 + t2587;
  t3163 = -0.086996*t898;
  t3191 = 0.022225*t935;
  t3390 = 0. + t3163 + t3191;
  t3670 = -0.156996*t2194;
  t3682 = -0.31508*t2383;
  t3701 = 0. + t3670 + t3682;
  t5527 = t130*t2771;
  t5561 = t2983*t2889*t3101;
  t5563 = t5527 + t5561;
  t3907 = -0.022225*t3888;
  t4051 = 0.38008*t4018;
  t4103 = 0. + t3907 + t4051;
  t5480 = t2771*t2983*t2889;
  t5490 = -1.*t130*t3101;
  t5506 = t5480 + t5490;
  t5601 = -1.*t144*t935*t2983;
  t5603 = t300*t5563;
  t5635 = t5601 + t5603;
  t4445 = -0.38008*t3888;
  t4494 = -0.022225*t4018;
  t4594 = 0. + t4445 + t4494;
  t4789 = -0.86008*t4734;
  t4869 = -0.022225*t4797;
  t4903 = 0. + t4789 + t4869;
  t5717 = t300*t144*t2983;
  t5721 = t935*t5563;
  t5760 = t5717 + t5721;
  t5909 = t2031*t5506;
  t5915 = -1.*t2383*t5635;
  t5934 = t5909 + t5915;
  t5090 = -0.022225*t4734;
  t5101 = 0.86008*t4797;
  t5122 = 0. + t5090 + t5101;
  t5266 = -0.021147*t5255;
  t5291 = 1.34008*t5268;
  t5299 = 0. + t5266 + t5291;
  t5965 = t4018*t5760;
  t5968 = t3858*t5934;
  t5981 = t5965 + t5968;
  t5991 = t3858*t5760;
  t5996 = -1.*t4018*t5934;
  t5998 = t5991 + t5996;
  t5321 = -1.34008*t5255;
  t5352 = -0.021147*t5268;
  t5354 = 0. + t5321 + t5352;
  t6040 = -1.*t4797*t5981;
  t6042 = t4666*t5998;
  t6046 = t6040 + t6042;
  t6118 = t4666*t5981;
  t6120 = t4797*t5998;
  t6127 = t6118 + t6120;
  t6333 = t935*t2889;
  t6359 = t300*t144*t3101;
  t6370 = t6333 + t6359;
  t6413 = -1.*t300*t2889;
  t6417 = t144*t935*t3101;
  t6419 = t6413 + t6417;
  t6529 = t2031*t144*t2771;
  t6540 = -1.*t2383*t6370;
  t6543 = t6529 + t6540;
  t6559 = t4018*t6419;
  t6582 = t3858*t6543;
  t6584 = t6559 + t6582;
  t6603 = t3858*t6419;
  t6607 = -1.*t4018*t6543;
  t6617 = t6603 + t6607;
  t6635 = -1.*t4797*t6584;
  t6637 = t4666*t6617;
  t6640 = t6635 + t6637;
  t6654 = t4666*t6584;
  t6663 = t4797*t6617;
  t6665 = t6654 + t6663;

  p_output1(0)=0. + t130*t144*t1724 + t2714*t3144 + t3390*t3636 + t3701*t3828 - 0.166996*(t2383*t3144 + t2031*t3828) + t4103*t4200 + t4594*t4647 + t4903*t5061 + t5122*t5180 + t5299*t5313 + t5354*t5373 - 1.250132*(t5268*t5313 + t5218*t5373) + 0.043925*(t5218*t5313 - 1.*t5268*t5373) + var1(0);
  p_output1(1)=0. + t144*t1724*t2983 + t2714*t5506 + t3390*t5563 + t3701*t5635 - 0.166996*(t2383*t5506 + t2031*t5635) + t4103*t5760 + t4594*t5934 + t4903*t5981 + t5122*t5998 + t5299*t6046 + t5354*t6127 - 1.250132*(t5268*t6046 + t5218*t6127) + 0.043925*(t5218*t6046 - 1.*t5268*t6127) + var1(1);
  p_output1(2)=0. + t144*t2714*t2771 - 1.*t1724*t2889 + t144*t3101*t3390 + t3701*t6370 - 0.166996*(t144*t2383*t2771 + t2031*t6370) + t4103*t6419 + t4594*t6543 + t4903*t6584 + t5122*t6617 + t5299*t6640 + t5354*t6665 - 1.250132*(t5268*t6640 + t5218*t6665) + 0.043925*(t5218*t6640 - 1.*t5268*t6665) + var1(2);
}


       
void p_RightFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
