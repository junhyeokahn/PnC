/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:47 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lKnee.h"

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
  double t598;
  double t2111;
  double t2122;
  double t2163;
  double t2358;
  double t226;
  double t279;
  double t595;
  double t712;
  double t1372;
  double t1989;
  double t1996;
  double t3269;
  double t5121;
  double t5227;
  double t5494;
  double t5714;
  double t3840;
  double t4474;
  double t4994;
  double t6017;
  double t6033;
  double t6101;
  double t6166;
  double t6167;
  double t6168;
  double t6203;
  double t6161;
  double t6163;
  double t6164;
  double t6264;
  double t6283;
  double t6296;
  double t6321;
  double t6339;
  double t6344;
  double t6352;
  double t6364;
  double t6365;
  double t6368;
  double t6414;
  double t6425;
  double t6437;
  double t2347;
  double t2439;
  double t3027;
  double t3312;
  double t3499;
  double t3729;
  double t6499;
  double t6504;
  double t6514;
  double t5573;
  double t5900;
  double t5933;
  double t6106;
  double t6114;
  double t6117;
  double t6531;
  double t6537;
  double t6540;
  double t6560;
  double t6562;
  double t6566;
  double t6171;
  double t6245;
  double t6255;
  double t6304;
  double t6306;
  double t6310;
  double t6345;
  double t6357;
  double t6360;
  double t6591;
  double t6595;
  double t6610;
  double t6619;
  double t6655;
  double t6659;
  double t6402;
  double t6409;
  double t6412;
  double t6669;
  double t6672;
  double t6674;
  double t6692;
  double t6699;
  double t6703;
  double t6779;
  double t6783;
  double t6784;
  double t6834;
  double t6860;
  double t6863;
  double t6866;
  double t6870;
  double t6872;
  double t6883;
  double t6884;
  double t6886;
  double t6895;
  double t6899;
  double t6901;
  t598 = Cos(var1[3]);
  t2111 = Cos(var1[6]);
  t2122 = -1.*t2111;
  t2163 = 1. + t2122;
  t2358 = Sin(var1[6]);
  t226 = Cos(var1[5]);
  t279 = Sin(var1[3]);
  t595 = -1.*t226*t279;
  t712 = Sin(var1[4]);
  t1372 = Sin(var1[5]);
  t1989 = t598*t712*t1372;
  t1996 = t595 + t1989;
  t3269 = Cos(var1[4]);
  t5121 = Cos(var1[7]);
  t5227 = -1.*t5121;
  t5494 = 1. + t5227;
  t5714 = Sin(var1[7]);
  t3840 = t2111*t1996;
  t4474 = -1.*t598*t3269*t2358;
  t4994 = t3840 + t4474;
  t6017 = t598*t226*t712;
  t6033 = t279*t1372;
  t6101 = t6017 + t6033;
  t6166 = Cos(var1[8]);
  t6167 = -1.*t6166;
  t6168 = 1. + t6167;
  t6203 = Sin(var1[8]);
  t6161 = t5121*t6101;
  t6163 = -1.*t4994*t5714;
  t6164 = t6161 + t6163;
  t6264 = t598*t3269*t2111;
  t6283 = t1996*t2358;
  t6296 = t6264 + t6283;
  t6321 = Cos(var1[9]);
  t6339 = -1.*t6321;
  t6344 = 1. + t6339;
  t6352 = Sin(var1[9]);
  t6364 = t6166*t6164;
  t6365 = t6296*t6203;
  t6368 = t6364 + t6365;
  t6414 = t6166*t6296;
  t6425 = -1.*t6164*t6203;
  t6437 = t6414 + t6425;
  t2347 = 0.087004*t2163;
  t2439 = 0.022225*t2358;
  t3027 = 0. + t2347 + t2439;
  t3312 = -0.022225*t2163;
  t3499 = 0.087004*t2358;
  t3729 = 0. + t3312 + t3499;
  t6499 = t598*t226;
  t6504 = t279*t712*t1372;
  t6514 = t6499 + t6504;
  t5573 = 0.157004*t5494;
  t5900 = -0.31508*t5714;
  t5933 = 0. + t5573 + t5900;
  t6106 = -0.31508*t5494;
  t6114 = -0.157004*t5714;
  t6117 = 0. + t6106 + t6114;
  t6531 = t2111*t6514;
  t6537 = -1.*t3269*t279*t2358;
  t6540 = t6531 + t6537;
  t6560 = t226*t279*t712;
  t6562 = -1.*t598*t1372;
  t6566 = t6560 + t6562;
  t6171 = -0.38008*t6168;
  t6245 = -0.022225*t6203;
  t6255 = 0. + t6171 + t6245;
  t6304 = -0.022225*t6168;
  t6306 = 0.38008*t6203;
  t6310 = 0. + t6304 + t6306;
  t6345 = -0.86008*t6344;
  t6357 = -0.022225*t6352;
  t6360 = 0. + t6345 + t6357;
  t6591 = t5121*t6566;
  t6595 = -1.*t6540*t5714;
  t6610 = t6591 + t6595;
  t6619 = t3269*t2111*t279;
  t6655 = t6514*t2358;
  t6659 = t6619 + t6655;
  t6402 = -0.022225*t6344;
  t6409 = 0.86008*t6352;
  t6412 = 0. + t6402 + t6409;
  t6669 = t6166*t6610;
  t6672 = t6659*t6203;
  t6674 = t6669 + t6672;
  t6692 = t6166*t6659;
  t6699 = -1.*t6610*t6203;
  t6703 = t6692 + t6699;
  t6779 = t3269*t2111*t1372;
  t6783 = t712*t2358;
  t6784 = t6779 + t6783;
  t6834 = t3269*t226*t5121;
  t6860 = -1.*t6784*t5714;
  t6863 = t6834 + t6860;
  t6866 = -1.*t2111*t712;
  t6870 = t3269*t1372*t2358;
  t6872 = t6866 + t6870;
  t6883 = t6166*t6863;
  t6884 = t6872*t6203;
  t6886 = t6883 + t6884;
  t6895 = t6166*t6872;
  t6899 = -1.*t6863*t6203;
  t6901 = t6895 + t6899;

  p_output1(0)=0. + t1996*t3027 + t4994*t5933 + t3269*t3729*t598 + 0.150254*(t4994*t5121 + t5714*t6101) + t6101*t6117 + t6164*t6255 + t6296*t6310 + t6360*t6368 + t6412*t6437 - 0.022225*(-1.*t6352*t6368 + t6321*t6437) - 0.86008*(t6321*t6368 + t6352*t6437) + var1(0);
  p_output1(1)=0. + t279*t3269*t3729 + t3027*t6514 + t5933*t6540 + t6117*t6566 + 0.150254*(t5121*t6540 + t5714*t6566) + t6255*t6610 + t6310*t6659 + t6360*t6674 + t6412*t6703 - 0.022225*(-1.*t6352*t6674 + t6321*t6703) - 0.86008*(t6321*t6674 + t6352*t6703) + var1(1);
  p_output1(2)=0. + t1372*t3027*t3269 + t226*t3269*t6117 + t5933*t6784 + 0.150254*(t226*t3269*t5714 + t5121*t6784) + t6255*t6863 + t6310*t6872 + t6360*t6886 + t6412*t6901 - 0.022225*(-1.*t6352*t6886 + t6321*t6901) - 0.86008*(t6321*t6886 + t6352*t6901) - 1.*t3729*t712 + var1(2);
}


       
void p_lKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
