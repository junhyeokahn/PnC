/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:33 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBack.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t856;
  double t446;
  double t899;
  double t587;
  double t945;
  double t143;
  double t376;
  double t1111;
  double t1119;
  double t1535;
  double t1410;
  double t1455;
  double t1499;
  double t626;
  double t957;
  double t1002;
  double t1048;
  double t1274;
  double t1337;
  double t2714;
  double t2960;
  double t2964;
  double t2994;
  double t2721;
  double t2792;
  double t2824;
  double t3123;
  double t3186;
  double t2907;
  double t3135;
  double t3170;
  double t2701;
  double t3214;
  double t3226;
  double t3241;
  double t3330;
  double t3178;
  double t3244;
  double t3255;
  double t2658;
  double t3351;
  double t3361;
  double t3463;
  double t1945;
  double t1997;
  double t2020;
  double t1659;
  double t1693;
  double t1765;
  double t1777;
  double t1883;
  double t1912;
  double t3762;
  double t3767;
  double t3783;
  double t3710;
  double t3712;
  double t3725;
  double t3744;
  double t3795;
  double t3799;
  double t3805;
  double t3879;
  double t3945;
  double t3801;
  double t3997;
  double t4028;
  double t4066;
  double t4125;
  double t4141;
  double t2337;
  double t2450;
  double t2451;
  double t4549;
  double t4575;
  double t4620;
  double t4400;
  double t4454;
  double t4509;
  double t4541;
  double t4634;
  double t4642;
  double t4662;
  double t4672;
  double t4677;
  double t4648;
  double t4679;
  double t4686;
  double t4716;
  double t4726;
  double t4752;
  t856 = Cos(var1[3]);
  t446 = Cos(var1[5]);
  t899 = Sin(var1[4]);
  t587 = Sin(var1[3]);
  t945 = Sin(var1[5]);
  t143 = Cos(var1[7]);
  t376 = Cos(var1[6]);
  t1111 = Cos(var1[4]);
  t1119 = Sin(var1[6]);
  t1535 = Sin(var1[7]);
  t1410 = t856*t446*t899;
  t1455 = t587*t945;
  t1499 = t1410 + t1455;
  t626 = -1.*t446*t587;
  t957 = t856*t899*t945;
  t1002 = t626 + t957;
  t1048 = t376*t1002;
  t1274 = -1.*t856*t1111*t1119;
  t1337 = t1048 + t1274;
  t2714 = Cos(var1[8]);
  t2960 = t856*t1111*t376;
  t2964 = t1002*t1119;
  t2994 = t2960 + t2964;
  t2721 = t143*t1499;
  t2792 = -1.*t1337*t1535;
  t2824 = t2721 + t2792;
  t3123 = Sin(var1[8]);
  t3186 = Cos(var1[9]);
  t2907 = t2714*t2824;
  t3135 = t2994*t3123;
  t3170 = t2907 + t3135;
  t2701 = Sin(var1[9]);
  t3214 = t2714*t2994;
  t3226 = -1.*t2824*t3123;
  t3241 = t3214 + t3226;
  t3330 = Cos(var1[10]);
  t3178 = -1.*t2701*t3170;
  t3244 = t3186*t3241;
  t3255 = t3178 + t3244;
  t2658 = Sin(var1[10]);
  t3351 = t3186*t3170;
  t3361 = t2701*t3241;
  t3463 = t3351 + t3361;
  t1945 = t446*t587*t899;
  t1997 = -1.*t856*t945;
  t2020 = t1945 + t1997;
  t1659 = t856*t446;
  t1693 = t587*t899*t945;
  t1765 = t1659 + t1693;
  t1777 = t376*t1765;
  t1883 = -1.*t1111*t587*t1119;
  t1912 = t1777 + t1883;
  t3762 = t1111*t376*t587;
  t3767 = t1765*t1119;
  t3783 = t3762 + t3767;
  t3710 = t143*t2020;
  t3712 = -1.*t1912*t1535;
  t3725 = t3710 + t3712;
  t3744 = t2714*t3725;
  t3795 = t3783*t3123;
  t3799 = t3744 + t3795;
  t3805 = t2714*t3783;
  t3879 = -1.*t3725*t3123;
  t3945 = t3805 + t3879;
  t3801 = -1.*t2701*t3799;
  t3997 = t3186*t3945;
  t4028 = t3801 + t3997;
  t4066 = t3186*t3799;
  t4125 = t2701*t3945;
  t4141 = t4066 + t4125;
  t2337 = t1111*t376*t945;
  t2450 = t899*t1119;
  t2451 = t2337 + t2450;
  t4549 = -1.*t376*t899;
  t4575 = t1111*t945*t1119;
  t4620 = t4549 + t4575;
  t4400 = t1111*t446*t143;
  t4454 = -1.*t2451*t1535;
  t4509 = t4400 + t4454;
  t4541 = t2714*t4509;
  t4634 = t4620*t3123;
  t4642 = t4541 + t4634;
  t4662 = t2714*t4620;
  t4672 = -1.*t4509*t3123;
  t4677 = t4662 + t4672;
  t4648 = -1.*t2701*t4642;
  t4679 = t3186*t4677;
  t4686 = t4648 + t4679;
  t4716 = t3186*t4642;
  t4726 = t2701*t4677;
  t4752 = t4716 + t4726;

  p_output1(0)=t1337*t143 + t1499*t1535;
  p_output1(1)=t143*t1912 + t1535*t2020;
  p_output1(2)=t143*t2451 + t1111*t1535*t446;
  p_output1(3)=-1.*t2658*t3255 - 1.*t3330*t3463 - 0.000796*(t3255*t3330 - 1.*t2658*t3463);
  p_output1(4)=-1.*t2658*t4028 - 1.*t3330*t4141 - 0.000796*(t3330*t4028 - 1.*t2658*t4141);
  p_output1(5)=-1.*t2658*t4686 - 1.*t3330*t4752 - 0.000796*(t3330*t4686 - 1.*t2658*t4752);
  p_output1(6)=-1.*t3255*t3330 + t2658*t3463 + 0.000796*(t2658*t3255 + t3330*t3463);
  p_output1(7)=-1.*t3330*t4028 + t2658*t4141 + 0.000796*(t2658*t4028 + t3330*t4141);
  p_output1(8)=-1.*t3330*t4686 + t2658*t4752 + 0.000796*(t2658*t4686 + t3330*t4752);
}


       
void R_LeftFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
