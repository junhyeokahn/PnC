/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lHipPitch.h"

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
  double t420;
  double t573;
  double t936;
  double t575;
  double t975;
  double t498;
  double t837;
  double t1064;
  double t1066;
  double t475;
  double t1078;
  double t301;
  double t1603;
  double t2994;
  double t3039;
  double t3040;
  double t1989;
  double t2472;
  double t1912;
  double t1913;
  double t1947;
  double t1733;
  double t1765;
  double t1769;
  double t3694;
  double t3708;
  double t3715;
  double t3578;
  double t3584;
  double t3617;
  double t3879;
  double t3941;
  double t3959;
  double t1816;
  double t2334;
  double t2468;
  double t559;
  double t1082;
  double t1201;
  double t3672;
  double t3755;
  double t3758;
  double t2833;
  double t3045;
  double t3077;
  double t3823;
  double t4010;
  double t4020;
  double t3784;
  double t3789;
  double t3820;
  double t5070;
  double t5124;
  double t5287;
  double t5333;
  double t4499;
  double t4549;
  double t4634;
  double t5404;
  double t5413;
  double t4821;
  double t4960;
  double t5004;
  double t1259;
  double t2727;
  double t2792;
  double t5167;
  double t5190;
  double t5208;
  double t5233;
  double t5264;
  double t5276;
  double t5338;
  double t5339;
  double t5345;
  double t5353;
  double t5355;
  double t5380;
  double t4662;
  double t4671;
  double t4672;
  double t5466;
  double t5469;
  double t5483;
  double t5494;
  double t5499;
  double t5501;
  double t5017;
  double t5020;
  double t5021;
  double t3176;
  double t3771;
  double t3783;
  double t4720;
  double t4746;
  double t4747;
  double t5039;
  double t5058;
  double t5062;
  double t3821;
  double t4057;
  double t4260;
  t420 = Cos(var1[3]);
  t573 = Cos(var1[5]);
  t936 = Sin(var1[4]);
  t575 = Sin(var1[3]);
  t975 = Sin(var1[5]);
  t498 = Cos(var1[6]);
  t837 = -1.*t573*t575;
  t1064 = t420*t936*t975;
  t1066 = t837 + t1064;
  t475 = Cos(var1[4]);
  t1078 = Sin(var1[6]);
  t301 = Cos(var1[8]);
  t1603 = Cos(var1[7]);
  t2994 = t420*t573;
  t3039 = t575*t936*t975;
  t3040 = t2994 + t3039;
  t1989 = Sin(var1[7]);
  t2472 = Sin(var1[8]);
  t1912 = t498*t1066;
  t1913 = -1.*t420*t475*t1078;
  t1947 = t1912 + t1913;
  t1733 = t420*t573*t936;
  t1765 = t575*t975;
  t1769 = t1733 + t1765;
  t3694 = t498*t3040;
  t3708 = -1.*t475*t575*t1078;
  t3715 = t3694 + t3708;
  t3578 = t573*t575*t936;
  t3584 = -1.*t420*t975;
  t3617 = t3578 + t3584;
  t3879 = t475*t498*t975;
  t3941 = t936*t1078;
  t3959 = t3879 + t3941;
  t1816 = t1603*t1769;
  t2334 = -1.*t1947*t1989;
  t2468 = t1816 + t2334;
  t559 = t420*t475*t498;
  t1082 = t1066*t1078;
  t1201 = t559 + t1082;
  t3672 = t1603*t3617;
  t3755 = -1.*t3715*t1989;
  t3758 = t3672 + t3755;
  t2833 = t475*t498*t575;
  t3045 = t3040*t1078;
  t3077 = t2833 + t3045;
  t3823 = t475*t573*t1603;
  t4010 = -1.*t3959*t1989;
  t4020 = t3823 + t4010;
  t3784 = -1.*t498*t936;
  t3789 = t475*t975*t1078;
  t3820 = t3784 + t3789;
  t5070 = -1.*t498;
  t5124 = 1. + t5070;
  t5287 = -1.*t1603;
  t5333 = 1. + t5287;
  t4499 = t1603*t1947;
  t4549 = t1769*t1989;
  t4634 = t4499 + t4549;
  t5404 = -1.*t301;
  t5413 = 1. + t5404;
  t4821 = t301*t2468;
  t4960 = t1201*t2472;
  t5004 = t4821 + t4960;
  t1259 = t301*t1201;
  t2727 = -1.*t2468*t2472;
  t2792 = t1259 + t2727;
  t5167 = 0.087*t5124;
  t5190 = 0.0222*t1078;
  t5208 = 0. + t5167 + t5190;
  t5233 = -0.0222*t5124;
  t5264 = 0.087*t1078;
  t5276 = 0. + t5233 + t5264;
  t5338 = 0.157*t5333;
  t5339 = -0.3151*t1989;
  t5345 = 0. + t5338 + t5339;
  t5353 = -0.3151*t5333;
  t5355 = -0.157*t1989;
  t5380 = 0. + t5353 + t5355;
  t4662 = t1603*t3715;
  t4671 = t3617*t1989;
  t4672 = t4662 + t4671;
  t5466 = -0.3801*t5413;
  t5469 = -0.0222*t2472;
  t5483 = 0. + t5466 + t5469;
  t5494 = -0.0222*t5413;
  t5499 = 0.3801*t2472;
  t5501 = 0. + t5494 + t5499;
  t5017 = t301*t3758;
  t5020 = t3077*t2472;
  t5021 = t5017 + t5020;
  t3176 = t301*t3077;
  t3771 = -1.*t3758*t2472;
  t3783 = t3176 + t3771;
  t4720 = t1603*t3959;
  t4746 = t475*t573*t1989;
  t4747 = t4720 + t4746;
  t5039 = t301*t4020;
  t5058 = t3820*t2472;
  t5062 = t5039 + t5058;
  t3821 = t301*t3820;
  t4057 = -1.*t4020*t2472;
  t4260 = t3821 + t4057;

  p_output1(0)=t2792;
  p_output1(1)=t3783;
  p_output1(2)=t4260;
  p_output1(3)=0.;
  p_output1(4)=t4634;
  p_output1(5)=t4672;
  p_output1(6)=t4747;
  p_output1(7)=0.;
  p_output1(8)=t5004;
  p_output1(9)=t5021;
  p_output1(10)=t5062;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t2792 + 0.167*t4634 - 0.3801*t5004 + t1066*t5208 + t420*t475*t5276 + t1947*t5345 + t1769*t5380 + t2468*t5483 + t1201*t5501 + var1(0);
  p_output1(13)=0. - 0.0222*t3783 + 0.167*t4672 - 0.3801*t5021 + t3040*t5208 + t3715*t5345 + t3617*t5380 + t3758*t5483 + t3077*t5501 + t475*t5276*t575 + var1(1);
  p_output1(14)=0. - 0.0222*t4260 + 0.167*t4747 - 0.3801*t5062 + t3959*t5345 + t4020*t5483 + t3820*t5501 + t475*t5380*t573 - 1.*t5276*t936 + t475*t5208*t975 + var1(2);
  p_output1(15)=1.;
}


       
void H_lHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
