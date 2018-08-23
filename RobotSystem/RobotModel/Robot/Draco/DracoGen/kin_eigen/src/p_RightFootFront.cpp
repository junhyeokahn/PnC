/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:35 GMT-05:00
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
  double t322;
  double t1277;
  double t1375;
  double t1556;
  double t1929;
  double t2671;
  double t2810;
  double t2690;
  double t2863;
  double t2418;
  double t2478;
  double t2500;
  double t2568;
  double t543;
  double t3204;
  double t3316;
  double t3361;
  double t2722;
  double t2890;
  double t2939;
  double t3450;
  double t3643;
  double t3727;
  double t3764;
  double t3807;
  double t3823;
  double t3912;
  double t3964;
  double t3978;
  double t3989;
  double t4335;
  double t4387;
  double t4396;
  double t4525;
  double t4528;
  double t4546;
  double t4614;
  double t4624;
  double t4634;
  double t4700;
  double t4924;
  double t4952;
  double t4954;
  double t4987;
  double t4991;
  double t5011;
  double t5022;
  double t5047;
  double t5054;
  double t5069;
  double t5107;
  double t5108;
  double t5124;
  double t1886;
  double t1962;
  double t2142;
  double t2528;
  double t2639;
  double t2641;
  double t3023;
  double t3033;
  double t3085;
  double t3405;
  double t3438;
  double t3447;
  double t5368;
  double t5396;
  double t5398;
  double t3851;
  double t3913;
  double t3915;
  double t5344;
  double t5348;
  double t5360;
  double t5438;
  double t5439;
  double t5449;
  double t4151;
  double t4250;
  double t4264;
  double t4555;
  double t4617;
  double t4618;
  double t5480;
  double t5486;
  double t5496;
  double t5575;
  double t5578;
  double t5579;
  double t4772;
  double t4812;
  double t4877;
  double t5019;
  double t5024;
  double t5038;
  double t5612;
  double t5615;
  double t5622;
  double t5640;
  double t5641;
  double t5676;
  double t5078;
  double t5079;
  double t5104;
  double t5692;
  double t5698;
  double t5705;
  double t5713;
  double t5717;
  double t5719;
  double t5881;
  double t5905;
  double t5909;
  double t5962;
  double t5966;
  double t5977;
  double t6017;
  double t6033;
  double t6042;
  double t6054;
  double t6058;
  double t6081;
  double t6099;
  double t6107;
  double t6115;
  double t6134;
  double t6137;
  double t6143;
  double t6169;
  double t6183;
  double t6218;
  t322 = Cos(var1[3]);
  t1277 = Cos(var1[11]);
  t1375 = -1.*t1277;
  t1556 = 1. + t1375;
  t1929 = Sin(var1[11]);
  t2671 = Cos(var1[5]);
  t2810 = Sin(var1[3]);
  t2690 = Sin(var1[4]);
  t2863 = Sin(var1[5]);
  t2418 = Cos(var1[12]);
  t2478 = -1.*t2418;
  t2500 = 1. + t2478;
  t2568 = Sin(var1[12]);
  t543 = Cos(var1[4]);
  t3204 = -1.*t2671*t2810;
  t3316 = t322*t2690*t2863;
  t3361 = t3204 + t3316;
  t2722 = t322*t2671*t2690;
  t2890 = t2810*t2863;
  t2939 = t2722 + t2890;
  t3450 = -1.*t322*t543*t1929;
  t3643 = t1277*t3361;
  t3727 = t3450 + t3643;
  t3764 = Cos(var1[13]);
  t3807 = -1.*t3764;
  t3823 = 1. + t3807;
  t3912 = Sin(var1[13]);
  t3964 = t1277*t322*t543;
  t3978 = t1929*t3361;
  t3989 = t3964 + t3978;
  t4335 = t2418*t2939;
  t4387 = -1.*t2568*t3727;
  t4396 = t4335 + t4387;
  t4525 = Cos(var1[14]);
  t4528 = -1.*t4525;
  t4546 = 1. + t4528;
  t4614 = Sin(var1[14]);
  t4624 = t3912*t3989;
  t4634 = t3764*t4396;
  t4700 = t4624 + t4634;
  t4924 = t3764*t3989;
  t4952 = -1.*t3912*t4396;
  t4954 = t4924 + t4952;
  t4987 = Cos(var1[15]);
  t4991 = -1.*t4987;
  t5011 = 1. + t4991;
  t5022 = Sin(var1[15]);
  t5047 = -1.*t4614*t4700;
  t5054 = t4525*t4954;
  t5069 = t5047 + t5054;
  t5107 = t4525*t4700;
  t5108 = t4614*t4954;
  t5124 = t5107 + t5108;
  t1886 = -0.0222*t1556;
  t1962 = -0.087*t1929;
  t2142 = 0. + t1886 + t1962;
  t2528 = -0.3151*t2500;
  t2639 = 0.157*t2568;
  t2641 = 0. + t2528 + t2639;
  t3023 = -0.087*t1556;
  t3033 = 0.0222*t1929;
  t3085 = 0. + t3023 + t3033;
  t3405 = -0.157*t2500;
  t3438 = -0.3151*t2568;
  t3447 = 0. + t3405 + t3438;
  t5368 = t322*t2671;
  t5396 = t2810*t2690*t2863;
  t5398 = t5368 + t5396;
  t3851 = -0.0222*t3823;
  t3913 = 0.3801*t3912;
  t3915 = 0. + t3851 + t3913;
  t5344 = t2671*t2810*t2690;
  t5348 = -1.*t322*t2863;
  t5360 = t5344 + t5348;
  t5438 = -1.*t543*t1929*t2810;
  t5439 = t1277*t5398;
  t5449 = t5438 + t5439;
  t4151 = -0.3801*t3823;
  t4250 = -0.0222*t3912;
  t4264 = 0. + t4151 + t4250;
  t4555 = -0.8601*t4546;
  t4617 = -0.0222*t4614;
  t4618 = 0. + t4555 + t4617;
  t5480 = t1277*t543*t2810;
  t5486 = t1929*t5398;
  t5496 = t5480 + t5486;
  t5575 = t2418*t5360;
  t5578 = -1.*t2568*t5449;
  t5579 = t5575 + t5578;
  t4772 = -0.0222*t4546;
  t4812 = 0.8601*t4614;
  t4877 = 0. + t4772 + t4812;
  t5019 = -0.0211*t5011;
  t5024 = 1.3401*t5022;
  t5038 = 0. + t5019 + t5024;
  t5612 = t3912*t5496;
  t5615 = t3764*t5579;
  t5622 = t5612 + t5615;
  t5640 = t3764*t5496;
  t5641 = -1.*t3912*t5579;
  t5676 = t5640 + t5641;
  t5078 = -1.3401*t5011;
  t5079 = -0.0211*t5022;
  t5104 = 0. + t5078 + t5079;
  t5692 = -1.*t4614*t5622;
  t5698 = t4525*t5676;
  t5705 = t5692 + t5698;
  t5713 = t4525*t5622;
  t5717 = t4614*t5676;
  t5719 = t5713 + t5717;
  t5881 = t1929*t2690;
  t5905 = t1277*t543*t2863;
  t5909 = t5881 + t5905;
  t5962 = -1.*t1277*t2690;
  t5966 = t543*t1929*t2863;
  t5977 = t5962 + t5966;
  t6017 = t2418*t543*t2671;
  t6033 = -1.*t2568*t5909;
  t6042 = t6017 + t6033;
  t6054 = t3912*t5977;
  t6058 = t3764*t6042;
  t6081 = t6054 + t6058;
  t6099 = t3764*t5977;
  t6107 = -1.*t3912*t6042;
  t6115 = t6099 + t6107;
  t6134 = -1.*t4614*t6081;
  t6137 = t4525*t6115;
  t6143 = t6134 + t6137;
  t6169 = t4525*t6081;
  t6183 = t4614*t6115;
  t6218 = t6169 + t6183;

  p_output1(0)=0. + t2641*t2939 + t3085*t3361 + t3447*t3727 - 0.09205*(t2568*t2939 + t2418*t3727) + t3915*t3989 + t4264*t4396 + t4618*t4700 + t4877*t4954 + t5038*t5069 + t5104*t5124 - 1.325152*(t5022*t5069 + t4987*t5124) + 0.043912*(t4987*t5069 - 1.*t5022*t5124) + t2142*t322*t543 + var1(0);
  p_output1(1)=0. + t2641*t5360 + t3085*t5398 + t2142*t2810*t543 + t3447*t5449 - 0.09205*(t2568*t5360 + t2418*t5449) + t3915*t5496 + t4264*t5579 + t4618*t5622 + t4877*t5676 + t5038*t5705 + t5104*t5719 - 1.325152*(t5022*t5705 + t4987*t5719) + 0.043912*(t4987*t5705 - 1.*t5022*t5719) + var1(1);
  p_output1(2)=0. - 1.*t2142*t2690 + t2641*t2671*t543 + t2863*t3085*t543 + t3447*t5909 - 0.09205*(t2568*t2671*t543 + t2418*t5909) + t3915*t5977 + t4264*t6042 + t4618*t6081 + t4877*t6115 + t5038*t6143 + t5104*t6218 - 1.325152*(t5022*t6143 + t4987*t6218) + 0.043912*(t4987*t6143 - 1.*t5022*t6218) + var1(2);
}


       
void p_RightFootFront(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
