/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:38 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/R_RightFootBack.h"

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
  double t130;
  double t346;
  double t661;
  double t532;
  double t745;
  double t127;
  double t1098;
  double t1188;
  double t1227;
  double t1371;
  double t1434;
  double t1450;
  double t1492;
  double t573;
  double t937;
  double t1068;
  double t1242;
  double t1493;
  double t1509;
  double t2958;
  double t2850;
  double t2858;
  double t2931;
  double t2814;
  double t2983;
  double t3027;
  double t3056;
  double t3106;
  double t2935;
  double t3060;
  double t3064;
  double t2727;
  double t3119;
  double t3185;
  double t3267;
  double t3382;
  double t3101;
  double t3309;
  double t3311;
  double t2724;
  double t3426;
  double t3511;
  double t3529;
  double t1901;
  double t1903;
  double t1907;
  double t1708;
  double t1739;
  double t1764;
  double t1830;
  double t2018;
  double t2119;
  double t3714;
  double t3741;
  double t3789;
  double t3866;
  double t3895;
  double t3943;
  double t3791;
  double t3959;
  double t4000;
  double t4009;
  double t4010;
  double t4032;
  double t4008;
  double t4037;
  double t4060;
  double t4101;
  double t4102;
  double t4118;
  double t2329;
  double t2439;
  double t2517;
  double t4472;
  double t4497;
  double t4565;
  double t4574;
  double t4584;
  double t4589;
  double t4567;
  double t4622;
  double t4668;
  double t4695;
  double t4760;
  double t4778;
  double t4675;
  double t4827;
  double t4829;
  double t4848;
  double t4881;
  double t4882;
  t130 = Cos(var1[3]);
  t346 = Cos(var1[5]);
  t661 = Sin(var1[3]);
  t532 = Sin(var1[4]);
  t745 = Sin(var1[5]);
  t127 = Sin(var1[12]);
  t1098 = Cos(var1[12]);
  t1188 = Cos(var1[4]);
  t1227 = Sin(var1[11]);
  t1371 = Cos(var1[11]);
  t1434 = -1.*t346*t661;
  t1450 = t130*t532*t745;
  t1492 = t1434 + t1450;
  t573 = t130*t346*t532;
  t937 = t661*t745;
  t1068 = t573 + t937;
  t1242 = -1.*t130*t1188*t1227;
  t1493 = t1371*t1492;
  t1509 = t1242 + t1493;
  t2958 = Cos(var1[13]);
  t2850 = t1371*t130*t1188;
  t2858 = t1227*t1492;
  t2931 = t2850 + t2858;
  t2814 = Sin(var1[13]);
  t2983 = t1098*t1068;
  t3027 = -1.*t127*t1509;
  t3056 = t2983 + t3027;
  t3106 = Cos(var1[14]);
  t2935 = t2814*t2931;
  t3060 = t2958*t3056;
  t3064 = t2935 + t3060;
  t2727 = Sin(var1[14]);
  t3119 = t2958*t2931;
  t3185 = -1.*t2814*t3056;
  t3267 = t3119 + t3185;
  t3382 = Cos(var1[15]);
  t3101 = -1.*t2727*t3064;
  t3309 = t3106*t3267;
  t3311 = t3101 + t3309;
  t2724 = Sin(var1[15]);
  t3426 = t3106*t3064;
  t3511 = t2727*t3267;
  t3529 = t3426 + t3511;
  t1901 = t130*t346;
  t1903 = t661*t532*t745;
  t1907 = t1901 + t1903;
  t1708 = t346*t661*t532;
  t1739 = -1.*t130*t745;
  t1764 = t1708 + t1739;
  t1830 = -1.*t1188*t1227*t661;
  t2018 = t1371*t1907;
  t2119 = t1830 + t2018;
  t3714 = t1371*t1188*t661;
  t3741 = t1227*t1907;
  t3789 = t3714 + t3741;
  t3866 = t1098*t1764;
  t3895 = -1.*t127*t2119;
  t3943 = t3866 + t3895;
  t3791 = t2814*t3789;
  t3959 = t2958*t3943;
  t4000 = t3791 + t3959;
  t4009 = t2958*t3789;
  t4010 = -1.*t2814*t3943;
  t4032 = t4009 + t4010;
  t4008 = -1.*t2727*t4000;
  t4037 = t3106*t4032;
  t4060 = t4008 + t4037;
  t4101 = t3106*t4000;
  t4102 = t2727*t4032;
  t4118 = t4101 + t4102;
  t2329 = t1227*t532;
  t2439 = t1371*t1188*t745;
  t2517 = t2329 + t2439;
  t4472 = -1.*t1371*t532;
  t4497 = t1188*t1227*t745;
  t4565 = t4472 + t4497;
  t4574 = t1098*t1188*t346;
  t4584 = -1.*t127*t2517;
  t4589 = t4574 + t4584;
  t4567 = t2814*t4565;
  t4622 = t2958*t4589;
  t4668 = t4567 + t4622;
  t4695 = t2958*t4565;
  t4760 = -1.*t2814*t4589;
  t4778 = t4695 + t4760;
  t4675 = -1.*t2727*t4668;
  t4827 = t3106*t4778;
  t4829 = t4675 + t4827;
  t4848 = t3106*t4668;
  t4881 = t2727*t4778;
  t4882 = t4848 + t4881;

  p_output1(0)=t1068*t127 + t1098*t1509;
  p_output1(1)=t127*t1764 + t1098*t2119;
  p_output1(2)=t1098*t2517 + t1188*t127*t346;
  p_output1(3)=-1.*t2724*t3311 - 1.*t3382*t3529 - 0.000796*(t3311*t3382 - 1.*t2724*t3529);
  p_output1(4)=-1.*t2724*t4060 - 1.*t3382*t4118 - 0.000796*(t3382*t4060 - 1.*t2724*t4118);
  p_output1(5)=-1.*t2724*t4829 - 1.*t3382*t4882 - 0.000796*(t3382*t4829 - 1.*t2724*t4882);
  p_output1(6)=-1.*t3311*t3382 + t2724*t3529 + 0.000796*(t2724*t3311 + t3382*t3529);
  p_output1(7)=-1.*t3382*t4060 + t2724*t4118 + 0.000796*(t2724*t4060 + t3382*t4118);
  p_output1(8)=-1.*t3382*t4829 + t2724*t4882 + 0.000796*(t2724*t4829 + t3382*t4882);
}


       
void R_RightFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
