/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:40 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lKnee.h"

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
  double t2487;
  double t2682;
  double t2163;
  double t2624;
  double t2684;
  double t4660;
  double t3988;
  double t4559;
  double t4605;
  double t4625;
  double t4690;
  double t890;
  double t4805;
  double t4816;
  double t4825;
  double t1627;
  double t2673;
  double t3063;
  double t3795;
  double t3984;
  double t4631;
  double t4748;
  double t4752;
  double t4759;
  double t4770;
  double t4785;
  double t4944;
  double t780;
  double t5548;
  double t5559;
  double t5560;
  double t5245;
  double t5706;
  double t5708;
  double t5730;
  double t5437;
  double t5457;
  double t5486;
  double t5540;
  double t5570;
  double t5585;
  double t5602;
  double t5679;
  double t5683;
  double t5850;
  double t5866;
  double t5870;
  double t5798;
  double t5800;
  double t5801;
  double t5809;
  double t5818;
  double t5832;
  double t4791;
  double t5022;
  double t5049;
  double t5246;
  double t5287;
  double t5340;
  double t5698;
  double t5732;
  double t5736;
  double t5752;
  double t5754;
  double t5759;
  double t5846;
  double t5871;
  double t5912;
  double t5937;
  double t5948;
  double t5960;
  t2487 = Cos(var1[5]);
  t2682 = Sin(var1[3]);
  t2163 = Cos(var1[3]);
  t2624 = Sin(var1[4]);
  t2684 = Sin(var1[5]);
  t4660 = Cos(var1[4]);
  t3988 = Cos(var1[6]);
  t4559 = -1.*t2487*t2682;
  t4605 = t2163*t2624*t2684;
  t4625 = t4559 + t4605;
  t4690 = Sin(var1[6]);
  t890 = Cos(var1[8]);
  t4805 = t2163*t4660*t3988;
  t4816 = t4625*t4690;
  t4825 = t4805 + t4816;
  t1627 = Cos(var1[7]);
  t2673 = t2163*t2487*t2624;
  t3063 = t2682*t2684;
  t3795 = t2673 + t3063;
  t3984 = t1627*t3795;
  t4631 = t3988*t4625;
  t4748 = -1.*t2163*t4660*t4690;
  t4752 = t4631 + t4748;
  t4759 = Sin(var1[7]);
  t4770 = -1.*t4752*t4759;
  t4785 = t3984 + t4770;
  t4944 = Sin(var1[8]);
  t780 = Sin(var1[9]);
  t5548 = t2163*t2487;
  t5559 = t2682*t2624*t2684;
  t5560 = t5548 + t5559;
  t5245 = Cos(var1[9]);
  t5706 = t4660*t3988*t2682;
  t5708 = t5560*t4690;
  t5730 = t5706 + t5708;
  t5437 = t2487*t2682*t2624;
  t5457 = -1.*t2163*t2684;
  t5486 = t5437 + t5457;
  t5540 = t1627*t5486;
  t5570 = t3988*t5560;
  t5585 = -1.*t4660*t2682*t4690;
  t5602 = t5570 + t5585;
  t5679 = -1.*t5602*t4759;
  t5683 = t5540 + t5679;
  t5850 = -1.*t3988*t2624;
  t5866 = t4660*t2684*t4690;
  t5870 = t5850 + t5866;
  t5798 = t4660*t2487*t1627;
  t5800 = t4660*t3988*t2684;
  t5801 = t2624*t4690;
  t5809 = t5800 + t5801;
  t5818 = -1.*t5809*t4759;
  t5832 = t5798 + t5818;
  t4791 = t890*t4785;
  t5022 = t4825*t4944;
  t5049 = t4791 + t5022;
  t5246 = t890*t4825;
  t5287 = -1.*t4785*t4944;
  t5340 = t5246 + t5287;
  t5698 = t890*t5683;
  t5732 = t5730*t4944;
  t5736 = t5698 + t5732;
  t5752 = t890*t5730;
  t5754 = -1.*t5683*t4944;
  t5759 = t5752 + t5754;
  t5846 = t890*t5832;
  t5871 = t5870*t4944;
  t5912 = t5846 + t5871;
  t5937 = t890*t5870;
  t5948 = -1.*t5832*t4944;
  t5960 = t5937 + t5948;

  p_output1(0)=t5245*t5340 - 1.*t5049*t780;
  p_output1(1)=t5245*t5759 - 1.*t5736*t780;
  p_output1(2)=t5245*t5960 - 1.*t5912*t780;
  p_output1(3)=t1627*t4752 + t3795*t4759;
  p_output1(4)=t4759*t5486 + t1627*t5602;
  p_output1(5)=t2487*t4660*t4759 + t1627*t5809;
  p_output1(6)=t5049*t5245 + t5340*t780;
  p_output1(7)=t5245*t5736 + t5759*t780;
  p_output1(8)=t5245*t5912 + t5960*t780;
}


       
void R_lKnee(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
