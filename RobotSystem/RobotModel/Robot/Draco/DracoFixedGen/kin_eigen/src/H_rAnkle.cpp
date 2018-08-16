/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:15 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rAnkle.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t910;
  double t997;
  double t1116;
  double t766;
  double t1271;
  double t622;
  double t1681;
  double t1764;
  double t1770;
  double t912;
  double t1272;
  double t1318;
  double t1872;
  double t2413;
  double t1594;
  double t1882;
  double t1892;
  double t467;
  double t2414;
  double t2543;
  double t2581;
  double t2760;
  double t2765;
  double t2789;
  double t2647;
  double t2663;
  double t2719;
  double t2756;
  double t2793;
  double t2796;
  double t2803;
  double t2813;
  double t2818;
  double t2848;
  double t2849;
  double t2854;
  double t2855;
  double t2851;
  double t2852;
  double t2853;
  double t2858;
  double t2859;
  double t2869;
  double t2879;
  double t2883;
  double t2084;
  double t2588;
  double t2802;
  double t2822;
  double t2867;
  double t2884;
  double t3000;
  double t3001;
  double t3023;
  double t3024;
  double t3045;
  double t3047;
  double t2915;
  double t2617;
  double t2630;
  double t2639;
  double t2963;
  double t2966;
  double t2984;
  double t2989;
  double t2990;
  double t2996;
  double t2997;
  double t3002;
  double t3009;
  double t3010;
  double t3013;
  double t3016;
  double t3018;
  double t3026;
  double t3032;
  double t3034;
  double t3038;
  double t3040;
  double t3042;
  double t3053;
  double t3054;
  double t3055;
  double t3063;
  double t3065;
  double t3072;
  double t2943;
  double t2838;
  double t2839;
  double t2841;
  double t2903;
  double t2958;
  double t2890;
  double t2892;
  double t2893;
  t910 = Cos(var1[7]);
  t997 = Sin(var1[5]);
  t1116 = Sin(var1[6]);
  t766 = Cos(var1[5]);
  t1271 = Sin(var1[7]);
  t622 = Cos(var1[8]);
  t1681 = t910*t997*t1116;
  t1764 = t766*t1271;
  t1770 = t1681 + t1764;
  t912 = t766*t910;
  t1272 = -1.*t997*t1116*t1271;
  t1318 = t912 + t1272;
  t1872 = Sin(var1[8]);
  t2413 = Cos(var1[9]);
  t1594 = t622*t1318;
  t1882 = -1.*t1770*t1872;
  t1892 = t1594 + t1882;
  t467 = Sin(var1[9]);
  t2414 = t622*t1770;
  t2543 = t1318*t1872;
  t2581 = t2414 + t2543;
  t2760 = -1.*t766*t910*t1116;
  t2765 = t997*t1271;
  t2789 = t2760 + t2765;
  t2647 = t910*t997;
  t2663 = t766*t1116*t1271;
  t2719 = t2647 + t2663;
  t2756 = t622*t2719;
  t2793 = -1.*t2789*t1872;
  t2796 = t2756 + t2793;
  t2803 = t622*t2789;
  t2813 = t2719*t1872;
  t2818 = t2803 + t2813;
  t2848 = Cos(var1[6]);
  t2849 = 0. + t2848;
  t2854 = t2849*t910;
  t2855 = 0. + t2854;
  t2851 = -1.*t2849*t1271;
  t2852 = 0. + t2851;
  t2853 = t622*t2852;
  t2858 = -1.*t2855*t1872;
  t2859 = t2853 + t2858;
  t2869 = t2855*t622;
  t2879 = t2852*t1872;
  t2883 = t2869 + t2879;
  t2084 = t467*t1892;
  t2588 = t2413*t2581;
  t2802 = t467*t2796;
  t2822 = t2413*t2818;
  t2867 = t467*t2859;
  t2884 = t2413*t2883;
  t3000 = -1.*t910;
  t3001 = 1. + t3000;
  t3023 = -1.*t622;
  t3024 = 1. + t3023;
  t3045 = -1.*t2413;
  t3047 = 1. + t3045;
  t2915 = t2084 + t2588;
  t2617 = t2413*t1892;
  t2630 = -1.*t467*t2581;
  t2639 = t2617 + t2630;
  t2963 = -1.*t766;
  t2966 = 1. + t2963;
  t2984 = -1.*t2848;
  t2989 = 1. + t2984;
  t2990 = -0.330988*t2989;
  t2996 = -0.90524*t1116;
  t2997 = 0. + t2990 + t2996;
  t3002 = -0.97024*t3001;
  t3009 = -0.066675*t1271;
  t3010 = 0. + t3002 + t3009;
  t3013 = -0.066675*t3001;
  t3016 = 0.97024*t1271;
  t3018 = 0. + t3013 + t3016;
  t3026 = -1.45024*t3024;
  t3032 = -0.066675*t1872;
  t3034 = 0. + t3026 + t3032;
  t3038 = -0.066675*t3024;
  t3040 = 1.45024*t1872;
  t3042 = 0. + t3038 + t3040;
  t3053 = -0.065597*t3047;
  t3054 = 1.93024*t467;
  t3055 = 0. + t3053 + t3054;
  t3063 = -1.93024*t3047;
  t3065 = -0.065597*t467;
  t3072 = 0. + t3063 + t3065;
  t2943 = t2802 + t2822;
  t2838 = t2413*t2796;
  t2839 = -1.*t467*t2818;
  t2841 = t2838 + t2839;
  t2903 = 0. + t1116;
  t2958 = t2867 + t2884;
  t2890 = t2413*t2859;
  t2892 = -1.*t467*t2883;
  t2893 = t2890 + t2892;

  p_output1(0)=t2084 + t2588 + 0.000796*t2639;
  p_output1(1)=t2802 + t2822 + 0.000796*t2841;
  p_output1(2)=t2867 + t2884 + 0.000796*t2893;
  p_output1(3)=0.;
  p_output1(4)=-1.*t2848*t997;
  p_output1(5)=t2848*t766;
  p_output1(6)=t2903;
  p_output1(7)=0.;
  p_output1(8)=-1.*t1892*t2413 + 0.000796*t2915 + t2581*t467;
  p_output1(9)=-1.*t2413*t2796 + 0.000796*t2943 + t2818*t467;
  p_output1(10)=-1.*t2413*t2859 + 0.000796*t2958 + t2883*t467;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.065597*t2639 - 1.93024*t2915 - 0.066675*t2966 + t1770*t3034 + t1318*t3042 + t1892*t3055 + t2581*t3072 + t3018*t766 - 0.260988*t997 + 0.340988*t2848*t997 - 1.*t2997*t997 + t1116*t3010*t997;
  p_output1(13)=0. - 0.065597*t2841 - 1.93024*t2943 - 0.260988*t2966 + t2789*t3034 + t2719*t3042 + t2796*t3055 + t2818*t3072 - 0.340988*t2848*t766 + t2997*t766 - 1.*t1116*t3010*t766 + 0.066675*t997 + t3018*t997;
  p_output1(14)=0. + 0.330988*t1116 - 0.065597*t2893 - 0.340988*t2903 - 1.93024*t2958 - 0.90524*t2989 + t2849*t3010 + t2855*t3034 + t2852*t3042 + t2859*t3055 + t2883*t3072;
  p_output1(15)=1.;
}


       
void H_rAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
