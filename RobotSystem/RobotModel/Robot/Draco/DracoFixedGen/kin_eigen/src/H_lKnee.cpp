/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lKnee.h"

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
  double t264;
  double t408;
  double t476;
  double t67;
  double t625;
  double t6;
  double t763;
  double t836;
  double t844;
  double t719;
  double t753;
  double t762;
  double t269;
  double t693;
  double t698;
  double t811;
  double t817;
  double t819;
  double t785;
  double t796;
  double t808;
  double t856;
  double t857;
  double t845;
  double t847;
  double t949;
  double t951;
  double t968;
  double t970;
  double t700;
  double t777;
  double t782;
  double t880;
  double t882;
  double t886;
  double t914;
  double t921;
  double t936;
  double t937;
  double t941;
  double t942;
  double t944;
  double t952;
  double t953;
  double t956;
  double t959;
  double t961;
  double t963;
  double t971;
  double t977;
  double t978;
  double t982;
  double t983;
  double t995;
  double t810;
  double t822;
  double t835;
  double t891;
  double t894;
  double t900;
  double t877;
  double t853;
  double t859;
  double t860;
  double t902;
  double t905;
  double t909;
  t264 = Cos(var1[2]);
  t408 = Sin(var1[0]);
  t476 = Sin(var1[1]);
  t67 = Cos(var1[0]);
  t625 = Sin(var1[2]);
  t6 = Cos(var1[3]);
  t763 = Sin(var1[3]);
  t836 = Cos(var1[1]);
  t844 = 0. + t836;
  t719 = t264*t408*t476;
  t753 = t67*t625;
  t762 = t719 + t753;
  t269 = t67*t264;
  t693 = -1.*t408*t476*t625;
  t698 = t269 + t693;
  t811 = -1.*t67*t264*t476;
  t817 = t408*t625;
  t819 = t811 + t817;
  t785 = t264*t408;
  t796 = t67*t476*t625;
  t808 = t785 + t796;
  t856 = t844*t264;
  t857 = 0. + t856;
  t845 = -1.*t844*t625;
  t847 = 0. + t845;
  t949 = -1.*t264;
  t951 = 1. + t949;
  t968 = -1.*t6;
  t970 = 1. + t968;
  t700 = t6*t698;
  t777 = -1.*t762*t763;
  t782 = t700 + t777;
  t880 = t6*t762;
  t882 = t698*t763;
  t886 = t880 + t882;
  t914 = -1.*t67;
  t921 = 1. + t914;
  t936 = -1.*t836;
  t937 = 1. + t936;
  t941 = 0.331012*t937;
  t942 = -0.90524*t476;
  t944 = 0. + t941 + t942;
  t952 = -0.97024*t951;
  t953 = -0.066675*t625;
  t956 = 0. + t952 + t953;
  t959 = -0.066675*t951;
  t961 = 0.97024*t625;
  t963 = 0. + t959 + t961;
  t971 = -1.45024*t970;
  t977 = -0.066675*t763;
  t978 = 0. + t971 + t977;
  t982 = -0.066675*t970;
  t983 = 1.45024*t763;
  t995 = 0. + t982 + t983;
  t810 = t6*t808;
  t822 = -1.*t819*t763;
  t835 = t810 + t822;
  t891 = t6*t819;
  t894 = t808*t763;
  t900 = t891 + t894;
  t877 = 0. + t476;
  t853 = t6*t847;
  t859 = -1.*t857*t763;
  t860 = t853 + t859;
  t902 = t857*t6;
  t905 = t847*t763;
  t909 = t902 + t905;

  p_output1(0)=t782;
  p_output1(1)=t835;
  p_output1(2)=t860;
  p_output1(3)=0.;
  p_output1(4)=-1.*t408*t836;
  p_output1(5)=t67*t836;
  p_output1(6)=t877;
  p_output1(7)=0.;
  p_output1(8)=t886;
  p_output1(9)=t900;
  p_output1(10)=t909;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.261012*t408 - 0.066675*t782 - 0.324262*t408*t836 - 1.45024*t886 - 0.066675*t921 - 1.*t408*t944 + t408*t476*t956 + t67*t963 + t762*t978 + t698*t995;
  p_output1(13)=0. + 0.066675*t408 - 0.066675*t835 + 0.324262*t67*t836 - 1.45024*t900 + 0.261012*t921 + t67*t944 - 1.*t476*t67*t956 + t408*t963 + t819*t978 + t808*t995;
  p_output1(14)=0. - 0.331012*t476 - 0.066675*t860 + 0.324262*t877 - 1.45024*t909 - 0.90524*t937 + t844*t956 + t857*t978 + t847*t995;
  p_output1(15)=1.;
}


       
void H_lKnee(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
