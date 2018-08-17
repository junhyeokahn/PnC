/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:34 GMT-05:00
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
  double t913;
  double t1597;
  double t1649;
  double t1680;
  double t1990;
  double t2266;
  double t288;
  double t293;
  double t454;
  double t915;
  double t916;
  double t1259;
  double t1300;
  double t3338;
  double t3758;
  double t4079;
  double t4085;
  double t2978;
  double t3309;
  double t3326;
  double t4587;
  double t4613;
  double t4711;
  double t1862;
  double t1991;
  double t2074;
  double t2274;
  double t2302;
  double t2576;
  double t5272;
  double t5277;
  double t5279;
  double t4081;
  double t4206;
  double t4251;
  double t4712;
  double t4727;
  double t4940;
  double t5380;
  double t5405;
  double t5407;
  double t5434;
  double t5435;
  double t5447;
  double t5604;
  double t5607;
  double t5610;
  double t5720;
  double t5724;
  double t5728;
  double t5789;
  double t5791;
  double t5795;
  double t5844;
  double t5848;
  double t5854;
  double t5926;
  double t5933;
  double t5935;
  double t6129;
  double t6133;
  double t6134;
  double t6085;
  double t6087;
  double t6091;
  double t6100;
  double t6102;
  double t6158;
  double t6162;
  double t6168;
  double t2613;
  double t6186;
  double t6207;
  double t6281;
  double t6285;
  double t6296;
  double t5494;
  double t5498;
  double t5500;
  double t6308;
  double t6324;
  double t6173;
  double t6178;
  double t6180;
  double t6331;
  double t6337;
  double t6346;
  double t6248;
  double t6253;
  double t6255;
  t913 = Sin(var1[3]);
  t1597 = Cos(var1[6]);
  t1649 = -1.*t1597;
  t1680 = 1. + t1649;
  t1990 = Sin(var1[6]);
  t2266 = Cos(var1[4]);
  t288 = Cos(var1[3]);
  t293 = Cos(var1[5]);
  t454 = -1.*t288*t293;
  t915 = Sin(var1[4]);
  t916 = Sin(var1[5]);
  t1259 = -1.*t913*t915*t916;
  t1300 = t454 + t1259;
  t3338 = Cos(var1[7]);
  t3758 = -1.*t3338;
  t4079 = 1. + t3758;
  t4085 = Sin(var1[7]);
  t2978 = t1597*t1300;
  t3309 = t2266*t913*t1990;
  t3326 = t2978 + t3309;
  t4587 = -1.*t293*t913*t915;
  t4613 = t288*t916;
  t4711 = t4587 + t4613;
  t1862 = 0.087004*t1680;
  t1991 = 0.022225*t1990;
  t2074 = 0. + t1862 + t1991;
  t2274 = -0.022225*t1680;
  t2302 = 0.087004*t1990;
  t2576 = 0. + t2274 + t2302;
  t5272 = -1.*t293*t913;
  t5277 = t288*t915*t916;
  t5279 = t5272 + t5277;
  t4081 = 0.157004*t4079;
  t4206 = -0.31508*t4085;
  t4251 = 0. + t4081 + t4206;
  t4712 = -0.31508*t4079;
  t4727 = -0.157004*t4085;
  t4940 = 0. + t4712 + t4727;
  t5380 = t1597*t5279;
  t5405 = -1.*t288*t2266*t1990;
  t5407 = t5380 + t5405;
  t5434 = t288*t293*t915;
  t5435 = t913*t916;
  t5447 = t5434 + t5435;
  t5604 = t288*t2266*t1597*t916;
  t5607 = t288*t915*t1990;
  t5610 = t5604 + t5607;
  t5720 = t2266*t1597*t913*t916;
  t5724 = t913*t915*t1990;
  t5728 = t5720 + t5724;
  t5789 = -1.*t1597*t915*t916;
  t5791 = t2266*t1990;
  t5795 = t5789 + t5791;
  t5844 = t293*t913;
  t5848 = -1.*t288*t915*t916;
  t5854 = t5844 + t5848;
  t5926 = t293*t913*t915;
  t5933 = -1.*t288*t916;
  t5935 = t5926 + t5933;
  t6129 = -1.*t288*t2266*t1597;
  t6133 = -1.*t5279*t1990;
  t6134 = t6129 + t6133;
  t6085 = 0.087004*t1597;
  t6087 = -0.022225*t1990;
  t6091 = t6085 + t6087;
  t6100 = 0.022225*t1597;
  t6102 = t6100 + t2302;
  t6158 = t288*t293;
  t6162 = t913*t915*t916;
  t6168 = t6158 + t6162;
  t2613 = -1.*t2266*t1597*t913;
  t6186 = -1.*t6168*t1990;
  t6207 = t2613 + t6186;
  t6281 = t1597*t915;
  t6285 = -1.*t2266*t916*t1990;
  t6296 = t6281 + t6285;
  t5494 = t3338*t5447;
  t5498 = -1.*t5407*t4085;
  t5500 = t5494 + t5498;
  t6308 = -0.157004*t3338;
  t6324 = t6308 + t4206;
  t6173 = t1597*t6168;
  t6178 = -1.*t2266*t913*t1990;
  t6180 = t6173 + t6178;
  t6331 = -0.31508*t3338;
  t6337 = 0.157004*t4085;
  t6346 = t6331 + t6337;
  t6248 = t2266*t1597*t916;
  t6253 = t915*t1990;
  t6255 = t6248 + t6253;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1300*t2074 - 0.022225*(t1300*t1990 + t2613) + t3326*t4251 - 0.31508*(-1.*t3326*t4085 + t3338*t4711) + 0.157004*(t3326*t3338 + t4085*t4711) + t4711*t4940 - 1.*t2266*t2576*t913;
  p_output1(10)=t2266*t2576*t288 + t2074*t5279 - 0.022225*(t1597*t2266*t288 + t1990*t5279) + t4251*t5407 + t4940*t5447 + 0.157004*(t3338*t5407 + t4085*t5447) - 0.31508*t5500;
  p_output1(11)=0;
  p_output1(12)=t2266*t288*t293*t4940 + t4251*t5610 + 0.157004*(t2266*t288*t293*t4085 + t3338*t5610) - 0.31508*(t2266*t288*t293*t3338 - 1.*t4085*t5610) - 1.*t2576*t288*t915 + t2074*t2266*t288*t916 - 0.022225*(-1.*t1597*t288*t915 + t1990*t2266*t288*t916);
  p_output1(13)=t4251*t5728 + t2266*t293*t4940*t913 - 0.31508*(-1.*t4085*t5728 + t2266*t293*t3338*t913) + 0.157004*(t3338*t5728 + t2266*t293*t4085*t913) - 1.*t2576*t913*t915 + t2074*t2266*t913*t916 - 0.022225*(-1.*t1597*t913*t915 + t1990*t2266*t913*t916);
  p_output1(14)=-1.*t2266*t2576 + t4251*t5795 - 1.*t293*t4940*t915 - 0.31508*(-1.*t4085*t5795 - 1.*t293*t3338*t915) + 0.157004*(t3338*t5795 - 1.*t293*t4085*t915) - 1.*t2074*t915*t916 - 0.022225*(-1.*t1597*t2266 - 1.*t1990*t915*t916);
  p_output1(15)=-0.022225*t1990*t5447 + t2074*t5447 + t1597*t4251*t5447 + t4940*t5854 - 0.31508*(-1.*t1597*t4085*t5447 + t3338*t5854) + 0.157004*(t1597*t3338*t5447 + t4085*t5854);
  p_output1(16)=t1300*t4940 - 0.022225*t1990*t5935 + t2074*t5935 + t1597*t4251*t5935 + 0.157004*(t1300*t4085 + t1597*t3338*t5935) - 0.31508*(t1300*t3338 - 1.*t1597*t4085*t5935);
  p_output1(17)=-0.022225*t1990*t2266*t293 + t2074*t2266*t293 + t1597*t2266*t293*t4251 - 1.*t2266*t4940*t916 - 0.31508*(-1.*t1597*t2266*t293*t4085 - 1.*t2266*t3338*t916) + 0.157004*(t1597*t2266*t293*t3338 - 1.*t2266*t4085*t916);
  p_output1(18)=-0.022225*t5407 + t2266*t288*t6091 + t5279*t6102 + 0.157004*t3338*t6134 + 0.31508*t4085*t6134 + t4251*t6134;
  p_output1(19)=t6102*t6168 - 0.022225*t6180 + 0.157004*t3338*t6207 + 0.31508*t4085*t6207 + t4251*t6207 + t2266*t6091*t913;
  p_output1(20)=-0.022225*t6255 + 0.157004*t3338*t6296 + 0.31508*t4085*t6296 + t4251*t6296 - 1.*t6091*t915 + t2266*t6102*t916;
  p_output1(21)=-0.31508*(-1.*t3338*t5407 - 1.*t4085*t5447) + 0.157004*t5500 + t5447*t6324 + t5407*t6346;
  p_output1(22)=-0.31508*(-1.*t4085*t5935 - 1.*t3338*t6180) + 0.157004*(t3338*t5935 - 1.*t4085*t6180) + t5935*t6324 + t6180*t6346;
  p_output1(23)=-0.31508*(-1.*t2266*t293*t4085 - 1.*t3338*t6255) + 0.157004*(t2266*t293*t3338 - 1.*t4085*t6255) + t2266*t293*t6324 + t6255*t6346;
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
