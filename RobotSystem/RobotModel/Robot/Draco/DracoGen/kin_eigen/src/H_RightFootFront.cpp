/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:36 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RightFootFront.h"

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
  double t68;
  double t198;
  double t365;
  double t358;
  double t402;
  double t65;
  double t719;
  double t799;
  double t801;
  double t839;
  double t876;
  double t888;
  double t896;
  double t359;
  double t416;
  double t417;
  double t814;
  double t933;
  double t1000;
  double t2123;
  double t1886;
  double t1963;
  double t1971;
  double t1833;
  double t2182;
  double t2192;
  double t2209;
  double t2413;
  double t2009;
  double t2235;
  double t2260;
  double t1754;
  double t2429;
  double t2442;
  double t2455;
  double t2629;
  double t2325;
  double t2473;
  double t2536;
  double t1744;
  double t2674;
  double t2722;
  double t2780;
  double t1226;
  double t1302;
  double t1329;
  double t1087;
  double t1092;
  double t1101;
  double t1214;
  double t1363;
  double t1398;
  double t2950;
  double t2956;
  double t3025;
  double t3041;
  double t3058;
  double t3070;
  double t3029;
  double t3083;
  double t3099;
  double t3206;
  double t3215;
  double t3221;
  double t3142;
  double t3232;
  double t3234;
  double t3270;
  double t3361;
  double t3368;
  double t1517;
  double t1563;
  double t1593;
  double t3457;
  double t3478;
  double t3488;
  double t3507;
  double t3540;
  double t3547;
  double t3505;
  double t3618;
  double t3688;
  double t3699;
  double t3708;
  double t3711;
  double t3689;
  double t3714;
  double t3734;
  double t3788;
  double t3820;
  double t3831;
  double t4539;
  double t4604;
  double t4650;
  double t4661;
  double t713;
  double t1069;
  double t1079;
  double t4915;
  double t4916;
  double t5146;
  double t5152;
  double t5240;
  double t5284;
  double t4026;
  double t4032;
  double t4039;
  double t2821;
  double t2846;
  double t2851;
  double t4611;
  double t4624;
  double t4632;
  double t4675;
  double t4720;
  double t4724;
  double t4798;
  double t4799;
  double t4802;
  double t4819;
  double t4829;
  double t4839;
  double t4923;
  double t4924;
  double t4929;
  double t1162;
  double t1408;
  double t1468;
  double t5074;
  double t5104;
  double t5108;
  double t5165;
  double t5173;
  double t5177;
  double t5203;
  double t5214;
  double t5215;
  double t5286;
  double t5312;
  double t5325;
  double t5330;
  double t5341;
  double t5344;
  double t4131;
  double t4165;
  double t4201;
  double t3405;
  double t3422;
  double t3445;
  double t1503;
  double t1613;
  double t1646;
  double t4365;
  double t4455;
  double t4463;
  double t3930;
  double t3932;
  double t3964;
  t68 = Cos(var1[3]);
  t198 = Cos(var1[5]);
  t365 = Sin(var1[3]);
  t358 = Sin(var1[4]);
  t402 = Sin(var1[5]);
  t65 = Sin(var1[12]);
  t719 = Cos(var1[12]);
  t799 = Cos(var1[4]);
  t801 = Sin(var1[11]);
  t839 = Cos(var1[11]);
  t876 = -1.*t198*t365;
  t888 = t68*t358*t402;
  t896 = t876 + t888;
  t359 = t68*t198*t358;
  t416 = t365*t402;
  t417 = t359 + t416;
  t814 = -1.*t68*t799*t801;
  t933 = t839*t896;
  t1000 = t814 + t933;
  t2123 = Cos(var1[13]);
  t1886 = t839*t68*t799;
  t1963 = t801*t896;
  t1971 = t1886 + t1963;
  t1833 = Sin(var1[13]);
  t2182 = t719*t417;
  t2192 = -1.*t65*t1000;
  t2209 = t2182 + t2192;
  t2413 = Cos(var1[14]);
  t2009 = t1833*t1971;
  t2235 = t2123*t2209;
  t2260 = t2009 + t2235;
  t1754 = Sin(var1[14]);
  t2429 = t2123*t1971;
  t2442 = -1.*t1833*t2209;
  t2455 = t2429 + t2442;
  t2629 = Cos(var1[15]);
  t2325 = -1.*t1754*t2260;
  t2473 = t2413*t2455;
  t2536 = t2325 + t2473;
  t1744 = Sin(var1[15]);
  t2674 = t2413*t2260;
  t2722 = t1754*t2455;
  t2780 = t2674 + t2722;
  t1226 = t68*t198;
  t1302 = t365*t358*t402;
  t1329 = t1226 + t1302;
  t1087 = t198*t365*t358;
  t1092 = -1.*t68*t402;
  t1101 = t1087 + t1092;
  t1214 = -1.*t799*t801*t365;
  t1363 = t839*t1329;
  t1398 = t1214 + t1363;
  t2950 = t839*t799*t365;
  t2956 = t801*t1329;
  t3025 = t2950 + t2956;
  t3041 = t719*t1101;
  t3058 = -1.*t65*t1398;
  t3070 = t3041 + t3058;
  t3029 = t1833*t3025;
  t3083 = t2123*t3070;
  t3099 = t3029 + t3083;
  t3206 = t2123*t3025;
  t3215 = -1.*t1833*t3070;
  t3221 = t3206 + t3215;
  t3142 = -1.*t1754*t3099;
  t3232 = t2413*t3221;
  t3234 = t3142 + t3232;
  t3270 = t2413*t3099;
  t3361 = t1754*t3221;
  t3368 = t3270 + t3361;
  t1517 = t801*t358;
  t1563 = t839*t799*t402;
  t1593 = t1517 + t1563;
  t3457 = -1.*t839*t358;
  t3478 = t799*t801*t402;
  t3488 = t3457 + t3478;
  t3507 = t719*t799*t198;
  t3540 = -1.*t65*t1593;
  t3547 = t3507 + t3540;
  t3505 = t1833*t3488;
  t3618 = t2123*t3547;
  t3688 = t3505 + t3618;
  t3699 = t2123*t3488;
  t3708 = -1.*t1833*t3547;
  t3711 = t3699 + t3708;
  t3689 = -1.*t1754*t3688;
  t3714 = t2413*t3711;
  t3734 = t3689 + t3714;
  t3788 = t2413*t3688;
  t3820 = t1754*t3711;
  t3831 = t3788 + t3820;
  t4539 = -1.*t839;
  t4604 = 1. + t4539;
  t4650 = -1.*t719;
  t4661 = 1. + t4650;
  t713 = t65*t417;
  t1069 = t719*t1000;
  t1079 = t713 + t1069;
  t4915 = -1.*t2123;
  t4916 = 1. + t4915;
  t5146 = -1.*t2413;
  t5152 = 1. + t5146;
  t5240 = -1.*t2629;
  t5284 = 1. + t5240;
  t4026 = t1744*t2536;
  t4032 = t2629*t2780;
  t4039 = t4026 + t4032;
  t2821 = t2629*t2536;
  t2846 = -1.*t1744*t2780;
  t2851 = t2821 + t2846;
  t4611 = -0.0222*t4604;
  t4624 = -0.087*t801;
  t4632 = 0. + t4611 + t4624;
  t4675 = -0.3151*t4661;
  t4720 = 0.157*t65;
  t4724 = 0. + t4675 + t4720;
  t4798 = -0.087*t4604;
  t4799 = 0.0222*t801;
  t4802 = 0. + t4798 + t4799;
  t4819 = -0.157*t4661;
  t4829 = -0.3151*t65;
  t4839 = 0. + t4819 + t4829;
  t4923 = -0.0222*t4916;
  t4924 = 0.3801*t1833;
  t4929 = 0. + t4923 + t4924;
  t1162 = t65*t1101;
  t1408 = t719*t1398;
  t1468 = t1162 + t1408;
  t5074 = -0.3801*t4916;
  t5104 = -0.0222*t1833;
  t5108 = 0. + t5074 + t5104;
  t5165 = -0.8601*t5152;
  t5173 = -0.0222*t1754;
  t5177 = 0. + t5165 + t5173;
  t5203 = -0.0222*t5152;
  t5214 = 0.8601*t1754;
  t5215 = 0. + t5203 + t5214;
  t5286 = -0.0211*t5284;
  t5312 = 1.3401*t1744;
  t5325 = 0. + t5286 + t5312;
  t5330 = -1.3401*t5284;
  t5341 = -0.0211*t1744;
  t5344 = 0. + t5330 + t5341;
  t4131 = t1744*t3234;
  t4165 = t2629*t3368;
  t4201 = t4131 + t4165;
  t3405 = t2629*t3234;
  t3422 = -1.*t1744*t3368;
  t3445 = t3405 + t3422;
  t1503 = t799*t198*t65;
  t1613 = t719*t1593;
  t1646 = t1503 + t1613;
  t4365 = t1744*t3734;
  t4455 = t2629*t3831;
  t4463 = t4365 + t4455;
  t3930 = t2629*t3734;
  t3932 = -1.*t1744*t3831;
  t3964 = t3930 + t3932;

  p_output1(0)=t1079;
  p_output1(1)=t1468;
  p_output1(2)=t1646;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1744*t2536 - 1.*t2629*t2780 - 0.000796*t2851;
  p_output1(5)=-1.*t1744*t3234 - 1.*t2629*t3368 - 0.000796*t3445;
  p_output1(6)=-1.*t1744*t3734 - 1.*t2629*t3831 - 0.000796*t3964;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2536*t2629 + t1744*t2780 + 0.000796*t4039;
  p_output1(9)=-1.*t2629*t3234 + t1744*t3368 + 0.000796*t4201;
  p_output1(10)=-1.*t2629*t3734 + t1744*t3831 + 0.000796*t4463;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.09205*t1079 + 0.043912*t2851 - 1.325152*t4039 + t417*t4724 + t1000*t4839 + t1971*t4929 + t2209*t5108 + t2260*t5177 + t2455*t5215 + t2536*t5325 + t2780*t5344 + t4632*t68*t799 + t4802*t896 + var1(0);
  p_output1(13)=0. - 0.09205*t1468 + 0.043912*t3445 - 1.325152*t4201 + t1101*t4724 + t1329*t4802 + t1398*t4839 + t3025*t4929 + t3070*t5108 + t3099*t5177 + t3221*t5215 + t3234*t5325 + t3368*t5344 + t365*t4632*t799 + var1(1);
  p_output1(14)=0. - 0.09205*t1646 + 0.043912*t3964 - 1.325152*t4463 - 1.*t358*t4632 + t1593*t4839 + t3488*t4929 + t3547*t5108 + t3688*t5177 + t3711*t5215 + t3734*t5325 + t3831*t5344 + t198*t4724*t799 + t402*t4802*t799 + var1(2);
  p_output1(15)=1.;
}


       
void H_RightFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
