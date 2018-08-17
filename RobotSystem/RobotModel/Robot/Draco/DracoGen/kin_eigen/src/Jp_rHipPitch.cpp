/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:48 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rHipPitch.h"

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
  double t1393;
  double t23;
  double t26;
  double t28;
  double t1004;
  double t2277;
  double t2181;
  double t2216;
  double t2289;
  double t1579;
  double t1641;
  double t1792;
  double t1940;
  double t21;
  double t2875;
  double t2931;
  double t3216;
  double t2250;
  double t2307;
  double t2337;
  double t3536;
  double t3700;
  double t3710;
  double t3734;
  double t3879;
  double t4034;
  double t4052;
  double t4162;
  double t4169;
  double t4202;
  double t4822;
  double t4823;
  double t4826;
  double t994;
  double t1081;
  double t1160;
  double t1905;
  double t1986;
  double t2169;
  double t2726;
  double t2850;
  double t2866;
  double t3334;
  double t3423;
  double t3467;
  double t5633;
  double t5656;
  double t5664;
  double t4041;
  double t4124;
  double t4125;
  double t5528;
  double t5531;
  double t5573;
  double t5729;
  double t5745;
  double t5855;
  double t4393;
  double t4463;
  double t4584;
  double t5876;
  double t5888;
  double t5893;
  double t5916;
  double t5931;
  double t5959;
  double t6114;
  double t6122;
  double t6128;
  double t6145;
  double t6177;
  double t6189;
  double t6222;
  double t6254;
  double t6264;
  double t6388;
  double t6390;
  double t6404;
  double t6422;
  double t6433;
  double t6455;
  double t6532;
  double t6547;
  double t6553;
  double t6604;
  double t6609;
  double t6612;
  double t6643;
  double t6668;
  double t6673;
  double t6707;
  double t6717;
  double t6718;
  double t6860;
  double t6878;
  double t6911;
  double t6913;
  double t6917;
  double t6920;
  double t7008;
  double t7011;
  double t7016;
  double t7033;
  double t7041;
  double t7042;
  double t7142;
  double t7144;
  double t7147;
  double t7280;
  double t7287;
  double t7289;
  double t7243;
  double t7257;
  double t7260;
  double t7270;
  double t7273;
  double t7356;
  double t7359;
  double t7367;
  double t7386;
  double t7388;
  double t7372;
  double t7378;
  double t7383;
  double t7487;
  double t7491;
  double t7492;
  double t7479;
  double t7480;
  double t7485;
  double t7598;
  double t7600;
  double t7603;
  double t7546;
  double t7548;
  double t7562;
  double t7578;
  double t7588;
  double t7634;
  double t7641;
  double t7647;
  double t7702;
  double t7703;
  double t7705;
  double t6029;
  double t6034;
  double t6039;
  double t7740;
  double t7747;
  double t7753;
  double t7759;
  double t7765;
  double t7666;
  double t7670;
  double t7676;
  double t7820;
  double t7821;
  double t7825;
  double t7730;
  double t7732;
  double t7733;
  double t7885;
  double t7891;
  double t7892;
  t1393 = Sin(var1[3]);
  t23 = Cos(var1[11]);
  t26 = -1.*t23;
  t28 = 1. + t26;
  t1004 = Sin(var1[11]);
  t2277 = Cos(var1[3]);
  t2181 = Cos(var1[5]);
  t2216 = Sin(var1[4]);
  t2289 = Sin(var1[5]);
  t1579 = Cos(var1[12]);
  t1641 = -1.*t1579;
  t1792 = 1. + t1641;
  t1940 = Sin(var1[12]);
  t21 = Cos(var1[4]);
  t2875 = -1.*t2277*t2181;
  t2931 = -1.*t1393*t2216*t2289;
  t3216 = t2875 + t2931;
  t2250 = -1.*t2181*t1393*t2216;
  t2307 = t2277*t2289;
  t2337 = t2250 + t2307;
  t3536 = t21*t1004*t1393;
  t3700 = t23*t3216;
  t3710 = t3536 + t3700;
  t3734 = Cos(var1[13]);
  t3879 = -1.*t3734;
  t4034 = 1. + t3879;
  t4052 = Sin(var1[13]);
  t4162 = -1.*t23*t21*t1393;
  t4169 = t1004*t3216;
  t4202 = t4162 + t4169;
  t4822 = t1579*t2337;
  t4823 = -1.*t1940*t3710;
  t4826 = t4822 + t4823;
  t994 = -0.022225*t28;
  t1081 = -0.086996*t1004;
  t1160 = 0. + t994 + t1081;
  t1905 = -0.31508*t1792;
  t1986 = 0.156996*t1940;
  t2169 = 0. + t1905 + t1986;
  t2726 = -0.086996*t28;
  t2850 = 0.022225*t1004;
  t2866 = 0. + t2726 + t2850;
  t3334 = -0.156996*t1792;
  t3423 = -0.31508*t1940;
  t3467 = 0. + t3334 + t3423;
  t5633 = -1.*t2181*t1393;
  t5656 = t2277*t2216*t2289;
  t5664 = t5633 + t5656;
  t4041 = -0.022225*t4034;
  t4124 = 0.38008*t4052;
  t4125 = 0. + t4041 + t4124;
  t5528 = t2277*t2181*t2216;
  t5531 = t1393*t2289;
  t5573 = t5528 + t5531;
  t5729 = -1.*t2277*t21*t1004;
  t5745 = t23*t5664;
  t5855 = t5729 + t5745;
  t4393 = -0.38008*t4034;
  t4463 = -0.022225*t4052;
  t4584 = 0. + t4393 + t4463;
  t5876 = t23*t2277*t21;
  t5888 = t1004*t5664;
  t5893 = t5876 + t5888;
  t5916 = t1579*t5573;
  t5931 = -1.*t1940*t5855;
  t5959 = t5916 + t5931;
  t6114 = t2277*t1004*t2216;
  t6122 = t23*t2277*t21*t2289;
  t6128 = t6114 + t6122;
  t6145 = -1.*t23*t2277*t2216;
  t6177 = t2277*t21*t1004*t2289;
  t6189 = t6145 + t6177;
  t6222 = t1579*t2277*t21*t2181;
  t6254 = -1.*t1940*t6128;
  t6264 = t6222 + t6254;
  t6388 = t1004*t1393*t2216;
  t6390 = t23*t21*t1393*t2289;
  t6404 = t6388 + t6390;
  t6422 = -1.*t23*t1393*t2216;
  t6433 = t21*t1004*t1393*t2289;
  t6455 = t6422 + t6433;
  t6532 = t1579*t21*t2181*t1393;
  t6547 = -1.*t1940*t6404;
  t6553 = t6532 + t6547;
  t6604 = t21*t1004;
  t6609 = -1.*t23*t2216*t2289;
  t6612 = t6604 + t6609;
  t6643 = -1.*t23*t21;
  t6668 = -1.*t1004*t2216*t2289;
  t6673 = t6643 + t6668;
  t6707 = -1.*t1579*t2181*t2216;
  t6717 = -1.*t1940*t6612;
  t6718 = t6707 + t6717;
  t6860 = t2181*t1393;
  t6878 = -1.*t2277*t2216*t2289;
  t6911 = t6860 + t6878;
  t6913 = -1.*t23*t1940*t5573;
  t6917 = t1579*t6911;
  t6920 = t6913 + t6917;
  t7008 = t2181*t1393*t2216;
  t7011 = -1.*t2277*t2289;
  t7016 = t7008 + t7011;
  t7033 = -1.*t23*t1940*t7016;
  t7041 = t1579*t3216;
  t7042 = t7033 + t7041;
  t7142 = -1.*t23*t21*t2181*t1940;
  t7144 = -1.*t1579*t21*t2289;
  t7147 = t7142 + t7144;
  t7280 = -1.*t23*t2277*t21;
  t7287 = -1.*t1004*t5664;
  t7289 = t7280 + t7287;
  t7243 = -0.086996*t23;
  t7257 = -0.022225*t1004;
  t7260 = t7243 + t7257;
  t7270 = 0.022225*t23;
  t7273 = t7270 + t1081;
  t7356 = t2277*t2181;
  t7359 = t1393*t2216*t2289;
  t7367 = t7356 + t7359;
  t7386 = -1.*t1004*t7367;
  t7388 = t4162 + t7386;
  t7372 = -1.*t21*t1004*t1393;
  t7378 = t23*t7367;
  t7383 = t7372 + t7378;
  t7487 = t23*t2216;
  t7491 = -1.*t21*t1004*t2289;
  t7492 = t7487 + t7491;
  t7479 = t1004*t2216;
  t7480 = t23*t21*t2289;
  t7485 = t7479 + t7480;
  t7598 = -1.*t1940*t5573;
  t7600 = -1.*t1579*t5855;
  t7603 = t7598 + t7600;
  t7546 = 0.156996*t1579;
  t7548 = t7546 + t3423;
  t7562 = -0.31508*t1579;
  t7578 = -0.156996*t1940;
  t7588 = t7562 + t7578;
  t7634 = -1.*t1940*t7016;
  t7641 = -1.*t1579*t7383;
  t7647 = t7634 + t7641;
  t7702 = -1.*t21*t2181*t1940;
  t7703 = -1.*t1579*t7485;
  t7705 = t7702 + t7703;
  t6029 = t3734*t5893;
  t6034 = -1.*t4052*t5959;
  t6039 = t6029 + t6034;
  t7740 = 0.38008*t3734;
  t7747 = t7740 + t4463;
  t7753 = -0.022225*t3734;
  t7759 = -0.38008*t4052;
  t7765 = t7753 + t7759;
  t7666 = t1579*t7016;
  t7670 = -1.*t1940*t7383;
  t7676 = t7666 + t7670;
  t7820 = t23*t21*t1393;
  t7821 = t1004*t7367;
  t7825 = t7820 + t7821;
  t7730 = t1579*t21*t2181;
  t7732 = -1.*t1940*t7485;
  t7733 = t7730 + t7732;
  t7885 = -1.*t23*t2216;
  t7891 = t21*t1004*t2289;
  t7892 = t7885 + t7891;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=-1.*t1160*t1393*t21 + t2169*t2337 + t2866*t3216 + t3467*t3710 - 0.166996*(t1940*t2337 + t1579*t3710) + t4125*t4202 + t4584*t4826 - 0.38008*(t4052*t4202 + t3734*t4826) - 0.022225*(t3734*t4202 - 1.*t4052*t4826);
  p_output1(10)=t1160*t21*t2277 + t2169*t5573 + t2866*t5664 + t3467*t5855 - 0.166996*(t1940*t5573 + t1579*t5855) + t4125*t5893 + t4584*t5959 - 0.38008*(t4052*t5893 + t3734*t5959) - 0.022225*t6039;
  p_output1(11)=0;
  p_output1(12)=t21*t2169*t2181*t2277 - 1.*t1160*t2216*t2277 + t21*t2277*t2289*t2866 + t3467*t6128 - 0.166996*(t1940*t21*t2181*t2277 + t1579*t6128) + t4125*t6189 + t4584*t6264 - 0.38008*(t4052*t6189 + t3734*t6264) - 0.022225*(t3734*t6189 - 1.*t4052*t6264);
  p_output1(13)=t1393*t21*t2169*t2181 - 1.*t1160*t1393*t2216 + t1393*t21*t2289*t2866 + t3467*t6404 - 0.166996*(t1393*t1940*t21*t2181 + t1579*t6404) + t4125*t6455 + t4584*t6553 - 0.38008*(t4052*t6455 + t3734*t6553) - 0.022225*(t3734*t6455 - 1.*t4052*t6553);
  p_output1(14)=-1.*t1160*t21 - 1.*t2169*t2181*t2216 - 1.*t2216*t2289*t2866 + t3467*t6612 - 0.166996*(-1.*t1940*t2181*t2216 + t1579*t6612) + t4125*t6673 + t4584*t6718 - 0.38008*(t4052*t6673 + t3734*t6718) - 0.022225*(t3734*t6673 - 1.*t4052*t6718);
  p_output1(15)=t2866*t5573 + t23*t3467*t5573 + t1004*t4125*t5573 + t2169*t6911 - 0.166996*(t1579*t23*t5573 + t1940*t6911) + t4584*t6920 - 0.38008*(t1004*t4052*t5573 + t3734*t6920) - 0.022225*(t1004*t3734*t5573 - 1.*t4052*t6920);
  p_output1(16)=t2169*t3216 + t2866*t7016 + t23*t3467*t7016 + t1004*t4125*t7016 - 0.166996*(t1940*t3216 + t1579*t23*t7016) + t4584*t7042 - 0.38008*(t1004*t4052*t7016 + t3734*t7042) - 0.022225*(t1004*t3734*t7016 - 1.*t4052*t7042);
  p_output1(17)=-1.*t21*t2169*t2289 - 0.166996*(-1.*t1940*t21*t2289 + t1579*t21*t2181*t23) + t21*t2181*t2866 + t21*t2181*t23*t3467 + t1004*t21*t2181*t4125 + t4584*t7147 - 0.38008*(t1004*t21*t2181*t4052 + t3734*t7147) - 0.022225*(t1004*t21*t2181*t3734 - 1.*t4052*t7147);
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t4125*t5855 + t21*t2277*t7260 + t5664*t7273 - 0.166996*t1579*t7289 + t3467*t7289 - 1.*t1940*t4584*t7289 - 0.38008*(t4052*t5855 - 1.*t1940*t3734*t7289) - 0.022225*(t3734*t5855 + t1940*t4052*t7289);
  p_output1(34)=t1393*t21*t7260 + t7273*t7367 + t4125*t7383 - 0.166996*t1579*t7388 + t3467*t7388 - 1.*t1940*t4584*t7388 - 0.38008*(t4052*t7383 - 1.*t1940*t3734*t7388) - 0.022225*(t3734*t7383 + t1940*t4052*t7388);
  p_output1(35)=-1.*t2216*t7260 + t21*t2289*t7273 + t4125*t7485 - 0.166996*t1579*t7492 + t3467*t7492 - 1.*t1940*t4584*t7492 - 0.38008*(t4052*t7485 - 1.*t1940*t3734*t7492) - 0.022225*(t3734*t7485 + t1940*t4052*t7492);
  p_output1(36)=-0.166996*t5959 + t5573*t7548 + t5855*t7588 - 0.38008*t3734*t7603 + 0.022225*t4052*t7603 + t4584*t7603;
  p_output1(37)=t7016*t7548 + t7383*t7588 - 0.38008*t3734*t7647 + 0.022225*t4052*t7647 + t4584*t7647 - 0.166996*t7676;
  p_output1(38)=t21*t2181*t7548 + t7485*t7588 - 0.38008*t3734*t7705 + 0.022225*t4052*t7705 + t4584*t7705 - 0.166996*t7733;
  p_output1(39)=-0.022225*(-1.*t4052*t5893 - 1.*t3734*t5959) - 0.38008*t6039 + t5893*t7747 + t5959*t7765;
  p_output1(40)=t7676*t7765 + t7747*t7825 - 0.38008*(-1.*t4052*t7676 + t3734*t7825) - 0.022225*(-1.*t3734*t7676 - 1.*t4052*t7825);
  p_output1(41)=t7733*t7765 + t7747*t7892 - 0.38008*(-1.*t4052*t7733 + t3734*t7892) - 0.022225*(-1.*t3734*t7733 - 1.*t4052*t7892);
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rHipPitch(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
