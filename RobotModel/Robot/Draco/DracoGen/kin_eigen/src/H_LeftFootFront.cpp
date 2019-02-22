/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:31 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/H_LeftFootFront.h"

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
  double t561;
  double t453;
  double t584;
  double t462;
  double t621;
  double t40;
  double t452;
  double t740;
  double t743;
  double t973;
  double t873;
  double t878;
  double t963;
  double t534;
  double t632;
  double t651;
  double t707;
  double t744;
  double t833;
  double t1740;
  double t1882;
  double t1885;
  double t1920;
  double t1767;
  double t1791;
  double t1822;
  double t1964;
  double t2038;
  double t1849;
  double t1974;
  double t1982;
  double t1719;
  double t2216;
  double t2217;
  double t2269;
  double t2367;
  double t2007;
  double t2313;
  double t2316;
  double t1662;
  double t2406;
  double t2421;
  double t2497;
  double t1236;
  double t1271;
  double t1348;
  double t1047;
  double t1057;
  double t1094;
  double t1103;
  double t1145;
  double t1213;
  double t2745;
  double t2769;
  double t2781;
  double t2642;
  double t2652;
  double t2673;
  double t2733;
  double t2788;
  double t2827;
  double t2918;
  double t2946;
  double t2966;
  double t2905;
  double t2969;
  double t2977;
  double t3069;
  double t3087;
  double t3109;
  double t1429;
  double t1459;
  double t1496;
  double t3403;
  double t3429;
  double t3441;
  double t3230;
  double t3231;
  double t3317;
  double t3363;
  double t3466;
  double t3509;
  double t3541;
  double t3552;
  double t3553;
  double t3535;
  double t3556;
  double t3565;
  double t3592;
  double t3602;
  double t3611;
  double t4367;
  double t4408;
  double t4616;
  double t4639;
  double t867;
  double t983;
  double t1009;
  double t4885;
  double t4899;
  double t5042;
  double t5075;
  double t5227;
  double t5234;
  double t3766;
  double t3810;
  double t3833;
  double t2539;
  double t2552;
  double t2564;
  double t4427;
  double t4467;
  double t4515;
  double t4536;
  double t4544;
  double t4562;
  double t4645;
  double t4742;
  double t4843;
  double t4857;
  double t4858;
  double t4863;
  double t1235;
  double t1385;
  double t1386;
  double t4912;
  double t4917;
  double t4957;
  double t5007;
  double t5009;
  double t5027;
  double t5077;
  double t5125;
  double t5131;
  double t5139;
  double t5191;
  double t5207;
  double t5241;
  double t5243;
  double t5249;
  double t5321;
  double t5357;
  double t5395;
  double t4006;
  double t4027;
  double t4031;
  double t3131;
  double t3133;
  double t3209;
  double t1554;
  double t1620;
  double t1638;
  double t4192;
  double t4198;
  double t4258;
  double t3649;
  double t3658;
  double t3660;
  t561 = Cos(var1[3]);
  t453 = Cos(var1[5]);
  t584 = Sin(var1[4]);
  t462 = Sin(var1[3]);
  t621 = Sin(var1[5]);
  t40 = Cos(var1[7]);
  t452 = Cos(var1[6]);
  t740 = Cos(var1[4]);
  t743 = Sin(var1[6]);
  t973 = Sin(var1[7]);
  t873 = t561*t453*t584;
  t878 = t462*t621;
  t963 = t873 + t878;
  t534 = -1.*t453*t462;
  t632 = t561*t584*t621;
  t651 = t534 + t632;
  t707 = t452*t651;
  t744 = -1.*t561*t740*t743;
  t833 = t707 + t744;
  t1740 = Cos(var1[8]);
  t1882 = t561*t740*t452;
  t1885 = t651*t743;
  t1920 = t1882 + t1885;
  t1767 = t40*t963;
  t1791 = -1.*t833*t973;
  t1822 = t1767 + t1791;
  t1964 = Sin(var1[8]);
  t2038 = Cos(var1[9]);
  t1849 = t1740*t1822;
  t1974 = t1920*t1964;
  t1982 = t1849 + t1974;
  t1719 = Sin(var1[9]);
  t2216 = t1740*t1920;
  t2217 = -1.*t1822*t1964;
  t2269 = t2216 + t2217;
  t2367 = Cos(var1[10]);
  t2007 = -1.*t1719*t1982;
  t2313 = t2038*t2269;
  t2316 = t2007 + t2313;
  t1662 = Sin(var1[10]);
  t2406 = t2038*t1982;
  t2421 = t1719*t2269;
  t2497 = t2406 + t2421;
  t1236 = t453*t462*t584;
  t1271 = -1.*t561*t621;
  t1348 = t1236 + t1271;
  t1047 = t561*t453;
  t1057 = t462*t584*t621;
  t1094 = t1047 + t1057;
  t1103 = t452*t1094;
  t1145 = -1.*t740*t462*t743;
  t1213 = t1103 + t1145;
  t2745 = t740*t452*t462;
  t2769 = t1094*t743;
  t2781 = t2745 + t2769;
  t2642 = t40*t1348;
  t2652 = -1.*t1213*t973;
  t2673 = t2642 + t2652;
  t2733 = t1740*t2673;
  t2788 = t2781*t1964;
  t2827 = t2733 + t2788;
  t2918 = t1740*t2781;
  t2946 = -1.*t2673*t1964;
  t2966 = t2918 + t2946;
  t2905 = -1.*t1719*t2827;
  t2969 = t2038*t2966;
  t2977 = t2905 + t2969;
  t3069 = t2038*t2827;
  t3087 = t1719*t2966;
  t3109 = t3069 + t3087;
  t1429 = t740*t452*t621;
  t1459 = t584*t743;
  t1496 = t1429 + t1459;
  t3403 = -1.*t452*t584;
  t3429 = t740*t621*t743;
  t3441 = t3403 + t3429;
  t3230 = t740*t453*t40;
  t3231 = -1.*t1496*t973;
  t3317 = t3230 + t3231;
  t3363 = t1740*t3317;
  t3466 = t3441*t1964;
  t3509 = t3363 + t3466;
  t3541 = t1740*t3441;
  t3552 = -1.*t3317*t1964;
  t3553 = t3541 + t3552;
  t3535 = -1.*t1719*t3509;
  t3556 = t2038*t3553;
  t3565 = t3535 + t3556;
  t3592 = t2038*t3509;
  t3602 = t1719*t3553;
  t3611 = t3592 + t3602;
  t4367 = -1.*t452;
  t4408 = 1. + t4367;
  t4616 = -1.*t40;
  t4639 = 1. + t4616;
  t867 = t40*t833;
  t983 = t963*t973;
  t1009 = t867 + t983;
  t4885 = -1.*t1740;
  t4899 = 1. + t4885;
  t5042 = -1.*t2038;
  t5075 = 1. + t5042;
  t5227 = -1.*t2367;
  t5234 = 1. + t5227;
  t3766 = t1662*t2316;
  t3810 = t2367*t2497;
  t3833 = t3766 + t3810;
  t2539 = t2367*t2316;
  t2552 = -1.*t1662*t2497;
  t2564 = t2539 + t2552;
  t4427 = 0.087*t4408;
  t4467 = 0.0222*t743;
  t4515 = 0. + t4427 + t4467;
  t4536 = -0.0222*t4408;
  t4544 = 0.087*t743;
  t4562 = 0. + t4536 + t4544;
  t4645 = 0.157*t4639;
  t4742 = -0.3151*t973;
  t4843 = 0. + t4645 + t4742;
  t4857 = -0.3151*t4639;
  t4858 = -0.157*t973;
  t4863 = 0. + t4857 + t4858;
  t1235 = t40*t1213;
  t1385 = t1348*t973;
  t1386 = t1235 + t1385;
  t4912 = -0.3801*t4899;
  t4917 = -0.0222*t1964;
  t4957 = 0. + t4912 + t4917;
  t5007 = -0.0222*t4899;
  t5009 = 0.3801*t1964;
  t5027 = 0. + t5007 + t5009;
  t5077 = -0.8601*t5075;
  t5125 = -0.0222*t1719;
  t5131 = 0. + t5077 + t5125;
  t5139 = -0.0222*t5075;
  t5191 = 0.8601*t1719;
  t5207 = 0. + t5139 + t5191;
  t5241 = -0.0211*t5234;
  t5243 = 1.3401*t1662;
  t5249 = 0. + t5241 + t5243;
  t5321 = -1.3401*t5234;
  t5357 = -0.0211*t1662;
  t5395 = 0. + t5321 + t5357;
  t4006 = t1662*t2977;
  t4027 = t2367*t3109;
  t4031 = t4006 + t4027;
  t3131 = t2367*t2977;
  t3133 = -1.*t1662*t3109;
  t3209 = t3131 + t3133;
  t1554 = t40*t1496;
  t1620 = t740*t453*t973;
  t1638 = t1554 + t1620;
  t4192 = t1662*t3565;
  t4198 = t2367*t3611;
  t4258 = t4192 + t4198;
  t3649 = t2367*t3565;
  t3658 = -1.*t1662*t3611;
  t3660 = t3649 + t3658;

  p_output1(0)=t1009;
  p_output1(1)=t1386;
  p_output1(2)=t1638;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1662*t2316 - 1.*t2367*t2497 - 0.000796*t2564;
  p_output1(5)=-1.*t1662*t2977 - 1.*t2367*t3109 - 0.000796*t3209;
  p_output1(6)=-1.*t1662*t3565 - 1.*t2367*t3611 - 0.000796*t3660;
  p_output1(7)=0.;
  p_output1(8)=-1.*t2316*t2367 + t1662*t2497 + 0.000796*t3833;
  p_output1(9)=-1.*t2367*t2977 + t1662*t3109 + 0.000796*t4031;
  p_output1(10)=-1.*t2367*t3565 + t1662*t3611 + 0.000796*t4258;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.242*t1009 + 0.043912*t2564 - 1.325152*t3833 + t1822*t4957 + t1920*t5027 + t1982*t5131 + t2269*t5207 + t2316*t5249 + t2497*t5395 + t4515*t651 + t4562*t561*t740 + t4843*t833 + t4863*t963 + var1(0);
  p_output1(13)=0. + 0.242*t1386 + 0.043912*t3209 - 1.325152*t4031 + t1094*t4515 + t1213*t4843 + t1348*t4863 + t2673*t4957 + t2781*t5027 + t2827*t5131 + t2966*t5207 + t2977*t5249 + t3109*t5395 + t4562*t462*t740 + var1(1);
  p_output1(14)=0. + 0.242*t1638 + 0.043912*t3660 - 1.325152*t4258 + t1496*t4843 + t3317*t4957 + t3441*t5027 + t3509*t5131 + t3553*t5207 + t3565*t5249 + t3611*t5395 - 1.*t4562*t584 + t453*t4863*t740 + t4515*t621*t740 + var1(2);
  p_output1(15)=1.;
}


       
void H_LeftFootFront(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
