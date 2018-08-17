/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:40 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lAnkle.h"

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
  double t344;
  double t1041;
  double t1634;
  double t1644;
  double t1831;
  double t188;
  double t312;
  double t340;
  double t381;
  double t385;
  double t517;
  double t541;
  double t2152;
  double t3776;
  double t3806;
  double t4546;
  double t4614;
  double t3336;
  double t3501;
  double t3621;
  double t4761;
  double t5030;
  double t5346;
  double t6002;
  double t6003;
  double t6020;
  double t6035;
  double t5973;
  double t5976;
  double t5982;
  double t6093;
  double t6110;
  double t6114;
  double t6154;
  double t6166;
  double t6183;
  double t6215;
  double t6268;
  double t6270;
  double t6276;
  double t6331;
  double t6350;
  double t6357;
  double t6369;
  double t6411;
  double t6420;
  double t6444;
  double t6474;
  double t6475;
  double t6482;
  double t6511;
  double t6514;
  double t6522;
  double t1708;
  double t1837;
  double t1838;
  double t2589;
  double t3038;
  double t3089;
  double t6611;
  double t6612;
  double t6621;
  double t4591;
  double t4715;
  double t4723;
  double t5573;
  double t5709;
  double t5748;
  double t6668;
  double t6674;
  double t6676;
  double t6689;
  double t6691;
  double t6718;
  double t6034;
  double t6036;
  double t6039;
  double t6116;
  double t6119;
  double t6123;
  double t6192;
  double t6240;
  double t6267;
  double t6763;
  double t6770;
  double t6773;
  double t6788;
  double t6792;
  double t6796;
  double t6308;
  double t6309;
  double t6316;
  double t6432;
  double t6456;
  double t6458;
  double t6802;
  double t6815;
  double t6819;
  double t6823;
  double t6827;
  double t6832;
  double t6495;
  double t6501;
  double t6503;
  double t6858;
  double t6871;
  double t6888;
  double t6891;
  double t6892;
  double t6905;
  double t7017;
  double t7033;
  double t7038;
  double t7101;
  double t7112;
  double t7119;
  double t7134;
  double t7142;
  double t7148;
  double t7158;
  double t7163;
  double t7167;
  double t7196;
  double t7198;
  double t7200;
  double t7207;
  double t7209;
  double t7210;
  double t7215;
  double t7222;
  double t7223;
  t344 = Cos(var1[3]);
  t1041 = Cos(var1[6]);
  t1634 = -1.*t1041;
  t1644 = 1. + t1634;
  t1831 = Sin(var1[6]);
  t188 = Cos(var1[5]);
  t312 = Sin(var1[3]);
  t340 = -1.*t188*t312;
  t381 = Sin(var1[4]);
  t385 = Sin(var1[5]);
  t517 = t344*t381*t385;
  t541 = t340 + t517;
  t2152 = Cos(var1[4]);
  t3776 = Cos(var1[7]);
  t3806 = -1.*t3776;
  t4546 = 1. + t3806;
  t4614 = Sin(var1[7]);
  t3336 = t1041*t541;
  t3501 = -1.*t344*t2152*t1831;
  t3621 = t3336 + t3501;
  t4761 = t344*t188*t381;
  t5030 = t312*t385;
  t5346 = t4761 + t5030;
  t6002 = Cos(var1[8]);
  t6003 = -1.*t6002;
  t6020 = 1. + t6003;
  t6035 = Sin(var1[8]);
  t5973 = t3776*t5346;
  t5976 = -1.*t3621*t4614;
  t5982 = t5973 + t5976;
  t6093 = t344*t2152*t1041;
  t6110 = t541*t1831;
  t6114 = t6093 + t6110;
  t6154 = Cos(var1[9]);
  t6166 = -1.*t6154;
  t6183 = 1. + t6166;
  t6215 = Sin(var1[9]);
  t6268 = t6002*t5982;
  t6270 = t6114*t6035;
  t6276 = t6268 + t6270;
  t6331 = t6002*t6114;
  t6350 = -1.*t5982*t6035;
  t6357 = t6331 + t6350;
  t6369 = Cos(var1[10]);
  t6411 = -1.*t6369;
  t6420 = 1. + t6411;
  t6444 = Sin(var1[10]);
  t6474 = -1.*t6215*t6276;
  t6475 = t6154*t6357;
  t6482 = t6474 + t6475;
  t6511 = t6154*t6276;
  t6514 = t6215*t6357;
  t6522 = t6511 + t6514;
  t1708 = 0.087004*t1644;
  t1837 = 0.022225*t1831;
  t1838 = 0. + t1708 + t1837;
  t2589 = -0.022225*t1644;
  t3038 = 0.087004*t1831;
  t3089 = 0. + t2589 + t3038;
  t6611 = t344*t188;
  t6612 = t312*t381*t385;
  t6621 = t6611 + t6612;
  t4591 = 0.157004*t4546;
  t4715 = -0.31508*t4614;
  t4723 = 0. + t4591 + t4715;
  t5573 = -0.31508*t4546;
  t5709 = -0.157004*t4614;
  t5748 = 0. + t5573 + t5709;
  t6668 = t1041*t6621;
  t6674 = -1.*t2152*t312*t1831;
  t6676 = t6668 + t6674;
  t6689 = t188*t312*t381;
  t6691 = -1.*t344*t385;
  t6718 = t6689 + t6691;
  t6034 = -0.38008*t6020;
  t6036 = -0.022225*t6035;
  t6039 = 0. + t6034 + t6036;
  t6116 = -0.022225*t6020;
  t6119 = 0.38008*t6035;
  t6123 = 0. + t6116 + t6119;
  t6192 = -0.86008*t6183;
  t6240 = -0.022225*t6215;
  t6267 = 0. + t6192 + t6240;
  t6763 = t3776*t6718;
  t6770 = -1.*t6676*t4614;
  t6773 = t6763 + t6770;
  t6788 = t2152*t1041*t312;
  t6792 = t6621*t1831;
  t6796 = t6788 + t6792;
  t6308 = -0.022225*t6183;
  t6309 = 0.86008*t6215;
  t6316 = 0. + t6308 + t6309;
  t6432 = -0.021147*t6420;
  t6456 = 1.34008*t6444;
  t6458 = 0. + t6432 + t6456;
  t6802 = t6002*t6773;
  t6815 = t6796*t6035;
  t6819 = t6802 + t6815;
  t6823 = t6002*t6796;
  t6827 = -1.*t6773*t6035;
  t6832 = t6823 + t6827;
  t6495 = -1.34008*t6420;
  t6501 = -0.021147*t6444;
  t6503 = 0. + t6495 + t6501;
  t6858 = -1.*t6215*t6819;
  t6871 = t6154*t6832;
  t6888 = t6858 + t6871;
  t6891 = t6154*t6819;
  t6892 = t6215*t6832;
  t6905 = t6891 + t6892;
  t7017 = t2152*t1041*t385;
  t7033 = t381*t1831;
  t7038 = t7017 + t7033;
  t7101 = t2152*t188*t3776;
  t7112 = -1.*t7038*t4614;
  t7119 = t7101 + t7112;
  t7134 = -1.*t1041*t381;
  t7142 = t2152*t385*t1831;
  t7148 = t7134 + t7142;
  t7158 = t6002*t7119;
  t7163 = t7148*t6035;
  t7167 = t7158 + t7163;
  t7196 = t6002*t7148;
  t7198 = -1.*t7119*t6035;
  t7200 = t7196 + t7198;
  t7207 = -1.*t6215*t7167;
  t7209 = t6154*t7200;
  t7210 = t7207 + t7209;
  t7215 = t6154*t7167;
  t7222 = t6215*t7200;
  t7223 = t7215 + t7222;

  p_output1(0)=0. + t2152*t3089*t344 + t3621*t4723 + 0.167004*(t3621*t3776 + t4614*t5346) + t1838*t541 + t5346*t5748 + t5982*t6039 + t6114*t6123 + t6267*t6276 + t6316*t6357 + t6458*t6482 + t6503*t6522 - 1.34008*(t6444*t6482 + t6369*t6522) - 0.021147*(t6369*t6482 - 1.*t6444*t6522) + var1(0);
  p_output1(1)=0. + t2152*t3089*t312 + t1838*t6621 + t4723*t6676 + t5748*t6718 + 0.167004*(t3776*t6676 + t4614*t6718) + t6039*t6773 + t6123*t6796 + t6267*t6819 + t6316*t6832 + t6458*t6888 + t6503*t6905 - 1.34008*(t6444*t6888 + t6369*t6905) - 0.021147*(t6369*t6888 - 1.*t6444*t6905) + var1(1);
  p_output1(2)=0. - 1.*t3089*t381 + t1838*t2152*t385 + t188*t2152*t5748 + t4723*t7038 + 0.167004*(t188*t2152*t4614 + t3776*t7038) + t6039*t7119 + t6123*t7148 + t6267*t7167 + t6316*t7200 + t6458*t7210 + t6503*t7223 - 1.34008*(t6444*t7210 + t6369*t7223) - 0.021147*(t6369*t7210 - 1.*t6444*t7223) + var1(2);
}


       
void p_lAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
