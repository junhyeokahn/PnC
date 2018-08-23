/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:25 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_rHipPitch.h"

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
  double t872;
  double t1160;
  double t1365;
  double t1304;
  double t1600;
  double t881;
  double t991;
  double t808;
  double t1352;
  double t1887;
  double t2255;
  double t156;
  double t2507;
  double t2512;
  double t2741;
  double t3453;
  double t3460;
  double t3546;
  double t2575;
  double t2586;
  double t2644;
  double t2763;
  double t2996;
  double t3036;
  double t4276;
  double t4346;
  double t4407;
  double t4560;
  double t4563;
  double t4625;
  double t4806;
  double t4867;
  double t4883;
  double t950;
  double t2474;
  double t2477;
  double t2665;
  double t3064;
  double t3136;
  double t3356;
  double t3633;
  double t3987;
  double t4422;
  double t4629;
  double t4644;
  double t4725;
  double t4729;
  double t4736;
  double t4801;
  double t4929;
  double t4939;
  double t5166;
  double t5168;
  double t5218;
  double t5230;
  double t4997;
  double t5012;
  double t5025;
  double t5447;
  double t5448;
  double t5064;
  double t5087;
  double t5089;
  double t2499;
  double t3295;
  double t3347;
  double t5188;
  double t5210;
  double t5213;
  double t5242;
  double t5258;
  double t5259;
  double t5291;
  double t5306;
  double t5318;
  double t5336;
  double t5379;
  double t5426;
  double t5451;
  double t5454;
  double t5455;
  double t5032;
  double t5040;
  double t5044;
  double t5485;
  double t5502;
  double t5505;
  double t5098;
  double t5120;
  double t5126;
  double t3988;
  double t4705;
  double t4714;
  double t5048;
  double t5049;
  double t5061;
  double t5127;
  double t5160;
  double t5163;
  double t4791;
  double t4983;
  double t4984;
  t872 = Cos(var1[3]);
  t1160 = Cos(var1[5]);
  t1365 = Sin(var1[4]);
  t1304 = Sin(var1[3]);
  t1600 = Sin(var1[5]);
  t881 = Cos(var1[4]);
  t991 = Sin(var1[11]);
  t808 = Cos(var1[11]);
  t1352 = -1.*t1160*t1304;
  t1887 = t872*t1365*t1600;
  t2255 = t1352 + t1887;
  t156 = Cos(var1[13]);
  t2507 = Sin(var1[13]);
  t2512 = Cos(var1[12]);
  t2741 = Sin(var1[12]);
  t3453 = t872*t1160;
  t3460 = t1304*t1365*t1600;
  t3546 = t3453 + t3460;
  t2575 = t872*t1160*t1365;
  t2586 = t1304*t1600;
  t2644 = t2575 + t2586;
  t2763 = -1.*t872*t881*t991;
  t2996 = t808*t2255;
  t3036 = t2763 + t2996;
  t4276 = t1160*t1304*t1365;
  t4346 = -1.*t872*t1600;
  t4407 = t4276 + t4346;
  t4560 = -1.*t881*t991*t1304;
  t4563 = t808*t3546;
  t4625 = t4560 + t4563;
  t4806 = t991*t1365;
  t4867 = t808*t881*t1600;
  t4883 = t4806 + t4867;
  t950 = t808*t872*t881;
  t2474 = t991*t2255;
  t2477 = t950 + t2474;
  t2665 = t2512*t2644;
  t3064 = -1.*t2741*t3036;
  t3136 = t2665 + t3064;
  t3356 = t808*t881*t1304;
  t3633 = t991*t3546;
  t3987 = t3356 + t3633;
  t4422 = t2512*t4407;
  t4629 = -1.*t2741*t4625;
  t4644 = t4422 + t4629;
  t4725 = -1.*t808*t1365;
  t4729 = t881*t991*t1600;
  t4736 = t4725 + t4729;
  t4801 = t2512*t881*t1160;
  t4929 = -1.*t2741*t4883;
  t4939 = t4801 + t4929;
  t5166 = -1.*t808;
  t5168 = 1. + t5166;
  t5218 = -1.*t2512;
  t5230 = 1. + t5218;
  t4997 = t2741*t2644;
  t5012 = t2512*t3036;
  t5025 = t4997 + t5012;
  t5447 = -1.*t156;
  t5448 = 1. + t5447;
  t5064 = t2507*t2477;
  t5087 = t156*t3136;
  t5089 = t5064 + t5087;
  t2499 = t156*t2477;
  t3295 = -1.*t2507*t3136;
  t3347 = t2499 + t3295;
  t5188 = -0.0222*t5168;
  t5210 = -0.087*t991;
  t5213 = 0. + t5188 + t5210;
  t5242 = -0.3151*t5230;
  t5258 = 0.157*t2741;
  t5259 = 0. + t5242 + t5258;
  t5291 = -0.087*t5168;
  t5306 = 0.0222*t991;
  t5318 = 0. + t5291 + t5306;
  t5336 = -0.157*t5230;
  t5379 = -0.3151*t2741;
  t5426 = 0. + t5336 + t5379;
  t5451 = -0.0222*t5448;
  t5454 = 0.3801*t2507;
  t5455 = 0. + t5451 + t5454;
  t5032 = t2741*t4407;
  t5040 = t2512*t4625;
  t5044 = t5032 + t5040;
  t5485 = -0.3801*t5448;
  t5502 = -0.0222*t2507;
  t5505 = 0. + t5485 + t5502;
  t5098 = t2507*t3987;
  t5120 = t156*t4644;
  t5126 = t5098 + t5120;
  t3988 = t156*t3987;
  t4705 = -1.*t2507*t4644;
  t4714 = t3988 + t4705;
  t5048 = t881*t1160*t2741;
  t5049 = t2512*t4883;
  t5061 = t5048 + t5049;
  t5127 = t2507*t4736;
  t5160 = t156*t4939;
  t5163 = t5127 + t5160;
  t4791 = t156*t4736;
  t4983 = -1.*t2507*t4939;
  t4984 = t4791 + t4983;

  p_output1(0)=t3347;
  p_output1(1)=t4714;
  p_output1(2)=t4984;
  p_output1(3)=0.;
  p_output1(4)=t5025;
  p_output1(5)=t5044;
  p_output1(6)=t5061;
  p_output1(7)=0.;
  p_output1(8)=t5089;
  p_output1(9)=t5126;
  p_output1(10)=t5163;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.0222*t3347 - 0.167*t5025 - 0.3801*t5089 + t2644*t5259 + t2255*t5318 + t3036*t5426 + t2477*t5455 + t3136*t5505 + t5213*t872*t881 + var1(0);
  p_output1(13)=0. - 0.0222*t4714 - 0.167*t5044 - 0.3801*t5126 + t4407*t5259 + t3546*t5318 + t4625*t5426 + t3987*t5455 + t4644*t5505 + t1304*t5213*t881 + var1(1);
  p_output1(14)=0. - 0.0222*t4984 - 0.167*t5061 - 0.3801*t5163 - 1.*t1365*t5213 + t4883*t5426 + t4736*t5455 + t4939*t5505 + t1160*t5259*t881 + t1600*t5318*t881 + var1(2);
  p_output1(15)=1.;
}


       
void H_rHipPitch(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
