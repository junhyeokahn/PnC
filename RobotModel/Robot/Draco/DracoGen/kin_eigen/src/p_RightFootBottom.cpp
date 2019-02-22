/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:34 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_RightFootBottom.h"

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
  double t362;
  double t891;
  double t957;
  double t1002;
  double t1160;
  double t1945;
  double t2079;
  double t2001;
  double t2170;
  double t1455;
  double t1478;
  double t1499;
  double t1765;
  double t886;
  double t2600;
  double t2641;
  double t2686;
  double t2020;
  double t2337;
  double t2450;
  double t3140;
  double t3164;
  double t3186;
  double t3241;
  double t3244;
  double t3301;
  double t3332;
  double t3452;
  double t3454;
  double t3514;
  double t3911;
  double t3945;
  double t3946;
  double t4066;
  double t4071;
  double t4125;
  double t4187;
  double t4337;
  double t4400;
  double t4506;
  double t4635;
  double t4662;
  double t4677;
  double t4710;
  double t4726;
  double t4785;
  double t4814;
  double t4853;
  double t4857;
  double t4860;
  double t4952;
  double t4954;
  double t4955;
  double t1141;
  double t1306;
  double t1341;
  double t1640;
  double t1777;
  double t1938;
  double t2520;
  double t2529;
  double t2555;
  double t2907;
  double t2994;
  double t3031;
  double t5191;
  double t5216;
  double t5236;
  double t3318;
  double t3351;
  double t3361;
  double t5124;
  double t5131;
  double t5173;
  double t5257;
  double t5266;
  double t5269;
  double t3783;
  double t3805;
  double t3876;
  double t4156;
  double t4249;
  double t4258;
  double t5293;
  double t5314;
  double t5325;
  double t5369;
  double t5381;
  double t5395;
  double t4583;
  double t4620;
  double t4634;
  double t4808;
  double t4820;
  double t4849;
  double t5398;
  double t5405;
  double t5430;
  double t5450;
  double t5465;
  double t5473;
  double t4911;
  double t4917;
  double t4942;
  double t5475;
  double t5476;
  double t5478;
  double t5486;
  double t5487;
  double t5511;
  double t5687;
  double t5692;
  double t5696;
  double t5703;
  double t5705;
  double t5712;
  double t5729;
  double t5737;
  double t5800;
  double t5809;
  double t5812;
  double t5825;
  double t5839;
  double t5843;
  double t5849;
  double t5860;
  double t5864;
  double t5871;
  double t5884;
  double t5896;
  double t5913;
  t362 = Cos(var1[3]);
  t891 = Cos(var1[11]);
  t957 = -1.*t891;
  t1002 = 1. + t957;
  t1160 = Sin(var1[11]);
  t1945 = Cos(var1[5]);
  t2079 = Sin(var1[3]);
  t2001 = Sin(var1[4]);
  t2170 = Sin(var1[5]);
  t1455 = Cos(var1[12]);
  t1478 = -1.*t1455;
  t1499 = 1. + t1478;
  t1765 = Sin(var1[12]);
  t886 = Cos(var1[4]);
  t2600 = -1.*t1945*t2079;
  t2641 = t362*t2001*t2170;
  t2686 = t2600 + t2641;
  t2020 = t362*t1945*t2001;
  t2337 = t2079*t2170;
  t2450 = t2020 + t2337;
  t3140 = -1.*t362*t886*t1160;
  t3164 = t891*t2686;
  t3186 = t3140 + t3164;
  t3241 = Cos(var1[13]);
  t3244 = -1.*t3241;
  t3301 = 1. + t3244;
  t3332 = Sin(var1[13]);
  t3452 = t891*t362*t886;
  t3454 = t1160*t2686;
  t3514 = t3452 + t3454;
  t3911 = t1455*t2450;
  t3945 = -1.*t1765*t3186;
  t3946 = t3911 + t3945;
  t4066 = Cos(var1[14]);
  t4071 = -1.*t4066;
  t4125 = 1. + t4071;
  t4187 = Sin(var1[14]);
  t4337 = t3332*t3514;
  t4400 = t3241*t3946;
  t4506 = t4337 + t4400;
  t4635 = t3241*t3514;
  t4662 = -1.*t3332*t3946;
  t4677 = t4635 + t4662;
  t4710 = Cos(var1[15]);
  t4726 = -1.*t4710;
  t4785 = 1. + t4726;
  t4814 = Sin(var1[15]);
  t4853 = -1.*t4187*t4506;
  t4857 = t4066*t4677;
  t4860 = t4853 + t4857;
  t4952 = t4066*t4506;
  t4954 = t4187*t4677;
  t4955 = t4952 + t4954;
  t1141 = -0.0222*t1002;
  t1306 = -0.087*t1160;
  t1341 = 0. + t1141 + t1306;
  t1640 = -0.3151*t1499;
  t1777 = 0.157*t1765;
  t1938 = 0. + t1640 + t1777;
  t2520 = -0.087*t1002;
  t2529 = 0.0222*t1160;
  t2555 = 0. + t2520 + t2529;
  t2907 = -0.157*t1499;
  t2994 = -0.3151*t1765;
  t3031 = 0. + t2907 + t2994;
  t5191 = t362*t1945;
  t5216 = t2079*t2001*t2170;
  t5236 = t5191 + t5216;
  t3318 = -0.0222*t3301;
  t3351 = 0.3801*t3332;
  t3361 = 0. + t3318 + t3351;
  t5124 = t1945*t2079*t2001;
  t5131 = -1.*t362*t2170;
  t5173 = t5124 + t5131;
  t5257 = -1.*t886*t1160*t2079;
  t5266 = t891*t5236;
  t5269 = t5257 + t5266;
  t3783 = -0.3801*t3301;
  t3805 = -0.0222*t3332;
  t3876 = 0. + t3783 + t3805;
  t4156 = -0.8601*t4125;
  t4249 = -0.0222*t4187;
  t4258 = 0. + t4156 + t4249;
  t5293 = t891*t886*t2079;
  t5314 = t1160*t5236;
  t5325 = t5293 + t5314;
  t5369 = t1455*t5173;
  t5381 = -1.*t1765*t5269;
  t5395 = t5369 + t5381;
  t4583 = -0.0222*t4125;
  t4620 = 0.8601*t4187;
  t4634 = 0. + t4583 + t4620;
  t4808 = -0.0211*t4785;
  t4820 = 1.3401*t4814;
  t4849 = 0. + t4808 + t4820;
  t5398 = t3332*t5325;
  t5405 = t3241*t5395;
  t5430 = t5398 + t5405;
  t5450 = t3241*t5325;
  t5465 = -1.*t3332*t5395;
  t5473 = t5450 + t5465;
  t4911 = -1.3401*t4785;
  t4917 = -0.0211*t4814;
  t4942 = 0. + t4911 + t4917;
  t5475 = -1.*t4187*t5430;
  t5476 = t4066*t5473;
  t5478 = t5475 + t5476;
  t5486 = t4066*t5430;
  t5487 = t4187*t5473;
  t5511 = t5486 + t5487;
  t5687 = t1160*t2001;
  t5692 = t891*t886*t2170;
  t5696 = t5687 + t5692;
  t5703 = -1.*t891*t2001;
  t5705 = t886*t1160*t2170;
  t5712 = t5703 + t5705;
  t5729 = t1455*t886*t1945;
  t5737 = -1.*t1765*t5696;
  t5800 = t5729 + t5737;
  t5809 = t3332*t5712;
  t5812 = t3241*t5800;
  t5825 = t5809 + t5812;
  t5839 = t3241*t5712;
  t5843 = -1.*t3332*t5800;
  t5849 = t5839 + t5843;
  t5860 = -1.*t4187*t5825;
  t5864 = t4066*t5849;
  t5871 = t5860 + t5864;
  t5884 = t4066*t5825;
  t5896 = t4187*t5849;
  t5913 = t5884 + t5896;

  p_output1(0)=0. + t1938*t2450 + t2555*t2686 + t3031*t3186 - 0.16705*(t1765*t2450 + t1455*t3186) + t3361*t3514 + t3876*t3946 + t4258*t4506 + t4634*t4677 + t4849*t4860 + t4942*t4955 - 1.325152*(t4814*t4860 + t4710*t4955) + 0.043912*(t4710*t4860 - 1.*t4814*t4955) + t1341*t362*t886 + var1(0);
  p_output1(1)=0. + t1938*t5173 + t2555*t5236 + t3031*t5269 - 0.16705*(t1765*t5173 + t1455*t5269) + t3361*t5325 + t3876*t5395 + t4258*t5430 + t4634*t5473 + t4849*t5478 + t4942*t5511 - 1.325152*(t4814*t5478 + t4710*t5511) + 0.043912*(t4710*t5478 - 1.*t4814*t5511) + t1341*t2079*t886 + var1(1);
  p_output1(2)=0. - 1.*t1341*t2001 + t3031*t5696 + t3361*t5712 + t3876*t5800 + t4258*t5825 + t4634*t5849 + t4849*t5871 + t4942*t5913 - 1.325152*(t4814*t5871 + t4710*t5913) + 0.043912*(t4710*t5871 - 1.*t4814*t5913) + t1938*t1945*t886 + t2170*t2555*t886 - 0.16705*(t1455*t5696 + t1765*t1945*t886) + var1(2);
}


       
void p_RightFootBottom(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
