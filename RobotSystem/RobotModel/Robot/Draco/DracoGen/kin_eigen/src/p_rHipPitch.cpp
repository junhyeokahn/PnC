/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:24 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/p_rHipPitch.h"

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
  double t853;
  double t2892;
  double t2898;
  double t3182;
  double t3238;
  double t4344;
  double t4402;
  double t4347;
  double t4405;
  double t3645;
  double t3835;
  double t3857;
  double t4078;
  double t1840;
  double t4485;
  double t4546;
  double t4550;
  double t4383;
  double t4407;
  double t4442;
  double t4715;
  double t4736;
  double t4783;
  double t4813;
  double t4815;
  double t4828;
  double t4845;
  double t4883;
  double t4888;
  double t4893;
  double t4972;
  double t4978;
  double t4992;
  double t3200;
  double t3258;
  double t3264;
  double t4021;
  double t4109;
  double t4239;
  double t4450;
  double t4451;
  double t4464;
  double t4563;
  double t4591;
  double t4705;
  double t5065;
  double t5071;
  double t5087;
  double t4840;
  double t4868;
  double t4871;
  double t5055;
  double t5060;
  double t5063;
  double t5092;
  double t5094;
  double t5095;
  double t4952;
  double t4970;
  double t4971;
  double t5098;
  double t5105;
  double t5109;
  double t5127;
  double t5134;
  double t5156;
  double t5218;
  double t5225;
  double t5229;
  double t5247;
  double t5250;
  double t5253;
  double t5296;
  double t5297;
  double t5298;
  t853 = Cos(var1[3]);
  t2892 = Cos(var1[11]);
  t2898 = -1.*t2892;
  t3182 = 1. + t2898;
  t3238 = Sin(var1[11]);
  t4344 = Cos(var1[5]);
  t4402 = Sin(var1[3]);
  t4347 = Sin(var1[4]);
  t4405 = Sin(var1[5]);
  t3645 = Cos(var1[12]);
  t3835 = -1.*t3645;
  t3857 = 1. + t3835;
  t4078 = Sin(var1[12]);
  t1840 = Cos(var1[4]);
  t4485 = -1.*t4344*t4402;
  t4546 = t853*t4347*t4405;
  t4550 = t4485 + t4546;
  t4383 = t853*t4344*t4347;
  t4407 = t4402*t4405;
  t4442 = t4383 + t4407;
  t4715 = -1.*t853*t1840*t3238;
  t4736 = t2892*t4550;
  t4783 = t4715 + t4736;
  t4813 = Cos(var1[13]);
  t4815 = -1.*t4813;
  t4828 = 1. + t4815;
  t4845 = Sin(var1[13]);
  t4883 = t2892*t853*t1840;
  t4888 = t3238*t4550;
  t4893 = t4883 + t4888;
  t4972 = t3645*t4442;
  t4978 = -1.*t4078*t4783;
  t4992 = t4972 + t4978;
  t3200 = -0.0222*t3182;
  t3258 = -0.087*t3238;
  t3264 = 0. + t3200 + t3258;
  t4021 = -0.3151*t3857;
  t4109 = 0.157*t4078;
  t4239 = 0. + t4021 + t4109;
  t4450 = -0.087*t3182;
  t4451 = 0.0222*t3238;
  t4464 = 0. + t4450 + t4451;
  t4563 = -0.157*t3857;
  t4591 = -0.3151*t4078;
  t4705 = 0. + t4563 + t4591;
  t5065 = t853*t4344;
  t5071 = t4402*t4347*t4405;
  t5087 = t5065 + t5071;
  t4840 = -0.0222*t4828;
  t4868 = 0.3801*t4845;
  t4871 = 0. + t4840 + t4868;
  t5055 = t4344*t4402*t4347;
  t5060 = -1.*t853*t4405;
  t5063 = t5055 + t5060;
  t5092 = -1.*t1840*t3238*t4402;
  t5094 = t2892*t5087;
  t5095 = t5092 + t5094;
  t4952 = -0.3801*t4828;
  t4970 = -0.0222*t4845;
  t4971 = 0. + t4952 + t4970;
  t5098 = t2892*t1840*t4402;
  t5105 = t3238*t5087;
  t5109 = t5098 + t5105;
  t5127 = t3645*t5063;
  t5134 = -1.*t4078*t5095;
  t5156 = t5127 + t5134;
  t5218 = t3238*t4347;
  t5225 = t2892*t1840*t4405;
  t5229 = t5218 + t5225;
  t5247 = -1.*t2892*t4347;
  t5250 = t1840*t3238*t4405;
  t5253 = t5247 + t5250;
  t5296 = t3645*t1840*t4344;
  t5297 = -1.*t4078*t5229;
  t5298 = t5296 + t5297;

  p_output1(0)=0. + t4239*t4442 + t4464*t4550 + t4705*t4783 - 0.167*(t4078*t4442 + t3645*t4783) + t4871*t4893 + t4971*t4992 - 0.3801*(t4845*t4893 + t4813*t4992) - 0.0222*(t4813*t4893 - 1.*t4845*t4992) + t1840*t3264*t853 + var1(0);
  p_output1(1)=0. + t1840*t3264*t4402 + t4239*t5063 + t4464*t5087 + t4705*t5095 - 0.167*(t4078*t5063 + t3645*t5095) + t4871*t5109 + t4971*t5156 - 0.3801*(t4845*t5109 + t4813*t5156) - 0.0222*(t4813*t5109 - 1.*t4845*t5156) + var1(1);
  p_output1(2)=0. + t1840*t4239*t4344 - 1.*t3264*t4347 + t1840*t4405*t4464 + t4705*t5229 - 0.167*(t1840*t4078*t4344 + t3645*t5229) + t4871*t5253 + t4971*t5298 - 0.3801*(t4845*t5253 + t4813*t5298) - 0.0222*(t4813*t5253 - 1.*t4845*t5298) + var1(2);
}


       
void p_rHipPitch(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
