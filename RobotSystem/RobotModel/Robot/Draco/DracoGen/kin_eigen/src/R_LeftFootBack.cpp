/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:59 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_LeftFootBack.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t600;
  double t1143;
  double t578;
  double t633;
  double t1259;
  double t2436;
  double t1504;
  double t1534;
  double t1735;
  double t2123;
  double t2445;
  double t406;
  double t2877;
  double t2942;
  double t2972;
  double t467;
  double t703;
  double t1279;
  double t1300;
  double t1399;
  double t2420;
  double t2474;
  double t2526;
  double t2638;
  double t2690;
  double t2815;
  double t2985;
  double t3134;
  double t2843;
  double t3052;
  double t3099;
  double t112;
  double t3139;
  double t3157;
  double t3179;
  double t3411;
  double t3105;
  double t3220;
  double t3300;
  double t94;
  double t3441;
  double t3455;
  double t3460;
  double t3837;
  double t3878;
  double t3900;
  double t4003;
  double t4038;
  double t4130;
  double t3701;
  double t3711;
  double t3738;
  double t3761;
  double t3910;
  double t3911;
  double t3917;
  double t3929;
  double t3943;
  double t3961;
  double t4146;
  double t4158;
  double t4186;
  double t4227;
  double t4356;
  double t4175;
  double t4375;
  double t4398;
  double t4494;
  double t4650;
  double t4664;
  double t4993;
  double t5002;
  double t5006;
  double t4845;
  double t4872;
  double t4909;
  double t4928;
  double t4953;
  double t4954;
  double t4983;
  double t5014;
  double t5015;
  double t5021;
  double t5069;
  double t5109;
  double t5017;
  double t5119;
  double t5120;
  double t5142;
  double t5143;
  double t5181;
  double t3392;
  double t3462;
  double t4403;
  double t4666;
  double t5139;
  double t5196;
  t600 = Cos(var1[5]);
  t1143 = Sin(var1[3]);
  t578 = Cos(var1[3]);
  t633 = Sin(var1[4]);
  t1259 = Sin(var1[5]);
  t2436 = Cos(var1[4]);
  t1504 = Cos(var1[6]);
  t1534 = -1.*t600*t1143;
  t1735 = t578*t633*t1259;
  t2123 = t1534 + t1735;
  t2445 = Sin(var1[6]);
  t406 = Cos(var1[8]);
  t2877 = t578*t2436*t1504;
  t2942 = t2123*t2445;
  t2972 = t2877 + t2942;
  t467 = Cos(var1[7]);
  t703 = t578*t600*t633;
  t1279 = t1143*t1259;
  t1300 = t703 + t1279;
  t1399 = t467*t1300;
  t2420 = t1504*t2123;
  t2474 = -1.*t578*t2436*t2445;
  t2526 = t2420 + t2474;
  t2638 = Sin(var1[7]);
  t2690 = -1.*t2526*t2638;
  t2815 = t1399 + t2690;
  t2985 = Sin(var1[8]);
  t3134 = Cos(var1[9]);
  t2843 = t406*t2815;
  t3052 = t2972*t2985;
  t3099 = t2843 + t3052;
  t112 = Sin(var1[9]);
  t3139 = t406*t2972;
  t3157 = -1.*t2815*t2985;
  t3179 = t3139 + t3157;
  t3411 = Cos(var1[10]);
  t3105 = -1.*t112*t3099;
  t3220 = t3134*t3179;
  t3300 = t3105 + t3220;
  t94 = Sin(var1[10]);
  t3441 = t3134*t3099;
  t3455 = t112*t3179;
  t3460 = t3441 + t3455;
  t3837 = t578*t600;
  t3878 = t1143*t633*t1259;
  t3900 = t3837 + t3878;
  t4003 = t2436*t1504*t1143;
  t4038 = t3900*t2445;
  t4130 = t4003 + t4038;
  t3701 = t600*t1143*t633;
  t3711 = -1.*t578*t1259;
  t3738 = t3701 + t3711;
  t3761 = t467*t3738;
  t3910 = t1504*t3900;
  t3911 = -1.*t2436*t1143*t2445;
  t3917 = t3910 + t3911;
  t3929 = -1.*t3917*t2638;
  t3943 = t3761 + t3929;
  t3961 = t406*t3943;
  t4146 = t4130*t2985;
  t4158 = t3961 + t4146;
  t4186 = t406*t4130;
  t4227 = -1.*t3943*t2985;
  t4356 = t4186 + t4227;
  t4175 = -1.*t112*t4158;
  t4375 = t3134*t4356;
  t4398 = t4175 + t4375;
  t4494 = t3134*t4158;
  t4650 = t112*t4356;
  t4664 = t4494 + t4650;
  t4993 = -1.*t1504*t633;
  t5002 = t2436*t1259*t2445;
  t5006 = t4993 + t5002;
  t4845 = t2436*t600*t467;
  t4872 = t2436*t1504*t1259;
  t4909 = t633*t2445;
  t4928 = t4872 + t4909;
  t4953 = -1.*t4928*t2638;
  t4954 = t4845 + t4953;
  t4983 = t406*t4954;
  t5014 = t5006*t2985;
  t5015 = t4983 + t5014;
  t5021 = t406*t5006;
  t5069 = -1.*t4954*t2985;
  t5109 = t5021 + t5069;
  t5017 = -1.*t112*t5015;
  t5119 = t3134*t5109;
  t5120 = t5017 + t5119;
  t5142 = t3134*t5015;
  t5143 = t112*t5109;
  t5181 = t5142 + t5143;
  t3392 = t94*t3300;
  t3462 = t3411*t3460;
  t4403 = t94*t4398;
  t4666 = t3411*t4664;
  t5139 = t94*t5120;
  t5196 = t3411*t5181;

  p_output1(0)=t3392 + t3462 + 0.000796*(t3300*t3411 - 1.*t3460*t94);
  p_output1(1)=t4403 + t4666 + 0.000796*(t3411*t4398 - 1.*t4664*t94);
  p_output1(2)=t5139 + t5196 + 0.000796*(t3411*t5120 - 1.*t5181*t94);
  p_output1(3)=t1300*t2638 + t2526*t467;
  p_output1(4)=t2638*t3738 + t3917*t467;
  p_output1(5)=t467*t4928 + t2436*t2638*t600;
  p_output1(6)=-1.*t3300*t3411 + 0.000796*(t3392 + t3462) + t3460*t94;
  p_output1(7)=-1.*t3411*t4398 + 0.000796*(t4403 + t4666) + t4664*t94;
  p_output1(8)=-1.*t3411*t5120 + 0.000796*(t5139 + t5196) + t5181*t94;
}


       
void R_LeftFootBack(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
