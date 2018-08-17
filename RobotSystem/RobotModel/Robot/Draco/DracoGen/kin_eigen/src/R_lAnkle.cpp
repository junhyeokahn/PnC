/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 18:04:51 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_lAnkle.h"

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
  double t1167;
  double t1534;
  double t1023;
  double t1399;
  double t1553;
  double t2353;
  double t1951;
  double t2088;
  double t2188;
  double t2195;
  double t2369;
  double t782;
  double t2738;
  double t2836;
  double t2939;
  double t1022;
  double t1430;
  double t1567;
  double t1606;
  double t1840;
  double t2329;
  double t2443;
  double t2458;
  double t2529;
  double t2548;
  double t2565;
  double t2985;
  double t3142;
  double t2722;
  double t3082;
  double t3118;
  double t769;
  double t3147;
  double t3172;
  double t3223;
  double t3477;
  double t3132;
  double t3241;
  double t3437;
  double t71;
  double t3494;
  double t3506;
  double t3546;
  double t3871;
  double t3890;
  double t3917;
  double t4159;
  double t4192;
  double t4205;
  double t3745;
  double t3746;
  double t3768;
  double t3799;
  double t3943;
  double t4001;
  double t4007;
  double t4057;
  double t4100;
  double t4114;
  double t4210;
  double t4219;
  double t4224;
  double t4283;
  double t4318;
  double t4221;
  double t4359;
  double t4361;
  double t4457;
  double t4521;
  double t4530;
  double t4892;
  double t4946;
  double t4947;
  double t4664;
  double t4682;
  double t4703;
  double t4708;
  double t4723;
  double t4803;
  double t4808;
  double t4949;
  double t4965;
  double t4990;
  double t5033;
  double t5037;
  double t4972;
  double t5043;
  double t5056;
  double t5074;
  double t5092;
  double t5131;
  double t3467;
  double t3614;
  double t4370;
  double t4558;
  double t5066;
  double t5135;
  t1167 = Cos(var1[5]);
  t1534 = Sin(var1[3]);
  t1023 = Cos(var1[3]);
  t1399 = Sin(var1[4]);
  t1553 = Sin(var1[5]);
  t2353 = Cos(var1[4]);
  t1951 = Cos(var1[6]);
  t2088 = -1.*t1167*t1534;
  t2188 = t1023*t1399*t1553;
  t2195 = t2088 + t2188;
  t2369 = Sin(var1[6]);
  t782 = Cos(var1[8]);
  t2738 = t1023*t2353*t1951;
  t2836 = t2195*t2369;
  t2939 = t2738 + t2836;
  t1022 = Cos(var1[7]);
  t1430 = t1023*t1167*t1399;
  t1567 = t1534*t1553;
  t1606 = t1430 + t1567;
  t1840 = t1022*t1606;
  t2329 = t1951*t2195;
  t2443 = -1.*t1023*t2353*t2369;
  t2458 = t2329 + t2443;
  t2529 = Sin(var1[7]);
  t2548 = -1.*t2458*t2529;
  t2565 = t1840 + t2548;
  t2985 = Sin(var1[8]);
  t3142 = Cos(var1[9]);
  t2722 = t782*t2565;
  t3082 = t2939*t2985;
  t3118 = t2722 + t3082;
  t769 = Sin(var1[9]);
  t3147 = t782*t2939;
  t3172 = -1.*t2565*t2985;
  t3223 = t3147 + t3172;
  t3477 = Cos(var1[10]);
  t3132 = -1.*t769*t3118;
  t3241 = t3142*t3223;
  t3437 = t3132 + t3241;
  t71 = Sin(var1[10]);
  t3494 = t3142*t3118;
  t3506 = t769*t3223;
  t3546 = t3494 + t3506;
  t3871 = t1023*t1167;
  t3890 = t1534*t1399*t1553;
  t3917 = t3871 + t3890;
  t4159 = t2353*t1951*t1534;
  t4192 = t3917*t2369;
  t4205 = t4159 + t4192;
  t3745 = t1167*t1534*t1399;
  t3746 = -1.*t1023*t1553;
  t3768 = t3745 + t3746;
  t3799 = t1022*t3768;
  t3943 = t1951*t3917;
  t4001 = -1.*t2353*t1534*t2369;
  t4007 = t3943 + t4001;
  t4057 = -1.*t4007*t2529;
  t4100 = t3799 + t4057;
  t4114 = t782*t4100;
  t4210 = t4205*t2985;
  t4219 = t4114 + t4210;
  t4224 = t782*t4205;
  t4283 = -1.*t4100*t2985;
  t4318 = t4224 + t4283;
  t4221 = -1.*t769*t4219;
  t4359 = t3142*t4318;
  t4361 = t4221 + t4359;
  t4457 = t3142*t4219;
  t4521 = t769*t4318;
  t4530 = t4457 + t4521;
  t4892 = -1.*t1951*t1399;
  t4946 = t2353*t1553*t2369;
  t4947 = t4892 + t4946;
  t4664 = t2353*t1167*t1022;
  t4682 = t2353*t1951*t1553;
  t4703 = t1399*t2369;
  t4708 = t4682 + t4703;
  t4723 = -1.*t4708*t2529;
  t4803 = t4664 + t4723;
  t4808 = t782*t4803;
  t4949 = t4947*t2985;
  t4965 = t4808 + t4949;
  t4990 = t782*t4947;
  t5033 = -1.*t4803*t2985;
  t5037 = t4990 + t5033;
  t4972 = -1.*t769*t4965;
  t5043 = t3142*t5037;
  t5056 = t4972 + t5043;
  t5074 = t3142*t4965;
  t5092 = t769*t5037;
  t5131 = t5074 + t5092;
  t3467 = t71*t3437;
  t3614 = t3477*t3546;
  t4370 = t71*t4361;
  t4558 = t3477*t4530;
  t5066 = t71*t5056;
  t5135 = t3477*t5131;

  p_output1(0)=t3467 + t3614 + 0.000796*(t3437*t3477 - 1.*t3546*t71);
  p_output1(1)=t4370 + t4558 + 0.000796*(t3477*t4361 - 1.*t4530*t71);
  p_output1(2)=t5066 + t5135 + 0.000796*(t3477*t5056 - 1.*t5131*t71);
  p_output1(3)=t1022*t2458 + t1606*t2529;
  p_output1(4)=t2529*t3768 + t1022*t4007;
  p_output1(5)=t1167*t2353*t2529 + t1022*t4708;
  p_output1(6)=-1.*t3437*t3477 + 0.000796*(t3467 + t3614) + t3546*t71;
  p_output1(7)=-1.*t3477*t4361 + 0.000796*(t4370 + t4558) + t4530*t71;
  p_output1(8)=-1.*t3477*t5056 + 0.000796*(t5066 + t5135) + t5131*t71;
}


       
void R_lAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
