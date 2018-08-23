/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:20 GMT-05:00
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
  double t879;
  double t1683;
  double t1694;
  double t1961;
  double t1978;
  double t370;
  double t373;
  double t580;
  double t1106;
  double t1107;
  double t1516;
  double t1567;
  double t2678;
  double t3483;
  double t3505;
  double t3507;
  double t3691;
  double t3085;
  double t3461;
  double t3481;
  double t3907;
  double t3908;
  double t3930;
  double t4032;
  double t4034;
  double t4123;
  double t4158;
  double t3989;
  double t4001;
  double t4018;
  double t4255;
  double t4283;
  double t4291;
  double t4628;
  double t4663;
  double t4789;
  double t4837;
  double t4951;
  double t4952;
  double t4961;
  double t5216;
  double t5223;
  double t5239;
  double t5308;
  double t5364;
  double t5380;
  double t5440;
  double t5517;
  double t5523;
  double t5555;
  double t5613;
  double t5621;
  double t5622;
  double t1976;
  double t2031;
  double t2236;
  double t2766;
  double t2845;
  double t2853;
  double t5797;
  double t5803;
  double t5834;
  double t3519;
  double t3716;
  double t3901;
  double t3934;
  double t3936;
  double t3937;
  double t5905;
  double t5911;
  double t5927;
  double t5941;
  double t5948;
  double t5957;
  double t4141;
  double t4206;
  double t4218;
  double t4295;
  double t4297;
  double t4553;
  double t4814;
  double t4901;
  double t4902;
  double t5971;
  double t5982;
  double t5983;
  double t6022;
  double t6036;
  double t6037;
  double t5071;
  double t5090;
  double t5212;
  double t5434;
  double t5453;
  double t5500;
  double t6047;
  double t6059;
  double t6071;
  double t6085;
  double t6090;
  double t6096;
  double t5586;
  double t5610;
  double t5612;
  double t6139;
  double t6145;
  double t6152;
  double t6155;
  double t6179;
  double t6184;
  double t6317;
  double t6321;
  double t6322;
  double t6373;
  double t6382;
  double t6385;
  double t6401;
  double t6403;
  double t6406;
  double t6437;
  double t6440;
  double t6443;
  double t6452;
  double t6453;
  double t6454;
  double t6458;
  double t6459;
  double t6463;
  double t6465;
  double t6468;
  double t6472;
  t879 = Cos(var1[3]);
  t1683 = Cos(var1[6]);
  t1694 = -1.*t1683;
  t1961 = 1. + t1694;
  t1978 = Sin(var1[6]);
  t370 = Cos(var1[5]);
  t373 = Sin(var1[3]);
  t580 = -1.*t370*t373;
  t1106 = Sin(var1[4]);
  t1107 = Sin(var1[5]);
  t1516 = t879*t1106*t1107;
  t1567 = t580 + t1516;
  t2678 = Cos(var1[4]);
  t3483 = Cos(var1[7]);
  t3505 = -1.*t3483;
  t3507 = 1. + t3505;
  t3691 = Sin(var1[7]);
  t3085 = t1683*t1567;
  t3461 = -1.*t879*t2678*t1978;
  t3481 = t3085 + t3461;
  t3907 = t879*t370*t1106;
  t3908 = t373*t1107;
  t3930 = t3907 + t3908;
  t4032 = Cos(var1[8]);
  t4034 = -1.*t4032;
  t4123 = 1. + t4034;
  t4158 = Sin(var1[8]);
  t3989 = t3483*t3930;
  t4001 = -1.*t3481*t3691;
  t4018 = t3989 + t4001;
  t4255 = t879*t2678*t1683;
  t4283 = t1567*t1978;
  t4291 = t4255 + t4283;
  t4628 = Cos(var1[9]);
  t4663 = -1.*t4628;
  t4789 = 1. + t4663;
  t4837 = Sin(var1[9]);
  t4951 = t4032*t4018;
  t4952 = t4291*t4158;
  t4961 = t4951 + t4952;
  t5216 = t4032*t4291;
  t5223 = -1.*t4018*t4158;
  t5239 = t5216 + t5223;
  t5308 = Cos(var1[10]);
  t5364 = -1.*t5308;
  t5380 = 1. + t5364;
  t5440 = Sin(var1[10]);
  t5517 = -1.*t4837*t4961;
  t5523 = t4628*t5239;
  t5555 = t5517 + t5523;
  t5613 = t4628*t4961;
  t5621 = t4837*t5239;
  t5622 = t5613 + t5621;
  t1976 = 0.087*t1961;
  t2031 = 0.0222*t1978;
  t2236 = 0. + t1976 + t2031;
  t2766 = -0.0222*t1961;
  t2845 = 0.087*t1978;
  t2853 = 0. + t2766 + t2845;
  t5797 = t879*t370;
  t5803 = t373*t1106*t1107;
  t5834 = t5797 + t5803;
  t3519 = 0.157*t3507;
  t3716 = -0.3151*t3691;
  t3901 = 0. + t3519 + t3716;
  t3934 = -0.3151*t3507;
  t3936 = -0.157*t3691;
  t3937 = 0. + t3934 + t3936;
  t5905 = t1683*t5834;
  t5911 = -1.*t2678*t373*t1978;
  t5927 = t5905 + t5911;
  t5941 = t370*t373*t1106;
  t5948 = -1.*t879*t1107;
  t5957 = t5941 + t5948;
  t4141 = -0.3801*t4123;
  t4206 = -0.0222*t4158;
  t4218 = 0. + t4141 + t4206;
  t4295 = -0.0222*t4123;
  t4297 = 0.3801*t4158;
  t4553 = 0. + t4295 + t4297;
  t4814 = -0.8601*t4789;
  t4901 = -0.0222*t4837;
  t4902 = 0. + t4814 + t4901;
  t5971 = t3483*t5957;
  t5982 = -1.*t5927*t3691;
  t5983 = t5971 + t5982;
  t6022 = t2678*t1683*t373;
  t6036 = t5834*t1978;
  t6037 = t6022 + t6036;
  t5071 = -0.0222*t4789;
  t5090 = 0.8601*t4837;
  t5212 = 0. + t5071 + t5090;
  t5434 = -0.0211*t5380;
  t5453 = 1.3401*t5440;
  t5500 = 0. + t5434 + t5453;
  t6047 = t4032*t5983;
  t6059 = t6037*t4158;
  t6071 = t6047 + t6059;
  t6085 = t4032*t6037;
  t6090 = -1.*t5983*t4158;
  t6096 = t6085 + t6090;
  t5586 = -1.3401*t5380;
  t5610 = -0.0211*t5440;
  t5612 = 0. + t5586 + t5610;
  t6139 = -1.*t4837*t6071;
  t6145 = t4628*t6096;
  t6152 = t6139 + t6145;
  t6155 = t4628*t6071;
  t6179 = t4837*t6096;
  t6184 = t6155 + t6179;
  t6317 = t2678*t1683*t1107;
  t6321 = t1106*t1978;
  t6322 = t6317 + t6321;
  t6373 = t2678*t370*t3483;
  t6382 = -1.*t6322*t3691;
  t6385 = t6373 + t6382;
  t6401 = -1.*t1683*t1106;
  t6403 = t2678*t1107*t1978;
  t6406 = t6401 + t6403;
  t6437 = t4032*t6385;
  t6440 = t6406*t4158;
  t6443 = t6437 + t6440;
  t6452 = t4032*t6406;
  t6453 = -1.*t6385*t4158;
  t6454 = t6452 + t6453;
  t6458 = -1.*t4837*t6443;
  t6459 = t4628*t6454;
  t6463 = t6458 + t6459;
  t6465 = t4628*t6443;
  t6468 = t4837*t6454;
  t6472 = t6465 + t6468;

  p_output1(0)=0. + t1567*t2236 + t3481*t3901 + 0.167*(t3481*t3483 + t3691*t3930) + t3930*t3937 + t4018*t4218 + t4291*t4553 + t4902*t4961 + t5212*t5239 + t5500*t5555 + t5612*t5622 - 1.3401*(t5440*t5555 + t5308*t5622) - 0.0211*(t5308*t5555 - 1.*t5440*t5622) + t2678*t2853*t879 + var1(0);
  p_output1(1)=0. + t2678*t2853*t373 + t2236*t5834 + t3901*t5927 + t3937*t5957 + 0.167*(t3483*t5927 + t3691*t5957) + t4218*t5983 + t4553*t6037 + t4902*t6071 + t5212*t6096 + t5500*t6152 + t5612*t6184 - 1.3401*(t5440*t6152 + t5308*t6184) - 0.0211*(t5308*t6152 - 1.*t5440*t6184) + var1(1);
  p_output1(2)=0. + t1107*t2236*t2678 - 1.*t1106*t2853 + t2678*t370*t3937 + t3901*t6322 + 0.167*(t2678*t3691*t370 + t3483*t6322) + t4218*t6385 + t4553*t6406 + t4902*t6443 + t5212*t6454 + t5500*t6463 + t5612*t6472 - 1.3401*(t5440*t6463 + t5308*t6472) - 0.0211*(t5308*t6463 - 1.*t5440*t6472) + var1(2);
}


       
void p_lAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
