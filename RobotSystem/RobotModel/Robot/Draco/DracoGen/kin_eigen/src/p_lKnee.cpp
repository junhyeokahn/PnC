/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:19 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_lKnee.h"

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
  double t397;
  double t1547;
  double t2220;
  double t2241;
  double t2257;
  double t241;
  double t252;
  double t396;
  double t636;
  double t1001;
  double t1387;
  double t1482;
  double t2789;
  double t3980;
  double t3985;
  double t4066;
  double t4221;
  double t3103;
  double t3913;
  double t3974;
  double t5297;
  double t5380;
  double t5483;
  double t5668;
  double t5669;
  double t5677;
  double t5690;
  double t5641;
  double t5663;
  double t5666;
  double t5721;
  double t5765;
  double t5789;
  double t5837;
  double t5839;
  double t5843;
  double t5848;
  double t5899;
  double t5900;
  double t5902;
  double t5930;
  double t5935;
  double t5939;
  double t2249;
  double t2581;
  double t2625;
  double t2813;
  double t2835;
  double t3047;
  double t5978;
  double t5982;
  double t5983;
  double t4110;
  double t4844;
  double t5020;
  double t5522;
  double t5552;
  double t5580;
  double t6018;
  double t6023;
  double t6031;
  double t6037;
  double t6038;
  double t6045;
  double t5682;
  double t5693;
  double t5715;
  double t5803;
  double t5813;
  double t5820;
  double t5844;
  double t5886;
  double t5896;
  double t6090;
  double t6091;
  double t6093;
  double t6112;
  double t6114;
  double t6143;
  double t5916;
  double t5926;
  double t5927;
  double t6146;
  double t6150;
  double t6159;
  double t6164;
  double t6165;
  double t6170;
  double t6236;
  double t6250;
  double t6252;
  double t6322;
  double t6323;
  double t6325;
  double t6332;
  double t6333;
  double t6336;
  double t6340;
  double t6345;
  double t6349;
  double t6366;
  double t6368;
  double t6369;
  t397 = Cos(var1[3]);
  t1547 = Cos(var1[6]);
  t2220 = -1.*t1547;
  t2241 = 1. + t2220;
  t2257 = Sin(var1[6]);
  t241 = Cos(var1[5]);
  t252 = Sin(var1[3]);
  t396 = -1.*t241*t252;
  t636 = Sin(var1[4]);
  t1001 = Sin(var1[5]);
  t1387 = t397*t636*t1001;
  t1482 = t396 + t1387;
  t2789 = Cos(var1[4]);
  t3980 = Cos(var1[7]);
  t3985 = -1.*t3980;
  t4066 = 1. + t3985;
  t4221 = Sin(var1[7]);
  t3103 = t1547*t1482;
  t3913 = -1.*t397*t2789*t2257;
  t3974 = t3103 + t3913;
  t5297 = t397*t241*t636;
  t5380 = t252*t1001;
  t5483 = t5297 + t5380;
  t5668 = Cos(var1[8]);
  t5669 = -1.*t5668;
  t5677 = 1. + t5669;
  t5690 = Sin(var1[8]);
  t5641 = t3980*t5483;
  t5663 = -1.*t3974*t4221;
  t5666 = t5641 + t5663;
  t5721 = t397*t2789*t1547;
  t5765 = t1482*t2257;
  t5789 = t5721 + t5765;
  t5837 = Cos(var1[9]);
  t5839 = -1.*t5837;
  t5843 = 1. + t5839;
  t5848 = Sin(var1[9]);
  t5899 = t5668*t5666;
  t5900 = t5789*t5690;
  t5902 = t5899 + t5900;
  t5930 = t5668*t5789;
  t5935 = -1.*t5666*t5690;
  t5939 = t5930 + t5935;
  t2249 = 0.087*t2241;
  t2581 = 0.0222*t2257;
  t2625 = 0. + t2249 + t2581;
  t2813 = -0.0222*t2241;
  t2835 = 0.087*t2257;
  t3047 = 0. + t2813 + t2835;
  t5978 = t397*t241;
  t5982 = t252*t636*t1001;
  t5983 = t5978 + t5982;
  t4110 = 0.157*t4066;
  t4844 = -0.3151*t4221;
  t5020 = 0. + t4110 + t4844;
  t5522 = -0.3151*t4066;
  t5552 = -0.157*t4221;
  t5580 = 0. + t5522 + t5552;
  t6018 = t1547*t5983;
  t6023 = -1.*t2789*t252*t2257;
  t6031 = t6018 + t6023;
  t6037 = t241*t252*t636;
  t6038 = -1.*t397*t1001;
  t6045 = t6037 + t6038;
  t5682 = -0.3801*t5677;
  t5693 = -0.0222*t5690;
  t5715 = 0. + t5682 + t5693;
  t5803 = -0.0222*t5677;
  t5813 = 0.3801*t5690;
  t5820 = 0. + t5803 + t5813;
  t5844 = -0.8601*t5843;
  t5886 = -0.0222*t5848;
  t5896 = 0. + t5844 + t5886;
  t6090 = t3980*t6045;
  t6091 = -1.*t6031*t4221;
  t6093 = t6090 + t6091;
  t6112 = t2789*t1547*t252;
  t6114 = t5983*t2257;
  t6143 = t6112 + t6114;
  t5916 = -0.0222*t5843;
  t5926 = 0.8601*t5848;
  t5927 = 0. + t5916 + t5926;
  t6146 = t5668*t6093;
  t6150 = t6143*t5690;
  t6159 = t6146 + t6150;
  t6164 = t5668*t6143;
  t6165 = -1.*t6093*t5690;
  t6170 = t6164 + t6165;
  t6236 = t2789*t1547*t1001;
  t6250 = t636*t2257;
  t6252 = t6236 + t6250;
  t6322 = t2789*t241*t3980;
  t6323 = -1.*t6252*t4221;
  t6325 = t6322 + t6323;
  t6332 = -1.*t1547*t636;
  t6333 = t2789*t1001*t2257;
  t6336 = t6332 + t6333;
  t6340 = t5668*t6325;
  t6345 = t6336*t5690;
  t6349 = t6340 + t6345;
  t6366 = t5668*t6336;
  t6368 = -1.*t6325*t5690;
  t6369 = t6366 + t6368;

  p_output1(0)=0. + t1482*t2625 + t2789*t3047*t397 + t3974*t5020 + 0.1502*(t3974*t3980 + t4221*t5483) + t5483*t5580 + t5666*t5715 + t5789*t5820 + t5896*t5902 + t5927*t5939 - 0.0222*(-1.*t5848*t5902 + t5837*t5939) - 0.8601*(t5837*t5902 + t5848*t5939) + var1(0);
  p_output1(1)=0. + t252*t2789*t3047 + t2625*t5983 + t5020*t6031 + t5580*t6045 + 0.1502*(t3980*t6031 + t4221*t6045) + t5715*t6093 + t5820*t6143 + t5896*t6159 + t5927*t6170 - 0.0222*(-1.*t5848*t6159 + t5837*t6170) - 0.8601*(t5837*t6159 + t5848*t6170) + var1(1);
  p_output1(2)=0. + t1001*t2625*t2789 + t241*t2789*t5580 + t5020*t6252 + 0.1502*(t241*t2789*t4221 + t3980*t6252) + t5715*t6325 + t5820*t6336 + t5896*t6349 - 1.*t3047*t636 + t5927*t6369 - 0.0222*(-1.*t5848*t6349 + t5837*t6369) - 0.8601*(t5837*t6349 + t5848*t6369) + var1(2);
}


       
void p_lKnee(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
