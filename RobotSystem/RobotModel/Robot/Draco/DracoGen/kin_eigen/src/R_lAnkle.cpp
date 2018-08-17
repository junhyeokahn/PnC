/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:42 GMT-05:00
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
  double t824;
  double t1071;
  double t749;
  double t876;
  double t1074;
  double t1552;
  double t1436;
  double t1443;
  double t1453;
  double t1454;
  double t1568;
  double t550;
  double t1887;
  double t1958;
  double t1961;
  double t625;
  double t1052;
  double t1100;
  double t1111;
  double t1203;
  double t1505;
  double t1678;
  double t1697;
  double t1701;
  double t1724;
  double t1779;
  double t1984;
  double t2290;
  double t1780;
  double t2016;
  double t2020;
  double t208;
  double t2360;
  double t2366;
  double t2538;
  double t2840;
  double t2164;
  double t2614;
  double t2700;
  double t189;
  double t3038;
  double t3042;
  double t3080;
  double t3663;
  double t3666;
  double t3674;
  double t3848;
  double t3855;
  double t3866;
  double t3492;
  double t3633;
  double t3634;
  double t3661;
  double t3751;
  double t3752;
  double t3775;
  double t3778;
  double t3815;
  double t3838;
  double t3872;
  double t3884;
  double t3887;
  double t3890;
  double t3891;
  double t3886;
  double t3892;
  double t3897;
  double t3900;
  double t3901;
  double t3902;
  double t4437;
  double t4441;
  double t4452;
  double t4061;
  double t4087;
  double t4089;
  double t4094;
  double t4183;
  double t4237;
  double t4244;
  double t4499;
  double t4520;
  double t4623;
  double t4639;
  double t4640;
  double t4598;
  double t4643;
  double t4704;
  double t4743;
  double t4798;
  double t4814;
  double t2729;
  double t3104;
  double t3898;
  double t3906;
  double t4734;
  double t4819;
  t824 = Cos(var1[5]);
  t1071 = Sin(var1[3]);
  t749 = Cos(var1[3]);
  t876 = Sin(var1[4]);
  t1074 = Sin(var1[5]);
  t1552 = Cos(var1[4]);
  t1436 = Cos(var1[6]);
  t1443 = -1.*t824*t1071;
  t1453 = t749*t876*t1074;
  t1454 = t1443 + t1453;
  t1568 = Sin(var1[6]);
  t550 = Cos(var1[8]);
  t1887 = t749*t1552*t1436;
  t1958 = t1454*t1568;
  t1961 = t1887 + t1958;
  t625 = Cos(var1[7]);
  t1052 = t749*t824*t876;
  t1100 = t1071*t1074;
  t1111 = t1052 + t1100;
  t1203 = t625*t1111;
  t1505 = t1436*t1454;
  t1678 = -1.*t749*t1552*t1568;
  t1697 = t1505 + t1678;
  t1701 = Sin(var1[7]);
  t1724 = -1.*t1697*t1701;
  t1779 = t1203 + t1724;
  t1984 = Sin(var1[8]);
  t2290 = Cos(var1[9]);
  t1780 = t550*t1779;
  t2016 = t1961*t1984;
  t2020 = t1780 + t2016;
  t208 = Sin(var1[9]);
  t2360 = t550*t1961;
  t2366 = -1.*t1779*t1984;
  t2538 = t2360 + t2366;
  t2840 = Cos(var1[10]);
  t2164 = -1.*t208*t2020;
  t2614 = t2290*t2538;
  t2700 = t2164 + t2614;
  t189 = Sin(var1[10]);
  t3038 = t2290*t2020;
  t3042 = t208*t2538;
  t3080 = t3038 + t3042;
  t3663 = t749*t824;
  t3666 = t1071*t876*t1074;
  t3674 = t3663 + t3666;
  t3848 = t1552*t1436*t1071;
  t3855 = t3674*t1568;
  t3866 = t3848 + t3855;
  t3492 = t824*t1071*t876;
  t3633 = -1.*t749*t1074;
  t3634 = t3492 + t3633;
  t3661 = t625*t3634;
  t3751 = t1436*t3674;
  t3752 = -1.*t1552*t1071*t1568;
  t3775 = t3751 + t3752;
  t3778 = -1.*t3775*t1701;
  t3815 = t3661 + t3778;
  t3838 = t550*t3815;
  t3872 = t3866*t1984;
  t3884 = t3838 + t3872;
  t3887 = t550*t3866;
  t3890 = -1.*t3815*t1984;
  t3891 = t3887 + t3890;
  t3886 = -1.*t208*t3884;
  t3892 = t2290*t3891;
  t3897 = t3886 + t3892;
  t3900 = t2290*t3884;
  t3901 = t208*t3891;
  t3902 = t3900 + t3901;
  t4437 = -1.*t1436*t876;
  t4441 = t1552*t1074*t1568;
  t4452 = t4437 + t4441;
  t4061 = t1552*t824*t625;
  t4087 = t1552*t1436*t1074;
  t4089 = t876*t1568;
  t4094 = t4087 + t4089;
  t4183 = -1.*t4094*t1701;
  t4237 = t4061 + t4183;
  t4244 = t550*t4237;
  t4499 = t4452*t1984;
  t4520 = t4244 + t4499;
  t4623 = t550*t4452;
  t4639 = -1.*t4237*t1984;
  t4640 = t4623 + t4639;
  t4598 = -1.*t208*t4520;
  t4643 = t2290*t4640;
  t4704 = t4598 + t4643;
  t4743 = t2290*t4520;
  t4798 = t208*t4640;
  t4814 = t4743 + t4798;
  t2729 = t189*t2700;
  t3104 = t2840*t3080;
  t3898 = t189*t3897;
  t3906 = t2840*t3902;
  t4734 = t189*t4704;
  t4819 = t2840*t4814;

  p_output1(0)=t2729 + 0.000796*(t2700*t2840 - 1.*t189*t3080) + t3104;
  p_output1(1)=t3898 + 0.000796*(t2840*t3897 - 1.*t189*t3902) + t3906;
  p_output1(2)=t4734 + 0.000796*(t2840*t4704 - 1.*t189*t4814) + t4819;
  p_output1(3)=t1111*t1701 + t1697*t625;
  p_output1(4)=t1701*t3634 + t3775*t625;
  p_output1(5)=t4094*t625 + t1552*t1701*t824;
  p_output1(6)=-1.*t2700*t2840 + t189*t3080 + 0.000796*(t2729 + t3104);
  p_output1(7)=-1.*t2840*t3897 + t189*t3902 + 0.000796*(t3898 + t3906);
  p_output1(8)=-1.*t2840*t4704 + t189*t4814 + 0.000796*(t4734 + t4819);
}


       
void R_lAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
