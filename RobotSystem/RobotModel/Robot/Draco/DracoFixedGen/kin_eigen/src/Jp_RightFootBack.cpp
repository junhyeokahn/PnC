/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:24 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RightFootBack.h"

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
static void output1(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t480;
  double t577;
  double t1618;
  double t792;
  double t2017;
  double t2153;
  double t2197;
  double t3220;
  double t4451;
  double t4465;
  double t4466;
  double t4475;
  double t4501;
  double t4510;
  double t4512;
  double t4377;
  double t4437;
  double t4447;
  double t4531;
  double t4545;
  double t4548;
  double t4552;
  double t4560;
  double t4562;
  double t4564;
  double t4581;
  double t4596;
  double t4598;
  double t917;
  double t946;
  double t1225;
  double t1756;
  double t1768;
  double t2699;
  double t3446;
  double t3693;
  double t4199;
  double t4261;
  double t4317;
  double t4467;
  double t4491;
  double t4495;
  double t4514;
  double t4519;
  double t4523;
  double t4549;
  double t4555;
  double t4557;
  double t4673;
  double t4674;
  double t4675;
  double t4666;
  double t4668;
  double t4669;
  double t4571;
  double t4573;
  double t4577;
  double t4678;
  double t4684;
  double t4685;
  double t4690;
  double t4692;
  double t4694;
  double t4726;
  double t4728;
  double t4729;
  double t4731;
  double t4733;
  double t4734;
  double t4710;
  double t4711;
  double t4712;
  double t4759;
  double t4763;
  double t4766;
  double t4768;
  double t4770;
  double t4771;
  double t4810;
  double t4811;
  double t4813;
  double t4817;
  double t4819;
  double t4820;
  double t4855;
  double t4859;
  double t4861;
  double t4865;
  double t4867;
  double t4869;
  double t4870;
  double t4872;
  double t4843;
  double t4845;
  double t4846;
  double t4848;
  double t4849;
  double t4898;
  double t4902;
  double t4903;
  double t4908;
  double t4909;
  double t4910;
  double t4915;
  double t4916;
  double t4938;
  double t4950;
  double t4959;
  double t4961;
  double t4965;
  double t4966;
  double t4968;
  double t5001;
  double t5002;
  double t4704;
  double t4988;
  double t4989;
  double t4991;
  double t4993;
  double t4995;
  double t5018;
  double t5019;
  double t5020;
  double t5023;
  double t5024;
  double t5026;
  double t5027;
  double t5046;
  double t5048;
  double t5041;
  double t5042;
  double t5052;
  double t5053;
  double t5056;
  double t5058;
  double t5059;
  double t5061;
  double t5004;
  double t4705;
  double t4706;
  double t5080;
  double t5081;
  double t5083;
  double t5084;
  double t5085;
  double t5029;
  double t5094;
  double t5095;
  double t5096;
  double t5034;
  double t5066;
  double t5108;
  double t5109;
  double t5110;
  double t5071;
  t480 = Cos(var1[5]);
  t577 = Cos(var1[6]);
  t1618 = Sin(var1[6]);
  t792 = Sin(var1[5]);
  t2017 = Cos(var1[7]);
  t2153 = -1.*t2017;
  t2197 = 1. + t2153;
  t3220 = Sin(var1[7]);
  t4451 = Cos(var1[8]);
  t4465 = -1.*t4451;
  t4466 = 1. + t4465;
  t4475 = Sin(var1[8]);
  t4501 = -1.*t2017*t792;
  t4510 = -1.*t480*t1618*t3220;
  t4512 = t4501 + t4510;
  t4377 = t480*t2017*t1618;
  t4437 = -1.*t792*t3220;
  t4447 = t4377 + t4437;
  t4531 = Cos(var1[9]);
  t4545 = -1.*t4531;
  t4548 = 1. + t4545;
  t4552 = Sin(var1[9]);
  t4560 = t4451*t4512;
  t4562 = -1.*t4447*t4475;
  t4564 = t4560 + t4562;
  t4581 = t4451*t4447;
  t4596 = t4512*t4475;
  t4598 = t4581 + t4596;
  t917 = -1.*t577;
  t946 = 1. + t917;
  t1225 = -0.330988*t946;
  t1756 = -0.90524*t1618;
  t1768 = 0. + t1225 + t1756;
  t2699 = -0.97024*t2197;
  t3446 = -0.066675*t3220;
  t3693 = 0. + t2699 + t3446;
  t4199 = -0.066675*t2197;
  t4261 = 0.97024*t3220;
  t4317 = 0. + t4199 + t4261;
  t4467 = -1.45024*t4466;
  t4491 = -0.066675*t4475;
  t4495 = 0. + t4467 + t4491;
  t4514 = -0.066675*t4466;
  t4519 = 1.45024*t4475;
  t4523 = 0. + t4514 + t4519;
  t4549 = -0.065597*t4548;
  t4555 = 1.93024*t4552;
  t4557 = 0. + t4549 + t4555;
  t4673 = t480*t2017;
  t4674 = -1.*t792*t1618*t3220;
  t4675 = t4673 + t4674;
  t4666 = t2017*t792*t1618;
  t4668 = t480*t3220;
  t4669 = t4666 + t4668;
  t4571 = -1.93024*t4548;
  t4573 = -0.065597*t4552;
  t4577 = 0. + t4571 + t4573;
  t4678 = t4451*t4675;
  t4684 = -1.*t4669*t4475;
  t4685 = t4678 + t4684;
  t4690 = t4451*t4669;
  t4692 = t4675*t4475;
  t4694 = t4690 + t4692;
  t4726 = -1.*t577*t4451*t792*t3220;
  t4728 = -1.*t577*t2017*t792*t4475;
  t4729 = t4726 + t4728;
  t4731 = t577*t2017*t4451*t792;
  t4733 = -1.*t577*t792*t3220*t4475;
  t4734 = t4731 + t4733;
  t4710 = -0.90524*t577;
  t4711 = -0.330988*t1618;
  t4712 = t4710 + t4711;
  t4759 = t480*t577*t4451*t3220;
  t4763 = t480*t577*t2017*t4475;
  t4766 = t4759 + t4763;
  t4768 = -1.*t480*t577*t2017*t4451;
  t4770 = t480*t577*t3220*t4475;
  t4771 = t4768 + t4770;
  t4810 = t4451*t1618*t3220;
  t4811 = t2017*t1618*t4475;
  t4813 = t4810 + t4811;
  t4817 = -1.*t2017*t4451*t1618;
  t4819 = t1618*t3220*t4475;
  t4820 = t4817 + t4819;
  t4855 = -1.*t2017*t792*t1618;
  t4859 = -1.*t480*t3220;
  t4861 = t4855 + t4859;
  t4865 = t4861*t4475;
  t4867 = t4678 + t4865;
  t4869 = t4451*t4861;
  t4870 = -1.*t4675*t4475;
  t4872 = t4869 + t4870;
  t4843 = -0.066675*t2017;
  t4845 = -0.97024*t3220;
  t4846 = t4843 + t4845;
  t4848 = 0.97024*t2017;
  t4849 = t4848 + t3446;
  t4898 = t2017*t792;
  t4902 = t480*t1618*t3220;
  t4903 = t4898 + t4902;
  t4908 = t4451*t4903;
  t4909 = t4447*t4475;
  t4910 = t4908 + t4909;
  t4915 = -1.*t4903*t4475;
  t4916 = t4581 + t4915;
  t4938 = 0. + t577;
  t4950 = -1.*t4938*t4451*t3220;
  t4959 = -1.*t4938*t2017*t4475;
  t4961 = t4950 + t4959;
  t4965 = -1.*t4938*t2017*t4451;
  t4966 = t4938*t3220*t4475;
  t4968 = t4965 + t4966;
  t5001 = -1.*t4451*t4669;
  t5002 = t5001 + t4870;
  t4704 = t4531*t4685;
  t4988 = -0.066675*t4451;
  t4989 = -1.45024*t4475;
  t4991 = t4988 + t4989;
  t4993 = 1.45024*t4451;
  t4995 = t4993 + t4491;
  t5018 = -1.*t480*t2017*t1618;
  t5019 = t792*t3220;
  t5020 = t5018 + t5019;
  t5023 = -1.*t5020*t4475;
  t5024 = t4908 + t5023;
  t5026 = -1.*t4451*t5020;
  t5027 = t5026 + t4915;
  t5046 = -1.*t4938*t3220;
  t5048 = 0. + t5046;
  t5041 = t4938*t2017;
  t5042 = 0. + t5041;
  t5052 = t4451*t5048;
  t5053 = -1.*t5042*t4475;
  t5056 = t5052 + t5053;
  t5058 = -1.*t5042*t4451;
  t5059 = -1.*t5048*t4475;
  t5061 = t5058 + t5059;
  t5004 = -1.*t4552*t4685;
  t4705 = -1.*t4552*t4694;
  t4706 = t4704 + t4705;
  t5080 = 1.93024*t4531;
  t5081 = t5080 + t4573;
  t5083 = -0.065597*t4531;
  t5084 = -1.93024*t4552;
  t5085 = t5083 + t5084;
  t5029 = -1.*t4552*t5024;
  t5094 = t4451*t5020;
  t5095 = t4903*t4475;
  t5096 = t5094 + t5095;
  t5034 = t4531*t5024;
  t5066 = -1.*t4552*t5056;
  t5108 = t5042*t4451;
  t5109 = t5048*t4475;
  t5110 = t5108 + t5109;
  t5071 = t4531*t5056;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=t4447*t4495 + t4512*t4523 + t4557*t4564 + t4577*t4598 - 1.990292*(t4552*t4564 + t4531*t4598) - 0.000645*(t4531*t4564 - 1.*t4552*t4598) - 0.260988*t480 - 1.*t1768*t480 + t1618*t3693*t480 + 0.340988*t480*t577 - 0.066675*t792 - 1.*t4317*t792;
  p_output1(16)=t4495*t4669 + t4523*t4675 + t4557*t4685 + t4577*t4694 - 1.990292*(t4552*t4685 + t4531*t4694) - 0.000645*t4706 + 0.066675*t480 + t4317*t480 - 0.260988*t792 - 1.*t1768*t792 + t1618*t3693*t792 + 0.340988*t577*t792;
  p_output1(17)=0;
  p_output1(18)=t4557*t4729 + t4577*t4734 - 1.990292*(t4552*t4729 + t4531*t4734) - 0.000645*(t4531*t4729 - 1.*t4552*t4734) - 0.340988*t1618*t792 - 1.*t4712*t792 + t3693*t577*t792 + t2017*t4495*t577*t792 - 1.*t3220*t4523*t577*t792;
  p_output1(19)=t4557*t4766 + t4577*t4771 - 1.990292*(t4552*t4766 + t4531*t4771) - 0.000645*(t4531*t4766 - 1.*t4552*t4771) + 0.340988*t1618*t480 + t4712*t480 - 1.*t3693*t480*t577 - 1.*t2017*t4495*t480*t577 + t3220*t4523*t480*t577;
  p_output1(20)=t1756 - 1.*t1618*t3693 - 1.*t1618*t2017*t4495 + t1618*t3220*t4523 + t4557*t4813 + t4577*t4820 - 1.990292*(t4552*t4813 + t4531*t4820) - 0.000645*(t4531*t4813 - 1.*t4552*t4820) - 0.010000000000000009*t577;
  p_output1(21)=t4495*t4675 + t480*t4849 + t4523*t4861 + t4577*t4867 + t4557*t4872 - 0.000645*(-1.*t4552*t4867 + t4531*t4872) - 1.990292*(t4531*t4867 + t4552*t4872) + t1618*t4846*t792;
  p_output1(22)=t4447*t4523 - 1.*t1618*t480*t4846 + t4495*t4903 + t4577*t4910 + t4557*t4916 - 0.000645*(-1.*t4552*t4910 + t4531*t4916) - 1.990292*(t4531*t4910 + t4552*t4916) + t4849*t792;
  p_output1(23)=-1.*t3220*t4495*t4938 - 1.*t2017*t4523*t4938 + t4846*t4938 + t4577*t4961 + t4557*t4968 - 0.000645*(-1.*t4552*t4961 + t4531*t4968) - 1.990292*(t4531*t4961 + t4552*t4968);
  p_output1(24)=t4577*t4685 + t4669*t4991 + t4675*t4995 + t4557*t5002 - 1.990292*(t4704 + t4552*t5002) - 0.000645*(t4531*t5002 + t5004);
  p_output1(25)=t4903*t4995 + t4991*t5020 + t4577*t5024 + t4557*t5027 - 0.000645*(t4531*t5027 + t5029) - 1.990292*(t4552*t5027 + t5034);
  p_output1(26)=t4991*t5042 + t4995*t5048 + t4577*t5056 + t4557*t5061 - 0.000645*(t4531*t5061 + t5066) - 1.990292*(t4552*t5061 + t5071);
  p_output1(27)=-1.990292*t4706 - 0.000645*(-1.*t4531*t4694 + t5004) + t4685*t5081 + t4694*t5085;
  p_output1(28)=t5024*t5081 + t5085*t5096 - 0.000645*(t5029 - 1.*t4531*t5096) - 1.990292*(t5034 - 1.*t4552*t5096);
  p_output1(29)=t5056*t5081 + t5085*t5110 - 0.000645*(t5066 - 1.*t4531*t5110) - 1.990292*(t5071 - 1.*t4552*t5110);
}


       
void Jp_RightFootBack(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
