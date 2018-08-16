/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:19 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_LeftFootBack.h"

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
  double t106;
  double t267;
  double t1381;
  double t414;
  double t1808;
  double t2226;
  double t2227;
  double t2275;
  double t3519;
  double t3532;
  double t3601;
  double t3616;
  double t3484;
  double t3490;
  double t3496;
  double t3682;
  double t3683;
  double t3687;
  double t3743;
  double t3745;
  double t3746;
  double t3752;
  double t3729;
  double t3738;
  double t3739;
  double t3769;
  double t3770;
  double t3772;
  double t809;
  double t1089;
  double t1257;
  double t1392;
  double t1420;
  double t2264;
  double t2378;
  double t2896;
  double t3267;
  double t3424;
  double t3464;
  double t3614;
  double t3624;
  double t3645;
  double t3706;
  double t3710;
  double t3718;
  double t3861;
  double t3866;
  double t3877;
  double t3880;
  double t3882;
  double t3884;
  double t3748;
  double t3753;
  double t3755;
  double t3773;
  double t3775;
  double t3776;
  double t3892;
  double t3893;
  double t3894;
  double t3900;
  double t3907;
  double t3912;
  double t3948;
  double t3950;
  double t3951;
  double t3955;
  double t3956;
  double t3957;
  double t3929;
  double t3931;
  double t3932;
  double t3985;
  double t3986;
  double t3987;
  double t3990;
  double t3991;
  double t3992;
  double t4024;
  double t4025;
  double t4027;
  double t4029;
  double t4030;
  double t4033;
  double t4068;
  double t4069;
  double t4070;
  double t4083;
  double t4085;
  double t4087;
  double t4073;
  double t4074;
  double t4053;
  double t4055;
  double t4059;
  double t4062;
  double t4063;
  double t4111;
  double t4113;
  double t4114;
  double t4124;
  double t4125;
  double t4119;
  double t4120;
  double t4121;
  double t4150;
  double t4163;
  double t4169;
  double t4172;
  double t4157;
  double t4159;
  double t4160;
  double t4212;
  double t4216;
  double t3919;
  double t4196;
  double t4197;
  double t4200;
  double t4202;
  double t4203;
  double t4237;
  double t4238;
  double t4239;
  double t4248;
  double t4249;
  double t4243;
  double t4245;
  double t4269;
  double t4270;
  double t4264;
  double t4266;
  double t4277;
  double t4278;
  double t4279;
  double t4272;
  double t4273;
  double t4274;
  double t4221;
  double t3920;
  double t3923;
  double t4297;
  double t4299;
  double t4300;
  double t4302;
  double t4304;
  double t4312;
  double t4313;
  double t4315;
  double t4252;
  double t4255;
  double t4333;
  double t4334;
  double t4335;
  double t4284;
  double t4289;
  t106 = Cos(var1[0]);
  t267 = Cos(var1[1]);
  t1381 = Sin(var1[1]);
  t414 = Sin(var1[0]);
  t1808 = Cos(var1[2]);
  t2226 = -1.*t1808;
  t2227 = 1. + t2226;
  t2275 = Sin(var1[2]);
  t3519 = Cos(var1[3]);
  t3532 = -1.*t3519;
  t3601 = 1. + t3532;
  t3616 = Sin(var1[3]);
  t3484 = t106*t1808*t1381;
  t3490 = -1.*t414*t2275;
  t3496 = t3484 + t3490;
  t3682 = -1.*t1808*t414;
  t3683 = -1.*t106*t1381*t2275;
  t3687 = t3682 + t3683;
  t3743 = Cos(var1[4]);
  t3745 = -1.*t3743;
  t3746 = 1. + t3745;
  t3752 = Sin(var1[4]);
  t3729 = t3519*t3496;
  t3738 = t3687*t3616;
  t3739 = t3729 + t3738;
  t3769 = t3519*t3687;
  t3770 = -1.*t3496*t3616;
  t3772 = t3769 + t3770;
  t809 = -1.*t267;
  t1089 = 1. + t809;
  t1257 = 0.331012*t1089;
  t1392 = -0.90524*t1381;
  t1420 = 0. + t1257 + t1392;
  t2264 = -0.97024*t2227;
  t2378 = -0.066675*t2275;
  t2896 = 0. + t2264 + t2378;
  t3267 = -0.066675*t2227;
  t3424 = 0.97024*t2275;
  t3464 = 0. + t3267 + t3424;
  t3614 = -1.45024*t3601;
  t3624 = -0.066675*t3616;
  t3645 = 0. + t3614 + t3624;
  t3706 = -0.066675*t3601;
  t3710 = 1.45024*t3616;
  t3718 = 0. + t3706 + t3710;
  t3861 = t1808*t414*t1381;
  t3866 = t106*t2275;
  t3877 = t3861 + t3866;
  t3880 = t106*t1808;
  t3882 = -1.*t414*t1381*t2275;
  t3884 = t3880 + t3882;
  t3748 = -1.93024*t3746;
  t3753 = -0.065597*t3752;
  t3755 = 0. + t3748 + t3753;
  t3773 = -0.065597*t3746;
  t3775 = 1.93024*t3752;
  t3776 = 0. + t3773 + t3775;
  t3892 = t3519*t3877;
  t3893 = t3884*t3616;
  t3894 = t3892 + t3893;
  t3900 = t3519*t3884;
  t3907 = -1.*t3877*t3616;
  t3912 = t3900 + t3907;
  t3948 = t267*t1808*t3519*t414;
  t3950 = -1.*t267*t414*t2275*t3616;
  t3951 = t3948 + t3950;
  t3955 = -1.*t267*t3519*t414*t2275;
  t3956 = -1.*t267*t1808*t414*t3616;
  t3957 = t3955 + t3956;
  t3929 = -0.90524*t267;
  t3931 = 0.331012*t1381;
  t3932 = t3929 + t3931;
  t3985 = -1.*t106*t267*t1808*t3519;
  t3986 = t106*t267*t2275*t3616;
  t3987 = t3985 + t3986;
  t3990 = t106*t267*t3519*t2275;
  t3991 = t106*t267*t1808*t3616;
  t3992 = t3990 + t3991;
  t4024 = -1.*t1808*t3519*t1381;
  t4025 = t1381*t2275*t3616;
  t4027 = t4024 + t4025;
  t4029 = t3519*t1381*t2275;
  t4030 = t1808*t1381*t3616;
  t4033 = t4029 + t4030;
  t4068 = -1.*t1808*t414*t1381;
  t4069 = -1.*t106*t2275;
  t4070 = t4068 + t4069;
  t4083 = t3519*t4070;
  t4085 = -1.*t3884*t3616;
  t4087 = t4083 + t4085;
  t4073 = t4070*t3616;
  t4074 = t3900 + t4073;
  t4053 = -0.066675*t1808;
  t4055 = -0.97024*t2275;
  t4059 = t4053 + t4055;
  t4062 = 0.97024*t1808;
  t4063 = t4062 + t2378;
  t4111 = t1808*t414;
  t4113 = t106*t1381*t2275;
  t4114 = t4111 + t4113;
  t4124 = -1.*t4114*t3616;
  t4125 = t3729 + t4124;
  t4119 = t3519*t4114;
  t4120 = t3496*t3616;
  t4121 = t4119 + t4120;
  t4150 = 0. + t267;
  t4163 = -1.*t4150*t1808*t3519;
  t4169 = t4150*t2275*t3616;
  t4172 = t4163 + t4169;
  t4157 = -1.*t4150*t3519*t2275;
  t4159 = -1.*t4150*t1808*t3616;
  t4160 = t4157 + t4159;
  t4212 = -1.*t3519*t3877;
  t4216 = t4212 + t4085;
  t3919 = t3743*t3912;
  t4196 = -0.066675*t3519;
  t4197 = -1.45024*t3616;
  t4200 = t4196 + t4197;
  t4202 = 1.45024*t3519;
  t4203 = t4202 + t3624;
  t4237 = -1.*t106*t1808*t1381;
  t4238 = t414*t2275;
  t4239 = t4237 + t4238;
  t4248 = -1.*t3519*t4239;
  t4249 = t4248 + t4124;
  t4243 = -1.*t4239*t3616;
  t4245 = t4119 + t4243;
  t4269 = -1.*t4150*t2275;
  t4270 = 0. + t4269;
  t4264 = t4150*t1808;
  t4266 = 0. + t4264;
  t4277 = -1.*t4266*t3519;
  t4278 = -1.*t4270*t3616;
  t4279 = t4277 + t4278;
  t4272 = t3519*t4270;
  t4273 = -1.*t4266*t3616;
  t4274 = t4272 + t4273;
  t4221 = -1.*t3912*t3752;
  t3920 = -1.*t3894*t3752;
  t3923 = t3919 + t3920;
  t4297 = -0.065597*t3743;
  t4299 = -1.93024*t3752;
  t4300 = t4297 + t4299;
  t4302 = 1.93024*t3743;
  t4304 = t4302 + t3753;
  t4312 = t3519*t4239;
  t4313 = t4114*t3616;
  t4315 = t4312 + t4313;
  t4252 = -1.*t4245*t3752;
  t4255 = t3743*t4245;
  t4333 = t4266*t3519;
  t4334 = t4270*t3616;
  t4335 = t4333 + t4334;
  t4284 = -1.*t4274*t3752;
  t4289 = t3743*t4274;

  p_output1(0)=0.261012*t106 - 1.*t106*t1420 - 0.341012*t106*t267 + t106*t1381*t2896 + t3496*t3645 + t3687*t3718 + t3739*t3755 - 0.000645*(-1.*t3739*t3752 + t3743*t3772) - 1.990292*(t3739*t3743 + t3752*t3772) + t3772*t3776 - 0.066675*t414 - 1.*t3464*t414;
  p_output1(1)=0.066675*t106 + t106*t3464 + t3645*t3877 + t3718*t3884 + t3755*t3894 + t3776*t3912 - 1.990292*(t3743*t3894 + t3752*t3912) - 0.000645*t3923 + 0.261012*t414 - 1.*t1420*t414 - 0.341012*t267*t414 + t1381*t2896*t414;
  p_output1(2)=0;
  p_output1(3)=t3755*t3951 + t3776*t3957 - 0.000645*(-1.*t3752*t3951 + t3743*t3957) - 1.990292*(t3743*t3951 + t3752*t3957) + 0.341012*t1381*t414 + t267*t2896*t414 + t1808*t267*t3645*t414 - 1.*t2275*t267*t3718*t414 - 1.*t3932*t414;
  p_output1(4)=-0.341012*t106*t1381 - 1.*t106*t267*t2896 - 1.*t106*t1808*t267*t3645 + t106*t2275*t267*t3718 + t106*t3932 + t3755*t3987 + t3776*t3992 - 0.000645*(-1.*t3752*t3987 + t3743*t3992) - 1.990292*(t3743*t3987 + t3752*t3992);
  p_output1(5)=t1392 + 0.010000000000000009*t267 - 1.*t1381*t2896 - 1.*t1381*t1808*t3645 + t1381*t2275*t3718 + t3755*t4027 + t3776*t4033 - 0.000645*(-1.*t3752*t4027 + t3743*t4033) - 1.990292*(t3743*t4027 + t3752*t4033);
  p_output1(6)=t3645*t3884 + t106*t4063 + t3718*t4070 + t3755*t4074 + t3776*t4087 - 0.000645*(-1.*t3752*t4074 + t3743*t4087) - 1.990292*(t3743*t4074 + t3752*t4087) + t1381*t4059*t414;
  p_output1(7)=t3496*t3718 - 1.*t106*t1381*t4059 + t3645*t4114 + t3755*t4121 + t3776*t4125 - 0.000645*(-1.*t3752*t4121 + t3743*t4125) - 1.990292*(t3743*t4121 + t3752*t4125) + t4063*t414;
  p_output1(8)=-1.*t2275*t3645*t4150 - 1.*t1808*t3718*t4150 + t4059*t4150 + t3755*t4160 + t3776*t4172 - 0.000645*(-1.*t3752*t4160 + t3743*t4172) - 1.990292*(t3743*t4160 + t3752*t4172);
  p_output1(9)=t3755*t3912 + t3877*t4200 + t3884*t4203 + t3776*t4216 - 1.990292*(t3919 + t3752*t4216) - 0.000645*(t3743*t4216 + t4221);
  p_output1(10)=t4114*t4203 + t4200*t4239 + t3755*t4245 + t3776*t4249 - 0.000645*(t3743*t4249 + t4252) - 1.990292*(t3752*t4249 + t4255);
  p_output1(11)=t4200*t4266 + t4203*t4270 + t3755*t4274 + t3776*t4279 - 0.000645*(t3743*t4279 + t4284) - 1.990292*(t3752*t4279 + t4289);
  p_output1(12)=-1.990292*t3923 - 0.000645*(-1.*t3743*t3894 + t4221) + t3894*t4300 + t3912*t4304;
  p_output1(13)=t4245*t4304 + t4300*t4315 - 0.000645*(t4252 - 1.*t3743*t4315) - 1.990292*(t4255 - 1.*t3752*t4315);
  p_output1(14)=t4274*t4304 + t4300*t4335 - 0.000645*(t4284 - 1.*t3743*t4335) - 1.990292*(t4289 - 1.*t3752*t4335);
  p_output1(15)=0;
  p_output1(16)=0;
  p_output1(17)=0;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
}


       
void Jp_LeftFootBack(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
