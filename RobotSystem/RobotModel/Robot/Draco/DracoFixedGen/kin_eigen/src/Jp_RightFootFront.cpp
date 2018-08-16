/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:21 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RightFootFront.h"

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
  double t209;
  double t396;
  double t1036;
  double t459;
  double t1435;
  double t1478;
  double t1561;
  double t1659;
  double t2357;
  double t2364;
  double t2535;
  double t2725;
  double t3175;
  double t3176;
  double t3236;
  double t2198;
  double t2233;
  double t2355;
  double t3946;
  double t3979;
  double t4004;
  double t4028;
  double t4104;
  double t4113;
  double t4120;
  double t4156;
  double t4161;
  double t4181;
  double t486;
  double t618;
  double t855;
  double t1376;
  double t1427;
  double t1635;
  double t1677;
  double t1731;
  double t1904;
  double t1906;
  double t1975;
  double t2605;
  double t2970;
  double t3022;
  double t3278;
  double t3397;
  double t3447;
  double t4016;
  double t4095;
  double t4098;
  double t4256;
  double t4257;
  double t4262;
  double t4236;
  double t4249;
  double t4250;
  double t4131;
  double t4147;
  double t4153;
  double t4268;
  double t4275;
  double t4279;
  double t4288;
  double t4292;
  double t4296;
  double t4343;
  double t4344;
  double t4346;
  double t4349;
  double t4350;
  double t4353;
  double t4319;
  double t4322;
  double t4323;
  double t4381;
  double t4383;
  double t4384;
  double t4386;
  double t4387;
  double t4389;
  double t4412;
  double t4413;
  double t4414;
  double t4417;
  double t4422;
  double t4424;
  double t4449;
  double t4450;
  double t4451;
  double t4453;
  double t4455;
  double t4457;
  double t4458;
  double t4459;
  double t4441;
  double t4442;
  double t4443;
  double t4445;
  double t4446;
  double t4479;
  double t4481;
  double t4486;
  double t4491;
  double t4493;
  double t4494;
  double t4496;
  double t4498;
  double t4515;
  double t4526;
  double t4527;
  double t4528;
  double t4530;
  double t4531;
  double t4534;
  double t4580;
  double t4581;
  double t4310;
  double t4562;
  double t4563;
  double t4564;
  double t4571;
  double t4573;
  double t4602;
  double t4603;
  double t4605;
  double t4609;
  double t4610;
  double t4612;
  double t4613;
  double t4637;
  double t4639;
  double t4634;
  double t4635;
  double t4642;
  double t4643;
  double t4644;
  double t4647;
  double t4648;
  double t4649;
  double t4587;
  double t4311;
  double t4312;
  double t4664;
  double t4665;
  double t4667;
  double t4668;
  double t4669;
  double t4618;
  double t4679;
  double t4680;
  double t4681;
  double t4626;
  double t4654;
  double t4696;
  double t4699;
  double t4700;
  double t4659;
  t209 = Cos(var1[5]);
  t396 = Cos(var1[6]);
  t1036 = Sin(var1[6]);
  t459 = Sin(var1[5]);
  t1435 = Cos(var1[7]);
  t1478 = -1.*t1435;
  t1561 = 1. + t1478;
  t1659 = Sin(var1[7]);
  t2357 = Cos(var1[8]);
  t2364 = -1.*t2357;
  t2535 = 1. + t2364;
  t2725 = Sin(var1[8]);
  t3175 = -1.*t1435*t459;
  t3176 = -1.*t209*t1036*t1659;
  t3236 = t3175 + t3176;
  t2198 = t209*t1435*t1036;
  t2233 = -1.*t459*t1659;
  t2355 = t2198 + t2233;
  t3946 = Cos(var1[9]);
  t3979 = -1.*t3946;
  t4004 = 1. + t3979;
  t4028 = Sin(var1[9]);
  t4104 = t2357*t3236;
  t4113 = -1.*t2355*t2725;
  t4120 = t4104 + t4113;
  t4156 = t2357*t2355;
  t4161 = t3236*t2725;
  t4181 = t4156 + t4161;
  t486 = -1.*t396;
  t618 = 1. + t486;
  t855 = -0.330988*t618;
  t1376 = -0.90524*t1036;
  t1427 = 0. + t855 + t1376;
  t1635 = -0.97024*t1561;
  t1677 = -0.066675*t1659;
  t1731 = 0. + t1635 + t1677;
  t1904 = -0.066675*t1561;
  t1906 = 0.97024*t1659;
  t1975 = 0. + t1904 + t1906;
  t2605 = -1.45024*t2535;
  t2970 = -0.066675*t2725;
  t3022 = 0. + t2605 + t2970;
  t3278 = -0.066675*t2535;
  t3397 = 1.45024*t2725;
  t3447 = 0. + t3278 + t3397;
  t4016 = -0.065597*t4004;
  t4095 = 1.93024*t4028;
  t4098 = 0. + t4016 + t4095;
  t4256 = t209*t1435;
  t4257 = -1.*t459*t1036*t1659;
  t4262 = t4256 + t4257;
  t4236 = t1435*t459*t1036;
  t4249 = t209*t1659;
  t4250 = t4236 + t4249;
  t4131 = -1.93024*t4004;
  t4147 = -0.065597*t4028;
  t4153 = 0. + t4131 + t4147;
  t4268 = t2357*t4262;
  t4275 = -1.*t4250*t2725;
  t4279 = t4268 + t4275;
  t4288 = t2357*t4250;
  t4292 = t4262*t2725;
  t4296 = t4288 + t4292;
  t4343 = -1.*t396*t2357*t459*t1659;
  t4344 = -1.*t396*t1435*t459*t2725;
  t4346 = t4343 + t4344;
  t4349 = t396*t1435*t2357*t459;
  t4350 = -1.*t396*t459*t1659*t2725;
  t4353 = t4349 + t4350;
  t4319 = -0.90524*t396;
  t4322 = -0.330988*t1036;
  t4323 = t4319 + t4322;
  t4381 = t209*t396*t2357*t1659;
  t4383 = t209*t396*t1435*t2725;
  t4384 = t4381 + t4383;
  t4386 = -1.*t209*t396*t1435*t2357;
  t4387 = t209*t396*t1659*t2725;
  t4389 = t4386 + t4387;
  t4412 = t2357*t1036*t1659;
  t4413 = t1435*t1036*t2725;
  t4414 = t4412 + t4413;
  t4417 = -1.*t1435*t2357*t1036;
  t4422 = t1036*t1659*t2725;
  t4424 = t4417 + t4422;
  t4449 = -1.*t1435*t459*t1036;
  t4450 = -1.*t209*t1659;
  t4451 = t4449 + t4450;
  t4453 = t4451*t2725;
  t4455 = t4268 + t4453;
  t4457 = t2357*t4451;
  t4458 = -1.*t4262*t2725;
  t4459 = t4457 + t4458;
  t4441 = -0.066675*t1435;
  t4442 = -0.97024*t1659;
  t4443 = t4441 + t4442;
  t4445 = 0.97024*t1435;
  t4446 = t4445 + t1677;
  t4479 = t1435*t459;
  t4481 = t209*t1036*t1659;
  t4486 = t4479 + t4481;
  t4491 = t2357*t4486;
  t4493 = t2355*t2725;
  t4494 = t4491 + t4493;
  t4496 = -1.*t4486*t2725;
  t4498 = t4156 + t4496;
  t4515 = 0. + t396;
  t4526 = -1.*t4515*t2357*t1659;
  t4527 = -1.*t4515*t1435*t2725;
  t4528 = t4526 + t4527;
  t4530 = -1.*t4515*t1435*t2357;
  t4531 = t4515*t1659*t2725;
  t4534 = t4530 + t4531;
  t4580 = -1.*t2357*t4250;
  t4581 = t4580 + t4458;
  t4310 = t3946*t4279;
  t4562 = -0.066675*t2357;
  t4563 = -1.45024*t2725;
  t4564 = t4562 + t4563;
  t4571 = 1.45024*t2357;
  t4573 = t4571 + t2970;
  t4602 = -1.*t209*t1435*t1036;
  t4603 = t459*t1659;
  t4605 = t4602 + t4603;
  t4609 = -1.*t4605*t2725;
  t4610 = t4491 + t4609;
  t4612 = -1.*t2357*t4605;
  t4613 = t4612 + t4496;
  t4637 = -1.*t4515*t1659;
  t4639 = 0. + t4637;
  t4634 = t4515*t1435;
  t4635 = 0. + t4634;
  t4642 = t2357*t4639;
  t4643 = -1.*t4635*t2725;
  t4644 = t4642 + t4643;
  t4647 = -1.*t4635*t2357;
  t4648 = -1.*t4639*t2725;
  t4649 = t4647 + t4648;
  t4587 = -1.*t4028*t4279;
  t4311 = -1.*t4028*t4296;
  t4312 = t4310 + t4311;
  t4664 = 1.93024*t3946;
  t4665 = t4664 + t4147;
  t4667 = -0.065597*t3946;
  t4668 = -1.93024*t4028;
  t4669 = t4667 + t4668;
  t4618 = -1.*t4028*t4610;
  t4679 = t2357*t4605;
  t4680 = t4486*t2725;
  t4681 = t4679 + t4680;
  t4626 = t3946*t4610;
  t4654 = -1.*t4028*t4644;
  t4696 = t4635*t2357;
  t4699 = t4639*t2725;
  t4700 = t4696 + t4699;
  t4659 = t3946*t4644;

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
  p_output1(15)=-0.260988*t209 - 1.*t1427*t209 + t1036*t1731*t209 + t2355*t3022 + t3236*t3447 + 0.340988*t209*t396 + t4098*t4120 + t4153*t4181 - 1.840292*(t4028*t4120 + t3946*t4181) - 0.000525*(t3946*t4120 - 1.*t4028*t4181) - 0.066675*t459 - 1.*t1975*t459;
  p_output1(16)=0.066675*t209 + t1975*t209 + t3022*t4250 + t3447*t4262 + t4098*t4279 + t4153*t4296 - 1.840292*(t4028*t4279 + t3946*t4296) - 0.000525*t4312 - 0.260988*t459 - 1.*t1427*t459 + t1036*t1731*t459 + 0.340988*t396*t459;
  p_output1(17)=0;
  p_output1(18)=t4098*t4346 + t4153*t4353 - 1.840292*(t4028*t4346 + t3946*t4353) - 0.000525*(t3946*t4346 - 1.*t4028*t4353) - 0.340988*t1036*t459 + t1731*t396*t459 + t1435*t3022*t396*t459 - 1.*t1659*t3447*t396*t459 - 1.*t4323*t459;
  p_output1(19)=0.340988*t1036*t209 - 1.*t1731*t209*t396 - 1.*t1435*t209*t3022*t396 + t1659*t209*t3447*t396 + t209*t4323 + t4098*t4384 + t4153*t4389 - 1.840292*(t4028*t4384 + t3946*t4389) - 0.000525*(t3946*t4384 - 1.*t4028*t4389);
  p_output1(20)=t1376 - 1.*t1036*t1731 - 1.*t1036*t1435*t3022 + t1036*t1659*t3447 - 0.010000000000000009*t396 + t4098*t4414 + t4153*t4424 - 1.840292*(t4028*t4414 + t3946*t4424) - 0.000525*(t3946*t4414 - 1.*t4028*t4424);
  p_output1(21)=t3022*t4262 + t209*t4446 + t3447*t4451 + t4153*t4455 + t4098*t4459 - 0.000525*(-1.*t4028*t4455 + t3946*t4459) - 1.840292*(t3946*t4455 + t4028*t4459) + t1036*t4443*t459;
  p_output1(22)=t2355*t3447 - 1.*t1036*t209*t4443 + t3022*t4486 + t4153*t4494 + t4098*t4498 - 0.000525*(-1.*t4028*t4494 + t3946*t4498) - 1.840292*(t3946*t4494 + t4028*t4498) + t4446*t459;
  p_output1(23)=-1.*t1659*t3022*t4515 - 1.*t1435*t3447*t4515 + t4443*t4515 + t4153*t4528 + t4098*t4534 - 0.000525*(-1.*t4028*t4528 + t3946*t4534) - 1.840292*(t3946*t4528 + t4028*t4534);
  p_output1(24)=t4153*t4279 + t4250*t4564 + t4262*t4573 + t4098*t4581 - 1.840292*(t4310 + t4028*t4581) - 0.000525*(t3946*t4581 + t4587);
  p_output1(25)=t4486*t4573 + t4564*t4605 + t4153*t4610 + t4098*t4613 - 0.000525*(t3946*t4613 + t4618) - 1.840292*(t4028*t4613 + t4626);
  p_output1(26)=t4564*t4635 + t4573*t4639 + t4153*t4644 + t4098*t4649 - 0.000525*(t3946*t4649 + t4654) - 1.840292*(t4028*t4649 + t4659);
  p_output1(27)=-1.840292*t4312 - 0.000525*(-1.*t3946*t4296 + t4587) + t4279*t4665 + t4296*t4669;
  p_output1(28)=t4610*t4665 + t4669*t4681 - 0.000525*(t4618 - 1.*t3946*t4681) - 1.840292*(t4626 - 1.*t4028*t4681);
  p_output1(29)=t4644*t4665 + t4669*t4700 - 0.000525*(t4654 - 1.*t3946*t4700) - 1.840292*(t4659 - 1.*t4028*t4700);
}


       
void Jp_RightFootFront(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
