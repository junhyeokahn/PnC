/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:16 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_LeftFootFront.h"

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
  double t22;
  double t174;
  double t1994;
  double t677;
  double t2587;
  double t2647;
  double t2869;
  double t2917;
  double t3118;
  double t3123;
  double t3132;
  double t3143;
  double t3087;
  double t3101;
  double t3107;
  double t3183;
  double t3186;
  double t3187;
  double t3213;
  double t3219;
  double t3232;
  double t3238;
  double t3200;
  double t3202;
  double t3204;
  double t3249;
  double t3255;
  double t3257;
  double t1461;
  double t1493;
  double t1968;
  double t2030;
  double t2076;
  double t2876;
  double t2937;
  double t2990;
  double t3035;
  double t3038;
  double t3055;
  double t3142;
  double t3148;
  double t3161;
  double t3195;
  double t3197;
  double t3198;
  double t3338;
  double t3339;
  double t3344;
  double t3348;
  double t3349;
  double t3352;
  double t3234;
  double t3240;
  double t3247;
  double t3259;
  double t3263;
  double t3264;
  double t3360;
  double t3375;
  double t3380;
  double t3383;
  double t3385;
  double t3386;
  double t3435;
  double t3436;
  double t3437;
  double t3439;
  double t3440;
  double t3441;
  double t3419;
  double t3420;
  double t3421;
  double t3485;
  double t3488;
  double t3489;
  double t3495;
  double t3498;
  double t3499;
  double t3540;
  double t3543;
  double t3544;
  double t3548;
  double t3550;
  double t3552;
  double t3594;
  double t3596;
  double t3598;
  double t3606;
  double t3611;
  double t3612;
  double t3601;
  double t3602;
  double t3576;
  double t3577;
  double t3578;
  double t3582;
  double t3583;
  double t3633;
  double t3634;
  double t3635;
  double t3669;
  double t3672;
  double t3648;
  double t3650;
  double t3660;
  double t3702;
  double t3719;
  double t3728;
  double t3730;
  double t3712;
  double t3713;
  double t3714;
  double t3767;
  double t3768;
  double t3404;
  double t3752;
  double t3753;
  double t3754;
  double t3757;
  double t3759;
  double t3784;
  double t3785;
  double t3786;
  double t3799;
  double t3800;
  double t3792;
  double t3796;
  double t3822;
  double t3826;
  double t3816;
  double t3818;
  double t3833;
  double t3836;
  double t3838;
  double t3829;
  double t3830;
  double t3831;
  double t3771;
  double t3406;
  double t3407;
  double t3855;
  double t3857;
  double t3858;
  double t3862;
  double t3866;
  double t3882;
  double t3884;
  double t3887;
  double t3803;
  double t3806;
  double t3907;
  double t3908;
  double t3909;
  double t3843;
  double t3847;
  t22 = Cos(var1[0]);
  t174 = Cos(var1[1]);
  t1994 = Sin(var1[1]);
  t677 = Sin(var1[0]);
  t2587 = Cos(var1[2]);
  t2647 = -1.*t2587;
  t2869 = 1. + t2647;
  t2917 = Sin(var1[2]);
  t3118 = Cos(var1[3]);
  t3123 = -1.*t3118;
  t3132 = 1. + t3123;
  t3143 = Sin(var1[3]);
  t3087 = t22*t2587*t1994;
  t3101 = -1.*t677*t2917;
  t3107 = t3087 + t3101;
  t3183 = -1.*t2587*t677;
  t3186 = -1.*t22*t1994*t2917;
  t3187 = t3183 + t3186;
  t3213 = Cos(var1[4]);
  t3219 = -1.*t3213;
  t3232 = 1. + t3219;
  t3238 = Sin(var1[4]);
  t3200 = t3118*t3107;
  t3202 = t3187*t3143;
  t3204 = t3200 + t3202;
  t3249 = t3118*t3187;
  t3255 = -1.*t3107*t3143;
  t3257 = t3249 + t3255;
  t1461 = -1.*t174;
  t1493 = 1. + t1461;
  t1968 = 0.331012*t1493;
  t2030 = -0.90524*t1994;
  t2076 = 0. + t1968 + t2030;
  t2876 = -0.97024*t2869;
  t2937 = -0.066675*t2917;
  t2990 = 0. + t2876 + t2937;
  t3035 = -0.066675*t2869;
  t3038 = 0.97024*t2917;
  t3055 = 0. + t3035 + t3038;
  t3142 = -1.45024*t3132;
  t3148 = -0.066675*t3143;
  t3161 = 0. + t3142 + t3148;
  t3195 = -0.066675*t3132;
  t3197 = 1.45024*t3143;
  t3198 = 0. + t3195 + t3197;
  t3338 = t2587*t677*t1994;
  t3339 = t22*t2917;
  t3344 = t3338 + t3339;
  t3348 = t22*t2587;
  t3349 = -1.*t677*t1994*t2917;
  t3352 = t3348 + t3349;
  t3234 = -1.93024*t3232;
  t3240 = -0.065597*t3238;
  t3247 = 0. + t3234 + t3240;
  t3259 = -0.065597*t3232;
  t3263 = 1.93024*t3238;
  t3264 = 0. + t3259 + t3263;
  t3360 = t3118*t3344;
  t3375 = t3352*t3143;
  t3380 = t3360 + t3375;
  t3383 = t3118*t3352;
  t3385 = -1.*t3344*t3143;
  t3386 = t3383 + t3385;
  t3435 = t174*t2587*t3118*t677;
  t3436 = -1.*t174*t677*t2917*t3143;
  t3437 = t3435 + t3436;
  t3439 = -1.*t174*t3118*t677*t2917;
  t3440 = -1.*t174*t2587*t677*t3143;
  t3441 = t3439 + t3440;
  t3419 = -0.90524*t174;
  t3420 = 0.331012*t1994;
  t3421 = t3419 + t3420;
  t3485 = -1.*t22*t174*t2587*t3118;
  t3488 = t22*t174*t2917*t3143;
  t3489 = t3485 + t3488;
  t3495 = t22*t174*t3118*t2917;
  t3498 = t22*t174*t2587*t3143;
  t3499 = t3495 + t3498;
  t3540 = -1.*t2587*t3118*t1994;
  t3543 = t1994*t2917*t3143;
  t3544 = t3540 + t3543;
  t3548 = t3118*t1994*t2917;
  t3550 = t2587*t1994*t3143;
  t3552 = t3548 + t3550;
  t3594 = -1.*t2587*t677*t1994;
  t3596 = -1.*t22*t2917;
  t3598 = t3594 + t3596;
  t3606 = t3118*t3598;
  t3611 = -1.*t3352*t3143;
  t3612 = t3606 + t3611;
  t3601 = t3598*t3143;
  t3602 = t3383 + t3601;
  t3576 = -0.066675*t2587;
  t3577 = -0.97024*t2917;
  t3578 = t3576 + t3577;
  t3582 = 0.97024*t2587;
  t3583 = t3582 + t2937;
  t3633 = t2587*t677;
  t3634 = t22*t1994*t2917;
  t3635 = t3633 + t3634;
  t3669 = -1.*t3635*t3143;
  t3672 = t3200 + t3669;
  t3648 = t3118*t3635;
  t3650 = t3107*t3143;
  t3660 = t3648 + t3650;
  t3702 = 0. + t174;
  t3719 = -1.*t3702*t2587*t3118;
  t3728 = t3702*t2917*t3143;
  t3730 = t3719 + t3728;
  t3712 = -1.*t3702*t3118*t2917;
  t3713 = -1.*t3702*t2587*t3143;
  t3714 = t3712 + t3713;
  t3767 = -1.*t3118*t3344;
  t3768 = t3767 + t3611;
  t3404 = t3213*t3386;
  t3752 = -0.066675*t3118;
  t3753 = -1.45024*t3143;
  t3754 = t3752 + t3753;
  t3757 = 1.45024*t3118;
  t3759 = t3757 + t3148;
  t3784 = -1.*t22*t2587*t1994;
  t3785 = t677*t2917;
  t3786 = t3784 + t3785;
  t3799 = -1.*t3118*t3786;
  t3800 = t3799 + t3669;
  t3792 = -1.*t3786*t3143;
  t3796 = t3648 + t3792;
  t3822 = -1.*t3702*t2917;
  t3826 = 0. + t3822;
  t3816 = t3702*t2587;
  t3818 = 0. + t3816;
  t3833 = -1.*t3818*t3118;
  t3836 = -1.*t3826*t3143;
  t3838 = t3833 + t3836;
  t3829 = t3118*t3826;
  t3830 = -1.*t3818*t3143;
  t3831 = t3829 + t3830;
  t3771 = -1.*t3386*t3238;
  t3406 = -1.*t3380*t3238;
  t3407 = t3404 + t3406;
  t3855 = -0.065597*t3213;
  t3857 = -1.93024*t3238;
  t3858 = t3855 + t3857;
  t3862 = 1.93024*t3213;
  t3866 = t3862 + t3240;
  t3882 = t3118*t3786;
  t3884 = t3635*t3143;
  t3887 = t3882 + t3884;
  t3803 = -1.*t3796*t3238;
  t3806 = t3213*t3796;
  t3907 = t3818*t3118;
  t3908 = t3826*t3143;
  t3909 = t3907 + t3908;
  t3843 = -1.*t3831*t3238;
  t3847 = t3213*t3831;

  p_output1(0)=0.261012*t22 - 0.341012*t174*t22 - 1.*t2076*t22 + t1994*t22*t2990 + t3107*t3161 + t3187*t3198 + t3204*t3247 - 0.000525*(-1.*t3204*t3238 + t3213*t3257) - 1.840292*(t3204*t3213 + t3238*t3257) + t3257*t3264 - 0.066675*t677 - 1.*t3055*t677;
  p_output1(1)=0.066675*t22 + t22*t3055 + t3161*t3344 + t3198*t3352 + t3247*t3380 + t3264*t3386 - 1.840292*(t3213*t3380 + t3238*t3386) - 0.000525*t3407 + 0.261012*t677 - 0.341012*t174*t677 - 1.*t2076*t677 + t1994*t2990*t677;
  p_output1(2)=0;
  p_output1(3)=t3247*t3437 + t3264*t3441 - 0.000525*(-1.*t3238*t3437 + t3213*t3441) - 1.840292*(t3213*t3437 + t3238*t3441) + 0.341012*t1994*t677 + t174*t2990*t677 + t174*t2587*t3161*t677 - 1.*t174*t2917*t3198*t677 - 1.*t3421*t677;
  p_output1(4)=-0.341012*t1994*t22 - 1.*t174*t22*t2990 - 1.*t174*t22*t2587*t3161 + t174*t22*t2917*t3198 + t22*t3421 + t3247*t3489 + t3264*t3499 - 0.000525*(-1.*t3238*t3489 + t3213*t3499) - 1.840292*(t3213*t3489 + t3238*t3499);
  p_output1(5)=0.010000000000000009*t174 + t2030 - 1.*t1994*t2990 - 1.*t1994*t2587*t3161 + t1994*t2917*t3198 + t3247*t3544 + t3264*t3552 - 0.000525*(-1.*t3238*t3544 + t3213*t3552) - 1.840292*(t3213*t3544 + t3238*t3552);
  p_output1(6)=t3161*t3352 + t22*t3583 + t3198*t3598 + t3247*t3602 + t3264*t3612 - 0.000525*(-1.*t3238*t3602 + t3213*t3612) - 1.840292*(t3213*t3602 + t3238*t3612) + t1994*t3578*t677;
  p_output1(7)=t3107*t3198 - 1.*t1994*t22*t3578 + t3161*t3635 + t3247*t3660 + t3264*t3672 - 0.000525*(-1.*t3238*t3660 + t3213*t3672) - 1.840292*(t3213*t3660 + t3238*t3672) + t3583*t677;
  p_output1(8)=-1.*t2917*t3161*t3702 - 1.*t2587*t3198*t3702 + t3578*t3702 + t3247*t3714 + t3264*t3730 - 0.000525*(-1.*t3238*t3714 + t3213*t3730) - 1.840292*(t3213*t3714 + t3238*t3730);
  p_output1(9)=t3247*t3386 + t3344*t3754 + t3352*t3759 + t3264*t3768 - 1.840292*(t3404 + t3238*t3768) - 0.000525*(t3213*t3768 + t3771);
  p_output1(10)=t3635*t3759 + t3754*t3786 + t3247*t3796 + t3264*t3800 - 0.000525*(t3213*t3800 + t3803) - 1.840292*(t3238*t3800 + t3806);
  p_output1(11)=t3754*t3818 + t3759*t3826 + t3247*t3831 + t3264*t3838 - 0.000525*(t3213*t3838 + t3843) - 1.840292*(t3238*t3838 + t3847);
  p_output1(12)=-1.840292*t3407 - 0.000525*(-1.*t3213*t3380 + t3771) + t3380*t3858 + t3386*t3866;
  p_output1(13)=t3796*t3866 + t3858*t3887 - 0.000525*(t3803 - 1.*t3213*t3887) - 1.840292*(t3806 - 1.*t3238*t3887);
  p_output1(14)=t3831*t3866 + t3858*t3909 - 0.000525*(t3843 - 1.*t3213*t3909) - 1.840292*(t3847 - 1.*t3238*t3909);
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


       
void Jp_LeftFootFront(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
