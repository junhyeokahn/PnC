/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:14 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rAnkle.h"

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
  double t211;
  double t659;
  double t1928;
  double t1362;
  double t2546;
  double t2549;
  double t2583;
  double t2596;
  double t2716;
  double t2718;
  double t2719;
  double t2732;
  double t2749;
  double t2754;
  double t2756;
  double t2689;
  double t2695;
  double t2711;
  double t2778;
  double t2780;
  double t2783;
  double t2790;
  double t2796;
  double t2799;
  double t2800;
  double t2811;
  double t2813;
  double t2815;
  double t1487;
  double t1726;
  double t1882;
  double t2084;
  double t2484;
  double t2588;
  double t2630;
  double t2634;
  double t2643;
  double t2647;
  double t2654;
  double t2723;
  double t2734;
  double t2735;
  double t2760;
  double t2765;
  double t2772;
  double t2789;
  double t2793;
  double t2795;
  double t2859;
  double t2860;
  double t2861;
  double t2854;
  double t2855;
  double t2857;
  double t2803;
  double t2807;
  double t2808;
  double t2869;
  double t2873;
  double t2874;
  double t2883;
  double t2884;
  double t2888;
  double t2921;
  double t2923;
  double t2924;
  double t2926;
  double t2927;
  double t2928;
  double t2903;
  double t2905;
  double t2908;
  double t2963;
  double t2966;
  double t2968;
  double t2972;
  double t2975;
  double t2976;
  double t3011;
  double t3013;
  double t3015;
  double t3017;
  double t3018;
  double t3020;
  double t3066;
  double t3071;
  double t3072;
  double t3077;
  double t3079;
  double t3082;
  double t3083;
  double t3084;
  double t3043;
  double t3045;
  double t3046;
  double t3053;
  double t3054;
  double t3108;
  double t3110;
  double t3113;
  double t3118;
  double t3123;
  double t3126;
  double t3131;
  double t3134;
  double t3160;
  double t3172;
  double t3177;
  double t3178;
  double t3181;
  double t3183;
  double t3184;
  double t3221;
  double t3223;
  double t2897;
  double t3200;
  double t3202;
  double t3204;
  double t3210;
  double t3213;
  double t3244;
  double t3245;
  double t3246;
  double t3251;
  double t3254;
  double t3256;
  double t3257;
  double t3276;
  double t3277;
  double t3273;
  double t3274;
  double t3281;
  double t3282;
  double t3283;
  double t3286;
  double t3288;
  double t3290;
  double t3226;
  double t2898;
  double t2900;
  double t3317;
  double t3319;
  double t3326;
  double t3328;
  double t3330;
  double t3260;
  double t3348;
  double t3349;
  double t3351;
  double t3266;
  double t3297;
  double t3380;
  double t3381;
  double t3382;
  double t3308;
  t211 = Cos(var1[5]);
  t659 = Cos(var1[6]);
  t1928 = Sin(var1[6]);
  t1362 = Sin(var1[5]);
  t2546 = Cos(var1[7]);
  t2549 = -1.*t2546;
  t2583 = 1. + t2549;
  t2596 = Sin(var1[7]);
  t2716 = Cos(var1[8]);
  t2718 = -1.*t2716;
  t2719 = 1. + t2718;
  t2732 = Sin(var1[8]);
  t2749 = -1.*t2546*t1362;
  t2754 = -1.*t211*t1928*t2596;
  t2756 = t2749 + t2754;
  t2689 = t211*t2546*t1928;
  t2695 = -1.*t1362*t2596;
  t2711 = t2689 + t2695;
  t2778 = Cos(var1[9]);
  t2780 = -1.*t2778;
  t2783 = 1. + t2780;
  t2790 = Sin(var1[9]);
  t2796 = t2716*t2756;
  t2799 = -1.*t2711*t2732;
  t2800 = t2796 + t2799;
  t2811 = t2716*t2711;
  t2813 = t2756*t2732;
  t2815 = t2811 + t2813;
  t1487 = -1.*t659;
  t1726 = 1. + t1487;
  t1882 = -0.330988*t1726;
  t2084 = -0.90524*t1928;
  t2484 = 0. + t1882 + t2084;
  t2588 = -0.97024*t2583;
  t2630 = -0.066675*t2596;
  t2634 = 0. + t2588 + t2630;
  t2643 = -0.066675*t2583;
  t2647 = 0.97024*t2596;
  t2654 = 0. + t2643 + t2647;
  t2723 = -1.45024*t2719;
  t2734 = -0.066675*t2732;
  t2735 = 0. + t2723 + t2734;
  t2760 = -0.066675*t2719;
  t2765 = 1.45024*t2732;
  t2772 = 0. + t2760 + t2765;
  t2789 = -0.065597*t2783;
  t2793 = 1.93024*t2790;
  t2795 = 0. + t2789 + t2793;
  t2859 = t211*t2546;
  t2860 = -1.*t1362*t1928*t2596;
  t2861 = t2859 + t2860;
  t2854 = t2546*t1362*t1928;
  t2855 = t211*t2596;
  t2857 = t2854 + t2855;
  t2803 = -1.93024*t2783;
  t2807 = -0.065597*t2790;
  t2808 = 0. + t2803 + t2807;
  t2869 = t2716*t2861;
  t2873 = -1.*t2857*t2732;
  t2874 = t2869 + t2873;
  t2883 = t2716*t2857;
  t2884 = t2861*t2732;
  t2888 = t2883 + t2884;
  t2921 = -1.*t659*t2716*t1362*t2596;
  t2923 = -1.*t659*t2546*t1362*t2732;
  t2924 = t2921 + t2923;
  t2926 = t659*t2546*t2716*t1362;
  t2927 = -1.*t659*t1362*t2596*t2732;
  t2928 = t2926 + t2927;
  t2903 = -0.90524*t659;
  t2905 = -0.330988*t1928;
  t2908 = t2903 + t2905;
  t2963 = t211*t659*t2716*t2596;
  t2966 = t211*t659*t2546*t2732;
  t2968 = t2963 + t2966;
  t2972 = -1.*t211*t659*t2546*t2716;
  t2975 = t211*t659*t2596*t2732;
  t2976 = t2972 + t2975;
  t3011 = t2716*t1928*t2596;
  t3013 = t2546*t1928*t2732;
  t3015 = t3011 + t3013;
  t3017 = -1.*t2546*t2716*t1928;
  t3018 = t1928*t2596*t2732;
  t3020 = t3017 + t3018;
  t3066 = -1.*t2546*t1362*t1928;
  t3071 = -1.*t211*t2596;
  t3072 = t3066 + t3071;
  t3077 = t3072*t2732;
  t3079 = t2869 + t3077;
  t3082 = t2716*t3072;
  t3083 = -1.*t2861*t2732;
  t3084 = t3082 + t3083;
  t3043 = -0.066675*t2546;
  t3045 = -0.97024*t2596;
  t3046 = t3043 + t3045;
  t3053 = 0.97024*t2546;
  t3054 = t3053 + t2630;
  t3108 = t2546*t1362;
  t3110 = t211*t1928*t2596;
  t3113 = t3108 + t3110;
  t3118 = t2716*t3113;
  t3123 = t2711*t2732;
  t3126 = t3118 + t3123;
  t3131 = -1.*t3113*t2732;
  t3134 = t2811 + t3131;
  t3160 = 0. + t659;
  t3172 = -1.*t3160*t2716*t2596;
  t3177 = -1.*t3160*t2546*t2732;
  t3178 = t3172 + t3177;
  t3181 = -1.*t3160*t2546*t2716;
  t3183 = t3160*t2596*t2732;
  t3184 = t3181 + t3183;
  t3221 = -1.*t2716*t2857;
  t3223 = t3221 + t3083;
  t2897 = t2778*t2874;
  t3200 = -0.066675*t2716;
  t3202 = -1.45024*t2732;
  t3204 = t3200 + t3202;
  t3210 = 1.45024*t2716;
  t3213 = t3210 + t2734;
  t3244 = -1.*t211*t2546*t1928;
  t3245 = t1362*t2596;
  t3246 = t3244 + t3245;
  t3251 = -1.*t3246*t2732;
  t3254 = t3118 + t3251;
  t3256 = -1.*t2716*t3246;
  t3257 = t3256 + t3131;
  t3276 = -1.*t3160*t2596;
  t3277 = 0. + t3276;
  t3273 = t3160*t2546;
  t3274 = 0. + t3273;
  t3281 = t2716*t3277;
  t3282 = -1.*t3274*t2732;
  t3283 = t3281 + t3282;
  t3286 = -1.*t3274*t2716;
  t3288 = -1.*t3277*t2732;
  t3290 = t3286 + t3288;
  t3226 = -1.*t2790*t2874;
  t2898 = -1.*t2790*t2888;
  t2900 = t2897 + t2898;
  t3317 = 1.93024*t2778;
  t3319 = t3317 + t2807;
  t3326 = -0.065597*t2778;
  t3328 = -1.93024*t2790;
  t3330 = t3326 + t3328;
  t3260 = -1.*t2790*t3254;
  t3348 = t2716*t3246;
  t3349 = t3113*t2732;
  t3351 = t3348 + t3349;
  t3266 = t2778*t3254;
  t3297 = -1.*t2790*t3283;
  t3380 = t3274*t2716;
  t3381 = t3277*t2732;
  t3382 = t3380 + t3381;
  t3308 = t2778*t3283;

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
  p_output1(15)=-0.066675*t1362 - 0.260988*t211 - 1.*t211*t2484 + t1928*t211*t2634 - 1.*t1362*t2654 + t2711*t2735 + t2756*t2772 + t2795*t2800 + t2808*t2815 - 1.93024*(t2790*t2800 + t2778*t2815) - 0.065597*(t2778*t2800 - 1.*t2790*t2815) + 0.340988*t211*t659;
  p_output1(16)=-0.260988*t1362 + 0.066675*t211 - 1.*t1362*t2484 + t1362*t1928*t2634 + t211*t2654 + t2735*t2857 + t2772*t2861 + t2795*t2874 + t2808*t2888 - 1.93024*(t2790*t2874 + t2778*t2888) - 0.065597*t2900 + 0.340988*t1362*t659;
  p_output1(17)=0;
  p_output1(18)=-0.340988*t1362*t1928 - 1.*t1362*t2908 + t2795*t2924 + t2808*t2928 - 1.93024*(t2790*t2924 + t2778*t2928) - 0.065597*(t2778*t2924 - 1.*t2790*t2928) + t1362*t2634*t659 + t1362*t2546*t2735*t659 - 1.*t1362*t2596*t2772*t659;
  p_output1(19)=0.340988*t1928*t211 + t211*t2908 + t2795*t2968 + t2808*t2976 - 1.93024*(t2790*t2968 + t2778*t2976) - 0.065597*(t2778*t2968 - 1.*t2790*t2976) - 1.*t211*t2634*t659 - 1.*t211*t2546*t2735*t659 + t211*t2596*t2772*t659;
  p_output1(20)=t2084 - 1.*t1928*t2634 - 1.*t1928*t2546*t2735 + t1928*t2596*t2772 + t2795*t3015 + t2808*t3020 - 1.93024*(t2790*t3015 + t2778*t3020) - 0.065597*(t2778*t3015 - 1.*t2790*t3020) - 0.010000000000000009*t659;
  p_output1(21)=t2735*t2861 + t1362*t1928*t3046 + t211*t3054 + t2772*t3072 + t2808*t3079 + t2795*t3084 - 0.065597*(-1.*t2790*t3079 + t2778*t3084) - 1.93024*(t2778*t3079 + t2790*t3084);
  p_output1(22)=t2711*t2772 - 1.*t1928*t211*t3046 + t1362*t3054 + t2735*t3113 + t2808*t3126 + t2795*t3134 - 0.065597*(-1.*t2790*t3126 + t2778*t3134) - 1.93024*(t2778*t3126 + t2790*t3134);
  p_output1(23)=-1.*t2596*t2735*t3160 - 1.*t2546*t2772*t3160 + t3046*t3160 + t2808*t3178 + t2795*t3184 - 0.065597*(-1.*t2790*t3178 + t2778*t3184) - 1.93024*(t2778*t3178 + t2790*t3184);
  p_output1(24)=t2808*t2874 + t2857*t3204 + t2861*t3213 + t2795*t3223 - 1.93024*(t2897 + t2790*t3223) - 0.065597*(t2778*t3223 + t3226);
  p_output1(25)=t3113*t3213 + t3204*t3246 + t2808*t3254 + t2795*t3257 - 0.065597*(t2778*t3257 + t3260) - 1.93024*(t2790*t3257 + t3266);
  p_output1(26)=t3204*t3274 + t3213*t3277 + t2808*t3283 + t2795*t3290 - 0.065597*(t2778*t3290 + t3297) - 1.93024*(t2790*t3290 + t3308);
  p_output1(27)=-1.93024*t2900 - 0.065597*(-1.*t2778*t2888 + t3226) + t2874*t3319 + t2888*t3330;
  p_output1(28)=t3254*t3319 + t3330*t3351 - 0.065597*(t3260 - 1.*t2778*t3351) - 1.93024*(t3266 - 1.*t2790*t3351);
  p_output1(29)=t3283*t3319 + t3330*t3382 - 0.065597*(t3297 - 1.*t2778*t3382) - 1.93024*(t3308 - 1.*t2790*t3382);
}


       
void Jp_rAnkle(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
