/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:13 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_rAnkle.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t2469;
  double t2500;
  double t2544;
  double t1951;
  double t2550;
  double t2559;
  double t2566;
  double t2574;
  double t2610;
  double t2621;
  double t2625;
  double t2628;
  double t2635;
  double t2636;
  double t2639;
  double t2603;
  double t2608;
  double t2609;
  double t2655;
  double t2656;
  double t2657;
  double t2664;
  double t2667;
  double t2674;
  double t2686;
  double t2695;
  double t2706;
  double t2707;
  double t2242;
  double t2423;
  double t2529;
  double t2540;
  double t2543;
  double t2545;
  double t2547;
  double t2571;
  double t2577;
  double t2580;
  double t2586;
  double t2588;
  double t2591;
  double t2626;
  double t2630;
  double t2631;
  double t2643;
  double t2647;
  double t2649;
  double t2663;
  double t2665;
  double t2666;
  double t2749;
  double t2754;
  double t2756;
  double t2739;
  double t2741;
  double t2743;
  double t2688;
  double t2689;
  double t2694;
  double t2765;
  double t2766;
  double t2767;
  double t2773;
  double t2774;
  double t2777;
  double t2809;
  double t2816;
  double t2817;
  double t2813;
  double t2814;
  double t2822;
  double t2823;
  double t2824;
  double t2827;
  double t2830;
  double t2831;
  t2469 = Sin(var1[5]);
  t2500 = Cos(var1[6]);
  t2544 = Sin(var1[6]);
  t1951 = Cos(var1[5]);
  t2550 = Cos(var1[7]);
  t2559 = -1.*t2550;
  t2566 = 1. + t2559;
  t2574 = Sin(var1[7]);
  t2610 = Cos(var1[8]);
  t2621 = -1.*t2610;
  t2625 = 1. + t2621;
  t2628 = Sin(var1[8]);
  t2635 = t1951*t2550;
  t2636 = -1.*t2469*t2544*t2574;
  t2639 = t2635 + t2636;
  t2603 = t2550*t2469*t2544;
  t2608 = t1951*t2574;
  t2609 = t2603 + t2608;
  t2655 = Cos(var1[9]);
  t2656 = -1.*t2655;
  t2657 = 1. + t2656;
  t2664 = Sin(var1[9]);
  t2667 = t2610*t2639;
  t2674 = -1.*t2609*t2628;
  t2686 = t2667 + t2674;
  t2695 = t2610*t2609;
  t2706 = t2639*t2628;
  t2707 = t2695 + t2706;
  t2242 = -1.*t1951;
  t2423 = 1. + t2242;
  t2529 = -1.*t2500;
  t2540 = 1. + t2529;
  t2543 = -0.330988*t2540;
  t2545 = -0.90524*t2544;
  t2547 = 0. + t2543 + t2545;
  t2571 = -0.97024*t2566;
  t2577 = -0.066675*t2574;
  t2580 = 0. + t2571 + t2577;
  t2586 = -0.066675*t2566;
  t2588 = 0.97024*t2574;
  t2591 = 0. + t2586 + t2588;
  t2626 = -1.45024*t2625;
  t2630 = -0.066675*t2628;
  t2631 = 0. + t2626 + t2630;
  t2643 = -0.066675*t2625;
  t2647 = 1.45024*t2628;
  t2649 = 0. + t2643 + t2647;
  t2663 = -0.065597*t2657;
  t2665 = 1.93024*t2664;
  t2666 = 0. + t2663 + t2665;
  t2749 = t2550*t2469;
  t2754 = t1951*t2544*t2574;
  t2756 = t2749 + t2754;
  t2739 = -1.*t1951*t2550*t2544;
  t2741 = t2469*t2574;
  t2743 = t2739 + t2741;
  t2688 = -1.93024*t2657;
  t2689 = -0.065597*t2664;
  t2694 = 0. + t2688 + t2689;
  t2765 = t2610*t2756;
  t2766 = -1.*t2743*t2628;
  t2767 = t2765 + t2766;
  t2773 = t2610*t2743;
  t2774 = t2756*t2628;
  t2777 = t2773 + t2774;
  t2809 = 0. + t2500;
  t2816 = -1.*t2809*t2574;
  t2817 = 0. + t2816;
  t2813 = t2809*t2550;
  t2814 = 0. + t2813;
  t2822 = t2610*t2817;
  t2823 = -1.*t2814*t2628;
  t2824 = t2822 + t2823;
  t2827 = t2814*t2610;
  t2830 = t2817*t2628;
  t2831 = t2827 + t2830;

  p_output1(0)=0. - 0.066675*t2423 - 0.260988*t2469 + 0.340988*t2469*t2500 - 1.*t2469*t2547 + t2469*t2544*t2580 + t1951*t2591 + t2609*t2631 + t2639*t2649 + t2666*t2686 + t2694*t2707 - 1.93024*(t2664*t2686 + t2655*t2707) - 0.065597*(t2655*t2686 - 1.*t2664*t2707);
  p_output1(1)=0. - 0.260988*t2423 + 0.066675*t2469 - 0.340988*t1951*t2500 + t1951*t2547 - 1.*t1951*t2544*t2580 + t2469*t2591 + t2631*t2743 + t2649*t2756 + t2666*t2767 + t2694*t2777 - 1.93024*(t2664*t2767 + t2655*t2777) - 0.065597*(t2655*t2767 - 1.*t2664*t2777);
  p_output1(2)=0. - 0.90524*t2540 + 0.330988*t2544 - 0.340988*(0. + t2544) + t2580*t2809 + t2631*t2814 + t2649*t2817 + t2666*t2824 + t2694*t2831 - 1.93024*(t2664*t2824 + t2655*t2831) - 0.065597*(t2655*t2824 - 1.*t2664*t2831);
}


       
void p_rAnkle(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
