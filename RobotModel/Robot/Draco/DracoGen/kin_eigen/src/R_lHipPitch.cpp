/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:18 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/R_lHipPitch.h"

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
  double t652;
  double t1769;
  double t2452;
  double t2336;
  double t2624;
  double t663;
  double t2349;
  double t2628;
  double t2881;
  double t659;
  double t3045;
  double t644;
  double t4200;
  double t5058;
  double t5062;
  double t5167;
  double t4986;
  double t5017;
  double t4747;
  double t4821;
  double t4960;
  double t4208;
  double t4260;
  double t4499;
  double t5339;
  double t5345;
  double t5349;
  double t5233;
  double t5264;
  double t5282;
  double t5494;
  double t5499;
  double t5501;
  double t4746;
  double t4990;
  double t5004;
  double t1578;
  double t3077;
  double t3782;
  double t5338;
  double t5353;
  double t5355;
  double t5039;
  double t5190;
  double t5208;
  double t5493;
  double t5503;
  double t5508;
  double t5392;
  double t5466;
  double t5469;
  t652 = Cos(var1[3]);
  t1769 = Cos(var1[5]);
  t2452 = Sin(var1[4]);
  t2336 = Sin(var1[3]);
  t2624 = Sin(var1[5]);
  t663 = Cos(var1[6]);
  t2349 = -1.*t1769*t2336;
  t2628 = t652*t2452*t2624;
  t2881 = t2349 + t2628;
  t659 = Cos(var1[4]);
  t3045 = Sin(var1[6]);
  t644 = Cos(var1[8]);
  t4200 = Cos(var1[7]);
  t5058 = t652*t1769;
  t5062 = t2336*t2452*t2624;
  t5167 = t5058 + t5062;
  t4986 = Sin(var1[7]);
  t5017 = Sin(var1[8]);
  t4747 = t663*t2881;
  t4821 = -1.*t652*t659*t3045;
  t4960 = t4747 + t4821;
  t4208 = t652*t1769*t2452;
  t4260 = t2336*t2624;
  t4499 = t4208 + t4260;
  t5339 = t663*t5167;
  t5345 = -1.*t659*t2336*t3045;
  t5349 = t5339 + t5345;
  t5233 = t1769*t2336*t2452;
  t5264 = -1.*t652*t2624;
  t5282 = t5233 + t5264;
  t5494 = t659*t663*t2624;
  t5499 = t2452*t3045;
  t5501 = t5494 + t5499;
  t4746 = t4200*t4499;
  t4990 = -1.*t4960*t4986;
  t5004 = t4746 + t4990;
  t1578 = t652*t659*t663;
  t3077 = t2881*t3045;
  t3782 = t1578 + t3077;
  t5338 = t4200*t5282;
  t5353 = -1.*t5349*t4986;
  t5355 = t5338 + t5353;
  t5039 = t659*t663*t2336;
  t5190 = t5167*t3045;
  t5208 = t5039 + t5190;
  t5493 = t659*t1769*t4200;
  t5503 = -1.*t5501*t4986;
  t5508 = t5493 + t5503;
  t5392 = -1.*t663*t2452;
  t5466 = t659*t2624*t3045;
  t5469 = t5392 + t5466;

  p_output1(0)=-1.*t5004*t5017 + t3782*t644;
  p_output1(1)=-1.*t5017*t5355 + t5208*t644;
  p_output1(2)=-1.*t5017*t5508 + t5469*t644;
  p_output1(3)=t4200*t4960 + t4499*t4986;
  p_output1(4)=t4986*t5282 + t4200*t5349;
  p_output1(5)=t4200*t5501 + t1769*t4986*t659;
  p_output1(6)=t3782*t5017 + t5004*t644;
  p_output1(7)=t5017*t5208 + t5355*t644;
  p_output1(8)=t5017*t5469 + t5508*t644;
}


       
void R_lHipPitch(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
