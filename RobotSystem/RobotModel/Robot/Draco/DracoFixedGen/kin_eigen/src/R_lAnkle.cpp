/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:03 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t940;
  double t249;
  double t251;
  double t323;
  double t1173;
  double t214;
  double t1351;
  double t1366;
  double t1374;
  double t380;
  double t1287;
  double t1323;
  double t1382;
  double t196;
  double t1454;
  double t1466;
  double t1469;
  double t1325;
  double t1425;
  double t1426;
  double t1491;
  double t1604;
  double t1610;
  double t1611;
  double t1571;
  double t1587;
  double t1590;
  double t1633;
  double t1645;
  double t1655;
  double t1593;
  double t1619;
  double t1623;
  double t1697;
  double t1706;
  double t1711;
  double t1717;
  double t1708;
  double t1709;
  double t1739;
  double t1740;
  double t1741;
  double t1710;
  double t1721;
  double t1723;
  double t1446;
  double t1506;
  double t1631;
  double t1673;
  double t1727;
  double t1742;
  t940 = Cos(var1[0]);
  t249 = Cos(var1[2]);
  t251 = Sin(var1[0]);
  t323 = Sin(var1[1]);
  t1173 = Sin(var1[2]);
  t214 = Cos(var1[3]);
  t1351 = t940*t249;
  t1366 = -1.*t251*t323*t1173;
  t1374 = t1351 + t1366;
  t380 = t249*t251*t323;
  t1287 = t940*t1173;
  t1323 = t380 + t1287;
  t1382 = Sin(var1[3]);
  t196 = Cos(var1[4]);
  t1454 = t214*t1374;
  t1466 = -1.*t1323*t1382;
  t1469 = t1454 + t1466;
  t1325 = t214*t1323;
  t1425 = t1374*t1382;
  t1426 = t1325 + t1425;
  t1491 = Sin(var1[4]);
  t1604 = t249*t251;
  t1610 = t940*t323*t1173;
  t1611 = t1604 + t1610;
  t1571 = -1.*t940*t249*t323;
  t1587 = t251*t1173;
  t1590 = t1571 + t1587;
  t1633 = t214*t1611;
  t1645 = -1.*t1590*t1382;
  t1655 = t1633 + t1645;
  t1593 = t214*t1590;
  t1619 = t1611*t1382;
  t1623 = t1593 + t1619;
  t1697 = Cos(var1[1]);
  t1706 = 0. + t1697;
  t1711 = -1.*t1706*t1173;
  t1717 = 0. + t1711;
  t1708 = t1706*t249;
  t1709 = 0. + t1708;
  t1739 = t214*t1717;
  t1740 = -1.*t1709*t1382;
  t1741 = t1739 + t1740;
  t1710 = t1709*t214;
  t1721 = t1717*t1382;
  t1723 = t1710 + t1721;
  t1446 = t196*t1426;
  t1506 = t1469*t1491;
  t1631 = t196*t1623;
  t1673 = t1655*t1491;
  t1727 = t196*t1723;
  t1742 = t1741*t1491;

  p_output1(0)=t1446 + t1506 + 0.000796*(-1.*t1426*t1491 + t1469*t196);
  p_output1(1)=t1631 + t1673 + 0.000796*(-1.*t1491*t1623 + t1655*t196);
  p_output1(2)=t1727 + t1742 + 0.000796*(-1.*t1491*t1723 + t1741*t196);
  p_output1(3)=-1.*t1697*t251;
  p_output1(4)=t1697*t940;
  p_output1(5)=0. + t323;
  p_output1(6)=t1426*t1491 + 0.000796*(t1446 + t1506) - 1.*t1469*t196;
  p_output1(7)=t1491*t1623 + 0.000796*(t1631 + t1673) - 1.*t1655*t196;
  p_output1(8)=t1491*t1723 + 0.000796*(t1727 + t1742) - 1.*t1741*t196;
}


       
void R_lAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
