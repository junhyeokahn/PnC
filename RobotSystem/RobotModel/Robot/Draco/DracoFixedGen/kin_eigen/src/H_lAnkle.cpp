/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:03 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_lAnkle.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  double t941;
  double t555;
  double t580;
  double t666;
  double t995;
  double t484;
  double t1173;
  double t1184;
  double t1218;
  double t940;
  double t1061;
  double t1112;
  double t1231;
  double t447;
  double t1290;
  double t1291;
  double t1300;
  double t1133;
  double t1263;
  double t1282;
  double t1307;
  double t1374;
  double t1382;
  double t1384;
  double t1355;
  double t1366;
  double t1369;
  double t1426;
  double t1433;
  double t1434;
  double t1373;
  double t1411;
  double t1414;
  double t1480;
  double t1481;
  double t1504;
  double t1506;
  double t1491;
  double t1496;
  double t1516;
  double t1539;
  double t1542;
  double t1500;
  double t1508;
  double t1509;
  double t1287;
  double t1323;
  double t1425;
  double t1446;
  double t1510;
  double t1546;
  double t1690;
  double t1691;
  double t1712;
  double t1716;
  double t1748;
  double t1750;
  double t1610;
  double t1325;
  double t1348;
  double t1349;
  double t1658;
  double t1669;
  double t1678;
  double t1679;
  double t1680;
  double t1682;
  double t1683;
  double t1694;
  double t1697;
  double t1701;
  double t1708;
  double t1709;
  double t1710;
  double t1717;
  double t1721;
  double t1723;
  double t1739;
  double t1740;
  double t1741;
  double t1753;
  double t1755;
  double t1758;
  double t1763;
  double t1766;
  double t1771;
  double t1621;
  double t1454;
  double t1462;
  double t1466;
  double t1590;
  double t1641;
  double t1552;
  double t1555;
  double t1559;
  t941 = Cos(var1[0]);
  t555 = Cos(var1[2]);
  t580 = Sin(var1[0]);
  t666 = Sin(var1[1]);
  t995 = Sin(var1[2]);
  t484 = Cos(var1[3]);
  t1173 = t941*t555;
  t1184 = -1.*t580*t666*t995;
  t1218 = t1173 + t1184;
  t940 = t555*t580*t666;
  t1061 = t941*t995;
  t1112 = t940 + t1061;
  t1231 = Sin(var1[3]);
  t447 = Cos(var1[4]);
  t1290 = t484*t1218;
  t1291 = -1.*t1112*t1231;
  t1300 = t1290 + t1291;
  t1133 = t484*t1112;
  t1263 = t1218*t1231;
  t1282 = t1133 + t1263;
  t1307 = Sin(var1[4]);
  t1374 = t555*t580;
  t1382 = t941*t666*t995;
  t1384 = t1374 + t1382;
  t1355 = -1.*t941*t555*t666;
  t1366 = t580*t995;
  t1369 = t1355 + t1366;
  t1426 = t484*t1384;
  t1433 = -1.*t1369*t1231;
  t1434 = t1426 + t1433;
  t1373 = t484*t1369;
  t1411 = t1384*t1231;
  t1414 = t1373 + t1411;
  t1480 = Cos(var1[1]);
  t1481 = 0. + t1480;
  t1504 = -1.*t1481*t995;
  t1506 = 0. + t1504;
  t1491 = t1481*t555;
  t1496 = 0. + t1491;
  t1516 = t484*t1506;
  t1539 = -1.*t1496*t1231;
  t1542 = t1516 + t1539;
  t1500 = t1496*t484;
  t1508 = t1506*t1231;
  t1509 = t1500 + t1508;
  t1287 = t447*t1282;
  t1323 = t1300*t1307;
  t1425 = t447*t1414;
  t1446 = t1434*t1307;
  t1510 = t447*t1509;
  t1546 = t1542*t1307;
  t1690 = -1.*t555;
  t1691 = 1. + t1690;
  t1712 = -1.*t484;
  t1716 = 1. + t1712;
  t1748 = -1.*t447;
  t1750 = 1. + t1748;
  t1610 = t1287 + t1323;
  t1325 = t447*t1300;
  t1348 = -1.*t1282*t1307;
  t1349 = t1325 + t1348;
  t1658 = -1.*t941;
  t1669 = 1. + t1658;
  t1678 = -1.*t1480;
  t1679 = 1. + t1678;
  t1680 = 0.331012*t1679;
  t1682 = -0.90524*t666;
  t1683 = 0. + t1680 + t1682;
  t1694 = -0.97024*t1691;
  t1697 = -0.066675*t995;
  t1701 = 0. + t1694 + t1697;
  t1708 = -0.066675*t1691;
  t1709 = 0.97024*t995;
  t1710 = 0. + t1708 + t1709;
  t1717 = -1.45024*t1716;
  t1721 = -0.066675*t1231;
  t1723 = 0. + t1717 + t1721;
  t1739 = -0.066675*t1716;
  t1740 = 1.45024*t1231;
  t1741 = 0. + t1739 + t1740;
  t1753 = -1.93024*t1750;
  t1755 = -0.065597*t1307;
  t1758 = 0. + t1753 + t1755;
  t1763 = -0.065597*t1750;
  t1766 = 1.93024*t1307;
  t1771 = 0. + t1763 + t1766;
  t1621 = t1425 + t1446;
  t1454 = t447*t1434;
  t1462 = -1.*t1414*t1307;
  t1466 = t1454 + t1462;
  t1590 = 0. + t666;
  t1641 = t1510 + t1546;
  t1552 = t447*t1542;
  t1555 = -1.*t1509*t1307;
  t1559 = t1552 + t1555;

  p_output1(0)=t1287 + t1323 + 0.000796*t1349;
  p_output1(1)=t1425 + t1446 + 0.000796*t1466;
  p_output1(2)=t1510 + t1546 + 0.000796*t1559;
  p_output1(3)=0.;
  p_output1(4)=-1.*t1480*t580;
  p_output1(5)=t1480*t941;
  p_output1(6)=t1590;
  p_output1(7)=0.;
  p_output1(8)=t1282*t1307 + 0.000796*t1610 - 1.*t1300*t447;
  p_output1(9)=t1307*t1414 + 0.000796*t1621 - 1.*t1434*t447;
  p_output1(10)=t1307*t1509 + 0.000796*t1641 - 1.*t1542*t447;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.065597*t1349 - 1.93024*t1610 - 0.066675*t1669 + t1112*t1723 + t1218*t1741 + t1282*t1758 + t1300*t1771 + 0.261012*t580 - 0.341012*t1480*t580 - 1.*t1683*t580 + t1701*t580*t666 + t1710*t941;
  p_output1(13)=0. - 0.065597*t1466 - 1.93024*t1621 + 0.261012*t1669 + t1369*t1723 + t1384*t1741 + t1414*t1758 + t1434*t1771 + 0.066675*t580 + t1710*t580 + 0.341012*t1480*t941 + t1683*t941 - 1.*t1701*t666*t941;
  p_output1(14)=0. - 0.065597*t1559 + 0.341012*t1590 - 1.93024*t1641 - 0.90524*t1679 + t1481*t1701 + t1496*t1723 + t1506*t1741 + t1509*t1758 + t1542*t1771 - 0.331012*t666;
  p_output1(15)=1.;
}


       
void H_lAnkle(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
