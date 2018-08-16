/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:20:02 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_lAnkle.h"

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
  double t100;
  double t728;
  double t1058;
  double t988;
  double t1091;
  double t1093;
  double t1096;
  double t1126;
  double t1195;
  double t1196;
  double t1216;
  double t1226;
  double t1184;
  double t1192;
  double t1193;
  double t1265;
  double t1269;
  double t1275;
  double t1294;
  double t1295;
  double t1297;
  double t1303;
  double t1290;
  double t1291;
  double t1293;
  double t1325;
  double t1326;
  double t1327;
  double t1017;
  double t1040;
  double t1050;
  double t1061;
  double t1080;
  double t1112;
  double t1133;
  double t1149;
  double t1157;
  double t1168;
  double t1170;
  double t1218;
  double t1231;
  double t1233;
  double t1282;
  double t1284;
  double t1285;
  double t1395;
  double t1405;
  double t1410;
  double t1414;
  double t1418;
  double t1419;
  double t1300;
  double t1307;
  double t1313;
  double t1332;
  double t1335;
  double t1348;
  double t1425;
  double t1426;
  double t1428;
  double t1434;
  double t1437;
  double t1441;
  double t1516;
  double t1519;
  double t1526;
  double t1532;
  double t1534;
  double t1537;
  double t1496;
  double t1500;
  double t1502;
  double t1580;
  double t1581;
  double t1585;
  double t1590;
  double t1593;
  double t1595;
  double t1638;
  double t1641;
  double t1643;
  double t1655;
  double t1658;
  double t1666;
  double t1713;
  double t1715;
  double t1716;
  double t1727;
  double t1730;
  double t1736;
  double t1721;
  double t1722;
  double t1691;
  double t1694;
  double t1704;
  double t1708;
  double t1710;
  double t1771;
  double t1772;
  double t1776;
  double t1793;
  double t1798;
  double t1783;
  double t1786;
  double t1790;
  double t1848;
  double t1867;
  double t1868;
  double t1871;
  double t1858;
  double t1861;
  double t1862;
  double t1929;
  double t1931;
  double t1471;
  double t1905;
  double t1913;
  double t1916;
  double t1921;
  double t1922;
  double t1955;
  double t1956;
  double t1962;
  double t1971;
  double t1972;
  double t1967;
  double t1969;
  double t2006;
  double t2007;
  double t1998;
  double t1999;
  double t2018;
  double t2021;
  double t2023;
  double t2012;
  double t2013;
  double t2014;
  double t1939;
  double t1477;
  double t1480;
  double t2042;
  double t2044;
  double t2046;
  double t2052;
  double t2053;
  double t2064;
  double t2067;
  double t2068;
  double t1979;
  double t1985;
  double t2091;
  double t2095;
  double t2097;
  double t2026;
  double t2031;
  t100 = Cos(var1[0]);
  t728 = Cos(var1[1]);
  t1058 = Sin(var1[1]);
  t988 = Sin(var1[0]);
  t1091 = Cos(var1[2]);
  t1093 = -1.*t1091;
  t1096 = 1. + t1093;
  t1126 = Sin(var1[2]);
  t1195 = Cos(var1[3]);
  t1196 = -1.*t1195;
  t1216 = 1. + t1196;
  t1226 = Sin(var1[3]);
  t1184 = t100*t1091*t1058;
  t1192 = -1.*t988*t1126;
  t1193 = t1184 + t1192;
  t1265 = -1.*t1091*t988;
  t1269 = -1.*t100*t1058*t1126;
  t1275 = t1265 + t1269;
  t1294 = Cos(var1[4]);
  t1295 = -1.*t1294;
  t1297 = 1. + t1295;
  t1303 = Sin(var1[4]);
  t1290 = t1195*t1193;
  t1291 = t1275*t1226;
  t1293 = t1290 + t1291;
  t1325 = t1195*t1275;
  t1326 = -1.*t1193*t1226;
  t1327 = t1325 + t1326;
  t1017 = -1.*t728;
  t1040 = 1. + t1017;
  t1050 = 0.331012*t1040;
  t1061 = -0.90524*t1058;
  t1080 = 0. + t1050 + t1061;
  t1112 = -0.97024*t1096;
  t1133 = -0.066675*t1126;
  t1149 = 0. + t1112 + t1133;
  t1157 = -0.066675*t1096;
  t1168 = 0.97024*t1126;
  t1170 = 0. + t1157 + t1168;
  t1218 = -1.45024*t1216;
  t1231 = -0.066675*t1226;
  t1233 = 0. + t1218 + t1231;
  t1282 = -0.066675*t1216;
  t1284 = 1.45024*t1226;
  t1285 = 0. + t1282 + t1284;
  t1395 = t1091*t988*t1058;
  t1405 = t100*t1126;
  t1410 = t1395 + t1405;
  t1414 = t100*t1091;
  t1418 = -1.*t988*t1058*t1126;
  t1419 = t1414 + t1418;
  t1300 = -1.93024*t1297;
  t1307 = -0.065597*t1303;
  t1313 = 0. + t1300 + t1307;
  t1332 = -0.065597*t1297;
  t1335 = 1.93024*t1303;
  t1348 = 0. + t1332 + t1335;
  t1425 = t1195*t1410;
  t1426 = t1419*t1226;
  t1428 = t1425 + t1426;
  t1434 = t1195*t1419;
  t1437 = -1.*t1410*t1226;
  t1441 = t1434 + t1437;
  t1516 = t728*t1091*t1195*t988;
  t1519 = -1.*t728*t988*t1126*t1226;
  t1526 = t1516 + t1519;
  t1532 = -1.*t728*t1195*t988*t1126;
  t1534 = -1.*t728*t1091*t988*t1226;
  t1537 = t1532 + t1534;
  t1496 = -0.90524*t728;
  t1500 = 0.331012*t1058;
  t1502 = t1496 + t1500;
  t1580 = -1.*t100*t728*t1091*t1195;
  t1581 = t100*t728*t1126*t1226;
  t1585 = t1580 + t1581;
  t1590 = t100*t728*t1195*t1126;
  t1593 = t100*t728*t1091*t1226;
  t1595 = t1590 + t1593;
  t1638 = -1.*t1091*t1195*t1058;
  t1641 = t1058*t1126*t1226;
  t1643 = t1638 + t1641;
  t1655 = t1195*t1058*t1126;
  t1658 = t1091*t1058*t1226;
  t1666 = t1655 + t1658;
  t1713 = -1.*t1091*t988*t1058;
  t1715 = -1.*t100*t1126;
  t1716 = t1713 + t1715;
  t1727 = t1195*t1716;
  t1730 = -1.*t1419*t1226;
  t1736 = t1727 + t1730;
  t1721 = t1716*t1226;
  t1722 = t1434 + t1721;
  t1691 = -0.066675*t1091;
  t1694 = -0.97024*t1126;
  t1704 = t1691 + t1694;
  t1708 = 0.97024*t1091;
  t1710 = t1708 + t1133;
  t1771 = t1091*t988;
  t1772 = t100*t1058*t1126;
  t1776 = t1771 + t1772;
  t1793 = -1.*t1776*t1226;
  t1798 = t1290 + t1793;
  t1783 = t1195*t1776;
  t1786 = t1193*t1226;
  t1790 = t1783 + t1786;
  t1848 = 0. + t728;
  t1867 = -1.*t1848*t1091*t1195;
  t1868 = t1848*t1126*t1226;
  t1871 = t1867 + t1868;
  t1858 = -1.*t1848*t1195*t1126;
  t1861 = -1.*t1848*t1091*t1226;
  t1862 = t1858 + t1861;
  t1929 = -1.*t1195*t1410;
  t1931 = t1929 + t1730;
  t1471 = t1294*t1441;
  t1905 = -0.066675*t1195;
  t1913 = -1.45024*t1226;
  t1916 = t1905 + t1913;
  t1921 = 1.45024*t1195;
  t1922 = t1921 + t1231;
  t1955 = -1.*t100*t1091*t1058;
  t1956 = t988*t1126;
  t1962 = t1955 + t1956;
  t1971 = -1.*t1195*t1962;
  t1972 = t1971 + t1793;
  t1967 = -1.*t1962*t1226;
  t1969 = t1783 + t1967;
  t2006 = -1.*t1848*t1126;
  t2007 = 0. + t2006;
  t1998 = t1848*t1091;
  t1999 = 0. + t1998;
  t2018 = -1.*t1999*t1195;
  t2021 = -1.*t2007*t1226;
  t2023 = t2018 + t2021;
  t2012 = t1195*t2007;
  t2013 = -1.*t1999*t1226;
  t2014 = t2012 + t2013;
  t1939 = -1.*t1441*t1303;
  t1477 = -1.*t1428*t1303;
  t1480 = t1471 + t1477;
  t2042 = -0.065597*t1294;
  t2044 = -1.93024*t1303;
  t2046 = t2042 + t2044;
  t2052 = 1.93024*t1294;
  t2053 = t2052 + t1307;
  t2064 = t1195*t1962;
  t2067 = t1776*t1226;
  t2068 = t2064 + t2067;
  t1979 = -1.*t1969*t1303;
  t1985 = t1294*t1969;
  t2091 = t1999*t1195;
  t2095 = t2007*t1226;
  t2097 = t2091 + t2095;
  t2026 = -1.*t2014*t1303;
  t2031 = t1294*t2014;

  p_output1(0)=0.261012*t100 - 1.*t100*t1080 + t100*t1058*t1149 + t1193*t1233 + t1275*t1285 + t1293*t1313 - 0.065597*(-1.*t1293*t1303 + t1294*t1327) - 1.93024*(t1293*t1294 + t1303*t1327) + t1327*t1348 - 0.341012*t100*t728 - 0.066675*t988 - 1.*t1170*t988;
  p_output1(1)=0.066675*t100 + t100*t1170 + t1233*t1410 + t1285*t1419 + t1313*t1428 + t1348*t1441 - 1.93024*(t1294*t1428 + t1303*t1441) - 0.065597*t1480 + 0.261012*t988 - 1.*t1080*t988 + t1058*t1149*t988 - 0.341012*t728*t988;
  p_output1(2)=0;
  p_output1(3)=t1313*t1526 + t1348*t1537 - 0.065597*(-1.*t1303*t1526 + t1294*t1537) - 1.93024*(t1294*t1526 + t1303*t1537) + 0.341012*t1058*t988 - 1.*t1502*t988 + t1149*t728*t988 + t1091*t1233*t728*t988 - 1.*t1126*t1285*t728*t988;
  p_output1(4)=-0.341012*t100*t1058 + t100*t1502 + t1313*t1585 + t1348*t1595 - 0.065597*(-1.*t1303*t1585 + t1294*t1595) - 1.93024*(t1294*t1585 + t1303*t1595) - 1.*t100*t1149*t728 - 1.*t100*t1091*t1233*t728 + t100*t1126*t1285*t728;
  p_output1(5)=t1061 - 1.*t1058*t1149 - 1.*t1058*t1091*t1233 + t1058*t1126*t1285 + t1313*t1643 + t1348*t1666 - 0.065597*(-1.*t1303*t1643 + t1294*t1666) - 1.93024*(t1294*t1643 + t1303*t1666) + 0.010000000000000009*t728;
  p_output1(6)=t1233*t1419 + t100*t1710 + t1285*t1716 + t1313*t1722 + t1348*t1736 - 0.065597*(-1.*t1303*t1722 + t1294*t1736) - 1.93024*(t1294*t1722 + t1303*t1736) + t1058*t1704*t988;
  p_output1(7)=t1193*t1285 - 1.*t100*t1058*t1704 + t1233*t1776 + t1313*t1790 + t1348*t1798 - 0.065597*(-1.*t1303*t1790 + t1294*t1798) - 1.93024*(t1294*t1790 + t1303*t1798) + t1710*t988;
  p_output1(8)=-1.*t1126*t1233*t1848 - 1.*t1091*t1285*t1848 + t1704*t1848 + t1313*t1862 + t1348*t1871 - 0.065597*(-1.*t1303*t1862 + t1294*t1871) - 1.93024*(t1294*t1862 + t1303*t1871);
  p_output1(9)=t1313*t1441 + t1410*t1916 + t1419*t1922 + t1348*t1931 - 1.93024*(t1471 + t1303*t1931) - 0.065597*(t1294*t1931 + t1939);
  p_output1(10)=t1776*t1922 + t1916*t1962 + t1313*t1969 + t1348*t1972 - 0.065597*(t1294*t1972 + t1979) - 1.93024*(t1303*t1972 + t1985);
  p_output1(11)=t1916*t1999 + t1922*t2007 + t1313*t2014 + t1348*t2023 - 0.065597*(t1294*t2023 + t2026) - 1.93024*(t1303*t2023 + t2031);
  p_output1(12)=-1.93024*t1480 - 0.065597*(-1.*t1294*t1428 + t1939) + t1428*t2046 + t1441*t2053;
  p_output1(13)=t1969*t2053 + t2046*t2068 - 0.065597*(t1979 - 1.*t1294*t2068) - 1.93024*(t1985 - 1.*t1303*t2068);
  p_output1(14)=t2014*t2053 + t2046*t2097 - 0.065597*(t2026 - 1.*t1294*t2097) - 1.93024*(t2031 - 1.*t1303*t2097);
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


       
void Jp_lAnkle(Eigen::Matrix<double,3,10> &p_output1, const Eigen::Matrix<double,10,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
