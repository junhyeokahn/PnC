/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 14:29:54 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "vcom_Draco.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1, const Eigen::Matrix<double,10,1> &var2)
{
  double t221;
  double t193;
  double t194;
  double t212;
  double t230;
  double t79;
  double t259;
  double t299;
  double t308;
  double t217;
  double t240;
  double t254;
  double t317;
  double t334;
  double t352;
  double t258;
  double t318;
  double t324;
  double t396;
  double t400;
  double t460;
  double t568;
  double t624;
  double t654;
  double t659;
  double t675;
  double t770;
  double t774;
  double t779;
  double t797;
  double t800;
  double t811;
  double t835;
  double t837;
  double t840;
  double t842;
  double t845;
  double t846;
  double t854;
  double t855;
  double t857;
  double t859;
  double t884;
  double t900;
  double t901;
  double t485;
  double t861;
  double t867;
  double t876;
  double t964;
  double t965;
  double t1052;
  double t1061;
  double t1062;
  double t1063;
  double t1073;
  double t1083;
  double t1086;
  double t1118;
  double t1122;
  double t1125;
  double t1136;
  double t1138;
  double t1145;
  double t1146;
  double t1148;
  double t1149;
  double t1152;
  double t1156;
  double t1157;
  double t1167;
  double t1171;
  double t1201;
  double t1212;
  double t1213;
  double t969;
  double t972;
  double t1174;
  double t1175;
  double t1179;
  double t975;
  double t976;
  double t981;
  double t1303;
  double t1308;
  double t1310;
  double t1314;
  double t1329;
  double t1336;
  double t1337;
  double t1344;
  double t1348;
  double t1349;
  double t1371;
  double t1375;
  double t1384;
  double t1394;
  double t1398;
  double t1399;
  double t1402;
  double t1493;
  double t1494;
  double t1495;
  double t1496;
  double t1500;
  double t1501;
  double t1504;
  double t1510;
  double t1513;
  double t536;
  double t549;
  double t1632;
  double t1648;
  double t1650;
  double t1629;
  double t1652;
  double t1603;
  double t1616;
  double t1628;
  double t1665;
  double t1666;
  double t1668;
  double t1633;
  double t1654;
  double t1659;
  double t1669;
  double t1661;
  double t1670;
  double t1671;
  double t1689;
  double t1690;
  double t1692;
  double t1732;
  double t1738;
  double t1741;
  double t1743;
  double t1753;
  double t1773;
  double t1774;
  double t1775;
  double t1781;
  double t1784;
  double t1792;
  double t1834;
  double t1836;
  double t1837;
  double t1838;
  double t1839;
  double t1842;
  double t1843;
  double t1847;
  double t1849;
  double t1850;
  double t1852;
  double t1853;
  double t1855;
  double t1883;
  double t1884;
  double t1617;
  double t1865;
  double t1871;
  double t1874;
  double t1933;
  double t1939;
  double t1943;
  double t1944;
  double t1946;
  double t1951;
  double t1954;
  double t1963;
  double t1964;
  double t1966;
  double t1968;
  double t1969;
  double t1971;
  double t1972;
  double t1973;
  double t1975;
  double t1980;
  double t1981;
  double t1982;
  double t1999;
  double t2002;
  double t1885;
  double t1888;
  double t1889;
  double t2003;
  double t2005;
  double t2006;
  double t1895;
  double t1897;
  double t2008;
  double t2010;
  double t2011;
  double t2052;
  double t2054;
  double t2055;
  double t2060;
  double t2061;
  double t2062;
  double t2063;
  double t2064;
  double t2065;
  double t2068;
  double t2080;
  double t2082;
  double t2090;
  double t2091;
  double t2093;
  double t2096;
  double t2100;
  double t2133;
  double t2135;
  double t2137;
  double t2139;
  double t2147;
  double t2149;
  double t2150;
  double t2157;
  double t2159;
  double t1695;
  double t1716;
  double t344;
  double t369;
  double t381;
  double t2204;
  double t2208;
  double t2209;
  double t2196;
  double t2197;
  double t2198;
  double t462;
  double t493;
  double t2200;
  double t2210;
  double t2217;
  double t2228;
  double t2229;
  double t2232;
  double t2262;
  double t2281;
  double t2292;
  double t2293;
  double t2302;
  double t2305;
  double t2306;
  double t2294;
  double t2295;
  double t2296;
  double t2364;
  double t2367;
  double t2380;
  double t2384;
  double t2385;
  double t2386;
  double t2399;
  double t2400;
  double t2439;
  double t2441;
  double t2444;
  double t2446;
  double t2237;
  double t2241;
  double t2527;
  double t2530;
  double t2550;
  double t2553;
  double t2562;
  double t2578;
  double t2579;
  double t551;
  double t555;
  double t1608;
  double t1622;
  double t1679;
  double t1685;
  double t1686;
  double t2638;
  double t2642;
  double t2643;
  double t2627;
  double t2628;
  double t2629;
  double t2636;
  double t2646;
  double t2648;
  double t2653;
  double t2657;
  double t2658;
  double t2697;
  double t2716;
  double t2732;
  double t2734;
  double t2741;
  double t2756;
  double t2770;
  double t2772;
  double t2774;
  double t2781;
  double t2824;
  double t2826;
  double t2840;
  double t2841;
  double t2842;
  double t2845;
  double t2847;
  double t2849;
  double t2903;
  double t2904;
  double t2908;
  double t2910;
  double t2661;
  double t2669;
  double t2948;
  double t2950;
  double t2958;
  double t2960;
  double t2961;
  double t2976;
  double t2978;
  double t1717;
  double t1718;
  double t3020;
  double t3022;
  double t3025;
  double t3031;
  double t3037;
  double t3038;
  double t3057;
  double t3059;
  double t3062;
  double t3063;
  double t3064;
  double t3042;
  double t3049;
  double t3101;
  double t3060;
  double t3066;
  double t3109;
  double t3113;
  double t3070;
  double t3071;
  double t3074;
  double t3075;
  double t3077;
  double t3078;
  double t3079;
  double t3083;
  double t3143;
  double t3154;
  double t3156;
  double t3165;
  double t3166;
  double t3167;
  double t3158;
  double t3159;
  double t3161;
  double t3212;
  double t3218;
  double t3222;
  double t3224;
  double t3226;
  double t3227;
  double t3246;
  double t3247;
  double t3250;
  double t3255;
  double t3257;
  double t3232;
  double t3239;
  double t3288;
  double t3248;
  double t3258;
  double t3299;
  double t3300;
  double t3261;
  double t3262;
  double t3267;
  double t3270;
  double t3273;
  double t3275;
  double t3276;
  double t3277;
  double t3327;
  double t3342;
  double t3344;
  double t3347;
  double t3348;
  double t3350;
  double t3353;
  double t3354;
  double t3355;
  t221 = Cos(var1[0]);
  t193 = Cos(var1[2]);
  t194 = Sin(var1[0]);
  t212 = Sin(var1[1]);
  t230 = Sin(var1[2]);
  t79 = Cos(var1[3]);
  t259 = t221*t193;
  t299 = -1.*t194*t212*t230;
  t308 = t259 + t299;
  t217 = t193*t194*t212;
  t240 = t221*t230;
  t254 = t217 + t240;
  t317 = Sin(var1[3]);
  t334 = Cos(var1[4]);
  t352 = Sin(var1[4]);
  t258 = t79*t254;
  t318 = t308*t317;
  t324 = t258 + t318;
  t396 = t79*t308;
  t400 = -1.*t254*t317;
  t460 = t396 + t400;
  t568 = Cos(var1[1]);
  t624 = -0.90524*t568;
  t654 = 0.331012*t212;
  t659 = t624 + t654;
  t675 = -1.*t194*t659;
  t770 = -1.*t193;
  t774 = 1. + t770;
  t779 = -0.97024*t774;
  t797 = -0.066675*t230;
  t800 = t779 + t797;
  t811 = t568*t194*t800;
  t835 = -1.*t79;
  t837 = 1. + t835;
  t840 = -1.45024*t837;
  t842 = -0.066675*t317;
  t845 = t840 + t842;
  t846 = t568*t193*t194*t845;
  t854 = -0.066675*t837;
  t855 = 1.45024*t317;
  t857 = t854 + t855;
  t859 = -1.*t568*t194*t230*t857;
  t884 = t568*t193*t79*t194;
  t900 = -1.*t568*t194*t230*t317;
  t901 = t884 + t900;
  t485 = -0.065597*t352;
  t861 = -1.*t568*t79*t194*t230;
  t867 = -1.*t568*t193*t194*t317;
  t876 = t861 + t867;
  t964 = -1.*t334;
  t965 = 1. + t964;
  t1052 = 0.261012*t221;
  t1061 = -1.*t568;
  t1062 = 1. + t1061;
  t1063 = 0.331012*t1062;
  t1073 = -0.90524*t212;
  t1083 = t1063 + t1073;
  t1086 = -1.*t221*t1083;
  t1118 = -0.066675*t194;
  t1122 = t221*t212*t800;
  t1125 = -0.066675*t774;
  t1136 = 0.97024*t230;
  t1138 = t1125 + t1136;
  t1145 = -1.*t194*t1138;
  t1146 = t221*t193*t212;
  t1148 = -1.*t194*t230;
  t1149 = t1146 + t1148;
  t1152 = -1.*t193*t194;
  t1156 = -1.*t221*t212*t230;
  t1157 = t1152 + t1156;
  t1167 = t1149*t845;
  t1171 = t1157*t857;
  t1201 = t79*t1149;
  t1212 = t1157*t317;
  t1213 = t1201 + t1212;
  t969 = -1.93024*t965;
  t972 = t969 + t485;
  t1174 = t79*t1157;
  t1175 = -1.*t1149*t317;
  t1179 = t1174 + t1175;
  t975 = -0.065597*t965;
  t976 = 1.93024*t352;
  t981 = t975 + t976;
  t1303 = -0.066675*t193;
  t1308 = -0.97024*t230;
  t1310 = t1303 + t1308;
  t1314 = t194*t212*t1310;
  t1329 = 0.97024*t193;
  t1336 = t1329 + t797;
  t1337 = t221*t1336;
  t1344 = -1.*t193*t194*t212;
  t1348 = -1.*t221*t230;
  t1349 = t1344 + t1348;
  t1371 = t308*t845;
  t1375 = t1349*t857;
  t1384 = t1349*t317;
  t1394 = t396 + t1384;
  t1398 = t79*t1349;
  t1399 = -1.*t308*t317;
  t1402 = t1398 + t1399;
  t1493 = -0.066675*t79;
  t1494 = -1.45024*t317;
  t1495 = t1493 + t1494;
  t1496 = t254*t1495;
  t1500 = 1.45024*t79;
  t1501 = t1500 + t842;
  t1504 = t308*t1501;
  t1510 = -1.*t79*t254;
  t1513 = t1510 + t1399;
  t536 = -1.*t460*t352;
  t549 = t334*t460;
  t1632 = Cos(var1[7]);
  t1648 = Sin(var1[5]);
  t1650 = Sin(var1[6]);
  t1629 = Cos(var1[5]);
  t1652 = Sin(var1[7]);
  t1603 = Cos(var1[9]);
  t1616 = Sin(var1[9]);
  t1628 = Cos(var1[8]);
  t1665 = t1632*t1648*t1650;
  t1666 = t1629*t1652;
  t1668 = t1665 + t1666;
  t1633 = t1629*t1632;
  t1654 = -1.*t1648*t1650*t1652;
  t1659 = t1633 + t1654;
  t1669 = Sin(var1[8]);
  t1661 = t1628*t1659;
  t1670 = -1.*t1668*t1669;
  t1671 = t1661 + t1670;
  t1689 = t1628*t1668;
  t1690 = t1659*t1669;
  t1692 = t1689 + t1690;
  t1732 = Cos(var1[6]);
  t1738 = -0.90524*t1732;
  t1741 = -0.330988*t1650;
  t1743 = t1738 + t1741;
  t1753 = -1.*t1648*t1743;
  t1773 = -1.*t1632;
  t1774 = 1. + t1773;
  t1775 = -0.97024*t1774;
  t1781 = -0.066675*t1652;
  t1784 = t1775 + t1781;
  t1792 = t1732*t1648*t1784;
  t1834 = -1.*t1628;
  t1836 = 1. + t1834;
  t1837 = -1.45024*t1836;
  t1838 = -0.066675*t1669;
  t1839 = t1837 + t1838;
  t1842 = t1732*t1632*t1648*t1839;
  t1843 = -0.066675*t1836;
  t1847 = 1.45024*t1669;
  t1849 = t1843 + t1847;
  t1850 = -1.*t1732*t1648*t1652*t1849;
  t1852 = -1.*t1732*t1628*t1648*t1652;
  t1853 = -1.*t1732*t1632*t1648*t1669;
  t1855 = t1852 + t1853;
  t1883 = -1.*t1603;
  t1884 = 1. + t1883;
  t1617 = -0.065597*t1616;
  t1865 = t1732*t1632*t1628*t1648;
  t1871 = -1.*t1732*t1648*t1652*t1669;
  t1874 = t1865 + t1871;
  t1933 = -0.260988*t1629;
  t1939 = -1.*t1732;
  t1943 = 1. + t1939;
  t1944 = -0.330988*t1943;
  t1946 = -0.90524*t1650;
  t1951 = t1944 + t1946;
  t1954 = -1.*t1629*t1951;
  t1963 = -0.066675*t1648;
  t1964 = t1629*t1650*t1784;
  t1966 = -0.066675*t1774;
  t1968 = 0.97024*t1652;
  t1969 = t1966 + t1968;
  t1971 = -1.*t1648*t1969;
  t1972 = t1629*t1632*t1650;
  t1973 = -1.*t1648*t1652;
  t1975 = t1972 + t1973;
  t1980 = -1.*t1632*t1648;
  t1981 = -1.*t1629*t1650*t1652;
  t1982 = t1980 + t1981;
  t1999 = t1975*t1839;
  t2002 = t1982*t1849;
  t1885 = -0.065597*t1884;
  t1888 = 1.93024*t1616;
  t1889 = t1885 + t1888;
  t2003 = t1628*t1982;
  t2005 = -1.*t1975*t1669;
  t2006 = t2003 + t2005;
  t1895 = -1.93024*t1884;
  t1897 = t1895 + t1617;
  t2008 = t1628*t1975;
  t2010 = t1982*t1669;
  t2011 = t2008 + t2010;
  t2052 = -0.066675*t1632;
  t2054 = -0.97024*t1652;
  t2055 = t2052 + t2054;
  t2060 = t1648*t1650*t2055;
  t2061 = 0.97024*t1632;
  t2062 = t2061 + t1781;
  t2063 = t1629*t2062;
  t2064 = -1.*t1632*t1648*t1650;
  t2065 = -1.*t1629*t1652;
  t2068 = t2064 + t2065;
  t2080 = t1659*t1839;
  t2082 = t2068*t1849;
  t2090 = t2068*t1669;
  t2091 = t1661 + t2090;
  t2093 = t1628*t2068;
  t2096 = -1.*t1659*t1669;
  t2100 = t2093 + t2096;
  t2133 = -0.066675*t1628;
  t2135 = -1.45024*t1669;
  t2137 = t2133 + t2135;
  t2139 = t1668*t2137;
  t2147 = 1.45024*t1628;
  t2149 = t2147 + t1838;
  t2150 = t1659*t2149;
  t2157 = -1.*t1628*t1668;
  t2159 = t2157 + t2096;
  t1695 = -1.*t1616*t1671;
  t1716 = t1603*t1671;
  t344 = -0.065597*t334;
  t369 = -1.93024*t352;
  t381 = t344 + t369;
  t2204 = t193*t194;
  t2208 = t221*t212*t230;
  t2209 = t2204 + t2208;
  t2196 = -1.*t221*t193*t212;
  t2197 = t194*t230;
  t2198 = t2196 + t2197;
  t462 = 1.93024*t334;
  t493 = t462 + t485;
  t2200 = t79*t2198;
  t2210 = t2209*t317;
  t2217 = t2200 + t2210;
  t2228 = t79*t2209;
  t2229 = -1.*t2198*t317;
  t2232 = t2228 + t2229;
  t2262 = t221*t659;
  t2281 = -1.*t221*t568*t800;
  t2292 = -1.*t221*t568*t193*t845;
  t2293 = t221*t568*t230*t857;
  t2302 = -1.*t221*t568*t193*t79;
  t2305 = t221*t568*t230*t317;
  t2306 = t2302 + t2305;
  t2294 = t221*t568*t79*t230;
  t2295 = t221*t568*t193*t317;
  t2296 = t2294 + t2295;
  t2364 = -1.*t221*t212*t1310;
  t2367 = t194*t1336;
  t2380 = t2209*t845;
  t2384 = t1149*t857;
  t2385 = t1149*t317;
  t2386 = t2228 + t2385;
  t2399 = -1.*t2209*t317;
  t2400 = t1201 + t2399;
  t2439 = t2198*t1495;
  t2441 = t2209*t1501;
  t2444 = -1.*t79*t2198;
  t2446 = t2444 + t2399;
  t2237 = -1.*t2232*t352;
  t2241 = t334*t2232;
  t2527 = 0.261012*t194;
  t2530 = -1.*t194*t1083;
  t2550 = 0.066675*t221;
  t2553 = t194*t212*t800;
  t2562 = t221*t1138;
  t2578 = t254*t845;
  t2579 = t308*t857;
  t551 = -1.*t324*t352;
  t555 = t549 + t551;
  t1608 = 1.93024*t1603;
  t1622 = t1608 + t1617;
  t1679 = -0.065597*t1603;
  t1685 = -1.93024*t1616;
  t1686 = t1679 + t1685;
  t2638 = -1.*t1629*t1632*t1650;
  t2642 = t1648*t1652;
  t2643 = t2638 + t2642;
  t2627 = t1632*t1648;
  t2628 = t1629*t1650*t1652;
  t2629 = t2627 + t2628;
  t2636 = t1628*t2629;
  t2646 = -1.*t2643*t1669;
  t2648 = t2636 + t2646;
  t2653 = t1628*t2643;
  t2657 = t2629*t1669;
  t2658 = t2653 + t2657;
  t2697 = t1629*t1743;
  t2716 = -1.*t1629*t1732*t1784;
  t2732 = -1.*t1629*t1732*t1632*t1839;
  t2734 = t1629*t1732*t1652*t1849;
  t2741 = t1629*t1732*t1628*t1652;
  t2756 = t1629*t1732*t1632*t1669;
  t2770 = t2741 + t2756;
  t2772 = -1.*t1629*t1732*t1632*t1628;
  t2774 = t1629*t1732*t1652*t1669;
  t2781 = t2772 + t2774;
  t2824 = -1.*t1629*t1650*t2055;
  t2826 = t1648*t2062;
  t2840 = t2629*t1839;
  t2841 = t1975*t1849;
  t2842 = t1975*t1669;
  t2845 = t2636 + t2842;
  t2847 = -1.*t2629*t1669;
  t2849 = t2008 + t2847;
  t2903 = t2643*t2137;
  t2904 = t2629*t2149;
  t2908 = -1.*t1628*t2643;
  t2910 = t2908 + t2847;
  t2661 = -1.*t1616*t2648;
  t2669 = t1603*t2648;
  t2948 = -0.260988*t1648;
  t2950 = -1.*t1648*t1951;
  t2958 = 0.066675*t1629;
  t2960 = t1648*t1650*t1784;
  t2961 = t1629*t1969;
  t2976 = t1668*t1839;
  t2978 = t1659*t1849;
  t1717 = -1.*t1616*t1692;
  t1718 = t1716 + t1717;
  t3020 = t568*t193*t79;
  t3022 = -1.*t568*t230*t317;
  t3025 = t3020 + t3022;
  t3031 = -1.*t568*t79*t230;
  t3037 = -1.*t568*t193*t317;
  t3038 = t3031 + t3037;
  t3057 = t568*t193*t1495;
  t3059 = -1.*t568*t230*t1501;
  t3062 = -1.*t568*t193*t79;
  t3063 = t568*t230*t317;
  t3064 = t3062 + t3063;
  t3042 = -1.*t3038*t352;
  t3049 = t334*t3038;
  t3101 = t568*t1310;
  t3060 = -1.698062*t3038;
  t3066 = -0.082744*t3064;
  t3109 = -1.*t568*t230*t845;
  t3113 = -1.*t568*t193*t857;
  t3070 = t3038*t972;
  t3071 = t3064*t981;
  t3074 = t334*t3064;
  t3075 = t3074 + t3042;
  t3077 = -0.021314*t3075;
  t3078 = t3064*t352;
  t3079 = t3049 + t3078;
  t3083 = -1.919616*t3079;
  t3143 = -1.*t212*t800;
  t3154 = -1.*t193*t212*t845;
  t3156 = t212*t230*t857;
  t3165 = -1.*t193*t79*t212;
  t3166 = t212*t230*t317;
  t3167 = t3165 + t3166;
  t3158 = t79*t212*t230;
  t3159 = t193*t212*t317;
  t3161 = t3158 + t3159;
  t3212 = -1.*t1732*t1628*t1652;
  t3218 = -1.*t1732*t1632*t1669;
  t3222 = t3212 + t3218;
  t3224 = t1732*t1632*t1628;
  t3226 = -1.*t1732*t1652*t1669;
  t3227 = t3224 + t3226;
  t3246 = t1732*t1632*t2137;
  t3247 = -1.*t1732*t1652*t2149;
  t3250 = -1.*t1732*t1632*t1628;
  t3255 = t1732*t1652*t1669;
  t3257 = t3250 + t3255;
  t3232 = -1.*t1616*t3222;
  t3239 = t1603*t3222;
  t3288 = t1732*t2055;
  t3248 = -1.698062*t3222;
  t3258 = -0.082744*t3257;
  t3299 = -1.*t1732*t1652*t1839;
  t3300 = -1.*t1732*t1632*t1849;
  t3261 = t1897*t3222;
  t3262 = t1889*t3257;
  t3267 = t1603*t3257;
  t3270 = t3232 + t3267;
  t3273 = -0.021314*t3270;
  t3275 = t1616*t3257;
  t3276 = t3239 + t3275;
  t3277 = -1.919616*t3276;
  t3327 = -1.*t1650*t1784;
  t3342 = -1.*t1632*t1650*t1839;
  t3344 = t1650*t1652*t1849;
  t3347 = t1628*t1650*t1652;
  t3348 = t1632*t1650*t1669;
  t3350 = t3347 + t3348;
  t3353 = -1.*t1632*t1628*t1650;
  t3354 = t1650*t1652*t1669;
  t3355 = t3353 + t3354;

  p_output1(0)=0.0423728813559322*(0.001331*t194 - 0.076726*t221 + 0.5*(t1052 + t1086 - 0.003391*t194 - 0.930969*t212*t221 - 0.351673*t221*t568) + 6.*(t1052 + t1086 + t1118 + t1122 + t1145 - 1.210348*t1149 - 0.065782*t1157 - 0.347662*t221*t568) + 3.8*(t1052 + t1086 + t1118 + t1122 + t1145 + t1167 + t1171 - 0.082744*t1179 - 1.698062*t1213 - 0.340954*t221*t568) + 0.5*(t1052 + t1086 + t1118 + t1122 + t1145 + t1167 + t1171 - 1.919616*(t1213*t334 + t1179*t352) - 0.021314*(t1179*t334 - 1.*t1213*t352) - 0.341007*t221*t568 + t1213*t972 + t1179*t981))*var2(0) + 0.0423728813559322*(0.5*(0.351673*t194*t212 - 0.930969*t194*t568 + t675) + 6.*(0.347662*t194*t212 - 1.210348*t193*t194*t568 + 0.065782*t194*t230*t568 + t675 + t811) + 3.8*(0.340954*t194*t212 + t675 + t811 + t846 + t859 - 0.082744*t876 - 1.698062*t901) + 0.5*(0.341007*t194*t212 + t675 + t811 + t846 + t859 - 1.919616*(t352*t876 + t334*t901) - 0.021314*(t334*t876 - 1.*t352*t901) + t901*t972 + t876*t981))*var2(1) + 0.0423728813559322*(3.8*(t1314 + t1337 + t1371 + t1375 - 1.698062*t1394 - 0.082744*t1402) + 6.*(t1314 + t1337 - 0.065782*t1349 - 1.210348*t308) + 0.5*(t1314 + t1337 + t1371 + t1375 - 0.021314*(t1402*t334 - 1.*t1394*t352) - 1.919616*(t1394*t334 + t1402*t352) + t1394*t972 + t1402*t981))*var2(2) + 0.0423728813559322*(3.8*(t1496 + t1504 - 0.082744*t1513 - 1.698062*t460) + 0.5*(t1496 + t1504 - 0.021314*(t1513*t334 + t536) - 1.919616*(t1513*t352 + t549) + t460*t972 + t1513*t981))*var2(3) + 0.0211864406779661*(t324*t381 + t460*t493 - 0.021314*(-1.*t324*t334 + t536) - 1.919616*t555)*var2(4) + 0.0423728813559322*(0.076726*t1629 - 0.001331*t1648 + 0.5*(-0.003391*t1648 - 0.930969*t1629*t1650 + 0.351649*t1629*t1732 + t1933 + t1954) + 6.*(0.334338*t1629*t1732 + t1933 + t1954 + t1963 + t1964 + t1971 - 1.210348*t1975 - 0.065782*t1982) + 3.8*(0.341046*t1629*t1732 + t1933 + t1954 + t1963 + t1964 + t1971 + t1999 + t2002 - 0.082744*t2006 - 1.698062*t2011) + 0.5*(0.340993*t1629*t1732 + t1933 + t1954 + t1963 + t1964 + t1971 + t1999 + t2002 + t1889*t2006 + t1897*t2011 - 1.919616*(t1616*t2006 + t1603*t2011) - 0.021314*(t1603*t2006 - 1.*t1616*t2011)))*var2(5) + 0.0423728813559322*(0.5*(-0.351649*t1648*t1650 - 0.930969*t1648*t1732 + t1753) + 6.*(-0.334338*t1648*t1650 - 1.210348*t1632*t1648*t1732 + 0.065782*t1648*t1652*t1732 + t1753 + t1792) + 3.8*(-0.341046*t1648*t1650 + t1753 + t1792 + t1842 + t1850 - 0.082744*t1855 - 1.698062*t1874) + 0.5*(-0.340993*t1648*t1650 + t1753 + t1792 + t1842 + t1850 - 1.919616*(t1616*t1855 + t1603*t1874) - 0.021314*(t1603*t1855 - 1.*t1616*t1874) + t1855*t1889 + t1874*t1897))*var2(6) + 0.0423728813559322*(6.*(-1.210348*t1659 + t2060 + t2063 - 0.065782*t2068) + 3.8*(t2060 + t2063 + t2080 + t2082 - 1.698062*t2091 - 0.082744*t2100) + 0.5*(t2060 + t2063 + t2080 + t2082 + t1897*t2091 + t1889*t2100 - 0.021314*(-1.*t1616*t2091 + t1603*t2100) - 1.919616*(t1603*t2091 + t1616*t2100)))*var2(7) + 0.0423728813559322*(3.8*(-1.698062*t1671 + t2139 + t2150 - 0.082744*t2159) + 0.5*(t1671*t1897 + t2139 + t2150 + t1889*t2159 - 0.021314*(t1695 + t1603*t2159) - 1.919616*(t1716 + t1616*t2159)))*var2(8) + 0.0211864406779661*(t1622*t1671 + t1686*t1692 - 0.021314*(-1.*t1603*t1692 + t1695) - 1.919616*t1718)*var2(9);
  p_output1(1)=0.0423728813559322*(-0.076726*t194 - 0.001331*t221 + 0.5*(-0.930969*t194*t212 + 0.003391*t221 + t2527 + t2530 - 0.351673*t194*t568) + 6.*(t2527 + t2530 - 1.210348*t254 + t2550 + t2553 + t2562 - 0.065782*t308 - 0.347662*t194*t568) + 3.8*(t2527 + t2530 + t2550 + t2553 + t2562 + t2578 + t2579 - 1.698062*t324 - 0.082744*t460 - 0.340954*t194*t568) + 0.5*(t2527 + t2530 + t2550 + t2553 + t2562 + t2578 + t2579 - 1.919616*(t324*t334 + t352*t460) - 0.021314*t555 - 0.341007*t194*t568 + t324*t972 + t460*t981))*var2(0) + 0.0423728813559322*(3.8*(-0.340954*t212*t221 + t2262 + t2281 + t2292 + t2293 - 0.082744*t2296 - 1.698062*t2306) + 0.5*(-0.351673*t212*t221 + t2262 + 0.930969*t221*t568) + 6.*(-0.347662*t212*t221 + t2262 + t2281 + 1.210348*t193*t221*t568 - 0.065782*t221*t230*t568) + 0.5*(-0.341007*t212*t221 + t2262 + t2281 + t2292 + t2293 - 1.919616*(t2306*t334 + t2296*t352) - 0.021314*(t2296*t334 - 1.*t2306*t352) + t2306*t972 + t2296*t981))*var2(1) + 0.0423728813559322*(6.*(-0.065782*t1149 - 1.210348*t2209 + t2364 + t2367) + 3.8*(t2364 + t2367 + t2380 + t2384 - 1.698062*t2386 - 0.082744*t2400) + 0.5*(t2364 + t2367 + t2380 + t2384 - 0.021314*(t2400*t334 - 1.*t2386*t352) - 1.919616*(t2386*t334 + t2400*t352) + t2386*t972 + t2400*t981))*var2(2) + 0.0423728813559322*(3.8*(-1.698062*t2232 + t2439 + t2441 - 0.082744*t2446) + 0.5*(t2439 + t2441 - 0.021314*(t2237 + t2446*t334) - 1.919616*(t2241 + t2446*t352) + t2232*t972 + t2446*t981))*var2(3) + 0.0211864406779661*(-0.021314*(t2237 - 1.*t2217*t334) - 1.919616*(t2241 - 1.*t2217*t352) + t2217*t381 + t2232*t493)*var2(4) + 0.0423728813559322*(0.001331*t1629 + 0.076726*t1648 + 0.5*(0.003391*t1629 - 0.930969*t1648*t1650 + 0.351649*t1648*t1732 + t2948 + t2950) + 6.*(-0.065782*t1659 - 1.210348*t1668 + 0.334338*t1648*t1732 + t2948 + t2950 + t2958 + t2960 + t2961) + 3.8*(-0.082744*t1671 - 1.698062*t1692 + 0.341046*t1648*t1732 + t2948 + t2950 + t2958 + t2960 + t2961 + t2976 + t2978) + 0.5*(-1.919616*(t1616*t1671 + t1603*t1692) - 0.021314*t1718 + 0.340993*t1648*t1732 + t1671*t1889 + t1692*t1897 + t2948 + t2950 + t2958 + t2960 + t2961 + t2976 + t2978))*var2(5) + 0.0423728813559322*(0.5*(0.351649*t1629*t1650 + 0.930969*t1629*t1732 + t2697) + 6.*(0.334338*t1629*t1650 + 1.210348*t1629*t1632*t1732 - 0.065782*t1629*t1652*t1732 + t2697 + t2716) + 3.8*(0.341046*t1629*t1650 + t2697 + t2716 + t2732 + t2734 - 0.082744*t2770 - 1.698062*t2781) + 0.5*(0.340993*t1629*t1650 + t2697 + t2716 + t2732 + t2734 + t1889*t2770 + t1897*t2781 - 1.919616*(t1616*t2770 + t1603*t2781) - 0.021314*(t1603*t2770 - 1.*t1616*t2781)))*var2(6) + 0.0423728813559322*(6.*(-0.065782*t1975 - 1.210348*t2629 + t2824 + t2826) + 3.8*(t2824 + t2826 + t2840 + t2841 - 1.698062*t2845 - 0.082744*t2849) + 0.5*(t2824 + t2826 + t2840 + t2841 + t1897*t2845 + t1889*t2849 - 0.021314*(-1.*t1616*t2845 + t1603*t2849) - 1.919616*(t1603*t2845 + t1616*t2849)))*var2(7) + 0.0423728813559322*(3.8*(-1.698062*t2648 + t2903 + t2904 - 0.082744*t2910) + 0.5*(t1897*t2648 + t2903 + t2904 + t1889*t2910 - 0.021314*(t2661 + t1603*t2910) - 1.919616*(t2669 + t1616*t2910)))*var2(8) + 0.0211864406779661*(t1622*t2648 + t1686*t2658 - 0.021314*(-1.*t1603*t2658 + t2661) - 1.919616*(-1.*t1616*t2658 + t2669))*var2(9);
  p_output1(2)=0.0423728813559322*(3.8*(t1073 + t3143 + t3154 + t3156 - 0.082744*t3161 - 1.698062*t3167 + 0.009942*t568) + 6.*(t1073 + 1.210348*t193*t212 - 0.065782*t212*t230 + t3143 + 0.01665*t568) + 0.5*(0.025729000000000002*t212 + 0.02066100000000004*t568) + 0.5*(t1073 + t3143 + t3154 + t3156 - 1.919616*(t3167*t334 + t3161*t352) - 0.021314*(t3161*t334 - 1.*t3167*t352) + 0.009995000000000032*t568 + t3167*t972 + t3161*t981))*var2(1) + 0.0423728813559322*(3.8*(t3060 + t3066 + t3101 + t3109 + t3113) + 0.5*(t3070 + t3071 + t3077 + t3083 + t3101 + t3109 + t3113) + 6.*(t3101 + 0.065782*t193*t568 + 1.210348*t230*t568))*var2(2) + 0.0423728813559322*(3.8*(t3057 + t3059 + t3060 + t3066) + 0.5*(t3057 + t3059 + t3070 + t3071 + t3077 + t3083))*var2(3) + 0.0211864406779661*(-0.021314*(t3042 - 1.*t3025*t334) - 1.919616*(t3049 - 1.*t3025*t352) + t3025*t381 + t3038*t493)*var2(4) + 0.0423728813559322*(0.5*(0.025729000000000002*t1650 - 0.020660999999999985*t1732) + 6.*(1.210348*t1632*t1650 - 0.065782*t1650*t1652 - 0.00335*t1732 + t1946 + t3327) + 3.8*(-0.010058*t1732 + t1946 + t3327 + t3342 + t3344 - 0.082744*t3350 - 1.698062*t3355) + 0.5*(-0.010004999999999986*t1732 + t1946 + t3327 + t3342 + t3344 + t1889*t3350 + t1897*t3355 - 1.919616*(t1616*t3350 + t1603*t3355) - 0.021314*(t1603*t3350 - 1.*t1616*t3355)))*var2(6) + 0.0423728813559322*(6.*(0.065782*t1632*t1732 + 1.210348*t1652*t1732 + t3288) + 3.8*(t3248 + t3258 + t3288 + t3299 + t3300) + 0.5*(t3261 + t3262 + t3273 + t3277 + t3288 + t3299 + t3300))*var2(7) + 0.0423728813559322*(3.8*(t3246 + t3247 + t3248 + t3258) + 0.5*(t3246 + t3247 + t3261 + t3262 + t3273 + t3277))*var2(8) + 0.0211864406779661*(t1622*t3222 + t1686*t3227 - 0.021314*(-1.*t1603*t3227 + t3232) - 1.919616*(-1.*t1616*t3227 + t3239))*var2(9);
}


       
void vcom_Draco(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,10,1> &var1, const Eigen::Matrix<double,10,1> &var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}