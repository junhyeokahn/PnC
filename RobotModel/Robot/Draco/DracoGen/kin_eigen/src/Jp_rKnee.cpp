/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:26 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RobotSystem/RobotModel/Robot/Draco/DracoGen/kin_eigen/include/Jp_rKnee.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t802;
  double t86;
  double t256;
  double t330;
  double t483;
  double t1463;
  double t1347;
  double t1432;
  double t1481;
  double t834;
  double t875;
  double t897;
  double t1208;
  double t15;
  double t1809;
  double t1824;
  double t1842;
  double t1442;
  double t1535;
  double t1551;
  double t2264;
  double t2298;
  double t2331;
  double t2394;
  double t2403;
  double t2446;
  double t2613;
  double t2773;
  double t2929;
  double t2962;
  double t3378;
  double t3406;
  double t3412;
  double t3669;
  double t3829;
  double t3923;
  double t3961;
  double t4196;
  double t4253;
  double t4307;
  double t4654;
  double t4683;
  double t4751;
  double t457;
  double t679;
  double t680;
  double t1186;
  double t1290;
  double t1322;
  double t1743;
  double t1758;
  double t1772;
  double t1923;
  double t1975;
  double t2103;
  double t5437;
  double t5541;
  double t5591;
  double t2612;
  double t2747;
  double t2762;
  double t5143;
  double t5261;
  double t5270;
  double t5610;
  double t5617;
  double t5621;
  double t3283;
  double t3286;
  double t3355;
  double t3935;
  double t4029;
  double t4136;
  double t5628;
  double t5652;
  double t5657;
  double t5793;
  double t5796;
  double t5797;
  double t4386;
  double t4389;
  double t4510;
  double t5805;
  double t5819;
  double t5832;
  double t5846;
  double t5867;
  double t5876;
  double t6119;
  double t6121;
  double t6122;
  double t6144;
  double t6154;
  double t6171;
  double t6217;
  double t6219;
  double t6221;
  double t6224;
  double t6227;
  double t6230;
  double t6239;
  double t6242;
  double t6246;
  double t6457;
  double t6459;
  double t6460;
  double t6467;
  double t6470;
  double t6484;
  double t6574;
  double t6577;
  double t6584;
  double t6586;
  double t6587;
  double t6588;
  double t6591;
  double t6592;
  double t6595;
  double t6691;
  double t6707;
  double t6710;
  double t6720;
  double t6729;
  double t6741;
  double t6764;
  double t6765;
  double t6767;
  double t6772;
  double t6774;
  double t6778;
  double t6789;
  double t6790;
  double t6804;
  double t6878;
  double t6893;
  double t6896;
  double t6899;
  double t6902;
  double t6903;
  double t6923;
  double t6926;
  double t6934;
  double t6941;
  double t6942;
  double t6946;
  double t7007;
  double t7010;
  double t7015;
  double t7024;
  double t7025;
  double t7027;
  double t7048;
  double t7054;
  double t7062;
  double t7069;
  double t7070;
  double t7079;
  double t7125;
  double t7131;
  double t7134;
  double t7167;
  double t7169;
  double t7171;
  double t7175;
  double t7177;
  double t7180;
  double t7266;
  double t7270;
  double t7277;
  double t7289;
  double t7292;
  double t7293;
  double t7302;
  double t7310;
  double t7311;
  double t7239;
  double t7245;
  double t7250;
  double t7257;
  double t7258;
  double t7344;
  double t7345;
  double t7349;
  double t7375;
  double t7378;
  double t7359;
  double t7360;
  double t7361;
  double t7393;
  double t7396;
  double t7401;
  double t7404;
  double t7407;
  double t7412;
  double t7491;
  double t7494;
  double t7495;
  double t7475;
  double t7476;
  double t7485;
  double t7508;
  double t7518;
  double t7523;
  double t7527;
  double t7528;
  double t7532;
  double t7600;
  double t7605;
  double t7606;
  double t7579;
  double t7584;
  double t7588;
  double t7591;
  double t7593;
  double t7665;
  double t7667;
  double t7672;
  double t7749;
  double t7750;
  double t7751;
  double t7828;
  double t7837;
  double t7851;
  double t5892;
  double t7812;
  double t7815;
  double t7819;
  double t7821;
  double t7826;
  double t7685;
  double t7692;
  double t7698;
  double t7893;
  double t7899;
  double t7907;
  double t7915;
  double t7920;
  double t7928;
  double t7935;
  double t7939;
  double t7943;
  double t7760;
  double t7766;
  double t7768;
  double t8006;
  double t8007;
  double t8014;
  double t8024;
  double t8029;
  double t8034;
  double t8043;
  double t8044;
  double t8048;
  double t5889;
  double t5895;
  double t7875;
  double t8078;
  double t8079;
  double t8080;
  double t8086;
  double t8089;
  double t8114;
  double t8116;
  double t8135;
  double t7966;
  double t7981;
  double t8186;
  double t8190;
  double t8192;
  double t8053;
  double t8069;
  t802 = Sin(var1[3]);
  t86 = Cos(var1[11]);
  t256 = -1.*t86;
  t330 = 1. + t256;
  t483 = Sin(var1[11]);
  t1463 = Cos(var1[3]);
  t1347 = Cos(var1[5]);
  t1432 = Sin(var1[4]);
  t1481 = Sin(var1[5]);
  t834 = Cos(var1[12]);
  t875 = -1.*t834;
  t897 = 1. + t875;
  t1208 = Sin(var1[12]);
  t15 = Cos(var1[4]);
  t1809 = -1.*t1463*t1347;
  t1824 = -1.*t802*t1432*t1481;
  t1842 = t1809 + t1824;
  t1442 = -1.*t1347*t802*t1432;
  t1535 = t1463*t1481;
  t1551 = t1442 + t1535;
  t2264 = t15*t483*t802;
  t2298 = t86*t1842;
  t2331 = t2264 + t2298;
  t2394 = Cos(var1[13]);
  t2403 = -1.*t2394;
  t2446 = 1. + t2403;
  t2613 = Sin(var1[13]);
  t2773 = -1.*t86*t15*t802;
  t2929 = t483*t1842;
  t2962 = t2773 + t2929;
  t3378 = t834*t1551;
  t3406 = -1.*t1208*t2331;
  t3412 = t3378 + t3406;
  t3669 = Cos(var1[14]);
  t3829 = -1.*t3669;
  t3923 = 1. + t3829;
  t3961 = Sin(var1[14]);
  t4196 = t2613*t2962;
  t4253 = t2394*t3412;
  t4307 = t4196 + t4253;
  t4654 = t2394*t2962;
  t4683 = -1.*t2613*t3412;
  t4751 = t4654 + t4683;
  t457 = -0.0222*t330;
  t679 = -0.087*t483;
  t680 = 0. + t457 + t679;
  t1186 = -0.3151*t897;
  t1290 = 0.157*t1208;
  t1322 = 0. + t1186 + t1290;
  t1743 = -0.087*t330;
  t1758 = 0.0222*t483;
  t1772 = 0. + t1743 + t1758;
  t1923 = -0.157*t897;
  t1975 = -0.3151*t1208;
  t2103 = 0. + t1923 + t1975;
  t5437 = -1.*t1347*t802;
  t5541 = t1463*t1432*t1481;
  t5591 = t5437 + t5541;
  t2612 = -0.0222*t2446;
  t2747 = 0.3801*t2613;
  t2762 = 0. + t2612 + t2747;
  t5143 = t1463*t1347*t1432;
  t5261 = t802*t1481;
  t5270 = t5143 + t5261;
  t5610 = -1.*t1463*t15*t483;
  t5617 = t86*t5591;
  t5621 = t5610 + t5617;
  t3283 = -0.3801*t2446;
  t3286 = -0.0222*t2613;
  t3355 = 0. + t3283 + t3286;
  t3935 = -0.8601*t3923;
  t4029 = -0.0222*t3961;
  t4136 = 0. + t3935 + t4029;
  t5628 = t86*t1463*t15;
  t5652 = t483*t5591;
  t5657 = t5628 + t5652;
  t5793 = t834*t5270;
  t5796 = -1.*t1208*t5621;
  t5797 = t5793 + t5796;
  t4386 = -0.0222*t3923;
  t4389 = 0.8601*t3961;
  t4510 = 0. + t4386 + t4389;
  t5805 = t2613*t5657;
  t5819 = t2394*t5797;
  t5832 = t5805 + t5819;
  t5846 = t2394*t5657;
  t5867 = -1.*t2613*t5797;
  t5876 = t5846 + t5867;
  t6119 = t1463*t483*t1432;
  t6121 = t86*t1463*t15*t1481;
  t6122 = t6119 + t6121;
  t6144 = -1.*t86*t1463*t1432;
  t6154 = t1463*t15*t483*t1481;
  t6171 = t6144 + t6154;
  t6217 = t834*t1463*t15*t1347;
  t6219 = -1.*t1208*t6122;
  t6221 = t6217 + t6219;
  t6224 = t2613*t6171;
  t6227 = t2394*t6221;
  t6230 = t6224 + t6227;
  t6239 = t2394*t6171;
  t6242 = -1.*t2613*t6221;
  t6246 = t6239 + t6242;
  t6457 = t483*t802*t1432;
  t6459 = t86*t15*t802*t1481;
  t6460 = t6457 + t6459;
  t6467 = -1.*t86*t802*t1432;
  t6470 = t15*t483*t802*t1481;
  t6484 = t6467 + t6470;
  t6574 = t834*t15*t1347*t802;
  t6577 = -1.*t1208*t6460;
  t6584 = t6574 + t6577;
  t6586 = t2613*t6484;
  t6587 = t2394*t6584;
  t6588 = t6586 + t6587;
  t6591 = t2394*t6484;
  t6592 = -1.*t2613*t6584;
  t6595 = t6591 + t6592;
  t6691 = t15*t483;
  t6707 = -1.*t86*t1432*t1481;
  t6710 = t6691 + t6707;
  t6720 = -1.*t86*t15;
  t6729 = -1.*t483*t1432*t1481;
  t6741 = t6720 + t6729;
  t6764 = -1.*t834*t1347*t1432;
  t6765 = -1.*t1208*t6710;
  t6767 = t6764 + t6765;
  t6772 = t2613*t6741;
  t6774 = t2394*t6767;
  t6778 = t6772 + t6774;
  t6789 = t2394*t6741;
  t6790 = -1.*t2613*t6767;
  t6804 = t6789 + t6790;
  t6878 = t1347*t802;
  t6893 = -1.*t1463*t1432*t1481;
  t6896 = t6878 + t6893;
  t6899 = -1.*t86*t1208*t5270;
  t6902 = t834*t6896;
  t6903 = t6899 + t6902;
  t6923 = t483*t2613*t5270;
  t6926 = t2394*t6903;
  t6934 = t6923 + t6926;
  t6941 = t2394*t483*t5270;
  t6942 = -1.*t2613*t6903;
  t6946 = t6941 + t6942;
  t7007 = t1347*t802*t1432;
  t7010 = -1.*t1463*t1481;
  t7015 = t7007 + t7010;
  t7024 = -1.*t86*t1208*t7015;
  t7025 = t834*t1842;
  t7027 = t7024 + t7025;
  t7048 = t483*t2613*t7015;
  t7054 = t2394*t7027;
  t7062 = t7048 + t7054;
  t7069 = t2394*t483*t7015;
  t7070 = -1.*t2613*t7027;
  t7079 = t7069 + t7070;
  t7125 = -1.*t86*t15*t1347*t1208;
  t7131 = -1.*t834*t15*t1481;
  t7134 = t7125 + t7131;
  t7167 = t15*t1347*t483*t2613;
  t7169 = t2394*t7134;
  t7171 = t7167 + t7169;
  t7175 = t2394*t15*t1347*t483;
  t7177 = -1.*t2613*t7134;
  t7180 = t7175 + t7177;
  t7266 = -1.*t86*t1463*t15;
  t7270 = -1.*t483*t5591;
  t7277 = t7266 + t7270;
  t7289 = t2613*t5621;
  t7292 = -1.*t2394*t1208*t7277;
  t7293 = t7289 + t7292;
  t7302 = t2394*t5621;
  t7310 = t1208*t2613*t7277;
  t7311 = t7302 + t7310;
  t7239 = -0.087*t86;
  t7245 = -0.0222*t483;
  t7250 = t7239 + t7245;
  t7257 = 0.0222*t86;
  t7258 = t7257 + t679;
  t7344 = t1463*t1347;
  t7345 = t802*t1432*t1481;
  t7349 = t7344 + t7345;
  t7375 = -1.*t483*t7349;
  t7378 = t2773 + t7375;
  t7359 = -1.*t15*t483*t802;
  t7360 = t86*t7349;
  t7361 = t7359 + t7360;
  t7393 = t2613*t7361;
  t7396 = -1.*t2394*t1208*t7378;
  t7401 = t7393 + t7396;
  t7404 = t2394*t7361;
  t7407 = t1208*t2613*t7378;
  t7412 = t7404 + t7407;
  t7491 = t86*t1432;
  t7494 = -1.*t15*t483*t1481;
  t7495 = t7491 + t7494;
  t7475 = t483*t1432;
  t7476 = t86*t15*t1481;
  t7485 = t7475 + t7476;
  t7508 = t2613*t7485;
  t7518 = -1.*t2394*t1208*t7495;
  t7523 = t7508 + t7518;
  t7527 = t2394*t7485;
  t7528 = t1208*t2613*t7495;
  t7532 = t7527 + t7528;
  t7600 = -1.*t1208*t5270;
  t7605 = -1.*t834*t5621;
  t7606 = t7600 + t7605;
  t7579 = 0.157*t834;
  t7584 = t7579 + t1975;
  t7588 = -0.3151*t834;
  t7591 = -0.157*t1208;
  t7593 = t7588 + t7591;
  t7665 = -1.*t1208*t7015;
  t7667 = -1.*t834*t7361;
  t7672 = t7665 + t7667;
  t7749 = -1.*t15*t1347*t1208;
  t7750 = -1.*t834*t7485;
  t7751 = t7749 + t7750;
  t7828 = -1.*t2613*t5657;
  t7837 = -1.*t2394*t5797;
  t7851 = t7828 + t7837;
  t5892 = t3669*t5876;
  t7812 = 0.3801*t2394;
  t7815 = t7812 + t3286;
  t7819 = -0.0222*t2394;
  t7821 = -0.3801*t2613;
  t7826 = t7819 + t7821;
  t7685 = t834*t7015;
  t7692 = -1.*t1208*t7361;
  t7698 = t7685 + t7692;
  t7893 = t86*t15*t802;
  t7899 = t483*t7349;
  t7907 = t7893 + t7899;
  t7915 = -1.*t2613*t7907;
  t7920 = -1.*t2394*t7698;
  t7928 = t7915 + t7920;
  t7935 = t2394*t7907;
  t7939 = -1.*t2613*t7698;
  t7943 = t7935 + t7939;
  t7760 = t834*t15*t1347;
  t7766 = -1.*t1208*t7485;
  t7768 = t7760 + t7766;
  t8006 = -1.*t86*t1432;
  t8007 = t15*t483*t1481;
  t8014 = t8006 + t8007;
  t8024 = -1.*t2613*t8014;
  t8029 = -1.*t2394*t7768;
  t8034 = t8024 + t8029;
  t8043 = t2394*t8014;
  t8044 = -1.*t2613*t7768;
  t8048 = t8043 + t8044;
  t5889 = -1.*t3961*t5832;
  t5895 = t5889 + t5892;
  t7875 = -1.*t3961*t5876;
  t8078 = -0.0222*t3669;
  t8079 = -0.8601*t3961;
  t8080 = t8078 + t8079;
  t8086 = 0.8601*t3669;
  t8089 = t8086 + t4029;
  t8114 = t2613*t7907;
  t8116 = t2394*t7698;
  t8135 = t8114 + t8116;
  t7966 = t3669*t7943;
  t7981 = -1.*t3961*t7943;
  t8186 = t2613*t8014;
  t8190 = t2394*t7768;
  t8192 = t8186 + t8190;
  t8053 = t3669*t8048;
  t8069 = -1.*t3961*t8048;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1322*t1551 + t1772*t1842 + t2103*t2331 + t2762*t2962 + t3355*t3412 + t4136*t4307 + t4510*t4751 - 0.0222*(-1.*t3961*t4307 + t3669*t4751) - 0.8601*(t3669*t4307 + t3961*t4751) - 1.*t15*t680*t802 - 0.15025*(t1208*t1551 + t2331*t834);
  p_output1(10)=t1322*t5270 + t1772*t5591 + t2103*t5621 + t2762*t5657 + t3355*t5797 + t4136*t5832 + t4510*t5876 - 0.8601*(t3669*t5832 + t3961*t5876) - 0.0222*t5895 + t1463*t15*t680 - 0.15025*(t1208*t5270 + t5621*t834);
  p_output1(11)=0;
  p_output1(12)=t1322*t1347*t1463*t15 + t1463*t1481*t15*t1772 + t2103*t6122 + t2762*t6171 + t3355*t6221 + t4136*t6230 + t4510*t6246 - 0.0222*(-1.*t3961*t6230 + t3669*t6246) - 0.8601*(t3669*t6230 + t3961*t6246) - 1.*t1432*t1463*t680 - 0.15025*(t1208*t1347*t1463*t15 + t6122*t834);
  p_output1(13)=t2103*t6460 + t2762*t6484 + t3355*t6584 + t4136*t6588 + t4510*t6595 - 0.0222*(-1.*t3961*t6588 + t3669*t6595) - 0.8601*(t3669*t6588 + t3961*t6595) + t1322*t1347*t15*t802 + t1481*t15*t1772*t802 - 1.*t1432*t680*t802 - 0.15025*(t1208*t1347*t15*t802 + t6460*t834);
  p_output1(14)=-1.*t1322*t1347*t1432 - 1.*t1432*t1481*t1772 + t2103*t6710 + t2762*t6741 + t3355*t6767 + t4136*t6778 - 1.*t15*t680 + t4510*t6804 - 0.0222*(-1.*t3961*t6778 + t3669*t6804) - 0.8601*(t3669*t6778 + t3961*t6804) - 0.15025*(-1.*t1208*t1347*t1432 + t6710*t834);
  p_output1(15)=t1772*t5270 + t2762*t483*t5270 + t1322*t6896 + t3355*t6903 + t4136*t6934 + t4510*t6946 - 0.0222*(-1.*t3961*t6934 + t3669*t6946) - 0.8601*(t3669*t6934 + t3961*t6946) + t2103*t5270*t86 - 0.15025*(t1208*t6896 + t5270*t834*t86);
  p_output1(16)=t1322*t1842 + t1772*t7015 + t2762*t483*t7015 + t3355*t7027 + t4136*t7062 + t4510*t7079 - 0.0222*(-1.*t3961*t7062 + t3669*t7079) - 0.8601*(t3669*t7062 + t3961*t7079) + t2103*t7015*t86 - 0.15025*(t1208*t1842 + t7015*t834*t86);
  p_output1(17)=-1.*t1322*t1481*t15 + t1347*t15*t1772 + t1347*t15*t2762*t483 + t3355*t7134 + t4136*t7171 + t4510*t7180 - 0.0222*(-1.*t3961*t7171 + t3669*t7180) - 0.8601*(t3669*t7171 + t3961*t7180) + t1347*t15*t2103*t86 - 0.15025*(-1.*t1208*t1481*t15 + t1347*t15*t834*t86);
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
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t2762*t5621 + t1463*t15*t7250 + t5591*t7258 + t2103*t7277 - 1.*t1208*t3355*t7277 + t4136*t7293 + t4510*t7311 - 0.0222*(-1.*t3961*t7293 + t3669*t7311) - 0.8601*(t3669*t7293 + t3961*t7311) - 0.15025*t7277*t834;
  p_output1(34)=t7258*t7349 + t2762*t7361 + t2103*t7378 - 1.*t1208*t3355*t7378 + t4136*t7401 + t4510*t7412 - 0.0222*(-1.*t3961*t7401 + t3669*t7412) - 0.8601*(t3669*t7401 + t3961*t7412) + t15*t7250*t802 - 0.15025*t7378*t834;
  p_output1(35)=-1.*t1432*t7250 + t1481*t15*t7258 + t2762*t7485 + t2103*t7495 - 1.*t1208*t3355*t7495 + t4136*t7523 + t4510*t7532 - 0.0222*(-1.*t3961*t7523 + t3669*t7532) - 0.8601*(t3669*t7523 + t3961*t7532) - 0.15025*t7495*t834;
  p_output1(36)=-0.15025*t5797 + t5270*t7584 + t5621*t7593 + t3355*t7606 + t2394*t4136*t7606 - 1.*t2613*t4510*t7606 - 0.0222*(-1.*t2613*t3669*t7606 - 1.*t2394*t3961*t7606) - 0.8601*(t2394*t3669*t7606 - 1.*t2613*t3961*t7606);
  p_output1(37)=t7015*t7584 + t7361*t7593 + t3355*t7672 + t2394*t4136*t7672 - 1.*t2613*t4510*t7672 - 0.0222*(-1.*t2613*t3669*t7672 - 1.*t2394*t3961*t7672) - 0.8601*(t2394*t3669*t7672 - 1.*t2613*t3961*t7672) - 0.15025*t7698;
  p_output1(38)=t1347*t15*t7584 + t7485*t7593 + t3355*t7751 + t2394*t4136*t7751 - 1.*t2613*t4510*t7751 - 0.0222*(-1.*t2613*t3669*t7751 - 1.*t2394*t3961*t7751) - 0.8601*(t2394*t3669*t7751 - 1.*t2613*t3961*t7751) - 0.15025*t7768;
  p_output1(39)=t4136*t5876 + t5657*t7815 + t5797*t7826 + t4510*t7851 - 0.8601*(t5892 + t3961*t7851) - 0.0222*(t3669*t7851 + t7875);
  p_output1(40)=t7698*t7826 + t7815*t7907 + t4510*t7928 + t4136*t7943 - 0.8601*(t3961*t7928 + t7966) - 0.0222*(t3669*t7928 + t7981);
  p_output1(41)=t7768*t7826 + t7815*t8014 + t4510*t8034 + t4136*t8048 - 0.8601*(t3961*t8034 + t8053) - 0.0222*(t3669*t8034 + t8069);
  p_output1(42)=-0.8601*t5895 - 0.0222*(-1.*t3669*t5832 + t7875) + t5832*t8080 + t5876*t8089;
  p_output1(43)=t7943*t8089 + t8080*t8135 - 0.0222*(t7981 - 1.*t3669*t8135) - 0.8601*(t7966 - 1.*t3961*t8135);
  p_output1(44)=t8048*t8089 + t8080*t8192 - 0.0222*(t8069 - 1.*t3669*t8192) - 0.8601*(t8053 - 1.*t3961*t8192);
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_rKnee(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
