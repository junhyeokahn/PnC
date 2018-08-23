/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:21 GMT-05:00
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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t1027;
  double t1192;
  double t289;
  double t1083;
  double t1193;
  double t1713;
  double t1230;
  double t1567;
  double t1576;
  double t1601;
  double t1955;
  double t189;
  double t2083;
  double t2084;
  double t2096;
  double t208;
  double t1165;
  double t1197;
  double t1223;
  double t1226;
  double t1608;
  double t1962;
  double t2019;
  double t2035;
  double t2046;
  double t2047;
  double t2110;
  double t2125;
  double t2074;
  double t2115;
  double t2117;
  double t172;
  double t2126;
  double t2128;
  double t2129;
  double t2137;
  double t2122;
  double t2132;
  double t2134;
  double t153;
  double t2138;
  double t2140;
  double t2141;
  double t2154;
  double t2155;
  double t2156;
  double t2169;
  double t2170;
  double t2171;
  double t2148;
  double t2150;
  double t2151;
  double t2152;
  double t2157;
  double t2164;
  double t2165;
  double t2166;
  double t2167;
  double t2168;
  double t2172;
  double t2173;
  double t2175;
  double t2176;
  double t2180;
  double t2174;
  double t2181;
  double t2184;
  double t2186;
  double t2188;
  double t2189;
  double t2211;
  double t2212;
  double t2213;
  double t2201;
  double t2202;
  double t2205;
  double t2206;
  double t2207;
  double t2208;
  double t2209;
  double t2225;
  double t2305;
  double t2315;
  double t2379;
  double t2385;
  double t2308;
  double t2558;
  double t2571;
  double t2632;
  double t2766;
  double t2812;
  double t2136;
  double t2142;
  double t2185;
  double t2190;
  double t2618;
  double t2851;
  t1027 = Cos(var1[5]);
  t1192 = Sin(var1[3]);
  t289 = Cos(var1[3]);
  t1083 = Sin(var1[4]);
  t1193 = Sin(var1[5]);
  t1713 = Cos(var1[4]);
  t1230 = Cos(var1[6]);
  t1567 = -1.*t1027*t1192;
  t1576 = t289*t1083*t1193;
  t1601 = t1567 + t1576;
  t1955 = Sin(var1[6]);
  t189 = Cos(var1[8]);
  t2083 = t289*t1713*t1230;
  t2084 = t1601*t1955;
  t2096 = t2083 + t2084;
  t208 = Cos(var1[7]);
  t1165 = t289*t1027*t1083;
  t1197 = t1192*t1193;
  t1223 = t1165 + t1197;
  t1226 = t208*t1223;
  t1608 = t1230*t1601;
  t1962 = -1.*t289*t1713*t1955;
  t2019 = t1608 + t1962;
  t2035 = Sin(var1[7]);
  t2046 = -1.*t2019*t2035;
  t2047 = t1226 + t2046;
  t2110 = Sin(var1[8]);
  t2125 = Cos(var1[9]);
  t2074 = t189*t2047;
  t2115 = t2096*t2110;
  t2117 = t2074 + t2115;
  t172 = Sin(var1[9]);
  t2126 = t189*t2096;
  t2128 = -1.*t2047*t2110;
  t2129 = t2126 + t2128;
  t2137 = Cos(var1[10]);
  t2122 = -1.*t172*t2117;
  t2132 = t2125*t2129;
  t2134 = t2122 + t2132;
  t153 = Sin(var1[10]);
  t2138 = t2125*t2117;
  t2140 = t172*t2129;
  t2141 = t2138 + t2140;
  t2154 = t289*t1027;
  t2155 = t1192*t1083*t1193;
  t2156 = t2154 + t2155;
  t2169 = t1713*t1230*t1192;
  t2170 = t2156*t1955;
  t2171 = t2169 + t2170;
  t2148 = t1027*t1192*t1083;
  t2150 = -1.*t289*t1193;
  t2151 = t2148 + t2150;
  t2152 = t208*t2151;
  t2157 = t1230*t2156;
  t2164 = -1.*t1713*t1192*t1955;
  t2165 = t2157 + t2164;
  t2166 = -1.*t2165*t2035;
  t2167 = t2152 + t2166;
  t2168 = t189*t2167;
  t2172 = t2171*t2110;
  t2173 = t2168 + t2172;
  t2175 = t189*t2171;
  t2176 = -1.*t2167*t2110;
  t2180 = t2175 + t2176;
  t2174 = -1.*t172*t2173;
  t2181 = t2125*t2180;
  t2184 = t2174 + t2181;
  t2186 = t2125*t2173;
  t2188 = t172*t2180;
  t2189 = t2186 + t2188;
  t2211 = -1.*t1230*t1083;
  t2212 = t1713*t1193*t1955;
  t2213 = t2211 + t2212;
  t2201 = t1713*t1027*t208;
  t2202 = t1713*t1230*t1193;
  t2205 = t1083*t1955;
  t2206 = t2202 + t2205;
  t2207 = -1.*t2206*t2035;
  t2208 = t2201 + t2207;
  t2209 = t189*t2208;
  t2225 = t2213*t2110;
  t2305 = t2209 + t2225;
  t2315 = t189*t2213;
  t2379 = -1.*t2208*t2110;
  t2385 = t2315 + t2379;
  t2308 = -1.*t172*t2305;
  t2558 = t2125*t2385;
  t2571 = t2308 + t2558;
  t2632 = t2125*t2305;
  t2766 = t172*t2385;
  t2812 = t2632 + t2766;
  t2136 = t153*t2134;
  t2142 = t2137*t2141;
  t2185 = t153*t2184;
  t2190 = t2137*t2189;
  t2618 = t153*t2571;
  t2851 = t2137*t2812;

  p_output1(0)=t2136 + 0.000796*(t2134*t2137 - 1.*t153*t2141) + t2142;
  p_output1(1)=t2185 + 0.000796*(t2137*t2184 - 1.*t153*t2189) + t2190;
  p_output1(2)=t2618 + 0.000796*(t2137*t2571 - 1.*t153*t2812) + t2851;
  p_output1(3)=t1223*t2035 + t2019*t208;
  p_output1(4)=t2035*t2151 + t208*t2165;
  p_output1(5)=t1027*t1713*t2035 + t208*t2206;
  p_output1(6)=-1.*t2134*t2137 + t153*t2141 + 0.000796*(t2136 + t2142);
  p_output1(7)=-1.*t2137*t2184 + t153*t2189 + 0.000796*(t2185 + t2190);
  p_output1(8)=-1.*t2137*t2571 + t153*t2812 + 0.000796*(t2618 + t2851);
}


       
void R_lAnkle(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
