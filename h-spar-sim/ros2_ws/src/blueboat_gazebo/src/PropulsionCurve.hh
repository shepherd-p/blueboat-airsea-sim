// Derived from the attributed official 16 V chart samples. See propulsion_curve.json.
#pragma once
#include <array>
#include <algorithm>
#include <cmath>
namespace blueboat::propulsion {
inline constexpr std::array<double,33> Pwm{1100,1110,1136,1162,1188,1214,1240,1266,1292,1318,1344,1370,1396,1422,1448,1474,1500,1526,1552,1578,1604,1630,1656,1682,1708,1734,1760,1786,1812,1838,1864,1890,1900};
inline constexpr std::array<double,33> ForceN{-27.556686499999998,-27.360553499999998,-25.889556,-23.732093,-21.672696499999997,-20.005565999999998,-17.9461695,-15.396440499999999,-13.925442999999998,-11.76798,-9.610517,-8.041452999999999,-5.88399,-4.020726499999999,-2.3535959999999996,-0.8825985,0.0,2.0593964999999996,4.903325,8.041452999999999,11.473780499999998,15.494507,18.9268345,23.241760499999998,27.066353999999997,31.675479499999998,35.0097405,39.1285335,42.756994,47.464186,51.190712999999995,53.4462425,55.2114395};
inline constexpr std::array<double,33> Rpm{-2824,-2767,-2723,-2654,-2542,-2454,-2320,-2224,-2080,-1933,-1761,-1591,-1376,-1149,-888,-580,0,537,824,1060,1270,1477,1633,1802,1940,2089,2207,2342,2434,2548,2643,2715,2751};
inline double Interpolate(double v,const std::array<double,33>&x,const std::array<double,33>&y){
  if(!std::isfinite(v))return 0;
  if(v<=x.front())return y.front();if(v>=x.back())return y.back();
  auto i=std::upper_bound(x.begin(),x.end(),v)-x.begin();
  return y[i-1]+(v-x[i-1])/(x[i]-x[i-1])*(y[i]-y[i-1]);
}
inline double RpmFromThrust(double f){return Interpolate(f,ForceN,Rpm);}
inline double ThrustFromRpm(double n){return Interpolate(n,Rpm,ForceN);}
}
