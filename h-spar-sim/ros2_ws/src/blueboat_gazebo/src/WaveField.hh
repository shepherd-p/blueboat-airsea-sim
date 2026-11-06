// SPDX-License-Identifier: MIT
// Shared small-amplitude deep-water wave field for buoyancy and environment.
#pragma once
#include <cmath>
#include <stdexcept>
#include <vector>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>
namespace blueboat {
struct WaveSample {double height=0, dx=0, dy=0; gz::math::Vector3d velocity{0,0,0};};
class WaveField {
 public:
  struct Wave {double a,k,omega,dx,dy,phase;};
  std::vector<Wave> waves;
  double ramp=3;
  void Load(const std::shared_ptr<const sdf::Element> &sdf) {
    if(!sdf->HasElement("waves"))return;
    auto field=sdf->FindElement("waves");
    ramp=field->Get<double>("ramp_seconds",3).first;
    for(auto w=field->FindElement("wave");w;w=w->GetNextElement("wave")){
      double a=w->Get<double>("amplitude"),length=w->Get<double>("wavelength");
      double angle=w->Get<double>("direction_deg")*M_PI/180;
      if(!std::isfinite(a)||a<0||!std::isfinite(length)||length<=0)throw std::runtime_error("Invalid wave");
      double k=2*M_PI/length;
      waves.push_back({a,k,std::sqrt(9.80665*k),std::cos(angle),std::sin(angle),w->Get<double>("phase",0).first});
    }
  }
  WaveSample At(double x,double y,double z,double t)const{
    WaveSample s;
    const double r=ramp>0?1-std::exp(-std::max(0.,t)/ramp):1;
    const double rd=ramp>0?std::exp(-std::max(0.,t)/ramp)/ramp:0;
    for(const auto &w:waves){
      const double theta=w.k*(w.dx*x+w.dy*y)-w.omega*t+w.phase;
      const double c=std::cos(theta),v=std::sin(theta),a=w.a*r;
      s.height+=a*c;s.dx-=a*w.k*w.dx*v;s.dy-=a*w.k*w.dy*v;
      const double decay=std::exp(w.k*std::min(0.,z));
      s.velocity+=gz::math::Vector3d(a*w.omega*c*w.dx,a*w.omega*c*w.dy,a*w.omega*v+w.a*rd*c)*decay;
    }
    return s;
  }
};
}
