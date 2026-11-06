// SPDX-License-Identifier: MIT
#include "WaveField.hh"
#include <chrono>
#include <fstream>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
namespace blueboat {
class Environment:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate{
  gz::sim::Link link;WaveField waves;gz::transport::Node node;
  gz::transport::Node::Publisher currentPub,windPub,surfacePub;
  gz::math::Vector3d current,wind,area,center,com;
  double gust=0,period=8,rho=1.225,next=0;std::ofstream log;
 public:
  void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element> &s,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &)override{
    link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,"base_link"));link.EnableVelocityChecks(ecm);
    waves.Load(s);current=s->Get<gz::math::Vector3d>("current");wind=s->Get<gz::math::Vector3d>("wind");
    area=s->Get<gz::math::Vector3d>("drag_area_cd");center=s->Get<gz::math::Vector3d>("wind_center");com=s->Get<gz::math::Vector3d>("center_of_mass");
    gust=s->Get<double>("gust_fraction");period=s->Get<double>("gust_period");
    currentPub=node.Advertise<gz::msgs::Vector3d>("/model/blueboat/ocean_current");
    windPub=node.Advertise<gz::msgs::Vector3d>("/blueboat/environment/wind");
    surfacePub=node.Advertise<gz::msgs::Vector3d>("/blueboat/environment/surface");
    if(s->HasElement("telemetry")){log.open(s->Get<std::string>("telemetry"));log<<"t,x,y,eta,slope_x,slope_y,current_x,current_y,current_z,wind_x,wind_y,wind_fx,wind_fy,wind_mz,probe_eta\n";}
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm)override{
    if(info.paused)return;
    double t=std::chrono::duration<double>(info.simTime).count();
    auto p=link.WorldPose(ecm);auto v=link.WorldLinearVelocity(ecm);auto omega=link.WorldAngularVelocity(ecm);if(!p||!v||!omega)return;
    const auto water=waves.At(p->Pos().X(),p->Pos().Y(),-.06,t);
    const auto flow=current+water.velocity;
    const auto windNow=wind*(1+gust*std::sin(2*M_PI*t/period));
    const auto arm=p->Rot()*(center-com);
    const auto local=p->Rot().RotateVectorReverse(windNow-*v-omega->Cross(arm));
    gz::math::Vector3d f;
    for(int i=0;i<3;++i)f[i]=.5*rho*area[i]*local[i]*std::abs(local[i]);
    const auto fw=p->Rot()*f;const auto torque=arm.Cross(fw);
    link.AddWorldWrench(ecm,fw,torque);
    if(t<next)return;next=t+.01;
    auto publish=[&](auto &pub,const gz::math::Vector3d &vec){gz::msgs::Vector3d m;m.set_x(vec.X());m.set_y(vec.Y());m.set_z(vec.Z());m.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(t));m.mutable_header()->mutable_stamp()->set_nsec(static_cast<int>((t-std::floor(t))*1e9));pub.Publish(m);};
    publish(currentPub,flow);publish(windPub,windNow);publish(surfacePub,{water.height,water.dx,water.dy});
    // Fixed world-origin gauge: avoids the moving boat's encounter-frequency bias.
    if(log)log<<t<<','<<p->Pos().X()<<','<<p->Pos().Y()<<','<<water.height<<','<<water.dx<<','<<water.dy<<','<<flow.X()<<','<<flow.Y()<<','<<flow.Z()<<','<<windNow.X()<<','<<windNow.Y()<<','<<fw.X()<<','<<fw.Y()<<','<<torque.Z()<<','<<waves.At(0,0,0,t).height<<'\n';
  }
};
}
GZ_ADD_PLUGIN(blueboat::Environment,gz::sim::System,blueboat::Environment::ISystemConfigure,blueboat::Environment::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(blueboat::Environment,"blueboat::Environment")
