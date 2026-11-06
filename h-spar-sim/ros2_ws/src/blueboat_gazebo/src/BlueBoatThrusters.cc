// Copyright 2026 BlueBoatLab contributors. SPDX-License-Identifier: MIT
// Rigid-body actuation: thrust forces in N at measured/estimated motor offsets.
// RPM is a quasi-static 16 V bench-curve estimate; motor/inflow dynamics omitted.
#include "PropulsionCurve.hh"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>

namespace blueboat {
class Thrusters : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    auto model=gz::sim::Model(entity);
    link_=gz::sim::Link(model.LinkByName(ecm,sdf->Get<std::string>("link_name")));
    max_=sdf->Get<double>("max_thrust");
    reverse_=sdf->Get<double>("reverse_ratio");
    reverse_=std::min(reverse_,-propulsion::ForceN.front()/max_);
    timeout_=sdf->Get<double>("command_timeout");
    auto com=sdf->Get<gz::math::Vector3d>("center_of_mass");
    arms_[0]=sdf->Get<gz::math::Vector3d>("left_position")-com;
    arms_[1]=sdf->Get<gz::math::Vector3d>("right_position")-com;
    node_.Subscribe(sdf->Get<std::string>("left_topic"), &Thrusters::Left, this);
    node_.Subscribe(sdf->Get<std::string>("right_topic"), &Thrusters::Right, this);
    for(int i=0;i<2;++i){
      const std::string prefix=std::string("/blueboat/propulsion/")+(i==0?"left":"right");
      rpmPub_[i]=node_.Advertise<gz::msgs::Double>(prefix+"/rpm");
      forcePub_[i]=node_.Advertise<gz::msgs::Double>(prefix+"/applied_thrust");
    }
  }

  void Left(const gz::msgs::Double &msg) { Receive(0,msg.data()); }
  void Right(const gz::msgs::Double &msg) { Receive(1,msg.data()); }
  void Receive(int side,double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_[side]=std::isfinite(value)?std::clamp(value,-max_*reverse_,max_):0.;
    pending_[side]=true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    double force[2];
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for(int side=0;side<2;++side) {
        if(pending_[side]) { last_[side]=now; pending_[side]=false; }
        force[side]=(now>=last_[side] && now-last_[side]<=timeout_)?commands_[side]:0.;
      }
    }
    double rpm[2];
    for(int i=0;i<2;++i){
      rpm[i]=propulsion::RpmFromThrust(force[i]);
      force[i]=propulsion::ThrustFromRpm(rpm[i]);
    }
    if(now>=nextPublish_){
      for(int i=0;i<2;++i){
        gz::msgs::Double n,f;n.set_data(rpm[i]);f.set_data(force[i]);
        auto stamp=[now](gz::msgs::Double &m){m.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(now));m.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>((now-std::floor(now))*1e9));};
        stamp(n);stamp(f);rpmPub_[i].Publish(n);forcePub_[i].Publish(f);
      }
      nextPublish_=now+.02;
    }
    auto pose=link_.WorldPose(ecm);
    if(!pose) return;
    const gz::math::Vector3d fleft(force[0],0,0), fright(force[1],0,0);
    const auto bodyForce=fleft+fright;
    const auto bodyTorque=arms_[0].Cross(fleft)+arms_[1].Cross(fright);
    link_.AddWorldWrench(ecm,pose->Rot()*bodyForce,pose->Rot()*bodyTorque);
  }

 private:
  gz::sim::Link link_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher rpmPub_[2],forcePub_[2];
  double nextPublish_{0};
  std::mutex mutex_;
  double max_{0},reverse_{1},timeout_{.5};
  double commands_[2]{0,0},last_[2]{-1e10,-1e10};
  bool pending_[2]{false,false};
  gz::math::Vector3d arms_[2];
};
}
GZ_ADD_PLUGIN(blueboat::Thrusters,gz::sim::System,
              blueboat::Thrusters::ISystemConfigure,blueboat::Thrusters::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(blueboat::Thrusters,"blueboat::Thrusters")
