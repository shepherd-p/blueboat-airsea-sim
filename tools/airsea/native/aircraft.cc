// SPDX-License-Identifier: MIT
// Visualization of the native ArduPlane aerodynamic FDM; no force/pose path is scripted.
#include <mutex>
#include <gz/math/Pose3.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
namespace airsea {
class Aircraft:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
  gz::sim::Model model;gz::transport::Node node;std::mutex mutex;
  gz::math::Pose3d pose;bool received=false;
public:
  void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element> &s,gz::sim::EntityComponentManager &,gz::sim::EventManager &)override{
    model=gz::sim::Model(e);node.Subscribe(s->Get<std::string>("pose_topic"),&Aircraft::OnPose,this);
  }
  void OnPose(const gz::msgs::Pose &m){std::lock_guard<std::mutex> lock(mutex);auto p=m.position();auto q=m.orientation();pose=gz::math::Pose3d({p.x(),p.y(),p.z()},{q.w(),q.x(),q.y(),q.z()});received=true;}
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm)override{
    if(info.paused)return;std::lock_guard<std::mutex> lock(mutex);if(received)model.SetWorldPoseCmd(ecm,pose);
  }
};
}
GZ_ADD_PLUGIN(airsea::Aircraft,gz::sim::System,airsea::Aircraft::ISystemConfigure,airsea::Aircraft::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(airsea::Aircraft,"airsea::Aircraft")
