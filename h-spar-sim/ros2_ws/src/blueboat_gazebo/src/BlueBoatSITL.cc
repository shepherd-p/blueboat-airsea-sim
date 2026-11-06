// SPDX-License-Identifier: MIT
// ArduPilot documented JSON external-physics protocol, loopback UDP only.
#include "PropulsionCurve.hh"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
namespace blueboat {
class SITL:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
  int sock=-1;gz::sim::Link link;gz::transport::Node node;
  gz::transport::Node::Publisher pub[2];sockaddr_in peer{};bool connected=false;
  std::array<int,2> channels{0,2};std::array<uint16_t,32> pwm{};
  uint32_t counter=0;uint64_t packets=0;double last=-1e9,next=0,max=40.207265,reverse=27.5566865;
  std::ofstream log;double nextLog=0;
 public:
  ~SITL()override{if(sock>=0)::close(sock);}
  void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element> &s,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &)override{
    link=gz::sim::Link(gz::sim::Model(e).LinkByName(ecm,"base_link"));link.EnableVelocityChecks(ecm);link.EnableAccelerationChecks(ecm);
    channels={s->Get<int>("left_servo")-1,s->Get<int>("right_servo")-1};
    max=s->Get<double>("max_thrust");reverse=s->Get<double>("reverse_max");
    sock=::socket(AF_INET,SOCK_DGRAM,0);if(sock<0)throw std::runtime_error("SITL UDP socket failed");
    sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(s->Get<int>("port"));
    if(::bind(sock,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0)throw std::runtime_error("SITL UDP bind failed (port already used?)");
    fcntl(sock,F_SETFL,O_NONBLOCK);
    for(int i=0;i<2;++i)pub[i]=node.Advertise<gz::msgs::Double>(std::string("/wamv/thrusters/")+(i==0?"left":"right")+"/thrust");
    if(s->HasElement("telemetry")){log.open(s->Get<std::string>("telemetry"));log<<"t,packets,frame,pwm_left,pwm_right,left_N,right_N,age,roll_ned,pitch_ned,yaw_ned,accel_x,accel_y,accel_z\n";}
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &)override{
    if(info.paused)return;
    const double t=std::chrono::duration<double>(info.simTime).count();
    uint8_t buffer[128];sockaddr_in from{};socklen_t size=sizeof(from);ssize_t n;
    while((n=recvfrom(sock,buffer,sizeof(buffer),0,reinterpret_cast<sockaddr*>(&from),&size))>0){
      uint16_t magic=0;std::memcpy(&magic,buffer,2);
      if(from.sin_addr.s_addr!=htonl(INADDR_LOOPBACK)||!((n==40&&magic==18458)||(n==72&&magic==29569)))continue;
      uint32_t seq;std::memcpy(&seq,buffer+4,4);
      if(!connected||seq!=counter){std::memcpy(pwm.data(),buffer+8,n-8);counter=seq;last=t;++packets;}
      peer=from;connected=true;
    }
    // Same manufacturer's ESC PWM->static thrust samples as RPM mapping.
    for(int i=0;i<2;++i){
      double force=0;const int p=pwm[channels[i]];
      if(connected&&t-last<.5&&p>=1100&&p<=1900)force=std::clamp(propulsion::Interpolate(p,propulsion::Pwm,propulsion::ForceN),-reverse,max);
      gz::msgs::Double msg;msg.set_data(force);pub[i].Publish(msg);
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm)override{
    if(info.paused||!connected)return;
    const double t=std::chrono::duration<double>(info.simTime).count();if(t<next)return;next=t+.004;
    auto pose=link.WorldPose(ecm);auto vel=link.WorldLinearVelocity(ecm);auto angular=link.WorldAngularVelocity(ecm);auto accel=link.WorldLinearAcceleration(ecm);
    if(!pose||!vel||!angular||!accel)return;
    const auto gyro=pose->Rot().RotateVectorReverse(*angular);
    const auto specific=pose->Rot().RotateVectorReverse(*accel-gz::math::Vector3d(0,0,-9.80665));
    const auto euler=pose->Rot().Euler();const auto p=pose->Pos();
    const double yaw=std::remainder(M_PI/2-euler.Z(),2*M_PI);
    std::ostringstream out;out<<std::setprecision(12)<<"\n{\"timestamp\":"<<t
      <<",\"imu\":{\"gyro\":["<<gyro.X()<<','<<-gyro.Y()<<','<<-gyro.Z()<<"],\"accel_body\":["<<specific.X()<<','<<-specific.Y()<<','<<-specific.Z()<<"]}"
      <<",\"position\":["<<p.Y()<<','<<p.X()<<','<<-p.Z()<<"],\"attitude\":["<<euler.X()<<','<<-euler.Y()<<','<<yaw<<']'
      <<",\"velocity\":["<<vel->Y()<<','<<vel->X()<<','<<-vel->Z()<<"],\"battery\":{\"voltage\":16.0,\"current\":0.0}}\n";
    const auto data=out.str();sendto(sock,data.data(),data.size(),0,reinterpret_cast<sockaddr*>(&peer),sizeof(peer));
    if(log&&t>=nextLog){nextLog=t+.02;log<<t<<','<<packets<<','<<counter<<','<<pwm[channels[0]]<<','<<pwm[channels[1]]<<','
      <<((t-last<.5&&pwm[channels[0]]>=1100&&pwm[channels[0]]<=1900)?std::clamp(propulsion::Interpolate(pwm[channels[0]],propulsion::Pwm,propulsion::ForceN),-reverse,max):0)<<','
      <<((t-last<.5&&pwm[channels[1]]>=1100&&pwm[channels[1]]<=1900)?std::clamp(propulsion::Interpolate(pwm[channels[1]],propulsion::Pwm,propulsion::ForceN),-reverse,max):0)<<','
      <<t-last<<','<<euler.X()<<','<<-euler.Y()<<','<<yaw<<','<<specific.X()<<','<<-specific.Y()<<','<<-specific.Z()<<'\n';log.flush();}
  }
};
}
GZ_ADD_PLUGIN(blueboat::SITL,gz::sim::System,blueboat::SITL::ISystemConfigure,blueboat::SITL::ISystemPreUpdate,blueboat::SITL::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(blueboat::SITL,"blueboat::SITL")
