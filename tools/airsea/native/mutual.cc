// SPDX-License-Identifier: MIT
// Both cameras render the actual scene. Target truth only enters evidence
// records, never pointing: pointing uses ownship EKF plus received image aims.
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QQuickImageProvider>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

namespace {
constexpr double Pi=3.14159265358979323846;
gz::math::Pose3d asPose(const gz::msgs::Pose &p){return {{p.position().x(),p.position().y(),p.position().z()},{p.orientation().w(),p.orientation().x(),p.orientation().y(),p.orientation().z()}};}
class MutualProvider:public QQuickImageProvider{
 QImage image;std::mutex mutex;
public:
 MutualProvider():QQuickImageProvider(QQuickImageProvider::Image){image=QImage(1920,1080,QImage::Format_RGB888);image.fill(QColor("#14212d"));}
 void Set(const cv::Mat &m){cv::Mat rgb;cv::cvtColor(m,rgb,cv::COLOR_BGR2RGB);std::lock_guard<std::mutex> g(mutex);image=QImage(rgb.data,rgb.cols,rgb.rows,rgb.step,QImage::Format_RGB888).copy();}
 QImage requestImage(const QString &,QSize *size,const QSize &)override{std::lock_guard<std::mutex> g(mutex);if(size)*size=image.size();return image;}
};
void vectorJSON(std::ostream &s,const gz::math::Vector3d &p){s<<'['<<p.X()<<','<<p.Y()<<','<<p.Z()<<']';}
}

class AirSeaMutual:public gz::sim::GuiSystem{
 Q_OBJECT
 Q_PROPERTY(int frameId READ FrameId NOTIFY FrameChanged)
 std::mutex mutex;gz::transport::Node node;
 gz::msgs::Pose truth,nav,bnav;bool haveTruth=false,haveNav=false,haveBoatNav=false,active=false;
 gz::math::Vector3d aims[2]={{300,170,.16},{370,170,45}};
 gz::math::Vector3d mounts[2]={{.40,0,-.11},{.30,-.12,.30}};
 gz::math::Quaterniond mountRot[2];bool ready[2]={false,false};
 gz::rendering::CameraPtr cameras[2],chase;MutualProvider *provider=nullptr;
 std::ofstream meta;std::string directory=".",status="Awaiting images and navigation";
 double sim=0,last=-1e9,rate=6,hfov[2]={24*Pi/180,20*Pi/180};int frameId=0;
 int W=1920,H=1080;
public:
 int FrameId()const{return frameId;}
signals:
 void FrameChanged();
public:
 void LoadConfig(const tinyxml2::XMLElement *e)override{
  if(auto x=e->FirstChildElement("directory"))directory=x->GetText();
  if(auto x=e->FirstChildElement("rate_hz"))rate=std::stod(x->GetText());
  if(auto x=e->FirstChildElement("image_width"))W=std::stoi(x->GetText());
  if(auto x=e->FirstChildElement("image_height"))H=std::stoi(x->GetText());
  if(auto x=e->FirstChildElement("air_hfov"))hfov[0]=std::stod(x->GetText());
  if(auto x=e->FirstChildElement("boat_hfov"))hfov[1]=std::stod(x->GetText());
  if(auto x=e->FirstChildElement("boat_mount")){double a,b,c;std::istringstream s(x->GetText());s>>a>>b>>c;mounts[1]={a,b,c};}
  for(auto name:{"air","boat","dashboard","chase"})std::filesystem::create_directories(directory+"/frames/"+name);
  meta.open(directory+"/sensor_frames.jsonl");meta<<std::setprecision(16);
  node.Subscribe("/airsea/aircraft/truth_pose",&AirSeaMutual::Truth,this);
  node.Subscribe("/airsea/aircraft/navigation_pose",&AirSeaMutual::Nav,this);
  node.Subscribe("/cooperative/boat/navigation_pose",&AirSeaMutual::BoatNav,this);
  node.Subscribe("/cooperative/aim/air",&AirSeaMutual::AimAir,this);
  node.Subscribe("/cooperative/aim/boat",&AirSeaMutual::AimBoat,this);
  node.Subscribe("/cooperative/status",&AirSeaMutual::Status,this);
  node.Subscribe("/airsea/capture/active",&AirSeaMutual::Active,this);
  provider=new MutualProvider;gz::gui::App()->Engine()->addImageProvider("mutual",provider);
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
 }
 void Truth(const gz::msgs::Pose &p){std::lock_guard<std::mutex> g(mutex);truth=p;haveTruth=true;}
 void Nav(const gz::msgs::Pose &p){std::lock_guard<std::mutex> g(mutex);nav=p;haveNav=true;}
 void BoatNav(const gz::msgs::Pose &p){std::lock_guard<std::mutex> g(mutex);bnav=p;haveBoatNav=true;}
 void AimAir(const gz::msgs::Pose &p){std::lock_guard<std::mutex> g(mutex);aims[0]=asPose(p).Pos();}
 void AimBoat(const gz::msgs::Pose &p){std::lock_guard<std::mutex> g(mutex);aims[1]=asPose(p).Pos();}
 void Status(const gz::msgs::StringMsg &s){std::lock_guard<std::mutex> g(mutex);status=s.data();}
 void Active(const gz::msgs::Boolean &b){std::lock_guard<std::mutex> g(mutex);active=b.data();}
 void Update(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &)override{std::lock_guard<std::mutex> g(mutex);sim=std::chrono::duration<double>(i.simTime).count();}
 bool eventFilter(QObject *o,QEvent *e)override{if(e->type()==gz::gui::events::Render::kType)Sample();return QObject::eventFilter(o,e);}
 void Sample(){
  std::lock_guard<std::mutex> guard(mutex);
  if(!active||!haveTruth||!haveNav||!haveBoatNav||sim-last<1/rate)return;
  auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
  auto boat=scene->VisualByName("blueboat"),plane=scene->VisualByName("aircraft");if(!boat||!plane)return;
  gz::math::Pose3d physical[2]={asPose(truth),boat->WorldPose()},navigation[2]={asPose(nav),asPose(bnav)};
  plane->SetWorldPose(physical[0]);
  if(!cameras[0]){
   for(int k=0;k<2;++k){auto c=scene->CreateCamera(k==0?"mutual_air_onboard":"mutual_boat_upward");c->SetImageWidth(W);c->SetImageHeight(H);c->SetHFOV(hfov[k]);c->SetNearClipPlane(.02);c->SetFarClipPlane(5000);scene->RootVisual()->AddChild(c);cameras[k]=c;}
   chase=scene->CreateCamera("mutual_chase");chase->SetImageWidth(W);chase->SetImageHeight(H);chase->SetHFOV(50*Pi/180);chase->SetNearClipPlane(.02);chase->SetFarClipPlane(5000);scene->RootVisual()->AddChild(chase);
  }
  cv::Mat images[2];
  std::ostringstream filename;filename<<std::setw(6)<<std::setfill('0')<<frameId<<".jpg";
  meta<<"{\"i\":"<<frameId<<",\"t\":"<<sim<<",\"air\":";
  for(int k=0;k<2;++k){
   auto navEye=navigation[k].Pos()+navigation[k].Rot()*mounts[k];
   auto direction=aims[k]-navEye;
   // Limit requests to the appropriate hemisphere; there is no target pose lookup.
   if(k==0)direction.Z()=std::min(-.1,direction.Z());else direction.Z()=std::max(.1,direction.Z());
   double horizontal=std::hypot(direction.X(),direction.Y());double limit=std::abs(direction.Z())*std::tan(80*Pi/180);
   if(horizontal>limit){direction.X()*=limit/horizontal;direction.Y()*=limit/horizontal;}direction.Normalize();
   auto left=gz::math::Vector3d(0,1,0).Cross(direction);if(left.Length()<.0001)left={1,0,0};left.Normalize();auto up=direction.Cross(left);
   gz::math::Matrix3d basis(direction.X(),left.X(),up.X(),direction.Y(),left.Y(),up.Y(),direction.Z(),left.Z(),up.Z());
   auto desired=navigation[k].Rot().Inverse()*gz::math::Quaterniond(basis);desired.Normalize();
   if(!ready[k]){mountRot[k]=desired;ready[k]=true;}
   else{double dot=std::abs(mountRot[k].Dot(desired));double angle=2*std::acos(std::clamp(dot,0.,1.));mountRot[k]=gz::math::Quaterniond::Slerp(std::min(1.,(sim-last)*150*Pi/180/std::max(angle,1e-9)),mountRot[k],desired,true);}
   auto rotation=navigation[k].Rot()*mountRot[k];
   cameras[k]->SetWorldPosition(physical[k].Pos()+physical[k].Rot()*mounts[k]);cameras[k]->SetWorldRotation(physical[k].Rot()*mountRot[k]);
   cameras[k]->Update();auto raw=cameras[k]->CreateImage();cameras[k]->Copy(raw);cv::Mat rgb(H,W,CV_8UC3,raw.Data<unsigned char>());cv::cvtColor(rgb,images[k],cv::COLOR_RGB2BGR);
   auto path=directory+"/frames/"+(k==0?"air/":"boat/")+filename.str();cv::imwrite(path,images[k],{cv::IMWRITE_JPEG_QUALITY,95});images[k]=cv::imread(path);
   if(k==1)meta<<",\"boat\":";
   meta<<"{\"image_width\":"<<W<<",\"image_height\":"<<H<<",\"hfov_rad\":"<<hfov[k]<<",\"mount_body_m\":";vectorJSON(meta,mounts[k]);
   meta<<",\"navigation_eye\":";vectorJSON(meta,navEye);meta<<",\"navigation_origin\":";vectorJSON(meta,navigation[k].Pos());
   meta<<",\"aim\":";vectorJSON(meta,aims[k]);meta<<",\"mount_quaternion\":["<<mountRot[k].W()<<','<<mountRot[k].X()<<','<<mountRot[k].Y()<<','<<mountRot[k].Z()<<']';
   auto targetAttitude=navigation[1-k].Rot();
   meta<<",\"communicated_target_attitude_enu\":["<<targetAttitude.W()<<','<<targetAttitude.X()<<','<<targetAttitude.Y()<<','<<targetAttitude.Z()<<']';
   // Optical right/down/forward in Gazebo camera forward/left/up coordinates.
   auto rx=rotation*gz::math::Vector3d(0,-1,0),ry=rotation*gz::math::Vector3d(0,0,-1),rz=rotation*gz::math::Vector3d(1,0,0);
   meta<<",\"R_optical_to_enu\":["<<rx.X()<<','<<ry.X()<<','<<rz.X()<<','<<rx.Y()<<','<<ry.Y()<<','<<rz.Y()<<','<<rx.Z()<<','<<ry.Z()<<','<<rz.Z()<<']';
   // Upward estimates now refer to the aircraft body origin. Wing-skin
   // marker offsets are removed using calibrated corners and shared IMU.
   auto target=physical[1-k].Pos()+physical[1-k].Rot()*(k==0?gz::math::Vector3d(0,0,.1576):gz::math::Vector3d(0,0,0));
   meta<<",\"evaluation_target_only\":";vectorJSON(meta,target);meta<<'}';
  }
  meta<<"}\n";meta.flush();
  auto pp=physical[0];gz::math::Quaterniond heading(0,0,pp.Rot().Yaw());auto eye=pp.Pos()+heading*gz::math::Vector3d(-4.5,-5.5,2.3);auto d=pp.Pos()-eye;
  chase->SetWorldPosition(eye);chase->SetWorldRotation({0,std::atan2(-d.Z(),std::hypot(d.X(),d.Y())),std::atan2(d.Y(),d.X())});chase->Update();auto raw=chase->CreateImage();chase->Copy(raw);cv::Mat rgb(H,W,CV_8UC3,raw.Data<unsigned char>()),bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);cv::imwrite(directory+"/frames/chase/"+filename.str(),bgr);
  cv::Mat screen(1080,1920,CV_8UC3,cv::Scalar(28,23,17));cv::resize(images[0],screen(cv::Rect(24,110,924,520)),{924,520});cv::resize(images[1],screen(cv::Rect(972,110,924,520)),{924,520});
  auto text=[&](const std::string &s,int x,int y,double size=.7){cv::putText(screen,s,{x,y},cv::FONT_HERSHEY_SIMPLEX,size,{230,235,238},1,cv::LINE_AA);};
  text("MUTUAL VISUAL OBSERVATION / CLASSICAL SPEED COORDINATION",24,45,1);text("AIRCRAFT -> ORIGINAL BLUEBOAT",24,90,.8);text("BOAT UPWARD CAMERA -> AIRCRAFT",972,90,.8);
  text("Both are native sensor views. Latest processed telemetry below has its own timestamp.",24,674,.6);
  std::istringstream lines(status);std::string line;int y=725;while(std::getline(lines,line)&&y<1040){text(line,30,y,.66);y+=38;}
  cv::imwrite(directory+"/frames/dashboard/"+filename.str(),screen,{cv::IMWRITE_JPEG_QUALITY,90});provider->Set(screen);++frameId;last=sim;emit FrameChanged();
 }
};
GZ_ADD_PLUGIN(AirSeaMutual,gz::gui::Plugin)
#include "mutual.moc"
