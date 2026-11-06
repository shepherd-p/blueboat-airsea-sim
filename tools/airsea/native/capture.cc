// SPDX-License-Identifier: MIT
// Native rendered airborne camera. Estimation uses pixels, intrinsics,
// calibrated mount and MAVLink navigation pose. Boat truth is evaluation only.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QQuickImageProvider>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <gz/common/VideoEncoder.hh>
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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/double.pb.h>

namespace {
constexpr double Pi=3.14159265358979323846;
constexpr int W=1920,H=1080;
using Mat=cv::Matx33d;using Vec=cv::Vec3d;
double timestamp(const gz::msgs::Pose &p){return p.header().stamp().sec()+p.header().stamp().nsec()*1e-9;}
gz::math::Pose3d pose(const gz::msgs::Pose &p){auto x=p.position();auto q=p.orientation();return {{x.x(),x.y(),x.z()},{q.w(),q.x(),q.y(),q.z()}};}
Mat rotation(const gz::math::Quaterniond &q){auto x=q*gz::math::Vector3d(1,0,0),y=q*gz::math::Vector3d(0,1,0),z=q*gz::math::Vector3d(0,0,1);return {x.X(),y.X(),z.X(),x.Y(),y.Y(),z.Y(),x.Z(),y.Z(),z.Z()};}
Vec vec(const gz::math::Vector3d &p){return {p.X(),p.Y(),p.Z()};}
class Provider:public QQuickImageProvider{
 QImage image;std::mutex mutex;
public:
 Provider():QQuickImageProvider(QQuickImageProvider::Image){image=QImage(W,H,QImage::Format_RGB888);image.fill(QColor("#13232d"));}
 void Set(const cv::Mat &m){std::lock_guard<std::mutex> lock(mutex);image=QImage(m.data,m.cols,m.rows,m.step,QImage::Format_RGB888).copy();}
 QImage requestImage(const QString &,QSize *size,const QSize &)override{std::lock_guard<std::mutex> lock(mutex);if(size)*size=image.size();return image;}
};
}

class AirSeaCapture:public gz::sim::GuiSystem{
 Q_OBJECT
 Q_PROPERTY(int frameId READ FrameId NOTIFY FrameChanged)
 std::mutex mutex;gz::transport::Node node;gz::transport::Node::Publisher estimatePub;
 gz::msgs::Pose truth,nav;bool haveTruth=false,haveNav=false;
 gz::rendering::CameraPtr onboard,chase;Provider *provider=nullptr;
 gz::common::VideoEncoder chaseVideo,onboardVideo;std::ofstream csv;
 bool active=false,opened=false,closed=false;int frameId=0,index=0;double last=-1e9,sim=0,rate=12,throttle=0;
 std::string directory=".";double hfov=.8726646259971648,tagSize=.432;
 Mat K;
public:
 int FrameId()const{return frameId;}
signals:
 void FrameChanged();
public:
 void LoadConfig(const tinyxml2::XMLElement *e)override{
  if(auto x=e->FirstChildElement("directory"))directory=x->GetText();
  if(auto x=e->FirstChildElement("rate_hz"))rate=std::stod(x->GetText());
  std::filesystem::create_directories(directory+"/onboard_frames");
  node.Subscribe("/airsea/aircraft/truth_pose",&AirSeaCapture::OnTruth,this);
  node.Subscribe("/airsea/aircraft/navigation_pose",&AirSeaCapture::OnNav,this);
  node.Subscribe("/airsea/aircraft/throttle",&AirSeaCapture::OnThrottle,this);
  node.Subscribe("/airsea/capture/active",&AirSeaCapture::OnActive,this);
  estimatePub=node.Advertise<gz::msgs::Pose>("/airsea/boat/image_estimate");
  node.Advertise("/airsea/capture/start",&AirSeaCapture::Start,this);
  node.Advertise("/airsea/capture/stop",&AirSeaCapture::Stop,this);
  provider=new Provider;gz::gui::App()->Engine()->addImageProvider("airsea",provider);
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  double f=W/(2*std::tan(hfov/2));K=Mat(f,0,W/2.,0,f,H/2.,0,0,1);
  std::ofstream(directory+"/airborne_camera_calibration.json")<<std::setprecision(16)
   <<"{\"width\":"<<W<<",\"height\":"<<H<<",\"fx\":"<<f<<",\"fy\":"<<f<<",\"cx\":960,\"cy\":540,\"distortion\":[0,0,0,0,0],\"hfov_rad\":"<<hfov
   <<",\"black_border_m\":0.432,\"body_camera_xyz_m\":[0,0,-0.035],\"body_camera_rpy_rad\":[0,1.5707963267948966,0],\"body_frame\":\"forward,left,up\",\"optical_frame\":\"right,down,forward\",\"jpeg_quality\":95,\"truth_used_for_estimation\":false}\n";
 }
 void OnTruth(const gz::msgs::Pose &p){std::lock_guard<std::mutex> lock(mutex);truth=p;haveTruth=true;}
 void OnNav(const gz::msgs::Pose &p){std::lock_guard<std::mutex> lock(mutex);nav=p;haveNav=true;}
 void OnThrottle(const gz::msgs::Double &v){std::lock_guard<std::mutex> lock(mutex);throttle=v.data();}
 void OnActive(const gz::msgs::Boolean &v){std::lock_guard<std::mutex> lock(mutex);active=v.data();}
 bool Start(const gz::msgs::Boolean &,gz::msgs::Boolean &r){std::lock_guard<std::mutex> lock(mutex);active=true;r.set_data(true);return true;}
 bool Stop(const gz::msgs::Boolean &,gz::msgs::Boolean &r){std::lock_guard<std::mutex> lock(mutex);active=false;r.set_data(true);return true;}
 void Update(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &)override{std::lock_guard<std::mutex> lock(mutex);sim=std::chrono::duration<double>(i.simTime).count();}
 bool eventFilter(QObject *o,QEvent *e)override{if(e->type()==gz::gui::events::Render::kType)Capture();return QObject::eventFilter(o,e);}
 void Capture(){
  std::lock_guard<std::mutex> lock(mutex);
  if(!active){
   if(opened&&!closed){chaseVideo.SaveToFile(directory+"/FixedWing_Full_Mission_Clean.mp4");onboardVideo.SaveToFile(directory+"/Onboard_Camera_Clean.mp4");csv.close();closed=true;std::ofstream(directory+"/capture_closed.txt")<<index<<"\n";}
   return;
  }
  if(closed||!haveTruth||!haveNav||sim-last<1/rate)return;
  auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
  auto boat=scene->VisualByName("blueboat");auto plane=scene->VisualByName("aircraft");if(!boat||!plane)return;
  auto p=pose(truth),n=pose(nav);double nativeTime=timestamp(truth),navTime=timestamp(nav);
  if(!onboard){
   onboard=scene->CreateCamera("airsea_native_onboard");chase=scene->CreateCamera("airsea_native_chase");
   for(auto c:{onboard,chase}){c->SetImageWidth(W);c->SetImageHeight(H);c->SetHFOV(hfov);c->SetNearClipPlane(.03);c->SetFarClipPlane(5000);scene->RootVisual()->AddChild(c);}
  }
  // Same physical FDM pose drives both the rendered airframe and camera mount.
  plane->SetWorldPose(p);
  auto prop=scene->VisualByName("aircraft::base_link::propeller");
  if(!prop)prop=scene->VisualByName("aircraft/base_link/propeller");
  if(prop)prop->SetLocalRotation(gz::math::Quaterniond(sim*throttle*15,0,0));
  const gz::math::Vector3d mount(0,0,-.035);
  const gz::math::Quaterniond down(0,Pi/2,0);
  onboard->SetWorldPosition(p.Pos()+p.Rot()*mount);onboard->SetWorldRotation(p.Rot()*down);
  // Chase uses heading only so airframe bank and pitch are visible against horizon.
  gz::math::Quaterniond heading(0,0,p.Rot().Yaw());
  auto eye=p.Pos()+heading*gz::math::Vector3d(-7,-9,3.5);auto d=p.Pos()-eye;
  chase->SetWorldPosition(eye);chase->SetWorldRotation({0,std::atan2(-d.Z(),std::hypot(d.X(),d.Y())),std::atan2(d.Y(),d.X())});
  onboard->Update();auto raw=onboard->CreateImage();onboard->Copy(raw);
  cv::Mat rgb(H,W,CV_8UC3,raw.Data<unsigned char>()),bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
  std::ostringstream filename;filename<<std::setw(6)<<std::setfill('0')<<index<<".jpg";
  std::vector<uchar> jpeg;cv::imencode(".jpg",bgr,jpeg,{cv::IMWRITE_JPEG_QUALITY,95});
  std::ofstream image(directory+"/onboard_frames/"+filename.str(),std::ios::binary);image.write(reinterpret_cast<char*>(jpeg.data()),jpeg.size());image.close();
  // Estimate exactly the archived decoded sensor frame, independently reproducible.
  bgr=cv::imdecode(jpeg,cv::IMREAD_COLOR);cv::Mat gray;cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
  std::vector<std::vector<cv::Point2f>> corners;std::vector<int> ids;
  auto params=cv::aruco::DetectorParameters::create();params->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;params->cornerRefinementWinSize=3;
  cv::aruco::detectMarkers(gray,cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),corners,ids,params);
  auto found=std::find(ids.begin(),ids.end(),0);bool detected=false;Vec estimate(NAN,NAN,NAN);double rms=NAN,side=NAN;std::vector<cv::Point2f> selected;
  if(found!=ids.end()){
   selected=corners[found-ids.begin()];double s=tagSize/2;
   std::vector<cv::Point3d> object{{-s,s,0},{s,s,0},{s,-s,0},{-s,-s,0}};
   std::vector<cv::Mat> rv,tv;cv::solvePnPGeneric(object,selected,K,cv::Mat(),rv,tv,false,cv::SOLVEPNP_IPPE_SQUARE);
   double best=1e9;size_t chosen=0;
   for(size_t i=0;i<rv.size();++i){if(tv[i].at<double>(2)<=0)continue;std::vector<cv::Point2d> projected;cv::projectPoints(object,rv[i],tv[i],K,cv::Mat(),projected);double error=0;
    for(int j=0;j<4;++j){double delta=cv::norm(projected[j]-cv::Point2d(selected[j]));error+=delta*delta;}error=std::sqrt(error/4);if(error<best){best=error;chosen=i;}}
   if(best<2&&tv.size()){
    Mat C(0,-1,0,0,0,-1,1,0,0);Vec t(tv[chosen].at<double>(0),tv[chosen].at<double>(1),tv[chosen].at<double>(2));
    estimate=vec(n.Pos()+n.Rot()*mount)+rotation(n.Rot()*down)*C.t()*t;
    rms=best;side=0;for(int j=0;j<4;++j)side+=cv::norm(selected[j]-selected[(j+1)%4])/4;
    detected=true;gz::msgs::Pose msg;msg.set_name("image_only_tag0_enu");msg.mutable_header()->mutable_stamp()->CopyFrom(truth.header().stamp());
    msg.mutable_position()->set_x(estimate[0]);msg.mutable_position()->set_y(estimate[1]);msg.mutable_position()->set_z(estimate[2]);msg.mutable_orientation()->set_w(1);estimatePub.Publish(msg);
   }
  }
  if(!opened){
   if(!chaseVideo.Start("mp4",directory+"/chase.recording.mp4",W,H,25,8000000))return;
   if(!onboardVideo.Start("mp4",directory+"/onboard.recording.mp4",W,H,25,8000000))return;
   csv.open(directory+"/airborne_observations.csv");csv.precision(15);
   csv<<"index,gazebo_s,native_s,navigation_s,plane_x,plane_y,plane_z,nav_x,nav_y,nav_z,nav_qw,nav_qx,nav_qy,nav_qz,boat_x,boat_y,boat_z,detected,estimate_x,estimate_y,estimate_z,reprojection_rms_px,tag_side_px,position_error_m";
   for(int j=0;j<4;++j)csv<<",u"<<j<<",v"<<j;csv<<"\n";opened=true;
  }
  auto bp=boat->WorldPosition()+boat->WorldRotation()*gz::math::Vector3d(0,0,.1576);auto nq=n.Rot();
  csv<<index<<","<<sim<<","<<nativeTime<<","<<navTime<<","<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<","<<n.Pos().X()<<","<<n.Pos().Y()<<","<<n.Pos().Z()<<","<<nq.W()<<","<<nq.X()<<","<<nq.Y()<<","<<nq.Z()<<","<<bp.X()<<","<<bp.Y()<<","<<bp.Z()<<","<<detected<<","<<estimate[0]<<","<<estimate[1]<<","<<estimate[2]<<","<<rms<<","<<side<<","<<(detected?cv::norm(estimate-vec(bp)):NAN);
  for(int j=0;j<4;++j)csv<<","<<(selected.size()?selected[j].x:NAN)<<","<<(selected.size()?selected[j].y:NAN);csv<<"\n";csv.flush();
  chase->Update();auto view=chase->CreateImage();chase->Copy(view);cv::Mat chaseRgb(H,W,CV_8UC3,view.Data<unsigned char>());
  cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
  auto stamp=std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim)));
  chaseVideo.AddFrame(chaseRgb.data,W,H,stamp);onboardVideo.AddFrame(rgb.data,W,H,stamp);
  provider->Set(chaseRgb);++frameId;emit FrameChanged();++index;last=sim;
 }
};
GZ_ADD_PLUGIN(AirSeaCapture,gz::gui::Plugin)
#include "capture.moc"
