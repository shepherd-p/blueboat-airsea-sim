// Fixed native Gazebo camera -> AprilTag pixels -> PnP -> live annotated view.
// The estimator never receives scene/model poses. Truth is used only afterwards.
#include <algorithm>
#include <array>
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

namespace {
constexpr double Pi=3.14159265358979323846;
constexpr int Width=1920, Height=1080;
constexpr int CanvasWidth=2400;
using Mat=cv::Matx33d;
using Vec=cv::Vec3d;
struct Estimate {
  bool detected=false;
  cv::Mat rvec,tvec;
  Mat rotation;
  Vec translation;
  std::vector<cv::Point2f> corners;
  double rms=0,secondRms=0,sidePixels=0;
};
std::vector<cv::Point3d> objectPoints(double size){
  double s=size/2;
  return {{-s,s,0},{s,s,0},{s,-s,0},{-s,-s,0}};
}
Estimate estimatePixels(const cv::Mat &rgb,const Mat &K,double size){
  Estimate out;
  cv::Mat gray;cv::cvtColor(rgb,gray,cv::COLOR_RGB2GRAY);
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  auto params=cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
  params->cornerRefinementWinSize=4;
  cv::aruco::detectMarkers(gray,cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),corners,ids,params);
  auto found=std::find(ids.begin(),ids.end(),0);if(found==ids.end())return out;
  out.corners=corners[found-ids.begin()];
  auto obj=objectPoints(size);
  std::vector<cv::Mat> rv,tv;
  cv::solvePnPGeneric(obj,out.corners,K,cv::Mat(),rv,tv,false,cv::SOLVEPNP_IPPE_SQUARE);
  std::vector<std::pair<double,size_t>> scores;
  for(size_t i=0;i<rv.size();++i){
    cv::Mat R;cv::Rodrigues(rv[i],R);
    bool inFront=true;
    for(auto p:obj){cv::Mat x=R*(cv::Mat_<double>(3,1)<<p.x,p.y,p.z)+tv[i];if(x.at<double>(2)<=0)inFront=false;}
    if(!inFront)continue;
    std::vector<cv::Point2d> projected;cv::projectPoints(obj,rv[i],tv[i],K,cv::Mat(),projected);
    double sum=0;for(size_t j=0;j<4;++j)sum+=cv::norm(projected[j]-cv::Point2d(out.corners[j]))*cv::norm(projected[j]-cv::Point2d(out.corners[j]));
    scores.emplace_back(std::sqrt(sum/4),i);
  }
  if(scores.empty())return out;
  std::sort(scores.begin(),scores.end());auto best=scores.front();
  out.rvec=rv[best.second];out.tvec=tv[best.second];out.rms=best.first;
  out.secondRms=scores.size()>1?scores[1].first:1e9;
  cv::Mat R;cv::Rodrigues(out.rvec,R);out.rotation=Mat(R);
  out.translation=Vec(out.tvec.at<double>(0),out.tvec.at<double>(1),out.tvec.at<double>(2));
  for(int i=0;i<4;++i)out.sidePixels+=cv::norm(out.corners[i]-out.corners[(i+1)%4])/4;
  out.detected=true;return out;
}
Mat rotation(const gz::math::Quaterniond &q){
  auto x=q*gz::math::Vector3d(1,0,0),y=q*gz::math::Vector3d(0,1,0),z=q*gz::math::Vector3d(0,0,1);
  return {x.X(),y.X(),z.X(),x.Y(),y.Y(),z.Y(),x.Z(),y.Z(),z.Z()};
}
Vec vec(const gz::math::Vector3d &p){return {p.X(),p.Y(),p.Z()};}
Vec euler(const Mat &r){
  double p=std::atan2(-r(2,0),std::hypot(r(0,0),r(1,0)));
  return {std::atan2(r(2,1),r(2,2))*180/Pi,p*180/Pi,std::atan2(r(1,0),r(0,0))*180/Pi};
}
std::string number(double v,int decimals=3){std::ostringstream s;s<<std::fixed<<std::setprecision(decimals)<<std::showpos<<v;return s.str();}
class FrameProvider:public QQuickImageProvider {
  QImage current;
  std::mutex mutex;
 public:
  FrameProvider():QQuickImageProvider(QQuickImageProvider::Image){current=QImage(CanvasWidth,Height,QImage::Format_RGB888);current.fill(QColor("#101c25"));}
  void Set(const cv::Mat &rgb){std::lock_guard<std::mutex> g(mutex);current=QImage(rgb.data,rgb.cols,rgb.rows,rgb.step,QImage::Format_RGB888).copy();}
  QImage requestImage(const QString &,QSize *size,const QSize &) override {std::lock_guard<std::mutex> g(mutex);if(size)*size=current.size();return current;}
};
}

class BlueBoatTagPose:public gz::sim::GuiSystem {
  Q_OBJECT
  Q_PROPERTY(int frameId READ FrameId NOTIFY FrameChanged)
  std::mutex mutex;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  gz::rendering::CameraPtr camera;
  FrameProvider *provider=nullptr;
  gz::common::VideoEncoder encoder,rawEncoder;
  std::ofstream log;
  std::chrono::steady_clock::duration simTime{0};
  bool active=false,opened=false,closed=false;
  int index=0,frameId=0,state=2;
  double last=-1,origin=0,hfov=Pi/3,size=.432,rate=15;
  gz::math::Vector3d eye{7.5,-1,5};
  gz::math::Quaterniond attitude{0,std::atan2(5.,2.5),Pi/2};
  std::string directory=".",video="BlueBoat_Fixed_Camera_6DOF.mp4";
  Mat K;
 public:
  int FrameId() const{return frameId;}
 signals:
  void FrameChanged();
 public:
  void LoadConfig(const tinyxml2::XMLElement *p) override {
    if(auto x=p->FirstChildElement("directory"))directory=x->GetText();
    if(auto x=p->FirstChildElement("sea_state"))state=std::stoi(x->GetText());
    std::filesystem::create_directories(directory+"/native_frames");
    double f=Width/(2*std::tan(hfov/2));K=Mat(f,0,Width/2.,0,f,Height/2.,0,0,1);
    provider=new FrameProvider;gz::gui::App()->Engine()->addImageProvider("blueboattag",provider);
    publisher=node.Advertise<gz::msgs::Pose>("/blueboat/tag_pose/estimated_optical");
    node.Advertise("/blueboat/tag_pose/start",&BlueBoatTagPose::Start,this);
    node.Advertise("/blueboat/tag_pose/stop",&BlueBoatTagPose::Stop,this);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
    std::ofstream meta(directory+"/camera_calibration.json");
    meta<<std::setprecision(16)<<"{\"width\":"<<Width<<",\"height\":"<<Height<<",\"hfov_rad\":"<<hfov<<",\"fx\":"<<f<<",\"fy\":"<<f<<",\"cx\":"<<Width/2.<<",\"cy\":"<<Height/2.<<",\"distortion\":[0,0,0,0,0],\"black_border_size_m\":"<<size<<",\"camera_world_xyz\":[7.5,-1,5],\"camera_world_rpy_rad\":[0,"<<std::atan2(5.,2.5)<<","<<Pi/2<<"],\"tag_frame\":\"x boat forward, y port, z up; origin at marker center\",\"optical_frame\":\"x image right, y image down, z forward\",\"rotation_convention\":\"Rz(yaw) Ry(pitch) Rx(roll), degrees in CSV/display\",\"estimator\":\"OpenCV AprilTag36h11 ID0, subpixel corners, IPPE_SQUARE; positive-depth candidate with minimum pixel reprojection RMS; no truth input or smoothing\"}\n";
  }
  bool Start(const gz::msgs::Boolean &,gz::msgs::Boolean &r){std::lock_guard<std::mutex> g(mutex);active=true;r.set_data(true);return true;}
  bool Stop(const gz::msgs::Boolean &,gz::msgs::Boolean &r){std::lock_guard<std::mutex> g(mutex);active=false;r.set_data(true);return true;}
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {std::lock_guard<std::mutex> g(mutex);simTime=info.simTime;}
  bool eventFilter(QObject *o,QEvent *e) override {if(e->type()==gz::gui::events::Render::kType)Capture();return QObject::eventFilter(o,e);}
  void Capture(){
    std::lock_guard<std::mutex> guard(mutex);
    double sim=std::chrono::duration<double>(simTime).count();
    if(!active){
      if(opened&&!closed){encoder.SaveToFile(directory+"/"+video);rawEncoder.SaveToFile(directory+"/native_camera.mp4");log.close();closed=true;std::ofstream(directory+"/capture_closed.txt")<<index<<" frames\n";}
      return;
    }
    if(closed||sim-last<1/rate)return;
    auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
    auto boat=scene->VisualByName("blueboat");if(!boat)return;
    if(!camera){
      camera=scene->CreateCamera("blueboat_fixed_6dof_camera");camera->SetImageWidth(Width);camera->SetImageHeight(Height);camera->SetHFOV(hfov);camera->SetNearClipPlane(.05);camera->SetFarClipPlane(180);
      scene->RootVisual()->AddChild(camera);camera->SetWorldPosition(eye);camera->SetWorldRotation(attitude);
    }
    // These extrinsics are constant: no target-tracking, recentering, zoom or cuts.
    camera->Update();auto frame=camera->CreateImage();camera->Copy(frame);
    cv::Mat rgb(Height,Width,CV_8UC3,frame.Data<unsigned char>());
    auto estimated=estimatePixels(rgb,K,size);
    cv::Mat display(Height,CanvasWidth,CV_8UC3,cv::Scalar(12,25,34));
    rgb.copyTo(display(cv::Rect(0,0,Width,Height)));
    if(!opened){
      origin=sim;
      if(!encoder.Start("mp4",directory+"/annotated.recording.mp4",CanvasWidth,Height,25,8000000))return;
      if(!rawEncoder.Start("mp4",directory+"/native.recording.mp4",Width,Height,25,8000000))return;
      log.open(directory+"/pose_samples.csv");log.precision(14);
      log<<"index,sim_s,relative_s,detected,reprojection_rms_px,second_rms_px,side_px,est_x,est_y,est_z,est_roll_deg,est_pitch_deg,est_yaw_deg,world_x,world_y,world_z,world_roll_deg,world_pitch_deg,world_yaw_deg,truth_x,truth_y,truth_z,translation_error_m,rotation_error_deg,camera_x,camera_y,camera_z,camera_roll,camera_pitch,camera_yaw,boat_x,boat_y,boat_z,boat_roll,boat_pitch,boat_yaw";
      for(int j=0;j<4;++j)log<<",corner"<<j<<"_u,corner"<<j<<"_v";
      log<<"\n";opened=true;
    }
    Vec crpy=euler(rotation(camera->WorldRotation()));
    auto bp=boat->WorldPosition();auto bq=boat->WorldRotation();auto brpy=bq.Euler();
    Mat Rb=rotation(bq),Rc=rotation(camera->WorldRotation());
    Mat C(0,-1,0,0,0,-1,1,0,0),Rcw=C*Rc.t();
    Vec tagWorld=vec(bp)+Rb*Vec(0,0,.1576);
    Vec truth=Rcw*(tagWorld-vec(eye));Mat truthR=Rcw*Rb;
    Vec rpy(0,0,0),world(0,0,0),worldRpy(0,0,0);double te=0,re=0;
    auto text=[&](int x,int y,const std::string &s,double scale=.75,cv::Scalar color=cv::Scalar(235,244,245),int thick=1){cv::putText(display,s,{x,y},cv::FONT_HERSHEY_SIMPLEX,scale,color,thick,cv::LINE_AA);};
    if(estimated.detected){
      rpy=euler(estimated.rotation);world=vec(eye)+Rcw.t()*estimated.translation;worldRpy=euler(Rcw.t()*estimated.rotation);
      te=cv::norm(estimated.translation-truth);Mat d=estimated.rotation*truthR.t();re=std::acos(std::clamp((cv::trace(d)-1)/2,-1.,1.))*180/Pi;
      for(int j=0;j<4;++j){cv::line(display,estimated.corners[j],estimated.corners[(j+1)%4],cv::Scalar(80,255,130),2,cv::LINE_AA);cv::circle(display,estimated.corners[j],4,cv::Scalar(255,210,50),-1);}
      cv::drawFrameAxes(display,K,cv::Mat(),estimated.rvec,estimated.tvec,.24,3);
      gz::msgs::Pose msg;auto stamp=msg.mutable_header()->mutable_stamp();stamp->set_sec(static_cast<int64_t>(sim));stamp->set_nsec(static_cast<int32_t>((sim-std::floor(sim))*1e9));
      msg.set_name("tag36h11_0_in_fixed_camera_optical");auto p=msg.mutable_position();p->set_x(estimated.translation[0]);p->set_y(estimated.translation[1]);p->set_z(estimated.translation[2]);
      auto R=estimated.rotation;gz::math::Matrix3d gR(R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2));gz::math::Quaterniond q(gR);
      auto o=msg.mutable_orientation();o->set_w(q.W());o->set_x(q.X());o->set_y(q.Y());o->set_z(q.Z());publisher.Publish(msg);
    }
    cv::rectangle(display,{0,0},{CanvasWidth,95},cv::Scalar(12,25,34),-1);
    text(24,39,"BlueBoat | Fixed-camera AprilTag 6DOF | Sea State "+std::to_string(state),1.1,cv::Scalar(240,249,250),2);
    text(24,77,"Actual rendered pixels -> AprilTag ID 0 -> image-only PnP | Fixed world camera at (7.5, -1, 5) m",.78);
    cv::rectangle(display,{1930,115},{2380,945},cv::Scalar(12,25,34),-1);
    text(1950,150,estimated.detected?"TAG 0 DETECTED":"TAG NOT DETECTED",.86,estimated.detected?cv::Scalar(100,245,175):cv::Scalar(255,120,100),2);
    text(1950,187,"Tag -> CAMERA optical",.78);
    text(1950,220,"x right | y down | z forward",.62);
    std::array<std::string,6> labels={"x       ","y       ","z       ","roll    ","pitch   ","yaw     "};
    for(int j=0;j<6;++j){std::string value=estimated.detected?number(j<3?estimated.translation[j]:rpy[j-3],j<3?3:2):"---";text(1950,269+j*48,labels[j]+value+(j<3?" m":" deg"),.82);}
    text(1950,586,"ZYX Euler, wrapped +/-180 deg",.62);
    text(1950,622,estimated.detected?"Reproj RMS: "+number(estimated.rms,2)+" px":"No pose published",.67);
    text(1950,655,"Tag black border: 0.432 m",.65);
    text(1950,707,"Truth: validation only",.76,cv::Scalar(126,208,235));
    text(1950,745,estimated.detected?"Position error: "+number(te*100,2)+" cm":"Position error: ---",.65);
    text(1950,780,estimated.detected?"Rotation error: "+number(re,2)+" deg":"Rotation error: ---",.65);
    text(1950,828,"Planar ambiguity remains possible",.58);
    text(1950,863,"Unfiltered per-frame estimate",.63);
    text(1950,909,"t = "+number(sim-origin,1)+" s",.8);
    cv::rectangle(display,{0,Height-110},{CanvasWidth,Height},cv::Scalar(12,25,34),-1);
    text(24,Height-74,"WORLD pose from the same estimate + fixed camera calibration:",.78,cv::Scalar(130,225,217));
    text(24,Height-37,estimated.detected?"xyz [m]: "+number(world[0])+", "+number(world[1])+", "+number(world[2])+"    roll / pitch / yaw [deg]: "+number(worldRpy[0],2)+" / "+number(worldRpy[1],2)+" / "+number(worldRpy[2],2):"No current image-derived pose. No stale values are displayed.",.86);
    text(24,Height-10,"Simulation camera; no UAV dynamics. No camera motion, tracking, zoom or cuts. Raw frames and synchronized truth logged separately.",.53,cv::Scalar(180,196,205));
    rawEncoder.AddFrame(rgb.data,Width,Height,std::chrono::steady_clock::time_point(simTime));
    if(encoder.AddFrame(display.data,CanvasWidth,Height,std::chrono::steady_clock::time_point(simTime))){
      std::ostringstream name;name<<directory<<"/native_frames/"<<std::setw(6)<<std::setfill('0')<<index<<".png";
      cv::Mat bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);cv::imwrite(name.str(),bgr);
      log<<index<<","<<sim<<","<<sim-origin<<","<<estimated.detected;
      if(estimated.detected){log<<","<<estimated.rms<<","<<estimated.secondRms<<","<<estimated.sidePixels;for(auto v:estimated.translation.val)log<<","<<v;for(auto v:rpy.val)log<<","<<v;for(auto v:world.val)log<<","<<v;for(auto v:worldRpy.val)log<<","<<v;}
      else{for(int j=0;j<15;++j)log<<",nan";}
      for(auto v:truth.val)log<<","<<v;
      log<<","<<(estimated.detected?te:NAN)<<","<<(estimated.detected?re:NAN)<<","<<eye.X()<<","<<eye.Y()<<","<<eye.Z()<<","<<crpy[0]*Pi/180<<","<<crpy[1]*Pi/180<<","<<crpy[2]*Pi/180<<","<<bp.X()<<","<<bp.Y()<<","<<bp.Z()<<","<<brpy.X()<<","<<brpy.Y()<<","<<brpy.Z();
      for(int j=0;j<4;++j)log<<","<<(estimated.detected?estimated.corners[j].x:NAN)<<","<<(estimated.detected?estimated.corners[j].y:NAN);
      log<<"\n";log.flush();++index;
    }
    provider->Set(display);++frameId;emit FrameChanged();last=sim;
  }
};
GZ_ADD_PLUGIN(BlueBoatTagPose,gz::gui::Plugin)
#include "tag_pose.moc"
