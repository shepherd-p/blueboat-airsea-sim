// SPDX-License-Identifier: MIT
// Every output frame is freshly rendered at an explicit recorded trajectory time.
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <QProcess>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
constexpr double Pi=3.14159265358979323846;
constexpr unsigned W=1920,H=1080;
using Row=std::array<double,19>;
gz::rendering::VisualPtr find(gz::rendering::ScenePtr scene,std::string name){
 auto v=scene->VisualByName(name);if(v)return v;
 for(size_t i=0;(i=name.find("::",i))!=std::string::npos;)name.replace(i,2,"/");
 return scene->VisualByName(name);
}
gz::math::Pose3d pose(const Row &r,int offset){
 return {{r[offset],r[offset+1],r[offset+2]},{r[offset+3],r[offset+4],r[offset+5],r[offset+6]}};
}
}

class AirSeaReplay:public gz::sim::GuiSystem {
 std::string directory;std::vector<Row> rows;size_t index=0;unsigned settle=0;
 std::vector<gz::math::Quaterniond> mounts;
 bool both=false,opened=false,done=false;double phase=0,lastTime=-1;
 double onboardHfov=.8726646259971648;
 gz::rendering::CameraPtr camera;QProcess *mainEncoder=nullptr,*onboardEncoder=nullptr;
 std::ofstream stamps;
 bool start(QProcess *&p,const std::string &name){
  p=new QProcess; // Create the encoder in the rendering thread that owns its I/O.
  p->setStandardErrorFile(QString::fromStdString(directory+"/"+name+".ffmpeg.log"));
  p->setStandardOutputFile(QProcess::nullDevice());
  p->start("ffmpeg",QStringList{"-hide_banner","-loglevel","error","-y","-f","rawvideo","-pixel_format","rgb24",
    "-video_size","1920x1080","-framerate","30","-i","pipe:0","-an","-c:v","libx264",
    "-threads","4","-preset","veryfast","-crf","20","-pix_fmt","yuv420p","-movflags","+faststart",
    QString::fromStdString(directory+"/"+name+".mp4")});
  return p->waitForStarted(10000);
 }
 bool encode(QProcess *p,const unsigned char *data){
  if(p->state()!=QProcess::Running)return false;
  if(p->write(reinterpret_cast<const char *>(data),W*H*3)!=qint64(W*H*3))return false;
  while(p->bytesToWrite()>0)if(!p->waitForBytesWritten(30000))return false;
  return true;
 }
 void fail(const std::string &message){done=true;std::ofstream(directory+"/error.txt")<<message<<"\n";}
 bool finish(QProcess *p){p->closeWriteChannel();bool ok=p->waitForFinished(60000)&&p->exitStatus()==QProcess::NormalExit&&p->exitCode()==0;delete p;return ok;}
 public:
 void LoadConfig(const tinyxml2::XMLElement *e)override{
  directory=e->FirstChildElement("directory")->GetText();
  both=std::string(e->FirstChildElement("both_views")->GetText())=="true";
  if(auto x=e->FirstChildElement("onboard_hfov_rad"))onboardHfov=std::stod(x->GetText());
  std::ifstream file(directory+"/timeline.csv");std::string line;std::getline(file,line);
  while(std::getline(file,line)){
   std::stringstream in(line);std::string field;Row r{};int j=0;
   while(std::getline(in,field,',')&&j<19)r[j++]=std::stod(field);
   if(j==19)rows.push_back(r);
  }
  std::ifstream mountFile(directory+"/camera_mount.csv");std::getline(mountFile,line);
  while(std::getline(mountFile,line)){
   std::stringstream in(line);std::string field;std::array<double,4> q{};int j=0;
   while(std::getline(in,field,',')&&j<4)q[j++]=std::stod(field);
   if(j==4)mounts.emplace_back(q[0],q[1],q[2],q[3]);
  }
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
 }
 bool eventFilter(QObject *o,QEvent *e)override{
  if(e->type()==gz::gui::events::Render::kType)render();return QObject::eventFilter(o,e);
 }
 void render(){
  if(done||rows.empty())return;
  auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
  auto plane=find(scene,"aircraft"),boat=find(scene,"blueboat");if(!plane||!boat)return;
  auto water=find(scene,"ocean_surface::link::visual"),far=find(scene,"far_ocean::link::visual");
  if(!water||!far||!water->Material()||water->Material()->VertexShader().empty())return;
  if(++settle<12)return;
  if(!camera){
   camera=scene->CreateCamera("airsea_exact_frame_replay");camera->SetImageWidth(W);camera->SetImageHeight(H);
   camera->SetHFOV(.8726646259971648);camera->SetNearClipPlane(.03);camera->SetFarClipPlane(5000);
   scene->RootVisual()->AddChild(camera);
  }
  if(!opened){
   if(!start(mainEncoder,"Presentation_30fps")){fail("Cannot start main encoder");return;}
   if(both&&!start(onboardEncoder,"Onboard_30fps")){fail("Cannot start onboard encoder");return;}
   stamps.open(directory+"/rendered_frames.csv");stamps.precision(16);
   stamps<<"index,video_s,simulation_s,view,frame_signature\n";opened=true;
  }
  const auto &r=rows[index];double t=r[2];auto p=pose(r,4);plane->SetWorldPose(p);boat->SetWorldPose(pose(r,11));
  // Static shader t is overridden explicitly, independent of wall-clock speed.
  for(auto v:{water,far})if(v&&v->Material())(*v->Material()->VertexShaderParams())["t"]=static_cast<float>(t);
  double dt=lastTime<0||t<lastTime?1./30:t-lastTime;
  if(r[18]>.01)phase=std::fmod(phase+dt*(20+180*r[18]),2*Pi);
  for(const std::string side:{"left","right"}){
   auto prop=find(scene,"aircraft::base_link::propeller_"+side);
   if(prop)prop->SetLocalRotation(gz::math::Quaterniond(phase,0,0)*gz::math::Quaterniond(0,Pi/2,0));
   auto blur=find(scene,"aircraft::base_link::propeller_blur_"+side);
   if(blur&&blur->Material())blur->Material()->SetTransparency(r[18]>.05?.86:1.0);
  }
  auto capture=[&](int view,QProcess *encoder){
   camera->SetHFOV(view==1?onboardHfov:.8726646259971648);
   if(view==1){camera->SetWorldPosition(p.Pos()+p.Rot()*gz::math::Vector3d(0,0,-.035));camera->SetWorldRotation(p.Rot()*(mounts.size()==rows.size()?mounts[index]:gz::math::Quaterniond(0,Pi/2,0)));}
   else{
    gz::math::Quaterniond heading(0,0,p.Rot().Yaw());
    auto eye=p.Pos()+heading*gz::math::Vector3d(-4.5,-5.5,2.3);auto d=p.Pos()-eye;
    camera->SetWorldPosition(eye);camera->SetWorldRotation({0,std::atan2(-d.Z(),std::hypot(d.X(),d.Y())),std::atan2(d.Y(),d.X())});
   }
   camera->Update();auto image=camera->CreateImage();camera->Copy(image);auto data=image.Data<unsigned char>();
   cv::Mat rgb(H,W,CV_8UC3,data),small;cv::resize(rgb,small,{96,54});
   uint64_t hash=1469598103934665603ULL;for(size_t j=0;j<small.total()*3;++j){hash^=small.data[j];hash*=1099511628211ULL;}
   if(!encode(encoder,data))return false;
   if(view==int(r[3])||both){stamps<<index<<","<<r[1]<<","<<t<<","<<view<<","<<hash<<"\n";stamps.flush();}
   if(index==0||index==rows.size()/2){cv::Mat bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);cv::imwrite(directory+"/preview_"+std::to_string(index)+"_"+std::to_string(view)+".jpg",bgr);}
   return true;
  };
  if(!capture(both?0:int(r[3]),mainEncoder)){fail("Main video write failed");return;}
  if(both&&!capture(1,onboardEncoder)){fail("Onboard video write failed");return;}
  lastTime=t;++index;
  if(index%30==0||index==rows.size())std::ofstream(directory+"/progress.txt")<<index<<"\n";
  if(index==rows.size()){
   done=true;bool ok=finish(mainEncoder);if(both)ok=finish(onboardEncoder)&&ok;stamps.close();
   if(!ok){std::ofstream(directory+"/error.txt")<<"FFmpeg did not exit successfully\n";return;}
   std::ofstream(directory+"/complete.json")<<"{\"rendered_frames\":"<<index<<",\"fps\":30,\"both_views\":"<<(both?"true":"false")<<",\"status\":\"completed\"}\n";
  }
 }
};
GZ_ADD_PLUGIN(AirSeaReplay,gz::gui::Plugin)
