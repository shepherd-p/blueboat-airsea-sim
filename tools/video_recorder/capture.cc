// Actual GUI camera capture, with the absolute simulation stamp of each frame.
// Rendering/encoding follows Gazebo's Apache-2.0 VideoRecorder implementation.
#include <fstream>
#include <mutex>
#include <iostream>
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

class BlueBoatVideoCapture : public gz::sim::GuiSystem {
  std::mutex mutex;
  std::chrono::steady_clock::duration simTime{0};
  bool active=false, opened=false;
  gz::transport::Node node;
  gz::rendering::CameraPtr camera;
  gz::rendering::CameraPtr stern;
  gz::rendering::Image frame;
  gz::rendering::Image detailFrame;
  gz::common::VideoEncoder encoder;
  gz::common::VideoEncoder detailEncoder;
  std::ofstream stamps;
  double origin=0;
  bool fixedTestCamera=false;
  bool markerDetail=false;
  std::string detailFile="BlueBoat_Propeller_Detail.mp4";
  bool previewSaved=false;
  gz::math::Vector3d fixedOrigin;
  std::ofstream cameraLog;
  std::string file, csv;
 public:
  void LoadConfig(const tinyxml2::XMLElement *p) override {
    file=p->FirstChildElement("filename")->GetText();
    csv=p->FirstChildElement("timestamps")->GetText();
    auto fixed=p->FirstChildElement("fixed_test_camera");
    fixedTestCamera=fixed && std::string(fixed->GetText())=="true";
    auto marker=p->FirstChildElement("marker_detail");
    markerDetail=marker && std::string(marker->GetText())=="true";
    if(markerDetail)detailFile="BlueBoat_Marker_Aerial.mp4";
    node.Advertise("/blueboat/video/start", &BlueBoatVideoCapture::Start, this);
    node.Advertise("/blueboat/video/stop", &BlueBoatVideoCapture::Stop, this);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }
  bool Start(const gz::msgs::Boolean &,gz::msgs::Boolean &res) {
    std::lock_guard<std::mutex> guard(mutex);active=true;res.set_data(true);return true;
  }
  bool Stop(const gz::msgs::Boolean &,gz::msgs::Boolean &res) {
    std::lock_guard<std::mutex> guard(mutex);active=false;res.set_data(true);return true;
  }
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    std::lock_guard<std::mutex> guard(mutex);simTime=info.simTime;
  }
  bool eventFilter(QObject *obj,QEvent *event) override {
    if(event->type()==gz::gui::events::Render::kType) Capture();
    return QObject::eventFilter(obj,event);
  }
  void Capture() {
    std::lock_guard<std::mutex> guard(mutex);
    if(!camera) {
      auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
      for(unsigned int i=0;i<scene->NodeCount();++i){
        auto c=std::dynamic_pointer_cast<gz::rendering::Camera>(scene->NodeByIndex(i));
        if(c&&c->HasUserData("user-camera")&&std::get<bool>(c->UserData("user-camera"))){camera=c;break;}
      }
      if(!camera)return;
    }
    if(fixedTestCamera && !previewSaved && std::chrono::duration<double>(simTime).count()>2){
      auto preview=camera->CreateImage();camera->Copy(preview);
      std::ofstream out("render_preview.ppm",std::ios::binary);
      out<<"P6\n"<<camera->ImageWidth()<<" "<<camera->ImageHeight()<<"\n255\n";
      out.write(reinterpret_cast<const char *>(preview.Data<unsigned char>()),camera->ImageWidth()*camera->ImageHeight()*3);
      previewSaved=true;
    }
    if(!active) {
      if(opened){
        const bool mainSaved=encoder.SaveToFile(file);
        const bool detailSaved=detailEncoder.SaveToFile(detailFile);
        if(!mainSaved || !detailSaved)
          std::cerr<<"Failed to preserve a recorded camera file\n";
        stamps.close();cameraLog.close();opened=false;
      }
      return;
    }
    auto w=camera->ImageWidth(), h=camera->ImageHeight();
    double sim=std::chrono::duration<double>(simTime).count();
    auto scene=camera->Scene();
    auto boat=scene->VisualByName("blueboat");
    auto water=scene->VisualByName("water_surface::link::visual");
    if(!water)water=scene->VisualByName("ocean_surface::link::visual");
    // The diagnostic inset omits the opaque surface, exposing submerged rotors.
    if(water)water->SetVisibilityFlags(2u);
    if(!stern){
      stern=scene->CreateCamera("blueboat_stern_detail");
      stern->SetImageWidth(640);stern->SetImageHeight(360);
      stern->SetHFOV(1.05);stern->SetNearClipPlane(.01);stern->SetFarClipPlane(200);
      stern->SetVisibilityMask(markerDetail?0xFFFFFFFFu:0xFFFFFFFDu);
      scene->RootVisual()->AddChild(stern);
    }
    if(boat){
      auto q=boat->WorldRotation();auto p=boat->WorldPosition();
      auto eye=p+q*gz::math::Vector3d(-1.05,-.78,-.025);
      auto target=p+q*gz::math::Vector3d(-.50,0,-.125);
      auto d=target-eye;
      stern->SetWorldPosition(eye);
      stern->SetWorldRotation(0,-std::atan2(d.Z(),std::hypot(d.X(),d.Y())),std::atan2(d.Y(),d.X()));
      if(markerDetail){
        // A second native camera, not a rendered marker pasted onto the scene.
        stern->SetWorldPosition(p+q*gz::math::Vector3d(0,0,.1576)+gz::math::Vector3d(0,0,2));
        stern->SetWorldRotation(0,M_PI/2,0);stern->SetHFOV(M_PI/3);
      }
      if(fixedTestCamera){
        if(!opened)fixedOrigin=gz::math::Vector3d(p.X(),p.Y(),0);
        // Two stationary tripods, with one hard cut at eight simulation seconds.
        // Neither camera receives boat attitude or follows its later position.
        bool wide=opened && sim-origin>=8.0;
        auto eye=fixedOrigin+(wide?gz::math::Vector3d(6,-10,4):gz::math::Vector3d(2,-5.5,1.8));
        auto target=fixedOrigin+(wide?gz::math::Vector3d(6,1,0):gz::math::Vector3d(2,.3,0));
        auto view=target-eye;
        camera->SetWorldPosition(eye);
        camera->SetWorldRotation(0,-std::atan2(view.Z(),std::hypot(view.X(),view.Y())),std::atan2(view.Y(),view.X()));
        camera->SetHFOV(1.15);
      }
    }
    if(!opened) {
      frame=camera->CreateImage();
      if(!encoder.Start("mp4",file+".recording.mp4",w,h,25,6000000))return;
      detailFrame=stern->CreateImage();
      if(!detailEncoder.Start("mp4",detailFile+".recording.mp4",640,360,25,2500000))return;
      stamps.open(csv);stamps<<"video_s,sim_s\n";stamps.precision(12);
      cameraLog.open("camera_pose.csv");cameraLog<<"sim_s,x,y,z,roll,pitch,yaw\n";cameraLog.precision(12);
      origin=sim;opened=true;
    }
    camera->Copy(frame);
    stern->Update();stern->Copy(detailFrame);
    detailEncoder.AddFrame(detailFrame.Data<unsigned char>(),640,360,
                          std::chrono::steady_clock::time_point(simTime));
    if(encoder.AddFrame(frame.Data<unsigned char>(),w,h,
                         std::chrono::steady_clock::time_point(simTime))){
      stamps<<sim-origin<<","<<sim<<"\n";stamps.flush();
      auto cp=camera->WorldPosition();auto cq=camera->WorldRotation().Euler();
      cameraLog<<sim<<","<<cp.X()<<","<<cp.Y()<<","<<cp.Z()<<","<<cq.X()<<","<<cq.Y()<<","<<cq.Z()<<"\n";
      cameraLog.flush();
    }
  }
};
GZ_ADD_PLUGIN(BlueBoatVideoCapture,gz::gui::Plugin)
