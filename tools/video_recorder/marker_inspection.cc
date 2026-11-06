// Native Gazebo rendering samples for marker visibility; no image compositing.
#include <fstream>
#include <mutex>
#include <cmath>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/gui/GuiSystem.hh>

class BlueBoatMarkerInspection : public gz::sim::GuiSystem {
  std::mutex mutex;
  double sim=0, last=-1;
  unsigned int index=0, settle=0;
  gz::rendering::CameraPtr camera;
  std::ofstream log;
 public:
  void LoadConfig(const tinyxml2::XMLElement *) override {
    log.open("aerial_camera_samples.csv");log.precision(12);
    log<<"index,sim_s,height_above_marker_m,oblique_deg,width,height,hfov_rad,camera_x,camera_y,camera_z,camera_roll,camera_pitch,camera_yaw,boat_x,boat_y,boat_z,boat_roll,boat_pitch,boat_yaw\n";
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    std::lock_guard<std::mutex> guard(mutex);sim=std::chrono::duration<double>(info.simTime).count();
  }
  bool eventFilter(QObject *obj,QEvent *event) override {
    if(event->type()==gz::gui::events::Render::kType) Sample();
    return QObject::eventFilter(obj,event);
  }
  void Sample(){
    std::lock_guard<std::mutex> guard(mutex);
    if(sim<4 || index>=7)return;
    auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
    auto boat=scene->VisualByName("blueboat");if(!boat)return;
    if(!camera){
      camera=scene->CreateCamera("blueboat_aerial_marker_inspection");
      camera->SetImageWidth(1280);camera->SetImageHeight(720);camera->SetHFOV(M_PI/3);
      camera->SetNearClipPlane(.05);camera->SetFarClipPlane(150);
      scene->RootVisual()->AddChild(camera);
    }
    const double heights[]={2,5,10,15,5,10,20};
    const double tilts[]={0,0,0,0,30,30,0};
    auto p=boat->WorldPosition();auto q=boat->WorldRotation();
    auto target=p+q*gz::math::Vector3d(0,0,.1576);
    double h=heights[index], tilt=tilts[index]*M_PI/180;
    auto eye=target+gz::math::Vector3d(-h*std::tan(tilt),0,h);
    camera->SetWorldPosition(eye);camera->SetWorldRotation(0,M_PI/2-tilt,0);
    camera->Update();
    if(++settle<4 || sim-last<.25)return;
    auto im=camera->CreateImage();camera->Copy(im);
    std::ofstream out("aerial_"+std::to_string(index)+".ppm",std::ios::binary);
    out<<"P6\n1280 720\n255\n";out.write(reinterpret_cast<const char *>(im.Data<unsigned char>()),1280*720*3);out.close();
    auto e=camera->WorldRotation().Euler(),b=q.Euler();
    log<<index<<","<<sim<<","<<h<<","<<tilts[index]<<",1280,720,"<<M_PI/3<<","<<eye.X()<<","<<eye.Y()<<","<<eye.Z()<<","<<e.X()<<","<<e.Y()<<","<<e.Z()<<","<<p.X()<<","<<p.Y()<<","<<p.Z()<<","<<b.X()<<","<<b.Y()<<","<<b.Z()<<"\n";log.flush();
    last=sim;settle=0;++index;
  }
};
GZ_ADD_PLUGIN(BlueBoatMarkerInspection,gz::gui::Plugin)
