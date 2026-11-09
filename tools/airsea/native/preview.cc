// SPDX-License-Identifier: MIT
// Actual Gazebo model inspection views; no image compositing or invented geometry.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class AirSeaAircraftPreview:public gz::sim::GuiSystem {
  std::mutex mutex;double sim=0;unsigned int index=0,settle=0;
  std::string directory=".";gz::rendering::CameraPtr camera;
  std::string targetModel="aircraft";
 public:
  void LoadConfig(const tinyxml2::XMLElement *e)override{
    if(auto x=e->FirstChildElement("directory"))directory=x->GetText();
    if(auto x=e->FirstChildElement("target_model"))targetModel=x->GetText();
    std::filesystem::create_directories(directory);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }
  void Update(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &)override{
    std::lock_guard<std::mutex> guard(mutex);sim=std::chrono::duration<double>(i.simTime).count();
  }
  bool eventFilter(QObject *o,QEvent *e)override{
    if(e->type()==gz::gui::events::Render::kType)Sample();return QObject::eventFilter(o,e);
  }
  void Sample(){
    std::lock_guard<std::mutex> guard(mutex);if(sim<.5||index>=5)return;
    auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
    auto plane=scene->VisualByName(targetModel);if(!plane)return;
    if(!camera){camera=scene->CreateCamera("airsea_aircraft_inspection");
      camera->SetImageWidth(1920);camera->SetImageHeight(1080);camera->SetHFOV(.8726646259971648);
      camera->SetNearClipPlane(.02);camera->SetFarClipPlane(100);scene->RootVisual()->AddChild(camera);}
    const gz::math::Vector3d eyes[]={{1.6,-2.1,.95},{-.2,-2.6,.65},{-1.8,-2.1,1.1},{-.2,-.15,2.8},{1.8,1.8,.9}};
    const char *names[]={"01_front_quarter","02_side","03_rear_quarter","04_top","05_opposite_quarter"};
    auto target=plane->WorldPosition()+(targetModel=="blueboat"?gz::math::Vector3d(0,0,.04):gz::math::Vector3d(-.22,0,.12));
    auto eye=plane->WorldPosition()+eyes[index];auto d=target-eye;
    camera->SetWorldPosition(eye);camera->SetWorldRotation({0,std::atan2(-d.Z(),std::hypot(d.X(),d.Y())),std::atan2(d.Y(),d.X())});
    camera->Update();if(++settle<5)return;
    auto image=camera->CreateImage();camera->Copy(image);
    cv::Mat rgb(1080,1920,CV_8UC3,image.Data<unsigned char>()),bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    cv::imwrite(directory+"/"+names[index]+".png",bgr);settle=0;++index;
    if(index==5)std::ofstream(directory+"/complete.txt")<<"Five native Gazebo rendering views saved.\n";
  }
};
GZ_ADD_PLUGIN(AirSeaAircraftPreview,gz::gui::Plugin)
