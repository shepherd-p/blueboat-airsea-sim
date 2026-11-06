// Render RPM published by the physical actuator's 16 V static bench mapping.
// A translucent swept disc represents fast rotation without strobing blades.
#include <array>
#include <cmath>
#include <mutex>
#include <fstream>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

class BlueBoatVisuals : public gz::sim::GuiSystem {
  std::mutex mutex;
  gz::transport::Node node;
  std::array<double,2> thrust{0,0}, received{-1,-1}, angle{0,0};
  std::array<double,2> rpm{0,0};
  std::array<gz::rendering::MaterialPtr,2> bladeMaterial, sweepMaterial;
  double sim=0, previous=0;
  std::ofstream debug;
 public:
  void LoadConfig(const tinyxml2::XMLElement *p) override {
    if(auto output=p->FirstChildElement("telemetry")){
      debug.open(output->GetText());debug<<"sim_s,left_N,right_N,left_angle_rad,right_angle_rad,left_found,right_found,left_rpm,right_rpm\n";
    }
    node.Subscribe("/blueboat/propulsion/left/applied_thrust", &BlueBoatVisuals::Left, this);
    node.Subscribe("/blueboat/propulsion/right/applied_thrust", &BlueBoatVisuals::Right, this);
    node.Subscribe("/blueboat/propulsion/left/rpm", &BlueBoatVisuals::LeftRpm, this);
    node.Subscribe("/blueboat/propulsion/right/rpm", &BlueBoatVisuals::RightRpm, this);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }
  void Left(const gz::msgs::Double &m){std::lock_guard<std::mutex> g(mutex);thrust[0]=m.data();received[0]=sim;}
  void Right(const gz::msgs::Double &m){std::lock_guard<std::mutex> g(mutex);thrust[1]=m.data();received[1]=sim;}
  void LeftRpm(const gz::msgs::Double &m){std::lock_guard<std::mutex> g(mutex);rpm[0]=m.data();received[0]=sim;}
  void RightRpm(const gz::msgs::Double &m){std::lock_guard<std::mutex> g(mutex);rpm[1]=m.data();received[1]=sim;}
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    std::lock_guard<std::mutex> g(mutex);
    sim=std::chrono::duration<double>(info.simTime).count();
    double dt=std::max(0.,sim-previous);previous=sim;
    for(int i=0;i<2;++i){
      double speed=(sim-received[i]>.5 || !std::isfinite(rpm[i]))?0:rpm[i];
      angle[i]=std::remainder(angle[i]+(i==0?1:-1)*speed*2*M_PI/60*dt,2*M_PI);
    }
  }
  bool eventFilter(QObject *obj,QEvent *event) override {
    if(event->type()==gz::gui::events::PreRender::kType){
      std::lock_guard<std::mutex> g(mutex);
      auto scene=gz::rendering::sceneFromFirstRenderEngine();
      bool found[2]={false,false};
      if(scene)for(int i=0;i<2;++i){
        auto v=scene->VisualByName(std::string("blueboat::base_link::propeller_")+(i==0?"left":"right"));
        found[i]=bool(v);
        const double blur=std::clamp((std::abs(rpm[i])-200)/400.,0.,1.);
        if(v){
          if(!bladeMaterial[i]){auto m=scene->CreateMaterial();m->SetAmbient(.65,.42,.07);m->SetDiffuse(.95,.67,.18);v->SetMaterial(m,false);bladeMaterial[i]=m;}
          v->SetLocalRotation(angle[i],0,0);v->SetVisible(blur<.99);bladeMaterial[i]->SetTransparency(.98*blur);
        }
        auto swept=scene->VisualByName(std::string("blueboat::base_link::propeller_sweep_")+(i==0?"left":"right"));
        if(swept){
          if(!sweepMaterial[i]){auto m=scene->CreateMaterial();m->SetAmbient(.7,.48,.13);m->SetDiffuse(.95,.67,.18);swept->SetMaterial(m,false);sweepMaterial[i]=m;}
          swept->SetVisible(blur>0);sweepMaterial[i]->SetTransparency(1-.45*blur);
        }
      }
      if(debug.is_open()){debug<<sim<<','<<thrust[0]<<','<<thrust[1]<<','<<angle[0]<<','<<angle[1]<<','<<found[0]<<','<<found[1]<<','<<rpm[0]<<','<<rpm[1]<<'\n';debug.flush();}
    }
    return QObject::eventFilter(obj,event);
  }
};
GZ_ADD_PLUGIN(BlueBoatVisuals,gz::gui::Plugin)
