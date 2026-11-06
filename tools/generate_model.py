"""Generate auditable SDF and controller parameters from one physical config."""
from pathlib import Path
import argparse
import json
import math
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / 'h-spar-sim/ros2_ws/src/blueboat_description'

def element(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = str(value)
    return child

def save(tree, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)

def box(parent, name, size, pose, color=None, collision=False):
    item = element(parent, 'collision' if collision else 'visual', name=name)
    element(item, 'pose', ' '.join(map(str, pose)))
    element(element(element(item, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
    if color:
        mat = element(item, 'material')
        element(mat, 'ambient', color)
        element(mat, 'diffuse', color)
    return item

def plugin(parent, filename, name):
    return element(parent, 'plugin', filename=filename, name=name)

def inertia(link, mass, com, matrix):
    ins = element(link, 'inertial')
    element(ins, 'pose', ' '.join(map(str, [*com, 0, 0, 0])))
    element(ins, 'mass', mass)
    tensor = element(ins, 'inertia')
    for tag, i, j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        element(tensor, tag, float(matrix[i,j]))

def generate(config=DESC/'config/blueboat.json'):
    c = json.loads(Path(config).read_text(encoding='utf-8'))
    base_mass = c['dry_mass_kg'] + c['battery_mass_kg']
    payload = c['payload_mass_kg']
    mass = base_mass + payload
    if payload < 0 or c['battery_mass_kg'] < 0 or c['battery_mass_kg']+payload > 15:
        raise ValueError('Battery + payload must be in [0,15] kg (manufacturer limit).')
    if c['thruster_separation_m'] <= 0 or mass <= 0:
        raise ValueError('Mass and thruster separation must be positive.')
    com0, comp = np.array(c['base_com_m']), np.array(c['payload_center_m'])
    com = (base_mass*com0+payload*comp)/mass
    motor_pos = [np.array([c['thruster_x_m'], s*c['thruster_separation_m']/2,c['thruster_z_m']]) for s in [1,-1]]
    tensor = np.diag(c['base_inertia_kg_m2']) + np.diag(c['payload_inertia_kg_m2'])
    for m,p in [(base_mass,com0),(payload,comp)]:
        d=p-com
        tensor += m*((d@d)*np.eye(3)-np.outer(d,d))
    if np.linalg.eigvalsh(tensor).min() <= 0:
        raise ValueError('Invalid inertia: matrix must be positive definite.')
    rho=c['water_density_kg_m3']
    L,W,H=c['buoyancy_box_m']
    draft=mass/(rho*2*L*W)
    if draft >= H:
        raise ValueError('Payload exceeds equivalent hull buoyancy capacity.')
    max_thrust=c['total_forward_thrust_kgf']*c['gravity_mps2']/2
    sdf=ET.Element('sdf',version='1.9')
    model=element(sdf,'model',name=c['name'])
    element(model,'self_collide','false')
    body=element(model,'link',name='base_link')
    inertia(body,mass,com,tensor)
    for side,sign in [('port',1),('starboard',-1)]:
        box(body,side+'_buoyancy',[L,W,H],[0,sign*c.get('buoyancy_separation_m',.74)/2,0,0,0,0],collision=True)
    mesh=DESC/'models/blueboat/meshes/blueboat_body.stl'
    if mesh.exists():
        visual=element(body,'visual',name='official_cad')
        geom=element(visual,'geometry')
        element(element(geom,'mesh'),'uri','model://blueboat/meshes/blueboat_body.stl')
        mat=element(visual,'material')
        element(mat,'diffuse','0.08 0.32 0.68 1')
        element(mat,'ambient','0.05 0.2 0.4 1')
        for side,info in json.loads((mesh.parent/'rotors.json').read_text()).items():
            rotor=element(body,'visual',name='propeller_'+side)
            element(rotor,'pose',' '.join(map(str,[*info['center_m'],0,0,0])))
            element(element(element(rotor,'geometry'),'mesh'),'uri',f'model://blueboat/meshes/propeller_{side}.stl')
            mat=element(rotor,'material')
            element(mat,'ambient','0.65 0.42 0.07 1')
            element(mat,'diffuse','0.95 0.67 0.18 1')
            swept=element(body,'visual',name='propeller_sweep_'+side)
            pos=info['center_m'].copy();pos[0]=-.525
            element(swept,'pose',' '.join(map(str,[*pos,0,math.pi/2,0])))
            element(swept,'cast_shadows','false');element(swept,'transparency',1)
            cylinder=element(element(swept,'geometry'),'cylinder')
            element(cylinder,'radius',.055);element(cylinder,'length',.001)
            mat=element(swept,'material');element(mat,'ambient','.7 .48 .13 1');element(mat,'diffuse','.95 .67 .18 1')
    else:
        for side,sign in [('port',1),('starboard',-1)]:
            box(body,side+'_hull',[1.2,.19,.22],[0,sign*.37,0,0,0,0],'0.05 0.3 0.7 1')
        for x in [-.3,.3]:
            box(body,f'crossbar_{x}',[.025,.93,.025],[x,0,.12,0,0,0],'0.15 0.15 0.15 1')
    # Payload is physically represented through combined mass/inertia, plus a visible box.
    if payload:
        box(body,'provisional_payload',[.25,.3,.15],[*comp,0,0,0],'0.8 0.5 0.1 1')
    # A visual attached to this rigid body follows waves/yaw exactly, as in the
    # user's prior fixed-link AprilTag method. No fictitious landing dynamics.
    from landing_marker import attach
    attach(body)
    thr=plugin(model,'blueboat-thrusters-system','blueboat::Thrusters')
    element(thr,'link_name','base_link')
    element(thr,'max_thrust',max_thrust)
    element(thr,'reverse_ratio',c['reverse_thrust_ratio'])
    element(thr,'command_timeout',.5)
    element(thr,'center_of_mass',' '.join(map(str,com)))
    for side,pos in zip(['left','right'],motor_pos):
        element(thr,side+'_position',' '.join(map(str,pos)))
        element(thr,side+'_topic',f'/wamv/thrusters/{side}/thrust')
    hydro=plugin(model,'gz-sim-hydrodynamics-system','gz::sim::systems::Hydrodynamics')
    element(hydro,'link_name','base_link')
    element(hydro,'disable_added_mass','true')
    for tag,v in zip(['xU','yV','zW','kP','mQ','nR'],c['damping_linear']):
        element(hydro,tag,-v)
    for tag,v in zip(['xUabsU','yVabsV','zWabsW','kPabsP','mQabsQ','nRabsR'],c['damping_quadratic']):
        element(hydro,tag,-v)
    odom=plugin(model,'gz-sim-odometry-publisher-system','gz::sim::systems::OdometryPublisher')
    for k,v in {'odom_frame':'map','robot_base_frame':'wamv_simple','odom_publish_frequency':50,'dimensions':3,'odom_topic':'/blueboat/odometry'}.items():
        element(odom,k,v)
    save(sdf,DESC/'models/blueboat/model.sdf')
    meta=ET.Element('model')
    element(meta,'name','BlueBoat baseline'); element(meta,'version','0.1')
    element(meta,'sdf','model.sdf',version='1.9')
    element(meta,'description','Optional official CAD or procedural visual; estimated hydrostatic and hydrodynamic baseline. See BlueBoatLab/docs/PARAMETERS.md.')
    save(meta,DESC/'models/blueboat/model.config')
    # World uses exact submerged box volumes. Water surface is a visual, not a collision.
    worldroot=ET.Element('sdf',version='1.9')
    world=element(worldroot,'world',name='blueboat_water')
    element(world,'gravity',f'0 0 -{c["gravity_mps2"]}')
    physics=element(world,'physics',name='step_2ms',type='ignored')
    element(physics,'max_step_size',.002); element(physics,'real_time_factor',1)
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        plugin(world,f'gz-sim-{filename}-system',f'gz::sim::systems::{name}')
    buoy=plugin(world,'blueboat-buoyancy-system','gz::sim::systems::BlueBoatBuoyancy')
    graded=element(buoy,'graded_buoyancy'); element(graded,'default_density',rho)
    change=element(graded,'density_change'); element(change,'above_depth',0); element(change,'density',0)
    element(buoy,'enable','blueboat::base_link')
    sun=element(world,'light',name='sun',type='directional')
    element(sun,'pose','0 0 20 0 0 0'); element(sun,'diffuse','1 1 1 1'); element(sun,'direction','-0.3 0.3 -1')
    water=element(world,'include');element(water,'uri','model://blueboat_water_visual')
    include=element(world,'include'); element(include,'uri','model://blueboat')
    element(include,'pose',f'0 0 {H/2-draft:.8f} 0 0 0')
    scene=element(world,'scene'); element(scene,'ambient','.6 .6 .6 1'); element(scene,'background','.7 .8 .9 1')
    element(scene,'grid','false');element(scene,'sky')
    gui=element(world,'gui',fullscreen='false')
    view=element(gui,'plugin',filename='MinimalScene',name='3D View')
    properties=element(view,'gz-gui')
    element(properties,'property','false',key='showTitleBar',type='bool')
    element(properties,'property','docked',key='state',type='string')
    element(view,'engine','ogre2'); element(view,'scene','scene')
    element(view,'camera_pose','2 -2 1.5 0 0.36 2.35')
    for filename,name in [('GzSceneManager','Scene Manager'),('InteractiveViewControl','Interactive View Control'),('EntityContextMenuPlugin','Entity Context Menu')]:
        helper=element(gui,'plugin',filename=filename,name=name)
        props=element(helper,'gz-gui')
        for key,type_,value in [('state','string','floating'),('width','double','5'),('height','double','5'),('showTitleBar','bool','false'),('resizable','bool','false')]:
            element(props,'property',value,key=key,type=type_)
    plugin(gui,'BlueBoatVisuals','BlueBoatVisuals')
    save(worldroot,DESC/'worlds/blueboat_water.sdf')
    ctrl={'max_thrust':max_thrust,'reverse_thrust_ratio':c['reverse_thrust_ratio'],
          'thruster_separation':c['thruster_separation_m'],'max_speed':c['max_speed_mps'],
          'max_yaw_rate':.8,'surge_linear':c['damping_linear'][0],
          'surge_quadratic':c['damping_quadratic'][0], 'yaw_linear':c['damping_linear'][5],
          'yaw_quadratic':c['damping_quadratic'][5],'surge_kp':22.,'yaw_kp':8.,
          'command_timeout':.5,'feedback_timeout':.5,'thrust_deadband':.02}
    (DESC/'config/controller.json').write_text(json.dumps(ctrl,indent=2)+'\n',encoding='utf-8')
    derived={'total_mass_kg':mass,'center_of_mass_m':com.tolist(),'inertia_kg_m2':tensor.tolist(),
             'per_motor_thrust_N':max_thrust,'equivalent_hull_draft_m':draft,
             'initial_model_z_m':H/2-draft,'payload_budget_remaining_kg':15-c['battery_mass_kg']-payload,
             'visual_mesh':'official CAD' if mesh.exists() else 'primitive fallback'}
    (DESC/'config/derived.json').write_text(json.dumps(derived,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(derived,indent=2))
    return derived

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--config',type=Path,default=DESC/'config/blueboat.json')
    generate(parser.parse_args().config)
