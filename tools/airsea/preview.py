"""Save five actual Gazebo model views without starting either autopilot."""
from pathlib import Path
import argparse
import copy
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from airframe import aircraft,ROOT
from world import solid
from generate_model import element,save


def run(out):
    out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    root=ET.Element('sdf',version='1.9');world=element(root,'world',name='aircraft_inspection')
    element(world,'gravity','0 0 -9.80665')
    physics=element(world,'physics',name='step_2ms',type='ignored');element(physics,'max_step_size',.002);element(physics,'real_time_factor',1)
    for file,name in (('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')):
        element(world,'plugin',filename='gz-sim-'+file+'-system',name='gz::sim::systems::'+name)
    scene=element(world,'scene');element(scene,'ambient','.65 .65 .65 1');element(scene,'background','.68 .77 .84 1');element(scene,'grid','false');element(scene,'shadows','true')
    sun=element(world,'light',name='sun',type='directional');element(sun,'pose','0 0 10 0 0 0');element(sun,'diffuse','1 .97 .93 1');element(sun,'direction','-.3 -.5 -1');element(sun,'cast_shadows','true')
    solid(world,'apron',[0,0,-.025],[40,40,.05],'.23 .26 .29 1')
    aircraft(world,out)
    original=ET.parse(ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/worlds/blueboat_ocean_sitl.sdf').find('world/gui')
    gui=copy.deepcopy(original)
    for plugin in list(gui):
        if plugin.get('filename') not in ('MinimalScene','GzSceneManager'):gui.remove(plugin)
    gui.find("plugin[@filename='MinimalScene']/camera_pose").text='1.6 -2.1 1.05 0 .3 2.3'
    plugin=element(gui,'plugin',filename='AirSeaAircraftPreview',name='Aircraft Preview');element(plugin,'directory',str(out))
    world.append(gui);path=out/'inspection.sdf';save(root,path)
    env=os.environ.copy();env['GZ_PARTITION']='airsea_aircraft_inspection'
    env['GZ_GUI_PLUGIN_PATH']=str(ROOT/'build/airsea_native')+':'+env.get('GZ_GUI_PLUGIN_PATH','')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT/'build/airsea_native')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    with (out/'gazebo.log').open('w') as log:
        process=subprocess.Popen(['gz','sim','-r','-v','2',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+120
            while not (out/'complete.txt').exists():
                if process.poll() is not None:raise RuntimeError('Gazebo preview exited; see gazebo.log')
                if time.monotonic()>deadline:raise TimeoutError('No model preview images; see gazebo.log')
                time.sleep(.2)
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);run(p.parse_args().out)
