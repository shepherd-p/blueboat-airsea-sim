"""Inspect the selected original boat inside Gazebo before recording a mission."""
from pathlib import Path
import argparse,os,signal,subprocess,time,xml.etree.ElementTree as ET
from world import prepare,ROOT

def run(out):
    out=out.resolve();out.mkdir(parents=True,exist_ok=False);path=prepare(out,1)
    tree=ET.parse(path);world=tree.find('world')
    for model in list(world.findall('model')):
        if model.get('name')=='aircraft':world.remove(model);continue
        static=model.find('static')
        if static is None:static=ET.SubElement(model,'static')
        static.text='true'
        for plugin in list(model.findall('plugin')):model.remove(plugin)
    for plugin in list(world.findall('plugin')):
        if plugin.get('filename') not in ('gz-sim-physics-system','gz-sim-user-commands-system','gz-sim-scene-broadcaster-system'):world.remove(plugin)
    gui=world.find('gui')
    for plugin in list(gui):
        if plugin.get('filename') not in ('MinimalScene','GzSceneManager'):gui.remove(plugin)
    p=ET.SubElement(gui,'plugin',filename='AirSeaAircraftPreview',name='Original Boat Inspection')
    ET.SubElement(p,'directory').text=str(out);ET.SubElement(p,'target_model').text='blueboat'
    tree.write(path,encoding='utf-8',xml_declaration=True)
    env=os.environ.copy();env['GZ_PARTITION']='airsea_original_boat_inspection'
    for name in ('GZ_GUI_PLUGIN_PATH','GZ_SIM_SYSTEM_PLUGIN_PATH'):env[name]=str(ROOT/'build/airsea_native')+':'+str(ROOT/'install/lib')+':'+env.get(name,'')
    env['GZ_SIM_RESOURCE_PATH']=str(ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/models')+':'+env.get('GZ_SIM_RESOURCE_PATH','')
    with (out/'gazebo.log').open('w') as log:
        p=subprocess.Popen(['gz','sim','-r','-v','2',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+180
            while not (out/'complete.txt').exists():
                if p.poll() is not None:raise RuntimeError('Boat preview exited; see gazebo.log')
                if time.monotonic()>deadline:raise TimeoutError('No boat preview; see gazebo.log')
                time.sleep(.2)
        finally:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    print(out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);run(p.parse_args().out)
