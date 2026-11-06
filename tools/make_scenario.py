"""Build an inspectable scenario using the same installed ocean plugins."""
from pathlib import Path
import argparse,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
DESC=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description'
def make(out,case='combined',sitl=False,rtf=1):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    modelname='blueboat_ocean_sitl' if sitl else 'blueboat_ocean'
    root=ET.parse(DESC/f'worlds/{modelname}.sdf').getroot();world=root.find('world');world.find('physics/real_time_factor').text=str(rtf)
    include=next(i for i in world.findall('include') if i.findtext('uri')=='model://'+modelname)
    model=ET.parse(DESC/f'models/{modelname}/model.sdf').getroot().find('model');model.set('name','blueboat');model.insert(0,include.find('pose'));world.remove(include);world.append(model)
    env=next(p for p in model.findall('plugin') if p.get('name')=='blueboat::Environment')
    ET.SubElement(env,'telemetry').text=str(out/'environment.csv')
    if sitl:ET.SubElement(next(p for p in model.findall('plugin') if p.get('name')=='blueboat::SITL'),'telemetry').text=str(out/'sitl_bridge.csv')
    waterinclude=next(i for i in world.findall('include') if i.findtext('uri')=='model://blueboat_ocean_visual')
    water=ET.parse(DESC/'models/blueboat_ocean_visual/model.sdf').getroot().find('model');world.remove(waterinclude);world.append(water)
    if case not in ['combined','waves']:
        for a in world.findall('.//waves/wave/amplitude'):a.text='0'
        for p in water.findall('.//param'):
            if p.findtext('name')=='amplitude':p.find('value').text='0 0 0'
    if case not in ['combined','wind']:env.find('wind').text='0 0 0'
    if case not in ['combined','current']:
        env.find('current').text='0 0 0';model.find("plugin[@name='gz::sim::systems::Hydrodynamics']/default_current").text='0 0 0'
    target=out/'world.sdf';ET.indent(root,space='  ');ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)
    return target
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--case',choices=['calm','waves','wind','current','combined'],default='combined');p.add_argument('--sitl',action='store_true');p.add_argument('--rtf',type=float,default=1)
    a=p.parse_args();print(make(a.out,a.case,a.sitl,a.rtf))
