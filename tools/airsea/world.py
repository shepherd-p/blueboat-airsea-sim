"""PX4 Believer visual and coastal runway in the physical Sea State 1 world."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from generate_model import element, box, save
from generate_environment import dense_water
from sea_states import make_case
from airframe import aircraft

CONFIG=json.loads((ROOT/'config/airsea.json').read_text())


def solid(world,name,xyz,size,color):
    model=element(world,'model',name=name);element(model,'static','true')
    element(model,'pose',' '.join(map(str,[*xyz,0,0,0])))
    link=element(model,'link',name='base_link')
    visual=element(link,'visual',name='visual');geometry=element(visual,'geometry')
    element(element(geometry,'box'),'size',' '.join(map(str,size)))
    material=element(visual,'material')
    element(material,'ambient',color);element(material,'diffuse',color)
    return model


def prepare(out,rtf=.6):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    path=make_case(out,1,True,rtf,9062)
    tree=ET.parse(path);world=tree.getroot().find('world');world.set('name','airsea')
    boat=world.find("model[@name='blueboat']")
    boat.find('pose').text=' '.join(map(str,[*CONFIG['boat_initial_enu_m'],0,0,0]))
    ocean=world.find("model[@name='ocean_surface']")
    # Absolute ENU mesh coordinates keep CPU and GLSL wave phases aligned.
    dense_water(out/'water_near.dae',size=120,step=.25,center=(325,180))
    ocean.find('.//geometry/mesh/uri').text=str(out/'water_near.dae')
    far=copy.deepcopy(ocean);far.set('name','far_ocean');far.find('pose').text='0 0 -.08 0 0 0'
    dense_water(out/'water_far.dae',size=1800,step=12,center=(250,960))
    far.find('.//geometry/mesh/uri').text=str(out/'water_far.dae');world.append(far)
    # Keep land below the z=0 runway top to avoid coplanar depth fighting.
    solid(world,'shore',[200,-300,-.20],[1800,720,.32],'.34 .40 .26 1')
    solid(world,'coastal_sand',[200,52,-.07],[1800,16,.14],'.66 .61 .43 1')
    solid(world,'runway',[200,0,-.012],[600,18,.024],'.115 .13 .145 1')
    for y in (-8.3,8.3):solid(world,'runway_edge_'+str(y),[200,y,.004],[594,.16,.008],'.95 .94 .88 1')
    for i,x in enumerate(range(-80,490,28)):
        solid(world,f'centerline_{i}',[x,0,.004],[13,.30,.008],'.95 .94 .88 1')
    for end in (-88,488):
        for y in (-6,-4,-2,2,4,6):
            solid(world,f'threshold_{end}_{y}',[end,y,.005],[8,.65,.01],'.97 .96 .91 1')
    solid(world,'hangar',[30,-50,4],[24,20,8],'.60 .65 .65 1')
    solid(world,'hangar_front',[42.02,-50,3],[.05,16,6],'.20 .24 .27 1')
    solid(world,'windsock_pole',[20,33,2],[.08,.08,4],'.82 .82 .78 1')
    for i in range(6):solid(world,f'windsock_{i}',[20+i*.15,33,4],[.15,.24,.24],'1 .22 .08 1' if i%2==0 else '.98 .97 .93 1')
    aircraft(world,out,CONFIG['aircraft']['wingspan_m'])
    gui=world.find('gui')
    gui.find("plugin[@filename='MinimalScene']/camera_pose").text='-6 -8 3 0 .25 .9'
    p=element(gui,'plugin',filename='AirSeaCapture',name='Air Sea Native Camera')
    element(p,'directory',str(out));element(p,'rate_hz',CONFIG['camera']['rate_hz'])
    props=element(p,'gz-gui')
    for key,typ,val in (('showTitleBar','bool','true'),('state','string','floating'),('width','double','1280'),('height','double','720'),('x','double','0'),('y','double','0')):
        element(props,'property',val,key=key,type=typ)
    world.find('scene/ambient').text='.72 .72 .72 1'
    world.find('light/diffuse').text='1 .97 .93 1'
    world.find('light/direction').text='-.3 -.5 -1'
    save(tree.getroot(),path)
    (out/'mission_config.json').write_text(json.dumps(CONFIG,indent=2)+'\n')
    inputs=json.loads((out/'input_hashes.json').read_text())
    inputs['world_sdf_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    inputs['mission_config_sha256']=hashlib.sha256((out/'mission_config.json').read_bytes()).hexdigest()
    (out/'input_hashes.json').write_text(json.dumps(inputs,indent=2)+'\n')
    return path


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--rtf',type=float,default=.6)
    args=parser.parse_args();print(prepare(args.out,args.rtf))
