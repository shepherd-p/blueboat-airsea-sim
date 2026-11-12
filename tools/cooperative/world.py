"""Original boat, wing-skin AprilTags and upward sensor in Sea State 1."""
from pathlib import Path
import copy,hashlib,json,sys,xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/airsea'))
from world import prepare as baseline_prepare
from generate_model import element,save
from generate_environment import dense_water
from skin_tags import apply as apply_skin_tags


def prepare(out,config,rtf):
    path=baseline_prepare(out,rtf,config=config);tree=ET.parse(path);world=tree.find('world')
    visual=json.loads((out/'boat_visual_manifest.json').read_text())
    if not visual['original_boat_restored']:raise RuntimeError('Cooperative demonstration requires the selected original boat; import it first')
    gui=world.find('gui');old=gui.find("plugin[@filename='AirSeaCapture']");gui.remove(old)
    p=element(gui,'plugin',filename='AirSeaMutual',name='Mutual Observation')
    element(p,'directory',str(out));element(p,'rate_hz',config['camera']['rate_hz'])
    assert all(config['camera'][n]==config['upward_camera'][n] for n in ('width','height'))
    for name,value in (('image_width',config['camera']['width']),('image_height',config['camera']['height']),
                       ('air_hfov',config['camera']['hfov_rad']),('boat_hfov',config['upward_camera']['hfov_rad'])):
        element(p,name,value)
    element(p,'boat_mount',' '.join(map(str,config['upward_camera']['mount_body_m'])))
    p.append(copy.deepcopy(old.find('gz-gui')))
    # Uniform sky makes this first classical silhouette baseline reproducible.
    # This is declared as a restricted appearance baseline, not generic vision.
    sky=world.find('scene/sky')
    if sky is not None:world.find('scene').remove(sky)
    world.find('scene/background').text='.35 .55 .76 1'
    dense_water(out/'water_near.dae',size=280,step=.5,center=(375,180))
    body=world.find("model[@name='aircraft']/link[@name='base_link']")
    apply_skin_tags(body,out,config)
    camera=element(body,'visual',name='cooperative_nose_gimbal');element(camera,'pose','.40 0 -.11 0 0 0')
    element(element(element(camera,'geometry'),'sphere'),'radius',.023)
    material=element(camera,'material');element(material,'ambient','.04 .05 .07 1');element(material,'diffuse','.04 .05 .07 1')
    # A small camera housing; camera image pose is exactly the same body mount.
    boat=world.find("model[@name='blueboat']/link[@name='base_link']")
    pod=element(boat,'visual',name='upward_camera_housing');element(pod,'pose',' '.join(map(str,[*config['upward_camera']['mount_body_m'],0,0,0])))
    element(element(element(pod,'geometry'),'sphere'),'radius',.024)
    material=element(pod,'material');element(material,'ambient','.04 .05 .07 1');element(material,'diffuse','.04 .05 .07 1')
    save(tree.getroot(),path)
    inputs=json.loads((out/'input_hashes.json').read_text());inputs['world_sdf_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    inputs['aircraft_skin_tags_sha256']=hashlib.sha256((out/'aircraft_skin_tags.json').read_bytes()).hexdigest()
    (out/'input_hashes.json').write_text(json.dumps(inputs,indent=2)+'\n')
    (out/'cooperative_config.json').write_text(json.dumps(config,indent=2)+'\n')
    return path
