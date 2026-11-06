"""Separate official CAD rotors and vendor the Gazebo water shader assets."""
from pathlib import Path
import concurrent.futures
import json
import urllib.request
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'h-spar-sim/ros2_ws/src/blueboat_description/models'

def rotors():
    folder = MODELS / 'blueboat/meshes'
    mesh = trimesh.load_mesh(folder/'blueboat_visual.stl')
    parts = mesh.split(only_watertight=False)
    keep = []
    found = {}
    for part in parts:
        lo, hi = part.bounds
        # Select the two connected, two-bladed propeller parts by CAD bounds.
        if (4000 < len(part.faces) < 6000 and -.56 < lo[0] < -.54
                and -.20 < lo[2] < -.18 and -.09 < hi[2] < -.07):
            side = 'left' if (lo[1]+hi[1]) > 0 else 'right'
            center = [-.5075, .2944 if side=='left' else -.2944, -.1355]
            found[side] = {'center_m':center, 'faces':len(part.faces),
                           'original_bounds_m':part.bounds.tolist()}
            part.apply_translation(-np.array(center))
            part.export(folder/f'propeller_{side}.stl')
        else:
            keep.append(part)
    assert set(found)=={'left','right'}, found
    trimesh.util.concatenate(keep).export(folder/'blueboat_body.stl')
    (folder/'rotors.json').write_text(json.dumps(found,indent=2)+'\n')
    print(found)

def water():
    dest = MODELS/'blueboat_water_visual'
    base='https://fuel.gazebosim.org/1.0/openrobotics/models/waves/4/files/'
    files=['model.config','model.sdf','meshes/mesh.dae',
           'materials/programs/GerstnerWaves_vs_330.glsl',
           'materials/programs/GerstnerWaves_fs_330.glsl',
           'materials/textures/wave_normals.dds','materials/textures/skybox_lowres.dds']
    def get(f):
        p=dest/f;p.parent.mkdir(parents=True,exist_ok=True)
        urllib.request.urlretrieve(base+f,p)
        return (f,p.stat().st_size)
    with concurrent.futures.ThreadPoolExecutor(7) as ex:
        for r in ex.map(get,files): print(r)
    tree=ET.parse(dest/'model.sdf')
    model=tree.getroot().find('model');model.set('name','water_surface')
    ET.SubElement(model,'pose').text='0 0 -0.005 0 0 0'
    pl=model.find('.//plugin')
    pl.set('filename','gz-sim-shader-param-system')
    pl.set('name','gz::sim::systems::ShaderParam')
    for s in pl.findall('shader'):
        if s.get('language')!='glsl':pl.remove(s)
    for p in pl.findall('param'):
        name=p.findtext('name')
        # Animated normal-map ripples on a level surface; no fake wave forces.
        values={'amplitude':'0 0 0','bumpScale':'50 50','bumpSpeed':'0.012 0.007',
                'shallowColor':'0.04 0.38 0.50 1', 'deepColor':'0.0 0.13 0.22 1',
                'hdrMultiplier':'0.7'}
        if name in values:p.find('value').text=values[name]
    ET.indent(tree,space='  ');tree.write(dest/'model.sdf',encoding='utf-8',xml_declaration=True)
    (dest/'UPSTREAM.txt').write_text(
        'OpenRobotics Waves model, version 4, Gazebo Fuel.\n'
        'https://app.gazebosim.org/OpenRobotics/fuel/models/Waves\n'
        'Source endpoint: '+base+'\n'
        'Modified for Gazebo Harmonic names and flat still-water geometry.\n'
        'Shaders / textures retain upstream authorship. See shader headers.\n')

if __name__=='__main__':
    rotors();water()
