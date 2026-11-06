"""Generate shared physical/visual waves and separate ocean/SITL models."""
from pathlib import Path
import copy,json,math,xml.etree.ElementTree as ET
from generate_model import element,plugin,box,save
ROOT=Path(__file__).resolve().parents[1]
DESC=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description'

def wave_tags(parent,c):
    waves=element(parent,'waves');element(waves,'ramp_seconds',c['wave_ramp_seconds'])
    for w in c['waves']:
        p=element(waves,'wave')
        for k,v in [('amplitude',w['amplitude_m']),('wavelength',w['wavelength_m']),('direction_deg',w['direction_deg']),('phase',w['phase_rad'])]:element(p,k,v)

def dense_water(path,size=120,step=.5,center=(0,0)):
    # Shader deformation needs vertices at a fraction of the shortest wavelength.
    n=round(size/step)+1
    ns='http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('',ns)
    r=ET.Element('COLLADA',xmlns=ns,version='1.4.1')
    asset=element(r,'asset');element(asset,'up_axis','Z_UP');element(asset,'unit',name='meter',meter='1')
    geom=element(element(r,'library_geometries'),'geometry',id='water');mesh=element(geom,'mesh')
    s=element(mesh,'source',id='pos');element(s,'float_array',' '.join(f'{i*step-size/2+center[0]:g} {j*step-size/2+center[1]:g} 0' for j in range(n) for i in range(n)),id='pos-array',count=str(n*n*3))
    ac=element(element(s,'technique_common'),'accessor',source='#pos-array',count=str(n*n),stride='3')
    for k in 'XYZ':element(ac,'param',name=k,type='float')
    s=element(mesh,'source',id='uv');element(s,'float_array',' '.join(f'{i*step:g} {j*step:g}' for j in range(n) for i in range(n)),id='uv-array',count=str(n*n*2))
    ac=element(element(s,'technique_common'),'accessor',source='#uv-array',count=str(n*n),stride='2')
    for k in ['S','T']:element(ac,'param',name=k,type='float')
    verts=element(mesh,'vertices',id='verts');element(verts,'input',semantic='POSITION',source='#pos')
    tris=element(mesh,'triangles',count=str((n-1)**2*2));element(tris,'input',semantic='VERTEX',source='#verts',offset='0')
    element(tris,'input',semantic='TEXCOORD',source='#uv',offset='1',set='0')
    element(tris,'p',' '.join(' '.join(f'{v} {v}' for v in [j*n+i,j*n+i+1,(j+1)*n+i+1,j*n+i,(j+1)*n+i+1,(j+1)*n+i]) for j in range(n-1) for i in range(n-1)))
    vs=element(element(r,'library_visual_scenes'),'visual_scene',id='Scene');node=element(vs,'node',id='surface');element(node,'instance_geometry',url='#water')
    element(element(r,'scene'),'instance_visual_scene',url='#Scene');save(r,path)

def generate():
    c=json.loads((DESC/'config/environment.json').read_text());physical=json.loads((DESC/'config/blueboat.json').read_text());d=json.loads((DESC/'config/derived.json').read_text())
    water=ET.parse(DESC/'models/blueboat_water_visual/model.sdf').getroot();water.find('model').set('name','ocean_surface')
    water.find('model/pose').text='0 0 0 0 0 0'
    water.find('.//geometry/mesh/uri').text='model://blueboat_ocean_visual/meshes/water.dae'
    sh=water.find('.//shader');sh.find('vertex').text='model://blueboat_ocean_visual/materials/programs/Ocean_vs.glsl';sh.find('fragment').text='model://blueboat_ocean_visual/materials/programs/Ocean_fs.glsl'
    values={'amplitude':[w['amplitude_m'] for w in c['waves']],'wavenumber':[2*math.pi/w['wavelength_m'] for w in c['waves']],
            'omega':[math.sqrt(9.80665*2*math.pi/w['wavelength_m']) for w in c['waves']],'steepness':[0,0,0],'tau':[c['wave_ramp_seconds']],
            'bumpScale':[.55,.55],'bumpSpeed':[.022,-.014],'rescale':[1.0],
            'shallowColor':[.055,.14,.16,1],'deepColor':[.018,.065,.085,1]}
    for i,w in enumerate(c['waves']):values['dir'+str(i)]=[math.cos(math.radians(w['direction_deg'])),math.sin(math.radians(w['direction_deg']))]
    for p in water.findall('.//param'):
        name=p.findtext('name')
        if name in ['cubeMap','hdrMultiplier','fresnelPower']:
            water.find('.//plugin').remove(p)
            continue
        if name in values:p.find('value').text=' '.join(map(str,values[name]))
        if name in ['cubeMap','bumpMap']:p.find('value').text='model://blueboat_water_visual/'+p.findtext('value')
    p=element(water.find('.//plugin'),'param');element(p,'shader','vertex');element(p,'name','phase');element(p,'type','float_array');element(p,'value',' '.join(str(w['phase_rad']) for w in c['waves']))
    save(water,DESC/'models/blueboat_ocean_visual/model.sdf')
    dense_water(DESC/'models/blueboat_ocean_visual/meshes/water.dae')
    for sitl in [False,True]:
        name='blueboat_ocean_sitl' if sitl else 'blueboat_ocean'
        modelroot=ET.parse(DESC/'models/blueboat/model.sdf').getroot();model=modelroot.find('model');model.set('name',name)
        body=model.find('link');L,W,H=physical['buoyancy_box_m'];strips=c['hull_strips_per_side']
        for col in list(body.findall('collision')):body.remove(col)
        for sign,side in [(1,'port'),(-1,'starboard')]:
            for i in range(strips):box(body,f'{side}_{i}',[L/strips,W,H],[-L/2+(i+.5)*L/strips,sign*physical['buoyancy_separation_m']/2,0,0,0,0],collision=True)
        hydro=next(p for p in model.findall('plugin') if 'Hydrodynamics' in p.attrib['name'])
        element(hydro,'namespace','blueboat');element(hydro,'default_current',' '.join(map(str,c['current_enu_mps'])))
        env=plugin(model,'blueboat-environment-system','blueboat::Environment');wave_tags(env,c)
        for key,value in [('current',c['current_enu_mps']),('wind',c['wind_enu_mps']),('drag_area_cd',c['wind_drag_area_cd_m2']),('wind_center',c['wind_center_m']),('center_of_mass',d['center_of_mass_m'])]:element(env,key,' '.join(map(str,value)))
        element(env,'gust_fraction',c['gust_fraction']);element(env,'gust_period',c['gust_period_s'])
        if sitl:
            p=plugin(model,'blueboat-sitl-system','blueboat::SITL');element(p,'port',9002);element(p,'left_servo',1);element(p,'right_servo',3);element(p,'max_thrust',d['per_motor_thrust_N']);element(p,'reverse_max',27.5566865)
        save(modelroot,DESC/f'models/{name}/model.sdf')
        root=ET.parse(DESC/'worlds/blueboat_water.sdf').getroot();world=root.find('world')
        buoy=next(p for p in world.findall('plugin') if 'BlueBoatBuoyancy' in p.attrib['name']);wave_tags(buoy,c)
        for include in world.findall('include'):
            uri=include.find('uri')
            if uri.text=='model://blueboat':uri.text='model://'+name;element(include,'name','blueboat')
            elif uri.text=='model://blueboat_water_visual':uri.text='model://blueboat_ocean_visual'
        save(root,DESC/f'worlds/{name}.sdf')
    print('Generated ocean and ArduRover worlds: shared linear waves, 24 buoyancy strips, gusts and current.')
if __name__=='__main__':generate()
