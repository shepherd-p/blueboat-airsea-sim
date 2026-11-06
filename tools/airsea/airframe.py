"""Apache-2.0 PX4 Believer visual adapted to the native ArduPlane body frame."""
from pathlib import Path
import copy
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'assets/aircraft/believer'
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}
sys.path.insert(0,str(ROOT/'tools'))
from generate_model import element


def vertices(tree):
    values=[]
    for inp in tree.findall('.//c:vertices/c:input',NS):
        if inp.get('semantic')!='POSITION':continue
        source=inp.get('source').lstrip('#')
        array=tree.find(f".//c:source[@id='{source}']/c:float_array",NS)
        points=list(map(float,array.text.split()))
        values.extend(zip(points[::3],points[1::3],points[2::3]))
    return values


def canonical_mesh(source,target,color):
    """Preserve vertex geometry, add valid materials and smooth curved surfaces."""
    original=ET.parse(source);uri=NS['c']
    def tag(name):return f'{{{uri}}}'+name
    def add(parent,kind,text=None,**attrs):
        node=ET.SubElement(parent,tag(kind),attrs)
        if text is not None:node.text=str(text)
        return node
    root=ET.Element(tag('COLLADA'),version='1.4.1')
    asset=copy.deepcopy(original.find('c:asset',NS));asset.find('c:up_axis',NS).text='Z_UP';root.append(asset)
    effects=add(root,'library_effects');effect=add(effects,'effect',id='paint_effect')
    shader=add(add(add(effect,'profile_COMMON'),'technique',sid='common'),'phong')
    for channel,value in (('ambient',color),('diffuse',color),('specular','.22 .25 .28 1')):
        add(add(shader,channel),'color',value)
    add(add(shader,'shininess'),'float','28')
    materials=add(root,'library_materials');add(add(materials,'material',id='paint_material'),'instance_effect',url='#paint_effect')
    library=add(root,'library_geometries');scenes=add(root,'library_visual_scenes');scene=add(scenes,'visual_scene',id='BelieverScene')
    for number,geometry in enumerate(original.findall('c:library_geometries/c:geometry',NS)):
        mesh=geometry.find('c:mesh',NS);position_input=mesh.find('c:vertices/c:input',NS)
        sid=position_input.get('source').lstrip('#')
        positions=np.fromstring(mesh.find(f"c:source[@id='{sid}']/c:float_array",NS).text,sep=' ').reshape(-1,3)
        faces=[]
        for primitive in list(mesh):
            kind=primitive.tag.split('}')[-1]
            if kind not in ('triangles','polylist'):continue
            inputs=primitive.findall('c:input',NS);stride=max(int(i.get('offset','0')) for i in inputs)+1
            offset=next(int(i.get('offset','0')) for i in inputs if i.get('semantic')=='VERTEX')
            indices=np.fromstring(primitive.findtext('c:p',namespaces=NS),sep=' ',dtype=int).reshape(-1,stride)[:,offset]
            counts=[3]*(len(indices)//3) if kind=='triangles' else list(map(int,primitive.findtext('c:vcount',namespaces=NS).split()))
            start=0
            for count in counts:
                polygon=indices[start:start+count];start+=count
                faces.extend((polygon[0],polygon[j],polygon[j+1]) for j in range(1,count-1))
        faces=np.asarray(faces,dtype=int);triangles=positions[faces]
        weighted=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        unit=weighted/np.maximum(np.linalg.norm(weighted,axis=1,keepdims=True),1e-15)
        incident={}
        for face,indices in enumerate(faces):
            for index in indices:incident.setdefault(tuple(np.round(positions[index],6)),set()).add(face)
        normals=[];limit=math.cos(math.radians(50))
        for face,indices in enumerate(faces):
            for index in indices:
                adjacent=np.array(sorted(incident[tuple(np.round(positions[index],6))]))
                selected=adjacent[(unit[adjacent]@unit[face])>limit]
                normal=weighted[selected].sum(axis=0);length=np.linalg.norm(normal)
                normals.append(normal/length if length>1e-15 else unit[face])
        normals=np.asarray(normals)
        gid=f'believer_geometry_{number}';output=add(add(library,'geometry',id=gid),'mesh')
        for name,values in (('positions',positions),('normals',normals)):
            sid=gid+'_'+name;data=add(output,'source',id=sid)
            add(data,'float_array',' '.join(f'{v:.8g}' for v in values.ravel()),id=sid+'_array',count=str(values.size))
            accessor=add(add(data,'technique_common'),'accessor',source='#'+sid+'_array',count=str(len(values)),stride='3')
            for axis in 'XYZ':add(accessor,'param',name=axis,type='float')
        add(add(output,'vertices',id=gid+'_vertices'),'input',semantic='POSITION',source='#'+gid+'_positions')
        primitive=add(output,'triangles',count=str(len(faces)),material='paint')
        add(primitive,'input',semantic='VERTEX',source='#'+gid+'_vertices',offset='0')
        add(primitive,'input',semantic='NORMAL',source='#'+gid+'_normals',offset='1')
        paired=np.column_stack((faces.ravel(),np.arange(faces.size)))
        add(primitive,'p',' '.join(map(str,paired.ravel())))
        instance=add(add(scene,'node',id=gid+'_node'),'instance_geometry',url='#'+gid)
        add(add(add(instance,'bind_material'),'technique_common'),'instance_material',symbol='paint',target='#paint_material')
    add(add(root,'scene'),'instance_visual_scene',url='#BelieverScene')
    ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)


def prepare_meshes(out,wingspan=1.6):
    """Keep authored Z-up vertex coordinates; correct the body file's Y_UP tag.

    Some MeshLab exports have no visual scene. Write an explicit identity
    scene for every geometry and omit the propeller file's unused camera/light.
    Original files and their license remain unchanged in assets/.
    """
    directory=out/'aircraft_meshes';directory.mkdir(parents=True,exist_ok=True)
    upstream=json.loads((ASSETS/'UPSTREAM.json').read_text())
    points=[];files={}
    uri=NS['c'];ET.register_namespace('',uri)
    for source in sorted((ASSETS/'upstream/meshes').glob('*.dae')):
        key='meshes/'+source.name
        assert hashlib.sha256(source.read_bytes()).hexdigest()==upstream['files'][key]['sha256'],source
        tree=ET.parse(source);root=tree.getroot()
        if source.name!='propeller_cw.dae':points.extend(vertices(tree))
        target=directory/source.name
        color='.075 .085 .095 1' if source.name=='propeller_cw.dae' else ('.86 .90 .94 1' if source.name=='believer_body.dae' else '.025 .30 .52 1')
        canonical_mesh(source,target,color)
        files[source.name]=hashlib.sha256(target.read_bytes()).hexdigest()
    low=[min(p[i] for p in points) for i in range(3)]
    high=[max(p[i] for p in points) for i in range(3)]
    scale=wingspan/(high[1]-low[1])
    metadata={'name':'PX4 Believer twin-motor V-tail fixed-wing visual',
        'source_repository':'https://github.com/'+upstream['repo'],
        'source_commit':upstream['commit'],'license':'Apache-2.0',
        'uniform_scale':scale,'length_m':(high[0]-low[0])*scale,'wingspan_m':wingspan,
        'visual_offset_z_m':.09,'body_mesh_min_z_m':low[2]*scale+.09,
        'camera_mount_z_m':-.035,'wheel_bottom_z_m':-.1,
        'mesh_sha256':files,
        'modifications':'Explicit identity visual scenes; Z-up metadata; smoothed normals with 50-degree crease threshold; complete white/blue material bindings; uniform scale; original tricycle gear and camera pod. Generic ArduPlane FDM retained; this is not a calibrated Believer aerodynamic model.'}
    (out/'aircraft_visual.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return directory,metadata


def aircraft(world,out,wingspan=1.6):
    directory,meta=prepare_meshes(out,wingspan);s=meta['uniform_scale'];height=.09
    model=element(world,'model',name='aircraft');element(model,'static','true')
    element(model,'pose','0 0 .1 0 0 0')
    link=element(model,'link',name='base_link')
    def visual(name,xyz,rpy=(0,0,0),color='.88 .92 .95 1'):
        v=element(link,'visual',name=name)
        element(v,'pose',' '.join(map(str,[*xyz,*rpy])))
        material=element(v,'material')
        element(material,'ambient',color);element(material,'diffuse',color)
        element(material,'specular','.28 .30 .32 1')
        return v
    def mesh(name,file,xyz,rpy,color):
        v=visual(name,xyz,rpy,color)
        shape=element(element(v,'geometry'),'mesh')
        element(shape,'uri',str(directory/file));element(shape,'scale',f'{s} {s} {s}')
        return v
    mesh('believer_body','believer_body.dae',(0,0,height),(0,0,math.pi),'.86 .90 .94 1')
    for side in ('left','right'):
        for part in ('aileron','taileron'):
            mesh(f'{part}_{side}',f'believer_{part}_{side}.dae',(0,0,height),(0,0,math.pi),'.025 .30 .52 1')
        y=(.25 if side=='left' else -.25)*s
        mesh('propeller_'+side,'propeller_cw.dae',(.05*s,y,.05*s+height),(0,math.pi/2,0),'.075 .085 .095 1')
        v=visual('propeller_blur_'+side,(.05*s,y,.05*s+height),(0,math.pi/2,0),'.17 .22 .26 1')
        element(v,'transparency','1')
        cyl=element(element(v,'geometry'),'cylinder');element(cyl,'radius',.12954*s);element(cyl,'length',.001)
    # Wheels meet the same z=0 runway at the native FDM's 0.1 m ground height.
    for i,(x,y) in enumerate(((-.12,.235),(-.12,-.235),(.18,0))):
        strut=visual(f'gear_strut_{i}',(x,y,.015),color='.13 .16 .19 1')
        cyl=element(element(strut,'geometry'),'cylinder');element(cyl,'radius',.006);element(cyl,'length',.14)
        for name,radius,length,color in (('tire',.045,.026,'.04 .045 .05 1'),('hub',.019,.028,'.46 .52 .57 1')):
            v=visual(f'{name}_{i}',(x,y,-.055),(math.pi/2,0,0),color)
            cyl=element(element(v,'geometry'),'cylinder');element(cyl,'radius',radius);element(cyl,'length',length)
    pod=visual('camera_pod',(0,0,-.022),color='.11 .14 .17 1')
    shape=element(element(pod,'geometry'),'sphere');element(shape,'radius',.012)
    p=element(model,'plugin',filename='AirSeaAircraft',name='airsea::Aircraft')
    element(p,'pose_topic','/airsea/aircraft/truth_pose')
    return model
