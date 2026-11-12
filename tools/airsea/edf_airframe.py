"""Original MIT swept-wing UAV visual with an enclosed fan and no exposed rotor.

The geometry is original; native ArduPlane aerodynamic/propulsion FDM remains
generic and is not calibrated to an EDF aircraft. Body frame: forward,left,up.
"""
from pathlib import Path
import hashlib,json,math
import xml.etree.ElementTree as ET
import numpy as np
from generate_model import element

NS='http://www.collada.org/2005/11/COLLADASchema'

def mesh_file(path,points,faces,uv=None,texture=None):
    points=np.asarray(points,dtype=float);faces=np.asarray(faces,dtype=int)
    normals=np.zeros_like(points)
    for face in faces:
        a,b,c=points[face];n=np.cross(b-a,c-a)
        normals[face]+=n
    normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
    ET.register_namespace('',NS)
    def add(parent,kind,text=None,**attrs):
        node=ET.SubElement(parent,'{'+NS+'}'+kind,attrs)
        if text is not None:node.text=str(text)
        return node
    root=ET.Element('{'+NS+'}COLLADA',version='1.4.1')
    asset=add(root,'asset');add(asset,'created','2026-11-08T00:00:00Z');add(asset,'modified','2026-11-08T00:00:00Z');add(asset,'unit',name='meter',meter='1');add(asset,'up_axis','Z_UP')
    if texture is not None:
        add(add(add(root,'library_images'),'image',id='skin-image'),'init_from',str(texture))
        profile=add(add(add(root,'library_effects'),'effect',id='skin-effect'),'profile_COMMON')
        add(add(add(profile,'newparam',sid='skin-surface'),'surface',type='2D'),'init_from','skin-image')
        add(add(add(profile,'newparam',sid='skin-sampler'),'sampler2D'),'source','skin-surface')
        shader=add(add(profile,'technique',sid='common'),'phong')
        add(add(shader,'diffuse'),'texture',texture='skin-sampler',texcoord='UVMap')
        add(add(shader,'ambient'),'color','1 1 1 1');add(add(shader,'specular'),'color','0 0 0 1')
        add(add(add(root,'library_materials'),'material',id='skin-material'),'instance_effect',url='#skin-effect')
    geometry=add(add(root,'library_geometries'),'geometry',id='surface');mesh=add(geometry,'mesh')
    for name,values in (('positions',points),('normals',normals)):
        source=add(mesh,'source',id=name)
        add(source,'float_array',' '.join(f'{v:.8g}' for v in values.ravel()),id=name+'_data',count=str(values.size))
        accessor=add(add(source,'technique_common'),'accessor',source='#'+name+'_data',count=str(len(values)),stride='3')
        for axis in 'XYZ':add(accessor,'param',name=axis,type='float')
    add(add(mesh,'vertices',id='vertices'),'input',semantic='POSITION',source='#positions')
    if uv is not None:
        uv=np.asarray(uv,dtype=float).reshape(-1,2)
        source=add(mesh,'source',id='uv')
        add(source,'float_array',' '.join(f'{v:.8g}' for v in uv.ravel()),id='uv_data',count=str(uv.size))
        accessor=add(add(source,'technique_common'),'accessor',source='#uv_data',count=str(len(uv)),stride='2')
        for axis in 'ST':add(accessor,'param',name=axis,type='float')
    triangles=add(mesh,'triangles',count=str(len(faces)),**({'material':'skin-material'} if texture else {}))
    add(triangles,'input',semantic='VERTEX',source='#vertices',offset='0')
    add(triangles,'input',semantic='NORMAL',source='#normals',offset='1')
    indices=np.repeat(faces.ravel()[:,None],2,axis=1)
    if uv is not None:
        add(triangles,'input',semantic='TEXCOORD',source='#uv',offset='2',set='0')
        indices=np.column_stack((indices,np.arange(faces.size)))
    add(triangles,'p',' '.join(str(int(v)) for v in indices.ravel()))
    scene=add(add(root,'library_visual_scenes'),'visual_scene',id='scene')
    instance=add(add(scene,'node',id='part'),'instance_geometry',url='#surface')
    if texture is not None:
        material=add(add(add(instance,'bind_material'),'technique_common'),'instance_material',symbol='skin-material',target='#skin-material')
        add(material,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set='0')
    add(add(root,'scene'),'instance_visual_scene',url='#scene')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def loft(path,sections,angles=64):
    points=[];faces=[]
    for x,ry,rz,zc in sections:
        for j in range(angles):
            a=2*math.pi*j/angles;points.append((x,ry*math.cos(a),zc+rz*math.sin(a)))
    for i in range(len(sections)-1):
        for j in range(angles):
            a=i*angles+j;b=i*angles+(j+1)%angles;c=b+angles;d=a+angles
            faces.extend(((a,c,b),(a,d,c)))
    for ring,reverse in ((0,False),(len(sections)-1,True)):
        center=len(points);x,_,_,z=sections[ring];points.append((x,0,z))
        for j in range(angles):
            a=ring*angles+j;b=ring*angles+(j+1)%angles
            faces.append((center,b,a) if reverse else (center,a,b))
    mesh_file(path,points,faces)

def wing(path,side=1,tail=False,accent=False):
    points=[];faces=[];count=48
    stations=np.linspace(.0,1,18)
    for fraction in stations:
        if tail:
            span=.28;root=.25;tip=.11;sweep=.17;leading=-.25;z=.085+fraction*.24
            y=side*(.045+fraction*span);thickness=.075
        else:
            span=.73;root=.45;tip=.15;sweep=.31;leading=.19;z=.055+fraction*.025
            y=side*(.07+fraction*span);thickness=.10
        chord=root+(tip-root)*fraction;xle=leading-sweep*fraction
        # Closed cosine-spaced NACA thickness surface; camber kept small.
        contour=[(u,1) for u in .5*(1-np.cos(np.linspace(0,math.pi,count)))]
        contour += [(u,-1) for u in .5*(1-np.cos(np.linspace(math.pi,0,count)))[1:-1]]
        for u,sign in contour:
            yt=5*thickness*(.2969*math.sqrt(max(u,0))-.126*u-.3516*u*u+.2843*u**3-.1036*u**4)
            camber=.012*math.sin(math.pi*u)
            points.append((xle-chord*u,y,z+chord*(sign*yt+camber)))
    m=len(contour)
    for i in range(len(stations)-1):
        for j in range(m):
            a=i*m+j;b=i*m+(j+1)%m;c=b+m;d=a+m
            # Outward winding: the upper skin must face upward. Previously
            # inward normals culled the near skin, exposing the far surface.
            faces.extend(((a,c,b),(a,d,c)) if side>0 else ((a,b,c),(a,c,d)))
    for ring in (0,len(stations)-1):
        ids=list(range(ring*m,(ring+1)*m));center=len(points);points.append(np.mean(np.array(points)[ids],axis=0))
        reverse=(ring==0)==(side>0)
        for j in range(m):
            a,b=ids[j],ids[(j+1)%m]
            faces.append((center,b,a) if reverse else (center,a,b))
    mesh_file(path,points,faces)

def aircraft(world,out,wingspan=1.6):
    directory=out/'aircraft_meshes';directory.mkdir(parents=True,exist_ok=True)
    body=[(.57,.002,.002,.06),(.55,.021,.019,.06),(.50,.047,.042,.06),(.40,.072,.063,.065),(.27,.092,.078,.07),(.1,.10,.083,.075),(-.1,.096,.080,.08),(-.30,.079,.067,.085),(-.43,.060,.051,.086),(-.50,.045,.039,.087),(-.55,.037,.033,.088)]
    loft(directory/'edf_fuselage.dae',body)
    loft(directory/'edf_canopy.dae',[(.39,.001,.001,.116),(.35,.029,.025,.126),(.27,.052,.041,.139),(.13,.058,.044,.145),(.025,.048,.032,.148),(-.02,.002,.002,.14)],48)
    loft(directory/'edf_exhaust.dae',[(-.535,.038,.034,.088),(-.566,.039,.035,.088),(-.575,.034,.030,.088)],48)
    loft(directory/'edf_nozzle_dark.dae',[(-.576,.030,.027,.088),(-.578,.030,.027,.088)],48)
    for side,sign in (('left',1),('right',-1)):
        wing(directory/f'edf_wing_{side}.dae',sign)
        wing(directory/f'edf_tail_{side}.dae',sign,tail=True)
    model=element(world,'model',name='aircraft');element(model,'static','true');element(model,'pose','0 0 .1 0 0 0')
    link=element(model,'link',name='base_link');scale=wingspan/1.6
    def visual(name,color,xyz=(0,0,0),rpy=(0,0,0)):
        v=element(link,'visual',name=name);element(v,'pose',' '.join(map(str,[*xyz,*rpy])))
        mat=element(v,'material');element(mat,'ambient',color);element(mat,'diffuse',color);element(mat,'specular','.32 .36 .4 1')
        return v
    for name,color in [('fuselage','.77 .83 .87 1'),('canopy','.025 .065 .09 1'),('exhaust','.23 .27 .30 1'),('nozzle_dark','.018 .023 .025 1'),('wing_left','.80 .86 .90 1'),('wing_right','.80 .86 .90 1'),('tail_left','.025 .28 .40 1'),('tail_right','.025 .28 .40 1')]:
        v=visual('edf_'+name,color);g=element(element(v,'geometry'),'mesh');element(g,'uri',str(directory/('edf_'+name+'.dae')));element(g,'scale',f'{scale} {scale} {scale}')
    for side in (-1,1):
        v=visual('intake_'+str(side),'.025 .045 .06 1',(-.075,.091*side,.09),(0,0,0))
        shape=element(element(v,'geometry'),'box');element(shape,'size','.17 .006 .045')
        v=visual('wingtip_'+str(side),'.025 .28 .4 1',(-.19,.78*side,.08),(0,.10,0))
        shape=element(element(v,'geometry'),'box');element(shape,'size','.12 .025 .008')
    for i,(x,y) in enumerate(((-.16,.14),(-.16,-.14),(.31,0))):
        v=visual('gear_strut_'+str(i),'.20 .23 .26 1',(x,y,-.015))
        cyl=element(element(v,'geometry'),'cylinder');element(cyl,'radius',.006);element(cyl,'length',.11)
        for part,radius,length,color in [('tire',.034,.024,'.025 .03 .035 1'),('hub',.014,.026,'.55 .60 .64 1')]:
            v=visual(part+'_'+str(i),color,(x,y,-.066),(math.pi/2,0,0));cyl=element(element(v,'geometry'),'cylinder');element(cyl,'radius',radius);element(cyl,'length',length)
    v=visual('camera_pod','.04 .06 .08 1',(0,0,-.028));element(element(v,'geometry'),'sphere');element(v.find('geometry/sphere'),'radius',.012)
    p=element(model,'plugin',filename='AirSeaAircraft',name='airsea::Aircraft');element(p,'pose_topic','/airsea/aircraft/truth_pose')
    meta={'name':'Coastal EDF UAV','license':'MIT','author':'Original project geometry','length_m':1.15*scale,'wingspan_m':wingspan,'exposed_propellers':False,'propulsion_visual':'Enclosed aft duct and exhaust; internal fan is not exposed','mesh_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in directory.glob('edf_*.dae')},'limitations':'Original conceptual visual only. Native generic ArduPlane FDM and propulsion retained; not calibrated EDF/jet dynamics.'}
    (out/'aircraft_visual.json').write_text(json.dumps(meta,indent=2)+'\n')
    return model
