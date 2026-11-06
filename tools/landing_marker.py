"""Attach a textured planar fiducial to the existing boat link, as in legacy Xacro."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
from generate_model import element,box,save
ROOT=Path(__file__).resolve().parents[1];DESC=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description'

def assets(c):
    from PIL import Image
    base=DESC/'models/blueboat_landing_marker';tex=base/'materials/textures/tag36h11_0.png'
    tex.parent.mkdir(parents=True,exist_ok=True)
    cells=c['binary_cells_black_border_included'];grid=Image.new('L',(10,10),255)
    for y,row in enumerate(cells):
        for x,v in enumerate(row):grid.putpixel((x+1,y+1),255*v)
    grid.resize((1000,1000),Image.Resampling.NEAREST).convert('RGB').save(tex)
    # Printable vector, outer width 540 mm including white quiet zone.
    s=c['sheet_size_m']*1000
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="{s:g}mm" height="{s:g}mm" viewBox="0 0 10 10"><rect width="10" height="10" fill="white"/>'
    for y,row in enumerate(cells):
        for x,v in enumerate(row):
            if not v:svg+=f'<rect x="{x+1}" y="{y+1}" width="1" height="1" fill="black"/>'
    (base/'tag36h11_0_print_540mm.svg').write_text(svg+'</svg>')
    ns='http://www.collada.org/2005/11/COLLADASchema';ET.register_namespace('',ns)
    root=ET.Element('COLLADA',xmlns=ns,version='1.4.1');a=element(root,'asset');element(a,'up_axis','Z_UP');element(a,'unit',name='meter',meter='1')
    im=element(element(root,'library_images'),'image',id='tag-image');element(im,'init_from','../materials/textures/tag36h11_0.png')
    profile=element(element(element(root,'library_effects'),'effect',id='tag-effect'),'profile_COMMON')
    surface=element(element(profile,'newparam',sid='tag-surface'),'surface',type='2D');element(surface,'init_from','tag-image')
    sampler=element(element(profile,'newparam',sid='tag-sampler'),'sampler2D');element(sampler,'source','tag-surface')
    phong=element(element(profile,'technique',sid='common'),'phong');element(element(phong,'diffuse'),'texture',texture='tag-sampler',texcoord='UVMap')
    element(element(phong,'ambient'),'color','1 1 1 1');element(element(phong,'specular'),'color','0 0 0 1')
    element(element(element(root,'library_materials'),'material',id='tag-material'),'instance_effect',url='#tag-effect')
    h=c['sheet_size_m']/2
    mesh=element(element(element(root,'library_geometries'),'geometry',id='marker'),'mesh')
    for name,values,stride,names in [('positions',[-h,-h,0,h,-h,0,h,h,0,-h,h,0],3,'XYZ'),('uv',[0,0,1,0,1,1,0,1],2,'ST'),('normals',[0,0,1]*4,3,'XYZ')]:
        src=element(mesh,'source',id=name);element(src,'float_array',' '.join(map(str,values)),id=name+'-array',count=str(len(values)))
        ac=element(element(src,'technique_common'),'accessor',source='#'+name+'-array',count='4',stride=str(stride))
        for n in names:element(ac,'param',name=n,type='float')
    v=element(mesh,'vertices',id='vertices');element(v,'input',semantic='POSITION',source='#positions')
    tris=element(mesh,'triangles',count='2',material='tag-material');element(tris,'input',semantic='VERTEX',source='#vertices',offset='0');element(tris,'input',semantic='TEXCOORD',source='#uv',offset='1',set='0');element(tris,'input',semantic='NORMAL',source='#normals',offset='2');element(tris,'p','0 0 0 1 1 1 2 2 2 0 0 0 2 2 2 3 3 3')
    vs=element(element(root,'library_visual_scenes'),'visual_scene',id='Scene');node=element(vs,'node',id='marker_surface');instance=element(node,'instance_geometry',url='#marker')
    mat=element(element(element(instance,'bind_material'),'technique_common'),'instance_material',symbol='tag-material',target='#tag-material');element(mat,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set='0')
    element(element(root,'scene'),'instance_visual_scene',url='#Scene')
    save(root,base/'meshes/marker.dae')
    # Explicit detector dimensions for direct image/PnP consumers.
    (DESC/'config/landing_marker_detector.yaml').write_text('''# Dimensions are BLACK-border side lengths, excluding the white quiet zone.
family: tag36h11
id: 0
size_m: 0.432
sheet_size_m: 0.54
body_to_marker_translation_m: [0.0, 0.0, 0.1576]
body_to_marker_rpy_rad: [0.0, 0.0, 0.0]
# Marker x/y align with boat forward/port; marker z points upward.
''')

def attach(body):
    path=DESC/'config/landing_marker.json'
    if not path.exists():return
    c=json.loads(path.read_text())
    if not c['enabled']:return
    assets(c)
    panel=box(body,'proposed_landing_panel',c['panel_size_m'],[*c['panel_center_m'],0,0,0],'.85 .85 .83 1')
    element(panel,'cast_shadows','false')
    v=element(body,'visual',name='landing_apriltag_0');element(v,'pose',' '.join(map(str,[*c['marker_center_m'],0,0,0])))
    element(v,'cast_shadows','false');element(element(element(v,'geometry'),'mesh'),'uri','model://blueboat_landing_marker/meshes/marker.dae')
    mat=element(v,'material');element(mat,'ambient','1 1 1 1');element(mat,'diffuse','1 1 1 1');element(mat,'specular','0 0 0 1')
    metal=element(element(mat,'pbr'),'metal');element(metal,'albedo_map','model://blueboat_landing_marker/materials/textures/tag36h11_0.png');element(metal,'metalness',0);element(metal,'roughness',1)
