"""AprilTag paint on existing wing triangles; no board, decal mesh or offset.

Corner calibration intersects the actual triangulated skin. The estimator
uses these 3D corners and a communicated IMU attitude, not a planar square.
"""
import hashlib,json,xml.etree.ElementTree as ET
import cv2,numpy as np
from edf_airframe import mesh_file

NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}


def geometry(path):
    tree=ET.parse(path)
    p=np.fromstring(tree.find(".//c:source[@id='positions']/c:float_array",NS).text,sep=' ').reshape(-1,3)
    triangles=tree.find('.//c:triangles',NS)
    stride=max(int(x.get('offset')) for x in triangles.findall('c:input',NS))+1
    index=np.fromstring(triangles.find('c:p',NS).text,sep=' ',dtype=int).reshape(-1,stride)
    return p,index[:,0].reshape(-1,3)


def geometry_hash(points,faces):
    return hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()+np.asarray(faces,dtype='<i8').tobytes()).hexdigest()


def lower_faces(points,faces):
    c=points[faces].mean(axis=1);fraction=(np.abs(c[:,1])-.07)/.73
    chord=.45-.30*fraction;u=(.19-.31*fraction-c[:,0])/chord
    mid=.055+.025*fraction+chord*.012*np.sin(np.pi*u)
    # Exclude end caps; their XY projection has zero area.
    area=np.abs(np.cross(points[faces[:,1],:2]-points[faces[:,0],:2],points[faces[:,2],:2]-points[faces[:,0],:2]))
    return (c[:,2]<mid-1e-7)&(area>1e-12)


def surface_point(x,y,points,triangles):
    for triangle in points[triangles]:
        matrix=np.vstack((triangle[:,:2].T,np.ones(3)))
        if abs(np.linalg.det(matrix))<1e-12:continue
        w=np.linalg.solve(matrix,[x,y,1])
        if min(w)>=-1e-7:return [x,y,float(w@triangle[:,2])]
    raise ValueError(f'Marker extends outside wing skin at {x,y}')


def apply(body,out,cfg):
    scale=cfg['aircraft']['wingspan_m']/1.6
    if scale!=1:raise ValueError('Recalibrate skin paint for a different aircraft scale')
    records=[]
    for marker in cfg['upward_camera']['aircraft_surface_tags']:
        side=marker['wing'];path=out/'aircraft_meshes'/f'edf_wing_{side}.dae'
        points,faces=geometry(path);before=geometry_hash(points,faces);lower=lower_faces(points,faces)
        cx,cy=marker['center_xy_body_m'];size=cfg['upward_camera']['black_border_m'];outer=size*1.25
        xmin,xmax=-.6,.3;ymin,ymax=(.05,.82) if side=='left' else (-.82,-.05)
        def xy_corners(length):
            s=length/2
            return [(cx-s,cy+s),(cx+s,cy+s),(cx+s,cy-s),(cx-s,cy-s)]
        # Check the entire white quiet zone, not only the black square.
        quiet=[surface_point(x,y,points,faces[lower]) for x,y in xy_corners(outer)]
        # Ogre's Collada UV image convention viewed from the underside needs
        # a horizontal image flip. Calibrate canonical decoder corner order
        # to the same physical flip, not a mirrored/planar substitute image.
        raw=xy_corners(size)
        corners=[surface_point(x,y,points,faces[lower]) for x,y in (raw[1],raw[0],raw[3],raw[2])]
        marker['corners_body_m']=corners
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        tag=cv2.aruco.drawMarker(dictionary,marker['id'],800)
        tag=cv2.copyMakeBorder(tag,100,100,100,100,cv2.BORDER_CONSTANT,value=255)
        texture=np.full((2048,2048,3),(230,219,204),np.uint8)
        def pixel(x,y):return (int(round((x-xmin)/(xmax-xmin)*2048)),int(round((ymax-y)/(ymax-ymin)*2048)))
        x0,y0=pixel(cx-outer/2,cy+outer/2);x1,y1=pixel(cx+outer/2,cy-outer/2)
        texture[y0:y1,x0:x1]=cv2.resize(cv2.flip(tag,1),(x1-x0,y1-y0),interpolation=cv2.INTER_NEAREST)[:,:,None]
        texpath=path.with_suffix('.png');cv2.imwrite(str(texpath),texture)
        xyz=points[faces];uv=np.full((*xyz.shape[:2],2),.99)
        uv[lower,:,0]=(xyz[lower,:,0]-xmin)/(xmax-xmin)
        uv[lower,:,1]=(xyz[lower,:,1]-ymin)/(ymax-ymin)
        mesh_file(path,points,faces,uv,texpath)
        v=body.find(f"visual[@name='edf_wing_{side}']");mat=v.find('material')
        mat.find('ambient').text='1 1 1 1';mat.find('diffuse').text='1 1 1 1';mat.find('specular').text='0 0 0 1'
        # A PBR map is also required by the Gazebo Ogre2 material path.
        from generate_model import element
        metal=element(element(mat,'pbr'),'metal');element(metal,'albedo_map',str(texpath));element(metal,'metalness',0);element(metal,'roughness',1)
        after=geometry_hash(*geometry(path));assert before==after
        # Quantify why a planar-square solver is inappropriate for this skin.
        c=np.array(corners);_,_,vh=np.linalg.svd(c-c.mean(axis=0))
        warp=float(np.max(np.abs((c-c.mean(axis=0))@vh[-1])))
        records.append(dict(**marker,black_border_m=size,quiet_zone_outer_m=outer,quiet_corners_body_m=quiet,
            corner_nonplanarity_m=warp,geometry_before_sha256=before,geometry_after_sha256=after,
            texture_sha256=hashlib.sha256(texpath.read_bytes()).hexdigest(),mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    result=dict(installation='Paint texture in existing main-wing underside UVs; zero added geometry, standoff, collision, mass or plate',
        geometry_unchanged=all(r['geometry_before_sha256']==r['geometry_after_sha256'] for r in records),markers=records,
        pose_method='Known-attitude translation fit to calibrated nonplanar skin corners; aircraft body origin output',
        limitations='Conceptual smooth paint/film installation. Adhesion, surface finish, structural clearance and aerodynamic effects require real airframe engineering; generic SITL is not CFD.')
    (out/'aircraft_skin_tags.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
