"""Verify actual Gazebo inspection pixels and original airframe geometry."""
from pathlib import Path
import argparse,json,math,sys
import cv2,numpy as np
from vision import tags,intrinsics
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/airsea')]
from skin_tags import geometry


def inspect(out,previous=None):
    skin=json.loads((out/'aircraft_skin_tags.json').read_text())
    image=cv2.imread(str(out/'06_wing_underside.png'));assert image is not None
    K=intrinsics(1920,1080,math.radians(50))
    eye=np.array([-.1,0,-2.2]);direction=np.array([-.22,0,.12])-eye
    yaw=math.atan2(direction[1],direction[0]);pitch=math.atan2(-direction[2],np.linalg.norm(direction[:2]))
    cy,sy=math.cos(yaw),math.sin(yaw);cp,sp=math.cos(pitch),math.sin(pitch)
    R=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    optical=R@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    reference=optical.T@(-eye);detections=[]
    for marker in skin['markers']:
        result=tags(image,K,marker['black_border_m'],marker['id'],marker['corners_body_m'],optical.T)
        detections.append(dict(id=marker['id'],detected=result is not None,
            body_origin_error_m=None if result is None else float(np.linalg.norm(result['translation']-reference)),
            reprojection_rms_px=None if result is None else result['rms_px']))
    comparison={}
    if previous:
        for side in ('left','right'):
            old=geometry(previous/'aircraft_meshes'/f'edf_wing_{side}.dae');new=geometry(out/'aircraft_meshes'/f'edf_wing_{side}.dae')
            comparison[side]=dict(same_vertices=bool(np.array_equal(old[0],new[0])),
                same_geometric_triangles=bool(np.array_equal(np.sort(old[1],axis=1),np.sort(new[1],axis=1))))
    passed=all(d['detected'] and d['body_origin_error_m']<.025 for d in detections) and all(all(c.values()) for c in comparison.values())
    result=dict(passed=passed,actual_render_detections=detections,historical_shape_comparison=comparison,
                note='Unmodified Gazebo pixels, known inspection camera only. This close-up check is separate from the flight mission.')
    (out/'inspection_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not passed:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--previous',type=Path)
    a=p.parse_args();inspect(a.out,a.previous)
