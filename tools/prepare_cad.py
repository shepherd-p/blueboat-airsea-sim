"""Make a lightweight visual mesh from the manufacturer's original STL.

The STEP/STL assembly is only a visual asset. Buoyancy and mass are not inferred
from this assembly, since STL has no material, battery or mass properties.
"""
from pathlib import Path
import json
import zipfile
import argparse
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--inspect-only',action='store_true')
    args=parser.parse_args()
    archive=ROOT/'references/blueboat_cad_official.zip'
    with zipfile.ZipFile(archive) as z:
        name=next(n for n in z.namelist() if n.lower().endswith('.stl'))
        target=ROOT/'references'/name
        if not target.exists():
            print('Extracting',name,flush=True); z.extract(name,ROOT/'references')
    print('Loading official STL',flush=True)
    mesh=trimesh.load_mesh(target,process=False)
    print('Raw bounds',mesh.bounds,'faces',len(mesh.faces),flush=True)
    if args.inspect_only:
        (ROOT/'references/cad_raw_bounds.json').write_text(json.dumps({'bounds':mesh.bounds.tolist(),'faces':len(mesh.faces)},indent=2))
        return
    # RevA CAD: millimetres, raw axis orientation established from bounds and drawing.
    # Axis transform is supplied after the inspect pass, to avoid silently guessing.
    axes=json.loads((ROOT/'references/cad_transform.json').read_text())
    mesh.apply_transform(np.array(axes['transform']))
    print('Merging tessellation vertices',flush=True)
    mesh.merge_vertices(digits_vertex=4)
    mesh.update_faces(mesh.nondegenerate_faces())
    print('Merged',len(mesh.vertices),'vertices',len(mesh.faces),'faces',flush=True)
    mesh=mesh.simplify_quadric_decimation(face_count=65000,aggression=7)
    dest=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/models/blueboat/meshes/blueboat_visual.stl'
    dest.parent.mkdir(parents=True,exist_ok=True); mesh.export(dest)
    record={'source_url':'https://cad.bluerobotics.com/BLUEBOAT_120_BR-101447_RevA_PUB.zip',
            'source_entry':name,'units':'m','transformation':axes,
            'visual_bounds_m':mesh.bounds.tolist(),'visual_faces':len(mesh.faces),
            'note':'Decimated official public CAD, for visual use only. Blue Robotics attribution retained.'}
    (ROOT/'references/cad_processing.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2),flush=True)

if __name__=='__main__': main()
