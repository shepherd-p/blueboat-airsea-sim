"""Explicit boat appearance selection; local original CAD never silently falls back.

Only visual elements are replaced. Collision, inertia, propulsion, waves and
the existing landing marker remain in the generated physical model.
"""
from pathlib import Path
import argparse,copy,hashlib,json,shutil,xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
LOCAL=ROOT/'config/boat_visual.local.json'
ORIGINAL_VISUALS={'official_cad','propeller_left','propeller_right','propeller_sweep_left','propeller_sweep_right'}
FALLBACK_VISUALS={'port_hull','starboard_hull','crossbar_-0.3','crossbar_0.3'}

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def selection():
    if not LOCAL.exists():
        if (ROOT/'.local_assets/blueboat_original').exists():raise RuntimeError('Original boat assets exist but their explicit selection file is missing. Refusing procedural fallback.')
        return {'mode':'procedural','description':'Public procedural twin hull; original CAD not configured'}
    config=json.loads(LOCAL.read_text())
    if config['mode']!='original_cad':raise ValueError('Local boat visual must explicitly select original_cad')
    folder=(ROOT/config['asset_directory']).resolve()
    for name,digest in config['sha256'].items():
        path=folder/name
        if not path.is_file() or sha(path)!=digest:
            raise RuntimeError(f'Original boat asset missing or changed: {name}. Refusing to replace it with a simplified model.')
    return dict(config,folder=folder)

def apply(boat,out,selected=None):
    selected=selected or selection();body=boat.find("link[@name='base_link']")
    before={key:ET.tostring(body.find(key)) for key in ('inertial',)}
    collisions=[ET.tostring(c) for c in body.findall('collision')]
    plugins=[ET.tostring(p) for p in boat.findall('plugin')]
    markers={v.get('name'):ET.tostring(v) for v in body.findall('visual') if v.get('name') in ('proposed_landing_panel','landing_apriltag_0')}
    metadata={'mode':selected['mode'],'original_boat_restored':selected['mode']=='original_cad'}
    if selected['mode']=='original_cad':
        folder=selected['folder'];snapshot=out/'boat_visual_assets';snapshot.mkdir(exist_ok=False)
        for name in selected['sha256']:shutil.copy2(folder/name,snapshot/name)
        original=ET.parse(snapshot/'model_visuals.sdf').find('model/link')
        visuals=list(original.findall('visual'));assert {v.get('name') for v in visuals}==ORIGINAL_VISUALS
        for v in list(body.findall('visual')):
            if v.get('name') in ORIGINAL_VISUALS|FALLBACK_VISUALS:body.remove(v)
        for visual in visuals:
            visual=copy.deepcopy(visual)
            for uri in visual.findall('.//mesh/uri'):
                name=Path(uri.text).name;assert (snapshot/name).is_file()
                uri.text=str(snapshot/name)
            body.append(visual)
        metadata.update(sha256={name:sha(snapshot/name) for name in selected['sha256']},
                        original_source_model_sha256=selected['original_source_model_sha256'],
                        visual_names=sorted(ORIGINAL_VISUALS),asset_directory='boat_visual_assets',
                        description='Original BlueBoat CAD body and separate rotors; local assets excluded from public source')
    else:
        metadata.update(description=selected['description'],visual_names=[v.get('name') for v in body.findall('visual')])
    assert ET.tostring(body.find('inertial'))==before['inertial']
    assert [ET.tostring(c) for c in body.findall('collision')]==collisions
    assert [ET.tostring(p) for p in boat.findall('plugin')]==plugins
    assert {v.get('name'):ET.tostring(v) for v in body.findall('visual') if v.get('name') in markers}==markers
    metadata['physical_model_and_marker_preserved']=True
    (out/'boat_visual_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('Boat appearance: '+metadata['mode']+' / '+metadata['description'],flush=True)
    return metadata

def import_model(source):
    source=source.resolve();model=source/'model.sdf'
    root=ET.parse(model);body=root.find('model/link');visuals=[v for v in body.findall('visual') if v.get('name') in ORIGINAL_VISUALS]
    assert {v.get('name') for v in visuals}==ORIGINAL_VISUALS,'Expected the original BlueBoat CAD with both separate rotors'
    target=ROOT/'.local_assets/blueboat_original';target.mkdir(parents=True,exist_ok=False)
    spec=ET.Element('sdf',version='1.9');link=ET.SubElement(ET.SubElement(spec,'model',name='blueboat'),'link',name='base_link')
    for visual in visuals:link.append(copy.deepcopy(visual))
    ET.indent(spec,space='  ');ET.ElementTree(spec).write(target/'model_visuals.sdf',encoding='utf-8',xml_declaration=True)
    names=['model_visuals.sdf']
    for name in ('blueboat_body.stl','propeller_left.stl','propeller_right.stl','rotors.json'):
        shutil.copy2(source/'meshes'/name,target/name);assert sha(target/name)==sha(source/'meshes'/name);names.append(name)
    config={'mode':'original_cad','asset_directory':'.local_assets/blueboat_original',
            'original_source_model_sha256':sha(model),'sha256':{name:sha(target/name) for name in names}}
    LOCAL.write_text(json.dumps(config,indent=2)+'\n');print(json.dumps(config,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--import-model',type=Path,required=True);import_model(p.parse_args().import_model)
