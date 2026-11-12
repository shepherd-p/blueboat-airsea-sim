"""Re-run pixel perception from archived JPEGs and evaluate the coupled run.

Truth is used only here to measure errors. Predictions and GNSS acquisition
cues never count as successful visual detections or image measurements.
"""
from pathlib import Path
import argparse,csv,hashlib,json,sys,tempfile,xml.etree.ElementTree as ET
import cv2,numpy as np
from vision import Perception,rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tools/airsea'))
from skin_tags import geometry,geometry_hash,surface_point,lower_faces
from edf_airframe import wing


def read(path):return [json.loads(x) for x in path.read_text().splitlines()]


def validate(run):
    manifest=json.loads((run/'session_manifest.json').read_text());cfg=manifest['config']
    frames=read(run/'sensor_frames.jsonl');live=read(run/'mutual_perception.jsonl');controls=read(run/'coordination.jsonl');commands=read(run/'gimbal_commands.jsonl')
    start=manifest['cooperation_start_s'];end=manifest['cooperation_end_s'];assert start is not None and end is not None
    events=json.loads((run/'flight_events.json').read_text());epoch=manifest['epoch_offset_s']
    states=read(run/'native_fdm.jsonl');times=np.array([x['timestamp']+epoch for x in states]);pos=np.array([[s['position'][1],s['position'][0],-s['position'][2]] for s in states]);vel=np.array([[s['velocity'][1],s['velocity'][0],-s['velocity'][2]] for s in states])
    perception=Perception(cfg);match=True;calibration=True;truth_independent=True;hashes={};metrics={k:[] for k in ('air','boat')};subset=[]
    for record in live:
        index=record['index'];row=frames[index];t=record['t'];assert t==row['t']
        pair={}
        for kind in ('air','boat'):
            path=run/f'frames/{kind}/{index:06d}.jpg';image=cv2.imread(str(path));assert image is not None
            camera=cfg['camera' if kind=='air' else 'upward_camera']
            recorded=row[kind]
            calibration &= (image.shape[:2]==(camera['height'],camera['width'])
                            and recorded.get('image_width')==camera['width'] and recorded.get('image_height')==camera['height']
                            and abs(recorded.get('hfov_rad',0)-camera['hfov_rad'])<1e-12)
            meta={k:row[kind][k] for k in ('navigation_eye','navigation_origin','R_optical_to_enu','communicated_target_attitude_enu') if k in row[kind]}
            result=perception.step(image,meta,kind,t);logged=record[kind]
            for name in ('measurement','shape_detected','tag_detected','source'):match &= result[name]==logged[name]
            for name in ('position','velocity'):
                if result[name] is not None:match &= logged[name] is not None and np.linalg.norm(np.array(result[name])-logged[name])<1e-6
            R=np.array(meta['R_optical_to_enu']).reshape(3,3)
            match &= np.max(np.abs(R.T@R-np.eye(3)))<1e-8 and np.linalg.det(R)>.99999
            # The whitelisted estimate input excludes all target position and
            # truth values, including when malicious evaluation fields change.
            truth_independent &= not any('evaluation' in k or k=='target_position' for k in meta)
            if start<=t<=end:
                target=np.array(row[kind]['evaluation_target_only']);estimate=result['position'];error=None if estimate is None else float(np.linalg.norm(np.array(estimate)-target))
                range_m=float(np.linalg.norm(target-np.array(meta['navigation_eye'])))
                velocity_error=None
                if kind=='boat' and result['velocity'] is not None:
                    reference=np.array([np.interp(t,times,vel[:,a]) for a in range(3)])
                    velocity_error=float(np.linalg.norm(np.array(result['velocity'])-reference))
                metrics[kind].append(dict(index=index,t=t,measurement=result['measurement'],shape=result['shape_detected'],tag=result['tag_detected'],source=result['source'],age_s=result['age_s'],error_m=error,velocity_error_mps=velocity_error,range_m=range_m))
                hashes[f'{kind}/{index:06d}.jpg']=hashlib.sha256(path.read_bytes()).hexdigest()
            pair[kind]=result
        if start<=t<=end:subset.append(dict(index=index,t=t,**pair))
    checks=dict(full_mission_completed=manifest['status']=='completed',sea_state_1=cfg['sea_state']==1,
        ordinary_plane_arming=manifest['normal_plane_arming'] and any(e['event']=='arming_ack' and e['result']==0 for e in events),
        rover_armed=manifest['rover_armed'],coordinated_duration_at_least_60s=end-start>=60,
        original_images_independently_reproduced=bool(match),estimator_truth_inputs_absent=bool(truth_independent),
        independent_camera_pairs=len(live)==len(frames),all_mission_phases=set(range(1,len(cfg['mission']))).issubset({e['seq'] for e in events if e['event']=='mission_current'}),
        autonomous_disarm=any(e['event']=='autonomous_disarm_after_landing' for e in events),
        both_camera_feeds_archived=all(len(list((run/f'frames/{k}').glob('*.jpg')))==len(frames) for k in ('air','boat')))
    checks['rendered_camera_calibration_matches_estimator']=bool(calibration)
    source_hashes=json.loads((run/'source_hashes.json').read_text())
    checks['source_snapshot_hashes_match']=all(hashlib.sha256((run/'source_snapshot'/p).read_bytes()).hexdigest()==h for p,h in source_hashes.items())
    checks['estimator_code_matches_recorded']=hashlib.sha256(Path(__file__).with_name('vision.py').read_bytes()).hexdigest()==source_hashes['tools/cooperative/vision.py']
    world=ET.parse(run/'world.sdf');v=json.loads((run/'boat_visual_manifest.json').read_text());body=world.find("world/model[@name='blueboat']/link[@name='base_link']")
    checks['original_boat_assets_exact']=v['original_boat_restored'] and all(hashlib.sha256((run/'boat_visual_assets'/n).read_bytes()).hexdigest()==h for n,h in v['sha256'].items())
    checks['actual_world_original_boat']=body.find("visual[@name='official_cad']") is not None and body.find("visual[@name='port_hull']") is None
    aircraft=world.find("world/model[@name='aircraft']/link")
    skin=json.loads((run/'aircraft_skin_tags.json').read_text())
    checks['actual_world_upward_camera_and_aircraft_tag']=body.find("visual[@name='upward_camera_housing']") is not None and all(aircraft.find(f"visual[@name='edf_wing_{m['wing']}']/material/pbr/metal/albedo_map") is not None for m in skin['markers'])
    checks['no_separate_aircraft_tag_board']=aircraft.find("visual[@name='aircraft_apriltag_1']") is None and not any('marker.dae' in (x.text or '') for x in aircraft.findall('.//geometry/mesh/uri'))
    unchanged=True;fitted=True;calibrated=True;skin_hashes=True
    with tempfile.TemporaryDirectory() as temp:
        for marker in skin['markers']:
            side=marker['wing'];path=run/'aircraft_meshes'/f'edf_wing_{side}.dae'
            points,faces=geometry(path);reference=Path(temp)/f'{side}.dae';wing(reference,1 if side=='left' else -1)
            reference_points,reference_faces=geometry(reference)
            unchanged &= np.array_equal(points,reference_points) and np.array_equal(faces,reference_faces)
            lower=faces[lower_faces(points,faces)]
            cfg_marker=next(m for m in cfg['upward_camera']['aircraft_surface_tags'] if m['id']==marker['id'])
            calibrated &= np.max(np.abs(np.array(marker['corners_body_m'])-cfg_marker['corners_body_m']))<1e-10
            for p in marker['quiet_corners_body_m']+marker['corners_body_m']:
                fitted &= np.linalg.norm(np.array(surface_point(*p[:2],points,lower))-p)<1e-9
            calibrated &= marker['black_border_m']==cfg['upward_camera']['black_border_m']
            skin_hashes &= hashlib.sha256(path.read_bytes()).hexdigest()==marker['mesh_sha256'] and hashlib.sha256(path.with_suffix('.png').read_bytes()).hexdigest()==marker['texture_sha256']
    checks['paint_preserves_original_wing_vertices_and_triangles']=bool(unchanged)
    checks['whole_marker_quiet_zone_fits_existing_skin']=bool(fitted)
    checks['nonplanar_skin_corner_calibration_matches_estimator']=bool(calibrated)
    checks['wing_texture_and_mesh_hashes_match']=bool(skin_hashes)
    checks['skin_generator_matches_recorded']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==source_hashes[p] for p in ('tools/airsea/skin_tags.py','tools/airsea/edf_airframe.py'))
    checks['world_hash_matches']=hashlib.sha256((run/'world.sdf').read_bytes()).hexdigest()==json.loads((run/'input_hashes.json').read_text())['world_sdf_sha256']
    lift=np.where(pos[:,2]>2)[0][0];touch=np.where((np.arange(len(pos))>lift)&(pos[:,2]<=.101))[0][0]
    runway=cfg['runway']
    def on_runway(p):return runway['x_min_m']<=p[0]<=runway['x_max_m'] and abs(p[1])<runway['width_m']/2
    checks['takeoff_and_touchdown_on_runway']=on_runway(pos[lift]) and on_runway(pos[touch])
    checks['touchdown_sink_below_2mps']=0<=-vel[touch-1,2]<2
    checks['stopped_on_runway']=on_runway(pos[-1]) and np.linalg.norm(vel[-1])<.5
    summary={}
    for kind,rs in metrics.items():
        ts=np.array([r['t'] for r in rs]);weights=np.r_[np.diff(ts),max(0,end-ts[-1])];span=end-start
        freshness=min(.4,3/cfg['camera']['rate_hz'])
        # A successful old image cannot cover a long period with no sensor
        # frames. Include acquisition/end gaps rather than hiding them.
        coverage=lambda key:float(np.sum(np.minimum(weights,freshness)*np.array([r[key] for r in rs],bool))/span)
        measured=[r['t'] for r in rs if r['measurement']]
        maximum_gap=float(np.max(np.diff([start,*measured,end])))
        errors=[r['error_m'] for r in rs if r['measurement'] and r['error_m'] is not None]
        verr=[r['velocity_error_mps'] for r in rs if r['measurement'] and r['velocity_error_mps'] is not None]
        shapeonly=[r for r in rs if r['measurement'] and r['shape'] and not r['tag']]
        tagranges=[r['range_m'] for r in rs if r['tag']];shaperanges=[r['range_m'] for r in shapeonly]
        summary[kind]=dict(original_frames=len(rs),measurement_frames=sum(r['measurement'] for r in rs),tag_frames=sum(r['tag'] for r in rs),
            visual_measurement_time_fraction=coverage('measurement'),shape_time_fraction=coverage('shape'),tag_time_fraction=coverage('tag'),
            maximum_measurement_age_s=max(r['age_s'] for r in rs),maximum_measurement_gap_s=maximum_gap,
            actual_sensor_mean_hz=float((len(ts)-1)/(ts[-1]-ts[0])),maximum_sensor_gap_s=float(np.diff(ts).max()),
            position_error_m=dict(median=float(np.median(errors)),p95=float(np.percentile(errors,95)),maximum=max(errors)) if errors else None,
            velocity_error_mps=None if not verr else dict(median=float(np.median(verr)),p95=float(np.percentile(verr,95))),
            shape_only_frames=len(shapeonly),shape_only_max_range_m=max(shaperanges,default=0),tag_max_range_m=max(tagranges,default=0))
        checks[kind+'_visual_coverage_at_least_95_percent']=coverage('measurement')>=.95
        checks[kind+'_no_unmeasured_gap_over_1s']=max(max(r['age_s'] for r in rs),maximum_gap)<=1
        checks[kind+'_tag_observed']=len(tagranges)>=3
    checks['long_range_shape_continues_without_tag']=summary['boat']['shape_only_frames']>=30
    checks['air_to_boat_position_p95_below_5m']=summary['air']['position_error_m'] is not None and summary['air']['position_error_m']['p95']<5
    checks['boat_to_aircraft_position_p95_below_20m']=summary['boat']['position_error_m'] is not None and summary['boat']['position_error_m']['p95']<20
    checks['boat_to_aircraft_velocity_p95_below_12mps']=summary['boat']['velocity_error_mps'] is not None and summary['boat']['velocity_error_mps']['p95']<12
    c=cfg['coordination'];speeds=np.array([[r['plane_airspeed_command_mps'],r['boat_speed_command_mps']] for r in controls]);actual=np.array([[r['plane_measured_airspeed_mps'],r['boat_measured_speed_mps']] for r in controls])
    rover_params={}
    for line in (run/'ardurover_used.parm').read_text().splitlines():
        parts=line.split()
        if len(parts)==2 and not line.lstrip().startswith('#'):rover_params[parts[0]]=float(parts[1])
    checks['rover_speed_limit_matches_supervisor']=rover_params['WP_SPEED']>=c['boat_speed_max_mps']
    checks['speed_commands_bounded']=bool(np.all((speeds[:,0]>=c['plane_speed_min_mps'])&(speeds[:,0]<=c['plane_speed_max_mps'])&(speeds[:,1]>=c['boat_speed_min_mps'])&(speeds[:,1]<=c['boat_speed_max_mps'])))
    checks['both_commanded_speeds_adjust']=bool(np.ptp(speeds[:,0])>.5 and np.ptp(speeds[:,1])>.15)
    checks['both_physical_speeds_respond']=bool(np.ptp(actual[:,0])>.5 and np.ptp(actual[:,1])>.15)
    settled=np.array([r['t'] for r in controls])>start+10
    checks['boat_speed_adjusts_after_initial_acceleration']=bool(np.ptp(actual[settled,1])>.15)
    environment=np.genfromtxt(run/'environment.csv',delimiter=',',names=True)
    control_times=np.array([r['t'] for r in controls])
    # Centered position differences avoid using autopilot velocity reports as
    # the sole proof that the Gazebo boat actually changed physical speed.
    physical_velocity=np.column_stack([(np.interp(control_times+.25,environment['t'],environment[a])-
                                        np.interp(control_times-.25,environment['t'],environment[a]))/.5 for a in ('x','y')])
    physical_boat_speed=np.linalg.norm(physical_velocity,axis=1)
    physics_speed_error=float(np.percentile(np.abs(physical_boat_speed[settled]-actual[settled,1]),95))
    checks['boat_speed_response_matches_independent_physics']=bool(np.ptp(physical_boat_speed[settled])>.15 and physics_speed_error<.2)
    checks['controls_use_past_images']=all(r['air_track_sensor_t']<=r['t'] and r['boat_track_sensor_t']<=r['t'] for r in controls)
    acquisition=[r for r in commands if start<=r['t']<=end and r['source']=='communicated_aircraft_EKF_acquisition_cue']
    result=dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},cooperation_duration_s=end-start,
        observation=summary,speed_commands_range_mps={'aircraft':[speeds[:,0].min(),speeds[:,0].max()],'boat':[speeds[:,1].min(),speeds[:,1].max()]},
        physical_speeds_range_mps={'aircraft':[actual[:,0].min(),actual[:,0].max()],'boat':[actual[:,1].min(),actual[:,1].max()]},
        gazebo_boat_speed_range_mps=[float(physical_boat_speed.min()),float(physical_boat_speed.max())],boat_navigation_physics_speed_error_p95_mps=physics_speed_error,
        upward_EKF_reacquisition_cues_during_cooperation=len(acquisition),touchdown_sink_mps=float(-vel[touch-1,2]),source_frame_sha256=hashes,
        limitations='Restricted color/silhouette baseline, known geometry and communicated target IMU attitude; not a trained general detector. Upward acquisition/reacquisition uses communicated aircraft EKF, not truth. Shape range remains uncertain; ideal gimbals, images and generic FDM. Visual cruise acceptance is separate from takeoff/landing visibility. No RL or real-world validation.')
    (run/'cooperative_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    (run/'independent_perception.jsonl').write_text('\n'.join(json.dumps(x) for x in subset)+'\n')
    (run/'independent_validator_snapshot.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k!='source_frame_sha256'},indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);validate(p.parse_args().run)
