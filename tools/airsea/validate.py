"""Independent flight/vision acceptance using archived evidence, never live truth control."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def rotation(q):
    w,x,y,z=np.array(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def validate(out,pilot=False):
    out=out.resolve();manifest=json.loads((out/'session_manifest.json').read_text())
    cfg=json.loads((out/'mission_config.json').read_text())
    states=[json.loads(line) for line in (out/'native_fdm.jsonl').read_text().splitlines()]
    rows=list(csv.DictReader((out/'airborne_observations.csv').open()))
    events=json.loads((out/'flight_events.json').read_text())
    calibration=json.loads((out/'airborne_camera_calibration.json').read_text())
    inputs=json.loads((out/'input_hashes.json').read_text())
    t=np.array([s['timestamp'] for s in states]);positions=np.array([s['position'] for s in states])
    ground_height=.1;alt=-positions[:,2]
    airborne=np.where(alt>2)[0];assert len(airborne),'No flight'
    lift_index=airborne[0];ground=np.where((np.arange(len(states))>lift_index)&(alt<=.101))[0]
    touchdown_index=int(ground[0]) if len(ground) else None
    runway=cfg['runway']
    def on_runway(p):return runway['x_min_m']<=p[1]<=runway['x_max_m'] and abs(p[0]-runway['center_y_m'])<runway['width_m']/2
    touchdown=states[touchdown_index] if touchdown_index else None
    before=states[touchdown_index-1] if touchdown_index else None
    last=states[-1]
    checks={
        'world_hash_matches':hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest()==inputs['world_sdf_sha256'],
        'sea_state_1':manifest['sea_state']==1 and json.loads((out/'profile.json').read_text())['target_hm0_m']==.05,
        'normal_plane_arming':manifest['normal_plane_arming'] and any(e['event']=='arming_ack' and e['result']==0 for e in events),
        'real_fdm_not_scripted_path':len(states)>1000 and np.all(np.diff(t)>0),
        'aircraft_comparable_size':1<=cfg['aircraft']['wingspan_m']/1.2<=2,
        'runway_takeoff':on_runway(positions[lift_index]) and alt.max()>25,
        'rover_firmware_armed':manifest['rover_armed'],
        'archived_camera_frame_set':len(rows)==len(list((out/'onboard_frames').glob('*.jpg'))),
        'camera_follows_aircraft':np.ptp([float(r['plane_x']) for r in rows])>150 and np.ptp([float(r['plane_z']) for r in rows])>20,
    }
    boat_visual_path=out/'boat_visual_manifest.json'
    if boat_visual_path.exists():
        visual=json.loads(boat_visual_path.read_text());tree=ET.parse(out/'world.sdf')
        body=tree.find("world/model[@name='blueboat']/link[@name='base_link']")
        checks['boat_visual_selection_recorded']=visual['mode'] in ('original_cad','procedural')
        if visual['mode']=='original_cad':
            checks['original_boat_mesh_hashes_match']=all(hashlib.sha256((out/'boat_visual_assets'/name).read_bytes()).hexdigest()==digest for name,digest in visual['sha256'].items())
            names={v.get('name') for v in body.findall('visual')}
            checks['original_boat_present_without_fallback_hulls']='official_cad' in names and not {'port_hull','starboard_hull'}&names
            checks['original_boat_propellers_and_marker_present']={'propeller_left','propeller_right','landing_apriltag_0','proposed_landing_panel'}.issubset(names)
            original=ET.parse(out/'boat_visual_assets/model_visuals.sdf').find('model/link')
            def signature(element):return [element.tag,element.attrib,(element.text or '').strip(),[signature(c) for c in element]]
            visual_match=True
            for old in original.findall('visual'):
                current=body.find(f"visual[@name='{old.get('name')}']")
                for uri in old.findall('.//mesh/uri'):
                    uri.text=str(out/'boat_visual_assets'/Path(uri.text).name)
                visual_match &= current is not None and signature(old)==signature(current)
            checks['rendered_boat_visuals_match_imported_original']=visual_match
    if not pilot:
        checks.update({
            'all_mission_phases':set(range(1,len(cfg['mission']))).issubset({e['seq'] for e in events if e['event']=='mission_current'}),
            'autonomous_landing_and_disarm':manifest['status']=='completed' and any(e['event']=='autonomous_disarm_after_landing' for e in events),
            'touchdown_on_runway':bool(touchdown and on_runway(touchdown['position'])),
            'touchdown_sink_below_2mps':bool(before and 0<=before['velocity'][2]<2),
            'stopped_on_runway':on_runway(last['position']) and np.linalg.norm(last['velocity'])<.5,
            'image_based_second_pass_replanned':manifest['image_based_second_pass_replanned'] and any(e['event']=='image_based_waypoint_update_ack' and e['result']==0 for e in events),
        })
    K=np.array([[calibration['fx'],0,calibration['cx']],[0,calibration['fy'],calibration['cy']],[0,0,1]])
    size=calibration['black_border_m'];s=size/2
    obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],dtype=np.float64)
    C=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    down=np.array([[0,0,1],[0,1,0],[-1,0,0]])
    params=cv2.aruco.DetectorParameters_create();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;params.cornerRefinementWinSize=3
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    def solve(image,row):
        corners,ids,_=cv2.aruco.detectMarkers(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),dictionary,parameters=params)
        if ids is None or 0 not in ids:return None
        pixels=corners[list(ids.flatten()).index(0)].reshape(4,2)
        _,rvecs,tvecs,_=cv2.solvePnPGeneric(obj,pixels,K,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates=[]
        for rv,tv in zip(rvecs,tvecs):
            if tv[2,0]<=0:continue
            projected=cv2.projectPoints(obj,rv,tv,K,None)[0].reshape(4,2)
            rms=np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1)))
            candidates.append((rms,tv.flatten()))
        if not candidates:return None
        rms,translation=min(candidates,key=lambda v:v[0])
        edges=np.linalg.norm(pixels-np.roll(pixels,-1,axis=0),axis=1)
        gimballed='camera_qw' in row
        if rms>=(1.2 if gimballed else 2) or (gimballed and (np.mean(edges)<20 or np.min(edges)<12)):return None
        R=rotation([float(row[k]) for k in ('nav_qw','nav_qx','nav_qy','nav_qz')])
        origin=np.array([float(row[k]) for k in ('nav_x','nav_y','nav_z')])+R@np.array([0,0,-.035])
        mount_rotation=rotation([float(row[k]) for k in ('camera_qw','camera_qx','camera_qy','camera_qz')]) if 'camera_qw' in row else down
        return origin+R@mount_rotation@C.T@translation,rms
    matches=True;estimates=[];hashes={}
    gimballed=calibration.get('orientation_model')=='per_frame_body_camera_quaternion'
    aim=np.array(calibration.get('initial_search_point_enu_m',[0,0,0]),dtype=float)
    causal_aim=True;pointing_geometry=True;rate_bounded=True;previous_camera=None;previous_time=None;have_image_fix=False
    for i,row in enumerate(rows):
        file=out/'onboard_frames'/f'{i:06d}.jpg'
        image=cv2.imread(str(file));assert image is not None
        result=solve(image,row)
        stamp=float(row['gazebo_s'])
        if gimballed:
            causal_aim &= np.linalg.norm(aim-np.array([float(row[k]) for k in ('aim_x','aim_y','aim_z')]))<1e-7
            causal_aim &= row['pointing_source']==('past_image_estimate' if have_image_fix else 'planned_search_point')
            R=rotation([float(row[k]) for k in ('nav_qw','nav_qx','nav_qy','nav_qz')])
            camera=rotation([float(row[k]) for k in ('camera_qw','camera_qx','camera_qy','camera_qz')])
            origin=np.array([float(row[k]) for k in ('nav_x','nav_y','nav_z')])+R@np.array([0,0,-.035])
            direction=aim-origin;horizontal=np.linalg.norm(direction[:2]);limit=max(.1,-direction[2])*math.tan(math.radians(65))
            if horizontal>limit:direction[:2]*=limit/horizontal
            direction[2]=min(-.1,direction[2]);direction/=np.linalg.norm(direction)
            left=np.cross([0,1,0],direction);left/=np.linalg.norm(left);up=np.cross(direction,left)
            desired=R.T@np.column_stack((direction,left,up))
            def angle(a,b):return math.acos(float(np.clip((np.trace(a.T@b)-1)/2,-1,1)))
            if previous_camera is not None:
                budget=math.radians(120)*(stamp-previous_time)
                rate_bounded &= angle(previous_camera,camera)<=budget+1e-6
                pointing_geometry &= abs(angle(camera,desired)-max(0,angle(previous_camera,desired)-budget))<1e-5
            else:pointing_geometry &= angle(camera,desired)<1e-5
            previous_camera=camera;previous_time=stamp
            if result is not None:
                gain=min(.25,(stamp-float(rows[i-1]['gazebo_s']))/.4) if i else .25
                position=result[0];aim=aim*(1-gain)+np.array([position[0],position[1],np.clip(position[2],0,.5)])*gain
                have_image_fix=True
        logged=bool(int(row['detected']))
        matches &= (result is not None)==logged
        if result is not None:
            position,rms=result
            live=np.array([float(row[k]) for k in ('estimate_x','estimate_y','estimate_z')])
            delta=float(np.linalg.norm(position-live));matches &=delta<.0001
            truth=np.array([float(row[k]) for k in ('boat_x','boat_y','boat_z')])
            estimates.append(dict(index=i,t=float(row['gazebo_s']),error_m=float(np.linalg.norm(position-truth)),live_delta_m=delta,rms_px=rms))
            hashes[file.name]=hashlib.sha256(file.read_bytes()).hexdigest()
            if len(estimates)==1:
                changed=dict(row);changed['nav_x']=str(float(row['nav_x'])+10)
                shifted=solve(image,changed)[0]
                checks['estimator_uses_navigation_not_truth']=np.linalg.norm((shifted-position)-[10,0,0])<1e-6
                checks['blank_image_has_no_detection']=solve(np.full_like(image,127),row) is None
                cv2.imwrite(str(out/'first_independent_detection.jpg'),image)
    errors=np.array([e['error_m'] for e in estimates]);times=np.array([e['t'] for e in estimates])
    phase_events=[e for e in events if e['event']=='mission_current']
    detections_per_pass={3:0,6:0}
    for estimate in estimates:
        native_time=estimate['t']-manifest['epoch_offset_s']
        phases=[e['seq'] for e in phase_events if e['sim_s']<=native_time]
        phase=phases[-1] if phases else None
        estimate['mission_seq']=phase
        if phase in detections_per_pass:detections_per_pass[phase]+=1
    checks.update({
        'independent_pixel_estimates_match_live':bool(matches),
        'at_least_5_airborne_detections':len(estimates)>=5,
        'image_track_span_at_least_1s':bool(len(times) and np.ptp(times)>1),
        'position_error_p95_below_3m':bool(len(errors) and np.percentile(errors,95)<3),
        'boat_moves_during_mission':np.ptp([float(r['boat_x']) for r in rows])>1,
    })
    if not pilot:checks['detections_on_both_search_passes']=all(n>0 for n in detections_per_pass.values())
    visibility={}
    for phase in (3,6):
        phase_times=[e['t'] for e in estimates if e['mission_seq']==phase];groups=[]
        for stamp in phase_times:
            if not groups or stamp-groups[-1][-1]>.35:groups.append([])
            groups[-1].append(stamp)
        visibility[phase]={'accepted_frames':len(phase_times),'longest_continuous_detection_s':max((g[-1]-g[0] for g in groups),default=0),'total_observation_span_s':max(phase_times)-min(phase_times) if phase_times else 0}
    if gimballed:
        checks.update(gimbal_pointing_reproduced_from_past_images=bool(causal_aim),gimbal_geometry_reproduced=bool(pointing_geometry),gimbal_slew_within_120_deg_s=bool(rate_bounded))
        checks['sustained_image_detection_on_search_passes']=all(v['longest_continuous_detection_s']>=2 for phase,v in visibility.items() if not pilot or phase==3)
    videos={}
    for name in ('FixedWing_Full_Mission_Clean.mp4','Onboard_Camera_Clean.mp4'):
        path=out/name
        result=subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],capture_output=True,text=True)
        checks[name+'_decodes']=result.returncode==0
        videos[name]=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size','-of','json',str(path)]))
    checks={name:bool(value) for name,value in checks.items()}
    frame_times=np.array([float(r['gazebo_s']) for r in rows]);gaps=np.diff(frame_times)
    capture_timing=dict(archived_sensor_frames=len(rows),elapsed_simulation_s=float(np.ptp(frame_times)),
        effective_image_hz=float(len(gaps)/np.ptp(frame_times)),
        median_gap_s=float(np.median(gaps)),p95_gap_s=float(np.percentile(gaps,95)),maximum_gap_s=float(gaps.max()),
        note='Video container fps can contain repeated frames. Flight/vision acceptance does not establish presentation smoothness; use check_replay.py for frame-exact presentation exports.')
    report=dict(passed=all(checks.values()),pilot=pilot,checks=checks,capture_timing=capture_timing,
        camera_frames=len(rows),image_detections=len(estimates),image_track_span_s=float(np.ptp(times)) if len(times) else 0,
        detections_per_search_pass=detections_per_pass,visibility_per_search_pass=visibility,
        position_error_m=dict(median=float(np.median(errors)),p95=float(np.percentile(errors,95)),maximum=float(errors.max())) if len(errors) else None,
        touchdown=dict(t=touchdown['timestamp'],east=touchdown['position'][1],north=touchdown['position'][0],sink_mps=before['velocity'][2]) if touchdown else None,
        final_position_ned_m=last['position'],maximum_altitude_m=float(alt.max()),
        videos=videos,detection_frame_sha256=hashes,estimates=estimates,
        limitations='Native ArduPlane aerodynamic/flat-ground FDM drives Gazebo airframe and calibrated camera; Gazebo simulates boat/waves. No aircraft-water/terrain collision coupling, real-vessel/airframe calibration, camera blur/noise or actual flight validation. Image tracking is an intermittent two-pass survey, not persistent lock under occlusion.')
    (out/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('estimates','detection_frame_sha256')},indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--pilot',action='store_true')
    args=p.parse_args();validate(args.out,args.pilot)
