"""Frame-exact presentation replay of an independently validated recorded run.

Aircraft and boat poses come from their original native FDM logs. This does
not rerun flight control or create new sensor/estimator acceptance evidence.
"""
from pathlib import Path
import argparse,csv,hashlib,json,math,os,signal,subprocess,time
import xml.etree.ElementTree as ET
import numpy as np
from pymavlink import DFReader

ROOT=Path(__file__).resolve().parents[2]

def enu_quaternion(q):
    w,x,y,z=q
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(float(np.clip(2*(w*y-z*x),-1,1)))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    r,p,y=roll,-pitch,math.pi/2-yaw
    cr,sr=math.cos(r/2),math.sin(r/2);cp,sp=math.cos(p/2),math.sin(p/2);cy,sy=math.cos(y/2),math.sin(y/2)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]

def resample(t,p,q,targets):
    t=np.asarray(t);p=np.asarray(p);q=np.asarray(q)
    assert np.all(np.diff(t)>0)
    assert min(targets)>=t[0] and max(targets)<=t[-1],(min(targets),max(targets),t[0],t[-1])
    for i in range(1,len(q)):
        if np.dot(q[i-1],q[i])<0:q[i]*=-1
    indices=np.clip(np.searchsorted(t,targets)-1,0,len(t)-2)
    u=((targets-t[indices])/(t[indices+1]-t[indices]))[:,None]
    positions=(1-u)*p[indices]+u*p[indices+1]
    quaternions=(1-u)*q[indices]+u*q[indices+1]
    quaternions/=np.linalg.norm(quaternions,axis=1,keepdims=True)
    return positions,quaternions

def prepare(run,out,mode='highlights',pilot_seconds=6):
    run=run.resolve();out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((run/'session_manifest.json').read_text())
    report=json.loads((run/'validation.json').read_text());assert report['passed']
    rows=list(csv.DictReader((run/'airborne_observations.csv').open()))
    native=[json.loads(line) for line in (run/'native_fdm.jsonl').read_text().splitlines()]
    epoch=manifest['epoch_offset_s'];start=float(rows[0]['gazebo_s'])
    end=float(rows[-1]['gazebo_s']);fps=30
    detections=[float(r['gazebo_s']) for r in rows if r['detected']=='1']
    second=next(t for t in detections if t-detections[0]>20)
    lift=next(s['timestamp']+epoch for s in native if -s['position'][2]>2)
    landing=report['touchdown']['t']+epoch
    if mode=='review':
        segments=[(max(start,lift-4),16,0),(lift+17,8,0),
                  (detections[0]-5,4,0),(detections[0]-.7,3,1),
                  (second-7,6,0),(second-.7,3,1),
                  (landing-65,8,0),(landing-12,min(24,end-(landing-12)),0)]
    elif mode=='highlights':
        segments=[(max(start,lift-4),16,0),(detections[0]-5,4,0),
                  (detections[0]-.7,3,1),(second-7,6,0),(second-.7,3,1),
                  (landing-12,min(24,end-(landing-12)),0)]
    elif mode=='pilot':segments=[(start+20,pilot_seconds,2)]
    else:segments=[(start,end-start,2)]
    targets=[];views=[]
    for t,length,view in segments:
        n=int(math.floor(length*fps));targets.extend(t+np.arange(n)/fps);views.extend([view]*n)
    targets=np.asarray(targets)
    plane_t=[s['timestamp']+epoch for s in native]
    plane_p=[[s['position'][1],s['position'][0],-s['position'][2]] for s in native]
    plane_q=[enu_quaternion(s['quaternion']) for s in native]
    pp,pq=resample(plane_t,plane_p,plane_q,targets)
    # SIM2 has full precision native JSON NED position; matching SIM has its quaternion.
    binary=run/'ardurover/logs/00000001.BIN'
    reader=DFReader.DFReader_binary(str(binary));positions={};rotations={}
    while True:
        m=reader.recv_match(type=['SIM','SIM2'],blocking=False)
        if m is None:break
        stamp=m.TimeUS/1e6
        if m.get_type()=='SIM2':positions[stamp]=[m.PE,m.PN,-m.PD]
        else:rotations[stamp]=enu_quaternion([m.Q1,m.Q2,m.Q3,m.Q4])
    original_boat_t=sorted(positions.keys()&rotations.keys())
    boat_p=[positions[t] for t in original_boat_t];boat_q=[rotations[t] for t in original_boat_t]
    environment=list(csv.DictReader((run/'environment.csv').open()))
    et=np.array([float(r['t']) for r in environment])
    ex=np.array([float(r['x']) for r in environment]);ey=np.array([float(r['y']) for r in environment])
    sample=np.arange(100,len(original_boat_t)-100,20)
    bt=np.array(original_boat_t)[sample];xy=np.array(boat_p)[sample,:2]
    def alignment_error(offset):
        expected=np.column_stack((np.interp(bt+offset,et,ex),np.interp(bt+offset,et,ey)))
        return np.mean(np.sum((expected-xy)**2,axis=1))
    coarse=min(np.arange(-5,5.001,.02),key=alignment_error)
    boat_offset=float(min(np.arange(coarse-.02,coarse+.0201,.001),key=alignment_error))
    physics_alignment_rms=float(math.sqrt(alignment_error(boat_offset)))
    assert physics_alignment_rms<.003,physics_alignment_rms
    boat_t=np.array(original_boat_t)+boat_offset
    with (out/'boat_fdm.csv').open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(['simulation_s','x','y','z','qw','qx','qy','qz'])
        for t,p,q in zip(boat_t,boat_p,boat_q):writer.writerow([t,*p,*q])
    bp,bq=resample(boat_t,boat_p,boat_q,targets)
    # Measure the old live scene's latency separately from dense physics telemetry.
    recorded_t=np.array([float(r['gazebo_s']) for r in rows])
    rp,rq=resample(boat_t,boat_p,boat_q,recorded_t)
    predicted=[]
    for p,q in zip(rp,rq):
        w,x,y,z=q;predicted.append(p+.1576*np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]))
    actual=np.array([[float(r[k]) for k in ('boat_x','boat_y','boat_z')] for r in rows])
    boat_error=np.linalg.norm(np.array(predicted)-actual,axis=1)
    telemetry=list(csv.DictReader((run/'flight_telemetry.csv').open()))
    throttle=np.interp(targets,[float(r['t'])+epoch for r in telemetry],[float(r['throttle'])/100 for r in telemetry])
    camera_q=None
    if 'camera_qw' in rows[0]:
        cq=[[float(r[k]) for k in ('camera_qw','camera_qx','camera_qy','camera_qz')] for r in rows]
        _,camera_q=resample(recorded_t,np.zeros((len(rows),3)),cq,targets)
    with (out/'timeline.csv').open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(['index','video_s','simulation_s','view','px','py','pz','pqw','pqx','pqy','pqz','bx','by','bz','bqw','bqx','bqy','bqz','throttle'])
        for i,t in enumerate(targets):writer.writerow([i,i/fps,t,views[i],*pp[i],*pq[i],*bp[i],*bq[i],throttle[i]])
    # Separate file keeps existing trajectory/replay validators compatible.
    if camera_q is not None:
        with (out/'camera_mount.csv').open('w',newline='') as file:
            writer=csv.writer(file);writer.writerow(['qw','qx','qy','qz']);writer.writerows(camera_q)
    tree=ET.parse(run/'world.sdf');world=tree.find('world')
    for uri in world.findall('.//mesh/uri'):
        if uri.text.startswith('model://'):continue
        name=Path(uri.text).name
        if (run/name).exists():uri.text=str(run/name)
        elif (run/'aircraft_meshes'/name).exists():uri.text=str(run/'aircraft_meshes'/name)
    for shader in world.findall('.//shader'):
        if shader.find('vertex') is not None:shader.find('vertex').text=str(run/'wave_shader.glsl')
    for plugin in list(world.findall('plugin')):
        if plugin.get('filename') not in ('gz-sim-physics-system','gz-sim-user-commands-system','gz-sim-scene-broadcaster-system'):world.remove(plugin)
    for model in world.findall('model'):
        static=model.find('static')
        if static is None:static=ET.SubElement(model,'static')
        static.text='true'
        for plugin in list(model.findall('plugin')):model.remove(plugin)
        for sensor in list(model.findall('.//sensor')):
            for link in model.findall('.//link'):
                if sensor in list(link):link.remove(sensor)
        for param in model.findall('.//param'):
            if param.findtext('name')=='t':
                param.find('value').text='0'
                if param.find('type') is None:ET.SubElement(param,'type').text='float'
    gui=world.find('gui')
    for plugin in list(gui):
        if plugin.get('filename') not in ('MinimalScene','GzSceneManager'):gui.remove(plugin)
    plugin=ET.SubElement(gui,'plugin',filename='AirSeaReplay',name='Frame Exact Replay')
    ET.SubElement(plugin,'directory').text=str(out)
    ET.SubElement(plugin,'both_views').text='false' if mode in ('highlights','review') else 'true'
    ET.SubElement(plugin,'onboard_hfov_rad').text=str(json.loads((run/'airborne_camera_calibration.json').read_text())['hfov_rad'])
    tree.write(out/'replay.sdf',encoding='utf-8',xml_declaration=True)
    metadata={'kind':'30 fps presentation re-render of original recorded native FDM trajectories',
        'original_run':str(run),'mode':mode,'fps':fps,'frames':len(targets),'duration_s':len(targets)/fps,
        'segments':[{'simulation_start_s':t,'duration_s':length,'view':view} for t,length,view in segments],
        'aircraft_pose_source':'native_fdm.jsonl, 50 Hz, linear position and normalized quaternion interpolation',
        'boat_pose_source':'original ArduRover DataFlash SIM2 positions and SIM quaternions, 50 Hz',
        'boat_log_vs_archived_camera_truth_p95_m':float(np.percentile(boat_error,95)),
        'boat_native_fdm_vs_dense_gazebo_xy_rms_m':physics_alignment_rms,
        'old_live_camera_scene_latency_note':'Old GUI boat pose can lag the physics clock; replay uses dense physical states, not those delayed scene samples',
        'boat_native_clock_to_gazebo_offset_s':boat_offset,
        'boat_clock_alignment':'Matched native SIM2 XY positions to original dense Gazebo environment telemetry; checked against archived camera truth',
        'wave_time':'original sea-state shader with explicit simulation timestamp per rendered frame',
        'new_flight_or_vision_validation':False,'frame_interpolation_from_video':False,
        'onboard_orientation':'interpolated logged gimbal mount' if camera_q is not None else 'fixed downward mount',
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (run/'native_fdm.jsonl',binary,run/'flight_telemetry.csv',run/'validation.json')},
        'timeline_sha256':hashlib.sha256((out/'timeline.csv').read_bytes()).hexdigest()}
    (out/'replay_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return out

def render(out):
    env=os.environ.copy();env['GZ_PARTITION']='airsea_frame_replay'
    for name in ('GZ_GUI_PLUGIN_PATH','GZ_SIM_SYSTEM_PLUGIN_PATH'):
        env[name]=str(ROOT/'build/airsea_native')+':'+str(ROOT/'install/lib')+':'+env.get(name,'')
    models=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/models'
    env['GZ_SIM_RESOURCE_PATH']=str(models)+':'+env.get('GZ_SIM_RESOURCE_PATH','')
    with (out/'gazebo.log').open('w') as file:
        process=subprocess.Popen(['gz','sim','-r','-v','2',str(out/'replay.sdf')],env=env,stdout=file,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            last=-1;last_progress=time.monotonic()
            while not (out/'complete.json').exists():
                if (out/'error.txt').exists():raise RuntimeError((out/'error.txt').read_text())
                if process.poll() is not None:raise RuntimeError('Replay renderer exited; see gazebo.log')
                progress=out/'progress.txt'
                if progress.exists():
                    count=int(progress.read_text() or '0')
                    if count!=last:last=count;last_progress=time.monotonic()
                if time.monotonic()-last_progress>180:raise TimeoutError('No replay frame progress for 180 seconds')
                time.sleep(.5)
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    metadata=json.loads((out/'replay_manifest.json').read_text())
    ogre=Path.home()/'.gz/rendering/ogre2.log'
    if ogre.exists():
        for line in ogre.read_text(errors='replace').splitlines():
            if 'GL_RENDERER = ' in line:metadata['render_device']=line.split('GL_RENDERER = ',1)[1]
    (out/'replay_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print((out/'complete.json').read_text(),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--mode',choices=['pilot','highlights','review','full'],default='highlights');parser.add_argument('--pilot-seconds',type=float,default=6)
    args=parser.parse_args();render(prepare(args.run,args.out,args.mode,args.pilot_seconds))
