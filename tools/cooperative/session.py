"""Live mutual vision + classical coordination; ordinary SITL controls forces.

Aircraft GPS is a declared acquisition cue for the upward camera only.
Image estimates never read target truth fields from sensor evidence.
"""
from pathlib import Path
import argparse,copy,hashlib,json,math,os,signal,subprocess,sys,time
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/airsea'))
from flight import NativeFDM,PlaneMission,enu,gps
from session import Session as BaseSession,message,quaternion
from world import prepare as unused_baseline
sys.path.insert(0,str(Path(__file__).parent))
from vision import Perception
from control import Coordinator
# Avoid the baseline module's generic name already imported above.
import importlib.util
spec=importlib.util.spec_from_file_location('cooperative_world',Path(__file__).with_name('world.py'))
world_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(world_module)
from pymavlink import mavutil
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg


class Session(BaseSession):
    def __init__(self,out,config,rtf):
        super().__init__(out,rtf);self.config=config;self.vision=Perception(config);self.coordinator=Coordinator(config)
        self.coop_start=None;self.coop_end=None;self.returned=False;self.last_control=-1e9;self.last_aim=-1e9
        self.sensor_file=None;self.tracks=None;self.captures=0;self.boat_msgs={};self.boat_nav=None;self.safe_center=None
        self.bnav_pub=self.gz.advertise('/cooperative/boat/navigation_pose',Pose)
        self.aim_pub={k:self.gz.advertise('/cooperative/aim/'+k,Pose) for k in ('air','boat')}
        self.status_pub=self.gz.advertise('/cooperative/status',StringMsg)
        self.perception_log=(out/'mutual_perception.jsonl').open('w',buffering=1)
        self.control_log=(out/'coordination.jsonl').open('w',buffering=1)
        self.boat_log=(out/'boat_navigation.jsonl').open('w',buffering=1)
        self.aim_log=(out/'gimbal_commands.jsonl').open('w',buffering=1)

    def rover_tick(self,mission):
        now=time.monotonic()
        if now-self.last_rover_heartbeat>.15:
            self.rover.mav.heartbeat_send(6,8,0,0,0)
            self.rover.mav.rc_channels_override_send(self.rover.target_system,self.rover.target_component,1500,1500,1500,1500,1500,1500,1500,1500)
            self.last_rover_heartbeat=now
        while True:
            m=self.rover.recv_match(blocking=False)
            if m is None:break
            typ=m.get_type();self.boat_msgs[typ]=m.to_dict()
            if typ=='HEARTBEAT':self.rover_armed=bool(m.base_mode&128)
        if self.time>42 and not self.rover_armed and now-self.last_rover_arm>3:
            self.rover.set_mode('GUIDED')
            self.rover.mav.command_long_send(self.rover.target_system,self.rover.target_component,400,0,1,0,0,0,0,0,0)
            self.last_rover_arm=now
        p=self.boat_msgs.get('GLOBAL_POSITION_INT');a=self.boat_msgs.get('ATTITUDE')
        if p and a:
            e,n=enu(p['lat']/1e7,p['lon']/1e7)
            # Boat ownship EKF position/attitude. Height uses the water datum.
            self.boat_nav=[e,n,.01116959]
            self.bnav_pub.publish(message(self.time,self.boat_nav,quaternion(a['roll'],-a['pitch'],math.pi/2-a['yaw'])))

    def command_aim(self,kind,xyz,source,stamp):
        self.aim_pub[kind].publish(message(stamp,xyz,[1,0,0,0]))
        self.aim_log.write(json.dumps(dict(t=stamp,kind=kind,aim=list(map(float,xyz)),source=source))+'\n')

    def process_images(self):
        path=self.out/'sensor_frames.jsonl'
        if self.sensor_file is None:
            if not path.exists():return
            self.sensor_file=path.open()
        while True:
            mark=self.sensor_file.tell();line=self.sensor_file.readline()
            if not line.endswith('\n'):self.sensor_file.seek(mark);break
            row=json.loads(line);t=row['t'];results={}
            for kind in ('air','boat'):
                image=cv2.imread(str(self.out/f'frames/{kind}/{row["i"]:06d}.jpg'));assert image is not None
                # Deliberately pass a whitelist: the estimator cannot see truth.
                meta={k:row[kind][k] for k in ('navigation_eye','navigation_origin','R_optical_to_enu','communicated_target_attitude_enu') if k in row[kind]}
                result=self.vision.step(image,meta,kind,t);results[kind]=result
                if result['position'] is not None and result['age_s']<.75:
                    lookahead=float(np.clip(self.time-t+.18,.18,.9))
                    target=np.array(result['position'])+np.array(result['velocity'])*lookahead
                    if result['bearing_enu'] is not None:
                        direction=np.array(result['bearing_enu'])+np.array(result['bearing_rate_enu'])*lookahead
                        direction/=np.linalg.norm(direction)
                        eye=np.array(meta['navigation_eye']);distance=np.linalg.norm(target-eye)
                        target=eye+direction*distance
                    target[2]=float(np.clip(target[2],0,.5)) if kind=='air' else max(10,target[2])
                    source='past_image_bearing_'+result['source'] if result['measurement'] else 'image_filter_prediction_not_detection'
                    self.command_aim(kind,target,source,self.time)
            self.tracks=results;self.captures+=1
            self.perception_log.write(json.dumps(dict(index=row['i'],t=t,**results))+'\n')

    def tick(self,mission,fdm):
        p=mission.messages.get('GLOBAL_POSITION_INT');a=mission.messages.get('ATTITUDE')
        own_plane=None
        if p and a and self.epoch is not None:
            e,n=enu(p['lat']/1e7,p['lon']/1e7);own_plane=[e,n,p['relative_alt']/1000+.1]
            self.nav_pub.publish(message(p['time_boot_ms']/1000+self.epoch,own_plane,quaternion(a['roll'],-a['pitch'],math.pi/2-a['yaw'])))
        self.rover_tick(mission)
        if self.boat_nav is not None and self.time>30 and not self.returned:
            self.capture_pub.publish(Boolean(data=True));self.capture=True;self.capture_requested=True
        elif self.returned and self.capture:
            self.capture_pub.publish(Boolean(data=False));self.capture=False
        self.process_images()
        if own_plane and self.time-self.last_aim>.5 and not self.returned:
            self.last_aim=self.time
            # Shared EKF is used only to acquire/reacquire the upward image.
            # It is never a measurement update in Perception or reported pose.
            tr=self.tracks['boat'] if self.tracks else None
            if tr is None or tr['age_s']+max(0,self.time-tr['t'])>.75:
                self.command_aim('boat',own_plane,'communicated_aircraft_EKF_acquisition_cue',self.time)
            air=self.tracks['air'] if self.tracks else None
            if self.boat_nav is not None and (air is None or air['age_s']+max(0,self.time-air['t'])>.75):
                self.command_aim('air',[*self.boat_nav[:2],.16],'communicated_boat_EKF_acquisition_cue',self.time)
        if mission.current==3 and self.coop_start is None:
            self.coop_start=self.time;mission.mode('GUIDED')
            mission.event('cooperation_started',gazebo_s=self.time,duration_s=self.config['coordination']['duration_s'])
        if self.coop_start is not None and not self.returned and self.time-self.coop_start>=self.config['coordination']['duration_s']:
            self.coop_end=self.time;self.returned=True
            mission.m.mav.mission_set_current_send(mission.m.target_system,mission.m.target_component,4);mission.mode('AUTO')
            mission.event('cooperation_completed_return_to_runway',gazebo_s=self.time)
        if self.coop_start is not None and not self.returned and self.time-self.last_control>=.5 and self.tracks and own_plane and self.boat_nav:
            self.last_control=self.time
            ctrl=self.coordinator.step(self.time,self.tracks['air'],self.tracks['boat'],own_plane,self.boat_nav)
            # The orbit center follows ONLY the aircraft's image estimate of boat.
            tr=self.tracks['air'];center=tr['position'] or self.config['boat_initial_enu_m']
            center=np.array(center)+np.array(tr['velocity'] or [0,0,0])*2
            if tr['age_s']+max(0,self.time-tr['t'])<1:self.safe_center=center.copy()
            elif self.safe_center is not None:center=self.safe_center.copy()
            lat,lon=gps(center[0],center[1]);alt=self.config['coordination']['altitude_m']
            mission.m.mav.mission_item_int_send(mission.m.target_system,mission.m.target_component,0,6,16,2,0,0,0,0,0,int(round(lat*1e7)),int(round(lon*1e7)),alt)
            mission.m.mav.command_long_send(mission.m.target_system,mission.m.target_component,178,0,0,ctrl['plane_airspeed_command_mps'],-1,0,0,0,0)
            speed=ctrl['boat_speed_command_mps']
            # Rover GUIDED velocity; actual thrust comes from ArduRover's
            # existing speed/yaw controllers and the Gazebo propulsion model.
            self.rover.mav.set_position_target_local_ned_send(0,self.rover.target_system,self.rover.target_component,1,0x0DC7,0,0,0,0,speed,0,0,0,0,0,0)
            ctrl.update(orbit_center_enu_m=center.tolist(),air_track_sensor_t=tr['t'],boat_track_sensor_t=self.tracks['boat']['t'],own_plane_enu=own_plane,own_boat_enu=self.boat_nav,
                        plane_measured_airspeed_mps=mission.messages.get('VFR_HUD',{}).get('airspeed'),boat_measured_speed_mps=math.hypot(self.boat_msgs.get('GLOBAL_POSITION_INT',{}).get('vx',0),self.boat_msgs.get('GLOBAL_POSITION_INT',{}).get('vy',0))/100)
            self.control_log.write(json.dumps(ctrl)+'\n')
            if int(self.time)%10==0:print('COOP',json.dumps(ctrl),flush=True)
            lines=[f"t={self.time:.2f} s | {ctrl['mode']} | Sea State 1 | original BlueBoat verified"]
            for k,label in (('air','AIR -> BOAT'),('boat','BOAT -> FW')):
                r=self.tracks[k];pos=r['position'];vel=r['velocity']
                age=r['age_s']+max(0,self.time-r['t'])
                lines.append(f"{label}: {r['source']} | last image t={r['t']:.2f} | observation age {age:.2f} s")
                if pos:
                    rp=r['relative_position_enu'];rv=r['relative_velocity_enu']
                    lines.append('  Relative ENU: '+', '.join(f'{x:+.2f}' for x in rp)+' m | relative velocity: '+', '.join(f'{x:+.2f}' for x in rv)+' m/s')
            lines.append(f"FW airspeed: command {ctrl['plane_airspeed_command_mps']:.2f} / measured {ctrl['plane_measured_airspeed_mps'] or 0:.2f} m/s")
            lines.append(f"Boat speed: command {speed:.2f} / measured {ctrl['boat_measured_speed_mps']:.2f} m/s")
            self.status_pub.publish(StringMsg(data='\n'.join(lines)))
        if self.returned and self.rover_armed:
            self.rover.mav.set_position_target_local_ned_send(0,self.rover.target_system,self.rover.target_component,1,0x0DC7,0,0,0,0,0,0,0,0,0,0,0)
        target=self.config['coordination']['capture_rtf'] if self.time>58 and not self.returned else self.base_rtf
        if target!=self.rtf:
            subprocess.run([sys.executable,str(ROOT/'tools/airsea/physics_rate.py'),str(target)],check=True,timeout=15);self.rtf=target

    def close(self):
        for f in (self.perception_log,self.control_log,self.boat_log,self.aim_log,self.estimate_log):f.close()
        if self.sensor_file:self.sensor_file.close()


def run(out,rtf=.8,duration=None):
    config=json.loads((ROOT/'config/cooperative.json').read_text())
    if duration is not None:config['coordination']['duration_s']=duration
    out=out.resolve();out.mkdir(parents=True,exist_ok=False);world=world_module.prepare(out,config,rtf)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    sources=[*sorted((ROOT/'tools/cooperative').glob('*.py')),*sorted((ROOT/'tools/airsea').glob('*.py')),
             *sorted((ROOT/'tools').glob('*.py')),*sorted((ROOT/'tools/airsea/native').glob('*')),
             ROOT/'tools/run_cooperative_wsl.sh',ROOT/'tools/build_airsea_wsl.sh',ROOT/'tools/graphics_wsl.sh',
             ROOT/'config/cooperative.json',ROOT/'config/arduplane.parm',
             ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/config/ardurover.parm']
    sources=[p for p in sources if p.is_file()]
    hashes={}
    for source in sources:
        relative=source.relative_to(ROOT).as_posix();target=snapshot/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes());hashes[relative]=hashlib.sha256(source.read_bytes()).hexdigest()
    (out/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    libraries={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'build/airsea_native').glob('libAirSea*.so')}
    (out/'native_library_hashes.json').write_text(json.dumps(libraries,indent=2)+'\n')
    params=(ROOT/'config/arduplane.parm').read_text()+f"\nWP_LOITER_RAD {config['coordination']['orbit_radius_m']}\n"
    (out/'arduplane_used.parm').write_text(params)
    rover_params=(ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/config/ardurover.parm').read_text()
    rover_params+=f"\n# Match the supervisor's declared achievable GUIDED speed bound.\nWP_SPEED {config['coordination']['boat_speed_max_mps']}\n"
    (out/'ardurover_used.parm').write_text(rover_params)
    session=Session(out,config,rtf);children=[];logs=[];fdm=None;mission=None;status='failed';error=None
    def launch(cmd,name):
        log=(out/(name+'.log')).open('w');logs.append(log);folder=out/name;folder.mkdir(exist_ok=True)
        p=subprocess.Popen(cmd,cwd=folder,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    try:
        sim=launch(['gz','sim','-r','-v','2',str(world)],'gazebo')
        deadline=time.monotonic()+90
        while session.time is None:
            if sim.poll() is not None:raise RuntimeError('Gazebo startup failed')
            if time.monotonic()>deadline:raise TimeoutError('No Gazebo clock')
            time.sleep(.1)
        roverruntime=Path.home()/'blueboat_sitl';planeruntime=Path.home()/'blueboat_plane_sitl'
        launch([str(roverruntime/'ardurover'),'--model','JSON','--home',','.join(map(str,config['home'])),'--rate','250','--speedup','1','--instance','6','--sim-port-out','9062','--defaults',str(out/'ardurover_used.parm'),'--serial0','tcp:5890','--serial1','none','--serial2','none','--wipe'],'ardurover')
        session.rover=mavutil.mavlink_connection('tcp:127.0.0.1:5890',source_system=254,retries=30)
        if session.rover.wait_heartbeat(timeout=40) is None:raise TimeoutError('No rover heartbeat')
        for mid,hz in ((33,20),(30,20),(74,10)):
            session.rover.mav.command_long_send(session.rover.target_system,session.rover.target_component,511,0,mid,1e6/hz,0,0,0,0,0)
        fdm=NativeFDM(out,on_state=session.on_state,clock=session.plane_clock)
        plane=launch([str(planeruntime/'arduplane'),'--model','plane','--home',','.join(map(str,config['home'])),'--rate','400','--speedup','1','--instance','12','--slave','1','--defaults',str(out/'arduplane_used.parm'),'--serial0','tcp:5880','--serial1','none','--serial2','none','--wipe'],'arduplane')
        mission=PlaneMission(out,config=config)
        def tick(m,f):
            if sim.poll() is not None:raise RuntimeError('Gazebo exited; inspect gazebo.log')
            session.tick(m,f)
        mission.run(fdm,timeout=7200,process=plane,on_tick=tick);status='completed'
    except BaseException as exc:error=repr(exc);raise
    finally:
        if mission:mission.save()
        session.capture_pub.publish(Boolean(data=False))
        if fdm:fdm.close()
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        # The renderer can finish a pair between the last mission tick and
        # shutdown. Drain it after the writer exits so every JPEG pair has a
        # matching perception record; do not drop it from the validator.
        session.process_images()
        for log in logs:log.close()
        result=dict(status=status,error=error,epoch_offset_s=session.epoch,cooperation_start_s=session.coop_start,cooperation_end_s=session.coop_end,normal_plane_arming=bool(mission and mission.armed_once),rover_armed=session.rover_armed,processed_camera_pairs=session.captures,requirements='2026-11-12 professor next steps',perception_uses_target_truth=False,upward_acquisition_cue='communicated aircraft EKF; separate from image estimation',config=config)
        (out/'session_manifest.json').write_text(json.dumps(result,indent=2)+'\n');session.close();print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--rtf',type=float,default=.8);p.add_argument('--duration',type=float)
    a=p.parse_args();run(a.out,a.rtf,a.duration)
