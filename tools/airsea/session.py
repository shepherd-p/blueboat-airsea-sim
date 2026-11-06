"""Couple live native ArduPlane FDM, Gazebo Sea State 1 and image-only tracking."""
from pathlib import Path
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean

from flight import NativeFDM,PlaneMission,ROOT,CONFIG,enu
from world import prepare


def quaternion(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]


def native_rotation(q):
    w,x,y,z=q
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1,min(1,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return quaternion(roll,-pitch,math.pi/2-yaw)


def message(t,xyz,q):
    p=Pose();p.header.stamp.sec=int(t);p.header.stamp.nsec=int((t-int(t))*1e9)
    p.position.x,p.position.y,p.position.z=map(float,xyz)
    p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z=map(float,q)
    return p


class Session:
    def __init__(self,out,rtf=.6):
        self.out=out;self.gz=Node();self.time=None;self.epoch=None
        self.estimates=[];self.updated=False;self.capture=False;self.last_nav=-1;self.last_rover_heartbeat=0
        self.rover=None;self.rover_armed=False;self.last_rover_arm=-100
        self.truth_pub=self.gz.advertise('/airsea/aircraft/truth_pose',Pose)
        self.nav_pub=self.gz.advertise('/airsea/aircraft/navigation_pose',Pose)
        self.throttle_pub=self.gz.advertise('/airsea/aircraft/throttle',Double)
        self.capture_pub=self.gz.advertise('/airsea/capture/active',Boolean)
        self.capture_requested=False
        self.base_rtf=rtf;self.rtf=rtf;self.rate_changes=[]
        self.gz.subscribe(Clock,'/clock',self.on_clock)
        self.gz.subscribe(Clock,'/world/airsea/clock',self.on_clock)
        self.gz.subscribe(Pose,'/airsea/boat/image_estimate',self.on_estimate)
        self.estimate_log=(out/'image_track.jsonl').open('w',buffering=1)

    def on_clock(self,c):self.time=c.sim.sec+c.sim.nsec*1e-9

    def on_estimate(self,p):
        t=p.header.stamp.sec+p.header.stamp.nsec*1e-9
        row=dict(t=t,east=p.position.x,north=p.position.y,up=p.position.z)
        self.estimates.append(row);self.estimate_log.write(json.dumps(row)+'\n')

    def on_state(self,s):
        if self.epoch is None:self.epoch=self.time-s['timestamp']
        north,east,down=s['position']
        self.truth_pub.publish(message(s['timestamp']+self.epoch,[east,north,-down],native_rotation(s['quaternion'])))

    def plane_clock(self):
        return self.time-(self.epoch or 0)

    def tick(self,mission,fdm):
        msgs=mission.messages;pos=msgs.get('GLOBAL_POSITION_INT');att=msgs.get('ATTITUDE')
        if pos and att and abs(pos['lat']/1e7-CONFIG['home'][0])<.02 and self.epoch is not None:
            t=pos['time_boot_ms']/1000
            if t>self.last_nav:
                e,n=enu(pos['lat']/1e7,pos['lon']/1e7)
                q=quaternion(att['roll'],-att['pitch'],math.pi/2-att['yaw'])
                self.nav_pub.publish(message(t+self.epoch,[e,n,pos['relative_alt']/1000+.1],q))
                self.throttle_pub.publish(Double(data=msgs.get('VFR_HUD',{}).get('throttle',0)/100))
                self.last_nav=t
        if fdm.state and fdm.state['timestamp']>30 and not self.capture:
            self.capture_pub.publish(Boolean(data=True));self.capture_requested=True
            if (self.out/'airborne_observations.csv').exists():
                self.capture=True;mission.event('live_native_camera_started')
            elif fdm.state['timestamp']>34.5:raise RuntimeError('Native airborne camera did not produce frames')
        # Replan the second pass from IMAGE observations. No boat truth is received here.
        if not self.updated and mission.current==4 and len(self.estimates)>=5:
            samples=self.estimates[-min(40,len(self.estimates)):]
            t=np.array([r['t'] for r in samples]);y=np.array([r['north'] for r in samples])
            slope=float(np.polyfit(t-t[-1],y,1)[0]) if np.ptp(t)>1.5 else 0
            slope=float(np.clip(slope,-.4,.4))
            prediction=float(np.median(y[-5:])+35*slope)
            prediction=float(np.clip(prediction,135,220))
            mission.update_waypoint(6,550,prediction,25)
            mission.event('image_track_acquired',observations=len(self.estimates),predicted_second_pass_north=prediction,estimated_north_speed=slope)
            self.updated=True
        if self.rover:self.rover_tick(mission)
        # Slow only camera survey passes. This affects wall-clock runtime,
        # never physics step, autopilot clock or replay timestamps.
        target=self.base_rtf
        if fdm.state:
            north,east,down=fdm.state['position']
            if mission.current in (3,6) and 260<east<390:target=.05
        if target!=self.rtf:
            subprocess.run([sys.executable,str(ROOT/'tools/airsea/physics_rate.py'),str(target)],check=True,timeout=15)
            self.rtf=target;self.rate_changes.append(dict(native_s=fdm.state['timestamp'],real_time_factor=target))
            mission.event('camera_sampling_playback_rate',real_time_factor=target)

    def rover_tick(self,mission):
        now=time.monotonic()
        if now-self.last_rover_heartbeat>.15:
            self.rover.mav.heartbeat_send(6,8,0,0,0)
            self.rover.mav.rc_channels_override_send(self.rover.target_system,self.rover.target_component,1500,1500,1500,1500,1500,1500,1500,1500)
            self.last_rover_heartbeat=now
        while True:
            msg=self.rover.recv_match(blocking=False)
            if msg is None:break
            if msg.get_type()=='HEARTBEAT':self.rover_armed=bool(msg.base_mode&128)
        if self.time>42 and not self.rover_armed and now-self.last_rover_arm>3:
            self.rover.set_mode('MANUAL')
            self.rover.mav.command_long_send(self.rover.target_system,self.rover.target_component,400,0,1,0,0,0,0,0,0)
            self.last_rover_arm=now


def run(out,rtf=.6,pilot=False):
    out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    world=prepare(out,rtf)
    session=Session(out,rtf);children=[];logs=[];fdm=None;mission=None
    status='failed';error=None
    def launch(cmd,name):
        log=(out/(name+'.log')).open('w');logs.append(log)
        working=out/name;working.mkdir(exist_ok=True)
        p=subprocess.Popen(cmd,cwd=working,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    try:
        sim=launch(['gz','sim','-r','-v','2',str(world)],'gazebo')
        deadline=time.monotonic()+90
        while session.time is None:
            if sim.poll() is not None:raise RuntimeError('Gazebo exited during startup')
            if time.monotonic()>deadline:raise TimeoutError('No Gazebo clock')
            time.sleep(.1)
        runtime=Path(os.environ.get('BLUEBOAT_PLANE_RUNTIME',str(Path.home()/'blueboat_plane_sitl')))
        roverruntime=Path(os.environ.get('BLUEBOAT_SITL_RUNTIME',str(Path.home()/'blueboat_sitl')))
        rover=launch([str(roverruntime/'ardurover'),'--model','JSON','--home',','.join(map(str,CONFIG['home'])),
            '--rate','250','--speedup','1','--instance','6','--sim-port-out','9062',
            '--defaults',str(ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/config/ardurover.parm'),
            '--serial0','tcp:5890','--serial1','none','--serial2','none','--wipe'],'ardurover')
        session.rover=mavutil.mavlink_connection('tcp:127.0.0.1:5890',source_system=254,retries=30)
        if session.rover.wait_heartbeat(timeout=40) is None:raise TimeoutError('No ArduRover heartbeat')
        fdm=NativeFDM(out,on_state=session.on_state,clock=session.plane_clock)
        plane=launch([str(runtime/'arduplane'),'--model','plane','--home',','.join(map(str,CONFIG['home'])),
            '--rate','400','--speedup','1','--instance','12','--slave','1',
            '--defaults',str(ROOT/'config/arduplane.parm'),'--serial0','tcp:5880','--serial1','none','--serial2','none','--wipe'],'arduplane')
        mission=PlaneMission(out)
        def tick(m,f):
            session.tick(m,f)
            if pilot and f.state and f.state['timestamp']>100:raise InterruptedError('Pilot capture complete; full landing not requested in this pilot')
        mission.run(fdm,timeout=2400,process=plane,on_tick=tick)
        status='completed'
    except InterruptedError as exc:
        status='pilot_completed';error=str(exc)
    except BaseException as exc:
        error=repr(exc);raise
    finally:
        if mission:mission.save()
        if session.capture_requested:
            session.capture_pub.publish(Boolean(data=False))
            deadline=time.monotonic()+45
            while not (out/'capture_closed.txt').exists() and time.monotonic()<deadline:
                session.capture_pub.publish(Boolean(data=False));time.sleep(.1)
        if fdm:fdm.close()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        session.estimate_log.close()
        result=dict(status=status,error=error,sea_state=1,epoch_offset_s=session.epoch,
            normal_plane_arming=mission.armed_once if mission else False,rover_armed=session.rover_armed,
            image_observations=len(session.estimates),image_based_second_pass_replanned=session.updated,
            aircraft_physics='ArduPlane 4.6.3 native plane aerodynamic and ground-contact FDM; read-only JSON slave -> Gazebo rendering',
            boat_physics='Gazebo 6DOF buoyancy, 32 JONSWAP waves, wind/current and ArduRover 4.7.2',
            estimator='Native rendered pixels -> AprilTag ID0 -> IPPE PnP -> MAVLink EKF ownship navigation + calibrated mount -> ENU boat position; truth evaluation only',
            pilot=pilot)
        result['playback_rate_changes']=session.rate_changes
        (out/'session_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--rtf',type=float,default=.6);p.add_argument('--pilot',action='store_true')
    args=p.parse_args();run(args.out,args.rtf,args.pilot)
