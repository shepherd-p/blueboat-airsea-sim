"""Real ArduPlane AUTO mission and read-only native FDM state interface.

No trajectory is scripted. ArduPlane closes its aerodynamic/ground-contact
loop internally. Its documented JSON master/slave output supplies truth to
the Gazebo pose/clock bridge. Slave servo values never control the master.
"""
from pathlib import Path
import argparse
import csv
import json
import math
import os
import signal
import socket
import struct
import subprocess
import threading
import time

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT / 'config/airsea.json').read_text())
RADIUS = 6378137.0


def gps(east, north):
    lat, lon = CONFIG['home'][:2]
    return lat + math.degrees(north / RADIUS), lon + math.degrees(east / (RADIUS * math.cos(math.radians(lat))))


def enu(lat, lon):
    lat0, lon0 = CONFIG['home'][:2]
    return math.radians(lon-lon0)*RADIUS*math.cos(math.radians(lat0)), math.radians(lat-lat0)*RADIUS


class NativeFDM:
    def __init__(self, out, on_state=None, clock=None):
        self.out = Path(out)
        self.callback = on_state
        self.clock = clock
        self.state = None
        self.running = True
        self.error = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(.15)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        try:
            sequence = 0
            last_log = -1
            with (self.out/'native_fdm.jsonl').open('w', buffering=1) as log:
                while self.running:
                    if self.state is not None and self.clock:
                        while self.running and self.state['timestamp'] > self.clock() + .012:
                            time.sleep(.001)
                    # JSON_Master accepts only loopback. Master remains SIM_JSON_MASTER=0.
                    packet=struct.pack('<HHI16H',18458,400,sequence,*([1500,1500,1000,1500]+[1500]*12))
                    self.socket.sendto(packet,('127.0.0.1',9012))
                    sequence += 1
                    try:
                        raw,_=self.socket.recvfrom(8192)
                    except socket.timeout:
                        continue
                    state=json.loads(raw)
                    self.state=state
                    if state['timestamp']-last_log>=.019:
                        log.write(json.dumps(state,separators=(',',':'))+'\n')
                        last_log=state['timestamp']
                        if self.callback: self.callback(state)
        except Exception as exc:
            self.error=repr(exc)
            print('FDM BRIDGE ERROR',repr(exc),flush=True)

    def close(self):
        self.running=False
        self.thread.join(timeout=3)
        self.socket.close()


class PlaneMission:
    def __init__(self,out,port=5880):
        self.out=Path(out)
        self.m=mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}',source_system=255,retries=50)
        if self.m.wait_heartbeat(timeout=45) is None:
            raise TimeoutError('No ArduPlane heartbeat; inspect arduplane.log')
        self.messages={}
        self.events=[]
        self.rows=[]
        self.last_heartbeat=0
        self.last_rc=0
        self.last_log=-1
        self.start_wall=time.monotonic()
        self.arm_requested=False
        self.armed_once=False
        self.current=0
        self.finished=False
        self.last_print=-100
        self.pending_updates={}
        for msgid,hz in ((33,20),(30,20),(74,10),(36,10),(42,5),(193,5),(245,5),(62,5),(24,5),(1,2)):
            self.m.mav.command_long_send(self.m.target_system,self.m.target_component,511,0,msgid,1e6/hz,0,0,0,0,0)

    def event(self,name,**extra):
        stamp=self.messages.get('SYSTEM_TIME',{}).get('time_boot_ms',0)/1000
        record=dict(sim_s=stamp,event=name,**extra)
        self.events.append(record)
        print(json.dumps(record),flush=True)

    def rc(self):
        self.m.mav.rc_channels_override_send(self.m.target_system,self.m.target_component,1500,1500,1000,1500,1000,1500,1500,1500)

    def mode(self,name):
        self.m.set_mode(name)

    def update_waypoint(self,seq,east,north,alt):
        self.pending_updates[seq]=(east,north,alt)
        self.m.mav.mission_write_partial_list_send(self.m.target_system,self.m.target_component,seq,seq)
        self.event('image_based_waypoint_update_requested',seq=seq,east=east,north=north,alt=alt)

    def upload(self):
        self.rc()
        items=CONFIG['mission']
        self.m.mav.mission_clear_all_send(self.m.target_system,self.m.target_component)
        time.sleep(.1)
        self.m.mav.mission_count_send(self.m.target_system,self.m.target_component,len(items))
        self.event('mission_upload_started',target_system=self.m.target_system,target_component=self.m.target_component)
        deadline=time.monotonic()+90
        sent=set()
        retry=time.monotonic()
        while time.monotonic()<deadline:
            if time.monotonic()-retry>2:
                self.m.mav.heartbeat_send(6,8,0,0,0);self.rc()
                if not sent:self.m.mav.mission_count_send(self.m.target_system,self.m.target_component,len(items))
                retry=time.monotonic()
            msg=self.m.recv_match(blocking=True,timeout=.2)
            if msg is None: continue
            typ=msg.get_type()
            if typ in ('MISSION_REQUEST','MISSION_REQUEST_INT'):
                seq=msg.seq;item=items[seq];east,north,alt=item['enu_m'];lat,lon=gps(east,north)
                self.m.mav.mission_item_int_send(self.m.target_system,self.m.target_component,seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,item['command'],int(seq==0),1,
                    item.get('param1',0),0,0,0,int(round(lat*1e7)),int(round(lon*1e7)),alt)
                sent.add(seq)
                self.event('mission_item_sent',seq=seq)
            elif typ=='MISSION_ACK' and msg.type==0 and len(sent)==len(items):
                self.event('mission_uploaded',items=len(items));return
        raise TimeoutError('Mission upload incomplete')

    def tick(self):
        now=time.monotonic()
        if now-self.last_heartbeat>.5:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            self.last_heartbeat=now
        if now-self.last_rc>.1: self.rc();self.last_rc=now
        for _ in range(1000):
            msg=self.m.recv_match(blocking=False)
            if msg is None: break
            typ=msg.get_type();self.messages[typ]=msg.to_dict()
            if typ in ('MISSION_REQUEST','MISSION_REQUEST_INT') and msg.seq in self.pending_updates:
                east,north,alt=self.pending_updates[msg.seq];lat,lon=gps(east,north)
                self.m.mav.mission_item_int_send(self.m.target_system,self.m.target_component,msg.seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,16,0,1,0,0,0,0,int(round(lat*1e7)),int(round(lon*1e7)),alt)
            elif typ=='MISSION_ACK' and self.pending_updates:
                self.event('image_based_waypoint_update_ack',result=msg.type,sequences=list(self.pending_updates))
                if msg.type==0:self.pending_updates.clear()
            if typ=='STATUSTEXT': self.event('status',text=msg.text)
            elif typ=='COMMAND_ACK' and msg.command==400: self.event('arming_ack',result=msg.result)
            elif typ=='MISSION_CURRENT' and msg.seq!=self.current:
                self.current=msg.seq;self.event('mission_current',seq=msg.seq)
            elif typ=='HEARTBEAT':
                armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.armed_once |= armed
                if self.arm_requested and self.armed_once and not armed: self.finished=True
        pos=self.messages.get('GLOBAL_POSITION_INT');att=self.messages.get('ATTITUDE');vfr=self.messages.get('VFR_HUD',{})
        if pos and att:
            t=pos['time_boot_ms']/1000
            if t-self.last_log>.045:
                east,north=enu(pos['lat']/1e7,pos['lon']/1e7)
                self.rows.append(dict(t=t,east=east,north=north,relative_alt=pos['relative_alt']/1000,
                    roll=att['roll'],pitch=att['pitch'],yaw=att['yaw'],groundspeed=vfr.get('groundspeed',0),
                    airspeed=vfr.get('airspeed',0),throttle=vfr.get('throttle',0),mission=self.current,
                    armed=int(bool(self.messages.get('HEARTBEAT',{}).get('base_mode',0)&128))))
                self.last_log=t
                if t-self.last_print>10:
                    print('FLIGHT',self.rows[-1],flush=True);self.last_print=t
        return self.messages

    def run(self,fdm,timeout=600,process=None,on_tick=None):
        self.upload()
        wall_deadline=time.monotonic()+timeout
        self.mode('MANUAL')
        last_arm=-100
        while time.monotonic()<wall_deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError('ArduPlane exited; inspect arduplane.log')
            self.tick()
            if on_tick:on_tick(self,fdm)
            t=fdm.state['timestamp'] if fdm.state else 0
            if t>35 and not self.armed_once and time.monotonic()-last_arm>3:
                self.mode('AUTO')
                self.m.mav.mission_set_current_send(self.m.target_system,self.m.target_component,1)
                self.m.mav.command_long_send(self.m.target_system,self.m.target_component,400,0,1,0,0,0,0,0,0)
                self.arm_requested=True;last_arm=time.monotonic()
                self.event('normal_arm_requested')
            if self.finished:
                self.event('autonomous_disarm_after_landing');return
            if self.armed_once and self.current==1 and fdm.state:
                north,east,down=fdm.state['position']
                if east>220 and -down<1:
                    raise RuntimeError('Failed takeoff: ground run exceeded 220 m')
            if fdm.error: raise RuntimeError(fdm.error)
            if fdm.state is None and time.monotonic()-self.start_wall>15:
                raise TimeoutError('No native FDM state; another JSON slave may be holding port 9012')
            if t>700: raise TimeoutError('Flight exceeded 700 simulation seconds')
            time.sleep(.005)
        raise TimeoutError('Flight wall-clock timeout')

    def save(self):
        (self.out/'flight_events.json').write_text(json.dumps(self.events,indent=2)+'\n')
        if self.rows:
            with (self.out/'flight_telemetry.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=self.rows[0]);writer.writeheader();writer.writerows(self.rows)


def native_trial(out,speedup=6):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    runtime=Path(os.environ.get('BLUEBOAT_PLANE_RUNTIME',str(Path.home()/'blueboat_plane_sitl')))
    fdm=NativeFDM(out)
    log=(out/'arduplane.log').open('w')
    cmd=[str(runtime/'arduplane'),'--model','plane','--home',','.join(map(str,CONFIG['home'])),
         '--rate','400','--speedup',str(speedup),'--instance','12','--slave','1',
         '--defaults',str(ROOT/'config/arduplane.parm'),'--serial0','tcp:5880','--serial1','none',
         '--serial2','none','--wipe']
    p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT)
    mission=None
    try:
        mission=PlaneMission(out)
        mission.run(fdm,timeout=600,process=p)
    finally:
        if mission: mission.save()
        p.terminate();p.wait(timeout=15);fdm.close();log.close()


if __name__=='__main__':
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--speedup',type=float,default=6)
    args=parser.parse_args();native_trial(args.out,args.speedup)
