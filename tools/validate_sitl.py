#!/usr/bin/env python3
"""Drive the actual ArduRover firmware; measure resulting Gazebo motion."""
import argparse,csv,json,math,time,os,signal
from pathlib import Path
import numpy as np
import rclpy
from pymavlink import mavutil
from validate_ros import Probe
from std_msgs.msg import Float64
ROOT=Path(__file__).resolve().parents[1]
class Test:
    def __init__(self,out):
        self.out=out;self.node=Probe();self.m=mavutil.mavlink_connection('tcp:127.0.0.1:'+os.environ.get('BLUEBOAT_MAVLINK_PORT','5760'),source_system=255)
        for side in ['left','right']:
            self.node.create_subscription(Float64,'/blueboat/propulsion/'+side+'/applied_thrust',lambda m,s=side:setattr(self.node,s,m.data),10)
        assert self.m.wait_heartbeat(timeout=20),'No real ArduRover heartbeat'
        self.armed=False;self.mode=0;self.events=[];self.lastHeartbeat=-1;self.latest={};self.phases=[];self.heartbeat_enabled=True
        self.log=(out/'mavlink.jsonl').open('w')
        for msg_id in [30,32,33,36,193,148]:self.m.mav.command_long_send(self.m.target_system,self.m.target_component,511,0,msg_id,100000,0,0,0,0,0)
    def tick(self):
        rclpy.spin_once(self.node,timeout_sec=.005)
        if self.heartbeat_enabled and time.monotonic()-self.lastHeartbeat> .5:
            self.lastHeartbeat=time.monotonic();self.m.mav.heartbeat_send(6,8,0,0,0)
        for _ in range(100):
            msg=self.m.recv_match(blocking=False)
            if not msg:break
            d=msg.to_dict();kind=msg.get_type();self.latest[kind]=d
            if kind=='HEARTBEAT':self.armed=bool(msg.base_mode&128);self.mode=msg.custom_mode
            if kind in ['STATUSTEXT','COMMAND_ACK']:self.events.append(d);print(kind,d,flush=True)
            if kind in ['HEARTBEAT','ATTITUDE','LOCAL_POSITION_NED','GLOBAL_POSITION_INT','SERVO_OUTPUT_RAW','EKF_STATUS_REPORT','STATUSTEXT','COMMAND_ACK','AUTOPILOT_VERSION']:
                self.log.write(json.dumps({'gazebo_t':self.node.clock,'message':d},default=str)+'\n')
    def rc(self,steer=1500,throttle=1500):
        self.m.mav.rc_channels_override_send(self.m.target_system,self.m.target_component,steer,65535,throttle,65535,65535,65535,65535,65535)
    def phase(self,name,duration,action=lambda:None,goal=None):
        n=self.node;n.phase=name;n.target=(0,0);start=n.clock;deadline=time.monotonic()+duration*40+30;last=-1
        while n.clock-start<duration:
            self.tick()
            if n.clock-last>=.08:action();last=n.clock
            if goal and math.hypot(n.odom.pose.pose.position.x-goal[0],n.odom.pose.pose.position.y-goal[1])<1.2:break
            if time.monotonic()>deadline:raise TimeoutError(name)
        self.phases.append({'name':name,'start':start,'end':n.clock});print(f'{name}: t={n.clock:.1f} u={n.odom.twist.twist.linear.x:.3f} r={n.odom.twist.twist.angular.z:.3f}',flush=True)
    def mode_set(self,name):
        wanted=self.m.mode_mapping()[name];self.m.set_mode(name);deadline=time.monotonic()+8
        while self.mode!=wanted:
            self.tick()
            if time.monotonic()>deadline:raise TimeoutError('Mode '+name)
    def run(self):
        n=self.node;deadline=time.monotonic()+20
        while n.clock is None or n.odom is None:
            self.tick()
            if time.monotonic()>deadline:raise TimeoutError('No Gazebo odometry')
        self.mode_set('MANUAL');self.phase('warmup',40,lambda:self.rc())
        self.m.mav.command_long_send(self.m.target_system,self.m.target_component,400,0,1,0,0,0,0,0,0)
        deadline=time.monotonic()+12
        while not self.armed:
            self.tick();self.rc()
            if time.monotonic()>deadline:raise RuntimeError('Normal arming rejected; see MAVLink status log')
        for name,duration,steer,throttle in [('manual_forward',10,1500,1600),('neutral',5,1500,1500),('manual_right',8,1600,1500),('manual_left',8,1400,1500),('manual_reverse',10,1500,1400),('neutral_after_reverse',5,1500,1500)]:
            self.phase(name,duration,lambda s=steer,t=throttle:self.rc(s,t))
        position=n.odom.pose.pose.position;goal=(position.x+12,position.y+6)
        self.rc();self.mode_set('GUIDED')
        def waypoint():self.m.mav.set_position_target_local_ned_send(0,self.m.target_system,self.m.target_component,1,3576,goal[1],goal[0],0,0,0,0,0,0,0,0,0)
        self.phase('guided_waypoint',45,waypoint,goal)
        actual=n.odom.pose.pose.position;distance=math.hypot(actual.x-goal[0],actual.y-goal[1])
        self.mode_set('LOITER');loiterCenter=(actual.x,actual.y);loiterStart=len(n.rows);self.phase('loiter',25)
        loiterDistance=max(math.hypot(r['x']-loiterCenter[0],r['y']-loiterCenter[1]) for r in n.rows[loiterStart:])
        self.mode_set('HOLD');self.phase('hold',5)
        self.mode_set('MANUAL');self.phase('failsafe_prepare',4,lambda:self.rc(throttle=1560))
        self.heartbeat_enabled=False;self.phase('gcs_loss',10);gcsHeld=self.mode==4
        self.heartbeat_enabled=True;self.phase('gcs_restored',2,lambda:self.rc())
        self.mode_set('MANUAL');self.phase('bridge_timeout_prepare',4,lambda:self.rc(throttle=1560))
        pid=int((self.out/'ardurover.pid').read_text());assert Path(f'/proc/{pid}/exe').resolve().name=='ardurover'
        os.kill(pid,signal.SIGSTOP)
        try:self.phase('firmware_stopped',2);bridgeZero=abs(n.left)<.001 and abs(n.right)<.001
        finally:os.kill(pid,signal.SIGCONT)
        self.phase('firmware_restored',2,lambda:self.rc())
        self.m.mav.command_long_send(self.m.target_system,self.m.target_component,400,0,0,0,0,0,0,0,0)
        self.phase('disarmed',3)
        summary={}
        for p in self.phases:
            rows=[r for r in n.rows if r['phase']==p['name'] and r['t']>=p['end']-3]
            summary[p['name']]={k:float(np.mean([r[k] for r in rows])) for k in ['u','r','roll','pitch','x','y','z','left_N','right_N']}
        checks={'normal_arming_accepted':any(e.get('command')==400 and e.get('result')==0 for e in self.events),
                'forward':summary['manual_forward']['u']>.3,'reverse':summary['manual_reverse']['u']<-.2,
                'right_turn':summary['manual_right']['r']<-.1,'left_turn':summary['manual_left']['r']>.1,
                'guided_waypoint_reached':distance<1.5,'disarmed':not self.armed,
                'loiter_contained':loiterDistance<4.,'gcs_failsafe_hold':gcsHeld,'bridge_packet_watchdog':bridgeZero,
                'gcs_loss_zero_thrust':abs(summary['gcs_loss']['left_N'])<.001 and abs(summary['gcs_loss']['right_N'])<.001,
                'actual_actuator_feedback':summary['manual_forward']['left_N']>1 and summary['manual_forward']['right_N']>1,
                'finite_physics':all(math.isfinite(r[k]) for r in n.rows for k in ['u','r','x','y','z','roll','pitch']),
                'bounded_attitude':max(abs(r[k]) for r in n.rows for k in ['roll','pitch'])<.6,
                'ekf_available':'EKF_STATUS_REPORT' in self.latest}
        with (self.out/'telemetry.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(n.rows[0]));w.writeheader();w.writerows(n.rows)
        result={'passed':all(checks.values()),'checks':checks,'guided_goal_enu_m':goal,'guided_final_distance_m':distance,'loiter_max_distance_m':loiterDistance,'phase_summary':summary,'phases':self.phases,'samples':len(n.rows),'scope':'Real ArduRover V4.7.2 JSON + Gazebo wind/waves/current. EKF3 enabled; normal arming checks. Not real-boat calibration.'}
        (self.out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
        self.log.close()
        if not result['passed']:raise SystemExit(1)
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=ROOT/'results/sitl');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();test=Test(a.out)
    try:test.run()
    finally:
        test.rc();test.m.mav.command_long_send(test.m.target_system,test.m.target_component,400,0,0,0,0,0,0,0,0)
        test.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
