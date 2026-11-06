#!/usr/bin/env python3
"""Record a real command/feedback sequence, synchronised to the GUI recorder."""
import csv
import json
import math
import time
import shutil
from pathlib import Path
import rclpy
from validate_ros import Probe
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.cameratrack_pb2 import CameraTrack
from gz.msgs10.time_pb2 import Time

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/video_demo'
PHASES=[('Settled / zero command',3,0.,0.),('Forward',8,.8,0.),
        ('Stop',4,0.,0.),('Counter-clockwise turn',6,0.,.4),
        ('Clockwise turn',6,0.,-.4),('Reverse',6,-.5,0.),('Final stop',4,0.,0.)]

class DemoProbe(Probe):
    def __init__(self):
        self.live=(OUT/'telemetry_live.csv').open('w',newline='')
        self.writer=None
        super().__init__()
    def on_odom(self,msg):
        super().on_odom(msg)
        if self.writer is None:
            self.writer=csv.DictWriter(self.live,fieldnames=list(self.rows[-1]));self.writer.writeheader()
        self.writer.writerow(self.rows[-1]);self.live.flush()
    def run_phase(self,name,duration,u=0.,r=0.,send=True):
        self.phase=name;previous=self.target
        start=self.clock;deadline=time.monotonic()+duration*30+30;last=-1.
        while self.clock-start<duration:
            if time.monotonic()>deadline:raise TimeoutError('Demo simulation stalled: '+name)
            fraction=min(1.,max(0.,(self.clock-start)/.8))
            self.target=(previous[0]+fraction*(u-previous[0]),previous[1]+fraction*(r-previous[1]))
            if send and self.clock-last>=.08:self.send(*self.target);last=self.clock
            rclpy.spin_once(self,timeout_sec=.02)
        print(f'{name}: sim {self.clock:.3f}s, u={self.odom.twist.twist.linear.x:.3f}, r={self.odom.twist.twist.angular.z:.3f}',flush=True)

def main():
    rclpy.init()
    ros=DemoProbe()
    gz=Node()
    stats=[]
    gz.subscribe(Time,'/gui/record_video/stats',lambda m:stats.append(
        {'video_s':m.sec+m.nsec*1e-9,'clock_s':ros.clock,'wall_s':time.monotonic()}))
    tracking=gz.advertise('/gui/track',CameraTrack)
    deadline=time.monotonic()+60
    while ros.clock is None or ros.odom is None:
        if time.monotonic()>deadline: raise TimeoutError('No simulation feedback')
        rclpy.spin_once(ros,timeout_sec=.05)
    # Record a continuous run. Pause/resume caused an engine assertion in the
    # first recording attempt; retain that failure separately.
    ros.run_phase('Pre-record settling',3,0,0)
    def control(paused):
        ok,response=gz.request('/world/blueboat_water/control',WorldControl(pause=paused),
                               WorldControl,Boolean,5000)
        if not ok or not response.data: raise RuntimeError('World control failed')
    for _ in range(15):
        tracking.publish(CameraTrack(track_mode=CameraTrack.FOLLOW_LOOK_AT,
                                      follow_target={'name':'blueboat'},track_target={'name':'blueboat'},
                                      follow_offset={'x':2.,'y':-2.,'z':1.55},
                                      track_offset={'x':0.,'y':0.,'z':.3},
                                      follow_pgain=1.,track_pgain=1.0))
        rclpy.spin_once(ros,timeout_sec=.05)
    (OUT/'ready_to_record.json').write_text(json.dumps({'sim_s':ros.clock,'status':'running; waiting for first recorder frame'}))
    print('READY: running; starting timestamped GUI camera capture.',flush=True)
    stampfile=OUT/'frame_timestamps.csv'
    stampfile.unlink(missing_ok=True)
    shutil.copy2(ROOT/'tools/video_recorder/capture.cc',OUT/'capture_used.cc')
    for filename in ['BlueBoat_Gazebo_Raw_full.mp4','BlueBoat_Propeller_Detail.mp4']:
        (OUT/filename).unlink(missing_ok=True)
    ok,response=gz.request('/blueboat/video/start',Boolean(data=True),Boolean,Boolean,15000)
    if not ok or not response.data: raise RuntimeError('Capture service unavailable')
    deadline=time.monotonic()+60
    while not stampfile.exists() or len(stampfile.read_text().splitlines())<2:
        if time.monotonic()>deadline: raise TimeoutError('No recorded frames')
        rclpy.spin_once(ros,timeout_sec=.05)
    start=float(stampfile.read_text().splitlines()[1].split(',')[1])
    phase_log=[]
    try:
        for name,duration,u,r in PHASES:
            phase_start=ros.clock
            ros.run_phase(name,duration,u,r)
            phase_log.append({'phase':name,'start_video_s':phase_start-start,
                              'end_video_s':ros.clock-start,'command_u':u,'command_r':r})
        ros.send(0,0)
        print('DONE: zero command; finalising timestamped GUI capture.',flush=True)
    finally:
        end_sim=ros.clock
        deadline=time.monotonic()+15
        while rclpy.ok() and stampfile.exists() and time.monotonic()<deadline:
            lines=stampfile.read_text().splitlines()
            if len(lines)>1 and float(lines[-1].split(',')[1])>=end_sim:break
            rclpy.spin_once(ros,timeout_sec=.05)
        with (OUT/'telemetry.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(ros.rows[0]));writer.writeheader();writer.writerows(ros.rows)
        gz.request('/blueboat/video/stop',Boolean(data=True),Boolean,Boolean,5000)
        deadline=time.monotonic()+30
        while not all((OUT/f).exists() for f in ['BlueBoat_Gazebo_Raw_full.mp4','BlueBoat_Propeller_Detail.mp4']):
            if time.monotonic()>deadline:raise TimeoutError('Recorder did not preserve camera files')
            rclpy.spin_once(ros,timeout_sec=.05)
        summary={}
        for phase in phase_log:
            rows=[x for x in ros.rows if x['phase']==phase['phase']]
            tail=[x for x in rows if x['t']>=rows[-1]['t']-1]
            summary[phase['phase']]={k:sum(x[k] for x in tail)/len(tail) for k in ['u','r']}
        manifest={'scope':'Actual Gazebo Harmonic GUI scene recording; ROS 2 command/odometry feedback. Still-water baseline; uncalibrated hydrodynamic estimates.',
                  'simulation_start_s':start,'simulation_end_s':end_sim,
                  'duration_s':end_sim-start,'phases':phase_log,'tail_mean_feedback':summary,
                  'telemetry_rows':len(ros.rows),'video_time_basis':'Gazebo simulation time, 25 fps',
                  'alignment_note':'Absolute simulation timestamp saved by GUI capture plugin for every encoded source frame; telemetry aligned to first captured frame.',
                  'simulation_real_time_factor':0.10}
        manifest['command_ramp_seconds']=0.8
        (OUT/'demo_manifest.json').write_text(json.dumps(manifest,indent=2))
        ros.send(0,0);ros.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
