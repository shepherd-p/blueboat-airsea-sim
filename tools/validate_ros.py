#!/usr/bin/env python3
"""Exercise the real ROS -> bridge -> Gazebo -> odometry -> feedback loop."""
import argparse
import csv
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

ROOT=Path(__file__).resolve().parents[1]

def rpy(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,yaw

class Probe(Node):
    def __init__(self):
        super().__init__('blueboat_validation',parameter_overrides=[])
        self.pub=self.create_publisher(Twist,'/wamv/cmd_vel',10)
        self.clock=None; self.odom=None; self.left=0.; self.right=0.; self.rows=[]
        self.phase='startup'; self.target=(0.,0.)
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/blueboat/odometry',self.on_odom,10)
        self.create_subscription(Float64,'/wamv/thrusters/left/thrust',lambda m:setattr(self,'left',m.data),10)
        self.create_subscription(Float64,'/wamv/thrusters/right/thrust',lambda m:setattr(self,'right',m.data),10)

    def on_clock(self,msg):
        self.clock=msg.clock.sec+msg.clock.nanosec*1e-9

    def on_odom(self,msg):
        self.odom=msg
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        roll,pitch,yaw=rpy(msg.pose.pose.orientation)
        p=msg.pose.pose.position; v=msg.twist.twist
        self.rows.append(dict(t=t,phase=self.phase,target_u=self.target[0],target_r=self.target[1],
                              x=p.x,y=p.y,z=p.z,roll=roll,pitch=pitch,yaw=yaw,
                              u=v.linear.x,v=v.linear.y,w=v.linear.z,r=v.angular.z,
                              left_N=self.left,right_N=self.right))

    def send(self,u,r):
        msg=Twist(); msg.linear.x=float(u); msg.angular.z=float(r); self.pub.publish(msg)

    def run_phase(self,name,duration,u=0.,r=0.,send=True):
        self.phase=name; self.target=(u,r)
        start=self.clock; deadline=time.monotonic()+duration*5+20
        last=-1.
        while self.clock-start<duration:
            if time.monotonic()>deadline: raise TimeoutError(f'Simulation stalled during {name}')
            if send and self.clock-last>=.08:
                self.send(u,r); last=self.clock
            rclpy.spin_once(self,timeout_sec=.02)
        print(f'{name}: sim {self.clock:.2f}s, u={self.odom.twist.twist.linear.x:.3f}, r={self.odom.twist.twist.angular.z:.3f}',flush=True)

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--out',type=Path,default=ROOT/'results/baseline')
    args=parser.parse_args(); args.out.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=Probe()
    try:
        deadline=time.monotonic()+40
        while node.clock is None or node.odom is None:
            if time.monotonic()>deadline: raise TimeoutError('No clock or odometry: inspect launch.log')
            rclpy.spin_once(node,timeout_sec=.1)
        phases=[('float_settle',5,0,0,True),('forward_1mps',10,1,0,True),
                ('stop',6,0,0,True),('ccw_turn',10,0,.4,True),('turn_stop',5,0,0,True),
                ('cw_turn',10,0,-.4,True),('reverse',10,-.5,0,True),
                ('command_timeout',4,0,0,False),('forward_3mps',15,3,0,True),
                ('speed_limit',5,100,0,True),('nan_rejection',3,float('nan'),0,True),
                ('final_stop',5,0,0,True)]
        for phase in phases: node.run_phase(*phase)
        with (args.out/'telemetry.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(node.rows[0])); writer.writeheader(); writer.writerows(node.rows)
        checks={}; summary={}
        for name,*_ in phases:
            rows=[r for r in node.rows if r['phase']==name]
            end=max(r['t'] for r in rows); tail=[r for r in rows if r['t']>=end-1.5]
            summary[name]={key:float(np.mean([r[key] for r in tail])) for key in ['u','r','roll','pitch','z','left_N','right_N']}
            summary[name]['samples']=len(rows)
        checks['finite_physics']=all(math.isfinite(r[k]) for r in node.rows for k in ['x','y','z','u','r','roll','pitch','left_N','right_N'])
        checks['float_stable']=abs(summary['float_settle']['roll'])<.05 and abs(summary['float_settle']['pitch'])<.05 and abs(summary['float_settle']['u'])<.05
        checks['forward_tracking']=abs(summary['forward_1mps']['u']-1)<.12
        checks['stop_tracking']=abs(summary['stop']['u'])<.08
        checks['positive_yaw_is_ccw']=abs(summary['ccw_turn']['r']-.4)<.08
        checks['negative_yaw_is_cw']=abs(summary['cw_turn']['r']+.4)<.08
        checks['reverse_tracking']=abs(summary['reverse']['u']+.5)<.12
        checks['thrust_limit']=max(abs(r[k]) for r in node.rows for k in ['left_N','right_N'])<=40.207265+.001
        checks['command_watchdog']=abs(summary['command_timeout']['left_N'])<.001 and abs(summary['command_timeout']['right_N'])<.001
        checks['three_mps_envelope']=2.7<summary['forward_3mps']['u']<3.15
        checks['speed_command_limit']=summary['speed_limit']['u']<3.15
        checks['invalid_command_zero_thrust']=abs(summary['nan_rejection']['left_N'])<.001 and abs(summary['nan_rejection']['right_N'])<.001
        checks['bounded_roll_pitch']=max(abs(r[k]) for r in node.rows for k in ['roll','pitch'])<.35
        result={'checks':checks,'passed':all(checks.values()),'phase_summary':summary,
                'samples':len(node.rows),'simulation_start':node.rows[0]['t'],'simulation_end':node.rows[-1]['t'],
                'scope':'Real ROS2 Jazzy + ros_gz_bridge + Gazebo Harmonic; still-water assumed coefficients. Not experimental validation.'}
        (args.out/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        if not result['passed']: raise SystemExit(1)
    finally:
        node.send(0,0); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
