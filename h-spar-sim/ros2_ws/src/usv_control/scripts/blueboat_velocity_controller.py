#!/usr/bin/env python3
"""Feedback surge/yaw controller, using H-SPAR's existing /wamv interfaces."""
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

def allocate(force, moment, separation, maximum, reverse_ratio=1.):
    # ROS FLU: port y>0; positive forward force there causes negative yaw.
    left=force/2-moment/separation
    right=force/2+moment/separation
    scale=max(1.,left/maximum,right/maximum,-left/(maximum*reverse_ratio),-right/(maximum*reverse_ratio))
    return left/scale,right/scale

class BlueBoatController(Node):
    def __init__(self):
        super().__init__('blueboat_velocity_controller')
        cfg=Path(get_package_share_directory('blueboat_description'))/'config/controller.json'
        params=json.loads(cfg.read_text())
        for name,default in params.items():
            self.declare_parameter(name,default)
        self.p={name:self.get_parameter(name).value for name in params}
        if (not all(math.isfinite(float(v)) for v in self.p.values()) or
            min(self.p['max_thrust'],self.p['reverse_thrust_ratio'],self.p['thruster_separation'],
                self.p['command_timeout'],self.p['feedback_timeout'])<=0):
            raise ValueError('Controller limits and timeouts must be finite and positive.')
        self.command=(0.,0.); self.odom=None
        self.last_command=None; self.last_feedback=None
        self.left=self.create_publisher(Float64,'/wamv/thrusters/left/thrust',10)
        self.right=self.create_publisher(Float64,'/wamv/thrusters/right/thrust',10)
        self.pose=self.create_publisher(PoseStamped,'/wamv/simple_pose',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Twist,'/wamv/cmd_vel',self.on_command,10)
        self.create_subscription(Odometry,'/blueboat/odometry',self.on_odom,10)
        self.create_timer(.02,self.control)
        self.get_logger().info(f'BlueBoat feedback control: {self.p["max_thrust"]:.3f} N/motor, separation {self.p["thruster_separation"]:.3f} m')

    def on_command(self,msg):
        if not all(math.isfinite(v) for v in [msg.linear.x,msg.angular.z]):
            self.command=(0.,0.); self.last_command=None
            self.publish(0.,0.)
            return
        self.command=(max(-self.p['max_speed'],min(self.p['max_speed'],msg.linear.x)),
                      max(-self.p['max_yaw_rate'],min(self.p['max_yaw_rate'],msg.angular.z)))
        self.last_command=self.get_clock().now()

    def on_odom(self,msg):
        self.odom=msg
        self.last_feedback=self.get_clock().now()
        pose=PoseStamped(); pose.header=msg.header; pose.header.frame_id='map'; pose.pose=msg.pose.pose
        self.pose.publish(pose)
        t=TransformStamped(); t.header=pose.header; t.child_frame_id='wamv_simple'
        t.transform.translation.x=pose.pose.position.x
        t.transform.translation.y=pose.pose.position.y
        t.transform.translation.z=pose.pose.position.z
        t.transform.rotation=pose.pose.orientation
        self.tf.sendTransform(t)

    def publish(self,left,right):
        self.left.publish(Float64(data=left)); self.right.publish(Float64(data=right))

    def control(self):
        now=self.get_clock().now()
        if (self.last_command is None or self.last_feedback is None or
            (now-self.last_command).nanoseconds*1e-9>self.p['command_timeout'] or
            (now-self.last_feedback).nanoseconds*1e-9>self.p['feedback_timeout']):
            self.publish(0.,0.)
            return
        u,r=self.command
        actual_u=self.odom.twist.twist.linear.x
        actual_r=self.odom.twist.twist.angular.z
        if not all(math.isfinite(v) for v in [actual_u,actual_r]):
            self.publish(0.,0.); return
        f=self.p['surge_linear']*u+self.p['surge_quadratic']*u*abs(u)+self.p['surge_kp']*(u-actual_u)
        n=self.p['yaw_linear']*r+self.p['yaw_quadratic']*r*abs(r)+self.p['yaw_kp']*(r-actual_r)
        left,right=allocate(f,n,self.p['thruster_separation'],self.p['max_thrust'],self.p['reverse_thrust_ratio'])
        self.publish(*(0. if abs(x)<self.p['thrust_deadband'] else x for x in [left,right]))

def main():
    rclpy.init(); node=BlueBoatController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok(): raise
    finally:
        if rclpy.ok():
            node.publish(0.,0.)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__':
    main()
