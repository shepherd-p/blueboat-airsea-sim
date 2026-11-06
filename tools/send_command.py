import argparse
import math
import time
import rclpy
from geometry_msgs.msg import Twist

def main():
    p=argparse.ArgumentParser(); p.add_argument('forward',type=float); p.add_argument('yaw_rate',type=float); p.add_argument('seconds',type=float)
    a=p.parse_args()
    if not all(math.isfinite(x) for x in [a.forward,a.yaw_rate,a.seconds]) or a.seconds<=0: p.error('Use finite values and a positive duration.')
    rclpy.init(); node=rclpy.create_node('blueboat_command'); pub=node.create_publisher(Twist,'/wamv/cmd_vel',10)
    msg=Twist(); msg.linear.x=a.forward; msg.angular.z=a.yaw_rate
    try:
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:
            pub.publish(msg); rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5): pub.publish(Twist()); time.sleep(.03)
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
