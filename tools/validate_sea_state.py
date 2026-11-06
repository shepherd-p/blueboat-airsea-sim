"""Real ArduRover/Gazebo trial plus a fixed 600-second wave-gauge record."""
from pathlib import Path
import argparse
import csv
import json
import math
import time
import io
import numpy as np
import rclpy
from validate_sitl import Test

parser = argparse.ArgumentParser()
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
out = args.out
c = json.loads((out / 'profile.json').read_text())
rclpy.init()
test = Test(out)
n = test.node

try:
    deadline = time.monotonic() + 40
    while n.clock is None or n.odom is None:
        test.tick()
        if time.monotonic() > deadline:
            raise TimeoutError('No sea-state odometry')
    test.mode_set('MANUAL')
    test.phase('warmup', 40, lambda: test.rc())
    test.m.mav.command_long_send(test.m.target_system, test.m.target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.monotonic() + 20
    while not test.armed:
        test.tick(); test.rc()
        if time.monotonic() > deadline:
            raise RuntimeError('Normal arming rejected; inspect MAVLink log')
    test.phase('neutral_drift', 60, lambda: test.rc())
    test.phase('forward', 25, lambda: test.rc(throttle=1600))
    test.phase('right_turn', 15, lambda: test.rc(steer=1580, throttle=1500))
    test.phase('reverse', 20, lambda: test.rc(throttle=1400))
    test.rc()
    actual = n.odom.pose.pose.position
    goal = (actual.x + 15, actual.y + 8)
    test.mode_set('GUIDED')
    def waypoint():
        test.m.mav.set_position_target_local_ned_send(0, test.m.target_system, test.m.target_component, 1, 3576,
                                                     goal[1], goal[0], 0, 0, 0, 0, 0, 0, 0, 0, 0)
    test.phase('guided_waypoint', 60, waypoint, goal)
    actual = n.odom.pose.pose.position
    goal_error = math.hypot(actual.x-goal[0], actual.y-goal[1])
    center = (actual.x, actual.y)
    test.mode_set('LOITER')
    test.phase('loiter', max(60, 665-n.clock))
    test.mode_set('HOLD')
    test.m.mav.command_long_send(test.m.target_system, test.m.target_component, 400, 0, 0, 0, 0, 0, 0, 0, 0)
    test.phase('disarmed', 3)
    with (out / 'telemetry.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(n.rows[0])); writer.writeheader(); writer.writerows(n.rows)
    completion = dict(goal_enu_m=goal, waypoint_final_distance_m=goal_error,
                      loiter_center_enu_m=center, armed_normally=True,
                      disarmed=not test.armed, ekf_available='EKF_STATUS_REPORT' in test.latest,
                      phases=test.phases)
    (out/'control_completion.json').write_text(json.dumps(completion,indent=2)+'\n')
    test.log.flush()
    # Analyze only after Gazebo has closed/flushed its telemetry streams.
    print('CONTROL_COMPLETE: saved exact endpoints; analyze after session shutdown.',flush=True)

finally:
    test.rc()
    test.m.mav.command_long_send(test.m.target_system,test.m.target_component,400,0,0,0,0,0,0,0,0)
    test.log.close();n.destroy_node();rclpy.shutdown()
