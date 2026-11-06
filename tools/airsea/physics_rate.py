"""Adjust wall-clock playback only; preserves 2 ms physics integration."""
import sys
from gz.transport13 import Node
from gz.msgs10.physics_pb2 import Physics
from gz.msgs10.boolean_pb2 import Boolean
n=Node();ok,response=n.request('/world/airsea/set_physics',Physics(real_time_factor=float(sys.argv[1]),max_step_size=.002),Physics,Boolean,10000)
if not ok or not response.data:raise RuntimeError('Simulation playback-rate request failed')
