"""Check actual actuator outputs against independent official chart JSON."""
from pathlib import Path
import bisect
import json
import math
import time
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

ROOT=Path(__file__).resolve().parents[1]
curve=json.loads((ROOT/'h-spar-sim/ros2_ws/src/blueboat_description/config/propulsion_curve.json').read_text())
rows=curve['samples'];xs=[r['thrust_N'] for r in rows];ys=[r['rpm'] for r in rows]
def expected_rpm(f):
    if f<=xs[0]:return ys[0]
    if f>=xs[-1]:return ys[-1]
    i=bisect.bisect_right(xs,f)
    return ys[i-1]+(f-xs[i-1])/(xs[i]-xs[i-1])*(ys[i]-ys[i-1])
node=Node();received={};latest=[0.]
def callback(key):
    def receive(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        received.setdefault(t,{})[key]=msg.data;latest[0]=max(latest[0],t)
    return receive
for side in ['left','right']:
    for field in ['rpm','applied_thrust']:
        node.subscribe(Double,f'/blueboat/propulsion/{side}/{field}',callback(side+'_'+field))
pub=[node.advertise(f'/wamv/thrusters/{s}/thrust',Double) for s in ['left','right']]
deadline=time.monotonic()+20
while not received:
    if time.monotonic()>deadline:raise TimeoutError('No actuator telemetry')
    time.sleep(.02)
time.sleep(.3)
maxN=8.2*9.80665/2;minN=xs[0]
cases=[]
for request in [*xs,3.7403081422,100.,-100.,float('nan')]:
    start=latest[0];deadline=time.monotonic()+5
    while latest[0]<start+.18:
        if time.monotonic()>deadline:raise TimeoutError('Simulation stalled')
        for p in pub:p.publish(Double(data=request))
        time.sleep(.015)
    pairs=[(t,v) for t,v in list(received.items()) if t>start+.10 and len(v)==4]
    assert pairs,'No complete time-matched actuator sample'
    t,v=max(pairs)
    force=max(minN,min(maxN,request)) if math.isfinite(request) else 0.
    rpm=expected_rpm(force)
    for side in ['left','right']:
        assert abs(v[side+'_applied_thrust']-force)<1e-8,(request,v,force)
        assert abs(v[side+'_rpm']-rpm)<1e-7,(request,v,rpm)
    cases.append({'requested_N':request if math.isfinite(request) else 'NaN','applied_N':force,'rpm':rpm,'passed':True})
# Assert the Gazebo-side command watchdog independently of the ROS controller.
for p in pub:p.publish(Double(data=10))
start=latest[0];deadline=time.monotonic()+10
while latest[0]<start+.8:
    if time.monotonic()>deadline:raise TimeoutError('Watchdog test stalled')
    time.sleep(.02)
_,v=max((t,v) for t,v in list(received.items()) if len(v)==4)
assert all(abs(x)<1e-8 for x in v.values()),v
out={'passed':True,'cases':len(cases),'sample_table_pairs':33,'voltage_V':16,
     'watchdog_zero_force_and_rpm':True,'cases_detail':cases,
     'scope':'Independent comparison of actual Gazebo applied-force/RPM outputs with official static chart samples; not experimental calibration of the vehicle under way.'}
(ROOT/'results/propulsion_validation.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'passed':True,'cases':len(cases),'watchdog':True,'forward_0_8mps_equivalent_rpm':expected_rpm(3.7403081422)},indent=2))
