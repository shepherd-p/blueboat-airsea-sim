"""Prepare and record native fixed / aerial cameras in the new sea-state cases."""
from pathlib import Path
import argparse
import csv
import json
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('action',choices=['prepare','record'])
parser.add_argument('--state',type=int,choices=[1,2],required=True)
a=parser.parse_args()
out=ROOT/f'results/sea_states_20261101/video_state_{a.state}'
out.mkdir(parents=True,exist_ok=True)
if a.action=='prepare':
    from sea_states import make_case
    subprocess.run([sys.executable,str(ROOT/'tools/prepare_video_world.py')],check=True)
    path=make_case(out,a.state,True,1,9042)
    tree=ET.parse(path);world=tree.getroot().find('world')
    world.remove(world.find('gui'))
    gui=ET.parse(ROOT/'results/video_demo/recording_world.sdf').getroot().find('world/gui')
    gui.remove(gui.find("plugin[@filename='CameraTracking']"))
    recorder=gui.find("plugin[@filename='BlueBoatVideoCapture']")
    ET.SubElement(recorder,'fixed_test_camera').text='true'
    ET.SubElement(recorder,'marker_detail').text='true'
    world.append(gui)
    world.find('scene/ambient').text='.72 .72 .72 1'
    world.find('light/diffuse').text='1 .97 .92 1'
    world.find('light/direction').text='-.18 -.9 -.38'
    model=ET.SubElement(world,'model',name='fixed_water_references')
    ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='references')
    def block(name,pos,size,color):
        v=ET.SubElement(link,'visual',name=name);ET.SubElement(v,'pose').text=' '.join(map(str,[*pos,0,0,0]))
        ET.SubElement(ET.SubElement(ET.SubElement(v,'geometry'),'box'),'size').text=' '.join(map(str,size))
        mat=ET.SubElement(v,'material');ET.SubElement(mat,'diffuse').text=color;ET.SubElement(mat,'ambient').text=color
    for x in range(0,37,4):
        block(f'post_{x}',[x,7.5,.05],[.10,.10,1.3],'.86 .83 .72 1')
        for i in range(-6,14):
            if i%2==0:block(f'band_{x}_{i}',[x,7.5,i*.05+.025],[.108,.108,.05],'.08 .12 .14 1')
        block(f'cap_{x}',[x,7.5,.78],[.15,.15,.08],'.95 .56 .11 1')
    block('quay_deck',[18,10,.6],[40,1.8,.18],'.31 .29 .26 1')
    for x in range(-2,39,3):block(f'pile_{x}',[x,9.5,-.1],[.24,.24,1.5],'.22 .23 .22 1')
    ET.indent(tree,space='  ');tree.write(path,encoding='utf-8',xml_declaration=True)
    print(path)
else:
    import rclpy
    from validate_sitl import Test
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.physics_pb2 import Physics
    rclpy.init();test=Test(out);n=test.node;gz=Node()
    deadline=time.monotonic()+60
    while n.clock is None or n.odom is None:
        test.tick()
        if time.monotonic()>deadline:raise TimeoutError('No video odometry')
    test.mode_set('MANUAL');test.phase('Sensor settling',40,lambda:test.rc())
    test.m.mav.command_long_send(test.m.target_system,test.m.target_component,400,0,1,0,0,0,0,0,0,0)
    deadline=time.monotonic()+25
    while not test.armed:
        test.tick();test.rc()
        if time.monotonic()>deadline:raise RuntimeError('Normal arming rejected')
    world_name=ET.parse(out/'world.sdf').getroot().find('world').get('name')
    ok,response=gz.request(f'/world/{world_name}/set_physics',Physics(real_time_factor=.20,max_step_size=.002),Physics,Boolean,15000)
    assert ok and response.data,'Could not slow capture without changing physics step'
    # Allow renderer/physics to settle after changing RTF.
    test.phase('Capture settling',2,lambda:test.rc())
    ok,response=gz.request('/blueboat/video/start',Boolean(data=True),Boolean,Boolean,15000)
    assert ok and response.data
    deadline=time.monotonic()+50
    stamps=out/'frame_timestamps.csv'
    while not stamps.exists() or len(stamps.read_text().splitlines())<2:
        test.tick()
        if time.monotonic()>deadline:raise TimeoutError('No recorded frames')
    start=float(stamps.read_text().splitlines()[1].split(',')[1])
    phases=[]
    try:
        for name,duration,steer,throttle in [('Neutral drift',8,1500,1500),('SITL forward',8,1500,1560),('SITL turn',5,1550,1540)]:
            before=n.clock;test.phase(name,duration,lambda s=steer,t=throttle:test.rc(s,t))
            phases.append(dict(name=name,start=before-start,end=n.clock-start))
    finally:
        end=n.clock;gz.request('/blueboat/video/stop',Boolean(data=True),Boolean,Boolean,5000)
        deadline=time.monotonic()+40
        while not (out/'BlueBoat_Gazebo_Raw_full.mp4').exists():
            test.tick()
            if time.monotonic()>deadline:raise TimeoutError('Video not finalized')
        with (out/'telemetry.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(n.rows[0]));w.writeheader();w.writerows(n.rows)
        (out/'demo_manifest.json').write_text(json.dumps(dict(sea_state=a.state,simulation_start_s=start,simulation_end_s=end,phases=phases,
            scope='Actual 32-component Gazebo sea-state scene, deck marker and real ArduRover 4.7.2. Aerial detail is a native virtual camera, not UAV flight.'),indent=2)+'\n')
        test.rc();test.m.mav.command_long_send(test.m.target_system,test.m.target_component,400,0,0,0,0,0,0,0,0)
        test.log.close();n.destroy_node();rclpy.shutdown()
    print('Recorded sea state',a.state,flush=True)
