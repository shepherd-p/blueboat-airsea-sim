"""Prepare/drive the fixed-camera image-based tag pose demonstration."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def prepare(out, state):
    from sea_states import make_case
    subprocess.run([sys.executable, str(ROOT / 'tools/prepare_video_world.py')], check=True)
    path = make_case(out, state, True, 1, 9052)
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    world.remove(world.find('gui'))
    gui = ET.parse(ROOT / 'results/video_demo/recording_world.sdf').getroot().find('world/gui')
    for name in ['CameraTracking', 'BlueBoatVideoCapture']:
        gui.remove(gui.find(f"plugin[@filename='{name}']"))
    # This ordinary overview is independent of the fixed measurement camera.
    gui.find("plugin[@filename='MinimalScene']/camera_pose").text = '8 -6 5 0 .6 1.57079632679'
    p = ET.SubElement(gui, 'plugin', filename='BlueBoatTagPose', name='Fixed Camera Tag 6DOF')
    ET.SubElement(p, 'directory').text = str(out)
    ET.SubElement(p, 'sea_state').text = str(state)
    props = ET.SubElement(p, 'gz-gui')
    for k, t, v in [('showTitleBar', 'bool', 'true'), ('state', 'string', 'floating'),
                    ('width', 'double', '1280'), ('height', 'double', '720'),
                    ('x', 'double', '0'), ('y', 'double', '0')]:
        ET.SubElement(props, 'property', key=k, type=t).text = v
    world.append(gui)
    world.find('scene/ambient').text = '.72 .72 .72 1'
    world.find('light/diffuse').text = '1 .97 .92 1'
    world.find('light/direction').text = '-.18 -.9 -.38'
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    records = {
        'date': '2026-11-02', 'sea_state': state,
        'camera_world_xyz_m': [7.5, -1, 5],
        'scope': 'One fixed native rendering camera, actual pixels -> AprilTag/PnP. No drone dynamics.',
        'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in [path, ROOT / 'tools/video_recorder/tag_pose.cc',
                             ROOT / 'tools/video_recorder/BlueBoatTagPose.qml',
                             ROOT / 'tools/fixed_camera_6dof.py']},
    }
    (out / 'input_provenance.json').write_text(json.dumps(records, indent=2) + '\n')
    print(path)


def record(out, state, pilot):
    import rclpy
    from validate_sitl import Test
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.physics_pb2 import Physics
    rclpy.init()
    test = Test(out)
    gz = Node()
    n = test.node
    deadline = time.monotonic() + 60
    while n.clock is None or n.odom is None:
        test.tick()
        if time.monotonic() > deadline:
            raise TimeoutError('No simulation odometry')
    test.mode_set('MANUAL')
    test.phase('Normal sensor settling', 40, lambda: test.rc())
    test.m.mav.command_long_send(test.m.target_system, test.m.target_component,
                               400, 0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.monotonic() + 25
    while not test.armed:
        test.tick()
        test.rc()
        if time.monotonic() > deadline:
            raise RuntimeError('Normal arming rejected')
    world = ET.parse(out / 'world.sdf').getroot().find('world').get('name')
    ok, response = gz.request(f'/world/{world}/set_physics',
                              Physics(real_time_factor=.20, max_step_size=.002),
                              Physics, Boolean, 15000)
    assert ok and response.data
    test.phase('Capture settling', 2, lambda: test.rc())
    ok, response = gz.request('/blueboat/tag_pose/start', Boolean(data=True), Boolean, Boolean, 15000)
    assert ok and response.data, 'Fixed camera plugin did not start'
    deadline = time.monotonic() + 40
    samples = out / 'pose_samples.csv'
    while not samples.exists() or len(samples.read_text().splitlines()) < 2:
        test.tick()
        if time.monotonic() > deadline:
            raise TimeoutError('No native camera / pose samples')
    start = float(samples.read_text().splitlines()[1].split(',')[1])
    phases = []
    sequence = [('Wave-driven neutral drift', 6, 1500, 1500),
                ('SITL slow forward', 5, 1500, 1520),
                ('SITL turn', 6, 1560, 1500),
                ('Neutral drift after turn', 5, 1500, 1500)]
    if pilot:
        sequence = [('Pilot: wave-driven drift', 4, 1500, 1500)]
    try:
        for name, duration, steer, throttle in sequence:
            before = n.clock
            test.phase(name, duration, lambda s=steer, t=throttle: test.rc(s, t))
            phases.append(dict(name=name, start=before-start, end=n.clock-start))
    finally:
        end = n.clock
        gz.request('/blueboat/tag_pose/stop', Boolean(data=True), Boolean, Boolean, 15000)
        deadline = time.monotonic() + 60
        while not (out / 'capture_closed.txt').exists():
            test.tick()
            if time.monotonic() > deadline:
                raise TimeoutError('Native videos were not finalized')
        with (out / 'telemetry.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(n.rows[0]))
            w.writeheader()
            w.writerows(n.rows)
        (out / 'demo_manifest.json').write_text(json.dumps({
            'sea_state': state, 'pilot': pilot, 'simulation_start_s': start,
            'simulation_end_s': end, 'phases': phases,
            'normal_arming_accepted': test.armed, 'real_firmware': 'ArduRover 4.7.2',
            'nominal_firmware_settling_rtf': 1, 'capture_rtf': .2, 'physics_step_s': .002,
            'scope': 'Actual fixed native camera and online image-only six-DOF pose estimation. Truth is a separate error reference.',
        }, indent=2) + '\n')
        test.rc()
        test.m.mav.command_long_send(test.m.target_system, test.m.target_component,
                                   400, 0, 0, 0, 0, 0, 0, 0, 0)
        test.log.close()
        n.destroy_node()
        rclpy.shutdown()
    print('Fixed-camera 6DOF recording complete', flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('action', choices=['prepare', 'record'])
    p.add_argument('--state', type=int, choices=[1, 2], default=2)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pilot', action='store_true')
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    if a.action == 'prepare':
        prepare(a.out.resolve(), a.state)
    else:
        record(a.out.resolve(), a.state, a.pilot)
