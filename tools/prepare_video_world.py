"""Create a recording-only GUI/world copy; keep baseline physics unchanged."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/video_demo'
OUT.mkdir(parents=True, exist_ok=True)
source = ROOT / 'h-spar-sim/ros2_ws/src/blueboat_description/worlds/blueboat_water.sdf'
tree = ET.parse(source)
world = tree.getroot().find('world')
world.find('physics/real_time_factor').text='0.10'
world.remove(world.find('gui'))
gui = ET.SubElement(world, 'gui', {'fullscreen': 'false'})
window = ET.SubElement(gui, 'window')
ET.SubElement(window, 'width').text = '1280'
ET.SubElement(window, 'height').text = '800'
ET.SubElement(window, 'dialog_on_exit').text = 'false'

def plugin(filename, name, docked=False):
    p = ET.SubElement(gui, 'plugin', {'filename': filename, 'name': name})
    g = ET.SubElement(p, 'gz-gui')
    ET.SubElement(g, 'property', {'key':'showTitleBar', 'type':'bool'}).text='false'
    ET.SubElement(g, 'property', {'key':'state', 'type':'string'}).text='docked' if docked else 'floating'
    if not docked:
        for key, value in [('width','5'), ('height','5')]:
            ET.SubElement(g, 'property', {'key':key, 'type':'double'}).text=value
        ET.SubElement(g, 'property', {'key':'resizable', 'type':'bool'}).text='false'
    return p

p=plugin('MinimalScene','3D View',True)
for key,value in [('engine','ogre2'), ('scene','scene'),
                  ('camera_pose','2 -2 1.55 0 0.41 2.3562'),
                  ('ambient_light','0.65 0.65 0.65'), ('background_color','0.70 0.80 0.90')]:
    ET.SubElement(p,key).text=value
plugin('GzSceneManager','Scene Manager')
plugin('InteractiveViewControl','Interactive View Control')
plugin('CameraTracking','Camera Tracking')
p=plugin('BlueBoatVisuals','BlueBoatVisuals')
ET.SubElement(p,'telemetry').text='rotor_visual_debug.csv'
p=plugin('BlueBoatVideoCapture','BlueBoatVideoCapture')
ET.SubElement(p,'filename').text='BlueBoat_Gazebo_Raw_full.mp4'
ET.SubElement(p,'timestamps').text='frame_timestamps.csv'

ET.indent(tree, space='  ')
target=OUT/'recording_world.sdf'
tree.write(target,encoding='utf-8',xml_declaration=True)
print(target)
