"""Regenerate a standard tag36h11 marker with OpenCV (optional asset step)."""
from pathlib import Path
import json,hashlib
import cv2
ROOT=Path(__file__).resolve().parents[1]
DESC=ROOT/'h-spar-sim/ros2_ws/src/blueboat_description'
dictionary=cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
bits=cv2.aruco.drawMarker(dictionary,0,8,borderBits=1)
config={
 'enabled':True,'family':'tag36h11','id':0,
 'sheet_size_m':.54,'black_border_size_m':.432,'quiet_border_modules':1,
 'panel_size_m':[.64,.56,.006],'panel_center_m':[0,0,.154445061],
 'marker_center_m':[0,0,.1576],
 'binary_cells_black_border_included':(bits//255).tolist(),
 'source':'OpenCV DICT_APRILTAG_36h11, ID 0; standard public fiducial.',
 'scope':'Proposed flush removable panel over the open center frame. Visual only: no added mass, collision or UAV touchdown validation. Panel/support/flag removal remain design decisions.',
 'dimension_convention':'Detector size is the outside BLACK border (0.432 m), not the full white sheet (0.54 m). Pattern has 8x8 cells plus one white quiet cell on each side.'}
(DESC/'config/landing_marker.json').write_text(json.dumps(config,indent=2)+'\n')
print('Prepared standard AprilTag config; run tools/generate_model.py then tools/generate_environment.py.')
