"""Independently verify saved native pixels, all 6 pose components and camera fixity."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import subprocess
import cv2
import numpy as np


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def angle(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1))))


def stats(a):
    a = np.asarray(a, dtype=float)
    return dict(median=float(np.median(a)), p95=float(np.percentile(a, 95)), maximum=float(np.max(a)))


def validate(src, pilot=False):
    with (src/'pose_samples.csv').open() as f:
        rows = list(csv.DictReader(f))
    calibration = json.loads((src/'camera_calibration.json').read_text())
    manifest = json.loads((src/'demo_manifest.json').read_text())
    K = np.array([[calibration['fx'], 0, calibration['cx']],
                  [0, calibration['fy'], calibration['cy']], [0, 0, 1]], dtype=float)
    s = calibration['black_border_size_m']/2
    obj = np.array([[-s,s,0], [s,s,0], [s,-s,0], [-s,-s,0]], dtype=float)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 4
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    C = np.array([[0,-1,0], [0,0,-1], [1,0,0]], dtype=float)
    Rc = rotation(*calibration['camera_world_rpy_rad'])
    eye = np.array(calibration['camera_world_xyz'])
    Rcw = C @ Rc.T
    translation_errors, rotation_errors, estimator_agreement = [], [], []
    images, samples = {}, []
    detections_match = True
    for row in rows:
        i = int(row['index'])
        path = src/'native_frames'/f'{i:06d}.png'
        image = cv2.imread(str(path))
        assert image is not None and image.shape == (calibration['height'], calibration['width'], 3)
        images[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), dictionary, parameters=params)
        seen = [] if ids is None else ids.flatten().tolist()
        found = 0 in seen
        detections_match &= found == (row['detected'] == '1')
        sample = dict(index=i, sim_s=float(row['sim_s']), detected=found)
        if found:
            points = corners[seen.index(0)].reshape(4,2)
            _, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, points, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            candidates = []
            for rv, tv in zip(rvecs, tvecs):
                R, _ = cv2.Rodrigues(rv)
                if np.any((R@obj.T + tv)[2] <= 0):
                    continue
                projected, _ = cv2.projectPoints(obj, rv, tv, K, np.zeros(5))
                rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-points)**2, axis=1))))
                candidates.append((rms, R, tv.flatten()))
            rms, R, t = min(candidates, key=lambda x:x[0])
            logged_t = np.array([float(row['est_'+k]) for k in ['x','y','z']])
            logged_R = rotation(*np.radians([float(row['est_'+k+'_deg']) for k in ['roll','pitch','yaw']]))
            agreement = float(np.linalg.norm(t-logged_t))
            assert angle(R@logged_R.T) < .001
            estimator_agreement.append(agreement)
            bp = np.array([float(row['boat_'+k]) for k in ['x','y','z']])
            Rb = rotation(*[float(row['boat_'+k]) for k in ['roll','pitch','yaw']])
            world_truth = bp + Rb@np.array([0,0,.1576])
            expected_t = Rcw@(world_truth-eye)
            expected_R = Rcw@Rb
            te = float(np.linalg.norm(t-expected_t))
            re = angle(R@expected_R.T)
            translation_errors.append(te)
            rotation_errors.append(re)
            sample.update(translation_optical_m=t.tolist(), translation_error_m=te,
                          rotation_error_deg=re, reprojection_rms_px=rms,
                          candidate_reprojection_rms_px=[x[0] for x in sorted(candidates,key=lambda x:x[0])])
        samples.append(sample)
    stamps = np.array([float(r['sim_s']) for r in rows])
    duration = float(stamps[-1]-stamps[0])
    camera_poses = np.array([[float(r['camera_'+k]) for k in ['x','y','z','roll','pitch','yaw']] for r in rows])
    blank = np.full((calibration['height'], calibration['width']), 127, dtype=np.uint8)
    _, blank_ids, _ = cv2.aruco.detectMarkers(blank, dictionary, parameters=params)
    rate = len(translation_errors)/len(rows)
    position = stats(translation_errors)
    orientation = stats(rotation_errors)
    actual_rate = (len(rows)-1)/duration
    checks = {
        'actual_normal_arming': manifest['normal_arming_accepted'],
        'full_native_frame_set': len(images) == len(list((src/'native_frames').glob('*.png'))),
        'independent_detections_match_live': bool(detections_match),
        'image_only_estimates_match_live_within_1um': max(estimator_agreement) < 1e-6,
        'one_fixed_camera_no_tracking_or_cuts': float(np.ptp(camera_poses, axis=0).max()) < 1e-12,
        'at_least_95_percent_detection': rate >= .95,
        'position_error_p95_below_12cm': position['p95'] < .12,
        'rotation_error_p95_below_5deg': orientation['p95'] < 5,
        'at_least_8_distinct_frames_per_sim_second': actual_rate > 8,
        'capture_length': duration > (3 if pilot else 20),
        'blank_image_has_no_tag': blank_ids is None,
        'monotonic_frame_timestamps': bool(np.all(np.diff(stamps) > 0)),
    }
    # Full decode checks the finalized online composite and unmodified camera video.
    metadata = {}
    for name in ['BlueBoat_Fixed_Camera_6DOF.mp4', 'native_camera.mp4']:
        subprocess.run(['ffmpeg', '-nostdin', '-v', 'error', '-i', str(src/name), '-f', 'null', '-'], check=True)
        metadata[name] = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries',
                                      'format=duration,size:stream=width,height,nb_frames','-of','json',str(src/name)]))
    checks['both_videos_decode'] = True
    body_motion = {k: float(np.ptp([float(r['boat_'+k]) for r in rows])) for k in ['x','y','z','roll','pitch','yaw']}
    result = dict(date='2026-11-02', passed=all(checks.values()), checks=checks, sea_state=manifest['sea_state'],
                  frames=len(rows), detected_frames=len(translation_errors), detection_fraction=rate,
                  sampled_span_sim_s=duration, distinct_frames_per_sim_s=actual_rate,
                  maximum_frame_gap_sim_s=float(np.diff(stamps).max()),
                  translation_error_m=position, rotation_error_deg=orientation,
                  maximum_camera_pose_span=float(np.ptp(camera_poses,axis=0).max()),
                  body_motion_range_m_rad=body_motion, native_frame_sha256=images,
                  metadata=metadata, samples=samples,
                  validation_method='All saved unannotated native frames independently re-detected and solved using pixels, intrinsics and known black-border size. Pose compared against rendered model geometry with explicit body/tag/camera transforms. No truth input to the live estimator.',
                  limitations='Synthetic ideal pinhole camera, zero distortion/noise/blur, single planar tag; unresolved planar ambiguity under other geometries, glare, blur, occlusion and real camera calibration need further work. No UAV flight or autonomous landing. Video uses simulation timestamps at 1x; capture is slower in wall time.')
    (src/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['passed','checks','frames','detection_fraction','translation_error_m','rotation_error_deg','distinct_frames_per_sim_s']},indent=2))
    assert result['passed'], checks
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--pilot',action='store_true')
    a=p.parse_args()
    validate(a.out,a.pilot)
