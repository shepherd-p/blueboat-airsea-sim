# Boat camera capture correction

This report records the earlier 30 degree camera run with the public
procedural boat. For the restored original BlueBoat, 24 degree optics,
antenna occlusion correction and current evidence, see
[the original-model revision](ORIGINAL_BOAT_RESTORED_20261110.md).

The earlier fixed-downward camera gave accepted image fixes for just 0.672 s
and 0.588 s on the two survey passes. The 72 s overview contained only six
seconds of onboard view; most of those seconds showed empty sea. Its held
thumbnail was labeled, but did not adequately demonstrate target capture.

## Actual sensor change

The camera now uses a virtual three-axis gimbal, a 30 degree horizontal field
of view, 1920 x 1080 original images and the same 12 Hz requested sampling.
The gimbal initially points at the configured mission search point
`[300, 175, 0.16]` m ENU. This is an approximate search location; it is not
read from the current boat pose. Ownship EKF pose computes the pointing
direction. A past accepted image estimate can refine the aim on the next
frame. Aim updates use a 0.4 s filter, capped at 0.25 per update, and a
water-height prior of 0 to 0.5 m. Gimbal rotation is limited to 120 deg/s;
the requested viewing direction is capped at 65 degrees from nadir.

Boat position is still calculated from the archived JPEG pixels, calibrated
intrinsics, the 0.432 m AprilTag black border, IPPE PnP, logged relative
gimbal quaternion and ownship navigation pose. Small or foreshortened tags
are rejected (mean edge at least 20 px, shortest edge at least 12 px,
reprojection RMS below 1.2 px). Boat truth supplies evaluation errors only.
The existing physical ArduPlane controller and mission are retained.

The live display opens with the onboard camera as its main view and the
aircraft chase view as an inset. Click the inset or **Swap camera views**
to exchange them. A current image fix displays ENU coordinates; missing
detections display **No current valid position fix**.

## Verification and video

`tools/airsea/validate.py` independently re-detects the original JPEGs and
recomputes every position estimate. For gimbal runs it also reconstructs
each aiming point using only the initial mission point and earlier image
fixes, checks pointing geometry and slew limits, and requires at least two
seconds of sustained detections on each search pass. A gap over 0.35 s
breaks a sustained detection interval.

`tools/airsea/camera_review.py` produces a camera-first evidence video and
a clean real-speed video. The evidence view shows the full original sensor
frame, a crop of that same frame, current ENU fix, reference error and the
planned/flown route. The longest sustained detection interval from each
pass is selected, with 0.25 s of context at either end. Both passes play at real speed first, followed by
clearly labeled half-speed reviews. The source image index and age are
logged for every output frame. It never uses a replay image to create a
fix, a future observation or a held target crop. The clean version has no
overlays and includes only the real-speed segments.

```bash
bash tools/build_airsea_wsl.sh
bash tools/run_airsea_wsl.sh --out "$PWD/results/camera_run" --rtf 0.8
python3 tools/airsea/validate.py --out "$PWD/results/camera_run"
python3 tools/airsea/camera_review.py --run "$PWD/results/camera_run" \
  --out "$PWD/results/camera_video"
```

The gimbal is an ideal virtual sensor mount with a slew limit, not a
calibrated servo/inertial hardware simulation. This remains a two-pass
survey, not a claim of continuous tracking throughout the entire mission.
Fresh sensor sample frequency and video export frequency are distinct:
the 30 fps video repeats the last original image until the next sample.
The full flight/vision results and observation intervals belong to the
saved run's `validation.json`; video visibility alone is not validation.

## Verified run on 2026-11-10

`airsea_gimbal_20261110_02` completed takeoff, both surveys, an acknowledged
image-based waypoint update, runway landing/stop and automatic disarm.
All 29 independent checks passed. There were 35 / 29 accepted original
images on the two passes. The longest sustained detection intervals were
2.772 / 2.604 s, using a maximum 0.35 s gap. The first pass's full detection
span was 3.612 s, including an interruption. These durations are simulation
time, not slow-motion video duration.

3D tag-center position error was 0.577 m median, 0.924 m P95 and 3.001 m
maximum. The maximum was a distant early observation; all accepted fixes
remain in the evidence archive. Search-window pitch-rate P95 remained
1.554 deg/s, and touchdown descent speed was 0.831 m/s. The whole-run
capture averaged 5.284 fresh images/s; survey windows use slower simulation
playback to acquire more samples. No real-time 30 Hz sensor performance is
claimed.
