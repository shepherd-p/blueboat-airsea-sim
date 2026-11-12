# Mutual observation and classical speed coordination

The accepted full run is `cooperative_20261112_06`: **38/38 independent
checks passed**, including all 938 original camera pairs, normal arming,
autonomous takeoff and runway landing/stop/disarm. Its 70.000 s coordinated
interval contains 812 camera pairs with actual mean sampling 11.588 Hz.

| Measured outcome | Aircraft observing boat | Boat observing aircraft |
| --- | ---: | ---: |
| Fresh visual coverage | 99.99% | 98.88% |
| Longest visual measurement interval | 0.092 s | 0.348 s |
| Image position error P95 | 1.054 m | 5.245 m |
| AprilTag observations | 30 | 12 |
| Shape-only maximum range | 104.163 m | 103.955 m |

Boat-to-aircraft target velocity error P95 is 5.918 m/s; the velocity
estimate is still coarse during turns and is not a publication-level
accuracy claim. No upward EKF reacquisition cue was needed during this
cruise interval. Acquisition and silhouette attitude assistance remain
declared assumptions. Touchdown sink speed was 0.875 m/s. Acceptance
thresholds are local engineering checks, not criteria supplied by the
professor. The result demonstrates a restricted classical baseline.

The rover's task-specific `WP_SPEED` is synchronized with the supervisor's
1.3 m/s upper bound. Its earlier 0.8 m/s default had clipped higher speed
requests. The final Gazebo boat reached 1.256 m/s and continued adjusting
speed after initial acceleration; navigation/physics speed disagreement
P95 was 0.031 m/s. The independent validator measures physical speed from
Gazebo positions rather than relying only on the autopilot's report.

The professor's 12 November next steps add five requirements to the earlier
two-pass survey. The new runner is separate from the accepted baseline:

| Requirement | Implementation and evidence |
| --- | --- |
| Adjust both speeds to sustain observation | Bounded PI range regulation, image-age fallback, moving aircraft observation orbit, ArduPlane airspeed commands and ArduRover GUIDED velocity commands. `coordination.jsonl` records commands and measured physical speeds. |
| Boat upward camera tracks the fixed wing | A native Gazebo gimballed camera at boat body `[0.30,-0.12,0.30]` m. Raw upward and aircraft-camera JPEGs are archived together. |
| Shape + AprilTag relative pose and velocity | Known blue twin-hull/white swept-wing silhouette extraction with size, geometry and motion association; AprilTag ID0 on boat, ID1 under aircraft; constant-velocity Kalman fusion with uncertain shape range. Both target and relative ENU position/velocity are logged. |
| Both views and telemetry in operator GUI | Native dual camera panel, latest image timestamp, detection source, relative position/velocity and both commanded/measured speeds. Raw camera images remain available without text. |
| Classical methods before RL | PI supervisor, rate/bound constraints and existing autopilot controllers. No RL is required or claimed for this phase. |

## Reproduce

Use the existing original boat import described in
[the restoration notes](ORIGINAL_BOAT_RESTORED_20261110.md). The new runner
refuses to demonstrate an unconfigured procedural boat.

```bash
bash tools/build_airsea_wsl.sh
python3 tools/cooperative/test_vision_control.py
bash tools/run_cooperative_wsl.sh --out "$PWD/results/cooperative_run_01" --rtf 0.95
python3 tools/cooperative/validate.py --run "$PWD/results/cooperative_run_01"
python3 tools/cooperative/video.py --run "$PWD/results/cooperative_run_01" --out "$PWD/results/cooperative_video_01"
python3 tools/cooperative/clean_closeups.py --run "$PWD/results/cooperative_run_01" --out "$PWD/results/cooperative_video_01"
python3 tools/cooperative/report.py --run "$PWD/results/cooperative_run_01" --out "$PWD/results/cooperative_report_01"
```

On the configured Windows machine, run `./RunCooperative.ps1` from the
project in PowerShell. It creates a timestamped run and validates it after
landing. The default cameras request 1920 x 1080 at 12 Hz in simulation
time; actual sampling frequency and gaps are measured separately.

Takeoff and landing retain the previous normal arming, native aerodynamic
FDM and runway. Once the approach waypoint completes, GUIDED observation
uses a moving orbit centered on aircraft-camera estimates of the boat.
After the configured cruise interval it returns to the AUTO runway circuit.
Gazebo supplies the original boat's waves, wind, current, propulsion and
six-DOF motion; ArduRover closes its physical speed/steering loop. Neither
vehicle is moved along a scripted trajectory.

Fixed-wing airspeed cannot match a slow boat without stalling. The orbit
provides the needed relative geometry while both speeds change inside their
declared operating bounds. An aircraft stall margin remains in force.
The acquisition/coordination interval is reported separately from takeoff
and runway recovery; target visibility during those phases is not claimed.

## Information actually used

Ownship camera placement uses each vehicle's own EKF pose and calibrated
mount. Image position comes from JPEG pixels and calibration. Tag PnP uses
the known black-border side; silhouette range uses known dimensions.
For the aircraft silhouette, communicated aircraft IMU attitude corrects
the projected span for bank and viewpoint. This is a cooperative attitude
prior, not an independent vision-only attitude estimate.

For acquisition or after an image outage, the other vehicle's communicated
EKF position can point the camera toward it. These commands are explicitly
recorded as acquisition cues. They are never passed into the pixel
estimator and never count as successful image detections. Prediction-only
frames are also not detections. The experiment is GNSS-assisted acquisition
with visual tracking and estimation, not a claim of GNSS-denied search.

`evaluation_target_only` in the raw sensor record is used only by the
validator to calculate errors. The perception call receives a whitelist
without target position or truth. Camera targets are logged separately,
with their causal image or acquisition source. Original JPEGs, source
snapshots, exact world/config, source hashes, physical logs and MAVLink
events make the run reviewable.

## Problems found during implementation

The 1280 x 720 pilot completed its short flight but did not establish
tag fusion: aircraft tags were too small/oblique. Its upward shape track
also lost measurements for over three seconds. It is a development trial,
not final continuous-observation evidence.

The first 1080p trial identified the aircraft belly tag in actual frames,
but an aircraft blue component stole the downward boat track. The false
velocity then moved the orbit center away from the vessel. This trial was
stopped and its logs retained. The correction adds physical size/motion
association, limits boat image-track velocity and freezes the orbit center
on stale images. The actual downward camera moved to a nose/low mount to
reduce the aircraft's own occlusion. A behavioral regression includes a
large unrelated blue foreground object beside a small correct vessel.

The third full development run landed successfully but failed visual
acceptance: upward position P95 38.206 m, velocity P95 17.580 m/s and a
2.580 s measurement age. Bright-skin thresholding had fragmented the gray
underside. Full-silhouette background segmentation fixes the range cue;
an 8% radial measurement uncertainty and continuous-time acceleration
process noise avoid stale radial state dominating turns. Image ages now
include processing delay. Short predicted gimbal commands are explicitly
marked as predictions, and shutdown drains the last completed image pair.
Eight behavioral tests cover distractors, shaded skin, tag scale/ID,
measurement loss, variable sampling and delayed images. Failed trials are
retained locally and are not presented as the accepted result.

## Limits and next research work

This is a classical restricted appearance/geometry baseline. It is not a
trained general aircraft detector and has not been evaluated with clouds,
other boats/aircraft, lighting changes, blur/noise or real camera footage.
Silhouette range has much higher uncertainty than tag range, especially at
long distance and with occlusion; its covariance is logged. Known geometry
and communicated attitude remain assumptions. A distant shape detection
must not be described as equally accurate to a near tag measurement.

The virtual gimbals have slew limits but no calibrated motor latency,
inertia, vibration or mounting compliance. Generic native aircraft FDM,
flat-ground runway and lack of aircraft/water collision coupling remain.
Sea State 1 stays fixed as requested. Camera sampling slows wall-clock
playback; a 30fps video container does not mean 30Hz live sensor acquisition.

Before making publication claims, repeat independent runs, add fixed-speed
and shape/tag-only ablations, measure long-range position/velocity error
and covariance consistency, and evaluate a broader detector under realistic
image perturbations. RL can follow this classical baseline. No paper
submission or real-world robustness is claimed by this implementation.
