# Fixed-wing coastal search and runway recovery

This extends the public BlueBoat project with the Apache-2.0 PX4 Believer
twin-motor V-tail visual, uniformly scaled to 0.961 m length and 1.6 m
span, a 600 x 18 m coastal runway, real
ArduPlane AUTO missions and a calibrated downward airborne camera.
The environment is the existing reproducible **Sea State 1** case
(Hm0=0.05 m, Tp=2 s, 32 JONSWAP components, 2 m/s wind, current).

## Reproduce

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic and OpenCV 4.6 are required.
From the project root, install dependencies as described in README.md:

```bash
bash tools/build_public_wsl.sh
bash tools/setup_sitl_wsl.sh
bash tools/setup_plane_sitl_wsl.sh
bash tools/build_airsea_wsl.sh
bash tools/run_airsea_wsl.sh --out "$PWD/results/airsea_run_01" --rtf 0.6
/usr/bin/python3 tools/airsea/validate.py --out "$PWD/results/airsea_run_01"
```

Output directories are never overwritten. GUI rendering needs a working
OpenGL desktop/WSLg. Independent file-based verification uses system OpenCV;
flight control uses the pymavlink venv installed by setup_sitl_wsl.sh.
Ports 5880/5890 and 9012/9062 must be free. The runner uses a separate
Gazebo partition and ROS domain, and stops only its own process groups.
ArduPlane and ArduRover EEPROM/log directories are isolated.

For fast flight-only debugging (no image acquisition):

```bash
"$HOME/blueboat_sitl/venv/bin/python" tools/airsea/flight.py \
  --out "$PWD/results/airsea_native_01" --speedup 8
```

## Architecture and evidence

ArduPlane 4.6.3 runs its native **plane aerodynamic and ground-contact FDM**.
AUTO commands its actual control surfaces/throttle; no script generates
the flight path. The read-only JSON master/slave state output drives the
Gazebo airframe pose and calibrated camera mount. Master servo selection
stays at its default instance 0; the slave observer cannot steer the FDM.
The observer paces its acknowledgements against Gazebo simulation time.
Playback slows to 0.05x in the two camera sampling windows while the
2 ms Gazebo physics step, 400 Hz native aircraft integration and simulation
timestamps stay intact.
The videos use simulation time; this is not accelerated physics or a
claim that capture ran at real-time wall-clock speed.
Gazebo separately simulates boat six-DOF/waves/wind/current; ArduRover
4.7.2 receives those sensors through the existing JSON bridge.

The aircraft takes off from rest, flies survey waypoints, detects AprilTag
ID0 in **native rendered images**, and solves IPPE-square PnP using its
known 0.432 m black border. Ownship world pose comes from MAVLink EKF
navigation and the fixed camera mount. Boat truth is logged only for
independent error evaluation and never enters detection, estimation or
waypoint updates. A successful first-pass image track updates the second
survey waypoint through MAVLink mission partial-write. The existing
runway circuit and NAV_LAND finish the mission; normal arming/EKF checks
remain enabled and ordinary automatic disarming is required.

The standard ideal SITL accelerometer calibration is explicit in the
parameter file, following upstream's native plane test configuration.
It represents simulator calibration, not a calibration of real hardware.
Airframe mass and wing area follow native plane FDM (2 kg, 0.45 m2).
The Believer replacement changes appearance only: the FDM is generic,
not calibrated to a physical Believer. Propeller rotation and blur follow
autopilot throttle and illustrate powered flight; they do not report
calibrated shaft RPM. The white/blue mesh materials, original tricycle
gear and camera pod are described in assets/aircraft/believer/NOTICE.md.

To inspect the model from five actual Gazebo camera views:

```bash
bash tools/preview_aircraft_wsl.sh --out "$PWD/results/aircraft_preview_01"
```

Each run saves native FDM JSONL, MAVLink telemetry and events, exact
world/config/wave profile, per-frame JPEG sensor images, live image
estimates, camera calibration, clean chase/onboard videos and a session
manifest. `validate.py` independently re-detects every archived image,
re-solves pose, checks navigation sensitivity/blank-image rejection,
requires detections on both search passes, evaluates runway
lift-off/touchdown/stop, sink rate, mission events,
and decodes the complete videos. Its `validation.json` must pass before
the run is described as a completed visual mission.

Camera CSV header timestamps (`native_s` and `navigation_s`) are aligned
to the Gazebo clock by the bridge. Subtract `epoch_offset_s` from the
session manifest to compare them with the raw `native_fdm.jsonl` clock.

## Limits

The aircraft physics uses ArduPlane's native FDM; it is not a Gazebo
lift/drag or terrain-collision model. Ground is a flat plane aligned with
the displayed runway, so leaving the runway would not itself crash the
FDM; acceptance explicitly checks runway bounds. Aircraft/water, aircraft/
boat collisions and carrier landings are not modeled. Airframe, boat and
camera parameters remain simulation approximations. The pinhole camera
has no distortion, exposure blur or sensor noise. Tracking is an
intermittent two-pass image survey, not continuous lock under occlusion
or a demonstration of real-world target search performance.

## External components

ArduPlane binary: official stable-4.6.3, commit
`3fc7011a7d3dc047cbb17d8bd98ee94577d144c6`, separately downloaded and
SHA-256 checked by the installer; it is not distributed in this project.
ArduPilot source/license: https://github.com/ArduPilot/ardupilot
(GPL-3.0-or-later). Parameter calibration/control conventions were checked
against `Tools/autotest/models/plane.parm` at that commit. New runway,
integration, landing gear and camera code here is original MIT-licensed
code. Aircraft meshes retain their Apache-2.0 license and upstream author
attribution; see THIRD_PARTY_NOTICES.md and assets/aircraft/believer/.
