# Validation provenance

## Final coupled fixed-wing mission, 2026-11-07

The public procedural-asset version completed a full Sea State 1 mission
(`airsea_full_02`) with ArduPlane 4.6.3 native FDM, Gazebo boat dynamics
and ArduRover 4.7.2. All 25 independent acceptance checks passed. The
aircraft took off, visited mission phases 1 through 10, localized the
boat from rendered images, updated its second survey pass from those
estimates, returned, touched down and stopped on the runway, then
automatically disarmed. There was no pilot intervention.

- Archived camera frames: 773; successful image localizations: 16
  (9 on mission phase 3 and 7 on phase 6).
- Two-pass observation span: 58.208 s.
- Position error: median 0.771 m, P95 1.263 m, maximum 1.303 m.
- Touchdown ENU: east 91.152 m, north 0.593 m; descent speed 0.355 m/s.
- Final ENU: east 112.248 m, north 0.298 m, on the runway.
- Maximum height above runway: 45.875 m.
- Both original full videos decoded successfully (255.800 s each).

The earlier full run (`airsea_full_01`) passed its original 24 checks
(13 detections, position P95 1.019 m, touchdown descent 0.355 m/s), but
the runway and shore surfaces were coplanar and caused visual depth
fighting. The shore surface was lowered by 4 cm without moving the runway
or native aircraft ground. The final full run above uses the corrected
scene and adds an explicit check for detections on both search passes.
Both runs and their distinct results remain in the local results folder;
the release evidence and videos are from `airsea_full_02`.

The checker independently re-detected the saved JPEGs and re-solved PnP;
all live estimates matched within 0.1 mm numerical tolerance. A blank
image produced no estimate, and changing ownship navigation by 10 m
shifted the estimate by 10 m. These checks support the pixel/navigation
pipeline; they do not establish performance with real noisy images.

Machine-readable results and selected original frames are in
[evidence/](evidence/). The local verification package retains the full
telemetry/native FDM logs and all original per-frame images in the
recorded-run archive, also available with the release videos.
Playback slowed during camera sampling, with fixed 2 ms Gazebo physics steps
and simulation-time videos. See AIRSEA_NEXT_PHASE.md for architecture
and model limits, including native flat-ground aircraft dynamics,
intermittent tracking and absent aircraft/water collision coupling.

## Earlier boat and fixed-camera runs

Previous private-workspace runs: calm control13/13; representative Sea State
1 and2 gauge/firmware tests17/17 each (approximately600s gauge windows);
fixed-camera image pose detection216/216 and223/223 frames. Fixed-camera
position-errorP95:5.34/5.52cm; orientation-errorP95:0.203/0.279deg.

Those recordings used optional manufacturer CAD and older water visual assets.
They are historical evidence rather than new trials of the public visual
variant. The coupled mission above is separate fresh evidence. The
simulation is not real-vessel calibration, a real-camera field test or
real-world autonomous UAV landing.
