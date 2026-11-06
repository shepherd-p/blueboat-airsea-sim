# Recorded simulation evidence

These are outputs from the latest independently checked public mission.
`validation.json` contains independent acceptance results;
`session_manifest.json` records the flight/boat architecture and capture rates;
`mission_config.json` contains the mission/runway/camera configuration.
`onboard_boat_detection.jpg` is an unchanged rendered sensor image used by
the estimator. `flight_summary.png` plots the actual recorded FDM trajectory.

The full original JPEG input set, native FDM and MAVLink logs, exact generated
scene and clean videos are separate release attachments. Extract the recorded
run archive and follow its adjacent RECHECK.md to repeat the file-based
acceptance checks. See ../VALIDATION_HISTORY.md for provenance and
../AIRSEA_NEXT_PHASE.md for setup and model limits. These are simulation
results with intermittent two-pass observations and an ideal camera.

The Believer aircraft appearance is adapted from Apache-2.0 PX4 meshes;
aircraft_visual.json records its source, scale and generated mesh hashes.
aircraft_preview.png is an actual Gazebo view of the replacement model.
Flight dynamics use the generic native ArduPlane FDM.
