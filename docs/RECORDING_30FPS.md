# Smooth presentation recordings

The original Believer live capture had 767 sensor images across 255.044 s:
3.003 images/s on average, a 0.998 s P95 interval and a 6.778 s maximum
interval. The video container encoded 25 fps by repeating frames. Passing
the flight/vision acceptance checks and decoding the video did not establish
smooth playback. The renderer was Mesa llvmpipe, using CPU rendering.

On the development WSL host, the installed NVIDIA Direct3D adapter was
verified as `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`. The air-sea,
model-preview and replay launchers now select the installed NVIDIA adapter
on compatible WSL hosts. Existing explicit driver choices are preserved;
`BLUEBOAT_SOFTWARE_RENDER=1` disables automatic selection. Other operating
systems keep their existing graphics configuration.

The 30 fps presentation version is a new Gazebo render of the recorded
physical states. Aircraft poses come from the original native ArduPlane
JSON log. Boat position, height and quaternion come from the original
ArduRover DataFlash SIM2/SIM messages at approximately 50 Hz. Its clock is
aligned to the dense Gazebo environment log; a strict 3 mm RMS XY agreement
is required. Position and normalized quaternion interpolation evaluate those
states at each 1/30 s output time. The original sea-state shader receives
the corresponding simulation time explicitly. Powered propeller motion
remains a throttle visualization, not calibrated shaft RPM.

Every frame is rendered afresh and written through a blocking raw RGB pipe
to the encoder. Slow rendering increases export wall time; no trajectory
time is skipped. No optical-flow interpolation or generative video is used.
The output has no captions, detection boxes or other overlays.

```bash
bash tools/build_airsea_wsl.sh
bash tools/replay_airsea_wsl.sh --run /absolute/path/to/original/run \
  --out "$PWD/results/replay_highlights_01" --mode highlights
python3 tools/airsea/check_replay.py --out "$PWD/results/replay_highlights_01"
# Use --mode full for the complete chase and onboard recordings.
```

The replay input archive includes the original DataFlash boat log, native
aircraft log, dense environment telemetry, mission/validation records and
exact scene/meshes. Extract it into a new directory, build the source project
as documented, and use that directory as `--run` above. Local mesh paths in
the original scene are rebound to the extracted files.

`timeline.csv` records each requested pose/time; `rendered_frames.csv`
records every actual render and its thumbnail signature. `check_replay.py`
requires the exact frame count, 1080p/30 fps metadata, uniform output time,
matching planned/rendered timestamps, successful full decoding and no
sustained frozen images while the camera is moving. Identical pixels while
parked over untextured ground are legitimate.

These presentation replays are not original sensor recordings and do not
constitute a new autonomous flight or image-estimation validation. The
original sensor images, live videos and 25-check acceptance report remain
the evidence for the completed mission. New presentation exports are
distributed separately so that distinction can be checked.
