# Third-party sources and distribution scope

- H-SPAR: https://github.com/naviiidz/h-spar-sim, base commit
  2fb6b980e224c220e5c15a696a9933b7448f0ad8. MIT license retained in
  h-spar-sim/LICENSE. ROS package declarations and original headers retained.
- Gazebo Sim buoyancy backport: Apache-2.0; see the original source headers,
  LICENSES/Apache-2.0.txt and blueboat_gazebo/NOTICE.md. Includes the rotated
  graded-buoyancy correction from Gazebo PR3903 / commit
  93c603b123584cb7afcd4eeba9b2bc2dbaf6aaa6.
- Water shaders retain UUV Simulator Authors (2016) / Rhys Mainwaring (2019)
  Apache-2.0 attribution. Adaptations include physical phase matching,
  32-component sea states and lighting. The public water plane, normal map
  and skybox are original procedural outputs of generate_public_assets.py.
  No binary Fuel water mesh, normal texture or cubemap is redistributed.
- The rendering/encoding portions of tools/video_recorder/capture.cc follow
  Gazebo VideoRecorder (Open Source Robotics Foundation), Apache-2.0; the
  source attribution and Apache license are retained.
- Blue Robotics vehicle dimensions/specifications and numerical 16V static
  M200/weedless thrust/RPM samples are attributed in the configuration and
  PropulsionCurve.hh. These factual numerical samples are not real-vessel
  calibration. Original chart JS, CAD, product PDFs and images are excluded.
  https://bluerobotics.com/store/boat/blueboat/blueboat/
  https://docs.bluerobotics.com/thrusters/m200weedless_force_plot.js
  https://docs.bluerobotics.com/thrusters/m200weedless_rpm_plot.js
- Optional manufacturer CAD: obtain directly under Blue Robotics' terms.
  No explicit redistribution license accompanied the original CAD archive;
  no manufacturer mesh or extracted propeller geometry is in this release.
- AprilTag36h11 ID0 is generated with the external OpenCV dictionary.
  AprilTag authorship: University of Michigan APRIL Lab / AprilRobotics,
  BSD-2-Clause: https://github.com/AprilRobotics/apriltag.
  No AprilTag/OpenCV library implementation is embedded here.
- ROS/Gazebo/OpenCV are separately installed. ArduRover4.7.2 is a separately
  downloaded GPLv3 executable; it is not included. Upstream source revision:
  https://github.com/ArduPilot/ardupilot/tree/fb773567896014da007bca58358704d4a0d8516f.
  The adapter is an independent external JSON protocol implementation.
  Pymavlink2.4.49 is a separately installed LGPLv3 dependency.

The root MIT license covers original contributions only; file/package-specific
licenses and upstream notices take precedence. Demonstration media are
distributed separately from this public source tree.

Fixed-wing extension: ArduPlane 4.6.3 is a separately downloaded
GPL-3.0-or-later executable, not distributed here. Official source commit:
https://github.com/ArduPilot/ardupilot/tree/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6.
Native plane parameter/calibration conventions were checked against
Tools/autotest/models/plane.parm. The independent JSON observer, Gazebo
bridge, original procedural aircraft/runway and airborne camera/mission
code are original MIT contributions. Runtime dependencies retain their
own licenses. Small evidence images/reports in docs/evidence are original
simulation outputs, distinct from the separately delivered videos.
