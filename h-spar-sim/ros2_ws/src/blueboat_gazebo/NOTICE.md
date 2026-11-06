# Gazebo buoyancy backport

Source: Open Source Robotics Foundation / Gazebo Sim, Apache-2.0.

- Upstream commit: `93c603b123584cb7afcd4eeba9b2bc2dbaf6aaa6`
- Upstream files: `src/systems/buoyancy/Buoyancy.cc`, `Buoyancy.hh`
- Fix: [Fix graded buoyancy for rotated collision shapes, #3903](https://github.com/gazebosim/gz-sim/pull/3903)
- Changes in this copy: rename `Buoyancy` to `BlueBoatBuoyancy` throughout;
  build a separate `libblueboat-buoyancy-system.so` so installed Gazebo is untouched.
- The water plane is transformed to each collision's local frame before
  computing submerged volume and its center. This restores pitch/roll moments.

The failed stock-plugin experiment and its telemetry are preserved in
`BlueBoatLab/results/failed_stock_buoyancy/`. A stock 8.15.0 library reproduced
the loss of restoring moments. Use this world's isolated plugin with the
installed Harmonic runtime. Do not load both buoyancy plugins on this vessel.
