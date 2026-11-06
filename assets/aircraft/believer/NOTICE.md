# PX4 Believer model attribution

Source: https://github.com/PX4/PX4-SITL_gazebo-classic/tree/ab7dbc572b4519238f8263ef4a288e6eb80a6076/models/believer

The model's own `model.config` explicitly declares **Apache 2.0** and names
Pavel Vechersky, with an upstream URL to ETH Zurich's RotorS simulator.
The original metadata is preserved verbatim, including its historical
Techpod description. The files are identified as Believer by their PX4
directory, model name and mesh names. See the adjacent LICENSE and
UPSTREAM.json for the fixed revision, original paths and SHA-256 hashes.
MeshLab/VCGLab exporter metadata and the propeller's original COLLADA metadata
remain in the source files.

The six meshes and upstream model/config files are redistributed unchanged
in `upstream/`. No upstream simulator plugin is loaded by this integration.
The authored mesh coordinates use Z up. The body export labels them Y_UP
and omits a visual scene; the runtime conversion creates explicit identity
visual scenes and consistent Z_UP metadata. Original geometry is preserved.

Adaptations in this project: uniform scaling to 1.6 m span (0.961 m mesh
length), white/blue materials, original tricycle landing gear/camera pod,
throttle-driven visual propeller rotation and blur, and a closer chase view.
These integration additions are MIT licensed. The visual replacement uses
the existing generic ArduPlane native FDM, not a calibrated aerodynamic,
mass, actuator or motor model of a physical Believer aircraft. Rotating
propellers illustrate commanded power and do not measure calibrated RPM.
