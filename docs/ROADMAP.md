# Status and future work

The fixed-wing phase is implemented in the representative Sea State 1 scene:
an original MIT Coastal EDF visual with no exposed propellers,
a coastal runway, autonomous takeoff,
waypoint flight, image-based boat localization, a second search pass updated
from image estimates, and autonomous runway recovery. See
[AIRSEA_NEXT_PHASE.md](AIRSEA_NEXT_PHASE.md) for setup, architecture and
evidence. Acceptance of a particular run requires its independent
`validation.json` to pass.

The [mutual-observation extension](COOPERATIVE_20261112.md) implements a
classical feedback baseline for both vehicle speeds, an upward boat camera,
shape/tag relative position and velocity estimates, and dual-view telemetry.
Acquisition/reacquisition uses communicated ownship EKF positions, and
aircraft attitude is a declared silhouette-scale prior. Sustained visual
coverage is evaluated over a separate coordinated cruise interval.

Future work includes independent repeated trials, fixed-speed and
shape/tag-only comparisons, clouds/distractors/noisy camera images,
general detectors, calibrated airframe/vessel parameters, Gazebo aircraft
terrain/water collision coupling, and hardware validation. Native ArduPlane
flat-ground FDM remains generic. Boat landing and RL are not demonstrated.
