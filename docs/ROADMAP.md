# Status and future work

The fixed-wing phase is implemented in the representative Sea State 1 scene:
a 1.3 m aircraft with 1.6 m span, a coastal runway, autonomous takeoff,
waypoint flight, image-based boat localization, a second search pass updated
from image estimates, and autonomous runway recovery. See
[AIRSEA_NEXT_PHASE.md](AIRSEA_NEXT_PHASE.md) for setup, architecture and
evidence. Acceptance of a particular run requires its independent
`validation.json` to pass.

Future work includes Gazebo aircraft terrain/water collision coupling,
calibrated airframe and vessel parameters, noisy/blurred camera images,
occlusion recovery and continuous target tracking. The current aircraft uses
ArduPlane's native flat-ground aerodynamic FDM and intermittent two-pass
observations. Landing on the boat and real-world validation remain outside
the demonstrated capability.
