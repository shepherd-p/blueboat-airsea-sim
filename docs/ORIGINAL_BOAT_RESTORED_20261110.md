# Restore the original BlueBoat appearance

The public source used procedural hull boxes when manufacturer CAD was
absent. The fixed-wing project inherited that appearance, and the previous
camera video showed those boxes. Flight and image validation did not check
whether the boat matched the user's existing model. That omission is now
corrected; the earlier camera release describes the procedural model only.

`tools/airsea/boat_visual.py` imports an existing original BlueBoat model
and its separate body/rotor meshes. A local configuration explicitly
selects that model. Every mission checks the asset hashes and snapshots
the selected visual files beside its world, refusing to use procedural
hulls if a selected original asset is missing or has changed.

Only the body and rotor visual elements are replaced. The script asserts
that inertia, collisions, propulsion/environment plugins, landing panel
and tag are preserved. The restored body mesh is byte-for-byte identical
to the user's previous `BlueBoatLab` model, SHA-256:

```
5b15eabd85cc71e09e79a67f9f3f17df3d855c51d797973f34f389586aaadbb9
```

Each run records `boat_visual_manifest.json`. The independent validator
checks the archived mesh hashes, original CAD visual, absence of fallback
hulls, and presence of both rotors and the marker, in addition to the
existing flight and original-image localization checks.

The first restored-model mission completed the flight, but failed sustained
vision acceptance: 2.436 s on the first survey and 1.764 s on the second.
The original antenna crossed the tag's border during the second approach.
That run is retained as a failed trial, not presented as accepted evidence.
The corrected observation geometry preserves the original antenna and tag:
a 24 degree horizontal field of view and a second-pass waypoint 18 m south
of the boat position predicted from first-pass images. The initial second
pass alignment is also south of the target corridor. No boat truth enters
the prediction or camera pointing.

A north-side corridor trial made the antenna occlusion worse and was
stopped after the second survey (0.420 s accepted interval). Mesh vertices
locate the tall antenna at local y = +0.32 m. The final corridor is on the
opposite, south side; this is an observation-planning correction, not a
change to the boat or its marker.

## Use the existing original model locally

```bash
python3 tools/airsea/boat_visual.py --import-model \
  /path/to/your/blueboat_description/models/blueboat
bash tools/run_airsea_wsl.sh --out "$PWD/results/original_boat_run" --rtf 0.8
python3 tools/airsea/validate.py --out "$PWD/results/original_boat_run"
python3 tools/airsea/camera_review.py --run "$PWD/results/original_boat_run" \
  --out "$PWD/results/original_boat_video"
```

The imported model must have an `official_cad` body visual, separate left
and right rotor visuals and their meshes, plus `meshes/rotors.json`.
The private asset directory and local selection file are excluded from
the public source. Public checkouts without an imported model explicitly
report `procedural`; they do not contain the manufacturer's CAD. The
local reproduction bundle includes the exact selected assets separately.

## Completed original-model verification

Run `airsea_original_boat_20261110_03` passed all 34 checks.
Normal arming, runway takeoff, both image surveys, image-based waypoint
update, runway landing, stop and disarm completed without pilot input.
69 accepted original-image fixes were independently re-solved;
sustained intervals were 3.444 / 2.352 seconds. Three-dimensional tag-center
error P95 was 0.966 m. The truth is used for this error only.
Survey-window raw body pitch-rate P95 was 1.555 deg/s.

The video decoder checked crops against the same archived JPEGs after
encoding, source hashes and every displayed fix flag against sensor times.
The annotated video shows both passes at real speed followed by labeled
half-speed review. The clean video has no overlays and runs at real speed.
Both use timestamped original camera samples in a 30fps container; neither
claims 30Hz sensor acquisition or full-mission persistent target tracking.
Original CAD/antenna/rotor visual XML and mesh references in the actual
world match the imported files. No physical hull, marker or antenna was
removed to improve detection.

See [machine-readable verification](evidence/original_boat_20261110/validation.json)
and [encoded video provenance](evidence/original_boat_20261110/encoded_video_checks.json).
