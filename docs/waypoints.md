# Waypoint registry

The registry is keyed by a qualified map identity and its ROS frame. P3-03
stores Habitat and Science IDs in config/waypoints.json.
scripts/waypoint_registry.py checks the pinned map YAML and image hashes, caller
map ID/frame, required IDs, and per-waypoint map/frame consistency. Consumers
must call resolve_waypoint before dispatch; an unbound entry is rejected.

## Current binding status

P8-03 qualification `p803-fixed-r64` produced a live map covering the
simulation start and both staging sites. `config/waypoints.json` binds both
poses to map `lunar_p803_roundtrip`, frame `map`, using the saved map at
`evidence/runs/p803-fixed-r64/phase-l/phase_l_map.yaml`.

- Habitat staging: world/map pose `(0.0, 6.0, 0.0)`.
- Science staging: world/map pose `(3.3, 2.5, 0.0)`, clear of the Science
  structure at `(6.0, 3.5)`.

Dispatch uses the stable SLAM `map` frame so map-to-odometry corrections do not
move an in-progress goal. The binder verifies both saved-map cells are known
free and stores the map metadata and image hashes. The three acceptance trials
and their delivery proofs are recorded in
`evidence/runs/p803-acceptance-final2-20261113/P8-03/`.

## Verification

    python3 -m unittest discover -s tests -p 'test_waypoint_registry.py'
    python3 scripts/waypoint_registry.py --map-id lunar_p302_loop --frame-id map
    python3 scripts/waypoint_registry.py --map-id wrong_map --frame-id map
    python3 scripts/waypoint_registry.py --map-id lunar_p302_loop --frame-id odom
