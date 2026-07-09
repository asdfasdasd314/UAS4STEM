# Test Maneuver

## Summary
The `test_maneuver` feature defines a simple MAVLink-guided altitude test that records the drone's current height, descends slowly to 10 feet above ground level, and then climbs back to the starting height. It is intended as a controlled autonomous-flight check with TOML-backed tuning for the maneuver's owned flight targets.

## Key Points
- **Current-Height Round Trip**: The maneuver starts from the vehicle's live altitude instead of a hardcoded launch height, then returns to that same altitude after the descent segment finishes.
- **Slow Vertical Motion**: Vertical movement must stay intentionally slow for system validation, with the initial configurable speed set to `0.5` meters per second in the paired parameter file.
- **10-Foot Low Point**: The descent target is fixed by configuration at 10 feet above ground so the low-altitude test remains explicit and easy to tune later if needed.
- **Altitude-Only Ownership**: This feature owns the descend-and-return behavior plus its mission-level tuning values, while shared MAVLink connection handling and generic maneuver helpers remain dependencies.

## Relevant Files
- `parameter_files/test_maneuver.toml`: Tunable altitude and speed settings for the maneuver.
- `src/test_maneuver.py`: MAVLink mission entry point for the altitude-only descend-and-return maneuver.

## Dev Mode
HACKING

## State Log
- 2026-07-09: Initialized the `test_maneuver` feature in HACKING mode with a configurable 10-foot low-altitude target and an initial 0.5 m/s slow-flight speed requirement.
- 2026-07-09: Added the `src/test_maneuver.py` entry point so the feature can be launched directly and registered by the MAVLink command system.
