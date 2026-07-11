# Mission Planner Script

## Summary
The `missionplanner_script` feature is a Mission Planner IronPython guided mission that arms, takes off to a configured altitude, flies four box-corner waypoints, optionally closes the box, and then commands RTL. It runs inside Mission Planner against the live vehicle connection rather than through the Pi-side MAVLink maneuver stack.

## Key Points
- **Simple Box Sequence**: GUIDED → arm → takeoff → four corners → optional return to corner 1 → RTL.
- **LOITER Safety Switch**: Selecting LOITER on the transmitter stops the script immediately and leaves the aircraft in LOITER; the script never overrides that with RTL, LAND, GUIDED, or DISARM.
- **Mirrored Parameters**: Tunables live in the paired parameter file for documentation and field editing, but Mission Planner IronPython cannot load `tomllib`, so the same values must be mirrored into the script's USER CONFIGURATION block before flight.
- **Error RTL**: Non-LOITER mission failures attempt RTL so the aircraft does not remain in GUIDED after a script fault.

## Relevant Files
- `missionplanner_script.py`: Mission Planner IronPython takeoff / box / RTL mission.
- `parameter_files/missionplanner_script.toml`: Altitude, speed, timeouts, abort mode, and box corner coordinates.

## Dev Mode
HACKING

## State Log
- 2026-07-10: Initialized the Mission Planner box-mission feature in HACKING mode and refactored the IronPython script down to a lean takeoff → box → RTL sequence with mirrored parameter-file tunables.
