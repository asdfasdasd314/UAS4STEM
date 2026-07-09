# Mission 4

## Summary
The `mission4` feature flies a rectangular guided mission at a configured altitude and speed, validates the box against the live launch position, configures ArduPilot WPNAV limits, and then returns to launch using the shared MAVLink maneuver executor.

## Key Points
- **Parameter-Driven Route**: Mission altitude, connection settings, ground speed, and box corner coordinates live in the paired parameter file so field tuning does not require source edits.
- **Shared Guided Navigation**: The mission builds `MissionItem` steps and hands them to `maneuvers.py`, which owns the MAVLink takeoff, guided waypoint, and RTL command emission.
- **Speed And WPNAV Confirmation**: Mission 4 now requires `MAV_CMD_DO_CHANGE_SPEED` ACKs and matching `WPNAV_SPEED` / `WPNAV_ACCEL` readback before trusting the Pixhawk to fly with the requested limits.
- **Waypoint Radius Gate**: Before arming, the mission measures the current GPS distance to every box corner and rejects the plan if any configured waypoint exceeds the configured safety radius.

## Relevant Files
- `parameter_files/mission4.toml`: Mission 4 connection settings, speed, altitude, and box waypoints.
- `src/mission4.py`: Mission 4 configuration loading, preflight validation, WPNAV safety setup, and mission assembly.
- `src/maneuvers.py`: Shared MAVLink guided navigation helpers, ACK validation, and ArduPilot parameter helpers used by Mission 4.
- `tests/test_mission4_navigation.py`: Regression coverage for Mission 4 config loading, waypoint safety checks, speed-command ACK handling, and WPNAV parameter confirmation.

## Dev Mode
TESTING

## State Log
- 2026-07-09: Initialized the Mission 4 feature file in TESTING mode for the box-flight mission and its shared guided-navigation dependency.
- 2026-07-09: Moved Mission 4 tunables into `parameter_files/mission4.toml` and reissued the requested ground-speed command with each guided waypoint update and RTL setup so box legs stop defaulting to the autopilot nav speed.
- 2026-07-09: Added Mission 4 pre-arm waypoint radius checks, Pixhawk WPNAV confirmation, and `MAV_CMD_DO_CHANGE_SPEED` ACK validation so the box mission now blocks arming when speed safety limits are not positively confirmed.
- 2026-07-09: Changed in-flight `MAV_CMD_DO_CHANGE_SPEED` ACK failures from mission-fatal aborts into pilot-facing warnings so Mission 4 keeps flying its active leg unless the human operator decides to intervene.
