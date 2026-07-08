# MAVLink Command System

## Summary
The `mavlink_command_system` feature adds a low-bandwidth command and status channel over the existing Pixhawk MAVLink telemetry path so a Ground Control Station can start and stop pre-defined Raspberry Pi maneuver and background scripts after normal Wi-Fi access is gone.

## Key Points
- **Split GCS Roles**: The ground station uses a decision-only sender console and a separate passive receiver so operator commands are not buried in status logs.
- **Single Pi Daemon**: The Raspberry Pi runs one daemon that listens for commands, supervises child processes, relays structured status frames, and triggers RTL plus process shutdown when the command link is lost.
- **Pre-Registered Scripts Only**: Commands may target only whitelisted repo-relative scripts from the paired parameter file, which keeps the system focused on reusable named maneuvers and controlled background utilities.
- **One Maneuver Rule**: Only one maneuver script may run at a time, while multiple background scripts may run concurrently.
- **STATUSTEXT Protocol**: The feature reuses compact MAVLink `STATUSTEXT` frames with short structured prefixes for commands, acknowledgements, events, logs, and heartbeats.

## Relevant Files
- `src/mavlink_command_common.py`: Shared configuration loading, command parsing, protocol framing, and validation.
- `src/mavlink_command_daemon.py`: Raspberry Pi sender/receiver daemon and subprocess supervisor.
- `src/mavlink_command_sender.py`: GCS decision console for typed `START` and `STOP` commands.
- `src/mavlink_command_receiver.py`: GCS passive event/log receiver.
- `tests/test_mavlink_command_system.py`: Unit and integration-style coverage for the command system.

## Dev Mode
PRODUCTION-READY

## State Log
- 2026-07-08: Initialized the MAVLink command system feature file in PRODUCTION-READY mode for the new daemon, sender, and receiver workflow.
- 2026-07-08: Implemented the shared STATUSTEXT protocol, Pi daemon subprocess supervisor, split GCS sender/receiver scripts, and automated tests covering parsing, process control, and link-loss RTL behavior.
- 2026-07-08: Simplified the MAVLink command system parameter file by removing per-script metadata blocks so it only keeps feature-owned connection, timeout, identity, and script registry settings.
