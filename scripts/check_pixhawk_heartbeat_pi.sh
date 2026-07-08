#!/usr/bin/env bash
set -euo pipefail

CONNECTION="${1:-/dev/serial0}"
BAUD="${2:-57600}"

python3 - "$CONNECTION" "$BAUD" <<'PY'
import sys
from pymavlink import mavutil

connection = sys.argv[1]
baud = int(sys.argv[2])
is_serial = "://" not in connection and ":" not in connection

if is_serial:
    print(f"Connecting to serial device {connection} at {baud} baud...")
    m = mavutil.mavlink_connection(connection, baud=baud)
else:
    print(f"Connecting to network MAVLink endpoint {connection}...")
    m = mavutil.mavlink_connection(connection)

print("Waiting for heartbeat...")
m.wait_heartbeat(timeout=10)
print(f"Heartbeat received from system {m.target_system}, component {m.target_component}")
PY
