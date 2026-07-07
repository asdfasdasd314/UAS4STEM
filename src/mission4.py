# mission4.py — Fly a box at 10 ft AGL, then return to launch

import sys
import time
import threading
from pymavlink import mavutil

from maneuvers import (
    Cmd,
    MissionItem,
    mode_monitor,
    mission_aborted,
    execute_mission,
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================
CONNECTION_STRING = 'udpin:0.0.0.0:14550'
BOX_ALTITUDE_FT = 10.0
BOX_ALTITUDE_M = BOX_ALTITUDE_FT * 0.3048

# Enter the four corners of the box in flight order.
# Replace each None with the latitude/longitude you want to fly.
BOX_WAYPOINTS = [
    {"label": "Box Corner 1", "lat": None, "lon": None},
    {"label": "Box Corner 2", "lat": None, "lon": None},
    {"label": "Box Corner 3", "lat": None, "lon": None},
    {"label": "Box Corner 4", "lat": None, "lon": None},
]


def validate_waypoints():
    for index, waypoint in enumerate(BOX_WAYPOINTS, start=1):
        if waypoint["lat"] is None or waypoint["lon"] is None:
            raise ValueError(
                f"Waypoint {index} is missing coordinates. "
                f"Update BOX_WAYPOINTS in mission4.py before flying."
            )


MISSION = [
    MissionItem(
        Cmd.TAKEOFF,
        params={"alt": BOX_ALTITUDE_M},
        label=f"Take off to {BOX_ALTITUDE_FT:.0f} ft",
    ),
    MissionItem(
        Cmd.GOTO,
        params={"lat": BOX_WAYPOINTS[0]["lat"], "lon": BOX_WAYPOINTS[0]["lon"], "alt": BOX_ALTITUDE_M},
        label=BOX_WAYPOINTS[0]["label"],
    ),
    MissionItem(
        Cmd.GOTO,
        params={"lat": BOX_WAYPOINTS[1]["lat"], "lon": BOX_WAYPOINTS[1]["lon"], "alt": BOX_ALTITUDE_M},
        label=BOX_WAYPOINTS[1]["label"],
    ),
    MissionItem(
        Cmd.GOTO,
        params={"lat": BOX_WAYPOINTS[2]["lat"], "lon": BOX_WAYPOINTS[2]["lon"], "alt": BOX_ALTITUDE_M},
        label=BOX_WAYPOINTS[2]["label"],
    ),
    MissionItem(
        Cmd.GOTO,
        params={"lat": BOX_WAYPOINTS[3]["lat"], "lon": BOX_WAYPOINTS[3]["lon"], "alt": BOX_ALTITUDE_M},
        label=BOX_WAYPOINTS[3]["label"],
    ),
    MissionItem(Cmd.RTL, label="Return to launch"),
]


def main():
    try:
        validate_waypoints()
    except ValueError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)

    print(f"Waiting for drone on {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)

    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED! Heartbeat from System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    print(">>> Requesting Data Streams...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )

    print(">>> Waiting for GPS Lock...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if not msg:
            continue
        if msg.fix_type >= 3:
            print(f"\n>>> GPS LOCKED! (Satellites: {msg.satellites_visible})")
            break
        sys.stdout.write(f"\r    Waiting for Satellites... Fix Type: {msg.fix_type}   ")
        sys.stdout.flush()

    print(">>> Switching to GUIDED Mode...")
    guided_mode_id = master.mode_mapping()['GUIDED']
    while master.flightmode != 'GUIDED':
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode_id,
        )
        end_time = time.time() + 1
        while time.time() < end_time:
            master.recv_match(blocking=False)
            if master.flightmode == 'GUIDED':
                break
            time.sleep(0.1)
        sys.stdout.write(".")
        sys.stdout.flush()

    print("\n>>> GUIDED Mode Confirmed!")

    monitor_thread = threading.Thread(target=mode_monitor, args=(master,), daemon=True)
    monitor_thread.start()

    print(">>> Arming Motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED!")

    execute_mission(master, MISSION)
    mission_aborted.set()


if __name__ == '__main__':
    main()
