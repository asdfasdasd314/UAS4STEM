# mission.py — Define and execute your mission here

import time
import sys
import threading
from pymavlink import mavutil

from maneuvers import (
    Cmd, MissionItem,
    mode_monitor, mission_aborted,
    execute_mission,
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================
CONNECTION_STRING = 'udpin:0.0.0.0:14550'

# ==============================================================================
# MISSION DEFINITION
# ==============================================================================
MISSION = [
    MissionItem(Cmd.TAKEOFF,   params={'alt': 50.0},                           label="Launch"),
    MissionItem(Cmd.GOTO,      params={'lat': None, 'lon': None, 'alt': 50.0}, label="Waypoint 1"),
    MissionItem(Cmd.SET_ROI,   params={'lat': None, 'lon': None, 'alt': 0.0},  label="Lock on target"),
    MissionItem(Cmd.GOTO,      params={'lat': None, 'lon': None, 'alt': 50.0}, label="Waypoint 2"),
    MissionItem(Cmd.ORBIT,     params={'lat': None, 'lon': None,
                                       'radius': 15.0, 'velocity': 3.0},       label="Circle POI"),
    MissionItem(Cmd.HOVER,     params={'seconds': 10.0},                       label="Hold"),
    MissionItem(Cmd.CLEAR_ROI,                                                 label="Release ROI"),
    MissionItem(Cmd.YAW_SPIN,  params={'degrees': 360, 'speed': 30},           label="Scout spin"),
    MissionItem(Cmd.GOTO,      params={'lat': None, 'lon': None, 'alt': 50.0}, label="Waypoint 3"),
    MissionItem(Cmd.RTL,                                                       label="Go home"),
]

# ==============================================================================
# MAIN
# ==============================================================================
def main():
    print(f"Waiting for drone on {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)

    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED! Heartbeat from System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    print(">>> Requesting Data Streams...")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )

    print(">>> Waiting for GPS Lock...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            if msg.fix_type >= 3:
                print(f"\n>>> GPS LOCKED! (Satellites: {msg.satellites_visible})")
                break
            else:
                sys.stdout.write(f"\r    Waiting for Satellites... Fix Type: {msg.fix_type}   ")
                sys.stdout.flush()

    print(">>> Switching to GUIDED Mode...")
    guided_mode_id = master.mode_mapping()['GUIDED']
    while master.flightmode != 'GUIDED':
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode_id
        )
        end_time = time.time() + 1
        while time.time() < end_time:
            msg = master.recv_match(blocking=False)
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