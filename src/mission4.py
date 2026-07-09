# mission4.py — Fly a box at a configured AGL altitude, then return to launch

import sys
import time
import threading
import tomllib
from pathlib import Path
from pymavlink import mavutil

from maneuvers import (
    Cmd,
    MissionItem,
    configure_wpnav_limits,
    distance_between_locations,
    get_current_location,
    mode_monitor,
    mission_aborted,
    execute_mission,
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================
REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMETER_FILE = REPO_ROOT / "parameter_files" / "mission4.toml"
FEET_TO_METERS = 0.3048


def load_mission4_config():
    with PARAMETER_FILE.open("rb") as handle:
        raw = tomllib.load(handle)

    waypoints = []
    for index, waypoint in enumerate(raw.get("box_waypoints", []), start=1):
        waypoints.append(
            {
                "label": str(waypoint.get("label", f"Box Corner {index}")),
                "lat": waypoint.get("lat"),
                "lon": waypoint.get("lon"),
            }
        )

    config = {
        "connection_string": str(raw["connection_string"]),
        "box_altitude_ft": float(raw["box_altitude_ft"]),
        "box_altitude_m": float(raw["box_altitude_ft"]) * FEET_TO_METERS,
        "max_ground_speed_mps": float(raw["max_ground_speed_mps"]),
        "wpnav_accel_mps2": float(raw["wpnav_accel_mps2"]),
        "max_waypoint_distance_m": float(raw["max_waypoint_distance_m"]),
        "box_waypoints": waypoints,
    }

    if config["max_ground_speed_mps"] <= 0:
        raise ValueError("max_ground_speed_mps must be greater than zero.")
    if config["wpnav_accel_mps2"] <= 0:
        raise ValueError("wpnav_accel_mps2 must be greater than zero.")
    if config["max_waypoint_distance_m"] <= 0:
        raise ValueError("max_waypoint_distance_m must be greater than zero.")

    validate_waypoints(config["box_waypoints"])
    return config


def validate_waypoints(waypoints):
    if len(waypoints) != 4:
        raise ValueError("Mission 4 requires exactly four configured box waypoints.")

    for index, waypoint in enumerate(waypoints, start=1):
        if waypoint["lat"] is None or waypoint["lon"] is None:
            raise ValueError(
                f"Waypoint {index} is missing coordinates. "
                f"Update box_waypoints in parameter_files/mission4.toml before flying."
            )


def build_mission(config):
    mission = [
        MissionItem(
            Cmd.TAKEOFF,
            params={
                "alt": config["box_altitude_m"],
                "ground_speed_mps": config["max_ground_speed_mps"],
            },
            label=f"Take off to {config['box_altitude_ft']:.0f} ft",
        ),
    ]

    for waypoint in config["box_waypoints"]:
        mission.append(
            MissionItem(
                Cmd.GOTO,
                params={
                    "lat": waypoint["lat"],
                    "lon": waypoint["lon"],
                    "alt": config["box_altitude_m"],
                    "ground_speed_mps": config["max_ground_speed_mps"],
                },
                label=waypoint["label"],
            )
        )

    mission.append(
        MissionItem(
            Cmd.RTL,
            params={"ground_speed_mps": config["max_ground_speed_mps"]},
            label="Return to launch",
        )
    )
    return mission


def request_data_streams(master):
    print(">>> Requesting Data Streams...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )


def wait_for_gps_lock(master):
    print(">>> Waiting for GPS Lock...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if not msg:
            continue
        if msg.fix_type >= 3:
            print(f"\n>>> GPS LOCKED! (Satellites: {msg.satellites_visible})")
            return
        sys.stdout.write(f"\r    Waiting for Satellites... Fix Type: {msg.fix_type}   ")
        sys.stdout.flush()


def switch_to_guided(master):
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


def require_current_location(master):
    location = get_current_location(master, timeout=2.0)
    if location is None:
        raise RuntimeError("Could not read the current GPS position from GLOBAL_POSITION_INT.")
    return location


def calculate_waypoint_distances(current_location, waypoints):
    distances = []
    for waypoint in waypoints:
        distances.append(
            {
                "label": waypoint["label"],
                "distance_m": distance_between_locations(
                    current_location["lat"],
                    current_location["lon"],
                    waypoint["lat"],
                    waypoint["lon"],
                ),
            }
        )
    return distances


def validate_waypoint_distances(waypoint_distances, max_waypoint_distance_m):
    for waypoint in waypoint_distances:
        if waypoint["distance_m"] > max_waypoint_distance_m:
            raise ValueError(
                f"Waypoint '{waypoint['label']}' is {waypoint['distance_m']:.1f} m away, "
                f"which exceeds max_waypoint_distance_m={max_waypoint_distance_m:.1f} m."
            )


def validate_wpnav_configuration(wpnav_status):
    if wpnav_status["success"]:
        return

    raise RuntimeError(
        "Mission 4 aborted because Pixhawk WPNAV_SPEED/WPNAV_ACCEL confirmation did not match the requested safety limits."
    )


def print_preflight_summary(config, current_location, waypoint_distances, wpnav_status):
    print("\n>>> FINAL PREFLIGHT SAFETY SUMMARY")
    print(f"    Target altitude: {config['box_altitude_ft']:.1f} ft ({config['box_altitude_m']:.2f} m AGL)")
    print(f"    Target ground speed: {config['max_ground_speed_mps']:.2f} m/s")
    print(
        f"    Target acceleration limit: {config['wpnav_accel_mps2']:.2f} m/s^2 "
        f"({wpnav_status['target_accel_cm_s2']:.0f} cm/s^2)"
    )
    print(
        f"    Current GPS location: lat={current_location['lat']:.7f}, "
        f"lon={current_location['lon']:.7f}, "
        f"alt={current_location['relative_alt_m']:.2f} m AGL"
    )
    for waypoint in waypoint_distances:
        print(f"    Distance to {waypoint['label']}: {waypoint['distance_m']:.1f} m")
    print(
        f"    Confirmed WPNAV_SPEED from Pixhawk: "
        f"{wpnav_status['confirmed_speed_cm_s']:.0f} cm/s"
    )
    print(
        f"    Confirmed WPNAV_ACCEL from Pixhawk: "
        f"{wpnav_status['confirmed_accel_cm_s2']:.0f} cm/s^2"
    )


def main():
    try:
        config = load_mission4_config()
    except (FileNotFoundError, KeyError, tomllib.TOMLDecodeError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)

    mission = build_mission(config)

    print(f"Waiting for drone on {config['connection_string']}...")
    master = mavutil.mavlink_connection(config["connection_string"], source_system=200)

    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED! Heartbeat from System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    request_data_streams(master)
    wait_for_gps_lock(master)
    switch_to_guided(master)

    try:
        current_location = require_current_location(master)
        waypoint_distances = calculate_waypoint_distances(current_location, config["box_waypoints"])
        validate_waypoint_distances(waypoint_distances, config["max_waypoint_distance_m"])

        wpnav_status = configure_wpnav_limits(
            master,
            config["max_ground_speed_mps"],
            config["wpnav_accel_mps2"],
        )
        validate_wpnav_configuration(wpnav_status)

        current_location = require_current_location(master)
        waypoint_distances = calculate_waypoint_distances(current_location, config["box_waypoints"])
        validate_waypoint_distances(waypoint_distances, config["max_waypoint_distance_m"])
        print_preflight_summary(config, current_location, waypoint_distances, wpnav_status)
    except (ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        mission_aborted.set()
        sys.exit(1)

    monitor_thread = threading.Thread(target=mode_monitor, args=(master,), daemon=True)
    monitor_thread.start()

    print(">>> Arming Motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED!")

    execute_mission(master, mission)
    mission_aborted.set()


if __name__ == '__main__':
    main()
