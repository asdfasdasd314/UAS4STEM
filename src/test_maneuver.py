import sys
import time
import tomllib
from pathlib import Path

from pymavlink import mavutil


REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMETER_FILE = REPO_ROOT / "parameter_files" / "test_maneuver.toml"
CONNECTION_STRING = "udpin:0.0.0.0:14550"
FEET_TO_METERS = 0.3048


def load_parameters():
    with PARAMETER_FILE.open("rb") as handle:
        raw = tomllib.load(handle)

    return {
        "vertical_speed_mps": float(raw["vertical_speed_mps"]),
        "target_altitude_agl_m": float(raw["target_altitude_agl_ft"]) * FEET_TO_METERS,
    }


def wait_for_gps_lock(master):
    print(">>> Waiting for GPS Lock...")
    while True:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True)
        if not msg:
            continue
        if msg.fix_type >= 3:
            print(f"\n>>> GPS LOCKED! (Satellites: {msg.satellites_visible})")
            return
        sys.stdout.write(f"\r    Waiting for Satellites... Fix Type: {msg.fix_type}   ")
        sys.stdout.flush()


def switch_to_guided(master):
    print(">>> Switching to GUIDED Mode...")
    guided_mode_id = master.mode_mapping()["GUIDED"]

    while master.flightmode != "GUIDED":
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode_id,
        )
        end_time = time.time() + 1.0
        while time.time() < end_time:
            master.recv_match(blocking=False)
            if master.flightmode == "GUIDED":
                break
            time.sleep(0.1)
        sys.stdout.write(".")
        sys.stdout.flush()

    print("\n>>> GUIDED Mode Confirmed!")


def arm_if_needed(master):
    print(">>> Arming Motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED!")


def request_data_streams(master):
    print(">>> Requesting Data Streams...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )


def read_position(master):
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg:
            return msg


def set_vertical_speed(master, vertical_speed_mps):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        2,
        abs(vertical_speed_mps),
        -1,
        0,
        0,
        0,
        0,
    )
    print(f">>> Vertical speed cap set to {abs(vertical_speed_mps):.1f} m/s")


def hold_current_lat_lon_and_alt(master, lat_int, lon_int, altitude_m):
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat_int,
        lon_int,
        altitude_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def move_to_altitude(master, lat_int, lon_int, target_altitude_m, label):
    print(f">>> {label} to {target_altitude_m:.1f} m AGL...")

    while True:
        hold_current_lat_lon_and_alt(master, lat_int, lon_int, target_altitude_m)
        msg = read_position(master)
        current_altitude_m = msg.relative_alt / 1000.0
        sys.stdout.write(
            f"\r    Current Altitude: {current_altitude_m:.1f} m | Target: {target_altitude_m:.1f} m   "
        )
        sys.stdout.flush()
        if abs(current_altitude_m - target_altitude_m) <= 0.5:
            print(f"\n>>> {label} complete.")
            return


def main():
    params = load_parameters()

    print(f"Waiting for drone on {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)

    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED! Heartbeat from System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    request_data_streams(master)
    wait_for_gps_lock(master)
    switch_to_guided(master)
    arm_if_needed(master)
    set_vertical_speed(master, params["vertical_speed_mps"])

    start_position = read_position(master)
    start_altitude_m = start_position.relative_alt / 1000.0
    target_altitude_m = params["target_altitude_agl_m"]

    print(f">>> Starting altitude: {start_altitude_m:.1f} m AGL")
    move_to_altitude(
        master,
        start_position.lat,
        start_position.lon,
        target_altitude_m,
        "Descending",
    )
    move_to_altitude(
        master,
        start_position.lat,
        start_position.lon,
        start_altitude_m,
        "Climbing back",
    )

    print(">>> Test maneuver complete.")


if __name__ == "__main__":
    main()
