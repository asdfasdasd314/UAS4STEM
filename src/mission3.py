# Payload pickup + delivery

import time
import sys
import math
from pymavlink import mavutil

CONNECTION_STRING = 'udpin:0.0.0.0:14550'
TARGET_ALTITUDE = 5.0  # Meters

SERVO_CHANNEL = 9        # AUX1 on Pixhawk (channels 9-16 map to AUX 1-8)
SERVO_PWM_OPEN  = 1900   # PWM for "open" position
SERVO_PWM_CLOSE = 1100   # PWM for "closed" position
SERVO_PWM_MID   = 1500   # PWM for neutral/center

# ==============================================================================
# SERVO CONTROL
# ==============================================================================
def set_servo(master, channel: int, pwm: int):
    """
    Command a servo output on the Pixhawk via MAV_CMD_DO_SET_SERVO.

    Args:
        channel: Output channel (9 = AUX1, 10 = AUX2, ..., 16 = AUX8)
        pwm:     PWM value in microseconds (typically 1100–1900)
    """
    pwm = max(1000, min(2000, pwm))  # Clamp to safe range
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,          # Confirmation
        channel,    # Param 1: servo output channel
        pwm,        # Param 2: PWM value (µs)
        0, 0, 0, 0, 0
    )
    print(f">>> Servo CH{channel} set to {pwm} µs")


# ==============================================================================
# MAIN SCRIPT
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
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, 1
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

    print(">>> Arming Motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED!")

    print(f">>> Taking off to {TARGET_ALTITUDE} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
    )

    print(">>> Monitoring Altitude...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if not msg:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                2, 1
            )
            continue
        current_alt = msg.relative_alt / 1000.0
        sys.stdout.write(f"\rCurrent Altitude: {current_alt:.1f} m   ")
        sys.stdout.flush()
        if current_alt >= TARGET_ALTITUDE * 0.95:
            print("\n>>> Target Altitude Reached!")
            break

    # --- Actuate servo before route (e.g. open a payload bay) ---
    set_servo(master, SERVO_CHANNEL, SERVO_PWM_OPEN)

    time.sleep(6)

    # --- Actuate servo after route (e.g. close payload bay) ---
    set_servo(master, SERVO_CHANNEL, SERVO_PWM_CLOSE)

    print(">>> Hovering for 5 seconds...")
    time.sleep(5)
    print(">>> Landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print(">>> LANDING COMPLETE.")


if __name__ == '__main__':
    main()