import time
import sys
from pymavlink import mavutil

# ==============================================================================
# CONFIGURATION
# ==============================================================================
CONNECTION_STRING = 'udpin:0.0.0.0:14550'
TARGET_ALTITUDE = 10  # Meters

# ==============================================================================
# MAIN SCRIPT
# ==============================================================================
def main():
    print(f"Waiting for drone on {CONNECTION_STRING}...")
    
    # 1. Connect
    # source_system=200 identifies us as a Companion Computer (Pi)
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)

    # 2. Wait for the first Heartbeat
    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED! Heartbeat from System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    # 3. Request Data Streams
    print(">>> Requesting Data Streams...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, 1
    )

    # 4. CRITICAL: Wait for GPS Lock
    # Guided mode will reject commands if there is no 3D Fix.
    print(">>> Waiting for GPS Lock...")
    while True:
        # We use recv_match so the connection stays alive and processes data
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            if msg.fix_type >= 3:
                print(f"\n>>> GPS LOCKED! (Satellites: {msg.satellites_visible})")
                break
            else:
                sys.stdout.write(f"\r    Waiting for Satellites... Fix Type: {msg.fix_type}   ")
                sys.stdout.flush()

    # 5. ROBUST Mode Switching (The Fix)
    print(">>> Switching to GUIDED Mode...")
    guided_mode_id = master.mode_mapping()['GUIDED']

    while master.flightmode != 'GUIDED':
        # Send command
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode_id
        )
        
        # LISTENING LOOP: Wait 1 second, processing messages the whole time.
        end_time = time.time() + 1
        while time.time() < end_time:
            # Check for new messages (non-blocking) to update master.flightmode
            msg = master.recv_match(blocking=False)
            if master.flightmode == 'GUIDED':
                break
            time.sleep(0.1) 

        sys.stdout.write(".")
        sys.stdout.flush()

    print("\n>>> GUIDED Mode Confirmed!")

    # 6. ARM the motors
    print(">>> Arming Motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED!")

    # 7. Takeoff
    print(f">>> Taking off to {TARGET_ALTITUDE} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
    )

    # 8. Monitor Altitude
    print(">>> Monitoring Altitude...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        
        if not msg:
            # Re-request stream if it times out
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
        
    # 9. Hover & Land
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