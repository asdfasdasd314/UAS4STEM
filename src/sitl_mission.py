import time
from pymavlink import mavutil

# --- CONFIGURATION ---
# UPDATE THIS IP AGAIN
CONNECTION_STRING = 'tcp:172.17.14.51:5762' 

def main():
    print(f"Connecting to Simulator at {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print(f"Connected to System {vehicle.target_system}")

    # 1. REQUEST DATA STREAM (This enables the message below to work!)
    print("Requesting Data Stream...")
    vehicle.mav.request_data_stream_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1
    )

    # 2. GUIDED MODE
    print("Switching to GUIDED Mode...")
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4
    )
    
    # 3. ARM
    print("Arming Motors...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    vehicle.motors_armed_wait()
    print("MOTORS ARMED!")

    # 4. TAKEOFF
    TARGET_ALTITUDE = 10 
    print(f"Taking off to {TARGET_ALTITUDE} meters...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
    )

    # 5. MONITORING LOOP (Using Relative Altitude)
    print("Monitoring Relative Altitude (AGL)...")
    while True:
        # We now ask for GLOBAL_POSITION_INT again.
        # Since we ran 'request_data_stream' above, this will NO LONGER FREEZE.
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        if msg:
            # relative_alt is in millimeters (integers)
            # Divide by 1000 to get meters
            current_alt = msg.relative_alt / 1000.0 
            
            print(f"   Altitude (AGL): {current_alt:.2f} m")
            
            if current_alt >= TARGET_ALTITUDE * 0.95:
                print("\n!!! TARGET ALTITUDE REACHED !!!")
                break

    print("Mission Script Complete. Hovering.")

if __name__ == "__main__":
    main()