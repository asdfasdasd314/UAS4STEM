import time
from pymavlink import mavutil

# --- CONFIGURATION ---
# Pi 5 UART is usually /dev/serial0
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 921600

def main():
    print(f"1. Attempting connection on {CONNECTION_STRING}...")
    
    # Connect to the Pixhawk
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    except Exception as e:
        print(f"CRITICAL ERROR: Could not access serial port. {e}")
        print("Tip: Did you run 'sudo raspi-config' and enable the Serial Port?")
        return

    # --- TEST 1: THE LISTENER (Pixhawk -> Pi) ---
    print("2. Waiting for Heartbeat (Listening)...")
    # This waits up to 5 seconds for a signal
    msg = master.wait_heartbeat(timeout=5)
    
    if msg:
        print(f"   SUCCESS! Heartbeat received from System {master.target_system}")
        print(f"   Mode: {mavutil.mode_string_v10(msg)}")
    else:
        print("   FAIL: No Heartbeat heard.")
        print("   Troubleshoot: Check your wiring. Swap RX and TX pins.")
        return

    # --- TEST 2: THE SPEAKER (Pi -> Pixhawk) ---
    print("\n3. Testing Command Sending (Speaking)...")
    print("   Requesting 'AUTOPILOT_VERSION' message...")
    
    # We send a command to request the board capabilities
    # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, # Confirmation
        1, # Param 1: Request version
        0, 0, 0, 0, 0, 0 # Unused params
    )

    # Wait for the reply
    version_msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
    
    if version_msg:
        print("   SUCCESS! Pixhawk replied with Version info.")
        print(f"   Board Firmware: {version_msg.flight_sw_version}")
        print("\n   >>> CONNECTION VERIFIED: READY FOR FLIGHT CODE <<<")
    else:
        print("   FAIL: Pixhawk did not reply to our command.")
        print("   Troubleshoot: RX works (we heard heartbeat), but TX might be loose.")

if __name__ == "__main__":
    main()