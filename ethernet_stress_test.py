import time
import socket
from pymavlink import mavutil

# --- CONFIGURATION ---
CONNECTION_STRING = 'udpin:0.0.0.0:14550'
EXPECTED_IP = "192.168.13.20" 

def check_ip_settings():
    print(f"--- 1. NETWORK CONFIGURATION CHECK ---")
    print(f"   Target IP Setting: {EXPECTED_IP}")
    
    # SMART CHECK: We try to determine which IP is used to talk to the Pixhawk
    # We don't actually send data, just ask the OS "If I wanted to talk to 192.168.13.10, which IP would I use?"
    current_ip = "Unknown"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # We assume Pixhawk is at .10
        s.connect(("192.168.13.10", 1))
        current_ip = s.getsockname()[0]
        s.close()
    except Exception:
        current_ip = "Network Unreachable"

    print(f"   Detected IP: {current_ip}")
    
    if current_ip == EXPECTED_IP:
        print("   >>> CONFIGURATION PASS: Pi is correctly set up.")
        return True
    else:
        print(f"   >>> CONFIGURATION WARNING: Detected {current_ip} instead of {EXPECTED_IP}")
        # We don't stop the script anymore, just warn
        return False

def connect_vehicle():
    print(f"\n--- 2. CONNECTING TO PIXHAWK ---")
    print(f"   Listening on {CONNECTION_STRING}...")
    
    # Create the connection
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    
    print("   Waiting for Heartbeat (Ensure Pixhawk is powered!)...")
    # Wait specifically for a heartbeat
    vehicle.wait_heartbeat()
    
    print(f"   >>> SUCCESS: Connected to System ID {vehicle.target_system}!")
    return vehicle

def test_bandwidth(vehicle, duration=5):
    print(f"\n[TEST 3] BANDWIDTH STRESS TEST ({duration}s)")
    print("   Requesting ALL data streams at 50 Hz...")
    
    vehicle.mav.request_data_stream_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1
    )
    
    start_time = time.time()
    packet_count = 0
    
    while time.time() - start_time < duration:
        msg = vehicle.recv_match(blocking=True, timeout=0.1)
        if msg:
            packet_count += 1
            
    actual_rate = packet_count / duration
    print(f"   RESULTS: Received {packet_count} packets.")
    print(f"   Throughput: {actual_rate:.1f} messages/sec")
    
    if actual_rate > 100:
        print("   STATUS: PASS")
        return True
    else:
        print("   STATUS: WARNING (Low throughput)")
        return False

def test_latency(vehicle, iterations=20):
    print(f"\n[TEST 4] LATENCY TEST ({iterations} iterations)")
    print("   Sending Pings...")
    
    total_time = 0
    lost_packets = 0
    
    for i in range(iterations):
        t_start = time.time()
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        t_end = time.time()
        
        if ack:
            total_time += (t_end - t_start) * 1000
            print(".", end="", flush=True)
        else:
            lost_packets += 1
            print("X", end="", flush=True)
            
    avg_latency = total_time / (iterations - lost_packets) if lost_packets < iterations else 0
    print(f"\n   RESULTS: Avg Latency: {avg_latency:.2f} ms | Loss: {lost_packets}/{iterations}")
    
    if lost_packets == 0 and avg_latency < 20:
        print("   STATUS: PASS")
        return True
    else:
        print("   STATUS: FAIL")
        return False

def test_bulk_transfer(vehicle):
    print(f"\n[TEST 5] BULK TRANSFER (Mission Upload)")
    
    vehicle.mav.mission_clear_all_send(vehicle.target_system, vehicle.target_component)
    vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=2)
    
    count = 50
    vehicle.mav.mission_count_send(vehicle.target_system, vehicle.target_component, count, 0)
    
    for i in range(count):
        req = vehicle.recv_match(type='MISSION_REQUEST', blocking=True, timeout=2)
        if not req or req.seq != i:
            print(f"   FAIL: Timeout or Sync Error at item {i}")
            return False
            
        vehicle.mav.mission_item_int_send(
            vehicle.target_system, vehicle.target_component,
            i, 3, 16, 0, 1, 0, 0, 0, 0, 
            int(-35.36 * 1e7), int(149.16 * 1e7), 20, 0
        )
    
    ack = vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == 0:
        print("   STATUS: PASS (50 Items Uploaded)")
        return True
    else:
        print("   STATUS: FAIL (No Final ACK)")
        return False

if __name__ == "__main__":
    check_ip_settings()
    v = connect_vehicle()
    
    p1 = test_bandwidth(v)
    p2 = test_latency(v)
    p3 = test_bulk_transfer(v)
    
    if p1 and p2 and p3:
        print("\n>>> CONCLUSION: SYSTEM IS FLIGHT READY. <<<")
    else:
        print("\n>>> CONCLUSION: ERRORS DETECTED. DO NOT FLY. <<<")