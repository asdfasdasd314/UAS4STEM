#!/usr/bin/env python3
"""
UAS4STEM Phase 2 - Tower Search with LOITER_TURNS
==================================================
Reads aggregated GPS data from a Phase 1 QR_Run, flies to the tower,
executes MAV_CMD_NAV_LOITER_TURNS while scanning for QR codes,
then RTLs upon detection (or after max turns elapse).

Usage:
    python3 phase2_search.py --log-dir QR_Run_2025-03-15_12-00-00
"""

import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict
from datetime import datetime

import cv2
import numpy as np
from pymavlink import mavutil
from pyzbar.pyzbar import decode, ZBarSymbol

try:
    from picamera2 import Picamera2
    from libcamera import controls
    _HAS_CAMERA = True
except ImportError:
    Picamera2 = None
    controls = None
    _HAS_CAMERA = False

# ==============================================================================
# CONFIGURATION
# ==============================================================================
CONNECTION_STRING = "udpin:0.0.0.0:14550"
TRANSIT_ALT_M = 15.24               # 50 ft cruise altitude
SEARCH_ALT_FT_DEFAULT = 15.0
LOITER_RADIUS_M_DEFAULT = 15.0
MAX_TURNS_DEFAULT = 5
QR_COOLDOWN = 2.0

QR_KNOWN_TARGETS = {
    "https://amablog.modelaircraft.org/uas4stem/BENT-NAIL-CLAW": "defect_bent_nail_claw_hammer",
    "https://amablog.modelaircraft.org/uas4stem/BENT-METAL-BALLPEEN": "defect_bent_metal_ballpeen_hammer",
    "https://amablog.modelaircraft.org/uas4stem/CLAW-HAMMER": "poi_claw_hammer",
    "https://amablog.modelaircraft.org/uas4stem/BALLPEEN-HAMMER": "poi_ballpeen_hammer",
    "https://i.sstatic.net/dHrQl.jpg": "poi_stationary_target",
    "https://amablog.modelaircraft.org/uas4stem/MOVING-TARGET": "poi_moving_target",
    "https://amablog.modelaircraft.org/uas4stem/TOWER": "poi_tower",
}


# ==============================================================================
# PHASE 1 DATA AGGREGATION
# ==============================================================================

def aggregate_phase1(log_dir):
    json_path = os.path.join(log_dir, "mission_log.json")
    if not os.path.exists(json_path):
        print(f"ERROR: Phase 1 log not found: {json_path}")
        sys.exit(1)

    with open(json_path) as f:
        entries = json.load(f)

    groups = defaultdict(list)
    for e in entries:
        name = e.get("target_name")
        if not name:
            continue
        groups[name].append(e)

    poi_db = {}
    print("Aggregated POIs from Phase 1:")
    for name, samples in groups.items():
        poi_db[name] = {
            "lat": sum(s["lat"] for s in samples) / len(samples),
            "lon": sum(s["lon"] for s in samples) / len(samples),
            "alt_m": sum(s["alt_m"] for s in samples) / len(samples),
        }
        print(f"  {name:35s}  ({poi_db[name]['lat']:.7f}, {poi_db[name]['lon']:.7f})")

    return poi_db


# ==============================================================================
# MAVLINK HELPERS
# ==============================================================================

def connect():
    print(f"Connecting on {CONNECTION_STRING} ...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=200)
    master.wait_heartbeat()
    print(f">>> LINK ESTABLISHED — System {master.target_system}")

    if master.target_system == 0:
        master.target_system = 1

    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )
    return master


def wait_for_gps(master):
    print(">>> Waiting for GPS 3D lock ...")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg and msg.fix_type >= 3:
            print(f">>> GPS LOCKED ({msg.satellites_visible} sats)")
            return
        sys.stdout.write(f"\r  Fix type: {msg.fix_type if msg else '?'}   ")
        sys.stdout.flush()


def set_mode(master, mode_name):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"ERROR: unknown mode '{mode_name}'")
        return False

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    deadline = time.time() + 8
    while time.time() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg:
            actual = mavutil.mode_string_v10(msg)
            if actual == mode_name:
                print(f">>> MODE {mode_name} confirmed")
                return True
            sys.stdout.write(f"\r  Waiting for {mode_name} (currently {actual})   ")
            sys.stdout.flush()
        time.sleep(0.1)
    print(f"\nWARNING: could not confirm {mode_name} mode")
    return False


def arm_and_takeoff(master, alt_m):
    print(">>> Arming ...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print(">>> MOTORS ARMED")

    print(f">>> Takeoff to {alt_m:.1f} m ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if not msg:
            continue
        cur = msg.relative_alt / 1000.0
        sys.stdout.write(f"\r  Alt: {cur:.1f} / {alt_m:.1f} m   ")
        sys.stdout.flush()
        if cur >= alt_m * 0.95:
            print("\n>>> Takeoff complete")
            return


def fly_to_position(master, lat, lon, alt_m):
    print(f">>> Fly to ({lat:.6f}, {lon:.6f}) @ {alt_m:.1f} m ...")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt_m,
        0, 0, 0, 0, 0, 0, 0, 0,
    )

    deadline = time.time() + 120
    last_check = 0.0
    while time.time() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if not msg:
            continue
        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        cur_alt = msg.relative_alt / 1000.0
        dist = haversine(cur_lat, cur_lon, lat, lon)

        now = time.time()
        if now - last_check > 1.0:
            sys.stdout.write(f"\r  Dist: {dist:.1f} m  Alt: {cur_alt:.1f} m   ")
            sys.stdout.flush()
            last_check = now

        if dist <= 3.0:
            print(f"\n>>> Position reached")
            return

    print("\nWARNING: position timeout — continuing")


def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ==============================================================================
# MISSION UPLOAD — LOITER_TURNS + RTL
# ==============================================================================

def _recv_mission_request(master, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        req = master.recv_match(type='MISSION_REQUEST_INT', blocking=False)
        if req:
            return req
        req = master.recv_match(type='MISSION_REQUEST', blocking=False)
        if req:
            return req
        time.sleep(0.1)
    return None


def upload_loiter_turns_mission(master, lat, lon, alt_m, num_turns, radius_m):
    print(f">>> Upload LOITER_TURNS ({num_turns} turns, r={radius_m:.0f} m, "
          f"alt={alt_m:.1f} m) ...")

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)

    master.mav.mission_count_send(
        master.target_system, master.target_component, 2, 0
    )

    items = [
        {
            "command": mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "param1": float(num_turns),
            "param2": 0.0,
            "param3": float(radius_m),
            "param4": 0.0,
            "x": int(lat * 1e7),
            "y": int(lon * 1e7),
            "z": alt_m,
        },
        {
            "command": mavutil.mavlink.MAV_CMD_NAV_RTL,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "param1": 0.0, "param2": 0.0, "param3": 0.0, "param4": 0.0,
            "x": 0, "y": 0, "z": 0.0,
        },
    ]

    for seq, item in enumerate(items):
        req = _recv_mission_request(master)
        if not req or req.seq != seq:
            print(f"ERROR: mission upload timeout at seq {seq}")
            return False

        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            seq, item["frame"], item["command"],
            0, 1,
            item["param1"], item["param2"], item["param3"], item["param4"],
            item["x"], item["y"], item["z"], 0,
        )

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if ack and ack.type == 0:
        print(">>> Mission uploaded OK")
        return True

    print(f"ERROR: Mission upload rejected (ack type={ack.type if ack else 'timeout'})")
    return False


# ==============================================================================
# CAMERA
# ==============================================================================

def setup_camera():
    if not _HAS_CAMERA:
        print("WARNING: picamera2 not available — QR scanning disabled")
        return None

    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"size": (4608, 2592), "format": "BGR888"},
        buffer_count=2,
    )
    picam2.configure(config)
    picam2.start()

    try:
        picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": 0.0,
        })
    except Exception as e:
        print(f"Focus hint: {e}")

    print("Camera ready  (4608×2592, infinity focus)")
    time.sleep(1.0)
    return picam2


# ==============================================================================
# QR SCAN LOOP (during AUTO-mode LOITER_TURNS)
# ==============================================================================

def scan_during_loiter(master, picam2, poi_db):
    """
    Scan for QR codes during LOITER_TURNS.

    Returns a POI dict {'name', 'lat', 'lon', 'alt_m'} when a non-tower
    QR code is found and its location exists in the Phase-1 aggregation,
    or None if LOITER_TURNS completes without finding a valid POI.
    """
    last_logged = {}
    frames = 0
    last_stats = time.time()

    if picam2 is None:
        print("No camera — waiting for mission to finish ...")
        while True:
            while True:
                msg = master.recv_match(blocking=False)
                if not msg:
                    break
                if msg.get_type() == 'MISSION_CURRENT' and msg.seq >= 1:
                    print(">>> LOITER_TURNS finished (no QR)")
                    return None
            time.sleep(0.5)

    print(">>> Scanning for QR codes ...")

    while True:
        now = time.time()

        # Drain MAVLink — check if LOITER_TURNS item has finished
        while True:
            msg = master.recv_match(blocking=False)
            if not msg:
                break
            if msg.get_type() == 'MISSION_CURRENT':
                if msg.seq >= 1:
                    print(">>> LOITER_TURNS completed  (seq ≥ 1)")
                    print(">>> No (non-tower) QR code detected during scan")
                    return None

        # Capture frame
        try:
            frame = picam2.capture_array()
        except Exception as e:
            print(f"Camera error: {e}")
            time.sleep(0.5)
            continue

        if frame is None or frame.size == 0:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded = decode(gray, symbols=[ZBarSymbol.QRCODE])
        frames += 1

        for obj in decoded:
            qr_data = obj.data.decode("utf-8")
            name = QR_KNOWN_TARGETS.get(qr_data, "UNKNOWN_TARGET")
            dedup = name if name != "UNKNOWN_TARGET" else qr_data

            if now - last_logged.get(dedup, 0.0) < QR_COOLDOWN:
                continue
            last_logged[dedup] = now

            # Ignore the tower's own identifier QR
            if name == "poi_tower":
                print("  [TOWER QR] skipping (tower identifier)")
                send_gcs_status(master, "TOWER QR — ignoring")
                continue

            # Valid non-tower POI found — look up its location
            print(f"\n[QR FOUND] {name}")
            send_gcs_status(master, f"QR: {name}")

            poi = poi_db.get(name)
            if poi:
                print(f"  POI location:  ({poi['lat']:.7f}, {poi['lon']:.7f})  "
                      f"alt {poi['alt_m']:.1f} m")
                send_gcs_status(
                    master,
                    f"POI: ({poi['lat']:.6f}, {poi['lon']:.6f})"
                )
                return poi
            else:
                print(f"  (no location data for '{name}' in Phase-1 log)")
                return None

        if now - last_stats >= 30.0:
            fps = frames / (now - last_stats)
            print(f"  Scan rate: {fps:.1f} fps")
            frames = 0
            last_stats = now

        time.sleep(0.05)


# ==============================================================================
# GCS STATUS TEXT
# ==============================================================================

def send_gcs_status(master, text):
    if len(text) <= 50:
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO, text.encode("utf-8")
        )
    else:
        for i in range(0, len(text), 44):
            chunk = text[i:i + 44]
            master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO, chunk.encode("utf-8")
            )
            time.sleep(0.05)


# ==============================================================================
# CLI
# ==============================================================================

def parse_args():
    p = argparse.ArgumentParser(
        description="UAS4STEM Phase 2 — Tower QR search with LOITER_TURNS"
    )
    p.add_argument("--log-dir", required=True,
                   help="Phase-1 QR_Run_YYYY-MM-DD_HH-MM-SS directory")
    p.add_argument("--alt-ft", type=float, default=SEARCH_ALT_FT_DEFAULT,
                   help=f"Search altitude in feet (default {SEARCH_ALT_FT_DEFAULT})")
    p.add_argument("--max-turns", type=int, default=MAX_TURNS_DEFAULT,
                   help=f"Max loiter turns (default {MAX_TURNS_DEFAULT})")
    p.add_argument("--radius", type=float, default=LOITER_RADIUS_M_DEFAULT,
                   help=f"Loiter radius in meters (default {LOITER_RADIUS_M_DEFAULT})")
    return p.parse_args()


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    args = parse_args()
    search_alt_m = args.alt_ft * 0.3048

    print("=" * 56)
    print("  UAS4STEM Phase 2 — Tower Search")
    print("=" * 56)

    # 1. Aggregate Phase 1 data -----------------------------------------------
    print("\n[1/6] Aggregate Phase 1 data")
    print(f"       log-dir: {args.log_dir}")
    poi_db = aggregate_phase1(args.log_dir)

    tower = poi_db.get("poi_tower")
    if not tower:
        print("ERROR: 'poi_tower' not found in Phase 1 data")
        print(f"       Available: {list(poi_db.keys())}")
        sys.exit(1)

    # 2. Connect --------------------------------------------------------------
    print("\n[2/6] Connect & GPS lock")
    master = connect()
    wait_for_gps(master)

    # 3. Arm & takeoff --------------------------------------------------------
    print("\n[3/6] Arm & takeoff")
    set_mode(master, "GUIDED")
    arm_and_takeoff(master, TRANSIT_ALT_M)

    # 4. Fly to tower ---------------------------------------------------------
    print("\n[4/6] Transit to tower")
    fly_to_position(master, tower["lat"], tower["lon"], TRANSIT_ALT_M)

    print(f"\n      Descend to {args.alt_ft:.0f} ft ({search_alt_m:.1f} m) ...")
    fly_to_position(master, tower["lat"], tower["lon"], search_alt_m)

    # 5. Camera + LOITER_TURNS mission ----------------------------------------
    print(f"\n[5/6] Start search")
    print(f"       loiter: {args.max_turns} turns, r={args.radius:.0f} m")
    picam2 = setup_camera()

    if not upload_loiter_turns_mission(
        master, tower["lat"], tower["lon"],
        search_alt_m, args.max_turns, args.radius,
    ):
        print("ABORT — mission upload failed")
        set_mode(master, "GUIDED")
        send_gcs_status(master, "MISSION UPLOAD FAILED — RTL")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return

    set_mode(master, "AUTO")

    # 6. Scan during loiter ---------------------------------------------------
    print("\n[6/6] Scanning")
    poi_found = scan_during_loiter(master, picam2, poi_db)

    if poi_found:
        print(f"\n>>> POI found: {poi_found['name']} — flying to its location")
        send_gcs_status(master, f"POI: {poi_found['name']} — navigating")
        set_mode(master, "GUIDED")

        fly_to_position(master, poi_found["lat"], poi_found["lon"], search_alt_m)

        print(">>> POI reached — RTL now")
        send_gcs_status(master, "POI REACHED — RTL")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
    else:
        print("\n>>> No (non-tower) QR found — LOITER_TURNS finished; aircraft will RTL via mission")
        send_gcs_status(master, "NO POI FOUND — RTL via mission")

    # Wait for landing
    print(">>> Waiting for disarm ...")
    try:
        master.motors_disarmed_wait()
        print(">>> LANDED & DISARMED")
    except Exception:
        pass

    # Cleanup
    if picam2 is not None:
        picam2.stop()
        picam2.close()
        print("Camera released")

    print("\nPhase 2 complete.")


if __name__ == "__main__":
    main()
