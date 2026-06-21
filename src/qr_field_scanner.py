#!/usr/bin/env python3
"""
UAS4STEM Phase 1 - Field QR Scanner (hardened)
================================================
Changes from bench version:
  * HEADLESS BY DEFAULT. No cv2.imshow unless --display is passed.
    Safe to run over SSH/tmux with no X session.
  * FULL-RESOLUTION CAPTURE (4608x2592). At 50 ft AGL with the 66-deg
    HFOV Camera Module 3 this yields ~71 px/ft -> ~213 px across a
    3'x3' QR target. (720p would give only ~59 px -- below the pyzbar
    reliability threshold.)
  * FOCUS LOCKED AT INFINITY by default. Everything beyond ~13 ft is
    sharp, and the lens never hunts mid-search. Use --caf to restore
    continuous autofocus for close-range bench testing.
  * PER-TARGET COOLDOWN. Each known target has its own dedup timer, so
    seeing the tower no longer suppresses logging of a hammer that
    enters frame a moment later. Every (cooled-down) sighting is logged,
    so phase 2 can average multiple fixes per target.
  * PIXEL CENTROID LOGGING. Each log entry records where in the frame
    the QR was seen, plus frame dimensions, so phase 2 can correct the
    drone-GPS fix by the target's offset from image center.
  * Annotation/drawing only happens on frames being saved (CPU saving).
  * Grayscale conversion before decode (pyzbar is faster on 1 channel).

Run on the Pi inside tmux:
    tmux new -s qr
    python3 qr_field_scanner.py
    (detach: Ctrl-B then D -- script keeps running through WiFi drops)
"""

import argparse
import cv2
import time
import os
import json
import numpy as np
from datetime import datetime
from pymavlink import mavutil
from picamera2 import Picamera2
from libcamera import controls
from pyzbar.pyzbar import decode, ZBarSymbol

# ==============================================================================
# 0. COMMAND LINE ARGUMENTS
# ==============================================================================
parser = argparse.ArgumentParser(description="UAS4STEM Phase 1 QR field scanner")
parser.add_argument("--display", action="store_true",
                    help="Show a live preview window (requires a desktop "
                         "session; never use over plain SSH)")
parser.add_argument("--caf", action="store_true",
                    help="Use continuous autofocus instead of infinity lock "
                         "(for close-range bench testing)")
parser.add_argument("--width", type=int, default=4608,
                    help="Capture width (default 4608 = full sensor)")
parser.add_argument("--height", type=int, default=2592,
                    help="Capture height (default 2592 = full sensor)")
parser.add_argument("--cooldown", type=float, default=2.0,
                    help="Per-target re-log cooldown in seconds (default 2.0)")
args = parser.parse_args()

# ==============================================================================
# 1. QR CODE TRANSLATION MAP
# ==============================================================================
# Replace the keys on the left with the exact strings from the 2025-2026 rulebook.
QR_KNOWN_TARGETS = {
    "https://amablog.modelaircraft.org/uas4stem/BENT-NAIL-CLAW": "defect_bent_nail_claw_hammer",
    "https://amablog.modelaircraft.org/uas4stem/BENT-METAL-BALLPEEN": "defect_bent_metal_ballpeen_hammer",
    "https://amablog.modelaircraft.org/uas4stem/CLAW-HAMMER": "poi_claw_hammer",
    "https://amablog.modelaircraft.org/uas4stem/BALLPEEN-HAMMER": "poi_ballpeen_hammer",
    "https://i.sstatic.net/dHrQl.jpg": "poi_stationary_target",
    "https://amablog.modelaircraft.org/uas4stem/TOWER": "poi_tower"

    # Scanning this one would be useless because its position will change
    # "https://amablog.modelaircraft.org/uas4stem/MOVING-TARGET": "poi_moving_target",
}

# ==============================================================================
# 2. DIRECTORY & LOGGING SETUP
# ==============================================================================
start_time = datetime.now()
dir_name = start_time.strftime("QR_Run_%Y-%m-%d_%H-%M-%S")
os.makedirs(dir_name, exist_ok=True)
print(f"Created directory for this run: {dir_name}")

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
images_dir = os.path.join(PROJECT_ROOT, "images")
os.makedirs(images_dir, exist_ok=True)
print(f"Debug frames will be saved to: {images_dir}")

json_log_path = os.path.join(dir_name, "mission_log.json")
with open(json_log_path, 'w') as f:
    json.dump([], f)

# Most recently received GPS state
latest_lat = 0.0
latest_lon = 0.0
latest_alt = 0.0
latest_hdg = 0.0   # heading in degrees, from GLOBAL_POSITION_INT (for phase-2 offset math)

# ==============================================================================
# 3. MAVLINK CONNECTION
# ==============================================================================
CONNECTION_STRING = 'udpin:0.0.0.0:14550'

master = mavutil.mavlink_connection(CONNECTION_STRING, source_system=1, source_component=191)
print(f"Waiting for heartbeat on {CONNECTION_STRING}...")
master.wait_heartbeat()
print(f"Heartbeat received! Link established with System {master.target_system}")

if master.target_system == 0:
    master.target_system = 1

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1
)

def send_to_gcs(text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    """Sends a text message to Mission Planner, chunking if over 50 chars."""
    text_str = str(text)
    if len(text_str) <= 50:
        master.mav.statustext_send(severity, text_str.encode('utf-8'))
    else:
        chunk_size = 44
        chunks = [text_str[i:i+chunk_size] for i in range(0, len(text_str), chunk_size)]
        total = len(chunks)
        for idx, chunk in enumerate(chunks):
            msg = f"[{idx+1}/{total}] {chunk}"
            master.mav.statustext_send(severity, msg.encode('utf-8'))
            time.sleep(0.05)

# ==============================================================================
# 4. CAMERA SETUP - FULL RESOLUTION, FIXED FOCUS
# ==============================================================================
picam2 = Picamera2()

# Still configuration at full sensor resolution. buffer_count=2 keeps memory
# reasonable (each 4608x2592 BGR frame is ~36 MB).
config = picam2.create_still_configuration(
    main={"size": (args.width, args.height), "format": "BGR888"},
    buffer_count=2
)
picam2.configure(config)
picam2.start()

try:
    if args.caf:
        picam2.set_controls({
            "AfMode": controls.AfModeEnum.Continuous,
            "AfSpeed": controls.AfSpeedEnum.Fast
        })
        print("Focus mode: CONTINUOUS AUTOFOCUS (bench mode)")
    else:
        # LensPosition 0.0 = infinity. Hyperfocal for this lens is ~4 m,
        # so everything from ~13 ft to infinity is in focus. At a 50 ft
        # search altitude the ground is always sharp and the lens never
        # hunts while the drone moves.
        picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": 0.0
        })
        print("Focus mode: LOCKED AT INFINITY (field mode)")
except Exception as e:
    print(f"Focus control warning: {e}")

print(f"Capture resolution: {args.width}x{args.height}")
print("Warming up camera sensor...")
time.sleep(1.5)

# ==============================================================================
# 5. TIMERS & STATE
# ==============================================================================
last_text_heartbeat_time = time.time()
last_sys_heartbeat_time = 0.0

TEXT_HEARTBEAT_INTERVAL = 10.0
SYS_HEARTBEAT_INTERVAL = 1.0

# Per-target cooldown: target_name -> last time we logged it.
# Unknown codes are tracked under their raw data string.
last_logged = {}

# Rolling decode-rate stats so the console (and a periodic GCS message)
# tells you the scanner is keeping up.
frames_processed = 0
stats_window_start = time.time()
STATS_INTERVAL = 30.0

# Save at most one debug frame per second (first capture in each second).
last_debug_save_second = None

headless = not args.display
print(f"Display mode: {'PREVIEW WINDOW' if args.display else 'HEADLESS'}")
print("Starting field scanner...")
send_to_gcs("QR SCRIPT: STARTED (FIELD MODE)")

# ==============================================================================
# 6. MAIN LOOP
# ==============================================================================
try:
    while True:
        current_time = time.time()

        # ---- Drain MAVLink buffer & parse GPS ----
        while True:
            msg = master.recv_match(blocking=False)
            if not msg:
                break
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                latest_lat = msg.lat / 1e7
                latest_lon = msg.lon / 1e7
                latest_alt = msg.relative_alt / 1000.0
                latest_hdg = msg.hdg / 100.0 if msg.hdg != 65535 else -1.0

        # ---- System heartbeat (keeps ArduPilot routing our STATUSTEXT) ----
        if current_time - last_sys_heartbeat_time >= SYS_HEARTBEAT_INTERVAL:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            last_sys_heartbeat_time = current_time

        # ---- Text heartbeat to GCS ----
        if current_time - last_text_heartbeat_time >= TEXT_HEARTBEAT_INTERVAL:
            send_to_gcs("QR SCRIPT: RUNNING (OK)")
            last_text_heartbeat_time = current_time

        # ---- Capture frame ----
        try:
            frame = picam2.capture_array()
        except Exception as e:
            print(f"Camera frame read error: {e}. Retrying...")
            continue

        if frame is None or frame.size == 0:
            continue

        frame_h, frame_w = frame.shape[:2]

        # ---- Save one debug frame per second so we can inspect camera view ----
        current_second = int(current_time)
        if current_second != last_debug_save_second:
            debug_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            debug_path = os.path.join(images_dir, f"frame_{debug_timestamp}.jpg")
            cv2.imwrite(debug_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            last_debug_save_second = current_second

        # ---- Decode (grayscale: ~3x less data for pyzbar to chew on) ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded_objects = decode(gray, symbols=[ZBarSymbol.QRCODE])

        frames_processed += 1

        # ---- Filter to targets whose per-target cooldown has expired ----
        hits = []   # list of (readable_name, raw_data, centroid_x, centroid_y, polygon)
        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            readable_name = QR_KNOWN_TARGETS.get(qr_data, "UNKNOWN_TARGET")
            dedup_key = readable_name if readable_name != "UNKNOWN_TARGET" else qr_data

            if current_time - last_logged.get(dedup_key, 0.0) < args.cooldown:
                continue
            last_logged[dedup_key] = current_time

            pts = obj.polygon
            if pts:
                cx = int(sum(p.x for p in pts) / len(pts))
                cy = int(sum(p.y for p in pts) / len(pts))
            else:
                cx, cy = -1, -1

            hits.append((readable_name, qr_data, cx, cy, pts))

            msg_txt = f"QR DECODED: {readable_name}"
            send_to_gcs(msg_txt)
            print(f"{msg_txt}  @ pixel ({cx},{cy})  GPS ({latest_lat:.6f},{latest_lon:.6f}) alt {latest_alt:.1f}m")

        # ---- Save annotated screenshot + append log entries ----
        if hits:
            # Draw annotations only now (saved/displayed frames only)
            for readable_name, _, cx, cy, pts in hits:
                if pts and len(pts) == 4:
                    poly = np.array([(p.x, p.y) for p in pts], dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [poly], True, (0, 255, 0), 6)
                    text_y = max(pts[0].y - 15, 40)
                    cv2.putText(frame, readable_name, (pts[0].x, text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)

            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename_only = f"qr_{timestamp_str}.jpg"
            full_filepath = os.path.join(dir_name, filename_only)
            cv2.imwrite(full_filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            print(f"Saved screenshot: {filename_only}")

            try:
                with open(json_log_path, 'r') as f:
                    log_data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                log_data = []

            for readable_name, qr_data, cx, cy, _ in hits:
                log_data.append({
                    "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                    "target_name": readable_name,
                    "raw_qr_data": qr_data,
                    "lat": round(latest_lat, 7),
                    "lon": round(latest_lon, 7),
                    "alt_m": round(latest_alt, 2),
                    "heading_deg": round(latest_hdg, 1),
                    "pixel_x": cx,
                    "pixel_y": cy,
                    "frame_w": frame_w,
                    "frame_h": frame_h,
                    "image_filename": filename_only
                })

            with open(json_log_path, 'w') as f:
                json.dump(log_data, f, indent=4)

        # ---- Periodic decode-rate report ----
        if current_time - stats_window_start >= STATS_INTERVAL:
            fps = frames_processed / (current_time - stats_window_start)
            print(f"Scanner rate: {fps:.1f} decode-fps over last {STATS_INTERVAL:.0f}s")
            frames_processed = 0
            stats_window_start = current_time

        # ---- Optional preview (bench only) ----
        if not headless:
            preview = cv2.resize(frame, (1152, 648))
            cv2.imshow("UAS4STEM Scanner (preview)", preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Quit command from keyboard. Shutting down...")
                break

except KeyboardInterrupt:
    print("\nStopping scanner (Keyboard Interrupt)...")
except Exception as e:
    print(f"\nFATAL ERROR: {e}")

finally:
    print("Executing shutdown sequence...")
    try:
        send_to_gcs("QR SCRIPT: TERMINATED", mavutil.mavlink.MAV_SEVERITY_WARNING)
    except Exception:
        pass
    if not headless:
        cv2.destroyAllWindows()
    picam2.stop()
    picam2.close()
    print("Camera released and script terminated cleanly.")
