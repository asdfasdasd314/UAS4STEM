#!/usr/bin/env python3
"""
UAS4STEM - MAVLink image stream sender, Pi -> Pixhawk Ethernet version
======================================================================

Purpose
-------
Capture a diagnostic image on the Raspberry Pi, compress it aggressively,
split it into MAVLink image chunks, and send those MAVLink packets TO THE
PIXHAWK over the Pi/Pixhawk Ethernet network.

Field architecture assumed by this version:

    Pi camera
      -> image_stream_sender_ethernet_pixhawk.py
      -> MAVLink over UDP/IP/Ethernet to Pixhawk Ethernet IP
      -> Pixhawk MAVLink routing
      -> Pixhawk telemetry radio port
      -> air telemetry radio
      -> RF link
      -> ground telemetry radio
      -> Windows COM port, e.g. COM4 @ 57600
      -> Mission Planner and/or image_stream_receiver.py

Important distinction
---------------------
- On the Pi, this script SENDS TO the Pixhawk:

      udpout:<PIXHAWK_IP>:14550

- It does NOT send to udpin:0.0.0.0:14550. That is a listener address.
- COM4 only exists on the Windows GCS side. The Pi should not know about COM4.

Typical Pi command
------------------
Replace 192.168.144.10 with the Pixhawk IP on the Pi/Pixhawk Ethernet network:

    python3 src/image_stream_sender.py \
        --pixhawk-ip 192.168.144.10 \
        --pixhawk-port 14550 \
        --width 320 --height 240 --quality 25 \
        --max-jpeg-bytes 8000 \
        --interval 10

Bench test without the Pixhawk/radios
-------------------------------------
Terminal 1:

    python3 src/image_stream_receiver.py --connection udpin:0.0.0.0:14550

Terminal 2:

    python3 src/image_stream_sender.py \
        --image path/to/test.jpg \
        --connection udpout:127.0.0.1:14550 \
        --interval 2

Finding the Pixhawk IP from the Pi
----------------------------------
These commands do not directly say "this is the Pixhawk", but they help narrow
it down:

    ip addr       # shows the Pi's Ethernet IP, often eth0
    ip route      # shows which subnet is reachable over Ethernet
    arp -a        # shows devices the Pi has seen on the local Ethernet network

If several devices appear in arp -a, test candidates by pinging them and/or by
checking which one is emitting/responding to MAVLink heartbeats.
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from pymavlink import mavutil

MAVLINK_DATA_STREAM_IMG_JPEG = 1
CHUNK_PAYLOAD = 253

DEFAULT_PIXHAWK_PORT = 14550
DEFAULT_HEARTBEAT_TIMEOUT = 5.0
DEFAULT_SOURCE_SYSTEM = 200
DEFAULT_SOURCE_COMPONENT = 191

# Conservative defaults for a telemetry-radio safety/diagnostic image path.
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
DEFAULT_QUALITY = 25
DEFAULT_MAX_JPEG_BYTES = 8000
DEFAULT_INTERVAL = 10.0
DEFAULT_CHUNKS_PER_LOOP = 1
DEFAULT_PACKET_DELAY = 0.03


class ImageStreamTransfer:
    """Send one JPEG over DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA."""

    def __init__(self, mav_outputs, debug=False):
        self.mav_outputs = mav_outputs
        self.debug = debug
        self.active = False
        self.data = b""
        self.seq = 0
        self.packets = 0
        self.last_progress_report = 0

    def start(self, jpeg_bytes, width, height, quality):
        self.data = jpeg_bytes
        self.packets = math.ceil(len(jpeg_bytes) / CHUNK_PAYLOAD)
        self.seq = 0
        self.active = True
        self.last_progress_report = 0

        for mav in self.mav_outputs:
            mav.data_transmission_handshake_send(
                MAVLINK_DATA_STREAM_IMG_JPEG,
                len(jpeg_bytes),
                width,
                height,
                self.packets,
                CHUNK_PAYLOAD,
                quality,
            )

        if self.debug:
            print(
                "Sent handshake: "
                f"size={len(jpeg_bytes)} bytes, {width}x{height}, "
                f"packets={self.packets}, payload={CHUNK_PAYLOAD}, quality={quality}"
            )

    def tick(self, max_chunks):
        if not self.active:
            return 0

        sent = 0
        while sent < max_chunks and self.seq < self.packets:
            start = self.seq * CHUNK_PAYLOAD
            chunk = self.data[start:start + CHUNK_PAYLOAD]
            if len(chunk) < CHUNK_PAYLOAD:
                chunk = chunk + bytes(CHUNK_PAYLOAD - len(chunk))

            for mav in self.mav_outputs:
                mav.encapsulated_data_send(self.seq, chunk)

            self.seq += 1
            sent += 1

        if self.debug and self.seq != self.last_progress_report:
            report_every = max(1, math.ceil(self.packets / 10))
            if self.seq >= self.packets or self.seq - self.last_progress_report >= report_every:
                print(f"Sent chunks: {self.seq}/{self.packets}")
                self.last_progress_report = self.seq

        if self.seq >= self.packets:
            self.active = False
            if self.debug:
                print("Transfer complete")

        return sent


def drain_mavlink(master, debug=False):
    """Drain pending MAVLink messages without blocking forever."""
    drained = 0
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        drained += 1
    if debug and drained:
        print(f"Drained {drained} MAVLink messages")
    return drained


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise ValueError(f"Could not read image: {path}")
    return img


def image_stats(img):
    return int(np.min(img)), int(np.max(img)), float(np.mean(img))


class PiCameraSource:
    """Capture frames from Raspberry Pi Camera using Picamera2."""

    def __init__(
        self,
        width,
        height,
        warmup,
        exposure_us=None,
        gain=None,
        exposure_value=0.0,
        debug=False,
    ):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "Picamera2 is required for --source camera. "
                "Use --image or --source image-dir for bench testing."
            ) from exc

        self.debug = debug
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(
            main={"size": (width, height), "format": "BGR888"},
            buffer_count=2,
        )
        self.picam2.configure(config)
        self.picam2.start()

        controls = {
            "AeEnable": exposure_us is None,
            "AwbEnable": True,
        }
        if exposure_us is None:
            controls["ExposureValue"] = exposure_value
        else:
            controls["ExposureTime"] = exposure_us
        if gain is not None:
            controls["AnalogueGain"] = gain

        try:
            self.picam2.set_controls(controls)
            if self.debug:
                print(f"Camera controls: {controls}")
        except Exception as exc:
            print(f"Camera control warning: {exc}")

        if self.debug:
            print(f"Camera started with capture size {width}x{height}")
            print(f"Warming camera for {warmup:.1f}s")
        time.sleep(warmup)

    def read(self):
        img = self.picam2.capture_array("main")
        return img, "camera"

    def close(self):
        self.picam2.stop()
        self.picam2.close()


def encode_jpeg(img, width, height, quality):
    preview = cv2.resize(img, (width, height))
    ok, jpeg_buf = cv2.imencode(
        ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
    )
    if not ok:
        raise ValueError("JPEG encode failed")
    return jpeg_buf.tobytes()


def encode_preview_under_limit(
    img,
    width,
    height,
    quality,
    max_bytes,
    min_quality=10,
    min_width=160,
    min_height=120,
    debug=False,
):
    """
    Encode a small diagnostic JPEG. If it is too large for the telemetry path,
    reduce JPEG quality first, then reduce dimensions.
    """
    current_width = int(width)
    current_height = int(height)

    while current_width >= min_width and current_height >= min_height:
        current_quality = int(quality)
        while current_quality >= min_quality:
            jpeg_bytes = encode_jpeg(img, current_width, current_height, current_quality)
            if max_bytes <= 0 or len(jpeg_bytes) <= max_bytes:
                return jpeg_bytes, current_width, current_height, current_quality
            current_quality -= 5

        next_width = int(current_width * 0.85)
        next_height = int(current_height * 0.85)
        if next_width == current_width or next_height == current_height:
            break
        current_width = next_width
        current_height = next_height

    # Last resort: send the smallest/lowest-quality result even if still above limit.
    jpeg_bytes = encode_jpeg(img, max(min_width, current_width), max(min_height, current_height), min_quality)
    if debug:
        print(
            "Warning: could not fit JPEG under limit; "
            f"sending {len(jpeg_bytes)} bytes anyway."
        )
    return jpeg_bytes, max(min_width, current_width), max(min_height, current_height), min_quality


def auto_brighten(img):
    min_px, max_px, _ = image_stats(img)
    if max_px <= min_px:
        return img
    return cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)


def images_in_dir(directory):
    if not os.path.isdir(directory):
        return []
    paths = []
    for name in sorted(os.listdir(directory)):
        if not name.lower().endswith((".jpg", ".jpeg", ".png")):
            continue
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            paths.append(path)
    return paths


def save_jpeg(directory, jpeg_bytes, label, frame_number):
    os.makedirs(directory, exist_ok=True)
    safe_label = os.path.basename(label).replace(os.sep, "_") or "frame"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(directory, f"sent_{frame_number:05d}_{timestamp}_{safe_label}.jpg")
    with open(path, "wb") as f:
        f.write(jpeg_bytes)
    return path


def connection_is_serial(connection):
    prefixes = ("udpout:", "udpin:", "udpbcast:", "tcp:", "tcpin:", "tcpout:")
    return not connection.startswith(prefixes)


def build_connection(args):
    if args.connection:
        return args.connection
    if not args.pixhawk_ip:
        print("ERROR: provide --pixhawk-ip <IP> or override with --connection <mavlink-connection>.")
        print("")
        print("Example:")
        print("  python3 src/image_stream_sender.py --pixhawk-ip 192.168.144.10")
        print("")
        print("Use 'ip addr', 'ip route', and 'arp -a' on the Pi to narrow down the Pixhawk IP.")
        sys.exit(2)
    return f"udpout:{args.pixhawk_ip}:{args.pixhawk_port}"


def print_heartbeat_help(args, connection):
    print("")
    print("No MAVLink heartbeat was received.")
    print("")
    print("For this Ethernet architecture, the Pi should send MAVLink packets TO the")
    print("Pixhawk Ethernet IP, usually with a connection like:")
    print("  udpout:<PIXHAWK_IP>:14550")
    print("")
    print("Do not use udpin:0.0.0.0:14550 as the sender destination. That means")
    print("'listen here', not 'send to the Pixhawk'.")
    print("")
    print("Check these items on the Pi:")
    print("  1. Confirm the Pi Ethernet interface and subnet:")
    print("       ip addr")
    print("       ip route")
    print("  2. Look for local Ethernet peers:")
    print("       arp -a")
    print("  3. Try pinging the Pixhawk candidate IP:")
    print("       ping <candidate-ip>")
    print("  4. Confirm the Pixhawk is configured to accept MAVLink over Ethernet/UDP")
    print("     on the selected UDP port.")
    print("")
    print("Current sender settings:")
    print(f"  connection={connection}")
    print(f"  pixhawk_ip={args.pixhawk_ip}")
    print(f"  pixhawk_port={args.pixhawk_port}")
    print(f"  source_system={args.source_system}")
    print(f"  source_component={args.source_component}")


def main():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_images_dir = os.path.join(project_root, "images")

    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image sender, Pi -> Pixhawk Ethernet version")
    parser.add_argument("--pixhawk-ip", help="Pixhawk IP address on the Pi/Pixhawk Ethernet network")
    parser.add_argument("--pixhawk-port", type=int, default=DEFAULT_PIXHAWK_PORT,
                        help=f"Pixhawk MAVLink UDP port (default {DEFAULT_PIXHAWK_PORT})")
    parser.add_argument("--connection",
                        help="Override full pymavlink connection string, e.g. udpout:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Only used for serial override connections, not normal Ethernet mode")
    parser.add_argument("--wait-heartbeat", action="store_true",
                        help="Wait for a Pixhawk heartbeat before streaming. Useful for validation, but not required for UDP send-only mode.")
    parser.add_argument("--heartbeat-timeout", type=float, default=DEFAULT_HEARTBEAT_TIMEOUT,
                        help=f"Seconds to wait for heartbeat when --wait-heartbeat is set (default {DEFAULT_HEARTBEAT_TIMEOUT})")
    parser.add_argument("--image", help="Send this image file instead of using the camera")
    parser.add_argument("--source", choices=("camera", "image-dir"), default="camera",
                        help="Image source when --image is not set (default camera)")
    parser.add_argument("--images-dir", default=default_images_dir,
                        help=f"Watch this folder and cycle through images in order (default {default_images_dir})")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH,
                        help=f"Requested stream width in pixels (default {DEFAULT_WIDTH})")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                        help=f"Requested stream height in pixels (default {DEFAULT_HEIGHT})")
    parser.add_argument("--quality", type=int, default=DEFAULT_QUALITY,
                        help=f"Requested JPEG quality 1-100 (default {DEFAULT_QUALITY})")
    parser.add_argument("--max-jpeg-bytes", type=int, default=DEFAULT_MAX_JPEG_BYTES,
                        help=f"Target maximum encoded JPEG size before chunking (default {DEFAULT_MAX_JPEG_BYTES})")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL,
                        help=f"Minimum seconds between image sends (default {DEFAULT_INTERVAL})")
    parser.add_argument("--chunks-per-loop", type=int, default=DEFAULT_CHUNKS_PER_LOOP,
                        help=f"ENCAPSULATED_DATA packets sent per loop (default {DEFAULT_CHUNKS_PER_LOOP})")
    parser.add_argument("--packet-delay", type=float, default=DEFAULT_PACKET_DELAY,
                        help=f"Sleep seconds between chunk loops to avoid saturating telemetry (default {DEFAULT_PACKET_DELAY})")
    parser.add_argument("--camera-warmup", type=float, default=2.0,
                        help="Seconds to let camera auto exposure settle (default 2.0)")
    parser.add_argument("--camera-exposure-us", type=int, help="Manual camera exposure time in microseconds")
    parser.add_argument("--camera-gain", type=float, help="Manual camera analogue gain")
    parser.add_argument("--camera-exposure-value", type=float, default=0.0,
                        help="Auto-exposure compensation value (default 0.0)")
    parser.add_argument("--auto-brighten", action="store_true",
                        help="Normalize each frame before JPEG encoding for diagnostics")
    parser.add_argument("--gcs-udpout",
                        help="Optional bench-test bypass: also send image packets directly to HOST:PORT")
    parser.add_argument("--source-system", type=int, default=DEFAULT_SOURCE_SYSTEM,
                        help=f"MAVLink source system id (default {DEFAULT_SOURCE_SYSTEM})")
    parser.add_argument("--source-component", type=int, default=DEFAULT_SOURCE_COMPONENT,
                        help=f"MAVLink source component id (default {DEFAULT_SOURCE_COMPONENT})")
    parser.add_argument("--save-sent-dir", help="Save each encoded JPEG before sending")
    parser.add_argument("--debug", action="store_true", help="Print detailed sender diagnostics")
    args = parser.parse_args()

    if args.source == "image-dir" and not args.image and not os.path.isdir(args.images_dir):
        os.makedirs(args.images_dir, exist_ok=True)

    connection = build_connection(args)

    if connection_is_serial(connection) and not os.path.exists(connection):
        print(f"Warning: serial device does not exist: {connection}")
        print("Normal Ethernet mode should look like udpout:<PIXHAWK_IP>:14550")

    print(f"Connecting to Pixhawk MAVLink endpoint: {connection}")
    master = mavutil.mavlink_connection(
        connection,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    print(
        "MAVLink sender identity: "
        f"system={args.source_system}, component={args.source_component}"
    )

    # In UDP send-only mode, heartbeat reception is not guaranteed. Do not block by default.
    if args.wait_heartbeat:
        print(f"Waiting for heartbeat ({args.heartbeat_timeout:.1f}s timeout)...")
        msg = master.wait_heartbeat(timeout=args.heartbeat_timeout)
        if msg is None:
            print_heartbeat_help(args, connection)
            sys.exit(1)
        print(f"Heartbeat received (system {master.target_system})")
    else:
        print("Not waiting for heartbeat. Use --wait-heartbeat only when the UDP endpoint returns heartbeats to this script.")

    mav_outputs = [master.mav]
    if args.gcs_udpout:
        gcs_connection = f"udpout:{args.gcs_udpout}"
        print(f"Bench-test bypass enabled: also sending image packets directly to {gcs_connection}")
        gcs_master = mavutil.mavlink_connection(
            gcs_connection,
            baud=args.baud,
            source_system=args.source_system,
            source_component=args.source_component,
        )
        mav_outputs.append(gcs_master.mav)

    transfer = ImageStreamTransfer(mav_outputs, debug=args.debug)
    last_send_time = 0.0
    cached_image = None
    camera_source = None
    cycle_index = 0
    frame_number = 0

    if args.image:
        cached_image = load_image(args.image)
        print(f"Image source: fixed file {args.image}")
        print(f"Loaded frame shape: {cached_image.shape}")
    elif args.source == "image-dir":
        image_paths = images_in_dir(args.images_dir)
        if not image_paths:
            print(f"No images found in {args.images_dir}")
            return
        print(f"Image source: cycling {len(image_paths)} images in {args.images_dir}")
        for path in image_paths:
            print(f"  - {os.path.basename(path)}")
    else:
        print("Image source: Raspberry Pi Camera")
        camera_source = PiCameraSource(
            args.width,
            args.height,
            warmup=args.camera_warmup,
            exposure_us=args.camera_exposure_us,
            gain=args.camera_gain,
            exposure_value=args.camera_exposure_value,
            debug=args.debug,
        )

    print(
        "Telemetry preview settings: "
        f"requested={args.width}x{args.height}, q={args.quality}, "
        f"max_jpeg_bytes={args.max_jpeg_bytes}, interval={args.interval}s, "
        f"chunks_per_loop={args.chunks_per_loop}, packet_delay={args.packet_delay}s"
    )

    try:
        while True:
            current_time = time.time()

            drain_mavlink(master, debug=args.debug)
            transfer.tick(args.chunks_per_loop)
            if transfer.active:
                time.sleep(args.packet_delay)
                continue

            if current_time - last_send_time < args.interval:
                time.sleep(0.05)
                continue

            if args.image:
                img = cached_image
                label = args.image
            elif args.source == "image-dir":
                image_paths = images_in_dir(args.images_dir)
                if not image_paths:
                    if args.debug:
                        print(f"No images currently found in {args.images_dir}")
                    time.sleep(0.2)
                    continue
                path = image_paths[cycle_index % len(image_paths)]
                img = load_image(path)
                label = os.path.basename(path)
                cycle_index += 1
            else:
                img, label = camera_source.read()

            frame_number += 1
            if args.debug:
                min_px, max_px, mean_px = image_stats(img)
                print(
                    f"Frame #{frame_number}: source={label}, shape={img.shape} "
                    f"min={min_px} max={max_px} mean={mean_px:.1f}"
                )

            if args.auto_brighten:
                img = auto_brighten(img)
                if args.debug:
                    min_px, max_px, mean_px = image_stats(img)
                    print(
                        "Auto-brightened frame: "
                        f"min={min_px} max={max_px} mean={mean_px:.1f}"
                    )

            jpeg_bytes, actual_width, actual_height, actual_quality = encode_preview_under_limit(
                img,
                args.width,
                args.height,
                args.quality,
                args.max_jpeg_bytes,
                debug=args.debug,
            )

            if args.debug:
                print(
                    "Encoded JPEG: "
                    f"{len(jpeg_bytes)} bytes at {actual_width}x{actual_height}, q={actual_quality}"
                )

            if args.save_sent_dir:
                saved_path = save_jpeg(args.save_sent_dir, jpeg_bytes, label, frame_number)
                print(f"Saved sent JPEG: {saved_path}")

            transfer.start(jpeg_bytes, actual_width, actual_height, actual_quality)
            last_send_time = current_time
            print(
                f"Queued {label} "
                f"({len(jpeg_bytes)} bytes, {transfer.packets} packets, "
                f"{actual_width}x{actual_height}, q={actual_quality})"
            )

            while transfer.active:
                drain_mavlink(master, debug=args.debug)
                transfer.tick(args.chunks_per_loop)
                time.sleep(args.packet_delay)
    finally:
        if camera_source is not None:
            camera_source.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSender stopped.")
