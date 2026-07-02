#!/usr/bin/env python3
"""
UAS4STEM - MAVLink image stream sender
======================================
Sends the most recent image over the standard MAVLink image transmission
protocol (DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA). Skips frames
while a transfer is in progress and sends at most one image per interval.

Run on the Pi (companion computer, alongside qr_field_scanner.py):
    python3 src/image_stream_sender.py

Bench test without a camera (send a fixed file repeatedly):
    # Terminal 1
    python3 src/image_stream_receiver.py --connection udpin:0.0.0.0:14550
    # Terminal 2
    python3 src/image_stream_sender.py --image path/to/test.jpg \\
        --connection udpout:127.0.0.1:14550 --no-wait-heartbeat
"""

import argparse
import math
import os
import time
from datetime import datetime

import cv2
from pymavlink import mavutil

MAVLINK_DATA_STREAM_IMG_JPEG = 1
CHUNK_PAYLOAD = 253
DEFAULT_CONNECTION = "udpin:0.0.0.0:14550"
DEFAULT_SOURCE_SYSTEM = 200
DEFAULT_SOURCE_COMPONENT = 191


class ImageStreamTransfer:
    """Send one JPEG over DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA."""

    def __init__(self, mav, debug=False):
        self.mav = mav
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
        self.mav.data_transmission_handshake_send(
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
            return
        sent = 0
        while sent < max_chunks and self.seq < self.packets:
            start = self.seq * CHUNK_PAYLOAD
            chunk = self.data[start:start + CHUNK_PAYLOAD]
            if len(chunk) < CHUNK_PAYLOAD:
                chunk = chunk + bytes(CHUNK_PAYLOAD - len(chunk))
            self.mav.encapsulated_data_send(self.seq, chunk)
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


class PiCameraSource:
    """Capture frames from Raspberry Pi Camera using Picamera2."""

    def __init__(self, width, height, debug=False):
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
        if self.debug:
            print(f"Camera started with capture size {width}x{height}")
        time.sleep(1.0)

    def read(self):
        img = self.picam2.capture_array()
        return img, "camera"

    def close(self):
        self.picam2.stop()
        self.picam2.close()


def encode_preview(img, width, height, quality):
    preview = cv2.resize(img, (width, height))
    ok, jpeg_buf = cv2.imencode(
        ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, quality]
    )
    if not ok:
        raise ValueError("JPEG encode failed")
    return jpeg_buf.tobytes()


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
    path = os.path.join(
        directory, f"sent_{frame_number:05d}_{timestamp}_{safe_label}.jpg"
    )
    with open(path, "wb") as f:
        f.write(jpeg_bytes)
    return path


def main():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_images_dir = os.path.join(project_root, "images")

    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image sender")
    parser.add_argument(
        "--connection",
        default=DEFAULT_CONNECTION,
        help=f"MAVLink connection string (default {DEFAULT_CONNECTION})",
    )
    parser.add_argument(
        "--image",
        help="Send this image file (for bench testing without a camera)",
    )
    parser.add_argument(
        "--source",
        choices=("camera", "image-dir"),
        default="camera",
        help="Image source when --image is not set (default camera)",
    )
    parser.add_argument(
        "--images-dir",
        default=default_images_dir,
        help=f"Watch this folder and cycle through images in order (default {default_images_dir})",
    )
    parser.add_argument("--width", type=int, default=640,
                        help="Stream width in pixels (default 640)")
    parser.add_argument("--height", type=int, default=360,
                        help="Stream height in pixels (default 360)")
    parser.add_argument("--quality", type=int, default=60,
                        help="JPEG quality 1-100 (default 60)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Minimum seconds between sends (default 1.0)")
    parser.add_argument("--chunks-per-loop", type=int, default=8,
                        help="ENCAPSULATED_DATA packets sent per loop (default 8)")
    parser.add_argument("--no-wait-heartbeat", action="store_true",
                        help="Start sending immediately (for local bench tests)")
    parser.add_argument("--source-system", type=int, default=DEFAULT_SOURCE_SYSTEM,
                        help=f"MAVLink source system id (default {DEFAULT_SOURCE_SYSTEM})")
    parser.add_argument("--source-component", type=int, default=DEFAULT_SOURCE_COMPONENT,
                        help=f"MAVLink source component id (default {DEFAULT_SOURCE_COMPONENT})")
    parser.add_argument("--save-sent-dir",
                        help="Save each encoded JPEG before sending")
    parser.add_argument("--debug", action="store_true",
                        help="Print detailed sender diagnostics")
    args = parser.parse_args()

    if args.source == "image-dir" and not args.image and not os.path.isdir(args.images_dir):
        os.makedirs(args.images_dir, exist_ok=True)

    print(f"Connecting on {args.connection}...")
    master = mavutil.mavlink_connection(
        args.connection,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    print(
        "MAVLink sender identity: "
        f"system={args.source_system}, component={args.source_component}"
    )
    if args.no_wait_heartbeat:
        print("Skipping heartbeat wait (--no-wait-heartbeat)")
    else:
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"Heartbeat received (system {master.target_system})")

    transfer = ImageStreamTransfer(master.mav, debug=args.debug)
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
        camera_source = PiCameraSource(args.width, args.height, debug=args.debug)

    print(f"Stream size: {args.width}x{args.height} q={args.quality}")

    try:
        while True:
            current_time = time.time()

            drain_mavlink(master, debug=args.debug)
            transfer.tick(args.chunks_per_loop)
            if transfer.active:
                time.sleep(0.01)
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
                print(f"Frame #{frame_number}: source={label}, shape={img.shape}")

            jpeg_bytes = encode_preview(img, args.width, args.height, args.quality)
            if args.debug:
                print(f"Encoded JPEG: {len(jpeg_bytes)} bytes")
            if args.save_sent_dir:
                saved_path = save_jpeg(args.save_sent_dir, jpeg_bytes, label, frame_number)
                print(f"Saved sent JPEG: {saved_path}")

            transfer.start(jpeg_bytes, args.width, args.height, args.quality)
            last_send_time = current_time
            print(f"Queued {label} ({len(jpeg_bytes)} bytes, {transfer.packets} packets)")

            while transfer.active:
                drain_mavlink(master, debug=args.debug)
                transfer.tick(args.chunks_per_loop)
                time.sleep(0.01)
    finally:
        if camera_source is not None:
            camera_source.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSender stopped.")
