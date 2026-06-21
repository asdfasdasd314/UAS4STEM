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
    python3 src/image_stream_receiver.py --connection udp:127.0.0.1:14550
    # Terminal 2
    python3 src/image_stream_sender.py --image path/to/test.jpg \\
        --connection udpout:127.0.0.1:14550 --no-wait-heartbeat
"""

import argparse
import math
import os
import time

import cv2
from pymavlink import mavutil

MAVLINK_DATA_STREAM_IMG_JPEG = 1
CHUNK_PAYLOAD = 253
DEFAULT_CONNECTION = "udpin:0.0.0.0:14550"


class ImageStreamTransfer:
    """Send one JPEG over DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA."""

    def __init__(self, mav):
        self.mav = mav
        self.active = False
        self.data = b""
        self.seq = 0
        self.packets = 0

    def start(self, jpeg_bytes, width, height, quality):
        self.data = jpeg_bytes
        self.packets = math.ceil(len(jpeg_bytes) / CHUNK_PAYLOAD)
        self.seq = 0
        self.active = True
        self.mav.data_transmission_handshake_send(
            MAVLINK_DATA_STREAM_IMG_JPEG,
            len(jpeg_bytes),
            width,
            height,
            self.packets,
            CHUNK_PAYLOAD,
            quality,
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
        if self.seq >= self.packets:
            self.active = False


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise ValueError(f"Could not read image: {path}")
    return img


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
    args = parser.parse_args()

    if not args.image and not os.path.isdir(args.images_dir):
        os.makedirs(args.images_dir, exist_ok=True)

    print(f"Connecting on {args.connection}...")
    master = mavutil.mavlink_connection(
        args.connection, source_system=1, source_component=191
    )
    if args.no_wait_heartbeat:
        print("Skipping heartbeat wait (--no-wait-heartbeat)")
    else:
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"Heartbeat received (system {master.target_system})")

    transfer = ImageStreamTransfer(master.mav)
    last_send_time = 0.0
    cached_image = None
    cycle_index = 0

    if args.image:
        cached_image = load_image(args.image)
        print(f"Test mode: sending {args.image} every {args.interval}s")
    else:
        image_paths = images_in_dir(args.images_dir)
        if not image_paths:
            print(f"No images found in {args.images_dir}")
            return
        print(f"Cycling {len(image_paths)} images in {args.images_dir} every {args.interval}s")
        for path in image_paths:
            print(f"  - {os.path.basename(path)}")

    print(f"Stream size: {args.width}x{args.height} q={args.quality}")

    while True:
        current_time = time.time()

        while True:
            master.recv_match(blocking=False)

        transfer.tick(args.chunks_per_loop)
        if transfer.active:
            time.sleep(0.01)
            continue

        if current_time - last_send_time < args.interval:
            time.sleep(0.05)
            continue

        img = None
        label = None
        if args.image:
            img = cached_image
            label = args.image
        else:
            image_paths = images_in_dir(args.images_dir)
            if not image_paths:
                time.sleep(0.2)
                continue
            path = image_paths[cycle_index % len(image_paths)]
            img = load_image(path)
            label = os.path.basename(path)
            cycle_index += 1

        jpeg_bytes = encode_preview(img, args.width, args.height, args.quality)
        transfer.start(jpeg_bytes, args.width, args.height, args.quality)
        last_send_time = current_time
        print(f"Queued {label} ({len(jpeg_bytes)} bytes, {transfer.packets} packets)")

        while transfer.active:
            while True:
                master.recv_match(blocking=False)
            transfer.tick(args.chunks_per_loop)
            time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSender stopped.")
