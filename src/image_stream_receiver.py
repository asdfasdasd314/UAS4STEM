#!/usr/bin/env python3
"""
UAS4STEM - GCS MAVLink image stream receiver
============================================
Reassembles JPEG previews sent by image_stream_sender.py over the standard
MAVLink image transmission protocol (DATA_TRANSMISSION_HANDSHAKE +
ENCAPSULATED_DATA) and displays them in a live matplotlib window.

Field telemetry-radio architecture:
    Pi camera
      -> image_stream_sender.py on the Pi
      -> Pi serial MAVLink connection to Pixhawk
      -> Pixhawk telemetry radio
      -> GCS telemetry radio
      -> Mission Planner or MAVProxy UDP forwarding
      -> image_stream_receiver.py

The receiver normally listens on UDP. Do not open the telemetry radio COM port
directly in this script if Mission Planner also needs it; a serial COM port is
usually owned by one program at a time. Use Mission Planner or MAVProxy to open
the GCS telemetry radio and forward a MAVLink copy to UDP.

Run this receiver on the GCS laptop after UDP forwarding is configured:
    python src/image_stream_receiver.py --connection udpin:0.0.0.0:14551

Windows GCS option A - MAVProxy owns the telemetry radio COM port and forwards
copies to Mission Planner and this receiver:
    mavproxy.py --master=COM5 --baudrate 57600 \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14551

    Mission Planner connects to UDP 127.0.0.1:14550.
    image_stream_receiver.py listens on udpin:0.0.0.0:14551.

Windows GCS option B - Mission Planner owns the telemetry radio COM port and
forwards MAVLink to UDP:
    Mission Planner connects to COM5 at the telemetry radio baud, commonly
    57600 or 115200, then forwards/output MAVLink UDP to 127.0.0.1:14551.

    python src/image_stream_receiver.py --connection udpin:0.0.0.0:14551

Wi-Fi/direct UDP bench testing is still useful for proving image transmission
before using the telemetry radio.

If Mission Planner already uses UDP port 14550, forward a copy of the
telemetry stream and point this script at the forwarded port:
    mavproxy.py --master=<vehicle-link> --out=udp:127.0.0.1:14551
    python3 src/image_stream_receiver.py --connection udpin:0.0.0.0:14551

Bench test without an aircraft:
    # Terminal 1
    python3 src/image_stream_receiver.py --connection udpin:0.0.0.0:14550
    # Terminal 2
    python3 src/image_stream_sender.py --image path/to/test.jpg \\
        --connection udpout:127.0.0.1:14550 --no-wait-heartbeat
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

CHUNK_PAYLOAD = 253
DEFAULT_CONNECTION = "udpin:0.0.0.0:14550"


def chunk_bytes(data_field):
    if isinstance(data_field, (bytes, bytearray)):
        return bytes(data_field)
    return bytes(bytearray(data_field))


class ImageStreamReceiver:
    """Reassemble one in-flight JPEG from MAVLink image packets."""

    def __init__(self, debug=False):
        self.debug = debug
        self.buffer = None
        self.size = 0
        self.packets = 0
        self.width = 0
        self.height = 0
        self.payload = CHUNK_PAYLOAD
        self.received = set()
        self.last_progress_report = 0

    def reset(self):
        self.buffer = None
        self.size = 0
        self.packets = 0
        self.width = 0
        self.height = 0
        self.payload = CHUNK_PAYLOAD
        self.received = set()
        self.last_progress_report = 0

    def on_handshake(self, msg):
        if msg.size == 0:
            if self.debug:
                print("Handshake requested reset: size=0")
            self.reset()
            return
        self.buffer = bytearray(msg.size)
        self.size = msg.size
        self.packets = msg.packets
        self.width = msg.width
        self.height = msg.height
        self.payload = msg.payload
        self.received = set()
        self.last_progress_report = 0
        if self.debug:
            print(
                "Handshake received: "
                f"size={msg.size} bytes, {msg.width}x{msg.height}, "
                f"packets={msg.packets}, payload={msg.payload}, jpg_quality={msg.jpg_quality}"
            )

    def on_chunk(self, msg):
        if self.buffer is None:
            if self.debug:
                print(f"Ignoring chunk {msg.seqnr}: no active handshake")
            return None
        seq = msg.seqnr
        if seq in self.received:
            if self.debug:
                print(f"Ignoring duplicate chunk {seq}")
            return None
        if seq >= self.packets:
            if self.debug:
                print(f"Ignoring out-of-range chunk {seq}; expected < {self.packets}")
            return None

        data = chunk_bytes(msg.data)
        offset = seq * CHUNK_PAYLOAD
        end = min(offset + len(data), self.size)
        self.buffer[offset:end] = data[: end - offset]
        self.received.add(seq)

        if self.debug:
            report_every = max(1, self.packets // 10)
            received_count = len(self.received)
            if (
                received_count >= self.packets
                or received_count - self.last_progress_report >= report_every
            ):
                print(f"Received chunks: {received_count}/{self.packets}")
                self.last_progress_report = received_count

        if len(self.received) >= self.packets:
            missing = sorted(set(range(self.packets)) - self.received)
            if missing:
                print(f"Warning: complete count reached but missing chunks: {missing[:20]}")
            jpeg_bytes = bytes(self.buffer)
            self.reset()
            return jpeg_bytes
        return None


def decode_jpeg_rgb(jpeg_bytes):
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def image_stats(img):
    return (
        int(np.min(img)),
        int(np.max(img)),
        float(np.mean(img)),
    )


def save_jpeg(directory, jpeg_bytes, frame_number):
    os.makedirs(directory, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(directory, f"received_{frame_number:05d}_{timestamp}.jpg")
    with open(path, "wb") as f:
        f.write(jpeg_bytes)
    return path


def save_rgb_image(directory, rgb, frame_number):
    os.makedirs(directory, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(directory, f"decoded_{frame_number:05d}_{timestamp}.jpg")
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, bgr)
    return path


def main():
    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image receiver")
    parser.add_argument(
        "--connection",
        default=DEFAULT_CONNECTION,
        help=f"MAVLink connection string (default {DEFAULT_CONNECTION})",
    )
    parser.add_argument("--debug", action="store_true",
                        help="Print detailed receiver diagnostics")
    parser.add_argument("--save-received-dir",
                        help="Save each fully reassembled JPEG before decode")
    parser.add_argument("--save-decoded-dir",
                        help="Save each decoded RGB frame after JPEG decode")
    parser.add_argument("--no-display", action="store_true",
                        help="Do not open a matplotlib window")
    parser.add_argument("--status-interval", type=float, default=5.0,
                        help="Seconds between no-image status messages (default 5.0)")
    args = parser.parse_args()

    print(f"Listening for image stream on {args.connection}...")
    master = mavutil.mavlink_connection(args.connection)
    receiver = ImageStreamReceiver(debug=args.debug)

    fig = None
    ax = None
    im = None
    if not args.no_display:
        fig, ax = plt.subplots()
        ax.set_title("MAVLink image stream")
        ax.axis("off")
        plt.ion()
        plt.show(block=False)
        print(
            "Display enabled: matplotlib live window should show the latest decoded frame "
            f"(backend={matplotlib.get_backend()})"
        )
    else:
        print("Display disabled (--no-display); decoded frames will not be shown in matplotlib")

    frames_shown = 0
    frames_received = 0
    total_messages = 0
    image_messages = 0
    last_status = time.time()
    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            if not args.no_display:
                plt.pause(0.01)
            now = time.time()
            if now - last_status >= args.status_interval:
                print(
                    "Waiting for image packets... "
                    f"total_mavlink_messages={total_messages}, "
                    f"image_messages={image_messages}, "
                    f"frames={frames_received}"
                )
                if total_messages == 0:
                    print(
                        "No MAVLink traffic is reaching this receiver. "
                        "Check the receiver port and sender target; for direct Pi-to-laptop testing, "
                        "run the sender with --gcs-udpout <laptop-ip>:14550."
                    )
                last_status = now
            continue

        total_messages += 1
        msg_type = msg.get_type()
        if args.debug:
            print(f"Received MAVLink message: {msg_type}")
        if msg_type == "DATA_TRANSMISSION_HANDSHAKE":
            image_messages += 1
            receiver.on_handshake(msg)
        elif msg_type == "ENCAPSULATED_DATA":
            image_messages += 1
            jpeg_bytes = receiver.on_chunk(msg)
            if jpeg_bytes is None:
                continue
            frames_received += 1
            print(f"Reassembled frame #{frames_received} ({len(jpeg_bytes)} bytes)")
            if args.save_received_dir:
                saved_path = save_jpeg(args.save_received_dir, jpeg_bytes, frames_received)
                print(f"Saved received JPEG: {saved_path}")
            rgb = decode_jpeg_rgb(jpeg_bytes)
            if rgb is None:
                print(
                    "Received complete image but JPEG decode failed "
                    f"({len(jpeg_bytes)} bytes)"
                )
                continue
            if args.debug:
                min_px, max_px, mean_px = image_stats(rgb)
                print(
                    f"Decoded JPEG: {rgb.shape[1]}x{rgb.shape[0]} "
                    f"min={min_px} max={max_px} mean={mean_px:.1f}"
                )
            if args.save_decoded_dir:
                saved_path = save_rgb_image(args.save_decoded_dir, rgb, frames_received)
                print(f"Saved decoded image: {saved_path}")
            if not args.no_display:
                if im is None:
                    im = ax.imshow(rgb)
                else:
                    im.set_data(rgb)
                ax.set_xlim(0, rgb.shape[1])
                ax.set_ylim(rgb.shape[0], 0)
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.05)
                frames_shown += 1
                print(f"Displayed frame #{frames_shown} ({rgb.shape[1]}x{rgb.shape[0]})")
            elif args.debug:
                print("Decoded frame not displayed because --no-display is active")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nReceiver stopped.")
