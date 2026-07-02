#!/usr/bin/env python3
"""
UAS4STEM - GCS MAVLink image stream receiver
============================================
Reassembles JPEG previews sent by image_stream_sender.py over the standard
MAVLink image transmission protocol (DATA_TRANSMISSION_HANDSHAKE +
ENCAPSULATED_DATA) and displays them in a live matplotlib window.

Run on the GCS laptop:
    python3 src/image_stream_receiver.py

If Mission Planner already uses UDP port 14550, forward a copy of the
telemetry stream and point this script at the forwarded port:
    mavproxy.py --master=<vehicle-link> --out=udp:127.0.0.1:14551
    python3 src/image_stream_receiver.py --connection udp:127.0.0.1:14551

Bench test without an aircraft:
    # Terminal 1
    python3 src/image_stream_receiver.py --connection udp:127.0.0.1:14550
    # Terminal 2
    python3 src/image_stream_sender.py --image path/to/test.jpg \\
        --connection udpout:127.0.0.1:14550 --no-wait-heartbeat
"""

import argparse
import os
from datetime import datetime

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

CHUNK_PAYLOAD = 253


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


def save_jpeg(directory, jpeg_bytes, frame_number):
    os.makedirs(directory, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(directory, f"received_{frame_number:05d}_{timestamp}.jpg")
    with open(path, "wb") as f:
        f.write(jpeg_bytes)
    return path


def main():
    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image receiver")
    parser.add_argument(
        "--connection",
        default="udp:0.0.0.0:14550",
        help="MAVLink connection string (default udp:0.0.0.0:14550)",
    )
    parser.add_argument("--debug", action="store_true",
                        help="Print detailed receiver diagnostics")
    parser.add_argument("--save-received-dir",
                        help="Save each fully reassembled JPEG before decode")
    parser.add_argument("--no-display", action="store_true",
                        help="Do not open a matplotlib window")
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
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        im = ax.imshow(placeholder)
        plt.ion()
        plt.show()
    else:
        print("Display disabled (--no-display)")

    frames_shown = 0
    frames_received = 0
    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            if not args.no_display:
                plt.pause(0.01)
            continue

        msg_type = msg.get_type()
        if args.debug:
            print(f"Received MAVLink message: {msg_type}")
        if msg_type == "DATA_TRANSMISSION_HANDSHAKE":
            receiver.on_handshake(msg)
        elif msg_type == "ENCAPSULATED_DATA":
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
                print(f"Decoded JPEG: {rgb.shape[1]}x{rgb.shape[0]}")
            if not args.no_display:
                im.set_data(rgb)
                ax.set_xlim(0, rgb.shape[1])
                ax.set_ylim(rgb.shape[0], 0)
                fig.canvas.draw_idle()
                plt.pause(0.001)
                frames_shown += 1
                print(f"Displayed frame #{frames_shown} ({rgb.shape[1]}x{rgb.shape[0]})")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nReceiver stopped.")
