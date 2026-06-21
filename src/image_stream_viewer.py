#!/usr/bin/env python3
"""
UAS4STEM - GCS MAVLink image stream viewer
==========================================
Reassembles JPEG previews sent by qr_field_scanner.py over the standard
MAVLink image transmission protocol (DATA_TRANSMISSION_HANDSHAKE +
ENCAPSULATED_DATA) and displays them in a live matplotlib window.

Run on the GCS laptop:
    python3 src/image_stream_viewer.py

If Mission Planner already uses UDP port 14550, forward a copy of the
telemetry stream and point this script at the forwarded port:
    mavproxy.py --master=<vehicle-link> --out=udp:127.0.0.1:14551
    python3 src/image_stream_viewer.py --connection udp:127.0.0.1:14551

Bench test without an aircraft (sender must also target the same port):
    # Terminal 1
    python3 src/image_stream_viewer.py --connection udp:127.0.0.1:14550
    # Terminal 2 on the Pi (or any machine with a camera + pymavlink)
    python3 src/qr_field_scanner.py --stream-images
"""

import argparse

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

    def __init__(self):
        self.buffer = None
        self.size = 0
        self.packets = 0
        self.received = set()

    def reset(self):
        self.buffer = None
        self.size = 0
        self.packets = 0
        self.received = set()

    def on_handshake(self, msg):
        if msg.size == 0:
            self.reset()
            return
        self.buffer = bytearray(msg.size)
        self.size = msg.size
        self.packets = msg.packets
        self.received = set()

    def on_chunk(self, msg):
        if self.buffer is None:
            return None
        seq = msg.seqnr
        if seq in self.received or seq >= self.packets:
            return None

        data = chunk_bytes(msg.data)
        offset = seq * CHUNK_PAYLOAD
        end = min(offset + len(data), self.size)
        self.buffer[offset:end] = data[: end - offset]
        self.received.add(seq)

        if len(self.received) >= self.packets:
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


def main():
    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image viewer")
    parser.add_argument(
        "--connection",
        default="udp:0.0.0.0:14550",
        help="MAVLink connection string (default udp:0.0.0.0:14550)",
    )
    args = parser.parse_args()

    print(f"Listening for image stream on {args.connection}...")
    master = mavutil.mavlink_connection(args.connection)
    receiver = ImageStreamReceiver()

    fig, ax = plt.subplots()
    ax.set_title("MAVLink image stream")
    placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
    im = ax.imshow(placeholder)
    plt.ion()
    plt.show()

    frames_shown = 0
    while True:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            plt.pause(0.01)
            continue

        msg_type = msg.get_type()
        if msg_type == "DATA_TRANSMISSION_HANDSHAKE":
            receiver.on_handshake(msg)
        elif msg_type == "ENCAPSULATED_DATA":
            jpeg_bytes = receiver.on_chunk(msg)
            if jpeg_bytes is None:
                continue
            rgb = decode_jpeg_rgb(jpeg_bytes)
            if rgb is None:
                print("Received complete image but JPEG decode failed")
                continue
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
        print("\nViewer stopped.")
