#!/usr/bin/env python3
"""
UAS4STEM - GCS MAVLink image stream receiver
============================================
Reassembles JPEG previews sent by the Pi image sender over the standard
MAVLink image transmission protocol:

    DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA

This receiver assumes the field architecture discussed for the drone:

    Pi camera
      -> image_stream_sender_ethernet_pixhawk.py on the Pi
      -> MAVLink over UDP/IP/Ethernet to the Pixhawk Ethernet IP
      -> Pixhawk MAVLink routing
      -> Pixhawk telemetry UART
      -> air telemetry radio
      -> ground telemetry radio
      -> Windows USB serial device such as COM4
      -> Mission Planner or MAVProxy
      -> local UDP forwarding
      -> this receiver

Important networking distinction:
    COM4 is the Windows serial device for the GCS telemetry radio. Mission
    Planner usually owns COM4. This receiver should normally NOT open COM4
    directly, because a serial COM port is usually exclusive to one process.

Recommended GCS setup:
    1. Mission Planner connects to the telemetry radio:
           COM4 @ 57600

    2. Mission Planner forwards a MAVLink copy to UDP localhost:
           127.0.0.1:14550

    3. This receiver listens for that forwarded UDP stream:
           python image_stream_receiver_gcs_udp.py

Default receiver connection:
    udpin:0.0.0.0:14550

That means:
    "Listen on UDP port 14550 on this GCS laptop."

It does NOT mean:
    "Listen on the Pixhawk"
    "Open COM4"
    "Send to the Pi"

If Mission Planner is already using UDP 14550 for something else, forward to
14551 instead and run:
    python image_stream_receiver_gcs_udp.py --connection udpin:0.0.0.0:14551

Direct COM4 mode is possible only when Mission Planner is not using COM4:
    python image_stream_receiver_gcs_udp.py --connection COM4 --baud 57600

Bench test without an aircraft:
    # Terminal 1
    python image_stream_receiver_gcs_udp.py --connection udpin:0.0.0.0:14550

    # Terminal 2
    python image_stream_sender_ethernet_pixhawk.py --image path/to/test.jpg \
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
DEFAULT_BAUD = 57600


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


def connection_is_serial(connection):
    """
    Return True for simple serial-style connection strings such as COM4 or
    /dev/ttyUSB0. UDP/TCP connection strings contain ':' and are not serial.
    """
    lowered = connection.lower()
    if lowered.startswith(("udp", "tcp")):
        return False
    return "://" not in connection and ":" not in connection


def print_networking_summary(args):
    print("")
    print("Receiver networking mode:")
    if connection_is_serial(args.connection):
        print(f"  Direct serial input: {args.connection} @ {args.baud}")
        print("  Use this only if Mission Planner is NOT connected to the same COM port.")
        print("  If Mission Planner owns COM4, use UDP forwarding instead:")
        print("      --connection udpin:0.0.0.0:14550")
    else:
        print(f"  UDP/TCP MAVLink input: {args.connection}")
        print("  Expected path:")
        print("      ground radio -> COM4 -> Mission Planner/MAVProxy -> UDP -> this script")
    print("")

def main():
    parser = argparse.ArgumentParser(description="UAS4STEM MAVLink image receiver")
    parser.add_argument(
        "--connection",
        default=DEFAULT_CONNECTION,
        help=(
            "MAVLink input connection. Default listens for Mission Planner/MAVProxy "
            f"UDP forwarding on {DEFAULT_CONNECTION}. Use COM4 only if Mission "
            "Planner is not using the radio."
        ),
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=(
            "Serial baud rate used only for direct COM-port mode, e.g. "
            "COM4 @ 57600. Ignored for UDP connections."
        ),
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
    print_networking_summary(args)

    if connection_is_serial(args.connection):
        master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    else:
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
                        "For field use, Mission Planner/MAVProxy must own COM4 and forward "
                        "a MAVLink copy to this UDP port. Check that Mission Planner is "
                        "connected to COM4 and forwarding to the same port this script is "
                        "listening on."
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
