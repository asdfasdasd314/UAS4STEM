# MAVLink Image Streaming Setup Guide

This guide explains how to send images from the Raspberry Pi camera through the Pixhawk telemetry system to the Ground Control Station (GCS).

Final architecture:

```text
Pi Camera
    ↓
image_stream_sender.py
    ↓
MAVLink over UDP/Ethernet
    ↓
Pixhawk
    ↓
Pixhawk MAVLink Router
    ↓
Telemetry Radio
    ↓
Ground Telemetry Radio
    ↓
Windows COM Port (COM4)
    ↓
Mission Planner
    ↓
MAVLink UDP Forward
    ↓
image_stream_receiver.py
```

---

# 1. Find the Pixhawk IP Address

The Raspberry Pi sends image packets **to the Pixhawk**, so it must know the Pixhawk's Ethernet IP address.

The sender uses:

```text
udpout:<PIXHAWK_IP>:14550
```

Example:

```text
udpout:192.168.144.10:14550
```

---

## Step 1 — SSH into the Raspberry Pi

Connect normally:

```bash
ssh pi@<PI_IP>
```

---

## Step 2 — Find the Pi Ethernet Interface

Run:

```bash
ip addr
```

Look for the wired Ethernet adapter:

Common names:

```text
eth0
eth1
enx...
```

Example:

```text
eth0:
    inet 192.168.144.20/24
```

This means:

```text
Pi Ethernet IP = 192.168.144.20
Network = 192.168.144.xxx
```

The Pixhawk is likely another device on this same network.

---

## Step 3 — Check the Route Table

Run:

```bash
ip route
```

Example:

```text
192.168.144.0/24 dev eth0
```

This confirms that Ethernet traffic for:

```text
192.168.144.xxx
```

goes through the Ethernet cable.

---

## Step 4 — Find Connected Ethernet Devices

Run:

```bash
arp -a
```

Example:

```text
? (192.168.144.10) at aa:bb:cc:dd:ee:ff on eth0
```

Possible Pixhawk:

```text
192.168.144.10
```

---

## Step 5 — Verify Pixhawk Connection

Ping the candidate IP:

```bash
ping 192.168.144.10
```

Expected:

```text
64 bytes from 192.168.144.10
```

If it responds, use that IP.

---

# 2. Run the Image Sender on the Pi

Start a persistent terminal session:

```bash
tmux new -s image_sender
```

Run:

```bash
python3 image_stream_sender_ethernet_pixhawk.py \
    --pixhawk-ip 192.168.144.10
```

Replace:

```text
192.168.144.10
```

with your actual Pixhawk IP.

The sender now sends:

```text
Pi
 ↓
UDP MAVLink packets
 ↓
Pixhawk Ethernet
```

The Pi does NOT send directly to the GCS laptop.

---

# 3. Connect Mission Planner

Plug the ground telemetry radio into the Windows laptop.

Mission Planner should show something like:

```text
COM4
57600
```

Connect normally.

The path is now:

```text
Pixhawk
 ↓
air telemetry radio
 ↓
ground telemetry radio
 ↓
COM4
 ↓
Mission Planner
```

---

# 4. Enable MAVLink Forwarding

Mission Planner owns COM4, so the image receiver cannot open COM4 directly.

Instead, Mission Planner forwards a copy of MAVLink.

In Mission Planner:

```text
Ctrl + F
```

Open:

```text
MAVLink
```

Add UDP forwarding:

```text
Host:
127.0.0.1

Port:
14550
```

Now the path is:

```text
COM4
 ↓
Mission Planner
 ↓
UDP localhost:14550
```

---

# 5. Start the Image Receiver on the GCS

On the Windows laptop:

```bash
python image_stream_receiver_gcs_udp.py \
    --connection udpin:127.0.0.1:14550
```

The receiver is listening to Mission Planner's forwarded packets.

---

# Complete Data Path

```text
Raspberry Pi Camera
        |
        |
        v
Image sender script
        |
        |
        | MAVLink
        | UDP/IP
        | Ethernet
        |
        v
Pixhawk Ethernet Port
        |
        |
        | MAVLink routing
        |
        v
Pixhawk Telemetry Port
        |
        |
        | UART serial
        |
        v
Air Telemetry Radio


========== RF LINK ==========


Ground Telemetry Radio
        |
        |
        | USB Serial
        |
        v
Windows COM4
        |
        |
        v
Mission Planner
        |
        |
        | UDP Forward
        |
        v
127.0.0.1:14550
        |
        |
        v
Image Receiver
```

---

# Troubleshooting

## Sender cannot reach Pixhawk

Test:

```bash
ping <PIXHAWK_IP>
```

If it fails:

- wrong IP
- Ethernet not connected
- Pixhawk networking not active

---

## Mission Planner works but receiver gets nothing

The problem is usually UDP forwarding.

Check:

```text
COM4 → Mission Planner works
Mission Planner → UDP forwarding missing
```

Make sure forwarding points to:

```text
127.0.0.1:14550
```

---

## Receiver says port already in use

Another program owns the UDP port.

Use another port.

Mission Planner:

```text
127.0.0.1:14551
```

Receiver:

```bash
python image_stream_receiver_gcs_udp.py \
    --connection udpin:127.0.0.1:14551
```

---

## Drone leaves Pi Wi-Fi range

Expected:

Lost:

- SSH
- VS Code remote
- direct Wi-Fi networking

Still working:

- Mission Planner telemetry
- image packets through MAVLink telemetry radio

because the path is:

```text
Pi
 ↓
Pixhawk
 ↓
Telemetry radio
 ↓
GCS
```