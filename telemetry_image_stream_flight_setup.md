# Telemetry Radio Image Stream Flight Setup

Goal: route Raspberry Pi camera snapshots through the Pixhawk telemetry path instead of the Pi WiFi hotspot.

Target path:

```text
Pi camera
  ↓
image_stream_sender.py on Raspberry Pi
  ↓ serial/USB MAVLink
Pixhawk
  ↓ air telemetry radio
Ground telemetry radio
  ↓ Mission Planner on Windows GCS
UDP forwarding
  ↓
image_stream_receiver.py on Windows GCS
```

This setup minimizes changes on the Raspberry Pi. Most configuration and testing should happen on the GCS laptop.

---

## 0. Assumptions

- The drone already has an air telemetry radio connected to the Pixhawk.
- The GCS laptop already has the matching ground telemetry radio.
- Mission Planner can already connect to the Pixhawk through the ground telemetry radio.
- `image_stream_sender.py` runs on the Raspberry Pi.
- `image_stream_receiver.py` runs on the Windows GCS laptop.
- The Pi must send MAVLink into the Pixhawk through either:
  - USB cable from Pi to Pixhawk, or
  - Pi GPIO UART to a Pixhawk TELEM port.

If the Pi is only connected to the laptop over WiFi and has no connection to the Pixhawk, the telemetry radio will not carry the images.

---

## 1. One-time code status check: baud support is already present

Machine: development computer or GCS laptop

`src/image_stream_sender.py` already includes:

- `--baud`
- serial-device warnings
- heartbeat timeout handling
- telemetry-radio guidance in the failure output

No additional code change is required before copying the sender to the Pi.

Helper launchers are also included now:

- Pi sender: `scripts/run_image_sender_pi.sh`
- Pi heartbeat test: `scripts/check_pixhawk_heartbeat_pi.sh`
- Windows receiver PowerShell: `scripts/run_image_receiver_windows.ps1`
- Windows receiver batch file: `scripts/run_image_receiver_windows.bat`
- Windows MAVProxy forwarder: `scripts/run_mavproxy_forwarder_windows.ps1`

---

## 2. Copy the updated sender and helper scripts to the Pi / Windows laptop

Machine: GCS laptop or development computer

Replace these placeholders:

```text
PI_USER = your Pi username, often pi
PI_HOST = Pi hostname or IP address
PROJECT_DIR = directory on the Pi containing the script
```

Example copying the sender and Pi helper scripts to the Pi:

```bash
scp src/image_stream_sender.py PI_USER@PI_HOST:PROJECT_DIR/src/image_stream_sender.py
scp scripts/run_image_sender_pi.sh PI_USER@PI_HOST:PROJECT_DIR/scripts/run_image_sender_pi.sh
scp scripts/check_pixhawk_heartbeat_pi.sh PI_USER@PI_HOST:PROJECT_DIR/scripts/check_pixhawk_heartbeat_pi.sh
```

Example:

```bash
scp src/image_stream_sender.py pi@raspberrypi.local:~/uas4stem/src/image_stream_sender.py
scp scripts/run_image_sender_pi.sh pi@raspberrypi.local:~/uas4stem/scripts/run_image_sender_pi.sh
scp scripts/check_pixhawk_heartbeat_pi.sh pi@raspberrypi.local:~/uas4stem/scripts/check_pixhawk_heartbeat_pi.sh
```

Copy these files to the Windows GCS laptop as part of the project checkout:

- `src/image_stream_receiver.py`
- `scripts/run_image_receiver_windows.ps1`
- `scripts/run_image_receiver_windows.bat`
- `scripts/run_mavproxy_forwarder_windows.ps1`

---

## 3. Confirm Mission Planner already works through telemetry

Machine: Windows GCS laptop

1. Plug the ground telemetry radio into the GCS laptop.
2. Open Mission Planner.
3. Select the telemetry radio COM port.
4. Select the correct baud rate.

Common baud rates:

```text
57600
115200
```

5. Click `Connect`.
6. Confirm Mission Planner shows live Pixhawk telemetry:
   - attitude
   - GPS if available
   - battery/status messages
   - mode
   - altitude

Do not proceed until this works. If Mission Planner cannot talk to the Pixhawk through the telemetry radios, the image stream cannot use that path either.

---

## 4. Identify how the Pi connects to the Pixhawk

Machine: Raspberry Pi

SSH into the Pi:

```bash
ssh PI_USER@PI_HOST
```

List serial devices:

```bash
ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

### If the Pi connects to the Pixhawk by USB

You may see a device under:

```text
/dev/serial/by-id/
```

Example:

```text
/dev/serial/by-id/usb-ArduPilot_Pixhawk1_XXXXXXXX-if00
```

Use that path for `--connection`.

### If the Pi connects to the Pixhawk through GPIO UART

The likely path is:

```text
/dev/serial0
```

Use `/dev/serial0` for `--connection`.

---

## 5. Configure the Pixhawk port for the Pi connection

Machine: Windows GCS laptop, in Mission Planner

Only do this if the Pi is connected to a Pixhawk TELEM port or serial port.

In Mission Planner:

```text
CONFIG
→ Full Parameter Tree
```

Find the SERIAL port corresponding to the Pixhawk port used by the Pi.

Common mapping:

```text
TELEM1 → SERIAL1
TELEM2 → SERIAL2
GPS2 / other ports vary by board
```

For example, if the Pi is connected to TELEM2:

```text
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD = 57
```

Meaning:

```text
Protocol 2 = MAVLink2
Baud 57 = 57600
```

Alternative baud:

```text
SERIAL2_BAUD = 115
```

Meaning:

```text
115200
```

After changing parameters:

1. Write params.
2. Reboot Pixhawk.
3. Reconnect Mission Planner.

---

## 6. Test Pi → Pixhawk heartbeat

Machine: Raspberry Pi

Test with the same device path and baud you plan to use.

### USB example

```bash
./scripts/check_pixhawk_heartbeat_pi.sh /dev/serial/by-id/REPLACE_WITH_YOUR_PIXHAWK_DEVICE 115200
```

### GPIO UART example

```bash
./scripts/check_pixhawk_heartbeat_pi.sh /dev/serial0 57600
```

If this stalls, the issue is not the image code. Check:

- wrong serial device
- wrong baud rate
- Pixhawk SERIALx port not configured for MAVLink
- Pi UART disabled
- Pi serial console still using the UART
- TX/RX wiring reversed or missing ground
- Pixhawk not powered

---

## 7. Run the sender through the Pixhawk

Machine: Raspberry Pi

Use conservative image settings first.

### GPIO UART example

```bash
cd PROJECT_DIR
./scripts/run_image_sender_pi.sh --debug
```

### USB example

```bash
cd PROJECT_DIR
CONNECTION=/dev/serial/by-id/REPLACE_WITH_YOUR_PIXHAWK_DEVICE BAUD=115200 ./scripts/run_image_sender_pi.sh --debug
```

Expected output:

```text
Connecting on ...
Waiting for heartbeat...
Heartbeat received
Image source: Raspberry Pi Camera
Queued camera (... bytes, ... packets)
```

---

## 8. Forward MAVLink from Mission Planner to the receiver

Machine: Windows GCS laptop

The telemetry radio COM port can usually only be opened by one program at a time. Mission Planner already owns it. Therefore, the receiver should not open the COM port directly. Instead, Mission Planner should forward MAVLink to UDP.

In Mission Planner:

```text
Ctrl+F
→ MAVLink
→ UDP Host
```

Set UDP output to:

```text
127.0.0.1
14551
```

Use `14551` for the receiver so it does not collide with Mission Planner's usual `14550` conventions.

---

## 9. Run the receiver on the GCS laptop

Machine: Windows GCS laptop

In PowerShell or Command Prompt:

```powershell
cd PATH\TO\PROJECT
powershell -ExecutionPolicy Bypass -File .\scripts\run_image_receiver_windows.ps1 --debug
```

Expected output:

```text
Listening for image stream on udpin:0.0.0.0:14551...
Received MAVLink message: DATA_TRANSMISSION_HANDSHAKE
Received MAVLink message: ENCAPSULATED_DATA
Reassembled frame #1 (... bytes)
Displayed frame #1 (...)
```

If you see normal MAVLink messages but no image messages, the receiver is getting telemetry but the Pi image packets are not reaching the Pixhawk telemetry stream.

---

## 10. Optional: use MAVProxy instead of Mission Planner forwarding

Machine: Windows GCS laptop

This is useful if Mission Planner UDP forwarding is unreliable or confusing.

Install MAVProxy:

```powershell
pip install MAVProxy
```

Run MAVProxy with the ground telemetry COM port:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_mavproxy_forwarder_windows.ps1 -ComPort COM5 -BaudRate 57600
```

Then connect:

- Mission Planner to UDP `127.0.0.1:14550`
- receiver script to UDP `14551`

Receiver:

```powershell
python src\image_stream_receiver.py --connection udpin:0.0.0.0:14551 --debug
```

Use the actual COM port shown in Windows Device Manager.

---

## 11. Flight-line startup checklist

### On the drone

1. Power Pixhawk.
2. Power Raspberry Pi.
3. Power telemetry radio.
4. Wait for Pi boot.
5. Start sender on the Pi.

Example:

```bash
cd PROJECT_DIR
./scripts/run_image_sender_pi.sh
```

Use `tmux` if you want the sender to survive SSH disconnects:

```bash
tmux new -s image_stream
cd PROJECT_DIR
./scripts/run_image_sender_pi.sh
```

Detach from tmux:

```text
Ctrl+B, then D
```

Reconnect later:

```bash
tmux attach -t image_stream
```

### On the GCS laptop

1. Plug in ground telemetry radio.
2. Open Mission Planner.
3. Connect to the telemetry radio COM port.
4. Confirm live telemetry.
5. Start Mission Planner UDP forwarding to `127.0.0.1:14551`.
6. Start receiver:

```powershell
cd PATH\TO\PROJECT
powershell -ExecutionPolicy Bypass -File .\scripts\run_image_receiver_windows.ps1
```

---

## 12. Parameter tuning at the flying field

Start conservative:

```bash
--width 320 --height 180 --quality 30 --interval 5 --chunks-per-loop 2
```

If telemetry remains responsive, try:

```bash
--width 480 --height 270 --quality 35 --interval 5 --chunks-per-loop 2
```

If Mission Planner becomes laggy, reduce bandwidth:

```bash
--width 240 --height 135 --quality 25 --interval 8 --chunks-per-loop 1
```

Use the stream for low-rate snapshots, not live video.

---

## 13. Common failure cases

### Sender stalls at `Waiting for heartbeat...`

Cause: the Pi is not receiving Pixhawk MAVLink on that connection.

Check:

```bash
ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

Try a different baud:

```bash
--baud 57600
--baud 115200
--baud 921600
```

Check Mission Planner parameters:

```text
SERIALx_PROTOCOL = 2
SERIALx_BAUD = 57 or 115
```

### Receiver says no MAVLink traffic

Cause: Mission Planner is not forwarding to the receiver port, or the receiver is listening on the wrong port.

Check:

```powershell
python src\image_stream_receiver.py --connection udpin:0.0.0.0:14551 --debug
```

Make sure Mission Planner forwards to:

```text
127.0.0.1:14551
```

### Mission Planner works, but no images arrive

Cause: the telemetry link works, but Pi image packets are not entering the Pixhawk MAVLink stream.

Check:

- sender got Pixhawk heartbeat
- correct Pixhawk serial port configured
- Pi connected to Pixhawk, not only to laptop WiFi
- image packets are being queued by sender

### WiFi disconnects and stream dies

Cause: the image stream is still using WiFi, not telemetry.

Correct setup should keep working without the Pi hotspot after startup, assuming the sender was already started locally on the Pi.

---

## 14. Minimal final commands

### Pi

```bash
cd PROJECT_DIR
./scripts/run_image_sender_pi.sh
```

### GCS laptop

Mission Planner:

```text
Connect to telemetry radio COM port
Forward MAVLink UDP to 127.0.0.1:14551
```

Receiver:

```powershell
cd PATH\TO\PROJECT
powershell -ExecutionPolicy Bypass -File .\scripts\run_image_receiver_windows.ps1
```
