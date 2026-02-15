# Raspberry Pi Camera — Usage Notes

This project contains a small Python runner that attempts to use `picamera2` (libcamera) and falls back
to the legacy `picamera` if available.

Quick commands (run from repository root):

```
python src/main.py --capture image.jpg
python src/main.py --record video.h264 --duration 10
python src/main.py --preview
```

Installation notes:

- On Raspberry Pi OS (Bullseye / Bookworm) install system packages:
  - `sudo apt update && sudo apt install -y python3-picamera2 libcamera-apps`
- You may also install Python packages via pip, though system packages are recommended for libcamera:
  - `pip install picamera2` (when available)
  - `pip install picamera` (legacy API, limited to older Pi OS setups)

Enable the camera in `raspi-config` if needed and reboot.

Permissions:
- Running camera code may require root or adding your user to appropriate groups depending on setup.

Notes:
- `picamera2` uses libcamera under the hood and is the recommended modern API.
- The `preview` mode in this example is a simple loop — for GUI previews use `libcamera-vid` or Picamera2 preview helpers.
