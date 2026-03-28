import time
import sys
from PIL import Image
from pyzbar.pyzbar import decode
from picamera2 import Picamera2
import numpy as np

# Optional display of the captured frames. Set to False to disable.
DISPLAY_STREAM = True

# Try to import matplotlib for in-script display; fall back to PIL.Image.show()
try:
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except Exception:
    _HAS_MPL = False


def ensure_rgb(arr: np.ndarray) -> np.ndarray:
    """Always convert a 3-channel array from BGR to RGB by swapping channels.

    This unconditionally flips the color channels for 3-channel images to
    prevent blue-tinted faces when the camera returns BGR ordering.
    """
    try:
        if arr is None:
            return arr
        if arr.ndim == 3 and arr.shape[2] == 3:
            return arr[..., ::-1].copy()
    except Exception:
        pass
    return arr

def scan_stream():
    # Initialize the camera
    picam2 = Picamera2()
    
    # Configure the camera. 
    # specific_sensor_mode allows you to choose frame rate/resolution trade-offs.
    # We use a lower resolution for scanning to speed up processing and reduce memory load.
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)

    # Try to enable autofocus using common libcamera control keys.
    def enable_autofocus(camera):
        attempts = [
            {"AfMode": 2},
            {"AfMode": 1},
            {"af_mode": 2},
            {"AutoFocus": 1},
            {"focus_auto": 1},
            {"FocusMode": 2},
        ]
        for controls in attempts:
            try:
                camera.set_controls(controls)
                print(f"Autofocus enabled with controls: {controls}")
                return True
            except Exception as e:
                # ignore and try next control key
                continue
        print("Autofocus: no supported control found or enabling failed.")
        return False

    try:
        enable_autofocus(picam2)
    except Exception:
        pass

    picam2.start()

    print("Camera started. Scanning for barcodes/QR codes... (Press Ctrl+C to stop)")

    # optional matplotlib image handle (created after first frame)
    mpl_im = None

    try:
        while True:
            # 1. Capture array (returns a numpy array)
            # This keeps it in memory without touching the filesystem
            image_array = picam2.capture_array()
            print(image_array.shape)

            # Detect and correct possible BGR->RGB channel swap (fixes blue faces)
            corrected_array = ensure_rgb(image_array)

            # 2. Convert Numpy array to PIL Image
            pil_image = Image.fromarray(corrected_array)

            # Display the frame if requested. Prefer matplotlib (interactive),
            # otherwise fall back to PIL's show (may open external viewer).
            if DISPLAY_STREAM:
                if _HAS_MPL:
                    if mpl_im is None:
                        plt.ion()
                        fig, ax = plt.subplots()
                        mpl_im = ax.imshow(corrected_array)
                        ax.axis('off')
                        plt.show(block=False)
                    else:
                        mpl_im.set_data(corrected_array)
                    plt.pause(0.001)
                else:
                    try:
                        pil_image.show()
                    except Exception:
                        pass

            

            # 3. Decode using pyzbar (libzbar0 wrapper)
            decoded_objects = decode(pil_image)

            # 4. Output results to terminal
            if decoded_objects:
                for obj in decoded_objects:
                    data = obj.data.decode('utf-8')
                    obj_type = obj.type
                    print(f"[FOUND] Type: {obj_type} | Data: {data}")
            
            # Optional: Add a tiny sleep if CPU usage hits 100%
            # time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping camera...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        picam2.stop()
        picam2.close()

if __name__ == "__main__":
    scan_stream()