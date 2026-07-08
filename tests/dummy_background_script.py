import signal
import sys
import time


def handle_shutdown(signum, frame):
    print(f"background dummy received signal {signum}", flush=True)
    sys.exit(0)


signal.signal(signal.SIGTERM, handle_shutdown)

print("background dummy started", flush=True)

while True:
    print("background dummy heartbeat", flush=True)
    time.sleep(0.2)
