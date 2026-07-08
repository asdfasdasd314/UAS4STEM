import signal
import sys
import time


def handle_shutdown(signum, frame):
    print(f"dummy received signal {signum}", flush=True)
    sys.exit(0)


signal.signal(signal.SIGTERM, handle_shutdown)

print("dummy started", flush=True)

while True:
    print("dummy heartbeat", flush=True)
    time.sleep(0.2)
