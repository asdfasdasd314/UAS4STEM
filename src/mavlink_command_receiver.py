from __future__ import annotations

import time
from datetime import datetime

from mavlink_command_common import (
    KIND_ACK,
    KIND_ERROR,
    KIND_EVENT,
    KIND_HEARTBEAT,
    KIND_LOG,
    PARAMETER_FILE,
    RECEIVER_LOG_DIR,
    load_system_config,
    parse_protocol_frame,
    statustext_to_string,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

KIND_LABELS = {
    KIND_ACK: "ACK",
    KIND_ERROR: "ERR",
    KIND_EVENT: "EVT",
    KIND_LOG: "LOG",
    KIND_HEARTBEAT: "HB",
}


class GroundControlReceiver:
    def __init__(self, config=None, master=None) -> None:
        self.config = config or load_system_config(PARAMETER_FILE)
        self.master = master

    def connect(self) -> None:
        if self.master is not None:
            return
        if mavutil is None:
            raise RuntimeError("pymavlink is required to run the GCS receiver.")

        self.master = mavutil.mavlink_connection(
            self.config.gcs_receiver_connection,
            source_system=self.config.receiver_source_system,
            source_component=self.config.receiver_source_component,
        )

    def run(self) -> None:
        self.connect()
        RECEIVER_LOG_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = RECEIVER_LOG_DIR / f"session_{timestamp}.log"

        print(f"Writing receiver log to {log_path}")

        with log_path.open("a", encoding="utf-8") as handle:
            while True:
                if self.master is None:
                    break
                message = self.master.recv_match(blocking=True, timeout=1.0)
                if message is None:
                    continue
                if getattr(message, "get_type", lambda: None)() != "STATUSTEXT":
                    continue

                frame = parse_protocol_frame(statustext_to_string(message))
                if frame is None:
                    continue

                rendered = self._render_frame(frame)
                stamped = f"{datetime.now().isoformat(timespec='seconds')} {rendered}"
                print(stamped)
                handle.write(stamped + "\n")
                handle.flush()

    def _render_frame(self, frame) -> str:
        label = KIND_LABELS.get(frame.kind, frame.kind)
        if frame.kind == KIND_HEARTBEAT:
            return f"[{label}] ref={frame.ref} payload={frame.payload}"
        if frame.kind in {KIND_ACK, KIND_ERROR}:
            return f"[{label}] cmd={frame.ref} {frame.payload}"
        return f"[{label}] {frame.payload}"


def main() -> None:
    receiver = GroundControlReceiver()
    receiver.run()


if __name__ == "__main__":
    main()
