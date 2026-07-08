from __future__ import annotations

import threading
import time

from mavlink_command_common import (
    KIND_ACK,
    KIND_COMMAND,
    KIND_ERROR,
    KIND_HEARTBEAT,
    PARAMETER_FILE,
    build_command_payload,
    encode_protocol_frames,
    load_system_config,
    new_session_id,
    parse_operator_command,
    parse_protocol_frame,
    protocol_severity,
    statustext_to_string,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


class GroundControlSender:
    def __init__(self, config=None, master=None, time_fn=time.monotonic, sleep_fn=time.sleep) -> None:
        self.config = config or load_system_config(PARAMETER_FILE)
        self.master = master
        self.time_fn = time_fn
        self.sleep_fn = sleep_fn
        self.session_id = new_session_id()
        self.send_lock = threading.Lock()
        self.pending_commands: dict[str, tuple[str, float]] = {}
        self.running = True
        self.command_counter = 0

    def connect(self) -> None:
        if self.master is not None:
            return
        if mavutil is None:
            raise RuntimeError("pymavlink is required to run the GCS sender.")

        self.master = mavutil.mavlink_connection(
            self.config.gcs_sender_connection,
            source_system=self.config.sender_source_system,
            source_component=self.config.sender_source_component,
        )

    def run(self) -> None:
        self.connect()
        self._send_heartbeat()

        listener = threading.Thread(target=self._listener_loop, daemon=True)
        heartbeats = threading.Thread(target=self._heartbeat_loop, daemon=True)
        listener.start()
        heartbeats.start()

        print(f"GCS sender session: {self.session_id}")
        print("Enter START src/<script>.py, STOP src/<script>.py, or STOP ALL")
        print("Type EXIT to close the sender.")

        while self.running:
            try:
                raw = input("COMMAND> ").strip()
            except EOFError:
                raw = "EXIT"

            if not raw:
                continue

            if raw.upper() == "EXIT":
                self.running = False
                break

            try:
                command = parse_operator_command(raw)
                if command.script_path is not None:
                    self.config.resolve_script(command.script_path)
                command_id = self._next_command_id()
                payload = build_command_payload(self.session_id, command)
                self._send_protocol(KIND_COMMAND, command_id, payload, allow_chunking=False)
                self.pending_commands[command_id] = (command.canonical_text, self.time_fn())
            except ValueError as exc:
                print(f"[LOCAL ERROR] {exc}")

        listener.join(timeout=1.0)
        heartbeats.join(timeout=1.0)

    def _heartbeat_loop(self) -> None:
        while self.running:
            self._send_heartbeat()
            self._expire_pending_commands()
            self.sleep_fn(self.config.heartbeat_period_s)

    def _listener_loop(self) -> None:
        while self.running:
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
            if frame.kind not in {KIND_ACK, KIND_ERROR}:
                continue

            command_text, _ = self.pending_commands.pop(frame.ref, ("unknown command", 0.0))
            label = "ACK" if frame.kind == KIND_ACK else "ERR"
            print(f"[{label} {frame.ref}] {command_text} -> {frame.payload}")

    def _send_heartbeat(self) -> None:
        self._send_protocol(KIND_HEARTBEAT, self.session_id, "GCS", allow_chunking=False)

    def _send_protocol(self, kind: str, ref: str, payload: str, allow_chunking: bool) -> None:
        if self.master is None:
            return

        frames = encode_protocol_frames(kind, ref, payload, allow_chunking=allow_chunking)
        severity = protocol_severity(kind)
        with self.send_lock:
            for frame_text in frames:
                self.master.mav.statustext_send(severity, frame_text.encode("utf-8"))

    def _expire_pending_commands(self) -> None:
        now = self.time_fn()
        expired = [
            command_id
            for command_id, (_, sent_at) in self.pending_commands.items()
            if now - sent_at >= self.config.ack_timeout_s
        ]
        for command_id in expired:
            command_text, _ = self.pending_commands.pop(command_id)
            print(f"[TIMEOUT {command_id}] {command_text} -> no reply within {self.config.ack_timeout_s:.1f}s")

    def _next_command_id(self) -> str:
        self.command_counter = (self.command_counter % 9999) + 1
        return f"{self.command_counter:04d}"


def main() -> None:
    sender = GroundControlSender()
    sender.run()


if __name__ == "__main__":
    main()
