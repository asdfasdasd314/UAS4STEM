from __future__ import annotations

import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from mavlink_command_common import (
    KIND_ACK,
    KIND_COMMAND,
    KIND_ERROR,
    KIND_EVENT,
    KIND_HEARTBEAT,
    KIND_LOG,
    PARAMETER_FILE,
    ProtocolFrame,
    ScriptSpec,
    encode_protocol_frames,
    load_system_config,
    parse_protocol_frame,
    protocol_severity,
    statustext_to_string,
    unpack_command_payload,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

REPO_ROOT = Path(__file__).resolve().parents[1]
DAEMON_HEARTBEAT_REF = "PI00"
RTL_COMMAND_ID = 20 if mavutil is None else mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH


@dataclass(slots=True)
class SupervisorEvent:
    event_type: str
    script_path: str
    message: str


@dataclass(slots=True)
class ManagedProcess:
    spec: ScriptSpec
    process: subprocess.Popen[str]
    started_monotonic: float
    stdout_thread: threading.Thread
    stderr_thread: threading.Thread
    last_log_monotonic: float
    exit_reported: bool = False


class CommandSupervisor:
    def __init__(
        self,
        script_specs: dict[str, ScriptSpec],
        stop_timeout_s: float,
        repo_root: Path = REPO_ROOT,
        time_fn=time.monotonic,
    ) -> None:
        self.script_specs = script_specs
        self.stop_timeout_s = stop_timeout_s
        self.repo_root = repo_root
        self.time_fn = time_fn
        self.active: dict[str, ManagedProcess] = {}
        self.event_queue: queue.Queue[SupervisorEvent] = queue.Queue()

    def active_maneuver(self) -> str | None:
        for script_path, managed in self.active.items():
            if managed.spec.category == "maneuver":
                return script_path
        return None

    def start_process(self, spec: ScriptSpec) -> tuple[bool, str]:
        if spec.script_path in self.active:
            return False, f"Script already running: {spec.script_path}"

        if spec.category == "maneuver":
            current_maneuver = self.active_maneuver()
            if current_maneuver is not None:
                return False, f"Maneuver already active: {current_maneuver}"

        script_path = self.repo_root / spec.script_path
        if not script_path.exists():
            return False, f"Script file does not exist: {spec.script_path}"

        kwargs: dict[str, object] = {
            "cwd": str(self.repo_root),
            "stdout": subprocess.PIPE,
            "stderr": subprocess.PIPE,
            "text": True,
            "bufsize": 1,
        }

        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True

        process = subprocess.Popen([sys.executable, str(script_path)], **kwargs)

        stdout_thread = threading.Thread(
            target=self._stream_reader,
            args=(spec, "stdout", process.stdout),
            daemon=True,
        )
        stderr_thread = threading.Thread(
            target=self._stream_reader,
            args=(spec, "stderr", process.stderr),
            daemon=True,
        )
        stdout_thread.start()
        stderr_thread.start()

        managed = ManagedProcess(
            spec=spec,
            process=process,
            started_monotonic=self.time_fn(),
            stdout_thread=stdout_thread,
            stderr_thread=stderr_thread,
            last_log_monotonic=0.0,
        )
        self.active[spec.script_path] = managed
        self.event_queue.put(
            SupervisorEvent(
                event_type="started",
                script_path=spec.script_path,
                message=f"started {spec.script_path} ({spec.category})",
            )
        )
        return True, f"START accepted for {spec.script_path}"

    def stop_process(self, script_path: str) -> tuple[bool, str]:
        managed = self.active.get(script_path)
        if managed is None:
            return False, f"Script is not running: {script_path}"

        self._terminate_process(managed)
        return True, f"STOP accepted for {script_path}"

    def stop_all_processes(self) -> tuple[bool, str]:
        if not self.active:
            return True, "STOP ALL accepted (no active scripts)"

        active_paths = sorted(self.active)
        for script_path in active_paths:
            managed = self.active.get(script_path)
            if managed is not None:
                self._terminate_process(managed)

        return True, f"STOP ALL accepted for {len(active_paths)} script(s)"

    def poll_events(self) -> list[SupervisorEvent]:
        events: list[SupervisorEvent] = []

        while True:
            try:
                event = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event.event_type == "log":
                managed = self.active.get(event.script_path)
                if managed is None:
                    continue
                min_interval = 0.0
                if managed.spec.log_rate_limit_hz > 0:
                    min_interval = 1.0 / managed.spec.log_rate_limit_hz
                now = self.time_fn()
                if now - managed.last_log_monotonic < min_interval:
                    continue
                managed.last_log_monotonic = now

            events.append(event)

        for script_path in list(self.active):
            managed = self.active[script_path]
            return_code = managed.process.poll()
            if return_code is None or managed.exit_reported:
                continue

            managed.exit_reported = True
            events.append(
                SupervisorEvent(
                    event_type="exited",
                    script_path=script_path,
                    message=f"exited {script_path} rc={return_code}",
                )
            )
            self._finalize_process(script_path)

        return events

    def _terminate_process(self, managed: ManagedProcess) -> None:
        script_path = managed.spec.script_path
        process = managed.process

        if process.poll() is not None:
            self._finalize_process(script_path)
            return

        if os.name == "nt":
            process.send_signal(signal.CTRL_BREAK_EVENT)
            try:
                process.wait(timeout=self.stop_timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=self.stop_timeout_s)
        else:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=self.stop_timeout_s)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=self.stop_timeout_s)

        self.event_queue.put(
            SupervisorEvent(
                event_type="stopped",
                script_path=script_path,
                message=f"stopped {script_path} rc={process.returncode}",
            )
        )
        self._finalize_process(script_path)

    def _finalize_process(self, script_path: str) -> None:
        managed = self.active.pop(script_path, None)
        if managed is None:
            return
        for pipe in (managed.process.stdout, managed.process.stderr):
            if pipe is not None:
                try:
                    pipe.close()
                except OSError:
                    pass

    def _stream_reader(self, spec: ScriptSpec, stream_name: str, pipe: object) -> None:
        if pipe is None:
            return

        for raw_line in pipe:
            line = raw_line.strip()
            if not line:
                continue
            self.event_queue.put(
                SupervisorEvent(
                    event_type="log",
                    script_path=spec.script_path,
                    message=f"{spec.script_path} {stream_name}: {line}",
                )
            )


class PiCommandDaemon:
    def __init__(
        self,
        config=None,
        master=None,
        supervisor: CommandSupervisor | None = None,
        time_fn=time.monotonic,
        sleep_fn=time.sleep,
    ) -> None:
        self.config = config or load_system_config(PARAMETER_FILE)
        self.time_fn = time_fn
        self.sleep_fn = sleep_fn
        self.master = master
        self.supervisor = supervisor or CommandSupervisor(
            script_specs=self.config.script_specs,
            stop_timeout_s=self.config.process_stop_timeout_s,
        )
        self.last_sender_heartbeat_monotonic: float | None = None
        self.current_sender_session: str | None = None
        self.failsafe_active = False
        self._last_daemon_heartbeat_monotonic = 0.0
        self._last_target_seen_monotonic = 0.0

    def connect(self) -> None:
        if self.master is not None:
            return
        if mavutil is None:
            raise RuntimeError("pymavlink is required to run the MAVLink command daemon.")

        self.master = mavutil.mavlink_connection(
            self.config.pixhawk_connection,
            source_system=self.config.daemon_source_system,
            source_component=self.config.daemon_source_component,
        )

    def run_forever(self) -> None:
        self.connect()
        self._emit_event("daemon online")

        while True:
            self._drain_supervisor_events()
            self._emit_daemon_heartbeat_if_due()
            self._poll_mavlink_once()
            self.maybe_trigger_link_loss_failsafe()
            self.sleep_fn(0.05)

    def maybe_trigger_link_loss_failsafe(self) -> bool:
        if self.last_sender_heartbeat_monotonic is None:
            return False
        if self.failsafe_active:
            return False

        elapsed = self.time_fn() - self.last_sender_heartbeat_monotonic
        if elapsed < self.config.link_loss_timeout_s:
            return False

        self.failsafe_active = True
        self._emit_event("link_loss_timeout")
        self._trigger_rtl()
        self._emit_event("rtl_triggered")
        self.supervisor.stop_all_processes()
        return True

    def handle_protocol_frame(self, frame: ProtocolFrame, source_system: int | None = None) -> None:
        if source_system is not None and source_system != self.config.sender_source_system:
            return

        if frame.kind == KIND_HEARTBEAT:
            self._handle_sender_heartbeat(frame.ref)
            return

        if frame.kind != KIND_COMMAND:
            return

        command_id = frame.ref
        try:
            session_id, command = unpack_command_payload(frame.payload)
        except ValueError as exc:
            self._emit_error(command_id, str(exc))
            return

        self._handle_sender_heartbeat(session_id)

        if self.failsafe_active and session_id != self.current_sender_session:
            self.failsafe_active = False
            self._emit_event(f"controller_reacquired {session_id}")

        try:
            if command.action == "STOP_ALL":
                ok, detail = self.supervisor.stop_all_processes()
            elif command.action == "START":
                spec = self.config.resolve_script(command.script_path or "")
                ok, detail = self.supervisor.start_process(spec)
            elif command.action == "STOP":
                self.config.resolve_script(command.script_path or "")
                ok, detail = self.supervisor.stop_process(command.script_path or "")
            else:
                ok, detail = False, f"Unsupported command action: {command.action}"
        except ValueError as exc:
            ok, detail = False, str(exc)

        if ok:
            self._emit_ack(command_id, detail)
        else:
            self._emit_error(command_id, detail)

    def _handle_sender_heartbeat(self, session_id: str) -> None:
        self.last_sender_heartbeat_monotonic = self.time_fn()
        if session_id != self.current_sender_session:
            previous_session = self.current_sender_session
            self.current_sender_session = session_id
            if previous_session is not None and self.failsafe_active:
                self.failsafe_active = False
                self._emit_event(f"controller_reacquired {session_id}")

    def _poll_mavlink_once(self) -> None:
        if self.master is None:
            return

        message = self.master.recv_match(blocking=False)
        if message is None:
            return

        if getattr(message, "get_type", lambda: None)() == "HEARTBEAT":
            source_system = getattr(message, "get_srcSystem", lambda: 0)()
            if source_system not in {
                self.config.sender_source_system,
                self.config.receiver_source_system,
                self.config.daemon_source_system,
            }:
                if getattr(self.master, "target_system", 0) == 0:
                    self.master.target_system = source_system or 1
                self._last_target_seen_monotonic = self.time_fn()
            return

        if getattr(message, "get_type", lambda: None)() != "STATUSTEXT":
            return

        frame = parse_protocol_frame(statustext_to_string(message))
        if frame is None:
            return

        source_system = getattr(message, "get_srcSystem", lambda: 0)()
        self.handle_protocol_frame(frame, source_system=source_system)

    def _emit_ack(self, command_id: str, payload: str) -> None:
        self._send_protocol(KIND_ACK, command_id, payload)

    def _emit_error(self, command_id: str, payload: str) -> None:
        self._send_protocol(KIND_ERROR, command_id, payload, allow_chunking=True)

    def _emit_event(self, payload: str) -> None:
        self._send_protocol(KIND_EVENT, "0000", payload, allow_chunking=True)

    def _emit_log(self, payload: str) -> None:
        self._send_protocol(KIND_LOG, "0000", payload, allow_chunking=True)

    def _emit_daemon_heartbeat_if_due(self) -> None:
        now = self.time_fn()
        if now - self._last_daemon_heartbeat_monotonic < self.config.heartbeat_period_s:
            return
        self._last_daemon_heartbeat_monotonic = now
        self._send_protocol(KIND_HEARTBEAT, DAEMON_HEARTBEAT_REF, "PI")

    def _send_protocol(self, kind: str, ref: str, payload: str, allow_chunking: bool = False) -> None:
        if self.master is None:
            return

        frames = encode_protocol_frames(kind, ref, payload, allow_chunking=allow_chunking)
        severity = protocol_severity(kind)
        for frame_text in frames:
            self.master.mav.statustext_send(severity, frame_text.encode("utf-8"))

    def _drain_supervisor_events(self) -> None:
        for event in self.supervisor.poll_events():
            if event.event_type == "log":
                self._emit_log(event.message)
            else:
                self._emit_event(event.message)

    def _trigger_rtl(self) -> None:
        if self.master is None:
            return

        target_system = getattr(self.master, "target_system", 0) or 1
        target_component = getattr(self.master, "target_component", 0) or 1
        self.master.mav.command_long_send(
            target_system,
            target_component,
            RTL_COMMAND_ID,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def main() -> None:
    daemon = PiCommandDaemon()
    daemon.run_forever()


if __name__ == "__main__":
    main()
