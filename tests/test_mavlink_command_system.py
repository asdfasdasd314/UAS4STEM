from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from mavlink_command_common import (  # noqa: E402
    KIND_ACK,
    KIND_COMMAND,
    KIND_EVENT,
    KIND_HEARTBEAT,
    KIND_LOG,
    ScriptSpec,
    build_command_payload,
    encode_protocol_frames,
    load_system_config,
    new_session_id,
    parse_operator_command,
    parse_protocol_frame,
)
from mavlink_command_daemon import CommandSupervisor, PiCommandDaemon  # noqa: E402

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


class FakeMav:
    def __init__(self) -> None:
        self.statustext_calls: list[tuple[int, str]] = []
        self.command_long_calls: list[tuple[object, ...]] = []

    def statustext_send(self, severity, payload):
        text = payload.decode("utf-8") if isinstance(payload, bytes) else str(payload)
        self.statustext_calls.append((severity, text))

    def command_long_send(self, *args):
        self.command_long_calls.append(args)


class FakeMaster:
    def __init__(self) -> None:
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1

    def recv_match(self, blocking=False, timeout=None):
        return None


class ConfigAndParsingTests(unittest.TestCase):
    def test_parse_operator_command_normalizes_paths(self):
        command = parse_operator_command(" start   ./src/mission4.py ")
        self.assertEqual(command.action, "START")
        self.assertEqual(command.script_path, "src/mission4.py")
        self.assertEqual(command.canonical_text, "START src/mission4.py")

    def test_parse_operator_command_supports_stop_all(self):
        command = parse_operator_command("STOP ALL")
        self.assertEqual(command.action, "STOP_ALL")
        self.assertIsNone(command.script_path)

    def test_parse_operator_command_rejects_non_src_paths(self):
        with self.assertRaises(ValueError):
            parse_operator_command("START scripts/run_image_sender_pi.sh")

    def test_load_system_config_enforces_whitelist(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            parameter_file = Path(tmpdir) / "test.toml"
            parameter_file.write_text(
                "\n".join(
                    [
                        'pixhawk_connection = "udpout:127.0.0.1:14550"',
                        'gcs_sender_connection = "udpin:0.0.0.0:14600"',
                        'gcs_receiver_connection = "udpin:0.0.0.0:14601"',
                        "link_loss_timeout_s = 3.0",
                        "heartbeat_period_s = 1.0",
                        "ack_timeout_s = 2.0",
                        "process_stop_timeout_s = 1.0",
                        "daemon_source_system = 200",
                        "daemon_source_component = 191",
                        "sender_source_system = 201",
                        "sender_source_component = 191",
                        "receiver_source_system = 202",
                        "receiver_source_component = 191",
                        'maneuver_scripts = ["src/mission4.py"]',
                        'background_scripts = ["src/image_stream_sender.py"]',
                    ]
                ),
                encoding="utf-8",
            )
            config = load_system_config(parameter_file)

        self.assertEqual(config.resolve_script("src/mission4.py").category, "maneuver")
        self.assertEqual(config.resolve_script("src/image_stream_sender.py").category, "background")
        with self.assertRaises(ValueError):
            config.resolve_script("src/not_registered.py")


class ProtocolFrameTests(unittest.TestCase):
    def test_encode_and_parse_single_frame(self):
        frames = encode_protocol_frames(KIND_ACK, "0001", "START accepted", allow_chunking=False)
        self.assertEqual(len(frames), 1)
        parsed = parse_protocol_frame(frames[0])
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.kind, KIND_ACK)
        self.assertEqual(parsed.ref, "0001")
        self.assertEqual(parsed.payload, "START accepted")

    def test_chunked_log_frames_stay_within_statustext_limit(self):
        payload = "x" * 120
        frames = encode_protocol_frames(KIND_LOG, "0000", payload, allow_chunking=True)
        self.assertGreater(len(frames), 1)
        self.assertTrue(all(len(frame) <= 50 for frame in frames))

    def test_command_payload_contains_session_id(self):
        session_id = new_session_id()
        command = parse_operator_command("START src/mission4.py")
        payload = build_command_payload(session_id, command)
        self.assertTrue(payload.startswith(session_id))


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        dummy_spec = ScriptSpec(
            script_path="tests/dummy_managed_script.py",
            category="maneuver",
            friendly_name="Dummy Maneuver",
            restart_policy="never",
            log_rate_limit_hz=20.0,
        )
        background_spec = ScriptSpec(
            script_path="tests/dummy_background_script.py",
            category="background",
            friendly_name="Dummy Background",
            restart_policy="never",
            log_rate_limit_hz=20.0,
        )
        self.specs = {
            "tests/dummy_managed_script.py": dummy_spec,
            "tests/dummy_background_script.py": background_spec,
        }

    def test_one_maneuver_plus_background_rule(self):
        supervisor = CommandSupervisor(script_specs=self.specs, stop_timeout_s=1.0, repo_root=REPO_ROOT)
        maneuver_spec = self.specs["tests/dummy_managed_script.py"]
        background_spec = self.specs["tests/dummy_background_script.py"]

        ok, _ = supervisor.start_process(maneuver_spec)
        self.assertTrue(ok)

        second_ok, second_message = supervisor.start_process(maneuver_spec)
        self.assertFalse(second_ok)
        self.assertIn("already running", second_message)

        background_ok, _ = supervisor.start_process(background_spec)
        self.assertTrue(background_ok)

        supervisor.stop_all_processes()
        time.sleep(0.2)
        supervisor.poll_events()

    def test_process_lifecycle_emits_started_and_stopped_events(self):
        spec = self.specs["tests/dummy_managed_script.py"]
        supervisor = CommandSupervisor(script_specs=self.specs, stop_timeout_s=1.0, repo_root=REPO_ROOT)

        ok, _ = supervisor.start_process(spec)
        self.assertTrue(ok)

        time.sleep(0.4)
        start_and_log_events = supervisor.poll_events()
        event_types = {event.event_type for event in start_and_log_events}
        self.assertIn("started", event_types)
        self.assertIn("log", event_types)

        stop_ok, _ = supervisor.stop_process(spec.script_path)
        self.assertTrue(stop_ok)
        time.sleep(0.2)
        stop_events = supervisor.poll_events()
        stop_types = {event.event_type for event in stop_events}
        self.assertIn("stopped", stop_types)


class DaemonBehaviorTests(unittest.TestCase):
    def _make_config(self):
        return load_system_config(REPO_ROOT / "parameter_files" / "mavlink_command_system.toml")

    def test_daemon_acknowledges_and_rejects_commands(self):
        config = self._make_config()
        master = FakeMaster()
        supervisor = CommandSupervisor(config.script_specs, stop_timeout_s=1.0, repo_root=REPO_ROOT)
        daemon = PiCommandDaemon(config=config, master=master, supervisor=supervisor)

        session_id = "AB12"
        invalid_command = parse_operator_command("START src/not_registered.py")
        invalid_frame = parse_protocol_frame(
            encode_protocol_frames(
                KIND_COMMAND,
                "0001",
                build_command_payload(session_id, invalid_command),
                allow_chunking=False,
            )[0]
        )

        daemon.handle_protocol_frame(invalid_frame, source_system=config.sender_source_system)
        self.assertTrue(any("|E|0001|" in call[1] for call in master.mav.statustext_calls))

        stop_all_command = parse_operator_command("STOP ALL")
        stop_all_frame = parse_protocol_frame(
            encode_protocol_frames(
                KIND_COMMAND,
                "0002",
                build_command_payload(session_id, stop_all_command),
                allow_chunking=False,
            )[0]
        )
        daemon.handle_protocol_frame(stop_all_frame, source_system=config.sender_source_system)
        self.assertTrue(any("|A|0002|" in call[1] for call in master.mav.statustext_calls))

    def test_link_loss_triggers_rtl_and_stop_all(self):
        config = self._make_config()
        config.link_loss_timeout_s = 0.2
        master = FakeMaster()

        spec = ScriptSpec(
            script_path="tests/dummy_managed_script.py",
            category="background",
            friendly_name="Dummy",
            restart_policy="never",
            log_rate_limit_hz=20.0,
        )
        supervisor = CommandSupervisor(
            script_specs={"tests/dummy_managed_script.py": spec},
            stop_timeout_s=1.0,
            repo_root=REPO_ROOT,
        )
        supervisor.start_process(spec)

        daemon = PiCommandDaemon(config=config, master=master, supervisor=supervisor)
        daemon.handle_protocol_frame(
            parse_protocol_frame(encode_protocol_frames(KIND_HEARTBEAT, "AB12", "GCS")[0]),
            source_system=config.sender_source_system,
        )

        time.sleep(0.3)
        triggered = daemon.maybe_trigger_link_loss_failsafe()
        self.assertTrue(triggered)
        self.assertTrue(master.mav.command_long_calls)
        self.assertTrue(any("|V|0000|rtl_triggered" in call[1] for call in master.mav.statustext_calls))

        time.sleep(0.2)
        supervisor.poll_events()


@unittest.skipIf(mavutil is None, "pymavlink is not installed")
class LoopbackIntegrationTests(unittest.TestCase):
    def test_statustext_round_trip_over_loopback_udp(self):
        listener = None
        speaker = None
        try:
            listener = mavutil.mavlink_connection("udpin:127.0.0.1:14990", source_system=42)
            speaker = mavutil.mavlink_connection("udpout:127.0.0.1:14990", source_system=43)
        except PermissionError as exc:
            if listener is not None:
                try:
                    listener.close()
                except Exception:
                    pass
            if speaker is not None:
                try:
                    speaker.close()
                except Exception:
                    pass
            self.skipTest(f"Loopback UDP is not permitted in this environment: {exc}")

        try:
            frame_text = encode_protocol_frames(KIND_EVENT, "0000", "loopback_ok")[0]
            speaker.mav.statustext_send(6, frame_text.encode("utf-8"))

            deadline = time.time() + 2.0
            received = None
            while time.time() < deadline and received is None:
                received = listener.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)

            self.assertIsNotNone(received)
            parsed = parse_protocol_frame(received.text)
            self.assertIsNotNone(parsed)
            self.assertEqual(parsed.payload, "loopback_ok")
        finally:
            try:
                listener.close()
            except Exception:
                pass
            try:
                speaker.close()
            except Exception:
                pass


if __name__ == "__main__":
    unittest.main()
