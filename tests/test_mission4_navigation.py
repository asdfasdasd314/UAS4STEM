from __future__ import annotations

import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import maneuvers  # noqa: E402
import mission4  # noqa: E402
from pymavlink import mavutil  # noqa: E402


def write_mission4_parameter_file(
    path: Path,
    *,
    max_ground_speed_mps: float = 0.5,
    wpnav_accel_mps2: float = 0.5,
    max_waypoint_distance_m: float = 150.0,
) -> None:
    path.write_text(
        "\n".join(
            [
                'connection_string = "udpout:127.0.0.1:14550"',
                "box_altitude_ft = 10.0",
                f"max_ground_speed_mps = {max_ground_speed_mps}",
                f"wpnav_accel_mps2 = {wpnav_accel_mps2}",
                f"max_waypoint_distance_m = {max_waypoint_distance_m}",
                "",
                "[[box_waypoints]]",
                'label = "A"',
                "lat = 42.0",
                "lon = -83.0",
                "",
                "[[box_waypoints]]",
                'label = "B"',
                "lat = 42.1",
                "lon = -83.1",
                "",
                "[[box_waypoints]]",
                'label = "C"',
                "lat = 42.2",
                "lon = -83.2",
                "",
                "[[box_waypoints]]",
                'label = "D"',
                "lat = 42.3",
                "lon = -83.3",
            ]
        ),
        encoding="utf-8",
    )


class FakeMessage:
    def __init__(self, message_type: str, **kwargs) -> None:
        self.message_type = message_type
        for key, value in kwargs.items():
            setattr(self, key, value)


class FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []
        self.position_target_calls: list[tuple[object, ...]] = []
        self.param_set_calls: list[tuple[object, ...]] = []
        self.param_request_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args):
        self.command_long_calls.append(args)

    def set_position_target_global_int_send(self, *args):
        self.position_target_calls.append(args)

    def param_set_send(self, *args):
        self.param_set_calls.append(args)

    def param_request_read_send(self, *args):
        self.param_request_calls.append(args)


class FakeMaster:
    def __init__(self, messages: list[FakeMessage] | None = None) -> None:
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1
        self.messages = list(messages or [])

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type is None:
            if self.messages:
                return self.messages.pop(0)
            return None

        if isinstance(type, str):
            wanted_types = {type}
        else:
            wanted_types = set(type)

        for index, message in enumerate(self.messages):
            if message.message_type in wanted_types:
                return self.messages.pop(index)
        return None


class Mission4ConfigTests(unittest.TestCase):
    def test_load_mission4_config_reads_safety_tunables(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            parameter_file = Path(tmpdir) / "mission4.toml"
            write_mission4_parameter_file(
                parameter_file,
                max_ground_speed_mps=0.5,
                wpnav_accel_mps2=0.7,
                max_waypoint_distance_m=200.0,
            )

            original_parameter_file = mission4.PARAMETER_FILE
            mission4.PARAMETER_FILE = parameter_file
            try:
                config = mission4.load_mission4_config()
            finally:
                mission4.PARAMETER_FILE = original_parameter_file

        self.assertEqual(config["connection_string"], "udpout:127.0.0.1:14550")
        self.assertAlmostEqual(config["box_altitude_m"], 3.048)
        self.assertEqual(config["max_ground_speed_mps"], 0.5)
        self.assertEqual(config["wpnav_accel_mps2"], 0.7)
        self.assertEqual(config["max_waypoint_distance_m"], 200.0)
        self.assertEqual(len(config["box_waypoints"]), 4)

    def test_build_mission_attaches_ground_speed_to_each_navigation_step(self):
        config = {
            "box_altitude_ft": 25.0,
            "box_altitude_m": 7.62,
            "max_ground_speed_mps": 0.5,
            "box_waypoints": [
                {"label": "A", "lat": 42.0, "lon": -83.0},
                {"label": "B", "lat": 42.1, "lon": -83.1},
                {"label": "C", "lat": 42.2, "lon": -83.2},
                {"label": "D", "lat": 42.3, "lon": -83.3},
            ],
        }

        mission = mission4.build_mission(config)

        self.assertTrue(all(item.params.get("ground_speed_mps") == 0.5 for item in mission))

    def test_load_mission4_config_rejects_non_positive_speed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            parameter_file = Path(tmpdir) / "mission4.toml"
            write_mission4_parameter_file(parameter_file, max_ground_speed_mps=0.0)

            original_parameter_file = mission4.PARAMETER_FILE
            mission4.PARAMETER_FILE = parameter_file
            try:
                with self.assertRaisesRegex(ValueError, "max_ground_speed_mps must be greater than zero"):
                    mission4.load_mission4_config()
            finally:
                mission4.PARAMETER_FILE = original_parameter_file

    def test_load_mission4_config_rejects_non_positive_wpnav_accel(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            parameter_file = Path(tmpdir) / "mission4.toml"
            write_mission4_parameter_file(parameter_file, wpnav_accel_mps2=0.0)

            original_parameter_file = mission4.PARAMETER_FILE
            mission4.PARAMETER_FILE = parameter_file
            try:
                with self.assertRaisesRegex(ValueError, "wpnav_accel_mps2 must be greater than zero"):
                    mission4.load_mission4_config()
            finally:
                mission4.PARAMETER_FILE = original_parameter_file

    def test_load_mission4_config_rejects_non_positive_waypoint_distance_limit(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            parameter_file = Path(tmpdir) / "mission4.toml"
            write_mission4_parameter_file(parameter_file, max_waypoint_distance_m=0.0)

            original_parameter_file = mission4.PARAMETER_FILE
            mission4.PARAMETER_FILE = parameter_file
            try:
                with self.assertRaisesRegex(ValueError, "max_waypoint_distance_m must be greater than zero"):
                    mission4.load_mission4_config()
            finally:
                mission4.PARAMETER_FILE = original_parameter_file

    def test_validate_waypoint_distances_rejects_out_of_bounds_waypoint(self):
        waypoint_distances = [
            {"label": "Near Corner", "distance_m": 25.0},
            {"label": "Far Corner", "distance_m": 175.0},
        ]

        with self.assertRaisesRegex(ValueError, "Far Corner"):
            mission4.validate_waypoint_distances(waypoint_distances, 150.0)


class GuidedSpeedCommandTests(unittest.TestCase):
    def test_set_ground_speed_accepts_matching_command_ack(self):
        master = FakeMaster(
            messages=[
                FakeMessage("COMMAND_ACK", command=999, result=mavutil.mavlink.MAV_RESULT_ACCEPTED),
                FakeMessage(
                    "COMMAND_ACK",
                    command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                ),
            ]
        )

        self.assertTrue(maneuvers.set_ground_speed(master, 0.5))
        self.assertEqual(len(master.mav.command_long_calls), 1)

    def test_set_ground_speed_rejects_denied_ack(self):
        master = FakeMaster(
            messages=[
                FakeMessage(
                    "COMMAND_ACK",
                    command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    result=mavutil.mavlink.MAV_RESULT_DENIED,
                ),
            ]
        )

        self.assertFalse(maneuvers.set_ground_speed(master, 0.5))

    def test_set_ground_speed_rejects_timeout(self):
        master = FakeMaster()

        with mock.patch("maneuvers.wait_for_command_ack", return_value=None):
            self.assertFalse(maneuvers.set_ground_speed(master, 0.5))

    def test_goto_waypoint_reasserts_ground_speed_before_position_target(self):
        master = FakeMaster(
            messages=[
                FakeMessage(
                    "COMMAND_ACK",
                    command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                ),
            ]
        )

        result = maneuvers.goto_waypoint(master, 42.2989526, -83.8428926, 7.62, ground_speed_mps=0.5)

        self.assertTrue(result)
        self.assertEqual(len(master.mav.command_long_calls), 1)
        speed_command = master.mav.command_long_calls[0]
        self.assertEqual(speed_command[2], mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(speed_command[5], 0.5)
        self.assertEqual(len(master.mav.position_target_calls), 1)

    def test_goto_waypoint_continues_when_speed_ack_times_out(self):
        master = FakeMaster()

        with mock.patch("maneuvers.wait_for_command_ack", return_value=None):
            result = maneuvers.goto_waypoint(master, 42.2989526, -83.8428926, 7.62, ground_speed_mps=0.5)

        self.assertTrue(result)
        self.assertEqual(len(master.mav.command_long_calls), 1)
        self.assertEqual(len(master.mav.position_target_calls), 1)

    def test_configure_wpnav_limits_sets_and_reads_back_parameters(self):
        master = FakeMaster(
            messages=[
                FakeMessage("PARAM_VALUE", param_id="WPNAV_SPEED", param_value=50.0),
                FakeMessage("PARAM_VALUE", param_id="WPNAV_ACCEL", param_value=50.0),
                FakeMessage("PARAM_VALUE", param_id="WPNAV_SPEED", param_value=50.0),
                FakeMessage("PARAM_VALUE", param_id="WPNAV_ACCEL", param_value=50.0),
            ]
        )

        wpnav_status = maneuvers.configure_wpnav_limits(master, 0.5, 0.5)

        self.assertTrue(wpnav_status["success"])
        self.assertEqual(wpnav_status["confirmed_speed_cm_s"], 50.0)
        self.assertEqual(wpnav_status["confirmed_accel_cm_s2"], 50.0)
        self.assertEqual(master.mav.param_set_calls[0][2], b"WPNAV_SPEED")
        self.assertEqual(master.mav.param_set_calls[1][2], b"WPNAV_ACCEL")
        self.assertEqual(master.mav.param_request_calls[0][2], b"WPNAV_SPEED")
        self.assertEqual(master.mav.param_request_calls[1][2], b"WPNAV_ACCEL")


if __name__ == "__main__":
    unittest.main()
