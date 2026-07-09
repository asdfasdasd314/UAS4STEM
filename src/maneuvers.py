# maneuvers.py — Mission planning primitives, types, and executor

import time
import sys
import math
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from pymavlink import mavutil

# ==============================================================================
# CONFIGURATION
# ==============================================================================
WAYPOINT_RADIUS  = 3.0   # Meters — considered "arrived" within this radius
WAYPOINT_TIMEOUT = 60.0  # Seconds before giving up on a waypoint
COMMAND_ACK_TIMEOUT = 2.0
PARAM_TIMEOUT = 2.0

# ==============================================================================
# MISSION TYPES
# ==============================================================================
class Cmd(Enum):
    TAKEOFF   = auto()
    GOTO      = auto()
    ORBIT     = auto()
    YAW_SPIN  = auto()
    SET_ROI   = auto()
    CLEAR_ROI = auto()
    HOVER     = auto()
    LAND      = auto()
    RTL       = auto()
    SERVO     = auto()


@dataclass
class MissionItem:
    cmd: Cmd
    params: dict = field(default_factory=dict)
    label: str = ""

    def __str__(self):
        label_str = f" [{self.label}]" if self.label else ""
        param_str = ", ".join(f"{k}={v}" for k, v in self.params.items())
        return f"{self.cmd.name:<12}{label_str} | {param_str}"


def print_mission(mission: list[MissionItem]):
    """Pretty-print the mission plan before executing it."""
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║                   MISSION PLAN                      ║")
    print("╠══════════════════════════════════════════════════════╣")
    for i, item in enumerate(mission):
        print(f"║  {i + 1:>2}. {str(item):<49}║")
    print("╚══════════════════════════════════════════════════════╝\n")


# ==============================================================================
# FAILSAFE STATE
# ==============================================================================
mission_paused  = threading.Event()
mission_aborted = threading.Event()


def mode_monitor(master):
    """
    Background thread — watches HEARTBEAT messages.
    Pauses the mission any time the Pixhawk leaves GUIDED mode,
    and resumes it automatically when GUIDED is restored.
    """
    print(">>> [Monitor] Failsafe monitor started.")
    while not mission_aborted.is_set():
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if not msg:
            continue
        mode = mavutil.mode_string_v10(msg)
        if mode != 'GUIDED':
            if not mission_paused.is_set():
                print(f"\n[!] PILOT TAKEOVER — mode: {mode}. Mission PAUSED.")
                mission_paused.set()
        else:
            if mission_paused.is_set():
                print("\n>>> GUIDED restored — resuming mission.")
                mission_paused.clear()


def wait_if_paused():
    """
    Blocks until mission_paused is cleared.
    Returns False if the mission was aborted, True to continue.
    """
    if mission_paused.is_set():
        print(">>> Mission holding — waiting for GUIDED mode...")
        while mission_paused.is_set():
            if mission_aborted.is_set():
                return False
            time.sleep(0.25)
        print(">>> Mission resumed.")
    return not mission_aborted.is_set()


# ==============================================================================
# PRIMITIVE COMMANDS (private — use MissionItem + execute_mission)
# ==============================================================================
def _haversine_distance(lat1, lon1, lat2, lon2) -> float:
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi    = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def distance_between_locations(lat1, lon1, lat2, lon2) -> float:
    return _haversine_distance(lat1, lon1, lat2, lon2)


def get_current_location(master, timeout: float = 2.0):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if not msg:
        return None

    return {
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "relative_alt_m": msg.relative_alt / 1000.0,
    }


def _normalize_param_id(param_id) -> str:
    if isinstance(param_id, bytes):
        param_id = param_id.decode("ascii", errors="ignore")
    return str(param_id).rstrip("\x00")


def _mav_result_name(result: int) -> str:
    result_names = {
        mavutil.mavlink.MAV_RESULT_ACCEPTED: "ACCEPTED",
        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
        mavutil.mavlink.MAV_RESULT_DENIED: "DENIED",
        mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
        mavutil.mavlink.MAV_RESULT_FAILED: "FAILED",
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "IN_PROGRESS",
        mavutil.mavlink.MAV_RESULT_CANCELLED: "CANCELLED",
    }
    return result_names.get(result, f"UNKNOWN({result})")


def wait_for_command_ack(master, command_id: int, timeout: float = COMMAND_ACK_TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=max(0.1, deadline - time.time()))
        if not ack:
            continue
        if getattr(ack, "command", None) != command_id:
            continue
        return ack
    return None


def set_ground_speed(master, speed_mps: float, announce: bool = True) -> bool:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,
        speed_mps,
        -1,
        0,
        0,
        0,
        0,
    )

    ack = wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED)
    if not ack:
        print(
            f"[ERROR] Timed out waiting for COMMAND_ACK for MAV_CMD_DO_CHANGE_SPEED. "
            f"Ground speed {speed_mps:.1f} m/s was not confirmed."
        )
        return False

    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(
            f"[ERROR] MAV_CMD_DO_CHANGE_SPEED was {_mav_result_name(ack.result)}. "
            f"Ground speed {speed_mps:.1f} m/s was not applied."
        )
        return False

    if announce:
        print(f">>> Ground speed target set to {speed_mps:.1f} m/s")
    return True


def goto_waypoint(master, lat: float, lon: float, alt: float, ground_speed_mps: float | None = None):
    if ground_speed_mps is not None:
        # Re-send the speed target with each GUIDED waypoint update so nav speed
        # does not fall back to the autopilot default between position commands.
        if not set_ground_speed(master, ground_speed_mps, announce=False):
            return False

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    return True


def set_parameter(master, parameter_name: str, value: float, timeout: float = PARAM_TIMEOUT):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        parameter_name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=max(0.1, deadline - time.time()))
        if not msg:
            continue
        if _normalize_param_id(msg.param_id) != parameter_name:
            continue
        return float(msg.param_value)
    return None


def read_parameter(master, parameter_name: str, timeout: float = PARAM_TIMEOUT):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        parameter_name.encode("ascii"),
        -1,
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=max(0.1, deadline - time.time()))
        if not msg:
            continue
        if _normalize_param_id(msg.param_id) != parameter_name:
            continue
        return float(msg.param_value)
    return None


def configure_wpnav_limits(master, ground_speed_mps: float, accel_mps2: float):
    target_speed_cm_s = round(ground_speed_mps * 100.0)
    target_accel_cm_s2 = round(accel_mps2 * 100.0)

    print(
        f">>> Configuring ArduPilot nav limits: "
        f"WPNAV_SPEED={target_speed_cm_s} cm/s, "
        f"WPNAV_ACCEL={target_accel_cm_s2} cm/s^2"
    )

    set_speed_value = set_parameter(master, "WPNAV_SPEED", target_speed_cm_s)
    if set_speed_value is None:
        print("[ERROR] Timed out while setting WPNAV_SPEED on the flight controller.")

    set_accel_value = set_parameter(master, "WPNAV_ACCEL", target_accel_cm_s2)
    if set_accel_value is None:
        print("[ERROR] Timed out while setting WPNAV_ACCEL on the flight controller.")

    confirmed_speed = read_parameter(master, "WPNAV_SPEED")
    if confirmed_speed is None:
        print("[ERROR] Timed out while reading back WPNAV_SPEED from the flight controller.")

    confirmed_accel = read_parameter(master, "WPNAV_ACCEL")
    if confirmed_accel is None:
        print("[ERROR] Timed out while reading back WPNAV_ACCEL from the flight controller.")

    if confirmed_speed is not None:
        print(f">>> Confirmed WPNAV_SPEED from Pixhawk: {confirmed_speed:.0f} cm/s")
    if confirmed_accel is not None:
        print(f">>> Confirmed WPNAV_ACCEL from Pixhawk: {confirmed_accel:.0f} cm/s^2")

    return {
        "target_speed_cm_s": float(target_speed_cm_s),
        "target_accel_cm_s2": float(target_accel_cm_s2),
        "set_speed_cm_s": set_speed_value,
        "set_accel_cm_s2": set_accel_value,
        "confirmed_speed_cm_s": confirmed_speed,
        "confirmed_accel_cm_s2": confirmed_accel,
        "success": (
            confirmed_speed is not None
            and confirmed_accel is not None
            and round(confirmed_speed) == target_speed_cm_s
            and round(confirmed_accel) == target_accel_cm_s2
        ),
    }


def _do_takeoff(master, alt: float, ground_speed_mps: float | None = None):
    if ground_speed_mps is not None:
        if not set_ground_speed(master, ground_speed_mps):
            return False

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )
    while True:
        if not wait_if_paused():
            return False
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if not msg:
            continue
        current_alt = msg.relative_alt / 1000.0
        sys.stdout.write(f"\r    Climbing... {current_alt:.1f} / {alt:.1f} m   ")
        sys.stdout.flush()
        if current_alt >= alt * 0.95:
            print(f"\n>>> Takeoff complete.")
            return True


def _do_goto(master, lat: float, lon: float, alt: float, ground_speed_mps: float | None = None):
    if not goto_waypoint(master, lat, lon, alt, ground_speed_mps=ground_speed_mps):
        return False
    start = time.time()
    while True:
        if not wait_if_paused():
            return False
        if not goto_waypoint(master, lat, lon, alt, ground_speed_mps=ground_speed_mps):
            return False

        if time.time() - start > WAYPOINT_TIMEOUT:
            print(f"\n[WARN] GOTO timed out.")
            return True

        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if not msg:
            continue

        dist = _haversine_distance(msg.lat / 1e7, msg.lon / 1e7, lat, lon)
        sys.stdout.write(f"\r    Distance: {dist:.1f}m | Alt: {msg.relative_alt / 1000:.1f}m   ")
        sys.stdout.flush()

        if dist <= WAYPOINT_RADIUS:
            print(f"\n>>> Waypoint reached.")
            return True


def _do_orbit(master, lat, lon, radius=10.0, velocity=3.0, clockwise=True):
    speed = velocity if clockwise else -velocity
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_ORBIT,
        0, radius, speed, 0, 0, lat, lon, 0
    )


def _do_yaw_spin(master, degrees=360, speed=30, clockwise=True):
    direction = 1 if clockwise else -1
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, degrees, speed, direction, 1, 0, 0, 0
    )


def _do_set_roi(master, lat, lon, alt=0.0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0, 0, 0, 0, 0, lat, lon, alt
    )
    print(f">>> ROI set to ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")


def _do_clear_roi(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0, mavutil.mavlink.MAV_ROI_NONE, 0, 0, 0, 0, 0, 0
    )
    print(">>> ROI cleared.")


def _do_hover(master, seconds=5.0):
    print(f">>> Hovering for {seconds}s...")
    deadline = time.time() + seconds
    while time.time() < deadline:
        if not wait_if_paused():
            return False
        time.sleep(0.25)
    return True


def _do_servo(master, channel: int, pwm: int):
    pwm = max(1000, min(2000, pwm))
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm, 0, 0, 0, 0, 0
    )
    print(f">>> Servo CH{channel} → {pwm} µs")


def _do_land(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print(">>> Landing — waiting for disarm...")
    master.motors_disarmed_wait()
    print(">>> LANDING COMPLETE.")


def _do_rtl(master, ground_speed_mps: float | None = None):
    if ground_speed_mps is not None:
        if not set_ground_speed(master, ground_speed_mps):
            print("[WARN] RTL speed target was not confirmed before return-to-launch.")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print(">>> RTL commanded — waiting for disarm...")
    master.motors_disarmed_wait()
    print(">>> RTL complete.")


# ==============================================================================
# MISSION EXECUTOR
# ==============================================================================
def execute_mission(master, mission: list[MissionItem]):
    """
    Execute a list of MissionItems in order.
    Checks the failsafe before every command.
    Stops early and returns False if the mission is aborted.
    """
    print_mission(mission)

    for i, item in enumerate(mission):
        if not wait_if_paused():
            print("\n>>> Mission aborted.")
            return False

        print(f"\n[{i + 1}/{len(mission)}] Executing: {item}")
        p = item.params

        if item.cmd == Cmd.TAKEOFF:
            if not _do_takeoff(master, p.get('alt', 50.0), p.get('ground_speed_mps')):
                return False

        elif item.cmd == Cmd.GOTO:
            if not _do_goto(master, p['lat'], p['lon'], p['alt'], p.get('ground_speed_mps')):
                return False

        elif item.cmd == Cmd.ORBIT:
            _do_orbit(master, p['lat'], p['lon'],
                      p.get('radius', 10.0),
                      p.get('velocity', 3.0),
                      p.get('clockwise', True))

        elif item.cmd == Cmd.YAW_SPIN:
            _do_yaw_spin(master,
                         p.get('degrees', 360),
                         p.get('speed', 30),
                         p.get('clockwise', True))

        elif item.cmd == Cmd.SET_ROI:
            _do_set_roi(master, p['lat'], p['lon'], p.get('alt', 0.0))

        elif item.cmd == Cmd.CLEAR_ROI:
            _do_clear_roi(master)

        elif item.cmd == Cmd.HOVER:
            if not _do_hover(master, p.get('seconds', 5.0)):
                return False

        elif item.cmd == Cmd.SERVO:
            _do_servo(master, p['channel'], p['pwm'])

        elif item.cmd == Cmd.LAND:
            _do_land(master)
            return True  # Nothing valid can follow a landing

        elif item.cmd == Cmd.RTL:
            _do_rtl(master, p.get('ground_speed_mps'))
            return True  # Nothing valid can follow RTL

    print("\n>>> Mission complete.")
    return True
