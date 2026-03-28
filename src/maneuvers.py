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


def goto_waypoint(master, lat: float, lon: float, alt: float):
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


def _do_takeoff(master, alt: float):
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


def _do_goto(master, lat: float, lon: float, alt: float):
    goto_waypoint(master, lat, lon, alt)
    start = time.time()
    while True:
        if not wait_if_paused():
            return False
        goto_waypoint(master, lat, lon, alt)

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


def _do_rtl(master):
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
            if not _do_takeoff(master, p.get('alt', 50.0)):
                return False

        elif item.cmd == Cmd.GOTO:
            if not _do_goto(master, p['lat'], p['lon'], p['alt']):
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
            _do_rtl(master)
            return True  # Nothing valid can follow RTL

    print("\n>>> Mission complete.")
    return True
