import time
import math
from dataclasses import dataclass
from typing import List
from pymavlink import mavutil

# --- CONFIGURATION ---
CONNECTION_STRING = 'tcp:172.17.14.51:5762'

# How close (in meters) the drone needs to be before we consider a waypoint "reached"
WAYPOINT_RADIUS = 2.0

# How long to hover at each waypoint before moving on (seconds). Set to 0 to skip.
WAYPOINT_HOVER_SECONDS = 2

# How often to print position updates (seconds)
STATUS_INTERVAL = 1.0

# How often to poll flight mode while waiting for crew to return to GUIDED (seconds)
MANUAL_POLL_INTERVAL = 1.0


# ---------------------------------------------------------------------------
# ArduCopter flight-mode constants
#
# The HEARTBEAT message's `custom_mode` field carries one of these values.
# Only GUIDED_MODE lets the Pi send position commands; everything else is
# treated as "crew has taken over — stand by."
# ---------------------------------------------------------------------------

GUIDED_MODE = 4

COPTER_MODE_NAMES = {
    0:  "STABILIZE",
    1:  "ACRO",
    2:  "ALT_HOLD",
    3:  "AUTO",
    4:  "GUIDED",
    5:  "LOITER",
    6:  "RTL",
    7:  "CIRCLE",
    9:  "LAND",
    11: "DRIFT",
    13: "SPORT",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    21: "SMART_RTL",
}


# ---------------------------------------------------------------------------
# Waypoint Definition
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    """A single navigation target defined in GPS coordinates.
    
    lat:  Latitude  in decimal degrees  (e.g. 37.7749)
    lon:  Longitude in decimal degrees  (e.g. -122.4194)
    alt:  Altitude  in meters AGL       (e.g. 10.0)
    name: Optional label shown in logs  (e.g. "Corner A")
    """
    lat: float
    lon: float
    alt: float
    name: str = ""

    def __str__(self):
        label = f" ({self.name})" if self.name else ""
        return f"[{self.lat:.6f}, {self.lon:.6f}, {self.alt:.1f}m AGL]{label}"


# ---------------------------------------------------------------------------
# Preset Mission Patterns
#
# Replace the coordinates below with real coordinates near your fly site.
# Each list can be passed directly to run_mission().
# ---------------------------------------------------------------------------

# --- Origin (used to build relative offsets below) ---
ORIGIN_LAT =   None # example: ArduPilot SITL default
ORIGIN_LON =   None # example: ArduPilot SITL default

def get_home_position(master, timeout=10):
    """
    Request and return the Pixhawk's stored home position (set at arming/launch).
    Returns (lat, lon, alt_m) or raises TimeoutError.
    """
    # Send a request for the home position message
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='HOME_POSITION', blocking=True, timeout=1)
        if msg:
            lat = msg.latitude  / 1e7   # stored as int32 (degrees * 1e7)
            lon = msg.longitude / 1e7
            alt = msg.altitude  / 1000  # stored as mm, convert to meters
            return lat, lon, alt

    raise TimeoutError("No HOME_POSITION message received from Pixhawk.")


def _offset(lat, lon, d_north_m, d_east_m):
    """Return a (lat, lon) shifted by d_north_m / d_east_m from origin."""
    delta_lat = d_north_m / 111_320.0
    delta_lon = d_east_m  / (111_320.0 * math.cos(math.radians(lat)))
    return lat + delta_lat, lon + delta_lon


# Simple four-corner box at 10 m altitude
def make_box_mission(side_m=30, alt=10.0) -> List[Waypoint]:
    corners = [
        ( side_m,  0),
        ( side_m,  side_m),
        (0,        side_m),
        (0,        0),
    ]
    wps = []
    for i, (dn, de) in enumerate(corners):
        lat, lon = _offset(ORIGIN_LAT, ORIGIN_LON, dn, de)
        wps.append(Waypoint(lat, lon, alt, name=f"Box corner {i+1}"))
    return wps


# Lawnmower / boustrophedon search over a rectangle
def make_lawnmower_mission(
    sw_corner_lat, sw_corner_lon, width_m=60, height_m=40, strip_spacing_m=10, alt=15.0
) -> List[Waypoint]:
    """
    Generates a lawnmower pattern covering width_m x height_m.
    Strips run east–west, separated by strip_spacing_m to the north.
    """
    wps = []
    y = 0.0
    row = 0
    while y <= height_m:
        # Even rows go east, odd rows go west
        x_start, x_end = (0, width_m) if row % 2 == 0 else (width_m, 0)
        lat_a, lon_a = _offset(sw_corner_lat, sw_corner_lon, y, x_start)
        lat_b, lon_b = _offset(sw_corner_lat, sw_corner_lon, y, x_end)
        wps.append(Waypoint(lat_a, lon_a, alt, name=f"Strip {row+1} start"))
        wps.append(Waypoint(lat_b, lon_b, alt, name=f"Strip {row+1} end"))
        y += strip_spacing_m
        row += 1
    return wps


# Tighter spiral / refined search around a point of interest
def make_refined_search_mission(
    center_lat, center_lon, radius_m=15, rings=3, alt=10.0
) -> List[Waypoint]:
    """
    Concentric rings of decreasing radius around a point of interest.
    Useful for a close-inspection pass after a lawnmower identifies a target.
    """
    wps = []
    for ring in range(rings, 0, -1):   # outermost ring first
        r = radius_m * (ring / rings)
        points = max(8, ring * 6)      # more points on larger rings
        for i in range(points):
            angle = 2 * math.pi * i / points
            dn = r * math.cos(angle)
            de = r * math.sin(angle)
            lat, lon = _offset(center_lat, center_lon, dn, de)
            wps.append(Waypoint(lat, lon, alt, name=f"Ring {ring} pt {i+1}"))
    return wps

# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def _haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in meters between two GPS points."""
    R = 6_371_000  # Earth radius in metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _distance_to_waypoint(vehicle, wp: Waypoint) -> float | None:
    """Return horizontal distance (m) to waypoint, or None if no GPS fix."""
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg is None:
        return None
    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7
    return _haversine_distance(current_lat, current_lon, wp.lat, wp.lon)


def _send_waypoint(vehicle, wp: Waypoint):
    """Command the vehicle to fly to a GPS waypoint (GUIDED mode)."""
    # SET_POSITION_TARGET_GLOBAL_INT is the right command for GUIDED GPS flight.
    # type_mask = 0b110111111000 = 0xFF8 keeps only position bits active.
    vehicle.mav.set_position_target_global_int_send(
        0,                                              # time_boot_ms (unused)
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                             # type_mask: only lat/lon/alt
        int(wp.lat * 1e7),                              # lat  (int 1e7)
        int(wp.lon * 1e7),                              # lon  (int 1e7)
        wp.alt,                                         # alt  (m AGL)
        0, 0, 0,                                        # vx, vy, vz (unused)
        0, 0, 0,                                        # afx, afy, afz (unused)
        0, 0,                                           # yaw, yaw_rate (unused)
    )


def _get_current_position(vehicle):
    """Return (lat, lon, alt_agl) from the latest GLOBAL_POSITION_INT message."""
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg is None:
        return None, None, None
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


# ---------------------------------------------------------------------------
# Manual-override detection
# ---------------------------------------------------------------------------

def get_flight_mode(vehicle) -> int | None:
    """Return the current ArduCopter custom_mode from the latest HEARTBEAT.

    Skips GCS heartbeats and returns the first one from the flight controller.
    Returns None if no FC heartbeat arrives within the timeout.
    """
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg is None:
            break   # timed out
        # Only trust heartbeats originating from the flight controller
        if msg.get_srcSystem() == vehicle.target_system:
            return msg.custom_mode
        # Otherwise it was a GCS heartbeat — keep looking
    return None


def mode_name(mode_id: int | None) -> str:
    """Human-readable label for a custom_mode integer."""
    if mode_id is None:
        return "UNKNOWN"
    return COPTER_MODE_NAMES.get(mode_id, f"MODE_{mode_id}")


def yield_to_manual(vehicle, wp: Waypoint):
    """Block until the flight controller returns to GUIDED mode.

    Called whenever the Pi detects the crew has switched away from GUIDED.
    During this wait the Pi sends NO commands — the crew has full authority.
    When GUIDED is restored the current waypoint is re-issued so the drone
    knows where to resume flying.
    """
    current_mode = get_flight_mode(vehicle)
    if current_mode == GUIDED_MODE:
        return   # nothing to do

    print(f"\n  ⚠️  MANUAL OVERRIDE DETECTED — mode is {mode_name(current_mode)}")
    print("     Pi is now SILENT. Crew has full control.")
    print("     Switch back to GUIDED to resume the mission.\n")

    # Keep polling until GUIDED is re-selected by the crew
    while True:
        time.sleep(MANUAL_POLL_INTERVAL)
        current_mode = get_flight_mode(vehicle)
        print(f"     Waiting for GUIDED... current mode: {mode_name(current_mode)}")
        if current_mode == GUIDED_MODE:
            break

    print(f"\n  ✅ GUIDED mode restored — resuming mission.")
    print(f"     Re-targeting waypoint: {wp}\n")
    # Re-send the waypoint so the drone knows the destination after the
    # mode switch (switching modes clears any in-progress target).
    _send_waypoint(vehicle, wp)


# ---------------------------------------------------------------------------
# Core mission runner
# ---------------------------------------------------------------------------

def fly_to_waypoint(vehicle, wp: Waypoint):
    """Send the drone to *wp* and block until it arrives within WAYPOINT_RADIUS.

    On every loop iteration the flight mode is checked first.  If the crew has
    switched away from GUIDED the Pi goes silent and waits (via yield_to_manual)
    until GUIDED is restored, then resumes toward the same waypoint.
    """
    print(f"\n  → Flying to waypoint {wp}")
    _send_waypoint(vehicle, wp)

    last_status = time.time()
    while True:
        # ── Safety check: pause if crew has taken manual control ──────────
        yield_to_manual(vehicle, wp)

        dist = _distance_to_waypoint(vehicle, wp)
        _, _, alt = _get_current_position(vehicle)

        now = time.time()
        if now - last_status >= STATUS_INTERVAL:
            if dist is not None and alt is not None:
                print(f"     Distance: {dist:.1f} m  |  Alt: {alt:.2f} m AGL")
            last_status = now

        if dist is not None and dist <= WAYPOINT_RADIUS:
            print(f"  ✓ Waypoint reached! ({dist:.1f} m)")
            break

        # Re-send the waypoint command periodically in case it gets dropped
        # (MAVLink UDP can occasionally drop packets)
        _send_waypoint(vehicle, wp)
        time.sleep(0.5)

    if WAYPOINT_HOVER_SECONDS > 0:
        print(f"  ⏸  Hovering for {WAYPOINT_HOVER_SECONDS}s...")
        time.sleep(WAYPOINT_HOVER_SECONDS)


def run_mission(vehicle, waypoints: List[Waypoint]):
    """Fly through a list of waypoints in order."""
    print(f"\n{'='*50}")
    print(f"  MISSION START — {len(waypoints)} waypoints")
    print(f"{'='*50}")

    for i, wp in enumerate(waypoints):
        print(f"\n  Waypoint {i+1}/{len(waypoints)}: {wp}")
        fly_to_waypoint(vehicle, wp)

    print(f"\n{'='*50}")
    print("  MISSION COMPLETE — all waypoints visited.")
    print(f"{'='*50}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # Base mission
    waypoints = make_box_mission(side_m=30, alt=10.0)

    TARGET_ALTITUDE = 10.0

    # ── 2. Connect ────────────────────────────────────────────────────────────
    print(f"Connecting to {CONNECTION_STRING}...")
    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
    vehicle.wait_heartbeat()
    print(f"Connected to System {vehicle.target_system}")

    # ── 3. Request data stream ────────────────────────────────────────────────
    print("Requesting data stream...")
    vehicle.mav.request_data_stream_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )

    # ── 4. GUIDED mode ────────────────────────────────────────────────────────
    print("Switching to GUIDED mode...")
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4
    )

    # ── 5. Arm ────────────────────────────────────────────────────────────────
    print("Arming motors...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    vehicle.motors_armed_wait()
    print("MOTORS ARMED!")

    # ── 6. Takeoff ────────────────────────────────────────────────────────────
    print(f"Taking off to {TARGET_ALTITUDE} m...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
    )

    # Wait until we're within 95 % of target altitude
    print("Climbing...")
    while True:
        # Check for manual override even during takeoff
        mode = get_flight_mode(vehicle)
        if mode is not None and mode != GUIDED_MODE:
            print(f"\n  ⚠️  Mode changed to {mode_name(mode)} during takeoff — waiting for GUIDED...")
            while get_flight_mode(vehicle) != GUIDED_MODE:
                time.sleep(MANUAL_POLL_INTERVAL)
            print("  ✅ GUIDED restored — re-sending takeoff command.")
            vehicle.mav.command_long_send(
                vehicle.target_system, vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE
            )

        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f"   Alt (AGL): {alt:.2f} m")
            if alt >= TARGET_ALTITUDE * 0.95:
                print("Target altitude reached!")
                break

    # Listen for commands from controller
    while True:
        msg = vehicle.recv_match(type='COMMAND_LONG', blocking=True)
    run_mission(vehicle, waypoints)

    # ── 8. Return to launch ───────────────────────────────────────────────────
    print("Returning to launch (RTL)...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("RTL command sent. Script complete.")


if __name__ == "__main__":
    main()