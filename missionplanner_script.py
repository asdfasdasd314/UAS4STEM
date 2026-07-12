# Mission Planner IronPython: takeoff -> fly box corners -> RTL
#
# LOITER on the transmitter stops the script immediately and leaves the
# aircraft in LOITER (no RTL/LAND/DISARM override).
#
# Keep the USER CONFIGURATION block in sync with
# parameter_files/missionplanner_script.toml — Mission Planner IronPython
# cannot load tomllib, so values are mirrored here.
#
# Test in SITL first. Keep the transmitter ready for LOITER takeover.

import clr
import math

clr.AddReference("MissionPlanner")
clr.AddReference("MissionPlanner.Utilities")
clr.AddReference("MAVLink")

import MissionPlanner
import MAVLink


# ======================================================================
# USER CONFIGURATION  (mirror of parameter_files/missionplanner_script.toml)
# ======================================================================

FLIGHT_ALTITUDE_FT = 25.0
GROUND_SPEED_MPS = 2.0
CLOSE_BOX = True
ABORT_MODE = "LOITER"
WAYPOINT_ARRIVAL_RADIUS_M = 2.0
WAYPOINT_TIMEOUT_MS = 90000
TAKEOFF_TIMEOUT_MS = 60000
ARM_TIMEOUT_MS = 15000
POLL_INTERVAL_MS = 250

# Replace with real box corners before flight.
WAYPOINTS = [
    (42.2989526, -83.8428926),  # Corner 1
    (42.2987225, -83.8428605),  # Corner 2
    (42.2986670, -83.8436866),  # Corner 3
    (42.2989923, -83.8437080),  # Corner 4
]

FLIGHT_ALTITUDE_M = FLIGHT_ALTITUDE_FT * 0.3048


# ======================================================================
# HELPERS
# ======================================================================

def distance_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2.0) ** 2
    )
    return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def current_mode():
    return str(cs.mode).strip().upper()


def abort_if_loiter():
    if current_mode() == ABORT_MODE:
        print("SAFETY ABORT: LOITER detected — script stopping, aircraft stays in LOITER")
        raise SystemExit


def sleep_poll(elapsed_ms):
    Script.Sleep(POLL_INTERVAL_MS)
    return elapsed_ms + POLL_INTERVAL_MS


def wait_for_mode(expected_mode, timeout_ms):
    expected_mode = expected_mode.upper()
    elapsed_ms = 0
    while elapsed_ms < timeout_ms:
        mode = current_mode()
        if mode == ABORT_MODE and expected_mode != ABORT_MODE:
            abort_if_loiter()
        if mode == expected_mode:
            return True
        elapsed_ms = sleep_poll(elapsed_ms)
    return False


def wait_until_armed(timeout_ms):
    elapsed_ms = 0
    while elapsed_ms < timeout_ms:
        abort_if_loiter()
        if cs.armed:
            return True
        elapsed_ms = sleep_poll(elapsed_ms)
    return False


def set_ground_speed(speed_mps):
    abort_if_loiter()
    print("Setting ground speed to %.1f m/s" % speed_mps)
    accepted = MAV.doCommand(
        MAV.sysid, MAV.compid,
        MAVLink.MAV_CMD.DO_CHANGE_SPEED,
        1, float(speed_mps), -1, 0, 0, 0, 0
    )
    if not accepted:
        print("WARNING: ground-speed command was not confirmed")


def send_guided_waypoint(latitude, longitude, altitude_m):
    abort_if_loiter()
    waypoint = MissionPlanner.Utilities.Locationwp()
    MissionPlanner.Utilities.Locationwp.lat.SetValue(waypoint, float(latitude))
    MissionPlanner.Utilities.Locationwp.lng.SetValue(waypoint, float(longitude))
    MissionPlanner.Utilities.Locationwp.alt.SetValue(waypoint, float(altitude_m))
    MAV.setGuidedModeWP(waypoint)


def wait_for_waypoint(latitude, longitude, timeout_ms):
    elapsed_ms = 0
    while elapsed_ms < timeout_ms:
        abort_if_loiter()
        if not cs.armed:
            raise Exception("Aircraft disarmed while traveling to a waypoint")
        mode = current_mode()
        if mode != "GUIDED":
            raise Exception("Aircraft left GUIDED mode and entered %s" % mode)

        remaining_m = distance_m(float(cs.lat), float(cs.lng), latitude, longitude)
        print(
            "Distance: %.1f m | Alt: %.1f m | GS: %.1f m/s | Mode: %s"
            % (remaining_m, float(cs.alt), float(cs.groundspeed), mode)
        )
        if remaining_m <= WAYPOINT_ARRIVAL_RADIUS_M:
            return True
        elapsed_ms = sleep_poll(elapsed_ms)
    return False


def wait_for_takeoff(timeout_ms):
    elapsed_ms = 0
    while elapsed_ms < timeout_ms:
        abort_if_loiter()
        if not cs.armed:
            raise Exception("Aircraft disarmed during takeoff")
        mode = current_mode()
        if mode != "GUIDED":
            raise Exception("Aircraft left GUIDED during takeoff (%s)" % mode)

        alt_m = float(cs.alt)
        print(
            "Takeoff: %.1f / %.1f m | GS: %.1f m/s | Mode: %s"
            % (alt_m, FLIGHT_ALTITUDE_M, float(cs.groundspeed), mode)
        )
        if alt_m >= FLIGHT_ALTITUDE_M - 1.0:
            return True
        elapsed_ms = sleep_poll(elapsed_ms)
    return False


def command_rtl():
    if current_mode() == ABORT_MODE:
        print("Aircraft is in LOITER; RTL suppressed")
        return
    print("Commanding RTL")
    Script.ChangeMode("RTL")
    if not wait_for_mode("RTL", 10000):
        print("WARNING: RTL mode was not confirmed")


def validate_config():
    if len(WAYPOINTS) != 4:
        raise Exception("WAYPOINTS must contain exactly four box corners")
    if FLIGHT_ALTITUDE_M <= 0:
        raise Exception("FLIGHT_ALTITUDE_FT must be greater than zero")
    if GROUND_SPEED_MPS <= 0:
        raise Exception("GROUND_SPEED_MPS must be greater than zero")
    if GROUND_SPEED_MPS > 5.0:
        print("WARNING: ground speed is above 5.0 m/s")
    if WAYPOINT_ARRIVAL_RADIUS_M <= 0:
        raise Exception("WAYPOINT_ARRIVAL_RADIUS_M must be greater than zero")
    if abs(float(cs.lat)) < 0.000001 or abs(float(cs.lng)) < 0.000001:
        raise Exception("No valid current GPS position is available")


# ======================================================================
# MAIN
# ======================================================================

validate_config()

try:
    print("Starting box mission: %.1f ft, %.1f m/s, abort=%s"
          % (FLIGHT_ALTITUDE_FT, GROUND_SPEED_MPS, ABORT_MODE))
    abort_if_loiter()

    print("Changing to GUIDED")
    Script.ChangeMode("GUIDED")
    if not wait_for_mode("GUIDED", 10000):
        raise Exception("Could not enter GUIDED mode")

    print("Arming")
    MAV.doARM(True)
    if not wait_until_armed(ARM_TIMEOUT_MS):
        raise Exception("Arming failed — check Mission Planner Messages for PreArm")

    print("Taking off to %.2f m" % FLIGHT_ALTITUDE_M)
    takeoff_accepted = MAV.doCommand(
        MAV.sysid, MAV.compid,
        MAVLink.MAV_CMD.TAKEOFF,
        0, 0, 0, 0, 0, 0, float(FLIGHT_ALTITUDE_M)
    )
    if not takeoff_accepted:
        if cs.armed:
            MAV.doARM(False)
        raise Exception("Takeoff command failed")
    if not wait_for_takeoff(TAKEOFF_TIMEOUT_MS):
        raise Exception("Takeoff altitude was not reached in time")
    print("Takeoff complete")

    set_ground_speed(GROUND_SPEED_MPS)

    mission_waypoints = list(WAYPOINTS)
    if CLOSE_BOX:
        mission_waypoints.append(WAYPOINTS[0])

    for index, (latitude, longitude) in enumerate(mission_waypoints):
        print("Flying to waypoint %d/%d (%.7f, %.7f)"
              % (index + 1, len(mission_waypoints), latitude, longitude))
        set_ground_speed(GROUND_SPEED_MPS)
        send_guided_waypoint(latitude, longitude, FLIGHT_ALTITUDE_M)
        if not wait_for_waypoint(latitude, longitude, WAYPOINT_TIMEOUT_MS):
            raise Exception("Waypoint %d timed out" % (index + 1))
        print("Waypoint %d reached" % (index + 1))

    print("Box mission complete")
    command_rtl()

except SystemExit:
    print("Mission script terminated by LOITER safety switch")
    raise

except Exception as error:
    print("MISSION ERROR: %s" % str(error))
    if current_mode() == ABORT_MODE:
        print("Aircraft is in LOITER — preserving manual control")
    else:
        command_rtl()
    raise
