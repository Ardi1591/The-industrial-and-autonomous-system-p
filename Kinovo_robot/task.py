import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

from kortex_api.MqttTransport import MqttTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2


# -----------------------------
# Data structures
# -----------------------------

@dataclass
class Pose:
    """Operational-space pose p in BASE frame: meters + degrees."""
    x: float
    y: float
    z: float
    theta_x: float  # degrees
    theta_y: float  # degrees
    theta_z: float  # degrees


# -----------------------------
# Utilities: enums, errors
# -----------------------------

def _error_cb(exc: Exception) -> None:
    print(f"[KORTEX ERROR CALLBACK] {exc}")


def _enum_value(container, name: str) -> int:
    """
    Robust enum resolver for protobuf-generated enums across versions.
    Tries:
      - container.NAME
      - container.Value("NAME")
    """
    if container is None:
        raise AttributeError("Enum container is None")

    if hasattr(container, name):
        return getattr(container, name)

    if hasattr(container, "Value"):
        try:
            return container.Value(name)
        except Exception:
            pass

    raise AttributeError(f"Could not resolve enum value '{name}' in {container}")


# -----------------------------
# Kinematics-friendly helpers (logging / interpolation)
# -----------------------------

def pose_vec(p: Pose):
    """Pose vector p = [x,y,z,theta_x,theta_y,theta_z]."""
    return [p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z]


def l2_pos_error(actual: Pose, target: Pose) -> float:
    """||e_p|| = sqrt((dx)^2+(dy)^2+(dz)^2)."""
    dx = actual.x - target.x
    dy = actual.y - target.y
    dz = actual.z - target.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def cubic_time_scaling(t: float, T: float) -> float:
    """
    Cubic time-scaling with zero end velocities:
        s(t)=3(t/T)^2 - 2(t/T)^3,  t∈[0,T]
    """
    if T <= 0:
        return 1.0
    tau = max(0.0, min(1.0, t / T))
    return 3.0 * tau * tau - 2.0 * tau * tau * tau


def interp_pose_cubic(p0: Pose, p1: Pose, t: float, T: float) -> Pose:
    """Interpolate each pose component using cubic time scaling."""
    s = cubic_time_scaling(t, T)
    return Pose(
        x=p0.x + s * (p1.x - p0.x),
        y=p0.y + s * (p1.y - p0.y),
        z=p0.z + s * (p1.z - p0.z),
        theta_x=p0.theta_x + s * (p1.theta_x - p0.theta_x),
        theta_y=p0.theta_y + s * (p1.theta_y - p0.theta_y),
        theta_z=p0.theta_z + s * (p1.theta_z - p0.theta_z),
    )


# -----------------------------
# Kortex connection helpers
# -----------------------------

def create_mqtt_base(ip: str, username: str, password: str, mqtt_port: int = 1883):
    """
    Standard Kortex control channel: MQTT (default 1883).
    Returns (transport, router, session_manager, base_client).
    """
    transport = MqttTransport()
    router = RouterClient(transport, _error_cb)
    transport.connect(ip, mqtt_port)

    session_info = Session_pb2.CreateSessionInfo()
    session_info.username = username
    session_info.password = password
    session_info.session_inactivity_timeout = 60000
    session_info.connection_inactivity_timeout = 2000

    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)

    base = BaseClient(router)
    return transport, router, session_manager, base


def create_udp_cyclic(ip: str, username: str, password: str, port: int = 10001):
    """
    Cyclic feedback channel: UDP (default 10001).
    Returns (transport, router, session_manager, cyclic_client).
    """
    transport = UDPTransport()
    router = RouterClient(transport, _error_cb)
    transport.connect(ip, port)

    session_info = Session_pb2.CreateSessionInfo()
    session_info.username = username
    session_info.password = password
    session_info.session_inactivity_timeout = 60000
    session_info.connection_inactivity_timeout = 2000

    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)

    cyclic = BaseCyclicClient(router)
    return transport, router, session_manager, cyclic


def close_session(transport, session_manager):
    try:
        session_manager.CloseSession()
    finally:
        transport.disconnect()


# -----------------------------
# Robot commands
# -----------------------------

def set_servoing_mode(base: BaseClient):
    """Enable SINGLE_LEVEL_SERVOING for trajectory execution."""
    mode = Base_pb2.ServoingModeInformation()

    try:
        mode.servoing_mode = _enum_value(Base_pb2, "SINGLE_LEVEL_SERVOING")
    except Exception:
        mode.servoing_mode = _enum_value(getattr(Base_pb2, "ServoingMode", None), "SINGLE_LEVEL_SERVOING")

    base.SetServoingMode(mode)


def wait_action_end_or_abort(base: BaseClient, timeout_s: float = 60.0) -> bool:
    """Wait for ACTION_END or ACTION_ABORT notification."""
    e = threading.Event()

    try:
        ACTION_END = _enum_value(Base_pb2, "ACTION_END")
        ACTION_ABORT = _enum_value(Base_pb2, "ACTION_ABORT")
    except Exception:
        ACTION_END = _enum_value(getattr(Base_pb2, "ActionEvent", None), "ACTION_END")
        ACTION_ABORT = _enum_value(getattr(Base_pb2, "ActionEvent", None), "ACTION_ABORT")

    def _check(notification, event=e):
        if notification.action_event in (ACTION_END, ACTION_ABORT):
            event.set()

    handle = base.OnNotificationActionTopic(_check, Base_pb2.NotificationOptions())
    ok = e.wait(timeout_s)
    base.Unsubscribe(handle)
    return ok


def _base_ref_frame_enum() -> int:
    """Resolve CARTESIAN_REFERENCE_FRAME_BASE enum across versions."""
    try:
        return _enum_value(Base_pb2, "CARTESIAN_REFERENCE_FRAME_BASE")
    except Exception:
        return _enum_value(getattr(Base_pb2, "CartesianReferenceFrame", None), "CARTESIAN_REFERENCE_FRAME_BASE")


def execute_pose_waypoint(base: BaseClient, pose: Pose, name: str = "move", timeout_s: float = 60.0) -> bool:
    """
    Move using a single Cartesian waypoint.
    This is the simplest operational-space command: directly send p_d.
    """
    waypoints = Base_pb2.WaypointList()

    if hasattr(waypoints, "use_optimal_blending"):
        waypoints.use_optimal_blending = False

    cart = Base_pb2.CartesianWaypoint()
    cart.pose.x = pose.x
    cart.pose.y = pose.y
    cart.pose.z = pose.z
    cart.pose.theta_x = pose.theta_x
    cart.pose.theta_y = pose.theta_y
    cart.pose.theta_z = pose.theta_z

    cart.reference_frame = _base_ref_frame_enum()
    cart.blending_radius = 0.0
    if hasattr(cart, "duration"):
        cart.duration = 0

    wp = waypoints.waypoints.add()
    wp.name = name
    wp.cartesian_waypoint.CopyFrom(cart)

    if hasattr(base, "ExecuteWaypointTrajectory"):
        base.ExecuteWaypointTrajectory(waypoints)
    elif hasattr(base, "ExecuteWaypointList"):
        base.ExecuteWaypointList(waypoints)
    else:
        raise RuntimeError("Your BaseClient has no ExecuteWaypointTrajectory/ExecuteWaypointList method.")

    return wait_action_end_or_abort(base, timeout_s=timeout_s)


def execute_cartesian_segment(
    base: BaseClient,
    start: Pose,
    goal: Pose,
    name_prefix: str,
    T: float = 2.0,
    n_waypoints: int = 10,
    timeout_s: float = 60.0,
) -> bool:
    """
    Execute a smooth Cartesian segment using cubic time scaling.

    p(t) = p0 + s(t)*(pf - p0),  s(t)=3(t/T)^2 - 2(t/T)^3
    Implemented by generating a waypoint list sampled along p(t).
    """
    waypoints = Base_pb2.WaypointList()
    if hasattr(waypoints, "use_optimal_blending"):
        waypoints.use_optimal_blending = False

    ref_frame = _base_ref_frame_enum()
    n_waypoints = max(2, int(n_waypoints))

    for k in range(n_waypoints):
        t = (T * k) / (n_waypoints - 1)
        p = interp_pose_cubic(start, goal, t, T)

        cart = Base_pb2.CartesianWaypoint()
        cart.pose.x = p.x
        cart.pose.y = p.y
        cart.pose.z = p.z
        cart.pose.theta_x = p.theta_x
        cart.pose.theta_y = p.theta_y
        cart.pose.theta_z = p.theta_z
        cart.reference_frame = ref_frame
        cart.blending_radius = 0.0
        if hasattr(cart, "duration"):
            cart.duration = 0

        wp = waypoints.waypoints.add()
        wp.name = f"{name_prefix}_{k:02d}"
        wp.cartesian_waypoint.CopyFrom(cart)

    if hasattr(base, "ExecuteWaypointTrajectory"):
        base.ExecuteWaypointTrajectory(waypoints)
    elif hasattr(base, "ExecuteWaypointList"):
        base.ExecuteWaypointList(waypoints)
    else:
        raise RuntimeError("Your BaseClient has no ExecuteWaypointTrajectory/ExecuteWaypointList method.")

    return wait_action_end_or_abort(base, timeout_s=timeout_s)


def gripper_position(base: BaseClient, value_0_to_1: float):
    """Open/close gripper using normalized position [0..1]."""
    value = max(0.0, min(1.0, value_0_to_1))
    cmd = Base_pb2.GripperCommand()

    mode_set = False
    for container in [
        Base_pb2,
        getattr(Base_pb2, "GripperCommand", None),
        getattr(getattr(Base_pb2, "GripperCommand", None), "Mode", None),
    ]:
        try:
            cmd.mode = _enum_value(container, "GRIPPER_POSITION")
            mode_set = True
            break
        except Exception:
            pass

    if not mode_set:
        raise RuntimeError("Could not set gripper mode to GRIPPER_POSITION (enum differs in your version).")

    finger = cmd.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = value
    base.SendGripperCommand(cmd)


def get_tool_pose(cyclic: BaseCyclicClient) -> Pose:
    """Read tool pose feedback from cyclic channel."""
    fb = cyclic.RefreshFeedback()
    return Pose(
        x=fb.base.tool_pose_x,
        y=fb.base.tool_pose_y,
        z=fb.base.tool_pose_z,
        theta_x=fb.base.tool_pose_theta_x,
        theta_y=fb.base.tool_pose_theta_y,
        theta_z=fb.base.tool_pose_theta_z,
    )


# -----------------------------
# Safety helper
# -----------------------------

def human_near(flag_file: Optional[Path]) -> bool:
    """
    If flag_file exists and contains '1', treat as human near => pause.
    """
    if flag_file is None or not flag_file.exists():
        return False
    try:
        return flag_file.read_text(encoding="utf-8").strip() == "1"
    except Exception:
        return False


# -----------------------------
# Main routine
# -----------------------------

def main():
    ap = argparse.ArgumentParser(description="Kinova Cartesian pick-place using Kortex API.")
    ap.add_argument("--ip", required=True)
    ap.add_argument("--username", default="admin")
    ap.add_argument("--password", default="admin")
    ap.add_argument("--mqtt_port", type=int, default=1883)
    ap.add_argument("--cycles", type=int, default=10)

    # Pose inputs: meters + degrees (BASE frame)
    ap.add_argument("--pick", nargs=6, type=float, required=True, metavar=("X","Y","Z","THX","THY","THZ"))
    ap.add_argument("--place", nargs=6, type=float, required=True, metavar=("X","Y","Z","THX","THY","THZ"))

    ap.add_argument("--approach_z", type=float, default=0.08, help="Approach offset in +Z (m)")

    # Safety
    ap.add_argument("--human_flag_file", default="", help="Path to file containing '1' when human is near")
    ap.add_argument("--stop_on_human", action="store_true", help="Try to send Stop() while human near")

    # Paper-friendly trajectory options
    ap.add_argument("--use_cubic_segment", action="store_true",
                    help="Use cubic-interpolated Cartesian segment (waypoint list) instead of single waypoint.")
    ap.add_argument("--segment_time", type=float, default=2.0,
                    help="Duration (s) of each Cartesian segment when --use_cubic_segment is enabled.")
    ap.add_argument("--waypoints", type=int, default=10,
                    help="Number of waypoints per segment when --use_cubic_segment is enabled.")
    ap.add_argument("--report_pose_error", action="store_true",
                    help="Print ||e_p|| position error norm after each move.")

    args = ap.parse_args()

    pick = Pose(*args.pick)
    place = Pose(*args.place)

    pick_above = Pose(pick.x, pick.y, pick.z + args.approach_z, pick.theta_x, pick.theta_y, pick.theta_z)
    place_above = Pose(place.x, place.y, place.z + args.approach_z, place.theta_x, place.theta_y, place.theta_z)

    flag_file = Path(args.human_flag_file) if args.human_flag_file.strip() else None

    # Connect: MQTT control + UDP cyclic feedback
    mqtt_transport, mqtt_router, mqtt_session, base = create_mqtt_base(
        args.ip, args.username, args.password, mqtt_port=args.mqtt_port
    )
    udp_transport, udp_router, udp_session, cyclic = create_udp_cyclic(args.ip, args.username, args.password)

    def move_to(target: Pose, label: str) -> bool:
        """
        Move end-effector to a target pose:
          - If --use_cubic_segment: generate a cubic interpolated waypoint list from current pose to target
          - Else: single waypoint move
        """
        cur = get_tool_pose(cyclic)

        if args.use_cubic_segment:
            ok = execute_cartesian_segment(
                base,
                start=cur,
                goal=target,
                name_prefix=label,
                T=args.segment_time,
                n_waypoints=args.waypoints,
                timeout_s=60.0,
            )
        else:
            ok = execute_pose_waypoint(base, target, name=label, timeout_s=60.0)

        if ok and args.report_pose_error:
            after = get_tool_pose(cyclic)
            e = l2_pos_error(after, target)
            print(f"[METRIC] ||e_p|| after {label} = {e:.6f} m")

        return ok

    try:
        set_servoing_mode(base)
        print("[INFO] Connected. Tool pose:", pose_vec(get_tool_pose(cyclic)))

        # Open gripper initially
        gripper_position(base, 0.0)
        time.sleep(0.3)

        for i in range(1, args.cycles + 1):
            # Safety pause loop
            while human_near(flag_file):
                print("[SAFETY] Human near -> pausing.")
                if args.stop_on_human:
                    for stop_name in ("Stop", "StopAction"):
                        if hasattr(base, stop_name):
                            try:
                                getattr(base, stop_name)()
                            except Exception:
                                pass
                time.sleep(0.2)

            print(f"\n[CYCLE {i}/{args.cycles}]")

            # Approach pick
            if not move_to(pick_above, "above_pick"):
                break
            gripper_position(base, 0.0)
            time.sleep(0.2)

            # Pick
            if not move_to(pick, "pick"):
                break
            gripper_position(base, 1.0)
            time.sleep(0.4)

            # Retreat
            if not move_to(pick_above, "retreat_pick"):
                break

            # Approach place
            if not move_to(place_above, "above_place"):
                break

            # Place
            if not move_to(place, "place"):
                break
            gripper_position(base, 0.0)
            time.sleep(0.2)

            # Retreat
            if not move_to(place_above, "retreat_place"):
                break

        print("[DONE]")

    finally:
        close_session(udp_transport, udp_session)
        close_session(mqtt_transport, mqtt_session)


if __name__ == "__main__":
    main()
