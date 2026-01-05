import argparse
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from kortex_api.MqttTransport import MqttTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2


@dataclass
class Pose:
    x: float
    y: float
    z: float
    theta_x: float  # degrees
    theta_y: float  # degrees
    theta_z: float  # degrees


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


def create_mqtt_base(ip: str, username: str, password: str, mqtt_port: int = 1883):
    """
    Standard Kortex control channel: MQTT 1883.
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
    Cyclic feedback channel: UDP 10001.
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


def set_servoing_mode(base: BaseClient):
    mode = Base_pb2.ServoingModeInformation()

    # Try multiple enum locations across versions
    try:
        mode.servoing_mode = _enum_value(Base_pb2, "SINGLE_LEVEL_SERVOING")
    except Exception:
        # Common alternative: Base_pb2.ServoingMode.Value("SINGLE_LEVEL_SERVOING")
        mode.servoing_mode = _enum_value(getattr(Base_pb2, "ServoingMode", None), "SINGLE_LEVEL_SERVOING")

    base.SetServoingMode(mode)


def wait_action_end_or_abort(base: BaseClient, timeout_s: float = 60.0) -> bool:
    e = threading.Event()

    # Resolve action events robustly
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


def execute_pose_waypoint(base: BaseClient, pose: Pose, name: str = "move", timeout_s: float = 60.0) -> bool:
    """
    Move using a single Cartesian waypoint (works when 'reach_pose' is not present).
    """
    waypoints = Base_pb2.WaypointList()

    # Optional fields may exist depending on version
    if hasattr(waypoints, "use_optimal_blending"):
        waypoints.use_optimal_blending = False

    cart = Base_pb2.CartesianWaypoint()
    cart.pose.x = pose.x
    cart.pose.y = pose.y
    cart.pose.z = pose.z
    cart.pose.theta_x = pose.theta_x
    cart.pose.theta_y = pose.theta_y
    cart.pose.theta_z = pose.theta_z

    # Reference frame enum varies by version
    try:
        cart.reference_frame = _enum_value(Base_pb2, "CARTESIAN_REFERENCE_FRAME_BASE")
    except Exception:
        cart.reference_frame = _enum_value(getattr(Base_pb2, "CartesianReferenceFrame", None), "CARTESIAN_REFERENCE_FRAME_BASE")

    cart.blending_radius = 0.0
    if hasattr(cart, "duration"):
        cart.duration = 0

    wp = waypoints.waypoints.add()
    wp.name = name
    wp.cartesian_waypoint.CopyFrom(cart)

    # Different versions expose different method names
    if hasattr(base, "ExecuteWaypointTrajectory"):
        base.ExecuteWaypointTrajectory(waypoints)
    elif hasattr(base, "ExecuteWaypointList"):
        base.ExecuteWaypointList(waypoints)
    else:
        raise RuntimeError("Your BaseClient has no ExecuteWaypointTrajectory/ExecuteWaypointList method.")

    return wait_action_end_or_abort(base, timeout_s=timeout_s)


def gripper_position(base: BaseClient, value_0_to_1: float):
    value = max(0.0, min(1.0, value_0_to_1))

    cmd = Base_pb2.GripperCommand()

    # Mode enum is often nested in newer versions
    mode_set = False
    for container in [Base_pb2, getattr(Base_pb2, "GripperCommand", None), getattr(getattr(Base_pb2, "GripperCommand", None), "Mode", None)]:
        try:
            cmd.mode = _enum_value(container, "GRIPPER_POSITION")
            mode_set = True
            break
        except Exception:
            pass

    if not mode_set:
        raise RuntimeError("Could not set gripper mode to GRIPPER_POSITION (enum name differs in your version).")

    finger = cmd.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = value

    base.SendGripperCommand(cmd)


def get_tool_pose(cyclic: BaseCyclicClient) -> Pose:
    fb = cyclic.RefreshFeedback()
    return Pose(
        x=fb.base.tool_pose_x,
        y=fb.base.tool_pose_y,
        z=fb.base.tool_pose_z,
        theta_x=fb.base.tool_pose_theta_x,
        theta_y=fb.base.tool_pose_theta_y,
        theta_z=fb.base.tool_pose_theta_z,
    )


def human_near(flag_file: Optional[Path]) -> bool:
    if flag_file is None or not flag_file.exists():
        return False
    try:
        return flag_file.read_text(encoding="utf-8").strip() == "1"
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--username", default="admin")
    ap.add_argument("--password", default="admin")
    ap.add_argument("--mqtt_port", type=int, default=1883)
    ap.add_argument("--cycles", type=int, default=10)

    ap.add_argument("--pick", nargs=6, type=float, required=True)
    ap.add_argument("--place", nargs=6, type=float, required=True)
    ap.add_argument("--approach_z", type=float, default=0.08)

    ap.add_argument("--human_flag_file", default="")
    ap.add_argument("--stop_on_human", action="store_true")

    args = ap.parse_args()

    pick = Pose(*args.pick)
    place = Pose(*args.place)

    pick_above = Pose(pick.x, pick.y, pick.z + args.approach_z, pick.theta_x, pick.theta_y, pick.theta_z)
    place_above = Pose(place.x, place.y, place.z + args.approach_z, place.theta_x, place.theta_y, place.theta_z)

    flag_file = Path(args.human_flag_file) if args.human_flag_file.strip() else None

    # MQTT control + UDP cyclic
    mqtt_transport, mqtt_router, mqtt_session, base = create_mqtt_base(
        args.ip, args.username, args.password, mqtt_port=args.mqtt_port
    )
    udp_transport, udp_router, udp_session, cyclic = create_udp_cyclic(
        args.ip, args.username, args.password
    )

    try:
        set_servoing_mode(base)
        print("[INFO] Connected. Tool pose:", get_tool_pose(cyclic))

        gripper_position(base, 0.0)
        time.sleep(0.3)

        for i in range(1, args.cycles + 1):
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

            if not execute_pose_waypoint(base, pick_above, "above_pick"):
                break
            gripper_position(base, 0.0)
            time.sleep(0.2)

            if not execute_pose_waypoint(base, pick, "pick"):
                break
            gripper_position(base, 1.0)
            time.sleep(0.4)

            if not execute_pose_waypoint(base, pick_above, "retreat_pick"):
                break

            if not execute_pose_waypoint(base, place_above, "above_place"):
                break
            if not execute_pose_waypoint(base, place, "place"):
                break

            gripper_position(base, 0.0)
            time.sleep(0.2)

            if not execute_pose_waypoint(base, place_above, "retreat_place"):
                break

        print("[DONE]")

    finally:
        close_session(udp_transport, udp_session)
        close_session(mqtt_transport, mqtt_session)
