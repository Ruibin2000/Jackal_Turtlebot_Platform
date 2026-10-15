#!/usr/bin/env python3
"""Start a fresh Jackal map at a chosen pose without resetting base odometry.

Run on Jackal, parked, with the previous Nav2/SLAM stopped:
  source /etc/clearpath/setup.bash
  python3 start_jackal_slam_at_pose.py --x 1 --y 0 --yaw-deg 0

The session TF chain is map -> odom_start -> odom -> base_link.
Only this SLAM session uses odom_start. Existing base and Nav2 settings stay intact.
Ctrl+C stops this SLAM session and its offset publisher. --preview is read-only.
"""

import argparse
import copy
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import yaml


NAMESPACE = "/j100_0949"
SESSION_FRAME = "odom_start"


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y,
            w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W,
            w*W - x*X - y*Y - z*Z)


def rotate(q, v):
    conjugate = (-q[0], -q[1], -q[2], q[3])
    return quaternion_product(quaternion_product(q, (*v, 0.0)), conjugate)[:3]


def session_offset(position, quaternion, x, y, yaw_deg):
    """T_start_odom = T_desired_base * inverse(T_odom_base), xyzw quaternions."""
    values = (*position, *quaternion, x, y, yaw_deg)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Pose values must be finite")
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-9:
        raise ValueError("Odometry quaternion is invalid")
    q = tuple(v / norm for v in quaternion)
    half_yaw = math.radians(yaw_deg) / 2.0
    desired = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    rotation = quaternion_product(desired, (-q[0], -q[1], -q[2], q[3]))
    rotated_position = rotate(rotation, position)
    translation = tuple(a-b for a, b in zip((x, y, 0.0), rotated_position))
    return translation, rotation


def prepare_parameters(source):
    result = copy.deepcopy(source["slam_toolbox"]["ros__parameters"])
    # Remove saved-session initialization; this entry point always starts a new map.
    for key in ("map_start_pose", "map_start_at_dock", "map_file_name"):
        result.pop(key, None)
    result.update({
        "use_sim_time": False,
        "use_lifecycle_manager": False,
        "mode": "mapping",
        "map_frame": "map",
        "odom_frame": SESSION_FRAME,
        "base_frame": "base_link",
        "map_name": NAMESPACE + "/map",
        "scan_topic": NAMESPACE + "/sensors/lidar3d_0/scan",
        "map_file_name": "",
    })
    return {NAMESPACE + "/slam_toolbox": {"ros__parameters": result}}


def read_current_pose(timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=[])
    node = Node("jackal_start_pose_probe", namespace=NAMESPACE,
                cli_args=["--ros-args", "-r", "/tf:=" + NAMESPACE + "/tf",
                          "-r", "/tf_static:=" + NAMESPACE + "/tf_static"],
                use_global_arguments=False)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    first = None
    latest = None
    started = time.monotonic()
    conflicts = {"slam_toolbox", "amcl", "controller_server", "bt_navigator",
                 "jackal_slam_pose_offset", "jackal_slam_startup"}
    try:
        while time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            running = sorted(name for name, ns in node.get_node_names_and_namespaces()
                             if ns == NAMESPACE and name in conflicts)
            if running:
                raise RuntimeError("Stop the previous Nav2/SLAM/offset first: " + ", ".join(running))
            try:
                transform = buffer.lookup_transform("odom", "base_link", Time())
            except TransformException:
                continue
            age = (node.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if not -0.1 <= age <= 1.0:
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            latest = ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            if not all(math.isfinite(v) for group in latest for v in group):
                raise RuntimeError("Non-finite odometry transform")
            if first is None:
                first = (time.monotonic(), latest)
            if time.monotonic() - first[0] < 1.0 or time.monotonic() - started < 3.0:
                continue
            displacement = math.dist(first[1][0], latest[0])
            q0, q1 = first[1][1], latest[1]
            norm = math.sqrt(sum(v*v for v in q0) * sum(v*v for v in q1))
            if norm < 1e-9:
                raise RuntimeError("Invalid odometry quaternion")
            dot = min(1.0, abs(sum(a*b for a, b in zip(q0, q1))) / norm)
            rotation = 2 * math.acos(dot)
            if displacement > 0.01 or rotation > math.radians(1.0):
                raise RuntimeError("Robot pose changed during sampling; park it and try again")
            # A live map transform from an unrecognized node would create a second parent of odom.
            if buffer.can_transform("map", "odom", Time()):
                raise RuntimeError("A map-to-odom transform is still being published; stop its owner first")
            return latest
        raise RuntimeError("No fresh, stationary odom -> base_link TF; check Jackal TF and clocks")
    finally:
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def drive_lifecycle(read_state, change_state, is_running, timeout=45.0,
                    clock=time.monotonic, pause=time.sleep):
    """Reconcile actual lifecycle state after every request, including a lost reply."""
    deadline = clock() + timeout
    last = None
    attempts = {}
    while clock() < deadline:
        if not is_running():
            raise RuntimeError("SLAM launch exited during startup; see its log above")
        state = read_state()
        if state is None:
            pause(0.2)
            continue
        if state != last:
            print("SLAM lifecycle: " + {1: "unconfigured", 2: "inactive", 3: "active",
                  4: "finalized"}.get(state, str(state)), flush=True)
            last = state
        if state == 3:
            return
        if state == 4:
            raise RuntimeError("SLAM finalized before activation")
        transition = {1: 1, 2: 3}.get(state)  # CONFIGURE from unconfigured; ACTIVATE from inactive.
        if transition is not None:
            attempts[state] = attempts.get(state, 0) + 1
            if attempts[state] > 3:
                raise RuntimeError(f"SLAM remains in state {state} after three transition attempts")
            response = change_state(transition)
            if response is False:
                raise RuntimeError(f"SLAM rejected transition {transition}; see its configuration log")
            if response is None:
                print("Lifecycle reply timed out; checking actual state before any retry.", flush=True)
        pause(0.2)
    raise RuntimeError(f"SLAM activation timed out after {timeout:g}s; last observed state: {last}")


def activate_slam(child):
    import rclpy
    from lifecycle_msgs.srv import ChangeState, GetState
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("jackal_slam_startup", namespace=NAMESPACE, use_global_arguments=False)
    state_client = node.create_client(GetState, NAMESPACE + "/slam_toolbox/get_state")
    change_client = node.create_client(ChangeState, NAMESPACE + "/slam_toolbox/change_state")

    def call(client, request, timeout):
        if not client.wait_for_service(timeout_sec=0.5):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError("SLAM process exited while waiting for a lifecycle response")
                rclpy.spin_once(node, timeout_sec=0.1)
                if future.done():
                    return future.result()
            return None
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def get_state():
        response = call(state_client, GetState.Request(), 2.0)
        return None if response is None else response.current_state.id

    def change_state(transition):
        request = ChangeState.Request()
        request.transition.id = transition
        response = call(change_client, request, 4.0)
        return None if response is None else response.success

    try:
        drive_lifecycle(get_state, change_state, lambda: child.poll() is None)
        print("SLAM ACTIVE. Check map -> base_link before starting Nav2.", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def stop_launch(child):
    """Only signal this launch's dedicated process group; never other ROS processes."""
    if child.poll() is not None:
        return
    for sig, wait_seconds in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            if sig == signal.SIGINT:
                # Let ros2 launch forward SIGINT to its children exactly once.
                child.send_signal(sig)
            else:
                os.killpg(child.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            try:
                child.wait(timeout=0.2)
                return
            except subprocess.TimeoutExpired:
                pass
            except KeyboardInterrupt:
                continue


def launch_text(params_path, translation, rotation):
    offset_args = []
    for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), (*translation, *rotation)):
        offset_args += ["--" + key, str(value)]
    offset_args += ["--frame-id", SESSION_FRAME, "--child-frame-id", "odom"]
    return f'''from launch import LaunchDescription
from launch.actions import GroupAction, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    closing = False

    def stop_once(context):
        nonlocal closing
        if closing or context.is_shutdown:
            return []
        closing = True
        return [EmitEvent(event=Shutdown(reason="SLAM session process exited"))]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[OpaqueFunction(function=stop_once)])),
        GroupAction([
            PushRosNamespace({NAMESPACE!r}),
            SetRemap("/tf", {NAMESPACE + "/tf"!r}),
            SetRemap("/tf_static", {NAMESPACE + "/tf_static"!r}),
            Node(package="tf2_ros", executable="static_transform_publisher",
                 name="jackal_slam_pose_offset", arguments={offset_args!r}, output="screen"),
            Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
                 name="slam_toolbox", parameters=[{str(params_path)!r}], output="screen"),
        ]),
    ])
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--x", type=float, required=True, help="Desired initial map x in metres")
    parser.add_argument("--y", type=float, required=True, help="Desired initial map y in metres")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Desired initial heading in degrees; 0 = +X")
    parser.add_argument("--preview", action="store_true", help="Only read TF and print the calculated offset")
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw_deg)):
        parser.error("x, y and yaw must be finite")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("clearpath_nav2_demos"))
        source = yaml.safe_load((share / "config/j100/slam.yaml").read_text())
        parameters = prepare_parameters(source)
        print("Park the robot; previous Nav2 and SLAM must be stopped. Reading current TF...", flush=True)
        position, quaternion = read_current_pose(timeout=12.0)
        translation, rotation = session_offset(position, quaternion, args.x, args.y, args.yaw_deg)
        print(f"Requested map start: x={args.x:g} m, y={args.y:g} m, yaw={args.yaw_deg:g} deg", flush=True)
        print(f"{SESSION_FRAME} -> odom: translation={translation}, quaternion={rotation}", flush=True)
        if args.preview:
            print("Preview only; no transform published and no SLAM started.")
            return 0
        with tempfile.TemporaryDirectory(prefix="jackal_slam_pose_") as directory:
            params_path = Path(directory) / "slam.yaml"
            entry = Path(directory) / "session.launch.py"
            params_path.write_text(yaml.safe_dump(parameters, sort_keys=False))
            entry.write_text(launch_text(params_path, translation, rotation))
            print("Starting new SLAM. Keep this terminal open; Ctrl+C stops SLAM and its offset.", flush=True)
            # The launcher has a separate process group, so Ctrl+C is forwarded once.
            child = subprocess.Popen(["ros2", "launch", str(entry)], start_new_session=True)
            try:
                activate_slam(child)
                return child.wait()
            finally:
                stop_launch(child)
    except (RuntimeError, ValueError, KeyError, OSError, ImportError) as error:
        print(f"Cannot start SLAM: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
