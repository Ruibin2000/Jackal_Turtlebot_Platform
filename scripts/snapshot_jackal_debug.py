#!/usr/bin/env python3
"""Read live ROS parameters and graph metadata; never start or reconfigure nodes."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

NS = "/j100_0949"
REQUIRED = [
    "controller_server", "planner_server", "behavior_server", "bt_navigator",
    "velocity_smoother", "collision_monitor", "local_costmap/local_costmap",
    "global_costmap/global_costmap", "slam_toolbox",
]
OPTIONAL = [
    "smoother_server", "sensors/lidar3d_0/ouster_driver", "robot_state_publisher",
    # J100 base nodes do not live below /platform/, unlike their topics.
    "platform_velocity_controller", "j100_hardware", "j100_hardware_interface",
]


def capture(args, timeout=12):
    try:
        result = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired as error:
        return 124, "", f"Timed out after {error.timeout} seconds\n"
    except OSError as error:
        return 127, "", str(error) + "\n"



def split_get_parameters(names, getter):
    """Isolate an uninitialized parameter without discarding its valid peers."""
    values, skipped = {}, {}

    def read(batch):
        if not batch:
            return
        try:
            result = getter(batch)
        except (RuntimeError, TimeoutError) as error:
            # A timeout is not evidence of one bad parameter: do not fan it out.
            skipped.update({name: str(error) for name in batch})
            return
        if len(result) != len(batch):
            if len(batch) == 1:
                skipped[batch[0]] = "No value returned (uninitialized or unreadable)"
            else:
                middle = len(batch) // 2
                read(batch[:middle])
                read(batch[middle:])
            return
        for name, value in zip(batch, result):
            if value is None:
                skipped[name] = "Parameter is not set"
            else:
                values[name] = value

    read(names)
    return values, skipped


def safe_dump_node(remote_node):
    """Read-only fallback using parameter services, with bounded waits."""
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters, ListParameters

    rclpy.init(args=[])
    probe = Node("jackal_debug_parameter_probe", enable_rosout=False,
                 start_parameter_services=False, use_global_arguments=False)
    deadline = time.monotonic() + 35
    try:
        list_client = probe.create_client(ListParameters, remote_node + "/list_parameters")
        get_client = probe.create_client(GetParameters, remote_node + "/get_parameters")
        for client in (list_client, get_client):
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError("Parameter service unavailable: " + client.srv_name)

        def call(client, request):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Parameter fallback total time limit exceeded")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(probe, future, timeout_sec=min(3, remaining))
            if not future.done():
                future.cancel()
                raise TimeoutError("Parameter service response timed out")
            result = future.result()
            if result is None:
                raise RuntimeError("Parameter service returned no response")
            return result

        request = ListParameters.Request()
        request.depth = 0  # recursive, all declared names
        names = sorted(call(list_client, request).result.names)

        def getter(batch):
            request = GetParameters.Request()
            request.names = batch
            response = call(get_client, request)
            return [parameter_value_to_python(value) if value.type != 0 else None
                    for value in response.values]

        values, skipped = split_get_parameters(names, getter)
        print(yaml.safe_dump({"parameters": values, "unreadable_parameters": skipped}, sort_keys=False))
        return 0
    except (RuntimeError, TimeoutError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, nargs="?", help="New snapshot directory (must not exist)")
    parser.add_argument("--dump-node", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.dump_node:
        return safe_dump_node(args.dump_node)
    if args.output is None:
        parser.error("output is required")
    args.output.mkdir(parents=True, exist_ok=False)
    started = datetime.now(timezone.utc).isoformat()
    graph = {}
    for name, command in {
        "nodes": ["ros2", "node", "list"],
        "topics": ["ros2", "topic", "list", "-t", "--include-hidden-topics"],
        "actions": ["ros2", "action", "list", "-t"],
    }.items():
        rc, out, err = capture(command)
        (args.output / f"{name}.txt").write_text(out)
        (args.output / f"{name}.stderr.txt").write_text(err)
        graph[name] = {"returncode": rc, "output": out}
    nodes = [NS + "/" + name for name in REQUIRED + OPTIONAL]
    # Include the actual platform EKF, controller and mux node names when present.
    for name in graph["nodes"]["output"].splitlines():
        name = name.strip()
        if name.startswith(NS + "/platform/") or (
            name.startswith(NS + "/") and any(word in name.lower() for word in ("ekf", "twist_mux"))
        ):
            nodes.append(name)
    merged, report, failures, partial = {}, [], [], []
    for node in dict.fromkeys(nodes):
        print(f"Reading parameters: {node}", flush=True)
        rc, out, err = capture(["ros2", "param", "dump", node, "--timeout", "3"])
        stem = node.lstrip("/").replace("/", "__")
        try:
            data = yaml.safe_load(out) if rc == 0 else None
            values = data.get(node, {}).get("ros__parameters") if isinstance(data, dict) else None
            valid = isinstance(values, dict) and bool(values)
        except yaml.YAMLError:
            valid = False
        skipped = {}
        method = "ros2 param dump"
        # rclcpp can return an empty array for the whole request if just one
        # declared parameter is uninitialized; ros2 param dump still exits 0.
        if not valid and node in graph["nodes"]["output"].splitlines():
            print(f"Retrying readable parameters individually: {node}", flush=True)
            (args.output / f"{stem}.bulk_attempt.txt").write_text(f"returncode={rc}\n{out}\n{err}")
            rc, fallback_out, fallback_err = capture(
                [sys.executable, str(Path(__file__).resolve()), "--dump-node", node], timeout=45)
            err += "\nFallback:\n" + fallback_err
            try:
                fallback = yaml.safe_load(fallback_out) if rc == 0 else None
                values = fallback.get("parameters") if isinstance(fallback, dict) else None
                valid = isinstance(values, dict) and bool(values)
                if valid:
                    skipped = fallback.get("unreadable_parameters", {})
                    data = {node: {"ros__parameters": values}}
                    out = yaml.safe_dump(data, sort_keys=False)
            except yaml.YAMLError:
                valid = False
            method = "parameter service fallback"
        (args.output / f"{stem}.stderr.txt").write_text(err)
        if skipped:
            partial.append(node)
            (args.output / f"{stem}.unreadable.yaml").write_text(yaml.safe_dump(skipped))
            print(f"NOTE: saved readable values; {len(skipped)} unavailable parameter(s) in {node}", flush=True)
        if valid:
            (args.output / f"{stem}.yaml").write_text(out)
            merged[node] = data[node]
        else:
            (args.output / f"{stem}.failed.txt").write_text(f"returncode={rc}\n{out}\n{err}")
            if node in {NS + "/" + name for name in REQUIRED}:
                failures.append(node)
        report.append({"node": node, "ok": valid, "complete": valid and not skipped,
                       "returncode": rc, "method": method, "unreadable_parameters": skipped})
    (args.output / "all_parameters.yaml").write_text(yaml.safe_dump(merged, sort_keys=False))
    (args.output / "snapshot_status.yaml").write_text(yaml.safe_dump({
        "started_utc": started,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {key: os.environ.get(key, "") for key in (
            "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY")},
        "graph_commands": {key: value["returncode"] for key, value in graph.items()},
        "parameters": report,
        "missing_required_nodes": failures,
        "partially_read_nodes": partial,
    }, sort_keys=False))
    print(f"Saved {len(merged)} parameter files to {args.output}", flush=True)
    if partial:
        print("NOTE: some declared parameters were unavailable; see *.unreadable.yaml", file=sys.stderr)
    if failures:
        print("WARNING: incomplete snapshot; inspect snapshot_status.yaml and *.failed.txt", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
