#!/usr/bin/env python3
"""Configure Ouster ring 15 and optional scan frames on j100-0949.

Run on Jackal after stopping Nav2/SLAM and parking the robot:
    sudo python3 configure_jackal_lidar.py --apply
Add --fix-scan-frame to correct the verified scan/point-cloud frame mismatch.
Installation xyz/rpy are always preserved. Without --apply, only preview changes.
This script never commands motion or restarts services.
Requires python3-yaml, included in the robot's ROS installation.
"""

import argparse
import copy
import os
from pathlib import Path
import shutil
import stat
import tempfile

import yaml


# OS-1-32-U1-SR metadata measured on this robot: ring 15 = +0.32 degrees.
DRIVER_PARAMETERS = {"scan_ring": 15}
# Ouster scans use its lidar coordinates; point clouds currently use sensor
# coordinates. Let the driver publish their calibration-derived transform.
# A distinct IMU frame also prevents the driver publishing a self-transform.
SCAN_FRAME_PARAMETERS = {
    "sensor_frame": "lidar3d_0_sensor_link",
    "point_cloud_frame": "lidar3d_0_sensor_link",
    "lidar_frame": "lidar3d_0_laser",
    "imu_frame": "lidar3d_0_imu",
    "pub_static_tf": True,
}


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate keys instead of silently discarding part of robot.yaml."""


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping
)


def updated_config(original, fix_scan_frame=False):
    if not isinstance(original, dict):
        raise ValueError("robot.yaml must contain a mapping")
    if original.get("serial_number") != "j100-0949":
        raise ValueError("This correction is only for serial_number: j100-0949")
    sensors = original.get("sensors", {}).get("lidar3d", [])
    if not sensors or sensors[0].get("model") != "ouster_os1":
        raise ValueError("Expected sensors.lidar3d[0].model: ouster_os1")
    sensor = copy.deepcopy(sensors[0])
    parameters = copy.deepcopy(sensor.get("ros_parameters", {}))
    if not isinstance(parameters, dict):
        raise ValueError("ros_parameters must be an inline mapping")
    driver = copy.deepcopy(parameters.get("ouster_driver", {}))
    if not isinstance(driver, dict):
        raise ValueError("ouster_driver parameters must be a mapping")
    desired = dict(DRIVER_PARAMETERS)
    if fix_scan_frame:
        # Preserve the verified sensor/point-cloud reference. Refuse an unknown
        # configuration rather than silently moving a working point cloud.
        for key in ("sensor_frame", "point_cloud_frame"):
            if driver.get(key, "lidar3d_0_sensor_link") != "lidar3d_0_sensor_link":
                raise ValueError(f"Unexpected {key}; recheck the actual TF before applying")
        desired.update(SCAN_FRAME_PARAMETERS)
    changes = []
    for name, value in desired.items():
        changes.append(("ouster_driver." + name, driver.get(name, "<default>"), value))
        driver[name] = value
    parameters["ouster_driver"] = driver
    sensor["ros_parameters"] = parameters
    result = copy.deepcopy(original)
    result["sensors"] = dict(result["sensors"])
    result["sensors"]["lidar3d"] = list(result["sensors"]["lidar3d"])
    result["sensors"]["lidar3d"][0] = sensor
    return result, changes


def configure(path, apply=False, fix_scan_frame=False):
    path = path.resolve(strict=True)
    before = path.read_text(encoding="utf-8")
    original = yaml.load(before, Loader=UniqueKeyLoader)
    updated, changes = updated_config(original, fix_scan_frame)
    for key, old, new in changes:
        print(f"{key}: {old!r} -> {new!r}")
    if updated == original:
        print("Already configured; no file changed.")
        return
    if not apply:
        print("Preview only. Use --apply on the parked robot with Nav2/SLAM stopped.")
        return
    # Only the requested driver parameters change. YAML formatting/comments
    # are rewritten; the exact original is retained in the adjacent backup.
    rendered = yaml.safe_dump(updated, sort_keys=False, allow_unicode=True)
    if yaml.safe_load(rendered) != updated:
        raise ValueError("YAML serialization changed configuration values")
    if path.read_text(encoding="utf-8") != before:
        raise ValueError("robot.yaml changed during preparation; retry after review")
    file_stat = path.stat()
    fd, backup_name = tempfile.mkstemp(prefix=path.name + ".lidar-backup.", dir=path.parent)
    os.close(fd)
    shutil.copy2(path, backup_name)
    print(f"Backup: {backup_name}")
    fd, temp_name = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            if os.geteuid() == 0:
                os.fchown(output.fileno(), file_stat.st_uid, file_stat.st_gid)
            os.fchmod(output.fileno(), stat.S_IMODE(file_stat.st_mode))
            output.write(rendered)
            output.flush()
            os.fsync(output.fileno())
        if path.read_text(encoding="utf-8") != before:
            raise ValueError("robot.yaml changed before replacement; nothing overwritten")
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    print(f"Updated: {path}")
    print("Clearpath may restart automatically after detecting the YAML change.")
    print("Verify scan_ring=15 after the driver restarts. Installation xyz/rpy were preserved.")
    if fix_scan_frame:
        print("Verify scan frame=lidar3d_0_laser and sensor-to-lidar TF, then compare against the point cloud.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-yaml", type=Path, default=Path("/etc/clearpath/robot.yaml"))
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--fix-scan-frame", action="store_true",
                        help="Separate scan and sensor frames and enable the driver's calibrated TF")
    args = parser.parse_args()
    try:
        configure(args.robot_yaml, args.apply, args.fix_scan_frame)
    except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError) as error:
        parser.exit(1, f"Not applied: {error}\n")


if __name__ == "__main__":
    main()
