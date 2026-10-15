# Jackal J100 + TurtleBot 4 + Host: ROS 2 Multi-Robot Command Manual

> **Everyday entry point:** [Jackal origin reset, SLAM, Nav2, Host RViz, and rosbag manual](Jackal_Startup_SLAM_Nav2_RViz_Rosbag_Manual_2026-10-12.md). It follows execution order and includes the speed-limit readbacks supplied on 2026-10-12 and the updated recording path. This page retains earlier baselines and tuning history; historical "not deployed" notes do not describe later runtime states.

> **Version:** 2026-10-09, based on actual terminal output and network records from that date.<br>
> **ROS**: Ubuntu 24.04 / ROS 2 Jazzy / Fast DDS (`rmw_fastrtps_cpp`)<br>
> **Objective:** distinguish **Jackal / TurtleBot / Host** command locations, architecture, startup, and checks to avoid confusing robot topics or motion controls.
>
> **Legend:** ✅ Verified configuration or measurement at that time; 🔍 read-only diagnostic; ⚠️ unverified, conditional, or planned. **Run every command on its designated device.**

## Contents

1. [Current network and system architecture](#1-current-network-and-system-architecture)
2. [ROS 2 namespaces, topics, and motion control](#2-ros-2-namespaces-topics-and-motion-control)
3. [Jackal: `robot@cpr-j100-0949`](#3-jackal-robotcpr-j100-0949)
4. [TurtleBot 4: `ubuntu@192.168.185.101`](#4-turtlebot-4-ubuntu192168185101)
5. [Host: Alienware](#5-host-alienware)
6. [Daily startup, verification, and shutdown](#6-daily-startup-verification-and-shutdown)
7. [Common troubleshooting](#7-common-troubleshooting)
8. [Configuration files and change principles](#8-configuration-files-and-change-principles)

---

## 1. Current network and system architecture

### 1.1 Roles of the three machines

| Machine | Current role | ROS 2 programs/devices | Key address |
|---|---|---|---|
| **Host (Alienware)** | SSH, network tests, subscriptions to both robots, RViz, recording | ROS 2 Jazzy, Fast DDS; **does not currently run Jackal SLAM/Nav2 computation** | `enp132s0 = 192.168.185.2/24` ✅ |
| **TurtleBot 4 (Raspberry Pi)** | Create 3 chassis and LiDAR/camera publication, independent chassis control | Create 3, RPLIDAR, OAK-D; topics currently mostly in root namespace `/` | `eth0 = 192.168.185.101/24` ✅ |
| **Jackal J100 (onboard computer)** | Chassis/sensors, **SLAM Toolbox + Nav2/MPPI running onboard** | Clearpath Platform, Ouster OS1, RealSense D435, SLAM, Nav2 | `br0 = 192.168.185.201/24` ✅ |

### 1.2 Logical topology based on observed IP / ROS facts

```text
                              ROS 2 Domain ID = 0
                        Fast DDS / 192.168.185.0/24

          ┌──────────────────── LAN ──────────────────────┐
          │                       │                       │
          ▼                       ▼                       ▼
  HOST / Alienware       TurtleBot 4                Jackal J100
  enp132s0: .185.2       Pi eth0: .185.101           PC br0: .185.201
  ├── SSH               ├── ROS /odom, /scan        ├── /j100_0949/*
  ├── RViz              ├── RPLIDAR + OAK-D         ├── Ouster + D435
  └── ros2 bag          ├── usb0: .186.3            ├── SLAM Toolbox
                        │   └── Create 3           ├── Nav2 + MPPI
                        │       (internal USB)     └── br0: .131.1
                        └── wlan0: campus Wi-Fi        └── Sensor subnet

  Shared ROS network: 192.168.185.0/24
  TurtleBot internal: 192.168.186.0/24 (Create 3; do not change to .185)
  Jackal internal: 192.168.131.0/24 (sensors; do not change to .185)
```

**Physical wiring note:** historical plans mentioned TP-Link secondary routers and a NETGEAR GS108Tv3 switch. **Switch ports, management IP, VLANs, and gateway hardware had not all been measured at this stage; a plan is not verified wiring.** TurtleBot Netplan referenced gateway `192.168.185.1`, which alone did not verify a particular device at that address. Previously planned `.100/.200` routers and `.102/.103/.202/.203` RFSoC/Pi-Radio devices were **not verified online at this stage**.

### 1.3 Shared ROS 2 communication settings

| Setting | Host | TurtleBot 4 | Jackal J100 |
|---|---|---|---|
| ROS 2 | Jazzy ✅ | Jazzy ✅ | Jazzy ✅ |
| `ROS_DOMAIN_ID` | `0` ✅ | `0` ✅ | `0` ✅ |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` ✅ | Same ✅ | Same ✅ |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` (target setting) | `SUBNET` ✅ | `SUBNET` ✅ |
| XML | No confirmed custom Host XML | `/etc/turtlebot4/fastdds_rpi.xml` ✅ | `/home/robot/fastdds_185.xml` ✅ |
| Main ROS topic namespace | Application-specific | `/` | `/j100_0949` |

Jackal's Fast DDS XML selects communication interface `192.168.185.201`, avoiding discovery solely through `192.168.131.1` that hid topics from the Host. Relevant systemd services already have environment overrides. Do not clear the Fast DDS configuration without understanding the effects.

---

## 2. ROS 2 namespaces, topics, and motion control

### 2.1 Keep TurtleBot and Jackal topics separate

| Data | TurtleBot 4 | Jackal J100 |
|---|---|---|
| Odometry | `/odom` | `/j100_0949/platform/odom` |
| 2D LiDAR | `/scan` | `/j100_0949/sensors/lidar3d_0/scan` |
| 3D LiDAR | No corresponding topic confirmed | `/j100_0949/sensors/lidar3d_0/points` |
| Motion velocity commands | `/cmd_vel` (TurtleBot chain) | `/j100_0949/cmd_vel` (Jackal external command input) |
| TF | `/tf`, `/tf_static` | `/j100_0949/tf`, `/j100_0949/tf_static` |
| Map | Depends on TurtleBot SLAM being active | `/j100_0949/map` (while SLAM runs) |
| RGB camera | `/oakd/rgb/preview/image_raw` (verify publication first) | `/j100_0949/sensors/camera_0/color/image` |
| Depth camera | Query actual OAK-D topic names with `ros2 topic list` | `/j100_0949/sensors/camera_0/depth/image` |

> **Important:** both robots' TF messages may contain frame IDs such as `odom` / `base_link`. **Topic namespaces do not automatically make frame IDs unique.** For Host RViz/TF analysis, subscribe separately to the appropriate `/tf` / `/tf_static`; do not merge both TF trees directly.

### 2.2 Verified Jackal motion-control chain

```text
Jackal SLAM Toolbox ──────► /j100_0949/map + /j100_0949/tf
                                      │
                              Nav2 Planner
                                      │
                              MPPI Controller
                                      │
                          /j100_0949/cmd_vel_nav
                                      │
                              Velocity Smoother
                                      │
                        /j100_0949/cmd_vel_smoothed
                                      │
                              Collision Monitor
                                      │
                              /j100_0949/cmd_vel ◄── Docking Server
                                      │
                           Twist Mux / command priorities
                                      │
                         /j100_0949/platform/cmd_vel
                                      │
                         platform_velocity_controller
                             Final normal software limits
                                      │
                               Jackal motors
```

- **Normal motion commands** ultimately pass through `platform_velocity_controller` limits.
- Current Twist Mux priorities: `rc=12`, `joy=10`, `interactive_marker=8`, `external(cmd_vel)=1`; `safety_stop=254` and `emergency_stop=255` are lock inputs.
- Docking Server can publish directly to `/j100_0949/cmd_vel`, **bypassing Nav2 Velocity Smoother / Collision Monitor but still passing through Twist Mux and the chassis controller**. Collision Monitor is therefore not a common safety barrier for every motion source.
- **Physical Motion Stop / emergency stop** is an independent safety mechanism; software limits do not guarantee physical speed. Before testing, ensure the robot is stationary, the area is clear, and an immediate stop is available.
- Root `/cmd_vel` already has TurtleBot `teleop_twist_joy_node` and `create3_repub`; **do not assume it controls Jackal**.

---

## 3. Jackal: `robot@cpr-j100-0949`

**Command location: Jackal's onboard computer.** From the Host, first:

```bash
ssh robot@192.168.185.201
```

### J-1. Initialize the ROS environment and perform read-only checks

```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash

hostname
ip -br -4 addr                  # br0 should contain both .185.201 and .131.1
ip -4 route
printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

ros2 node list | sort
ros2 topic list | grep '^/j100_0949/' | head -60
```

> Measured environment that day: `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. If output differs, inspect shell startup scripts before overwriting the existing XML configuration.

### J-2. Clearpath services and hardware state: read-only

```bash
systemctl is-active \
  clearpath-robot.service \
  clearpath-platform.service \
  clearpath-sensors.service

systemctl --no-pager --full status clearpath-platform.service
systemctl --no-pager --full status clearpath-sensors.service

ros2 topic info -v /j100_0949/platform/odom
ros2 topic info -v /j100_0949/sensors/lidar3d_0/scan
ros2 topic info -v /j100_0949/sensors/lidar3d_0/points

# Inspect rate; stop with Ctrl+C
ros2 topic hz /j100_0949/platform/odom
ros2 topic hz /j100_0949/sensors/lidar3d_0/scan
```

### J-2a. Ouster ring 15 and scan-frame correction (2026-10-12)

Driver readback confirmed `scan_ring=15`; on this OS-1-32-U1-SR its elevation is `+0.32°`, versus `+20.64°` for original ring 0. A physical-object check in `base_link` confirmed correct PointCloud2 front/back directions. Live data with matching timestamps showed that original `/scan` and `/points` shared `lidar3d_0_sensor_link`, but matched only after a 180° comparison rotation. Retain installation `xyz/rpy` and correct scan coordinates through a separate lidar frame.

At this stage the frame correction had not been deployed. By default the script still selects only ring 15; `--fix-scan-frame` additionally sets these driver parameters:

```yaml
scan_ring: 15
sensor_frame: lidar3d_0_sensor_link
point_cloud_frame: lidar3d_0_sensor_link
lidar_frame: lidar3d_0_laser
imu_frame: lidar3d_0_imu
pub_static_tf: true
```

The Ouster driver publishes sensor→lidar and sensor→imu transforms from metadata. Local metadata gives sensor→lidar a 180° yaw and `0.038195 m` Z translation. This is an internal sensor-frame transform, not a mounting-yaw change. A separate IMU frame avoids self-referencing transforms when driver TF is enabled. Sources: [Ouster TF implementation](https://github.com/ouster-lidar/ouster-ros/blob/ros2/ouster-ros/src/os_static_transforms_broadcaster.h), [scan/point-cloud frame usage](https://github.com/ouster-lidar/ouster-ros/blob/ros2/ouster-ros/src/os_driver_node.cpp).

Cancel the goal, stop the robot, and stop Nav2/SLAM first. Clearpath may automatically restart chassis and sensors after detecting a `robot.yaml` change.

Upload the updated script from the Host:

```bash
scp /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/configure_jackal_lidar.py \
  robot@192.168.185.201:/home/robot/
```

Run on Jackal; other settings are preserved and a backup retains original comments/formatting:

```bash
sudo python3 ~/configure_jackal_lidar.py --apply --fix-scan-frame &&
sudo systemctl restart clearpath-robot.service
```

After restart, verify before sending navigation goals:

```bash
source /etc/clearpath/setup.bash
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver scan_ring
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver lidar_frame
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver pub_static_tf
ros2 topic echo /j100_0949/sensors/lidar3d_0/scan \
  --once --field header --qos-reliability best_effort
ros2 run tf2_ros tf2_echo lidar3d_0_sensor_link lidar3d_0_laser --ros-args \
  -r /tf:=/j100_0949/tf -r /tf_static:=/j100_0949/tf_static
```

Expect `15`, `lidar3d_0_laser`, and `true`; scan `header.frame_id` should also be `lidar3d_0_laser`. TF should include approximately 180° yaw; press Ctrl+C afterward. Set RViz Fixed Frame to `base_link`, LaserScan to `/j100_0949/sensors/lidar3d_0/scan`, and PointCloud2 to `/j100_0949/sensors/lidar3d_0/points`. Cardboard physically behind the robot should appear at negative X in both; moving it front-left should put it at positive X/Y. Compare position/outline only: LaserScan uses ring 15; PointCloud2 includes all beams.

RViz previously subscribed to publisher-less `/j100_0949/sensors/lidar2d_0/scan`, which cannot establish live direction. After correction checks pass, start a new map rather than retaining a session built with incorrect scan coordinates. To roll back, stop the robot and Nav2/SLAM, restore the exact `robot.yaml.lidar-backup.*` file printed by the script, then restart Clearpath; do not guess the backup filename.

### J-3. Start SLAM: terminal J-A, keep running

> **Run only if this SLAM instance is not already active.** Do not start a second `slam.launch.py` while it runs.

```bash
source /etc/clearpath/setup.bash

ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

Verify in another Jackal terminal, read-only:

```bash
source /etc/clearpath/setup.bash
ros2 node list | grep slam_toolbox
ros2 topic info /j100_0949/map
ros2 topic hz /j100_0949/map
```

### J-3a. Use the current location and heading as a new map start (2026-10-12)

For **discarding the current mapping session and starting a new map**. The aim is `base_link` at approximately `x/y/yaw = 0/0/0`, with the robot front defining map `+X`. OccupancyGrid `info.origin` need not be zero: it defines the grid-image boundary and usually shifts as the map grows.

Read-only inspection found Jackal EKF using `world_frame=odom`, fusing `platform/odom` `x/y/yaw` with `odom0_relative=false`; IMU mainly supplies angular velocity and acceleration. A zero pose sent only to `/j100_0949/set_pose` may be pulled back by subsequent absolute wheel odometry. A fresh SLAM Toolbox session uses received odometry pose for its first scan; `/slam_toolbox/reset` or RViz `2D Pose Estimate` alone does not guarantee the current pose becomes the new map origin. See [SLAM Toolbox Jazzy](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp).

1. Park at the new start with the desired `+X` heading and cancel navigation. Save the grid map and pose graph before exiting SLAM if needed; restarting SLAM discards an unsaved session.
2. Exit the old Nav2 launch, then the old SLAM launch, and wait for nodes to exit. Remain stationary with no remote-control input throughout initialization.
3. Restart the platform localization service on **Jackal**, then check initialization. This restarts only the platform service, not the OS.

```bash
source /etc/clearpath/setup.bash
sudo systemctl restart clearpath-platform.service
systemctl is-active clearpath-platform.service
ros2 topic echo /j100_0949/platform/odom --once --field pose.pose
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/j100_0949/tf -r /tf_static:=/j100_0949/tf_static
```

`tf2_echo` translation `x/y` and RPY yaw should be near zero. Observe for several stationary seconds, then press `Ctrl+C`. This is a prerequisite for mapping; service `active` does not establish successful zeroing. If a substantial offset or jump remains, inspect raw/filtered odometry and the actual onboard platform launch. Do not start a new map with an unresolved offset or repeatedly send `set_pose`. One successful zeroing was confirmed by user field output on 2026-10-12: raw odometry `x≈0.000006 m`, `y≈0`, TF translation near zero, yaw approximately `0.009–0.010°`, stable for several seconds. Recheck on every later attempt.

4. Remain stationary and start fresh SLAM using J-3 without loading an old pose graph. Query `ros2 param get /j100_0949/slam_toolbox map_file_name` to check that no old graph is configured; an unset parameter is normal. Then inspect the new map pose:

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/j100_0949/tf -r /tf_static:=/j100_0949/tf_static
```

After confirming new-session `x/y/yaw` near zero, press `Ctrl+C` and start Nav2 using J-4. Use RViz Fixed Frame `map` and Jackal-namespaced TF topics. Send new navigation goals; old-session goal coordinates no longer apply.

To preserve the old map, run this on Jackal during step 1 while SLAM is still active:

```bash
jackal_map_dir="$HOME/maps/before_new_origin_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_map_dir"
ros2 run nav2_map_server map_saver_cli -f "$jackal_map_dir/map" --ros-args \
  -r map:=/j100_0949/map
ros2 service call /j100_0949/slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$jackal_map_dir/posegraph'}"
```

Confirm successful saving (serialization `result: 0`) before exiting SLAM. The grid map supports display/localization; continuing SLAM also requires the serialized pose graph.

### J-3b. Specify a new map start without restarting the chassis

To assign the current pose, for example `(x=1 m, y=0 m, yaw=0°)`, use [`start_jackal_slam_at_pose.py`](../scripts/start_jackal_slam_at_pose.py). Run after exiting old Nav2/SLAM. It retains chassis odometry, requires neither the J-3a platform-service restart nor a simultaneous standard J-3 SLAM launch.

Follow [operating manual section 9](Jackal_Startup_SLAM_Nav2_RViz_Rosbag_Manual_2026-10-12.md#9-start-mapping-at-a-specified-pose-without-restarting-the-chassis) for upload/startup/checks. The fix addresses indefinite waiting after old `change_state` timeouts and duplicate shutdown. Wait for `SLAM ACTIVE`, verify `map → base_link` near `(1,0,0°)`, then start Nav2 using J-4. The new version passed 10 regression checks and a local isolated SLAM test; the rerun on hardware remained unconfirmed at this stage.

### J-4. Start custom low-speed Nav2: terminal J-B, keep running

> **Use the custom launch, not default `clearpath_nav2_demos nav2.launch.py`.** Confirm old Nav2 has stopped; do not run two controller stacks simultaneously.

```bash
source /etc/clearpath/setup.bash

ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

Read-only verification:

```bash
for node in controller_server velocity_smoother collision_monitor bt_navigator; do
  echo "===== $node ====="
  ros2 lifecycle get "/j100_0949/$node"
done

# Normally active [3]
ps -eo pid,args | grep '[r]os2 launch'
```

### J-5. Active low-speed parameters and configuration files

Confirmed persistent **chassis-layer** settings:

| Control item | Current parameter |
|---|---|
| Forward/reverse | `linear.x.max_velocity=0.3`, `min_velocity=-0.2` m/s |
| Forward/reverse acceleration | `linear.x.max_acceleration=0.3`, `max_acceleration_reverse=-0.3` m/s² |
| Turning | `angular.z.max_velocity=0.6`, `min_velocity=-0.6` rad/s |
| Angular acceleration | `angular.z.max_acceleration=0.6`, `max_acceleration_reverse=-0.6` rad/s² |
| Original deceleration limits retained | Settings such as `max_deceleration=-4.0` for linear/angular axes |

Confirmed **Nav2-layer** settings in the persistent file read by the custom launch:

| Component | Configuration |
|---|---|
| MPPI | `vx_max=0.3`, `vx_min=-0.2`, `wz_max=0.6` |
| MPPI acceleration / negative acceleration | `ax_max=0.3`, `ax_min=-3.0`, `az_max=0.6` |
| Velocity Smoother | `max_velocity=[0.3,0,0.6]`, `min_velocity=[-0.2,0,-0.6]` |
| Velocity Smoother | `max_accel=[0.3,0,0.6]`, `max_decel=[-4,0,-4]` |

> `FollowPath.ax_min=-3.0` is MPPI's negative acceleration bound, **not maximum reverse speed**. These are indoor low-speed test values, not manufacturer-certified safety thresholds.

Verify runtime values:

```bash
C=/j100_0949/controller_server
S=/j100_0949/velocity_smoother
B=/j100_0949/platform_velocity_controller

for p in FollowPath.vx_max FollowPath.vx_min FollowPath.wz_max \
         FollowPath.ax_max FollowPath.ax_min FollowPath.az_max; do
  printf 'MPPI %-24s ' "$p"; ros2 param get "$C" "$p"
done
for p in max_velocity min_velocity max_accel max_decel; do
  printf 'Smoother %-20s ' "$p"; ros2 param get "$S" "$p"
done
for p in linear.x.max_velocity linear.x.min_velocity \
         linear.x.max_acceleration angular.z.max_velocity \
         angular.z.max_acceleration; do
  printf 'Base %-24s ' "$p"; ros2 param get "$B" "$p"
done
```

Configuration locations; **do not overwrite blindly**:

```text
/etc/clearpath/robot.yaml                          # Persistent shared chassis speed limits
/home/robot/jackal_nav2/nav2_low_speed.yaml        # Custom Nav2 parameters
/home/robot/jackal_nav2/nav2_low_speed.launch.py   # Custom Nav2 launch
/home/robot/fastdds_185.xml                        # Jackal DDS interface binding
/etc/netplan/50-cloud-init.yaml                    # Jackal networking: two br0 IPs
/etc/clearpath/platform/config/twist_mux.yaml     # Command arbitration priorities
```

Verify that the launch actually references the custom YAML:

```bash
grep -n -A 2 'file_parameters =' \
  "$HOME/jackal_nav2/nav2_low_speed.launch.py"
python3 -m py_compile "$HOME/jackal_nav2/nav2_low_speed.launch.py"
ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" --show-args
```

### J-5a. Fast turning and jerky forward motion: candidate configuration (2026-10-12)

The user reported these issues during Jackal RViz-goal navigation. The repository candidate [`config/jackal/nav2_low_speed.yaml`](../config/jackal/nav2_low_speed.yaml) was derived from the user's pasted onboard YAML. **J-5 retains the early runtime baseline. Later on 2026-10-12, the user confirmed `wz_max=0.3`, `ax_min=-0.3`, smoother `max_decel=[-0.3,0,-0.3]`, `50 Hz`, recovery spin limit `0.15`, and the single-goal slow-recovery BT path; full motion performance remained unverified.** At the earlier Host inspection, chassis limits were still `0.3 m/s` forward and `0.6 rad/s` angular. No Nav2 nodes were discovered from the Host, passwordless SSH failed, and no motion data was obtained.

| Parameter | User-supplied configuration | Candidate value |
|---|---|---|
| MPPI `wz_max` / `az_max` | `0.6` / `0.6` | `0.2` rad/s / `0.3` rad/s² |
| MPPI `ax_min` | `-3.0` | `-0.3` m/s² |
| MPPI `visualize` | `true` | `false` |
| Progress checker `movement_time_allowance` | `10.0` | `20.0` s |
| Behavior server `max_rotational_vel` / `min_rotational_vel` | `1.0` / `0.2` | `0.15` / `0.05` rad/s |
| Behavior server `rotational_acc_lim` | `0.5` | `0.15` rad/s² |
| Velocity smoother `smoothing_frequency` | `20.0` | `50.0` Hz |
| Velocity smoother `max_velocity` / `min_velocity` | `[0.3,0,0.6]` / `[-0.2,0,-0.6]` | `[0.3,0,0.2]` / `[-0.2,0,-0.2]` |
| Velocity smoother `max_accel` / `max_decel` | `[0.3,0,0.6]` / `[-4,0,-4]` | `[0.3,0,0.3]` / `[-0.3,0,-0.3]` |

**Subsequent single adjustment (2026-10-12):** at the user's request to reduce turning speed first, the repository candidate lowers MPPI `wz_max` from `0.3` to `0.2 rad/s` and both smoother angular bounds to `±0.2 rad/s`. Angular acceleration/deceleration, linear speed, and recovery settings remain unchanged. At this stage `0.2` had not been deployed or field-verified; `0.3` above was the previously confirmed readback. Temporary runtime commands are in [startup manual section 4.4](Jackal_Startup_SLAM_Nav2_RViz_Rosbag_Manual_2026-10-12.md#44-first-compare-a-lower-turning-speed).

These are initial tuning values; YAML alone cannot establish the cause of jerky motion. Decelerating from `0.3 m/s` at `4 m/s²` ideally takes only approximately `0.075 s`. The current candidate lowers normal linear/angular deceleration magnitudes from the first candidate's `0.6` to `0.3 m/s²` and `0.3 rad/s²`. An ideal stop from `0.3 m/s` takes approximately `1.0 s` over `0.15 m`; verify stopping in open space. Sampling, obstacle avoidance, and chassis limits also affect actual timing. Large turns take longer at low angular speed, so the displacement progress allowance is extended; this also delays detecting a truly stuck robot.

MPPI remains at `20 Hz` / `model_dt=0.05 s`; the smoother interpolates at a higher frequency. Disabling MPPI trajectory visualization reduces extra computation, but logs are needed to establish control overruns. See [Nav2 MPPI](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) and [Velocity Smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/). The candidate retains Collision Monitor, sensor timeouts, footprint, costmaps, critic weights, and message types. Chassis `/etc/clearpath/robot.yaml` is outside this change.

**Live check after the report of abrupt deceleration (2026-10-12):** onboard Nav2 later responded with `FollowPath.ax_min=-3.0`, `FollowPath.az_max=0.6`, smoother `max_decel=[-4,0,-4]`, `smoothing_frequency=20.0`, and `max_velocity=[0.3,0,0.6]`; chassis linear/angular deceleration remained `-4.0`. These values show the consolidated configuration was not running then, rather than proving it had been tested and remained abrupt. Only the repository candidate was updated; robot runtime parameters were not changed online.

Check normal deceleration separately from obstacle-triggered stopping. The `1.0 s` above is an ideal smoother value in the normal command chain, not a guarantee for cancellation, message timeout, or Collision Monitor intervention. For abrupt stops during avoidance or command interruption, compare velocity stages, timestamps, Collision Monitor state, and logs. Existing avoidance and command-timeout settings are retained. After deployment, read back `ax_min=-0.3` and `max_decel=[-0.3,0,-0.3]`, then test normal deceleration and goal-cancellation stops in open space.

**Unified configuration:** repository and onboard launch both use `nav2_low_speed.yaml`. It already combines the original configuration, smoothing, and recovery limits. Do not load a second YAML or concatenate files into duplicate node keys. This YAML still references the two recovery XML files.

**Deployment:** cancel the goal, confirm the robot is stationary, press `Ctrl+C` in the original Nav2 launch terminal, and wait for exit. SLAM may remain running for tuning only; for an origin reset, also exit SLAM as in J-3a. Both candidate YAML and `behavior_trees/` are needed. Upload overwrites matching filenames, so **back up on Jackal before uploading from the Host**. Record the backup path and resolve any error before continuing.

Back up current YAML and existing custom behavior trees on Jackal:

```bash
(
  set -e
  cd "$HOME/jackal_nav2"
  jackal_nav2_backup=$(mktemp -d "$PWD/nav2_backup.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_nav2_backup/"
  if [ -d behavior_trees ]; then
    cp -a behavior_trees "$jackal_nav2_backup/"
  fi
  printf 'Backup directory: %s\n' "$jackal_nav2_backup"
)
```

After backup succeeds, transfer the consolidated YAML and behavior trees from the Host:

```bash
scp -r /home/nyu/Desktop/Jackal_Turtlebot_Platform/config/jackal/nav2_low_speed.yaml \
  /home/nyu/Desktop/Jackal_Turtlebot_Platform/config/jackal/behavior_trees \
  robot@192.168.185.201:/home/robot/jackal_nav2/
```

After upload, onboard `~/jackal_nav2/nav2_low_speed.yaml` is the consolidated version. The custom launch reads the same path, so no additional candidate-to-active copy is needed.

Restart Nav2 with the J-4 custom launch, retaining `scan_topic:=/j100_0949/sensors/lidar3d_0/scan`. Candidate `sensors/lidar2d_0/scan` values came from the original YAML; Clearpath's standard launch rewrites them using `scan_topic`. Verify what the custom launch actually runs.

Check in another Jackal terminal with `/etc/clearpath/setup.bash` loaded:

```bash
ros2 param get /j100_0949/controller_server FollowPath.wz_max
ros2 param get /j100_0949/controller_server FollowPath.ax_min
ros2 param get /j100_0949/controller_server FollowPath.visualize
ros2 param get /j100_0949/velocity_smoother smoothing_frequency
ros2 param get /j100_0949/velocity_smoother max_velocity
ros2 param get /j100_0949/velocity_smoother min_velocity
ros2 param get /j100_0949/velocity_smoother max_decel
ros2 param get /j100_0949/behavior_server max_rotational_vel
ros2 param get /j100_0949/collision_monitor scan.topic
ros2 param get /j100_0949/local_costmap/local_costmap voxel_layer.scan.topic
ros2 param get /j100_0949/global_costmap/global_costmap obstacle_layer.scan.topic
ros2 node info /j100_0949/controller_server
```

The first three outputs should be `0.2`, `-0.3`, and `false`; smoother outputs should be `50.0`, `[0.3,0.0,0.2]`, `[-0.2,0.0,-0.2]`, and `[-0.3,0.0,-0.3]`. Recovery angular limit should be `0.15`. All three scan parameters should point to `/j100_0949/sensors/lidar3d_0/scan`. Verify the controller subscribes to an actually published Jackal odometry topic using `ros2 topic info -v <ACTUAL_SUBSCRIBED_TOPIC>`.

First verify short straight motion, turning, and cancellation stops in open space. If motion remains jerky, record a reproduction on **Jackal** and save Nav2 launch logs:

```bash
ros2 bag record -o "$HOME/jackal_nav2/stutter_$(date +%Y%m%d_%H%M%S)" \
  /j100_0949/cmd_vel_nav /j100_0949/cmd_vel_smoothed \
  /j100_0949/cmd_vel /j100_0949/platform/cmd_vel \
  /j100_0949/platform/cmd_vel_out /j100_0949/platform/odom \
  /j100_0949/platform/odom/filtered /j100_0949/collision_monitor_state \
  /rosout
```

Find the first stage showing zero values, gaps, or jumps. If `cmd_vel_nav` is already intermittent, inspect control timing, TF/odometry, and the local costmap. If smoother output is continuous but `cmd_vel` drops, inspect Collision Monitor state and sensor timeouts. If only downstream `platform/cmd_vel` changes, inspect Twist Mux and input timeouts. Evaluate compute load when logs show `Control loop missed`. Do not conceal stalls by disabling collision avoidance or extending command timeouts.

**Rollback:** cancel the goal, stop the robot and Nav2, then copy `nav2_low_speed.yaml` from the printed backup directory to `~/jackal_nav2/nav2_low_speed.yaml`. If the backup includes `behavior_trees/`, restore the corresponding XML files too. Restart using J-4; do not choose an ambiguous backup through wildcards.

### J-5b. Separate recovery speed limits: added 2026-10-12

The first candidate only lowered recovery spin to `0.3 rad/s` without changing the default BT backup request. Following additional user feedback, the current candidate uses:

| Action | Current candidate setting | Location |
|---|---|---|
| Recovery spin | Maximum `0.15 rad/s`, minimum `0.05 rad/s`, angular acceleration `0.15 rad/s²` | YAML `behavior_server.ros__parameters` |
| Recovery backup | `backup_speed="0.05"` m/s; distance remains `0.30 m` | `BackUp` in both custom BTs |
| Recovery time allowance | `Spin` 30 s; `BackUp` 20 s | `time_allowance` in both custom BTs |

Backup speed comes from the BT action goal; changing MPPI `vx_max` / `vx_min` alone does not set it. Jazzy `BackUp` has no generic `max_linear_vel` parameter. A 1.57 rad spin at 0.15 rad/s takes approximately 10.5 s at constant speed alone, so action allowances are also increased to avoid the default 10 s limit after slowing. See [Nav2 Behavior Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/). Sensor and command-loss timeouts are unchanged.

Files were copied from local Nav2 1.3.12 default BTs, changing only backup speed and the two recovery time allowances:

- [`navigate_to_pose_slow_recovery.xml`](../config/jackal/behavior_trees/navigate_to_pose_slow_recovery.xml): single-goal navigation.
- [`navigate_through_poses_slow_recovery.xml`](../config/jackal/behavior_trees/navigate_through_poses_slow_recovery.xml): multi-goal navigation.

YAML `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` explicitly reference the corresponding files under `/home/robot/jackal_nav2/behavior_trees/`. Upload YAML and XML together as in J-5a and restart Nav2; YAML alone leaves missing BT files. An action client's explicit `behavior_tree` overrides these defaults, so inspect the goal request if needed.

After restarting, verify actual loading on Jackal:

```bash
ros2 param get /j100_0949/behavior_server max_rotational_vel
ros2 param get /j100_0949/behavior_server min_rotational_vel
ros2 param get /j100_0949/behavior_server rotational_acc_lim
ros2 param get /j100_0949/bt_navigator default_nav_to_pose_bt_xml
ros2 param get /j100_0949/bt_navigator default_nav_through_poses_bt_xml
grep -nE '<Spin |<BackUp ' "$HOME/jackal_nav2/behavior_trees/"*.xml
```

The first three values should be `0.15`, `0.05`, and `0.15`. Target files should be the custom XMLs above with `backup_speed="0.05"`. Spin reads limits during configuration, so fully exit/restart Nav2 for this update; a successful online `ros2 param set` alone does not establish updated behavior.

**Verification scope at this stage:** only offline YAML, XML, Nav2 parameter parsing, and action-port checks were performed. Read-only requests received no response from onboard Nav2, so neither prior candidate activation nor recovery behavior was verified. If recovery remains fast after deployment, compare `cmd_vel_nav`, `cmd_vel_smoothed`, `cmd_vel`, and `platform/cmd_vel_out`, and inspect publishers, loaded files, and possible duplicate Nav2 stacks. If rollback YAML references an older custom XML version, restore that BT directory too.

### J-6. Command sources, limited output, and body odometry: read-only

```bash
ros2 topic info -v /j100_0949/cmd_vel_nav
ros2 topic info -v /j100_0949/cmd_vel_smoothed
ros2 topic info -v /j100_0949/cmd_vel
ros2 topic info -v /j100_0949/platform/cmd_vel
ros2 topic info -v /j100_0949/platform/cmd_vel_out

ros2 node info /j100_0949/platform_velocity_controller

# Observe actual odometry; zero velocity is normal while stationary
ros2 topic echo /j100_0949/platform/odom --field twist.twist

# Observe limited commands; output may not be continuous without new commands
ros2 topic echo /j100_0949/platform/cmd_vel_out --field twist
```

### J-7. Logs and configuration verification: read-only

```bash
sudo journalctl -u clearpath-robot.service \
  -u clearpath-platform.service \
  -u clearpath-sensors.service -n 100 --no-pager

systemctl show clearpath-platform.service -p Environment --no-pager
systemctl show clearpath-sensors.service -p Environment --no-pager

sed -n '1,160p' /etc/netplan/50-cloud-init.yaml
sed -n '1,160p' /etc/clearpath/platform/config/twist_mux.yaml
```

### J-8. Stop a test normally

1. Cancel the RViz/Nav2 goal and confirm the robot is stationary; keep Motion Stop available.
2. Press `Ctrl+C` in **J-B (Nav2 launch)** and wait for nodes to exit.
3. Press `Ctrl+C` in **J-A (SLAM launch)**, or retain SLAM if continuing mapping.
4. There is **no need to stop** system `clearpath-platform` / `clearpath-sensors` services.
5. Avoid `kill -9`, arbitrary nonzero motion-topic commands, or directly changing Netplan addresses over SSH.

---

## 4. TurtleBot 4: `ubuntu@192.168.185.101`

**Command location: TurtleBot 4 Raspberry Pi.** Log in from the Host:

```bash
ssh ubuntu@192.168.185.101
```

### T-1. Initialize and check networking

```bash
source /opt/ros/jazzy/setup.bash

hostname
ip -br -4 addr       # eth0=.185.101; usb0=.186.3
ip -4 route
printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

# Read-only inspection of persistent network configuration
sudo sed -n '1,200p' /etc/netplan/40-ethernets.yaml
```

Verified interface roles:

| Interface | Current address | Function |
|---|---|---|
| `eth0` | `192.168.185.101/24` ✅ | Shared Host/Jackal LAN |
| `usb0` | `192.168.186.3/24` ✅ | Raspberry Pi ↔ Create 3 internal communication; **do not change to a 185 address** |
| `wlan0` | `10.33.16.163/23` (at that time) | Campus Wi-Fi; DHCP may change |

`/etc/netplan/40-ethernets.yaml` stores the `eth0` static IP, default route through `.185.1`, and internal `usb0` address; persistence after reboot was verified. Normally **inspect rather than reconfigure**.

### T-2. Check Create 3, LiDAR, and camera topics

```bash
source /opt/ros/jazzy/setup.bash

ros2 node list | sort
ros2 topic list -t | sort

# TurtleBot root namespace, distinct from Jackal /j100_0949/*
ros2 topic info -v /odom
ros2 topic info -v /scan
ros2 topic info -v /cmd_vel
ros2 topic info -v /tf

# Measure publication rate; Ctrl+C to exit
ros2 topic hz /odom
ros2 topic hz /scan

# Print one odometry message only
ros2 topic echo /odom --once

# Check OAK-D against actual running topics; absent cameras may have no topic
ros2 topic list | grep -Ei 'oakd|camera|image|depth'
```

Previous Host measurements: TurtleBot `/odom` approximately **20 Hz**, `/scan` approximately **8.1 Hz**. `/odom` publisher was identified as `create3_repub`. Determine current camera publication from that session's `ros2 topic list` and `topic hz`.

**Note:** no field evidence at this stage established separate onboard TurtleBot SLAM/Nav2, so no unverified TurtleBot launch commands are supplied here. TurtleBot's global `/cmd_vel` and Jackal's `/j100_0949/cmd_vel` target different robots. **Do not directly test nonzero velocity with `ros2 topic pub` on shared Domain 0.**

### T-3. Troubleshoot TurtleBot Fast DDS and networking

```bash
printenv | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_AUTOMATIC_DISCOVERY_RANGE|FASTDDS|FASTRTPS'

# Deployed DDS XML; read-only
sed -n '1,160p' /etc/turtlebot4/fastdds_rpi.xml

# Test unicast connectivity to Host and Jackal
ping -c 3 -W 1 192.168.185.2
ping -c 3 -W 1 192.168.185.201

ip -4 route get 192.168.185.2
ip -4 route get 192.168.185.201
```

> Custom Fast DDS XML and Create 3 `usb0` were working. Unless a fault is specifically isolated there, do not delete XML, change the USB subnet, or run `netplan apply`, which may interrupt SSH.

---

## 5. Host: Alienware

**Command location: a local Host terminal**, for example `nyu@nyu-Alienware-18-Area-51-AA18250`.

### H-1. Quick read-only network checks

```bash
ip -br -4 addr
ip -4 route
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status

# Normal Host wired interface: enp132s0 = 192.168.185.2/24
ip -4 route get 192.168.185.101
ip -4 route get 192.168.185.201

ping -c 3 -W 1 192.168.185.101     # TurtleBot
ping -c 3 -W 1 192.168.185.201     # Jackal
```

If either `route get` avoids `enp132s0`, inspect Host Wi-Fi/Ethernet priorities first. Campus Wi-Fi uses another subnet. **Do not treat the old planned Host address `192.168.185.10` as current**; the measured address is `.2`.

### H-2. SSH into both robots

```bash
ssh ubuntu@192.168.185.101  # TurtleBot 4
ssh robot@192.168.185.201   # Jackal J100
```

### H-3. ROS 2 environment and topic checks for both robots

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0

printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

# Shared Domain 0, separate topic namespaces
ros2 node list | sort
ros2 topic list -t | sort

# TurtleBot
ros2 topic info -v /odom
ros2 topic info -v /scan

# Jackal
ros2 topic info -v /j100_0949/platform/odom
ros2 topic info -v /j100_0949/sensors/lidar3d_0/scan
ros2 topic info -v /j100_0949/map
```

### H-4. Cross-machine data quality: verify actual data, not just topic names

```bash
# Each command measures continuously; Ctrl+C before running the next
ros2 topic hz /odom                                      # TurtleBot: previously approximately 20 Hz
ros2 topic hz /scan                                      # TurtleBot: previously approximately 8.1 Hz
ros2 topic hz /j100_0949/platform/odom                    # Jackal: previously approximately 50 Hz
ros2 topic hz /j100_0949/sensors/lidar3d_0/scan
ros2 topic hz /j100_0949/sensors/camera_0/color/image    # Previously approximately 29.6 FPS
ros2 topic hz /j100_0949/sensors/camera_0/depth/image    # Previously approximately 14–15 FPS
```

Historical rates are references, not guaranteed on every startup. **Topic discovery does not prove continuous data publication**; use `topic hz`, `topic info -v`, and hardware state.

### H-5. RViz visualization: Host only, without a second Nav2 stack

If the Clearpath visualization package is installed, try:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix clearpath_viz

ros2 launch clearpath_viz view_navigation.launch.py \
  namespace:=/j100_0949 \
  use_sim_time:=false
```

To launch only RViz and inspect Jackal's namespaced TF:

```bash
source /opt/ros/jazzy/setup.bash
rviz2 --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

- For **Jackal**, set Map Topic to `/j100_0949/map` and LiDAR to `/j100_0949/sensors/lidar3d_0/scan`. Fixed Frame is usually `map`, provided the corresponding TF chain exists.
- For **TurtleBot**, use global `/scan`, `/odom`, `/tf`, and `/tf_static`; do not mix TurtleBot and Jackal TF subscriptions directly.
- The Host previously had NVIDIA `Driver/library version mismatch` / RViz `GLX` initialization errors. If RViz fails again, check `nvidia-smi` and `glxinfo -B` first. Graphics and ROS connectivity are separate issues.
- **The Host only visualizes; SLAM and Nav2 already run on Jackal. Do not start another navigation stack in the same namespace on the Host.**

### H-6. Optional: record core ROS data from both robots on the Host, using read-only subscriptions

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/rosbags"

ros2 bag record \
  -o "$HOME/rosbags/dual_robot_$(date +%Y%m%d_%H%M%S)" \
  /odom \
  /scan \
  /tf \
  /tf_static \
  /j100_0949/platform/odom \
  /j100_0949/sensors/lidar3d_0/scan \
  /j100_0949/tf \
  /j100_0949/tf_static
```

`Ctrl+C` stops normally and writes metadata; afterward check with `ros2 bag info <bag_directory>`. Full-rate clouds/images are excluded by default to reduce bandwidth and disk load. For `/tf_static` replay, account for static TF QoS.

### H-6a. Manually record Jackal LiDAR, camera, and navigation data on the Host (2026-10-12)

The user selected Host recording with manual start/stop. Use repository `scripts/record_jackal.sh` without restarting SLAM, Nav2, or sensors. The script only subscribes and sends no motion commands.

| Data | Recorded contents |
| --- | --- |
| Ouster | Full-beam `/points`, current-ring `/scan`, LiDAR `metadata`, LiDAR IMU |
| RealSense | Raw RGB/depth images, both `camera_info` and `metadata` streams, `depth_to_color` extrinsics |
| Body state | Raw/processed IMU, chassis/filtered odometry, wheel joint states |
| Coordinates and maps | Jackal `/tf`, `/tf_static`, `map`, `map_metadata`, `pose` |

2026-10-14 update: the default is the original 14 collection topics plus 7 camera topics, **21 total**, all under `/j100_0949`, with no extra option. See the [production recording manual](Jackal_RPP_Trial_2026-10-14.md#8-production-recording-save-on-jackal-then-transfer-to-host) for the full list. Do not use publisher-less `/j100_0949/sensors/lidar2d_0/scan`. `ring 15` only affects LaserScan; recorded Ouster PointCloud2 retains all beams.

Run in a new Host terminal:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/Desktop/jackal_rosbag_repo/recording"
cd "$HOME/Desktop/jackal_rosbag_repo/recording"
df -h .

jackal_bag="$PWD/jackal_$(date +%Y%m%d_%H%M%S)"
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal.sh "$jackal_bag"
```

The script defaults to a new timestamped directory under `/home/nyu/Desktop/jackal_rosbag_repo/recording/`. Startup displays `PAUSED` and waits for Space. The first Space press creates the recorder and subscriptions, preventing one-time static TF, LiDAR metadata, and camera extrinsics from being consumed and discarded under native `--start-paused`. After starting, Space toggles pause/resume; Ctrl+C stops and saves. Ctrl+C before the first Space cancels without creating a bag. Paused data is not recovered. Shutdown `Pausing recording` is normal. After Ctrl+C and clean recorder exit, check in the same terminal:

```bash
ros2 bag info "$jackal_bag"
```

Confirm nonzero counts for point clouds, scan, RGB/depth images, both camera_info streams, `tf`, and `tf_static`. Data on `map_metadata`, `pose`, and similar topics depends on their publishers; unpublished topics have no corresponding bag data. Discovery does not prove recording. Initial real recording still needed verification at this stage; only script syntax, topic sources, and QoS had been checked.

The script uses MCAP `zstd_fast` compression and splits at `2147483648` bytes (approximately 2 GiB), which is not a total size/duration limit. Images remain raw ROS Image messages; compression occurs during Host disk writes and does not reduce robot-to-Host network traffic. Static TF, LiDAR metadata, camera extrinsics, and map use `transient_local` reception; high-bandwidth image/LiDAR streams use `best_effort`. Recording does not recover wireless losses. Settings are in `config/jackal/rosbag_qos.yaml`.

MCAP options: [rosbag2 MCAP plugin](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_storage_mcap/README.md). For navigation-control and velocity-chain troubleshooting, use the separate [debug recorder](Jackal_Debug_Recording_2026-10-13.md).

---

## 6. Daily startup, verification, and shutdown

### 6.1 Recommended order after power-on

1. **Host:** verify wired `enp132s0=192.168.185.2` and ping access to `192.168.185.101` and `192.168.185.201`.
2. **TurtleBot:** verify `eth0=.185.101`, `usb0=.186.3`, and data on `/odom` / `/scan`; no Create 3 internal-network change is needed.
3. **Jackal:** verify all three Clearpath services are active and LiDAR/odometry publish normally.
4. **Jackal J-A:** run `slam.launch.py` unless already running.
5. **Jackal J-B:** run **`~/jackal_nav2/nav2_low_speed.launch.py`** unless already running.
6. **Jackal/Host:** check Nav2 lifecycle, velocity parameters, and topic connections; the Host only runs RViz/monitoring.
7. Send a small navigation goal only **after checking the area is safe**, keeping Motion Stop immediately available for the first run.

### 6.2 Quick checks for all three machines: copy to the corresponding terminal

**Host: **

```bash
ip -br -4 addr
ping -c 2 -W 1 192.168.185.101
ping -c 2 -W 1 192.168.185.201
ros2 topic info /odom
ros2 topic info /j100_0949/platform/odom
```

**TurtleBot: **

```bash
source /opt/ros/jazzy/setup.bash
ip -br -4 addr
ros2 topic info -v /odom
ros2 topic info -v /scan
```

**Jackal: **

```bash
source /etc/clearpath/setup.bash
systemctl is-active clearpath-robot clearpath-platform clearpath-sensors
ros2 node list | grep -E 'slam_toolbox|controller_server|velocity_smoother|collision_monitor'
ros2 lifecycle get /j100_0949/controller_server
ros2 param get /j100_0949/platform_velocity_controller linear.x.max_velocity
ros2 param get /j100_0949/controller_server FollowPath.vx_max
```

### 6.3 End the experiment

- Cancel the navigation goal and confirm the robot is fully stopped; keep Motion Stop accessible.
- Press `Ctrl+C` in Jackal's **Nav2**, then **SLAM** launch terminals. Clearpath platform services need not be stopped.
- Exit Host RViz and bag recording normally with `Ctrl+C`.
- To continue another day, saved `robot.yaml`, `nav2_low_speed.yaml`, and `nav2_low_speed.launch.py` need not be edited again.
- **Reboot does not automatically start custom SLAM/Nav2:** these currently use manual launches. Future automatic startup requires separate, verified service/launch management.

---

## 7. Common troubleshooting

| Symptom | First checks | Principle |
|---|---|---|
| SSH / ping unavailable | `ip -br -4 addr`, `ip route get <IP>`, `ping -c 3 <IP>` | Verify interface/IP/link before changing DDS |
| Host can ping but cannot see Jackal topics | Domain/RMW on both ends, Jackal Fast DDS XML, `/j100_0949/` namespace | Jackal `br0` has `.131.1` + `.185.201`; avoid the wrong `.131.1` discovery path |
| Topic visible but data rate zero | `ros2 topic info -v`, `ros2 topic hz`, hardware-driver/node logs | ROS graph presence does not prove continuous data |
| TurtleBot odometry/scan abnormal | `ros2 topic info -v /odom`, `/scan`, `usb0=.186.3` | Keep the independent Create 3 internal subnet unchanged |
| Jackal Nav2 not activating | `ros2 lifecycle get ...`, launch-terminal logs | Only lifecycle `active [3]` confirms activation |
| Nav2 limits revert after restart | `ps -eo pid,args | grep '[r]os2 launch'`, `grep file_parameters ...` | Use `nav2_low_speed.launch.py`, not the official default Nav2 launch |
| Inconsistent Jackal motion limits | `ros2 param get ... platform_velocity_controller`, `controller_server`, `velocity_smoother` | Chassis limits bound normal commands; align Nav2 planning constraints |
| `/cmd_vel` exists but Jackal does not move | Distinguish `/cmd_vel` from `/j100_0949/cmd_vel`; inspect Twist Mux | Global `/cmd_vel` currently belongs to TurtleBot; Jackal uses namespaced topics |
| RViz OpenGL fails | `nvidia-smi`, `glxinfo -B` | A graphics-driver fault does not imply a ROS network fault |

**DDS diagnostics on Jackal, only when needed:**

```bash
# Confirm services use the custom XML
systemctl show clearpath-platform.service -p Environment --no-pager
systemctl show clearpath-sensors.service -p Environment --no-pager

# If tcpdump is installed, observe discovery packets; exit with Ctrl+C/timeout
sudo timeout 10s tcpdump -ni br0 \
  'udp and dst host 239.255.0.1 and dst port 7400' -c 15
```

---

## 8. Configuration files and change principles

| Machine | File | Purpose | Status at this stage |
|---|---|---|---|
| Jackal | `/etc/clearpath/robot.yaml` | Final chassis software limits and robot configuration | ✅ Saved and loaded by services |
| Jackal | `/home/robot/jackal_nav2/nav2_low_speed.yaml` | MPPI + Velocity Smoother limits | `0.3 rad/s` runtime confirmed; repository `0.2 rad/s` candidate pending deployment verification |
| Jackal | `/home/robot/jackal_nav2/nav2_low_speed.launch.py` | Custom low-speed Nav2 launch | ✅ Process arguments confirmed use |
| Jackal | `/etc/clearpath/platform/config/twist_mux.yaml` | Control-source arbitration priorities | ✅ Read |
| Jackal | `/home/robot/fastdds_185.xml` | Select valid Fast DDS interface/IP | ✅ Deployed to platform/sensor services |
| Jackal | `/etc/netplan/50-cloud-init.yaml` | External/internal `br0` addresses | ✅ Persistence verified |
| TurtleBot | `/etc/netplan/40-ethernets.yaml` | External `eth0` / internal Create 3 `usb0` | ✅ Persistence verified |
| TurtleBot | `/etc/turtlebot4/fastdds_rpi.xml` | TurtleBot Fast DDS profile | ✅ Present and read |
| Host | Ethernet NetworkManager/Netplan | Current `enp132s0=.185.2` | ⚠️ Persistent configuration location unverified |

**Operating constraints:**

- Do not replace TurtleBot `/odom`, `/scan`, or `/cmd_vel` with imagined Jackal equivalents; Jackal uses `/j100_0949/...`.
- Do not directly edit official ROS package files under `/opt/ros/jazzy/share/clearpath_nav2_demos/...`; prefer copies under `/home/robot/jackal_nav2/`.
- Do not change controller parameters, restart services, or alter routes while a navigation goal is executing.
- Back up before network-interface changes to avoid losing SSH; retain configurations that are working normally.
- Verification covers the **normal ROS software control chain**, not a hardware safety guarantee of physical speed under every condition.

### References

- [Clearpath ROS configuration: Platform extras](https://docs.clearpathrobotics.com/docs/ros/config/yaml/platform/extras/)
- [Clearpath Platform API](https://docs.clearpathrobotics.com/docs/ros/api/platform_api/)
- [ROS 2 Controllers / diff_drive_controller (Jazzy)](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Nav2 MPPI Controller (Jazzy)](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)

---

**Runtime baseline: 2026-10-09; origin reset, specified-start mapping, speed tuning, and activation-timeout fixes updated: 2026-10-12.** The user confirmed physical zeroing and some limits by readback. The fixed start-pose script passed local tests; field acceptance was still pending at this stage. This manual uses measured baseline `Host .185.2 / TurtleBot .185.101 / Jackal .185.201`. Update it if IPs, ROS Domain, launch files, or topics change after experiments.
