# Jackal Quick Start: Specified Start Pose, Mapping, Navigation, and Recording

> 2026-10-14 RPP: all 5 goals in the 114801 bag succeeded. The current Host RPP file tightens the position tolerance to **0.10 m** and requires separate deployment. If already using RPP, update via [operating manual sections 2–4](Jackal_RPP_Trial_2026-10-14.md#2-upload-the-configuration-and-start-pose-script--host). The original indoor / Plan B deployment sections below remain as historical alternatives; their YAML copy commands overwrite the selected RPP configuration.


Applies to `j100_0949`, ROS 2 Jazzy. The example defines the **current pose as `(x=1 m, y=0 m, yaw=0°)` in a new map**, facing map `+X`. This starts mapping again without rebooting the system or chassis; chassis and sensor services must already be running normally.

**Sequence: stop the robot and end the old session → set the start pose and start SLAM → check coordinates → Nav2 → Host RViz → rosbag → send goals.** For first use of the updated script / indoor configuration, complete the deployment steps at the end first.

## 1. Prepare terminals

Cancel the old navigation goal and stop the robot. Save the old map first if needed. End the previous recording, then press `Ctrl+C` in the old Nav2 and SLAM terminals in that order, and close old RViz.

| Terminal | Location | Purpose |
| --- | --- | --- |
| J1 | Jackal | Set the start pose and keep SLAM running |
| J2 | Jackal | Check the start pose, then keep Nav2 running |
| H1 | Local Host | RViz |
| H2 | Local Host | rosbag |

Open separate terminals for J1 and J2 and log in to Jackal. Run all subsequent commands marked Jackal only after login succeeds:

```bash
ssh robot@192.168.185.201
```

## 2. Set the current pose and start SLAM — Jackal J1

Park where you want to start, and remain stationary during initialization and checks:

```bash
source /etc/clearpath/setup.bash
python3 "$HOME/start_jackal_slam_at_pose.py" --x 1 --y 0 --yaw-deg 0
```

`x/y` are in meters; `yaw-deg` is in degrees: `0` faces map `+X`, `90` faces `+Y`, and `-90` faces `-Y`. To use the current pose as the origin, set `--x 0 --y 0 --yaw-deg 0`. This command defines the current pose; **it does not drive the robot to the specified coordinates**.

Wait for **`SLAM ACTIVE. Check map -> base_link before starting Nav2.`** and keep J1 running. The script starts SLAM itself; do not start another `slam.launch.py`.

## 3. Check the start pose, then start Nav2 — Jackal J2

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/slam_toolbox
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

Expect `active [3]`, coordinates near `[1.000, 0.000, 0.000]`, and RPY yaw near `0°`. Press `Ctrl+C` to end **J2's TF query**; J1 continues running. If `map` remains unavailable, resolve SLAM startup first; see [detailed manual section 9](Jackal_Startup_SLAM_Nav2_RViz_Rosbag_Manual_2026-10-12.md#9-start-mapping-at-a-specified-pose-without-restarting-the-chassis).

Start the deployed indoor configuration in the same J2 terminal:

```bash
ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

Wait for **`Managed nodes are active`** and keep J2 running. This launcher reads `~/jackal_nav2/nav2_low_speed.yaml`, which must have been replaced with the updated indoor contents using the deployment steps below.

## 4. Open visualization — Host H1

Run locally on the Host:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
ros2 launch clearpath_viz view_navigation.launch.py \
  namespace:=/j100_0949 \
  use_sim_time:=false
```

Settings in RViz's left panel:

| Item | Value |
| --- | --- |
| Global Options → Fixed Frame | `map` |
| Map → Topic | `/j100_0949/map`; Reliable, Transient Local |
| LaserScan → Topic | `/j100_0949/sensors/lidar3d_0/scan`; Best Effort |
| Optional PointCloud2 → Topic | `/j100_0949/sensors/lidar3d_0/points`; Best Effort |

**Change the default LaserScan source from `lidar2d_0` to `lidar3d_0`.** This workflow uses SLAM, so `2D Pose Estimate` is unnecessary; the AMCL-related `Localization: inactive` state may appear.

## 5. Start recording manually, then send goals — Host H2 / RViz

To analyze overshoot, stops, or reversing, use [debug recording and log capture](Jackal_Debug_Recording_2026-10-13.md). It saves runtime parameters, navigation topics, and logs under `~/Desktop/jackal_rosbag_repo/debug/`.

Run in Host H2 to save data in a new timestamped folder under the recording directory:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/Desktop/jackal_rosbag_repo/recording"
cd "$HOME/Desktop/jackal_rosbag_repo/recording"
df -h .

jackal_bag="$PWD/jackal_$(date +%Y%m%d_%H%M%S)"
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal.sh "$jackal_bag"
```

- Startup displays **PAUSED**; recording has not started. Focus the **H2 terminal** and press **Space** once to start, then use Space to pause/resume.
- The default display is `Profile: GLIM + camera (21 topics)`: the original 14 collection topics plus 7 camera topics, including LiDAR, IMU, RGB/depth and calibration, odometry, joint states, TF, map, and pose. See the [production recording manual](Jackal_RPP_Trial_2026-10-14.md#8-production-recording-save-on-jackal-then-transfer-to-host) for the full list.
- After recording starts, use **Nav2 Goal** in RViz to click and drag in mapped free space, setting the target position and orientation. Choose a nearby target first.

## 6. Save and exit

Cancel the navigation goal and wait for the robot to stop. To save the map, keep SLAM running and execute the following in an **additional Jackal SSH terminal**:

```bash
source /etc/clearpath/setup.bash
jackal_map_dir="$HOME/maps/jackal_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_map_dir"
ros2 run nav2_map_server map_saver_cli -f "$jackal_map_dir/map" --ros-args \
  -r map:=/j100_0949/map
ros2 service call /j100_0949/slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$jackal_map_dir/posegraph'}"
```

Confirm successful map saving and serialization `result: 0`. Maps are under **`~/maps/` on Jackal**; the pose graph supports resuming mapping later.

Shutdown order: **H2 `Ctrl+C`, wait for bag writes → J2 `Ctrl+C` to exit Nav2 → J1 `Ctrl+C` to exit SLAM → close RViz.** Check the completed bag in the original H2 terminal:

```bash
ros2 bag info "$jackal_bag"
```

Verify messages on point-cloud, image, TF, and other topics. Start a new bag after each map-origin reset.

## Initial deployment or configuration updates

Run these steps only after script/configuration updates. Stop the robot and exit old Nav2 first; also stop old SLAM when updating the start-pose script. Keep using Jackal's existing `~/jackal_nav2/nav2_low_speed.launch.py`.

**Upload from the Host:**

```bash
cd /home/nyu/Desktop/Jackal_Turtlebot_Platform
ssh robot@192.168.185.201 'mkdir -p /home/robot/jackal_nav2/behavior_trees'
scp scripts/start_jackal_slam_at_pose.py robot@192.168.185.201:/home/robot/
scp config/jackal/nav2_low_speed_indoor.yaml robot@192.168.185.201:/home/robot/jackal_nav2/
scp config/jackal/behavior_trees/navigate_to_pose_indoor_recovery.xml \
  config/jackal/behavior_trees/navigate_through_poses_indoor_recovery.xml \
  robot@192.168.185.201:/home/robot/jackal_nav2/behavior_trees/
```

**After confirming all uploads succeeded, back up and select the indoor YAML in a Jackal terminal:**

```bash
(
  set -e
  cd "$HOME/jackal_nav2"
  test -f nav2_low_speed.launch.py
  test -f behavior_trees/navigate_to_pose_indoor_recovery.xml
  test -f behavior_trees/navigate_through_poses_indoor_recovery.xml
  jackal_indoor_backup=$(mktemp -d "$PWD/nav2_before_indoor.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_indoor_backup/"
  cp nav2_low_speed_indoor.yaml nav2_low_speed.yaml
  printf 'Previous active YAML: %s/nav2_low_speed.yaml\n' "$jackal_indoor_backup"
)
```

Current indoor settings: forward limit `0.2 m/s`, angular limit `0.2 rad/s`; raw footprint `0.52 × 0.44 m`, padding `0.02 m` per side, inflation radius `0.6 m`. These are configuration values; field performance still requires verification.

After deployment, start using steps 2–4 and read back key settings in another Jackal terminal:

```bash
source /etc/clearpath/setup.bash
ros2 param get /j100_0949/controller_server FollowPath.wz_max
ros2 param get /j100_0949/local_costmap/local_costmap footprint
ros2 param get /j100_0949/local_costmap/local_costmap footprint_padding
ros2 param get /j100_0949/local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /j100_0949/bt_navigator default_nav_to_pose_bt_xml
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver scan_ring
```

Expect, in order: `0.2`, a rectangle with corners at `±0.26 / ±0.22`, `0.02`, `0.6`, a path ending in `navigate_to_pose_indoor_recovery.xml`, and `15`. If values differ, check deployment before sending goals. For complete readback, rollback, and troubleshooting, see the [detailed operating manual](Jackal_Startup_SLAM_Nav2_RViz_Rosbag_Manual_2026-10-12.md).

## 8. Plan B: add Rotation Shim

Use this when already running the indoor YAML and comparing "align heading before tracking." The complete candidate is [nav2_low_speed_indoor_plan_b.yaml](../config/jackal/nav2_low_speed_indoor_plan_b.yaml). `FollowPath` loads Rotation Shim with MPPI as its inner controller. Original MPPI parameters remain directly under the same `FollowPath`; do not nest them under `primary_controller`.

| Plan B parameter | Value | Purpose |
| --- | --- | --- |
| `plugin` | `nav2_rotation_shim_controller::RotationShimController` | Outer heading controller |
| `primary_controller` | `nav2_mppi_controller::MPPIController` | Continue using MPPI after alignment |
| `angular_dist_threshold` | `0.35 rad`, approximately 20° | Engage alignment when heading error exceeds this threshold while checking a new path |
| `angular_disengage_threshold` | `0.17 rad`, approximately 10° | Return control to MPPI within this range, reducing switching near the boundary |
| `forward_sampling_distance` | `0.5 m` | Estimate heading from path geometry ahead |
| `rotate_to_heading_angular_vel` | `0.15 rad/s` | Shim alignment angular speed |
| `max_angular_accel` | `0.3 rad/s²` | Shim angular acceleration limit |
| `simulate_ahead_time` | `1.0 s` | Rotation collision-check horizon |
| `rotate_to_goal_heading` | `true` | Let Shim align final orientation once within goal position tolerance |
| `rotate_to_heading_once` | `false` | Recheck heading on subsequent replans |
| `closed_loop` | `true` | Shim uses odometry angular velocity to limit acceleration/deceleration |
| `use_path_orientations` | `false` | Derive heading from NavFn path geometry |

Here `closed_loop` belongs to Shim; the velocity smoother retains OPEN_LOOP. 20° is an engagement threshold, not a hard cap on body heading error. 10° is the handoff threshold, not final goal accuracy. Final tolerances remain 0.25 m / 0.25 rad. Parameter meanings and the shared namespace were checked against the [Nav2 1.3.12 source](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp).

**This file implements only Plan B.** MPPI still permits reversing with `vx_min: -0.1`; progress remains SimpleProgressChecker at 0.2 m / 30 s. Recovery trees, footprint, inflation, and MPPI/smoother speed limits remain unchanged. If the robot configuration has changed since this recording, compare differences before replacing the entire file.

Cancel the current goal and stop the robot. If recording, stop and wait for `Saved`, then press `Ctrl+C` in the Nav2 terminal. Keep existing SLAM and RViz running to compare on the same map.

**Upload the candidate from the Host:**

```bash
scp /home/nyu/Desktop/Jackal_Turtlebot_Platform/config/jackal/nav2_low_speed_indoor_plan_b.yaml \
  robot@192.168.185.201:/home/robot/jackal_nav2/
```

**Jackal: check the plugin, back up, and select Plan B:**

```bash
source /etc/clearpath/setup.bash
(
  set -e
  ros2 pkg prefix nav2_rotation_shim_controller
  cd "$HOME/jackal_nav2"
  test -f nav2_low_speed.launch.py
  test -f nav2_low_speed_indoor_plan_b.yaml
  test -f behavior_trees/navigate_to_pose_indoor_recovery.xml
  test -f behavior_trees/navigate_through_poses_indoor_recovery.xml
  jackal_plan_b_backup=$(mktemp -d "$PWD/nav2_before_plan_b.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_plan_b_backup/"
  cp nav2_low_speed_indoor_plan_b.yaml nav2_low_speed.yaml
  printf 'Previous active YAML: %s/nav2_low_speed.yaml\n' "$jackal_plan_b_backup"
)
```

Save the printed backup path. If the first plugin check reports `Package not found`, install it on Jackal, then rerun this block:

```bash
sudo apt install ros-jazzy-nav2-rotation-shim-controller
source /etc/clearpath/setup.bash
```

**Jackal: start Nav2 through the original entry point and save this run's logs:**

```bash
jackal_nav_log="$HOME/jackal_nav2/debug_logs/nav_plan_b_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_nav_log/config"
cp -a "$HOME/jackal_nav2/nav2_low_speed.yaml" \
  "$HOME/jackal_nav2/nav2_low_speed.launch.py" "$jackal_nav_log/config/"
cp -a "$HOME/jackal_nav2/behavior_trees" "$jackal_nav_log/config/"
set -o pipefail
ROS_LOG_DIR="$jackal_nav_log/ros" RCUTILS_COLORIZED_OUTPUT=0 \
  ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
    setup_path:=/etc/clearpath \
    scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
    use_sim_time:=false \
  2>&1 | tee -i "$jackal_nav_log/console.log"
```

Expect creation logs for both the outer `RotationShimController` and inner `MPPIController`, followed by `Managed nodes are active`. `Activated MPPI Controller` alone does not prove Shim is active.

**Read back parameters in another Jackal terminal:**

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/controller_server
ros2 param get /j100_0949/controller_server FollowPath.plugin
ros2 param get /j100_0949/controller_server FollowPath.primary_controller
ros2 param get /j100_0949/controller_server FollowPath.angular_dist_threshold
ros2 param get /j100_0949/controller_server FollowPath.angular_disengage_threshold
ros2 param get /j100_0949/controller_server FollowPath.rotate_to_heading_angular_vel
ros2 param get /j100_0949/controller_server FollowPath.rotate_to_goal_heading
ros2 param get /j100_0949/controller_server FollowPath.wz_max
ros2 param get /j100_0949/controller_server FollowPath.vx_min
ros2 param get /j100_0949/controller_server progress_checker.plugin
```

Expect, in order: `active [3]`, `nav2_rotation_shim_controller::RotationShimController`, `nav2_mppi_controller::MPPIController`, `0.35`, `0.17`, `0.15`, `True`, `0.2`, `-0.1`, and `nav2_controller::SimpleProgressChecker`. Resolve missing plugins or undeclared parameters through installation/version checks; increasing speed does not resolve startup errors.

Create a new Plan B bag using the [debug recording instructions](Jackal_Debug_Recording_2026-10-13.md). First test a goal approximately 2 m ahead with its arrow along the travel direction, then an initial heading difference of approximately 90°, and finally goal-orientation adjustment separately. Progress still counts only position displacement, so in-place rotation may still trigger progress failures; compare that separately in Plan D.

Rollback: cancel the goal, stop the robot, end this recording and Nav2 in the same order, restore the backup printed by this section to the active file, then restart Nav2 through the original entry point:

```bash
# Replace XXXXX with the backup directory actually printed above before running.
cp "$HOME/jackal_nav2/nav2_before_plan_b.XXXXX/nav2_low_speed.yaml" \
  "$HOME/jackal_nav2/nav2_low_speed.yaml"
```

Local validation: only 12 `FollowPath` entries differ from the indoor baseline; ROS parameter parsing passed. Host Nav2 1.3.12 successfully configured the real Shim + MPPI plugins in localhost-only Domain 213, read back parameters, and cleaned up. Tests did not activate the controller or send motion goals. A subsequent field bag confirmed deployment but still showed oscillation; see the next section for updated turning and progress settings.


## 9. Reduce MPPI angular speed and include rotation in progress checking

For this experiment, Shim was already at 0.10 rad/s and the PathAngleCritic threshold at 0.10 rad, but oscillation persisted after turning. The latest complete candidate is [nav2_low_speed_indoor_turn_control.yaml](../config/jackal/nav2_low_speed_indoor_turn_control.yaml); see the [122553 analysis](../analysis/jackal_20261013_122553/REPORT.md) for bag evidence.

| Item | New value |
| --- | --- |
| MPPI angular limit | 0.08 rad/s, approximately 4.6°/s |
| Shim target angular speed | 0.10 rad/s, approximately 5.7°/s |
| Shared velocity smoother angular range | −0.10 to +0.10 rad/s |
| Progress plugin | `nav2_controller::PoseProgressChecker` |
| Displacement / rotation / time allowance | 0.20 m / 0.35 rad (approximately 20°) / 30 s |

Exceeding either the displacement or rotation threshold resets the reference and timer. Rotation means the shortest angular change relative to the reference, not accumulated rotation. Oscillation may therefore count as progress; assess heading convergence, continuous forward motion, and goal success. Angular acceleration/deceleration limits remain 0.3, and linear limits remain 0.2 m/s forward / 0.1 m/s reverse.

**Restart Nav2 to load the new progress plugin.** Do not attempt to hot-swap only the string with `ros2 param set ... progress_checker.plugin ...`. Keep SLAM and RViz running. Cancel the goal and stop the robot; if recording, stop and wait for `Saved`, then press Ctrl+C in the Nav2 launch terminal.

**Upload from the Host:**

```bash
scp /home/nyu/Desktop/Jackal_Turtlebot_Platform/config/jackal/nav2_low_speed_indoor_turn_control.yaml \
  robot@192.168.185.201:/home/robot/jackal_nav2/
```

**Jackal: back up and select the new configuration:**

```bash
source /etc/clearpath/setup.bash
(
  set -e
  cd "$HOME/jackal_nav2"
  test -f nav2_low_speed.launch.py
  test -f nav2_low_speed_indoor_turn_control.yaml
  test -f behavior_trees/navigate_to_pose_indoor_recovery.xml
  test -f behavior_trees/navigate_through_poses_indoor_recovery.xml
  jackal_turn_backup=$(mktemp -d "$PWD/nav2_before_turn_control.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_turn_backup/"
  cp nav2_low_speed_indoor_turn_control.yaml nav2_low_speed.yaml
  printf 'Previous active YAML: %s/nav2_low_speed.yaml\n' "$jackal_turn_backup"
)
```

The backup saves the active YAML on disk. Runtime changes made only with `ros2 param set` are not automatically included. This new candidate explicitly contains Shim 0.10 and PathAngleCritic 0.10.

**Jackal: start and save this run's logs and configuration:**

```bash
jackal_nav_log="$HOME/jackal_nav2/debug_logs/nav_turn_control_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_nav_log/config"
cp -a "$HOME/jackal_nav2/nav2_low_speed.yaml" \
  "$HOME/jackal_nav2/nav2_low_speed.launch.py" "$jackal_nav_log/config/"
cp -a "$HOME/jackal_nav2/behavior_trees" "$jackal_nav_log/config/"
set -o pipefail
ROS_LOG_DIR="$jackal_nav_log/ros" RCUTILS_COLORIZED_OUTPUT=0 \
  ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
    setup_path:=/etc/clearpath \
    scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
    use_sim_time:=false \
  2>&1 | tee -i "$jackal_nav_log/console.log"
```

Expect `Created progress_checker ... nav2_controller::PoseProgressChecker`, followed by `Managed nodes are active`. If the plugin fails to load, retain the error log and use the rollback procedure below.

**Verify in another Jackal terminal:**

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/controller_server
ros2 param get /j100_0949/controller_server FollowPath.wz_max
ros2 param get /j100_0949/controller_server FollowPath.rotate_to_heading_angular_vel
ros2 param get /j100_0949/controller_server FollowPath.PathAngleCritic.max_angle_to_furthest
ros2 param get /j100_0949/velocity_smoother max_velocity
ros2 param get /j100_0949/velocity_smoother min_velocity
ros2 param get /j100_0949/controller_server progress_checker.plugin
ros2 param get /j100_0949/controller_server progress_checker.required_movement_radius
ros2 param get /j100_0949/controller_server progress_checker.required_movement_angle
ros2 param get /j100_0949/controller_server progress_checker.movement_time_allowance
```

Expected values, in order: `active [3]`, `0.08`, `0.1`, `0.1`, `[0.2, 0.0, 0.1]`, `[-0.1, 0.0, -0.1]`, `nav2_controller::PoseProgressChecker`, `0.2`, `0.35`, and `30.0`.

**Host: start a new recording:**

```bash
cd ~/Desktop/Jackal_Turtlebot_Platform
bash scripts/record_jackal_debug.sh
```

Wait for parameter snapshots, press Space, and wait for subscriptions before sending goals. Test a straight goal first, then an approximately 90° turn with the goal arrow along the travel direction. Cancel and save after one clear left-right overshoot cycle. Press Ctrl+C to end recording and wait for final snapshots and `Saved` before stopping Nav2/SLAM. Then retrieve the corresponding logs using the [debug recording instructions](Jackal_Debug_Recording_2026-10-13.md).

**Rollback:** stop recording and Nav2 in the same order, restore the printed backup to the active YAML, then restart using the entry point above:

```bash
# Replace XXXXX with the actual backup directory before running.
cp "$HOME/jackal_nav2/nav2_before_turn_control.XXXXX/nav2_low_speed.yaml" \
  "$HOME/jackal_nav2/nav2_low_speed.yaml"
```

Local validation is complete: ROS parameter parsing and isolated configuration/readback/cleanup passed for the real Shim, MPPI, PoseProgressChecker, and velocity smoother. No test node was activated and no motion goal was sent; field behavior still requires this run's bag verification.
