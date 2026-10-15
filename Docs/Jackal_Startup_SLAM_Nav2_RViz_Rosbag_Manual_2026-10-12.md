# Jackal Operating Manual: Reset Origin, Map, Navigate, and Record

For current everyday RPP operation, use the [complete operating manual](Jackal_RPP_Trial_2026-10-14.md), covering the specified start pose, SLAM, indoor RPP Nav2, Host RViz, and updated production recording. A [quick-start checklist](Jackal_Quick_Start_2026-10-13.md) is also available.

Applies to Jackal `j100_0949`, Ubuntu 24.04, ROS 2 Jazzy. Updated: 2026-10-12.

This workflow uses the **current location and heading as the start pose of a new map**, starts SLAM and low-speed Nav2 on Jackal, then opens RViz and records LiDAR, camera, and other data on the Host. At startup, the robot's map `x / y / yaw` should be near `0 / 0 / 0`, with its front facing map `+X`.

**Default workflow:** stop and end the old session → restart the chassis and check the origin → start SLAM → start low-speed Nav2 → Host RViz → Host rosbag → save and exit.

**Workflow used in this session:** [section 9: specified start pose](#9-start-mapping-at-a-specified-pose-without-restarting-the-chassis) → section 4 low-speed Nav2 → section 5 Host RViz → section 6 rosbag → section 7 save and exit. Define the current pose as `(x=1 m, y=0 m, yaw=0°)`, facing map `+X`. This avoids restarting the chassis and replaces startup steps in sections 2.2 and 3. Retain environment preparation in section 1, ending the old session in section 2.1, and service/LiDAR checks in section 2.3.

## 1 Prepare terminals and environments

| Terminal | Device | Purpose |
| --- | --- | --- |
| J0 | Jackal via SSH | Reset origin, check parameters, save maps |
| J1 | Jackal, another SSH terminal | Keep SLAM running |
| J2 | Jackal, another SSH terminal | Keep low-speed Nav2 running |
| H1 | Local Host | Keep RViz running |
| H2 | Local Host | Control rosbag recording manually |

Host is `192.168.185.2`; Jackal is `192.168.185.201`. Startup uses the existing network configuration.

**Host:** first check connectivity to Jackal.

```bash
ping -c 3 -W 1 192.168.185.201
```

**J0, J1, J2:** open separate terminals, log in via SSH, and load the onboard environment after entering Jackal.

```bash
ssh robot@192.168.185.201
```

```bash
source /etc/clearpath/setup.bash
```

**H1, H2:** load the environment locally on the Host; do not SSH into the robot.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
```

Retain the current Fast DDS interface configuration. Jackal topics use the `/j100_0949` prefix; frame IDs in this manual are `map`, `odom`, and `base_link`, without that topic prefix.

## 2 Use the current location and heading as the new map start

### 2.1 End the old session

1. Cancel the active navigation goal; confirm the robot is stopped and the remote controller has no motion input.
2. If retaining the old map, save it using section 7 before exiting SLAM.
3. Press `Ctrl+C` in the original Nav2 terminal and wait for exit, then do the same in the original SLAM terminal.
4. Place the robot at the new origin, facing the desired map `+X` direction. Remain stationary during initialization and checks.

This starts a new map without loading an old map or pose graph. RViz `2D Pose Estimate` is not the origin-reset operation in this workflow.

### 2.2 Restart the chassis and check odometry

**Jackal J0: **

```bash
source /etc/clearpath/setup.bash
sudo systemctl restart clearpath-platform.service
systemctl is-active clearpath-platform.service
```

Expect `active`. The platform service handles chassis control and localization and can be restarted independently of sensor services; see [Clearpath services](https://docs.clearpathrobotics.com/docs/ros/config/services/).

Then check raw odometry and the TF actually used:

```bash
ros2 topic echo /j100_0949/platform/odom --once --field pose.pose
```

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

**Pass criteria:** raw odometry `position.x/y` and yaw are near zero; `tf2_echo` translation `x/y` and RPY yaw are also near zero, with no substantial jumps while stationary for several seconds. Press `Ctrl+C` to end the query afterward.

`systemctl restart clearpath-platform.service` **restarts only the platform service, not the operating system or computer**.

An `active` service only means it started; **use readings to verify zeroing**. The user's 2026-10-12 output passed: raw odometry `x≈0.000006 m`, `y≈0`; `odom → base_link` translation near zero, yaw approximately `0.009–0.010°`, stable for several seconds. Recheck each time. If a restart does not zero it, stop before mapping and inspect chassis/filtered odometry rather than repeatedly sending zero poses to conceal the offset.

The current EKF fuses absolute wheel-odometry pose, so subsequent odometry may pull back a one-time `set_pose`. SLAM Toolbox initializes scans using odometry pose; therefore check odometry before starting fresh SLAM. See the [SLAM Toolbox Jazzy implementation](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp).

### 2.3 Check services and ring 15

**Jackal J0: **

```bash
systemctl is-active \
  clearpath-robot.service \
  clearpath-platform.service \
  clearpath-sensors.service

ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver scan_ring
ros2 topic echo /j100_0949/sensors/lidar3d_0/scan \
  --once --field header --qos-reliability best_effort
```

All three services should be `active`, and `scan_ring` should be `15`. On this LiDAR, ring 15 is approximately `+0.32°`, nearly horizontal. It selects only the LaserScan beam; PointCloud2 still includes all beams.

A physical-object check previously confirmed that **point-cloud front/back directions match reality**. Keep mounting yaw unchanged. Verify the applied scan-frame correction:

```bash
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver lidar_frame
ros2 param get /j100_0949/sensors/lidar3d_0/ouster_driver pub_static_tf
```

Expected corrected values are `lidar3d_0_laser` and `true`; scan `header.frame_id` should be `lidar3d_0_laser`. These readings verify the correction; `scan_ring=15` alone does not verify direction.

If the old scan-frame configuration remains, complete checks and necessary corrections in [original manual J-2a](ROS2_Jackal_TurtleBot_Host_CMD_Manual_2026-10-09.md#j-2a-ouster-ring-15-and-scan-frame-correction-2026-10-12), then restart from this section. Section 5 provides a stationary physical-object direction check.

## 3 Start new SLAM on Jackal

**Jackal J1: **

```bash
source /etc/clearpath/setup.bash
ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

Keep J1 running. SLAM uses the Ouster ring-15 LaserScan; do not start another SLAM instance in the same namespace.

**Jackal J0:** check the map and new origin.

```bash
ros2 param get /j100_0949/slam_toolbox scan_topic
ros2 param get /j100_0949/slam_toolbox map_file_name
ros2 topic echo /j100_0949/map --once --field header \
  --qos-durability transient_local
```

`scan_topic` should be `/j100_0949/sensors/lidar3d_0/scan`; `map_file_name` should be empty or unset, without loading an old pose graph. The map frame should be `map`.

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

Keep the robot stationary and confirm `x/y/yaw` near `0/0/0`, then press `Ctrl+C` to stop the query. Coordinates should change after subsequent motion. Map message `info.origin` describes the grid boundary, can be negative, and need not be zero.

## 4 Start low-speed Nav2 on Jackal

### 4.1 Check onboard files and start

**Jackal J2: **

```bash
source /etc/clearpath/setup.bash
ls -l \
  "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  "$HOME/jackal_nav2/nav2_low_speed.yaml" \
  "$HOME/jackal_nav2/behavior_trees/navigate_to_pose_slow_recovery.xml" \
  "$HOME/jackal_nav2/behavior_trees/navigate_through_poses_slow_recovery.xml"
```

All four files above must exist. Low-speed, smoothing, and recovery settings share the onboard `nav2_low_speed.yaml`; do not start separate smooth or recovery YAML configurations. If files are missing, deploy them using original manual J-5a/J-5b first.

```bash
ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

Keep J2 running and wait for lifecycle activation. This custom entry point loads the existing low-speed and smoothing configuration.

### 4.2 Check nodes and scan topics

**Jackal J0: **

```bash
for node in controller_server planner_server behavior_server bt_navigator velocity_smoother collision_monitor; do
  printf '%s: ' "$node"
  ros2 lifecycle get "/j100_0949/$node"
done
```

Each node should be `active [3]`. Then check the scans actually subscribed to by the costmaps and Collision Monitor:

```bash
ros2 param get /j100_0949/local_costmap/local_costmap voxel_layer.scan.topic
ros2 param get /j100_0949/global_costmap/global_costmap obstacle_layer.scan.topic
ros2 param get /j100_0949/collision_monitor scan.topic
```

All three should point to `/j100_0949/sensors/lidar3d_0/scan`. If relative names are returned, use the commands below to verify both costmaps and Collision Monitor subscribe to the full topic. If still using `lidar2d_0/scan`, correct parameter rewriting in the custom launch before sending goals; do not disable Collision Monitor to bypass the issue.

```bash
ros2 topic info -v /j100_0949/sensors/lidar3d_0/scan
```

### 4.3 Verify low-speed and recovery parameters

**Jackal J0: **

```bash
ros2 param get /j100_0949/controller_server FollowPath.vx_max
ros2 param get /j100_0949/controller_server FollowPath.wz_max
ros2 param get /j100_0949/controller_server FollowPath.ax_min
ros2 param get /j100_0949/velocity_smoother max_velocity
ros2 param get /j100_0949/velocity_smoother min_velocity
ros2 param get /j100_0949/velocity_smoother max_decel
ros2 param get /j100_0949/velocity_smoother smoothing_frequency
ros2 param get /j100_0949/behavior_server max_rotational_vel
ros2 param get /j100_0949/bt_navigator default_nav_to_pose_bt_xml
```

| Parameter | Expected current configuration value |
| --- | --- |
| `FollowPath.vx_max` | `0.3 m/s` |
| `FollowPath.wz_max` | `0.2 rad/s` |
| `FollowPath.ax_min` | `-0.3 m/s²` |
| smoother `max_velocity` | `[0.3, 0.0, 0.2]`: x, y, angular velocity |
| smoother `min_velocity` | `[-0.2, 0.0, -0.2]`: both turn directions limited to `0.2 rad/s` |
| smoother `max_decel` | `[-0.3, 0.0, -0.3]`: x, y, angular deceleration |
| `smoothing_frequency` | `50.0 Hz` |
| recovery `max_rotational_vel` | `0.15 rad/s` |
| `default_nav_to_pose_bt_xml` | `/home/robot/jackal_nav2/behavior_trees/navigate_to_pose_slow_recovery.xml` |

The user previously confirmed the `0.3 rad/s` angular limit, deceleration, smoothing frequency, recovery spin speed, and single-goal BT path. Following "reduce turning speed first," this repository lowers normal navigation angular speed to `0.2 rad/s` (approximately `11.5°/s`). The user later confirmed using `0.2 rad/s` with continued overshoot; other values require actual readback. See 4.5 for the combined tuning candidate. Angular acceleration/deceleration remains `0.3 rad/s²`, and recovery spin remains `0.15 rad/s`. These settings cover normal navigation/recovery; collision protection or emergency stops may stop immediately, so gentle deceleration is not guaranteed in every situation.

### 4.4 First compare a lower turning speed

Cancel the current goal in RViz and wait until stationary. If Nav2 is running, temporarily lower the normal-navigation angular limit in **Jackal J0**:

```bash
source /etc/clearpath/setup.bash
ros2 param set /j100_0949/controller_server FollowPath.wz_max 0.2
ros2 param set /j100_0949/velocity_smoother max_velocity '[0.3, 0.0, 0.2]'
ros2 param set /j100_0949/velocity_smoother min_velocity '[-0.2, 0.0, -0.2]'
```

After all three settings succeed, read back those values as in 4.3 before sending a new goal. If a node is missing, start Nav2 first; if setting fails, do not treat the change as active. This comparison changes only the normal-navigation angular limit, retaining linear speed, angular acceleration/deceleration, and reversing policy to compare overshoot and correction counts. Lower turning speed does not guarantee removal of backup recoveries.

`ros2 param set` does not save to YAML; Nav2 reloads the file on restart. To persist changes, stop the robot and Nav2, back up and upload the latest repository `nav2_low_speed.yaml` using original manual J-5a, then restart using 4.1 and verify with 4.3. Only the three angular-limit values change; existing slow-recovery XML files remain usable.

### 4.5 New indoor low-speed MPPI candidate

The user still observed large heading deviations, stops, and reversing at `0.2 rad/s`. Logs confirm that some large rotations and reverse motions were `Spin` / `BackUp` actions following `Failed to make progress`; not every event can be attributed to mechanical overshoot.

The new standalone file is [`nav2_low_speed_indoor.yaml`](../config/jackal/nav2_low_speed_indoor.yaml), with [`navigate_to_pose_indoor_recovery.xml`](../config/jackal/behavior_trees/navigate_to_pose_indoor_recovery.xml) and [`navigate_through_poses_indoor_recovery.xml`](../config/jackal/behavior_trees/navigate_through_poses_indoor_recovery.xml). It is a low-speed tuning candidate based on the original Jackal configuration, not an online example already validated on this robot.

| Item | Original repository configuration | New candidate |
| --- | --- | --- |
| MPPI / smoother forward limit | 0.3 m/s | 0.2 m/s |
| MPPI / smoother reverse-speed magnitude | 0.2 m/s | 0.1 m/s; reversing still allowed |
| Normal-navigation angular limit | 0.2 rad/s | Unchanged at 0.2 rad/s |
| Linear / angular acceleration/deceleration magnitudes | 0.3 | Unchanged at 0.3 m/s² / 0.3 rad/s² |
| MPPI prediction horizon | 56 × 0.05 = 2.8 s | 80 × 0.05 = 4 s |
| Samples / `vx_std` / `wz_std` | 2000 / 0.2 / 0.4 | 1500 / 0.1 / 0.1 |
| PathAlign `offset_from_furthest` | 20 path points | 10 path points; weight remains 14 |
| PathFollow / GoalCritic distance switching threshold | 1.4 m | 0.8 m |
| GoalAngle activation distance | 0.5 m | 0.3 m; evaluate final heading closer to the goal |
| PathAngle `max_angle_to_furthest` | 1.0 rad | 0.35 rad, approximately 20° |
| Displacement progress check | 0.5 m / 20 s | 0.2 m / 30 s |
| Per-recovery spin request / speed limit | 1.57 rad / 0.15 rad/s | 0.523599 rad (approximately 30°) / 0.1 rad/s |
| Per-recovery backup request / speed | 0.30 m / 0.05 m/s | 0.15 m / 0.05 m/s |
| Footprint relative to base_link, user-confirmed | 0.36 × 0.30 m | 0.26 m forward/backward and 0.22 m left/right: 0.52 × 0.44 m |
| `footprint_padding` | Not explicitly configured | 0.02 m per side; padded dimensions 0.56 × 0.48 m |
| Global / local `inflation_radius` | 0.8 m | 0.6 m; `cost_scaling_factor` remains 4.0 |

Controller frequency remains 20 Hz with matching `model_dt=0.05`; the velocity smoother remains 50 Hz. Total predicted sample points increase from 112000 to 120000. This is only a rough workload comparison and does not guarantee onboard 20 Hz operation. Critic weights, goal tolerances (0.25 m / 0.25 rad), and collision-monitor logic are unchanged. Collision Monitor still subscribes to the local costmap's footprint and therefore uses the updated body outline. All three scan parameters explicitly use `/j100_0949/sensors/lidar3d_0/scan`; ring 15 remains an Ouster driver setting.

`max_angle_to_furthest=0.35` activates a heading cost; it does not guarantee yaw error below 20°. `required_movement_radius=0.2` is displacement from the last reference position, not a path checkpoint or accumulated distance. In-place rotation does not count. A longer allowance also delays detection of a truly stuck robot. Shorter recovery actions do not resolve the underlying navigation blockage; recoveries may repeat, and MPPI itself still allows reversing.

The user confirmed a footprint extending 0.26 m forward/backward and 0.22 m left/right from `base_link`, with 0.02 m padding per side. Both costmaps now use this rectangle (0.56 × 0.48 m padded), with inflation reduced from 0.8 m to 0.6 m. Recheck the outline after changing protruding loads. A larger footprint may expand the forbidden area immediately beside obstacles; a smaller inflation radius mainly reduces the surrounding cost field. First compare short straight motion, turns, and stops in open space, then assess heading deviations and recoveries on the same path.

**Before deployment:** cancel the goal, stop the robot, and press `Ctrl+C` in the original Jackal Nav2 terminal, waiting for exit. SLAM, chassis, and sensors may continue running; no origin reset is needed. Upload the new file under a separate name, then back up and replace the active YAML read by the existing launcher.

**Upload from the Host:**

```bash
cd /home/nyu/Desktop/Jackal_Turtlebot_Platform
scp config/jackal/nav2_low_speed_indoor.yaml \
  robot@192.168.185.201:/home/robot/jackal_nav2/
scp config/jackal/behavior_trees/navigate_to_pose_indoor_recovery.xml \
  config/jackal/behavior_trees/navigate_through_poses_indoor_recovery.xml \
  robot@192.168.185.201:/home/robot/jackal_nav2/behavior_trees/
```

**Jackal: back up and select the new configuration:** run after both uploads succeed. Both new XML files must exist; replacing only YAML leaves missing behavior trees. Reuse the deployed custom launch without assuming it accepts a `params_file` argument.

```bash
(
  set -e
  cd "$HOME/jackal_nav2"
  test -f behavior_trees/navigate_to_pose_indoor_recovery.xml
  test -f behavior_trees/navigate_through_poses_indoor_recovery.xml
  jackal_indoor_backup=$(mktemp -d "$PWD/nav2_before_indoor.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_indoor_backup/"
  cp nav2_low_speed_indoor.yaml nav2_low_speed.yaml
  printf 'Previous active YAML: %s/nav2_low_speed.yaml\n' "$jackal_indoor_backup"
)
```

Note the printed backup path, confirm success, then start on Jackal:

```bash
source /etc/clearpath/setup.bash
ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

**Read back in another Jackal terminal:**

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/controller_server
for p in FollowPath.vx_max FollowPath.vx_min FollowPath.wz_max \
         FollowPath.time_steps FollowPath.model_dt FollowPath.batch_size \
         FollowPath.vx_std FollowPath.wz_std \
         FollowPath.PathAlignCritic.offset_from_furthest \
         FollowPath.PathAngleCritic.max_angle_to_furthest \
         progress_checker.required_movement_radius progress_checker.movement_time_allowance; do
  printf '%s: ' "$p"
  ros2 param get /j100_0949/controller_server "$p"
done
ros2 param get /j100_0949/velocity_smoother max_velocity
ros2 param get /j100_0949/velocity_smoother min_velocity
ros2 param get /j100_0949/behavior_server max_rotational_vel
ros2 param get /j100_0949/bt_navigator default_nav_to_pose_bt_xml
ros2 param get /j100_0949/bt_navigator default_nav_through_poses_bt_xml
ros2 param get /j100_0949/local_costmap/local_costmap voxel_layer.scan.topic
ros2 param get /j100_0949/global_costmap/global_costmap obstacle_layer.scan.topic
ros2 param get /j100_0949/collision_monitor scan.topic
for node in local_costmap global_costmap; do
  ros2 param get "/j100_0949/$node/$node" footprint
  ros2 param get "/j100_0949/$node/$node" footprint_padding
  ros2 param get "/j100_0949/$node/$node" inflation_layer.inflation_radius
  ros2 param get "/j100_0949/$node/$node" inflation_layer.cost_scaling_factor
done
```

The controller should be `active [3]`; the loop should return `0.2, -0.1, 0.2, 80, 0.05, 1500, 0.1, 0.1, 10, 0.35, 0.2, 30.0`. Smoother values should be `[0.2,0,0.2]` and `[-0.1,0,-0.2]`; recovery angular speed `0.1`. Both BT paths should end in `_indoor_recovery.xml`, and all three scan parameters should use the Ouster `/scan`. Both costmaps should report footprint `[[0.26,0.22],[0.26,-0.22],[-0.26,-0.22],[-0.26,0.22]]`, padding `0.02`, inflation radius `0.6`, and scaling factor `4.0`. In RViz `Controller → Polygon`, view `/j100_0949/local_costmap/published_footprint` and confirm it encloses the body. If readback differs, inspect the YAML actually used by the launch before sending goals.

**Rollback:** cancel the goal, stop the robot and Nav2, copy `nav2_low_speed.yaml` from the printed backup path back to `~/jackal_nav2/nav2_low_speed.yaml`, and restart with the same command. Original `_slow_recovery.xml` files remain unchanged, so backups referencing them need no XML edits. Do not select a backup using an ambiguous wildcard.

**Local validation:** duplicate YAML keys, XML syntax/references, MPPI/smoother velocity and acceleration/deceleration consistency, and recovery time allowances passed checks. Local Jazzy parameter rewriting and ROS 2 parsing read 275 parameters across 11 node sections. Checks used isolated Domain 211 with parameter-reader nodes only; Nav2/controllers were not started and no motion commands were published. Onboard deployment and physical tracking were not yet verified at this stage.

For parameter meanings, see [Jazzy MPPI](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) and [SimpleProgressChecker](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/simple_progress_checker/). These trial values are not a combination guaranteed by those documents.

## 5 Open RViz on the Host

### 5.1 Start the window

**Host H1:** use the Host environment from section 1, then run:

```bash
ros2 launch clearpath_viz view_navigation.launch.py \
  namespace:=/j100_0949 \
  use_sim_time:=false
```

This entry point remaps RViz TF subscriptions to `/j100_0949/tf` and `/j100_0949/tf_static`. Run only this window on the Host; SLAM/Nav2 continue in Jackal J1/J2.

### 5.2 Set display topics

Expand each item under **Displays** on the left and check:

| RViz item | Setting |
| --- | --- |
| `Global Options → Fixed Frame` | Use `map` for normal mapping and navigation |
| `Map → Topic` | `/j100_0949/map` |
| Map QoS | `Reliable`, `Transient Local` |
| `LaserScan → Topic` | **`/j100_0949/sensors/lidar3d_0/scan`** |
| LaserScan QoS | `Best Effort`, `Volatile` |
| `RobotModel → Description Topic` | `/j100_0949/robot_description`, `Reliable`, `Transient Local` |
| Optional `PointCloud2 → Topic` | `/j100_0949/sensors/lidar3d_0/points`, `Best Effort`, `Volatile` |

**Default Clearpath RViz LaserScan uses `sensors/lidar2d_0/scan`; change it to `lidar3d_0/scan` as above.** If PointCloud2 is absent, add it with bottom-left `Add → By display type → PointCloud2`. Disable PointCloud2 during everyday navigation to reduce Host rendering load if needed.

### 5.3 Check front/back directions while stationary

Keep the robot stationary and do not send navigation goals yet.

1. Click the value of `Global Options → Fixed Frame` and temporarily set it to `base_link`.
2. Temporarily disable Map and costmap displays; retain RobotModel, LaserScan, PointCloud2, and TF.
3. Inspect the `base_link` axes in TF: **red `+X` points forward; green `+Y` points left**. Do not infer front/back from screen up/down.
4. With cardboard behind the robot, its LaserScan and PointCloud2 returns should both lie opposite the red X arrow. Move it to the front-left; returns should appear on the `+X / +Y` side.
5. After passing the check, restore Fixed Frame to `map` and re-enable map displays.

LaserScan contains only ring 15; PointCloud2 contains all beams. Compare the cardboard's position and direction. If the cloud is correct but the scan is reversed, address the scan frame first rather than adding 180° to the entire LiDAR mounting yaw.

### 5.4 Prepare navigation goals

Confirm Navigation is `active` and map/obstacle directions are correct. This workflow maps with SLAM Toolbox; AMCL-related `Localization: inactive` in RViz is acceptable and does not require starting AMCL.

To record the complete experiment, start rosbag using section 6 before using the top **Nav2 Goal** tool. Click and drag in mapped free space; the arrow sets target orientation. Choose a nearby goal first and verify physical and map motion directions agree. A new map uses new coordinates; do not reuse goals from the old session.

## 6 Record rosbag on the Host

### 6.1 Prepare recording in the specified directory

**Host H2:** keep SLAM, Nav2, and RViz running, and execute in a separate local terminal:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/Desktop/jackal_rosbag_repo/recording"
cd "$HOME/Desktop/jackal_rosbag_repo/recording"
df -h .

jackal_bag="$PWD/jackal_$(date +%Y%m%d_%H%M%S)"
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal.sh "$jackal_bag"
```

The script defaults to the updated 21 topics without extra options. It loads the Host ROS environment and displays `Profile: GLIM + camera (21 topics)`, `Output: ...`, and `PAUSED — press Space to start`. Each run creates a new timestamped directory at:

```text
/home/nyu/Desktop/jackal_rosbag_repo/recording/jackal_YYYYMMDD_HHMMSS/
```

**Use these keys while the H2 recording terminal has focus, not inside RViz.**

| Operation | Effect |
| --- | --- |
| Start script | Wait without recording |
| First `Space` press | Create the recorder, subscribe, and save data |
| `Space` while recording | Pause saving subsequent data |
| `Space` while paused | Resume recording |
| `Ctrl+C` | Stop, flush the cache, and save; wait for the shell prompt |
| `Ctrl+C` before the first `Space` | Cancel without creating a bag |

The script implements the initial wait and starts the recorder only on the first Space press, helping retain already-published static TF and calibration. rosbag handles later pauses/resumes; data from paused intervals is not recovered. See [rosbag2 pause semantics](https://github.com/ros2/rosbag2/tree/jazzy#controlling-recordings-via-services).

### 6.2 Recorded data

| Data | Contents |
| --- | --- |
| LiDAR | Full-beam `points`, ring-15 `scan`, LiDAR metadata, LiDAR IMU |
| Camera | Raw color/depth images, both camera_info and metadata streams, depth_to_color extrinsics |
| Body state | Raw/processed IMU, raw/filtered odometry, platform/joint_states |
| Coordinates and maps | Jackal tf, tf_static, map, map_metadata, pose |

The selection contains **21 topics: the original 14 plus 7 camera topics**. Full names appear in the [production recording list](Jackal_RPP_Trial_2026-10-14.md#8-production-recording-save-on-jackal-then-transfer-to-host) and [recording script](../scripts/record_jackal.sh); subscription settings are in [recording QoS](../config/jackal/rosbag_qos.yaml). If the system does not publish `map_metadata` or `pose`, those data will be absent. Use separate [debug recording](Jackal_Debug_Recording_2026-10-13.md) for navigation troubleshooting.

Recording uses MCAP with `zstd_fast` disk compression and splits at approximately 2 GiB per file. **Splitting is not a total-size limit**; recording continues until manually stopped. Compression occurs on the Host and does not reduce wireless traffic from the robot.

### 6.3 Check data after stopping

Press `Ctrl+C` in H2, wait for cache writes and the shell prompt, then run in the **same terminal**:

```bash
ros2 bag info "$jackal_bag"
du -sh "$jackal_bag"
```

At minimum, verify that these topics exist and have `Count` greater than zero:

- `/j100_0949/sensors/lidar3d_0/points` and `/j100_0949/sensors/lidar3d_0/scan`.
- `/j100_0949/sensors/camera_0/color/image` and `/j100_0949/sensors/camera_0/depth/image`.
- Both `camera_info` streams, `/j100_0949/tf`, and `/j100_0949/tf_static`.
- Chassis odometry, plus `/j100_0949/map` if SLAM ran during this session.

`Pausing recording` during shutdown is normal. `Number of messages lost on the transport layer` reports detected receive-side message loss; it differs from manual pausing and is not directly the number of lost image frames. A nonzero Count proves some data was recorded, not lossless coverage throughout.

## 7 Save maps and end the experiment

Cancel the navigation goal and wait for the robot to stop. Save the map **while SLAM is still running** if needed.

**Jackal J0: **

```bash
source /etc/clearpath/setup.bash
jackal_map_dir="$HOME/maps/jackal_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_map_dir"

ros2 run nav2_map_server map_saver_cli \
  -f "$jackal_map_dir/map" --ros-args \
  -r map:=/j100_0949/map

ros2 service call /j100_0949/slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$jackal_map_dir/posegraph'}"

ls -lh "$jackal_map_dir"
```

Confirm successful map saving and pose-graph serialization `result: 0`. Files are under **`~/maps/` on Jackal**, while the rosbag is in the **Host desktop recording directory**. Grid maps support display/localization; resuming the same SLAM session also requires the pose graph.

Shutdown order:

1. **H2:** `Ctrl+C` to stop recording, wait for disk writes, then run `ros2 bag info`.
2. **J2:** `Ctrl+C` to exit Nav2.
3. **J1:** `Ctrl+C` to exit SLAM.
4. **H1:** close RViz.

Clearpath chassis and sensor services may remain running after an everyday experiment. To remap and reset the origin next time, start at section 2. Continuing an old map requires a separate pose-graph loading workflow.

## 8 Common issues

| Symptom | First checks |
| --- | --- |
| Coordinates remain nonzero after service restart | Raw `platform/odom` and `odom → base_link`; do not start a new map until the zeroing check passes |
| No laser scan in RViz | Check whether Topic still uses `lidar2d_0/scan`; select `lidar3d_0/scan` with Best Effort reliability |
| Point-cloud direction correct but scan reversed | Scan header.frame_id, separate lidar frame, and corresponding TF; see section 2.3 |
| RViz cannot find map | First verify SLAM `active [3]`, then map data and Jackal TF; the specified-start chain is `map → odom_start → odom → base_link` |
| SLAM stuck at Configuring with change_state timeout | Update the script using section 9.5, then verify map TF after `SLAM ACTIVE` |
| Old script reports Cannot shutdown a ROS adapter on exit | Wait for old processes to exit and upload the fixed script; this shutdown error is covered by regression checks |
| Nav2 does not move or reports a progress error | Lifecycle states, all three scan.topic settings, costmap obstacles, emergency-stop state, and Collision Monitor; do not increase speed first |
| Speed limits appear ineffective | Verify the custom launch is used and runtime values match section 4.3 |
| rosbag starts in PAUSED | This is the configured waiting state; press Space in H2 |
| Camera or point-cloud data missing from bag | Check actual topic messages, whether recording started, and wireless loss |
| RViz reports GLX or driver errors | Check `nvidia-smi` and `glxinfo -B` on the Host, then resolve the graphics environment |

For more deployment, rollback, and networking details, see the [complete command manual](ROS2_Jackal_TurtleBot_Host_CMD_Manual_2026-10-09.md) and [network topology](ROS2_Multi_Robot_Network_Topology_2026-10-09.md).


## 9 Start mapping at a specified pose without restarting the chassis

For example, define the current pose as **`x=1 m`, `y=0 m`, facing map `+X`**, corresponding to `yaw=0°`. Use the [specified-start SLAM script](../scripts/start_jackal_slam_at_pose.py) without rebooting the OS, restarting the chassis service, or changing the EKF.

This workflow is for **a new map**. Save any old map needed, cancel navigation, stop the robot, then exit Nav2 followed by SLAM. Stop the old rosbag and use a new bag for the new coordinate session. Keep the robot stationary throughout start-pose sampling.

### 9.1 Upload the script from the Host

Run in a **local Host terminal**. The fix is in the Host repository; the identically named Jackal script changes only after uploading it again:

```bash
scp /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/start_jackal_slam_at_pose.py \
  robot@192.168.185.201:/home/robot/
```

### 9.2 Specify the new map start and start SLAM on Jackal

Run in **Jackal J1**. With this command, skip the original chassis-service restart and standard `slam.launch.py` command.

```bash
source /etc/clearpath/setup.bash
python3 "$HOME/start_jackal_slam_at_pose.py" \
  --x 1 --y 0 --yaw-deg 0
```

The script reads current `odom → base_link`, computes the translation/rotation required for the specified start, and starts fresh SLAM. The new version actively queries lifecycle state, performs configure then activate, and displays `SLAM ACTIVE. Check map -> base_link before starting Nav2.` only after confirming `active`. On a service-response timeout, it checks the actual state before repeating a possibly completed transition. Persistent nonresponse times out and closes this session's processes.

Successful startup should show:

```text
SLAM lifecycle: unconfigured
SLAM lifecycle: inactive
SLAM lifecycle: active
SLAM ACTIVE. Check map -> base_link before starting Nav2.
```

Use the final `active` and `SLAM ACTIVE` messages as evidence of activation, then verify map TF with section 9.3. `Starting new SLAM` only means the launch process was created.

Keep this terminal running. `Ctrl+C` stops this SLAM session and its reference-transform publisher; wait for the shell prompt. If old SLAM, Nav2, AMCL, or a previous reference publisher remains running, the script exits and lists conflicting nodes.

The x/y parameters are meters; yaw is degrees: `0` faces map `+X`, `90` faces `+Y`, and `-90` faces `-Y`. To define the origin, use `--x 0 --y 0 --yaw-deg 0`.

To read and preview the calculation only, append `--preview`; this publishes no TF and does not start SLAM.

### 9.3 Verify the new coordinates on Jackal

First check lifecycle state in **Jackal J0**:

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/slam_toolbox
```

Expect `active [3]`. If startup shows only `Configuring`, a `change_state` response timeout, and no `Activating`, follow section 9.5. If the script has exited, `Node not found` is expected; restart it first.

After activation, continue verification:

```bash
source /etc/clearpath/setup.bash
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

Keep the robot stationary; expect output close to:

```text
Translation: [1.000, 0.000, 0.000]
Rotation: in RPY (degree) [0.000, 0.000, 0.000]
```

After checks pass, press `Ctrl+C` to end the query and start the existing low-speed Nav2 using section 4. Close old RViz and reopen it using section 5 so it receives this session's TF; Fixed Frame remains `map`. Create a new rosbag using section 6.

The startup log's `odom_start → odom` values compensate for old odometry and may differ from `(1,0)`. Raw `/j100_0949/platform/odom` and filtered odometry need not show `(1,0)`; this sets the **robot pose in the new map**. Use `map` for subsequent checks and goals.

### 9.4 How it works and returning to the default workflow

This session's TF chain is:

```text
map → odom_start → odom → base_link → LiDAR and cameras
```

The script adds `odom_start` only for this SLAM session and retains chassis `odom → base_link`. It generates temporary settings from the installed Clearpath J100 SLAM parameters, changing this SLAM session's `odom_frame` to `odom_start`; Nav2, LiDAR, and camera configurations remain unchanged. It does not publish an additional `map → odom` that conflicts with SLAM.

Given sampled transform `T_odom_base` and desired start `T_target_base`, the reference transform is `T_start_odom = T_target_base × inverse(T_odom_base)`. The current robot pose therefore has the requested coordinates in the mapping reference frame. SLAM initializes its new map from that pose; see [SLAM Toolbox pose initialization](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp).

The script passed transform checks and a real SLAM-node test in an isolated ROS Domain: synthetic odometry started at `(4,-2,90°)`, the new map pose was approximately `(1,0,0°)`, and original odometry was unchanged. Added regression checks cover lost lifecycle responses, timeouts, and duplicate shutdown. Field acceptance still depends on state, TF, and physical motion-direction checks in section 9.3.

To return to the default sections 2/3 workflow, exit Nav2, press `Ctrl+C` to stop this script, close old RViz, and then start the default workflow. Do not run both SLAM entry points simultaneously.


### 9.5 Activation timeout and updating the old script

The old field log stopped at `Configuring` and reported:

```text
failed to send response to /j100_0949/slam_toolbox/change_state (timeout)
```

`map` then remained unavailable, and shutdown reported `Cannot shutdown a ROS adapter that is not running`. These logs show incomplete automatic startup; they do not establish an error in the `(1,0,0°)` coordinate compensation. The underlying communication cause of the service-response timeout remains unknown.

The fix queries node state before deciding whether to configure or activate. After a response timeout, it checks whether the transition occurred instead of waiting indefinitely or blindly retrying. Persistent failure ends this startup and reports an error. Launch handles forwarding the interrupt signal during shutdown to avoid duplicate shutdown triggers.

Update sequence:

1. **Jackal J1:** if the old script is running, press `Ctrl+C` once and wait for the prompt. Skip if it has already exited.
2. **Local Host terminal:** run the section 9.1 `scp` to replace Jackal's script with the fix.
3. **Jackal J1:** run the section 9.2 command, keep the robot stationary, and wait for `SLAM ACTIVE`.
4. **Jackal J0:** follow section 9.3 to confirm `active [3]` and `map → base_link` near `(1,0,0°)`.
5. After verification, use sections 4, 5, and 6 to start Nav2, reopen RViz, and create a new rosbag.

If the new version still times out, retain the complete startup log and last observed lifecycle state. If it has exited and closed the node, do not send activate requests to that exited node. If already `active` but `map` remains missing, check the following while the node is still running:

```bash
source /etc/clearpath/setup.bash
ros2 param get /j100_0949/slam_toolbox odom_frame
ros2 param get /j100_0949/slam_toolbox scan_topic
ros2 topic echo /j100_0949/sensors/lidar3d_0/scan \
  --once --field header --qos-reliability best_effort
ros2 run tf2_ros tf2_echo odom_start base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

The first two values should be `odom_start` and `/j100_0949/sensors/lidar3d_0/scan`, and scan messages should arrive. With no robot movement, the final check should be near `(1,0,0°)`; press `Ctrl+C` afterward. If scan and this TF are normal but the map remains missing, inspect SLAM logs for LiDAR-frame and message-filter errors.

### 9.6 Verification status at this stage

| Item | Confirmed result |
| --- | --- |
| Zeroing after chassis-service restart | User field output confirmed raw odometry and `odom → base_link` near zero |
| Old specified-start script | Ran on Jackal; response timed out during configuration; map-pose verification was incomplete |
| Fixed-script regression checks | 10 passed, including lost responses, state timeout, transforms, and duplicate shutdown |
| Fixed-script isolated SLAM test | Activated in local Domain 211; map pose approximately `(1,0,0°)`; raw odometry unchanged; both child processes exited normally |
| Fixed-script field acceptance | Awaiting upload/run and confirmation of `active [3]`, map pose, and physical motion direction |
