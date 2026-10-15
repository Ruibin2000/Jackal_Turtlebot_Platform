# Jackal Operating Manual: Start Pose, SLAM, RPP Navigation, and Production Recording

Candidate configuration: [nav2_low_speed_indoor_rpp.yaml](../config/jackal/nav2_low_speed_indoor_rpp.yaml). This is a complete YAML file with Rotation Shim wrapping RPP in place of MPPI. The `114801` field recording confirmed that RPP was active and all 5 goals succeeded; position tolerance was still 0.25 m in that run. The current Host file now uses the user's selected 0.10 m tolerance, pending deployment and field testing.

This comparison checks whether geometric path tracking reduces the left-right corrections seen during MPPI forward motion. See the [112127 bag analysis](../analysis/jackal_20261014_112127/REPORT.md). Retain the original MPPI file for rollback; RPP cannot be assumed to resolve all low-speed wheel stalling or slipping.

**Everyday sequence: section 3.1 sets the start pose and starts SLAM → section 4 starts Nav2 → section 4.1 opens Host RViz → section 4.2 checks LiDAR data → section 8 records onboard Jackal → send goals.** Complete sections 2–3 first when selecting or updating RPP. Use the compact GLIM + Camera profile in section 8 for production collection.

| Terminal | Location | Purpose |
| --- | --- | --- |
| J1 | Jackal SSH | Start pose, SLAM, SLAM logs |
| J2 | Jackal SSH | Coordinate checks, Nav2, Nav2 logs |
| H1 | Local Host | RViz |
| J3 | Jackal SSH / tmux | Sensor checks, onboard production recording |
| H2 | Local Host | Transfer and verify recordings |

Open separate Host terminals for J1 and J2, log in to Jackal, then run the corresponding Jackal commands:

```bash
ssh robot@192.168.185.201
```

## 1. Key settings

| Item | RPP comparison value | Meaning |
| --- | --- | --- |
| `FollowPath.primary_controller` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | Replaces the internal MPPI controller |
| `desired_linear_vel` | `0.06 m/s` | Desired forward tracking speed; may decrease on curves, near obstacles, or near the goal |
| `lookahead_dist` / `curvature_lookahead_dist` | `0.6 m` / `0.6 m` | Fixed lookahead distances that do not shrink to very short values at low speed |
| `use_fixed_curvature_lookahead` / `interpolate_curvature_after_goal` | `true` / `true` | Samples curvature at a fixed distance and extends the final path direction for curvature calculation near the endpoint |
| `regulated_linear_scaling_min_radius` | `0.75 m` | Enables curvature-based slowing below this turn radius; not the chassis minimum turning radius |
| `regulated_linear_scaling_min_speed` | `0.03 m/s` | Minimum speed during curvature/cost regulation; goal-approach regulation may still reduce it to `0.02 m/s` |
| `use_rotate_to_heading` / `allow_reversing` | `false` / `false` | RPP tracks forward; the outer Shim handles large heading errors and final goal orientation |
| Shim engagement / disengagement heading error | `0.35 / 0.17 rad`, approximately `20° / 10°` | Retains the previous thresholds and allows realignment after deviation |
| Shim angular speed / acceleration | `0.10 rad/s` / `0.30 rad/s²` | Retains the previous settings |
| RPP collision prediction time | `3.0 s`, capped at the lookahead point | Selected for low speed and `0.06 m` map resolution; not a fixed safety distance in meters |
| Velocity smoother | `scale_velocities: true` | Scales acceleration/deceleration increments together; absolute velocity limits still apply per axis |
| Shared velocity limits | Forward `0.20 m/s`, reverse `0.10 m/s`, turning `±0.10 rad/s` | RPP desired tracking speed is only `0.06 m/s`; the shared limits also serve recovery behaviors |
| Progress / goal tolerances | Displacement `0.20 m` OR rotation `0.35 rad` / `30 s`; goal **`0.10 m`** / `0.25 rad` | Progress and heading tolerances are unchanged; position tolerance is tightened from 0.25 m |
| Footprint / padding / inflation | `±0.26 × ±0.22 m` / `0.02 m` / `0.6 m` | Matches the confirmed indoor configuration |

RPP computes curvature from the lookahead point, then derives angular velocity from linear velocity. This version has no MPPI `wz_max` parameter. The candidate uses lookahead distance and curvature-based slowing to reduce excessive turning; the smoother still enforces the final shared angular limit. `scale_velocities` does not guarantee a constant `v/ω` ratio through every limit or transition. [RPP implementation](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp), [velocity smoother implementation](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_velocity_smoother/src/velocity_smoother.cpp)

RPP's `allow_reversing: false` only restricts RPP tracking; the existing BT can still trigger BackUp. Collision monitoring, recovery behaviors, SLAM, LiDAR ring 15, and chassis wheel separation/radius remain unchanged. The initial RPP comparison changed both the internal `FollowPath` configuration and `scale_velocities`, so it was not a strict single-value experiment. The next run after `114801` only tightens `general_goal_checker.xy_goal_tolerance`, retaining these tracking settings.

## 2. Upload the configuration and start-pose script — Host

Cancel the navigation goal and stop the robot. End the previous recording and wait for `params_after` to finish saving, then press `Ctrl+C` in Jackal's Nav2 terminal. SLAM may remain running when only changing the navigation controller.

```bash
cd /home/nyu/Desktop/Jackal_Turtlebot_Platform
scp config/jackal/nav2_low_speed_indoor_rpp.yaml \
  robot@192.168.185.201:/home/robot/jackal_nav2/
scp scripts/start_jackal_slam_at_pose.py robot@192.168.185.201:/home/robot/
```

## 3. Check, back up, and select RPP — Jackal

The existing launcher reads `~/jackal_nav2/nav2_low_speed.yaml`; uploading a candidate alone does not change the startup configuration. After confirming a successful upload, run:

```bash
source /etc/clearpath/setup.bash
(
  set -e
  ros2 pkg prefix nav2_regulated_pure_pursuit_controller
  cd "$HOME/jackal_nav2"
  test -s nav2_low_speed.launch.py
  test -s nav2_low_speed.yaml
  test -s nav2_low_speed_indoor_rpp.yaml
  test -s behavior_trees/navigate_to_pose_indoor_recovery.xml
  test -s behavior_trees/navigate_through_poses_indoor_recovery.xml
  python3 - <<'PY'
import yaml
from pathlib import Path
p = yaml.safe_load(Path('nav2_low_speed_indoor_rpp.yaml').read_text())
f = p['controller_server']['ros__parameters']['FollowPath']
assert f['primary_controller'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
assert f['desired_linear_vel'] == 0.06
print('RPP candidate parsed OK')
PY
  jackal_rpp_backup=$(mktemp -d "$PWD/nav2_before_rpp.XXXXXX")
  cp -a nav2_low_speed.yaml "$jackal_rpp_backup/nav2_low_speed.yaml"
  cp nav2_low_speed_indoor_rpp.yaml nav2_low_speed.yaml
  printf '%s\n' "$jackal_rpp_backup" > rpp_backup_path.txt
  printf 'Previous YAML: %s/nav2_low_speed.yaml\n' "$jackal_rpp_backup"
)
```

If the first check cannot find the RPP package, the subshell above stops without replacing the old YAML. Install the package on Jackal, then rerun this section:

```bash
sudo apt install ros-jazzy-nav2-regulated-pure-pursuit-controller
```

RPP `1.3.12` has been verified on this Host; use the following startup logs and readback to establish what Jackal actually loads. Plugin changes require restarting Nav2; changing only the plugin-name string with `ros2 param set` is insufficient.

## 3.1 Set the current pose and start SLAM — Jackal J1

This section starts a new map. Cancel the old goal, stop the robot, end the previous recording, then exit old Nav2 followed by old SLAM. Save any old map you need first. Keep chassis and sensor services running. If retaining an already active SLAM session and map while restarting only Nav2, skip to section 4.

Park at this session's start location and remain stationary during initialization and coordinate checks. This example defines the **current pose as `(x=1 m, y=0 m, yaw=0°)` in the new map**, facing map `+X`, and saves SLAM startup/runtime logs.

Run in **J1**, already logged in to Jackal:

```bash
source /etc/clearpath/setup.bash
(
  set -e
  set -o pipefail
  test -s "$HOME/start_jackal_slam_at_pose.py"
  jackal_slam_log="$HOME/jackal_nav2/debug_logs/slam_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$jackal_slam_log"
  cp -a "$HOME/start_jackal_slam_at_pose.py" "$jackal_slam_log/"
  ROS_LOG_DIR="$jackal_slam_log/ros" RCUTILS_COLORIZED_OUTPUT=0 \
    python3 -u "$HOME/start_jackal_slam_at_pose.py" --x 1 --y 0 --yaw-deg 0 \
    2>&1 | tee -i "$jackal_slam_log/console.log"
)
```

`--x / --y` are in meters; `--yaw-deg` is in degrees. To use the current pose as the origin, set `--x 0 --y 0 --yaw-deg 0`; `90°` faces `+Y` and `-90°` faces `-Y`. This sets the new map's start pose without rebooting and does not drive the robot to those coordinates.

Wait for **`SLAM ACTIVE. Check map -> base_link before starting Nav2.`** and keep J1 running. This script already starts `slam_toolbox`; do not start a second SLAM instance.

Check in **J2**, already logged in to Jackal:

```bash
source /etc/clearpath/setup.bash
ros2 lifecycle get /j100_0949/slam_toolbox
ros2 run tf2_ros tf2_echo map base_link --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

Expect `active [3]`, followed by translation near `[1.000, 0.000, 0.000]` and yaw near `0°`. Press **Ctrl+C to stop only J2's TF query**; keep J1 running. If `map` remains unavailable or SLAM is not active, inspect J1's startup log before starting Nav2. Once checks pass, continue with section 4 in J2.

## 4. Start Nav2 and save logs — Jackal J2

Complete the checks in section 3.1 and keep SLAM active in J1. Run the following in J2:

```bash
source /etc/clearpath/setup.bash
(
  set -e
  set -o pipefail
  jackal_rpp_log="$HOME/jackal_nav2/debug_logs/nav_rpp_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$jackal_rpp_log"
  cp "$HOME/jackal_nav2/nav2_low_speed.yaml" "$jackal_rpp_log/"
  cp "$HOME/jackal_nav2/nav2_low_speed.launch.py" "$jackal_rpp_log/"
  cp -a "$HOME/jackal_nav2/behavior_trees" "$jackal_rpp_log/"
  ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
    setup_path:=/etc/clearpath \
    scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
    use_sim_time:=false 2>&1 | tee "$jackal_rpp_log/console.log"
)
```

Wait for `Managed nodes are active`. Read back parameters in another Jackal terminal:

```bash
source /etc/clearpath/setup.bash
ros2 param get /j100_0949/controller_server FollowPath.primary_controller
ros2 param get /j100_0949/controller_server FollowPath.desired_linear_vel
ros2 param get /j100_0949/controller_server FollowPath.use_rotate_to_heading
ros2 param get /j100_0949/controller_server FollowPath.rotate_to_heading_angular_vel
ros2 param get /j100_0949/velocity_smoother max_velocity
ros2 param get /j100_0949/velocity_smoother scale_velocities
ros2 param get /j100_0949/controller_server general_goal_checker.xy_goal_tolerance
```

Expect, in order: the full RPP plugin name, `0.06`, `False`, `0.1`, `[0.2, 0.0, 0.1]`, `True`, and **`0.1`**. This run no longer sets `PathAngleCritic`, `wz_max`, or MPPI `visualize`.

## 4.1 Open visualization — Host H1

Run locally on **Host H1**:

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

RViz settings:

| Item | Setting |
| --- | --- |
| Global Options → Fixed Frame | `map` |
| Map → Topic | `/j100_0949/map`; Reliable, Transient Local |
| LaserScan → Topic | `/j100_0949/sensors/lidar3d_0/scan`; Best Effort |
| Optional PointCloud2 → Topic | `/j100_0949/sensors/lidar3d_0/points`; Best Effort |

Select `lidar3d_0/scan` for LaserScan, retaining confirmed ring 15. This workflow uses SLAM mapping, so `2D Pose Estimate` is unnecessary. Start recording in section 8 and press Space first; then use **Nav2 Goal** in RViz to click the target position and drag the arrow to set goal orientation.



## 4.2 Check LiDAR IMU and point clouds before recording

Check **locally on Jackal** first. For onboard recording in section 8, Jackal's results are the relevant baseline; **Host reception** comparisons help troubleshoot networking and RViz. Keep J1/J2 running and open another SSH terminal for the commands below. Do not record during these checks. Close extra PointCloud2 and raw-camera previews on the Host, and perform each check separately to avoid interference from multiple large-data subscriptions.

**Jackal: inspect publishers and actually receive one message.**

```bash
source /etc/clearpath/setup.bash
ros2 topic info -v /j100_0949/sensors/lidar3d_0/imu/data_raw
ros2 topic info -v /j100_0949/sensors/lidar3d_0/points

ros2 topic echo /j100_0949/sensors/lidar3d_0/imu/data_raw \
  sensor_msgs/msg/Imu --once --timeout 10 \
  --qos-reliability best_effort --qos-durability volatile

ros2 topic echo /j100_0949/sensors/lidar3d_0/points \
  sensor_msgs/msg/PointCloud2 --field header --once --timeout 10 \
  --qos-reliability best_effort --qos-durability volatile
```

`Publisher count` should exceed 0, but only confirms publisher discovery. The IMU should then print one message, and the point cloud should print `stamp` and `frame_id`; only the cloud header is printed to avoid flooding the terminal. A 10-second timeout means no message was received during that check, not that IMU measurements are zero.

**Jackal: check each received rate for approximately 20 seconds.**

```bash
timeout --signal=INT 20s ros2 topic hz \
  /j100_0949/sensors/lidar3d_0/points --window 100
timeout --signal=INT 20s ros2 topic hz \
  /j100_0949/sensors/lidar3d_0/imu/data_raw --window 100
```

Both checks should continuously display `average rate`; also watch for unusually long `max` intervals. The expected IMU rate depends on the actual driver configuration. A normal `timeout` expiration usually returns `124`, which does not indicate a sensor fault.

**Optional Host comparison: repeat the rate checks locally in H2.**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0

timeout --signal=INT 20s ros2 topic hz \
  /j100_0949/sensors/lidar3d_0/points --window 100
timeout --signal=INT 20s ros2 topic hz \
  /j100_0949/sensors/lidar3d_0/imu/data_raw --window 100
```

`hz` measures the current terminal's **received rate**, which depends on networking, QoS, and local load. This Jazzy `hz` uses sensor-data QoS; do not add `--qos-reliability`. [ROS 2 Jazzy implementation](https://github.com/ros2/ros2cli/blob/jazzy/ros2topic/ros2topic/verb/hz.py)

| Result | Next step |
| --- | --- |
| No publisher on Jackal | Check the actual topic name, driver, and ROS environment |
| Publisher exists, but Jackal echo times out and hz shows no data | Investigate the driver data stream and QoS; topic presence alone is insufficient to start production collection |
| Jackal receives continuously, but Host reception is low or absent | Check cross-machine networking, DDS, and Host reception load first; this does not yet isolate the router as the sole cause |
| Both receive normally, but the bag has missing frames | Check recording QoS, load, and disk; verify with the camera-inclusive short bag in section 8.1 |

If the LiDAR IMU has no data, inspect actual output topics on **Jackal**:

```bash
ros2 topic list -t | rg 'lidar3d_0|imu'
ros2 node info /j100_0949/sensors/lidar3d_0/ouster_driver
```

The LiDAR IMU is `lidar3d_0/imu/data_raw`; the body IMU is `imu_0/data_raw`. Body IMU data does not prove the LiDAR IMU is working. Finish all rate checks before recording. Even if individual subscriptions work, use a camera-inclusive short bag to check for severe frame loss under the full collection load.

## 8. Production recording: save on Jackal, then transfer to Host

Complete the **local Jackal** LiDAR IMU / points checks in section 4.2 and confirm that cameras are publishing. Bags are saved under the robot's `~/jackal_bags/recording/` and transferred to the Host afterward using section 8.2. Retain the existing **21 topics**, MCAP format, `zstd_fast` compression, and approximately 2 GiB splits.

The supplied storage report shows an NVMe SSD with approximately **431 GiB** available. The script checks user-available space on the target directory's filesystem in GiB (1024³ bytes):

| Limit | Default behavior |
| --- | --- |
| Free space ≤ 100 GiB at startup or after the first Space press | Remind the operator to transfer and verify old bags or manually delete unwanted data; refuse to start |
| Free space drops to ≤ 100 GiB while recording | Check every 5 seconds; send SIGINT for a clean stop and wait for cache/metadata writes |
| Session reaches 60 minutes | Stop cleanly; timing starts when the first Space press launches the recorder and includes subsequent pauses |
| Individual MCAP reaches approximately 2 GiB | Split automatically and continue recording; this is not a total recording-size limit |

100 GiB is the reserved-space threshold. The 5-second check interval and cache writes after stopping may leave slightly less than that amount. The script does not delete bags automatically or force-kill the recorder. The reported free space leaves approximately 331 GiB before the threshold, but actual recording duration must be estimated from an onboard short bag. `--max-bag-size` and `--max-bag-duration` are both splitting options; this script manages the total session stop. [rosbag2 splitting documentation](https://github.com/ros2/rosbag2/tree/jazzy#splitting-files-during-recording)

**Initial deployment or script update: run on the Host.**

```bash
cd /home/nyu/Desktop/Jackal_Turtlebot_Platform
ssh robot@192.168.185.201 'mkdir -p ~/jackal_recorder/scripts ~/jackal_recorder/config/jackal'
scp scripts/record_jackal.sh scripts/guard_jackal_recording.py \
  robot@192.168.185.201:/home/robot/jackal_recorder/scripts/
scp config/jackal/rosbag_qos.yaml \
  robot@192.168.185.201:/home/robot/jackal_recorder/config/jackal/
```

**Jackal J3: open another SSH terminal and enter tmux.**

```bash
ssh robot@192.168.185.201
```

Run after login; if `tmux` / `rsync` are missing, first run `sudo apt install tmux rsync`:

```bash
tmux new-session -A -s jackal_record
```

Run the following inside tmux. If recording is still running when you reconnect, continue using that session rather than starting another:

```bash
source /etc/clearpath/setup.bash
mkdir -p "$HOME/jackal_bags/recording"
df -h "$HOME/jackal_bags/recording"
jackal_bag="$HOME/jackal_bags/recording/jackal_rpp_$(date +%Y%m%d_%H%M%S)"
bash "$HOME/jackal_recorder/scripts/record_jackal.sh" --onboard "$jackal_bag"
```

Expect the free-space report, `Profile: GLIM + camera (21 topics)`, and **PAUSED**. Press **Space** in J3 to start, wait for topic subscriptions, then send navigation goals. Subsequent Space presses pause/resume; **Ctrl+C stops and saves**. No subscriptions are created before the first Space press, avoiding early consumption of one-time static TF/calibration messages.

To detach temporarily, press **Ctrl+B, release, then D**; recording continues. After an SSH disconnect, log in again and run `tmux attach -t jackal_record`. Here tmux protects only the recording session; SLAM/Nav2 in J1/J2 still run in their original terminals. [tmux session documentation](https://github.com/tmux/tmux/wiki/Getting-Started)

To limit a session to 30 minutes, prefix the recording command with `JACKAL_MAX_RECORD_SECONDS=1800`; `0` disables the time limit while retaining space protection. `JACKAL_MIN_FREE_GIB` adjusts the 100 GiB threshold; retain the default initially.

This profile records data actually published on these 21 topics:

| Data | Topic |
| --- | --- |
| Full LiDAR point cloud | `/j100_0949/sensors/lidar3d_0/points` |
| 2D scan, retaining ring 15 | `/j100_0949/sensors/lidar3d_0/scan` |
| Raw LiDAR IMU | `/j100_0949/sensors/lidar3d_0/imu/data_raw` |
| LiDAR metadata | `/j100_0949/sensors/lidar3d_0/metadata` |
| Body IMU / raw IMU | `/j100_0949/sensors/imu_0/data`, `/j100_0949/sensors/imu_0/data_raw` |
| Chassis odometry / filtered odometry | `/j100_0949/platform/odom`, `/j100_0949/platform/odom/filtered` |
| Wheel joint states | `/j100_0949/platform/joint_states` |
| TF / static TF | `/j100_0949/tf`, `/j100_0949/tf_static` |
| Map / map metadata / pose | `/j100_0949/map`, `/j100_0949/map_metadata`, `/j100_0949/pose` |
| RGB image / intrinsics / metadata | `/j100_0949/sensors/camera_0/color/image`, `/j100_0949/sensors/camera_0/color/camera_info`, `/j100_0949/sensors/camera_0/color/metadata` |
| Depth image / intrinsics / metadata | `/j100_0949/sensors/camera_0/depth/image`, `/j100_0949/sensors/camera_0/depth/camera_info`, `/j100_0949/sensors/camera_0/depth/metadata` |
| Depth-to-RGB extrinsics | `/j100_0949/sensors/camera_0/extrinsics/depth_to_color` |

Chassis odometry, joint states, map, and pose are retained from the original list. The full debug set of motor feedback, diagnostics, costmaps, navigation actions, velocity commands, and parameter snapshots is excluded. Save SLAM/Nav2 console logs separately using sections 3.1 / 4. `--onboard` selects the Jackal environment and space protection; the topic list remains at 21. The script still supports the legacy Host invocation.

Standard GLIM LiDAR–IMU mapping uses full 3D point clouds and one matching IMU stream. The ring-15 2D scan serves current SLAM/Nav2 and cannot replace the full cloud. Odometry, maps, and similar original topics are supplementary records, not required inputs for this GLIM pipeline. [GLIM input configuration](https://github.com/koide3/glim/blob/master/docs/quickstart.md)

Retain both raw IMU streams for offline selection. **GLIM's `imu_topic` must match its LiDAR–IMU extrinsics; recording both streams does not automatically fuse them.** The script records PointCloud2 and images unchanged, without downsampling. Per-point time fields and units must match GLIM settings; TF and LiDAR metadata do not automatically replace GLIM's extrinsic configuration. [GLIM sensor parameters](https://github.com/koide3/glim/blob/master/docs/parameters.md)

Shutdown order: **cancel the goal and stop the robot → press Ctrl+C in J3 → wait for disk writes and the shell prompt → check the bag → exit Nav2 / SLAM as needed → transfer to the Host.** Reaching the space or time limit stops only recording; it neither cancels navigation goals nor stops SLAM/Nav2.

Check in the original **Jackal J3** terminal:

```bash
ros2 bag info "$jackal_bag"
du -sh "$jackal_bag"
df -h "$HOME/jackal_bags/recording"
printf 'Saved recording: %s\n' "$jackal_bag"
```

`metadata.yaml` and all `.mcap` files are in the same `$jackal_bag` directory; do not append `/bag`. Confirm sufficient messages for point clouds, the IMU selected for GLIM, and RGB/depth images. Low counts for static TF, calibration, and LiDAR metadata can be normal. Investigate zero IMU messages or very sparse point clouds first; onboard recording cannot recover data the driver never published. Start a new bag after each map-origin reset.

### 8.1 Optional: record a 20-second onboard test bag first

Run **inside tmux in Jackal J3**, without a simultaneous production recording:

```bash
source /etc/clearpath/setup.bash
jackal_test_dir="$HOME/jackal_bags/recording/jackal_rpp_test_$(date +%Y%m%d_%H%M%S)"
JACKAL_MAX_RECORD_SECONDS=20 \
  bash "$HOME/jackal_recorder/scripts/record_jackal.sh" --onboard "$jackal_test_dir"
ros2 bag info "$jackal_test_dir"
du -sh "$jackal_test_dir"
```

Press **Space** to start. Recording stops automatically and writes to disk after approximately 20 seconds, using the same 21 topics and space protection. Startup discovery makes the actual data span slightly shorter. Estimate future disk requirements from this short bag's size/duration and check cloud, IMU, and image counts. Do not estimate full-data storage needs from the previous Host bag with severe frame loss.

### 8.2 Transfer to Host after recording and verify

Confirm that Jackal has finished writing. Run locally in **Host H2**, entering the actual bag directory name without the `/home/robot/...` prefix:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/Desktop/jackal_rosbag_repo/recording"
df -h "$HOME/Desktop/jackal_rosbag_repo/recording"
read -r -p 'Jackal bag directory name: ' jackal_bag_name
```

Confirm sufficient Host space for the entire bag before transferring. The name check below accepts this section's production and test bags:

```bash
(
  set -e
  [[ "$jackal_bag_name" =~ ^jackal_rpp_(test_)?[0-9]{8}_[0-9]{6}$ ]] || {
    printf 'Enter the actual jackal_rpp_TIMESTAMP directory name.\n' >&2
    exit 1
  }
  jackal_remote_bag="robot@192.168.185.201:/home/robot/jackal_bags/recording/$jackal_bag_name"
  jackal_host_bag="$HOME/Desktop/jackal_rosbag_repo/recording/$jackal_bag_name"
  rsync -avh --partial --info=progress2 \
    "$jackal_remote_bag" "$HOME/Desktop/jackal_rosbag_repo/recording/"
  ros2 bag info "$jackal_host_bag"
  rsync -rcn --itemize-changes "$jackal_remote_bag/" "$jackal_host_bag/"
)
```

If transfer is interrupted, rerun the same commands; `--partial` retains partial files for subsequent synchronization. The final command verifies file contents. **No listed file differences and no errors** means the source files and Host copies match; verification itself does not modify them. [rsync options](https://download.samba.org/pub/rsync/rsync.1)

Decide whether to manually delete old Jackal bags only after transfer verification passes and `ros2 bag info` works on the Host. These commands only copy and check; they do not delete robot data.
