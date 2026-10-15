# Jackal Navigation Debug Recording and Log Capture

Purpose: save data for analyzing turn overshoot, stopping, and backup recovery to **`~/Desktop/jackal_rosbag_repo/debug/` on the Host**. Use a separate directory for each experiment, containing the rosbag, runtime parameters, and recorder log; retrieve Jackal's Nav2 / SLAM startup logs afterward.

## 1. Start debug recording on the Host

Start SLAM and Nav2 using the [quick-start instructions](Jackal_Quick_Start_2026-10-13.md). Keep the robot stationary and do not send a goal yet. Run in a local Host terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
jackal_debug_dir="$HOME/Desktop/jackal_rosbag_repo/debug/jackal_$(date +%Y%m%d_%H%M%S)"
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal_debug.sh "$jackal_debug_dir"
```

The script saves runtime parameters first, then displays **PAUSED**. Press **Space** in this terminal to start, and wait for recording/subscription messages before sending goals from RViz. Subsequent Space presses pause/resume; `Ctrl+C` stops recording. Motion data is not cached while waiting before the first start.

**Keep Nav2 and SLAM running when ending recording.** After `Ctrl+C`, the script collects final parameters and exits only after displaying `Saved: ...`. Stop Nav2 and SLAM afterward.

The following tuning data is saved by default; topics contain data only if the running system publishes them:

| Data | Purpose |
| --- | --- |
| `cmd_vel_nav`, `cmd_vel_smoothed`, `cmd_vel`, chassis velocity commands | Compare controller output, smoothed output, and chassis input |
| Wheel joint states, `platform/motors/cmd_drive`, `platform/motors/feedback` | Compare left/right setpoints, encoder speeds, currents, and drive faults; investigate low-speed wheel stalls |
| Raw / filtered odometry, raw / filtered IMU, TF | Compare wheel-based estimates, body rotation, and localization; encoders cannot independently measure lateral slip |
| `/diagnostics`, `/j100_0949/diagnostics` | Retain chassis and sensor diagnostics |
| LiDAR `/scan`, map, local / global costmaps and footprints | Inspect obstacles, body geometry, and collision costs |
| `/plan`, available local debug trajectories | Compare planned paths with actual motion |
| Feedback / status for navigation, FollowPath, Spin, BackUp, and other actions | Align tracking failures, rotations, and reverse-motion intervals |
| `/rosout`, parameter events, collision-monitor state, available BT logs | Retain runtime warnings, recovery actions, and parameter changes |

Full point clouds and raw images are excluded by default. To include LiDAR point clouds, RGB/depth, and camera calibration, add `--with-sensors` to the script command and use a new directory:

```bash
jackal_debug_dir="$HOME/Desktop/jackal_rosbag_repo/debug/jackal_$(date +%Y%m%d_%H%M%S)"
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal_debug.sh \
  --with-sensors "$jackal_debug_dir"
```

Transport message loss has occurred previously, so start with the default dataset for control tuning. MPPI `visualize` is `false` in the indoor YAML, so optimal-trajectory visualization topics may have no messages; the script does not change this setting. Enabling trajectory visualization increases controller load. See the [Nav2 MPPI documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/#visualize).

## 2. Where parameters are saved

```text
debug/jackal_DATE_TIME/
├── bag/                       # MCAP and metadata.yaml
├── params_before/             # Runtime parameters before recording
│   ├── all_parameters.yaml
│   ├── snapshot_status.yaml
│   └── per-node.yaml, nodes.txt, topics.txt, actions.txt
├── params_after/              # Runtime parameters after recording stops
├── recorder.log               # Recorder output, including message-loss warnings
├── bag_info.txt
├── requested_topics.txt
└── recording_qos.yaml
```

Parameters are read from running nodes, including MPPI, progress/goal checkers, the planner, velocity smoother, collision monitor, both costmaps, behavior server, BT Navigator, SLAM, and readable Ouster/chassis nodes. Startup YAML values may be overridden; analyze the runtime values. Snapshots now also explicitly query `platform_velocity_controller`, `j100_hardware`, and `j100_hardware_interface` to inspect wheel-separation multipliers, wheel radius, chassis limits, and readable drive settings. These nodes are outside the `/platform/` namespace and were not automatically selected by the old script.

`*.failed.txt` means no valid parameters were read from that node; check `missing_required_nodes` in `snapshot_status.yaml`. If an uninitialized parameter makes a batch read return empty, the script splits requests, saves the remaining readable parameters, and lists unreadable entries in `*.unreadable.yaml` and `partially_read_nodes`. An empty `missing_required_nodes` means core nodes have snapshots; only `complete: true` for a node means all its listed parameters were read. The script only reads parameters and does not initialize missing values.

One bag per configuration is easiest to compare. If parameters change during recording, wait until the changes finish, then save an additional snapshot in **another terminal with the Host ROS environment loaded**. Replace the path with the current recording directory:

```bash
bash /home/nyu/Desktop/Jackal_Turtlebot_Platform/scripts/record_jackal_debug.sh \
  --snapshot-only "$HOME/Desktop/jackal_rosbag_repo/debug/jackal_ACTUAL_DATE_TIME"
```

You can also run `--snapshot-only "$jackal_debug_dir"` after the recording terminal exits. Snapshots are collected node by node, so avoid tuning parameters concurrently. Action feedback/status recordings are not a complete replay of action service requests/results, and ordinary topic recording does not automatically capture a full ROS parameter snapshot.

## 3. Save SLAM startup logs on Jackal

Use this for the **next startup**, replacing the original launch command. Stop the old Nav2/SLAM first; logging does not require a second set of nodes. In Jackal terminal J1:

```bash
source /etc/clearpath/setup.bash
jackal_slam_log="$HOME/jackal_nav2/debug_logs/slam_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$jackal_slam_log"
cp -a "$HOME/start_jackal_slam_at_pose.py" "$jackal_slam_log/"
set -o pipefail
ROS_LOG_DIR="$jackal_slam_log/ros" RCUTILS_COLORIZED_OUTPUT=0 \
  python3 -u "$HOME/start_jackal_slam_at_pose.py" --x 1 --y 0 --yaw-deg 0 \
  2>&1 | tee -i "$jackal_slam_log/console.log"
```

Wait for `SLAM ACTIVE` as before, and check the start pose in another Jackal terminal before starting Nav2. Logs include the requested pose, lifecycle startup, SLAM output, and shutdown messages. Adjust `x/y/yaw` for this experiment.

## 4. Save Nav2 logs and configuration files on Jackal

This section applies to both **indoor** and **indoor + Plan B**. The custom launcher always reads `~/jackal_nav2/nav2_low_speed.yaml`; this is the common active-configuration filename, and the actual version depends on what was copied there during deployment:

| Source selected during deployment | File read by the launcher | Configuration |
| --- | --- | --- |
| `nav2_low_speed_indoor.yaml` | `nav2_low_speed.yaml` | Indoor baseline |
| `nav2_low_speed_indoor_plan_b.yaml` | `nav2_low_speed.yaml` | indoor + Rotation Shim (Plan B) |

For Plan B, first complete [Plan B upload, backup, and replacement](Jackal_Quick_Start_2026-10-13.md#8-plan-b-add-rotation-shim), then use the logging launch command below. Uploading a candidate alone does not switch the active configuration. SLAM may remain running; confirm old Nav2 has exited before running this in Jackal J2:

```bash
source /etc/clearpath/setup.bash
jackal_nav_log="$HOME/jackal_nav2/debug_logs/nav_$(date +%Y%m%d_%H%M%S)"
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

**The configuration saved here is the same active file that is loaded.** If Plan B was copied to `nav2_low_speed.yaml`, this starts indoor + Plan B; logging does not restore an older configuration. `config/` saves the YAML, launch file, and behavior trees used at startup; the Host script separately saves runtime parameters. Retain default INFO / WARN / ERROR logs; there is no need to switch the entire system to DEBUG.

After Plan B starts, verify the outer and inner controllers in another Jackal terminal:

```bash
source /etc/clearpath/setup.bash
ros2 param get /j100_0949/controller_server FollowPath.plugin
ros2 param get /j100_0949/controller_server FollowPath.primary_controller
```

Expect `nav2_rotation_shim_controller::RotationShimController` and `nav2_mppi_controller::MPPIController`, respectively. See the Plan B deployment instructions above for full parameter readback.

If Nav2/SLAM are already running, `/rosout` in a debug bag cannot guarantee recovery of logs from before recording began. Earlier automatic ROS logs are usually under Jackal's `~/.ros/log/`; use the directory printed by the original launch terminal. `tee` cannot recover previously unsaved terminal output. ROS log paths are controlled by settings such as `ROS_LOG_DIR`; see [ROS 2 logging](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Logging.rst).

## 5. Finish, retrieve logs, and check

Cancel the goal and stop the robot → press `Ctrl+C` in the Host recording terminal and wait for `Saved` → exit Nav2 and then SLAM on Jackal, waiting for logs to finish writing. Run in the **original Host recording terminal**:

```bash
# Retrieve existing debug_logs sessions; match nav_ / slam_ timestamps to this experiment.
scp -r robot@192.168.185.201:/home/robot/jackal_nav2/debug_logs \
  "$jackal_debug_dir/robot_logs"
ros2 bag info "$jackal_debug_dir/bag"
```

Check that `/scan`, `/tf`, `/tf_static`, odometry, velocity commands, `/plan`, and costmaps contain messages. Inspect `recorder.log` for transport-loss warnings. With `--with-sensors`, also check point clouds and RGB/depth. MCAP splits at 2 GiB; this is not a total-size cap. Selected topics are discovered continuously; see [rosbag2 Jazzy](https://github.com/ros2/rosbag2/tree/jazzy#recording-data).

Add a `notes.txt` in the session directory with the start pose, goal positions/orientations, parameters changed, and approximate overshoot/backup times. Confirm that Host and Jackal clocks are synchronized before aligning the bag and robot logs. Share the entire session directory so parameters, actual motion, and failure logs can be analyzed together.

Local validation: localhost-only ROS Domain 211 covered Space start/pause/resume, Ctrl+C shutdown, before/after snapshots from 12 test nodes, and retention of static TF published before recording. Jackal was not connected; verify field topic completeness and wireless message loss using this section.

## Field record: 2026-10-13 10:35 session

The `jackal_20261013_103510` bag was saved: 195.3 MiB, approximately 19 minutes 39 seconds. Initial snapshots captured major parameters including MPPI, SLAM, and both costmaps. Runtime values confirmed the indoor behavior trees, 0.2 rad/s angular limit, 0.2 m / 30 s progress threshold, 0.02 m footprint padding per side, and 0.6 m inflation radius.

`Node not found` in final snapshots matches the shutdown order: Nav2 exited approximately 15 seconds before recording ended, and SLAM approximately 7 seconds before. Next time, stop recording and wait for `Saved` before shutting down Nav2/SLAM. Snapshots collected after restarting describe the new runtime state, not the previous experiment's final parameters.

Robot logs confirmed why the initial collision_monitor batch returned no parameters: uninitialized `FootprintApproach.max_points` caused the batch read to fail. An empty EKF batch was also saved, but there is no equally direct evidence identifying its specific parameter. The snapshot script now falls back to split reads; collision-protection settings need not be changed for this issue. This matches uninitialized-parameter exception handling in the [rclcpp parameter service source](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/parameter_service.cpp).

This recording reported 131 transport-layer message losses. It remains analyzable but is not lossless. MCAP warned that some action message definitions were not embedded. Installed Jazzy interfaces successfully decoded scan, static TF, velocity-command, and FollowPath feedback samples locally. Retain the corresponding ROS interface packages for decoding on another machine.


## 6. Inside wheels stopping or lateral sliding during turns

The `124612` bag confirmed nearly stationary inside wheels on both left and right turns. It contains neither motor commands/current nor chassis-controller parameter snapshots. See the [wheel-speed report](../analysis/jackal_20261013_124612/REPORT.md).

For the next run, use the same Host recording command in section 1; the updated topic list also records:

```text
/j100_0949/platform/motors/cmd_drive
/j100_0949/platform/motors/feedback
/j100_0949/platform/dynamic_joint_states
/diagnostics
/j100_0949/diagnostics
```

These topics appeared in this run's ROS graph, but their messages were absent from the old bag and cannot be recovered afterward. Host support files are updated; the recorder script does not need uploading to Jackal. After the next bag, still wait for `Saved` before stopping Nav2/SLAM.

`cmd_drive` contains the drive mode and left/right setpoints; `feedback` contains encoder speed/position, current, duty cycle, temperature, and fault flags for each drive. Aligning these with raw IMU can distinguish an intentionally tiny wheel-speed request, a nonzero request with stationary encoders, and reported drive faults. Software limits, static friction, and actual lateral slip still require separate assessment.

To quantify lateral slip, record small-angle turns with an external camera fixed in a ground reference frame, including the body and ground references. Neither the robot's forward-facing camera nor `odom.linear.y = 0` alone proves absence of lateral slip. Keep floor, load, and tire conditions consistent. Observe short left/right turns first; do not compare while changing several Nav2 parameters at once.
