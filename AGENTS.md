# Project memory

Updated on 2026-10-15. Read this file before continuing work on this project, then consult the relevant manuals, scripts, and latest field records. Later user instructions and file updates take precedence over this snapshot.

## Language preferences

- Write all documentation in English, including manuals, READMEs, analysis reports, figure titles, axis labels, legends, PDF text, and accompanying notes.
- Use Chinese by default for conversations, progress updates, explanations in chat, and final replies.
- Preserve commands, file paths, ROS topic names, parameter names, and quoted logs exactly; do not translate technical identifiers.
- Follow any explicit user language request for the specific output.

## Current operating manual and user preferences

- The primary operating reference is the user-simplified [Jackal operating manual](Docs/Jackal_RPP_Trial_2026-10-14.md). Do not default to an older MPPI manual merely because this filename contains Trial.
- The user retained the specified start pose, SLAM, indoor RPP Nav2, Host RViz, and production recording workflow, and removed the former section 5 debug comparison, section 6 rollback, and section 7 standalone tolerance adjustment. Respect this concise structure; do not restore lengthy experiment histories or full debug procedures without a request.
- The everyday section order is 3.1 → 4 → 4.1 → 4.2 → 8. Section 4.2 adds the requested LiDAR IMU / points checks before recording; obsolete references to sections 5/7 were removed from the introduction.
- Keep operating instructions concise and commands directly copyable; clearly identify which terminal runs on Jackal and which runs on the Host.
- Current map and trajectory figures use separate colors for 11 RViz goal requests. Markers show the estimated robot position at each request, not target coordinates or confirmed Nav2 acceptance times. Outputs and English notes are in `analysis/jackal_recording_20261014_143642/map_path/`.

## Environment and startup

- Workspace: `/home/nyu/Desktop/Jackal_Turtlebot_Platform`; ROS 2 Jazzy.
- Jackal SSH: `robot@192.168.185.201`; namespace `/j100_0949`; Jackal environment `/etc/clearpath/setup.bash`; Host environment `/opt/ros/jazzy/setup.bash`.
- Everyday sequence: J1 sets the start pose and starts SLAM → J2 checks lifecycle state and TF, then starts Nav2 → H1 runs RViz → check LiDAR data on Jackal/Host → J3 records onboard inside tmux → send goals → after recording stops, H2 transfers and verifies the bag.
- Start-pose script: `python3 ~/start_jackal_slam_at_pose.py --x 1 --y 0 --yaw-deg 0` defines the current pose as `(1,0,0°)` in a new map. Previous Nav2/SLAM sessions must be stopped and the robot stationary. This script starts the new SLAM session itself.
- Wait for `SLAM ACTIVE`, verify that `/j100_0949/slam_toolbox` is active and `map -> base_link` is close to the requested pose, then start Nav2.
- The navigation entry point remains `~/jackal_nav2/nav2_low_speed.launch.py`, which reads `~/jackal_nav2/nav2_low_speed.yaml`. Uploading a candidate file alone does not activate it.
- RViz runs on the Host with Fixed Frame `map`. SLAM/Nav2 currently use `/j100_0949/sensors/lidar3d_0/scan`, confirmed as ring 15. Direction checks provided no basis for applying an additional 180° LiDAR rotation.

## Current navigation configuration

- Candidate: [nav2_low_speed_indoor_rpp.yaml](config/jackal/nav2_low_speed_indoor_rpp.yaml). Rotation Shim wraps RPP.
- RPP desired linear speed: `0.06 m/s`; lookahead distance: `0.6 m`. Shim angular speed: `0.10 rad/s`; angular acceleration: `0.30 rad/s²`.
- Velocity smoother: `max_velocity=[0.2,0.0,0.1]`, `scale_velocities=true`.
- Progress checking: displacement of `0.20 m` OR rotation of `0.35 rad`, with a `30 s` allowance.
- The Host candidate now uses a goal position tolerance of `0.10 m` and heading tolerance of `0.25 rad`. Do not treat an on-disk configuration as the latest onboard parameter readback.
- The 2026-10-14 `jackal_rpp_20261014_114801` field run succeeded on all 5 goals with the position tolerance still at `0.25 m`. Deployment and field verification of the subsequent `0.10 m` setting still require evidence.
- Confirmed footprint: `0.26 m` forward/backward and `0.22 m` left/right, with `0.02 m` padding and `0.6 m` inflation radius. Disabling RPP reversing does not disable the behavior tree's BackUp action.

## Production recording conventions

- The user now chooses **onboard Jackal recording followed by transfer to the Host**. Local scripts and the manual have been updated, but have not been uploaded to Jackal or field-tested. Do not describe a code update as a deployment. The legacy Host invocation remains supported.
- Before recording, follow manual section 4.2: inspect topic info on Jackal, receive one IMU/point-cloud message with best-effort echo, and measure hz on Jackal. A Host comparison is optional network troubleshooting. Publisher presence does not prove continuous messages; hz measures the received rate. Only the instructions have been updated; live checks have not yet been performed. Finish these checks before recording, then verify the full load with a short bag that includes cameras.
- [record_jackal.sh](scripts/record_jackal.sh) defaults to the updated **21-topic list**: the user's original 14 topics plus 7 camera topics, with no additional option required. `--glim-camera` remains a compatibility alias for the same list.
- Original 14 topics: full LiDAR points, scan, raw LiDAR IMU, LiDAR metadata, body IMU data/data_raw, platform/odom, odom/filtered, joint_states, tf, tf_static, map, map_metadata, and pose. The 7 camera topics are image, camera_info, and metadata for both color/depth, plus depth_to_color extrinsics.
- Production recording does not add the full debug set of motor feedback, velocity-chain topics, costmaps, navigation actions, or parameter snapshots. Use `record_jackal_debug.sh` separately for navigation troubleshooting.
- Onboard directory: Jackal `~/jackal_bags/recording/jackal_rpp_YYYYMMDD_HHMMSS`; Host transfer destination: `~/Desktop/jackal_rosbag_repo/recording/`. Without `--onboard`, the script retains the legacy default Host recording path.
- The new `scripts/guard_jackal_recording.py` is used by `record_jackal.sh --onboard` with the Clearpath environment. It checks the target filesystem at startup and after the first Space press, warns and refuses to start if free space is ≤100 GiB, then checks every 5 seconds while recording and sends SIGINT for a clean stop at the threshold. The default session limit is 3600 seconds, including pauses after recording starts. `JACKAL_MAX_RECORD_SECONDS=0` disables the time limit while retaining space protection. 2 GiB is the split size, not a total-size cap. The script never deletes data automatically or force-kills the recorder.
- Local validation: 7 guard tests passed, covering refusal on low space, space dropping during recording, timed SIGINT shutdown, locking, and terminal Space/Ctrl+C handling. An actual rosbag test in an isolated localhost ROS Domain recorded 44 test messages; MCAP/metadata remained readable after automatic shutdown. This does not establish onboard throughput for all 21 topics.
- User-reported onboard storage: NVMe TS512GMTE652T2, approximately 476.9 GiB; root filesystem approximately 468 GiB, with 431 GiB free at the time. Recheck free space for every session; this is not a continuing guarantee.
- Follow manual section 8 to upload scripts, the guard, and QoS settings to `~/jackal_recorder/`, then start inside Jackal's `tmux new-session -A -s jackal_record`:

```bash
source /etc/clearpath/setup.bash
jackal_bag="$HOME/jackal_bags/recording/jackal_rpp_$(date +%Y%m%d_%H%M%S)"
bash "$HOME/jackal_recorder/scripts/record_jackal.sh" --onboard "$jackal_bag"
```

- After recording and disk writes finish, copy the entire bag to the Host with rsync as in section 8.2, then verify with a checksum dry-run and `ros2 bag info`. Do not automatically delete the source. tmux protects only the recording session; it does not take over SLAM/Nav2 in J1/J2.
- Startup displays PAUSED. The first Space press starts the recorder and subscriptions, preserving static TF/calibration; subsequent Space presses pause/resume, and Ctrl+C flushes and saves. Format: MCAP, zstd_fast, approximately 2 GiB splits.
- Use `ros2 bag info "$jackal_bag"` without appending `/bag`. Shutdown order: cancel the goal and stop the robot → stop recording and wait for disk writes → exit Nav2 → exit SLAM.
- SLAM/Nav2 console logs remain under the robot's `~/jackal_nav2/debug_logs/slam_*` and `nav_rpp_*`, respectively.

## Recent data quality and unresolved issues

- [Production bag analysis](analysis/jackal_recording_20261014_143642/REPORT.md): `jackal_20261014_143642` spans 1045.59 seconds but contains only 79 full point clouds, with a maximum consecutive gap of 139.7 seconds. It is not a complete, continuous GLIM dataset.
- LiDAR `/sensors/lidar3d_0/imu/data_raw` has **0 messages** in this bag; body `/sensors/imu_0/data_raw` has 52,194 messages at approximately 50 Hz. Do not equate "not recorded in the bag" with "the live driver definitely did not publish" or "IMU readings were zero." Live publication has not yet been checked.
- Odometry intervals across both split boundaries are approximately 20 ms. Cache-write logs correspond to normal splitting and do not establish the cause of all missing frames.
- Raw point clouds/RGB/depth at the configured rates are estimated to require approximately 421 Mbps. Wi-Fi capacity, obstruction, a 100 Mbps Ethernet port, and DDS large-message reception remain factors to verify. The router has not been confirmed as the root cause, and RSSI was not collected.
- Network baseline: Jackal connects by Ethernet to an onboard client router, which connects wirelessly to the main router; the Host connects by Ethernet to the main network. For obstruction analysis, focus on the wireless link between the onboard and main routers.
- The onboard recording workflow is implemented in the scripts and manual, pending upload and field testing. First use section 8.1 to check message counts, disk growth, and local load in a short full 21-topic bag. Live LiDAR IMU publication remains unresolved.
