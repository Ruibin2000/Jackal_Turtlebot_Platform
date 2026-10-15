#!/usr/bin/env bash
# Host-only diagnostic recording. Does not start Nav2/SLAM or command motion.
set -e
usage() {
  cat <<'USAGE'
Usage: bash scripts/record_jackal_debug.sh [--with-sensors] [NEW_SESSION_DIRECTORY]
       bash scripts/record_jackal_debug.sh --snapshot-only EXISTING_SESSION_DIRECTORY
Default: ~/Desktop/jackal_rosbag_repo/debug/jackal_YYYYMMDD_HHMMSS
Start SLAM/Nav2 first. Saves live parameters, then waits for Space to record.
Space pauses/resumes; Ctrl+C saves the bag and a second parameter snapshot.
Keep SLAM/Nav2 running until the final snapshot completes.
--with-sensors adds full LiDAR points, RGB/depth images and calibration.
--snapshot-only saves a new timestamped parameter snapshot without recording.
USAGE
}
jackal_with_sensors=false
jackal_snapshot_only=false
jackal_session=''
for arg in "$@"; do
  case "$arg" in
    -h|--help) usage; exit 0 ;;
    --with-sensors) jackal_with_sensors=true ;;
    --snapshot-only) jackal_snapshot_only=true ;;
    --*) printf 'Unknown option: %s\n' "$arg" >&2; exit 2 ;;
    *)
      if [[ -n "$jackal_session" ]]; then usage >&2; exit 2; fi
      jackal_session="$arg"
      ;;
  esac
done
source /opt/ros/jazzy/setup.bash
set -uo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
jackal_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
jackal_qos_file="$jackal_repo_root/config/jackal/rosbag_qos.yaml"
jackal_topics_file="$jackal_repo_root/config/jackal/debug_topics.txt"
jackal_snapshot_script="$jackal_repo_root/scripts/snapshot_jackal_debug.py"
if [[ ! -r "$jackal_qos_file" || ! -r "$jackal_topics_file" || ! -r "$jackal_snapshot_script" ]]; then
  printf 'Missing recorder support files under %s\n' "$jackal_repo_root" >&2
  exit 1
fi
if $jackal_snapshot_only; then
  if [[ -z "$jackal_session" || ! -d "$jackal_session" ]]; then
    printf 'Snapshot-only requires an existing session directory.\n' >&2; exit 2
  fi
  exec python3 "$jackal_snapshot_script" "$jackal_session/params_manual_$(date +%Y%m%d_%H%M%S_%N)"
fi
jackal_session="${jackal_session:-$HOME/Desktop/jackal_rosbag_repo/debug/jackal_$(date +%Y%m%d_%H%M%S)}"
if [[ -e "$jackal_session" ]]; then
  printf 'Session already exists; choose a new directory: %s\n' "$jackal_session" >&2; exit 1
fi
if [[ ! -t 0 ]]; then
  printf 'Run in an interactive terminal; Space starts recording.\n' >&2; exit 1
fi
mkdir -p -- "$(dirname -- "$jackal_session")"
mkdir -- "$jackal_session"
cp -- "$jackal_qos_file" "$jackal_session/recording_qos.yaml"
jackal_topics=()
while IFS= read -r line; do
  [[ -z "$line" || "$line" == \#* ]] && continue
  jackal_topics+=("$line")
done < "$jackal_topics_file"
for action in navigate_to_pose navigate_through_poses follow_path compute_path_to_pose compute_path_through_poses spin backup wait; do
  jackal_topics+=("/j100_0949/$action/_action/feedback" "/j100_0949/$action/_action/status")
done
if $jackal_with_sensors; then
  jackal_topics+=(
    /j100_0949/sensors/lidar3d_0/points
    /j100_0949/sensors/camera_0/color/image
    /j100_0949/sensors/camera_0/color/camera_info
    /j100_0949/sensors/camera_0/color/metadata
    /j100_0949/sensors/camera_0/depth/image
    /j100_0949/sensors/camera_0/depth/camera_info
    /j100_0949/sensors/camera_0/depth/metadata
    /j100_0949/sensors/camera_0/extrinsics/depth_to_color
  )
fi
printf '%s\n' "${jackal_topics[@]}" > "$jackal_session/requested_topics.txt"
printf 'Created UTC: %s\nROS_DOMAIN_ID=%s\nWith full sensors: %s\n' \
  "$(date -u --iso-8601=seconds)" "$ROS_DOMAIN_ID" "$jackal_with_sensors" > "$jackal_session/session.txt"
printf 'Session: %s\nKeep SLAM/Nav2 running until the final snapshot completes.\n' "$jackal_session"
python3 "$jackal_snapshot_script" "$jackal_session/params_before" || \
  printf 'WARNING: parameter snapshot incomplete; inspect params_before/*.failed.txt\n' >&2
printf 'PAUSED — press Space to start. Wait for recorder subscriptions before sending a goal.\n'
trap 'printf "\nCancelled before recording; parameter snapshot retained.\n"; exit 130' INT
jackal_start_key=''
while IFS= read -r -s -n 1 jackal_start_key; do
  [[ "$jackal_start_key" == ' ' ]] && break
done
if [[ "$jackal_start_key" != ' ' ]]; then
  printf 'No Space received; bag not started.\n' >&2; exit 1
fi
# Keep the shell alive after terminal Ctrl+C so it can save params_after.
# The foreground recorder receives SIGINT; tee -i drains its shutdown messages.
trap ':' INT
printf '\nRecording. Space pauses/resumes; Ctrl+C saves and collects final parameters.\n'
set +e
ros2 bag record \
  --output "$jackal_session/bag" \
  --storage mcap --storage-preset-profile zstd_fast \
  --max-bag-size 2147483648 \
  --include-hidden-topics \
  --qos-profile-overrides-path "$jackal_session/recording_qos.yaml" \
  --topics "${jackal_topics[@]}" 2>&1 | tee -i "$jackal_session/recorder.log"
jackal_record_status=("${PIPESTATUS[@]}")
set -e
trap - INT
printf 'Recorder exit=%s; log writer exit=%s\n' "${jackal_record_status[0]}" "${jackal_record_status[1]}" >> "$jackal_session/session.txt"
python3 "$jackal_snapshot_script" "$jackal_session/params_after" || \
  printf 'WARNING: final parameter snapshot incomplete; inspect params_after/*.failed.txt\n' >&2
if [[ -f "$jackal_session/bag/metadata.yaml" ]]; then
  ros2 bag info "$jackal_session/bag" > "$jackal_session/bag_info.txt" 2>&1 || true
else
  printf 'WARNING: no bag metadata; inspect recorder.log.\n' >&2
fi
printf 'Saved: %s\n' "$jackal_session"
if (( jackal_record_status[1] != 0 )); then exit "${jackal_record_status[1]}"; fi
if (( jackal_record_status[0] != 0 && jackal_record_status[0] != 130 )); then
  exit "${jackal_record_status[0]}"
fi
