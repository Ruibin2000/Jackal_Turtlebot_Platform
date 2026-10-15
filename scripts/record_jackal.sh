#!/usr/bin/env bash
# Host or onboard recorder: original 14-topic collection plus 7 RGB-D topics.
set -e
# The original Host invocation remains available; --onboard selects Jackal.
jackal_onboard=false
while (( $# > 0 )); do
  case "$1" in
    --onboard) jackal_onboard=true; shift ;;
    --glim-camera) shift ;;
    --help|-h)
      cat <<'HELP'
Usage: bash record_jackal.sh [--onboard] [--glim-camera] [NEW_BAG_DIRECTORY]
Records the same 21 GLIM + camera topics on either machine.
Host default: ~/Desktop/jackal_rosbag_repo/recording/jackal_YYYYMMDD_HHMMSS
Onboard default: ~/jackal_bags/recording/jackal_rpp_YYYYMMDD_HHMMSS
--onboard uses /etc/clearpath/setup.bash and a free-space/time guard.
Onboard defaults: reserve 100 GiB, max session 3600 s, check every 5 s.
Override with JACKAL_MIN_FREE_GIB, JACKAL_MAX_RECORD_SECONDS (0 = unlimited),
and JACKAL_SPACE_CHECK_SECONDS. The time limit includes pauses after first Space.
Initially waits for Space; Space then pauses/resumes; Ctrl+C stops and saves.
HELP
      exit 0 ;;
    --*) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    *) break ;;
  esac
done
if (( $# > 1 )); then
  printf 'Expected at most one output bag directory.\n' >&2
  exit 2
fi
if [[ "$jackal_onboard" == true ]]; then
  source /etc/clearpath/setup.bash
else
  source /opt/ros/jazzy/setup.bash
fi
set -uo pipefail
if [[ "$jackal_onboard" != true ]]; then
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ROS_LOCALHOST_ONLY=0
fi
jackal_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$jackal_onboard" == true ]]; then
  jackal_default_dir="$HOME/jackal_bags/recording/jackal_rpp_$(date +%Y%m%d_%H%M%S)"
else
  jackal_default_dir="$HOME/Desktop/jackal_rosbag_repo/recording/jackal_$(date +%Y%m%d_%H%M%S)"
fi
jackal_bag_dir="${1:-$jackal_default_dir}"
jackal_guard_cmd=()
if [[ "$jackal_onboard" == true ]]; then
  jackal_guard_cmd=(python3 "$jackal_repo_root/scripts/guard_jackal_recording.py"
    --output "$jackal_bag_dir"
    --min-free-gib "${JACKAL_MIN_FREE_GIB:-100}"
    --max-seconds "${JACKAL_MAX_RECORD_SECONDS:-3600}"
    --check-seconds "${JACKAL_SPACE_CHECK_SECONDS:-5}")
  "${jackal_guard_cmd[@]}" --check-only
fi
jackal_qos_file="$jackal_repo_root/config/jackal/rosbag_qos.yaml"
if [[ -e "$jackal_bag_dir" ]]; then
  printf 'Output already exists; choose a new directory: %s\n' "$jackal_bag_dir" >&2
  exit 1
fi
if [[ ! -r "$jackal_qos_file" ]]; then
  printf 'Missing recording QoS file: %s\n' "$jackal_qos_file" >&2
  exit 1
fi
if [[ ! -t 0 ]]; then
  printf 'Run this script in an interactive terminal; press Space to start recording.\n' >&2
  exit 1
fi
# Original collection list plus camera. Both raw IMUs are retained so GLIM
# can use its calibrated source; recording them does not fuse them.
jackal_record_topics=(
  /j100_0949/sensors/lidar3d_0/points
  /j100_0949/sensors/lidar3d_0/scan
  /j100_0949/sensors/lidar3d_0/imu/data_raw
  /j100_0949/sensors/lidar3d_0/metadata
  /j100_0949/sensors/imu_0/data
  /j100_0949/sensors/imu_0/data_raw
  /j100_0949/platform/odom
  /j100_0949/platform/odom/filtered
  /j100_0949/platform/joint_states
  /j100_0949/tf
  /j100_0949/tf_static
  /j100_0949/map
  /j100_0949/map_metadata
  /j100_0949/pose
  /j100_0949/sensors/camera_0/color/image
  /j100_0949/sensors/camera_0/color/camera_info
  /j100_0949/sensors/camera_0/color/metadata
  /j100_0949/sensors/camera_0/depth/image
  /j100_0949/sensors/camera_0/depth/camera_info
  /j100_0949/sensors/camera_0/depth/metadata
  /j100_0949/sensors/camera_0/extrinsics/depth_to_color
)
printf 'Profile: GLIM + camera (%s topics)\n' "${#jackal_record_topics[@]}"
# Do not start rosbag with --start-paused: its subscriptions consume and drop
# one-shot transient-local TF/calibration while paused. Subscribe after Space.
printf 'Output: %s\nPAUSED — press Space to start. Ctrl+C cancels.\n' "$jackal_bag_dir"
trap 'printf "\nCancelled before recording.\n"; exit 130' INT
jackal_start_key=''
while IFS= read -r -s -n 1 jackal_start_key; do
  if [[ "$jackal_start_key" == ' ' ]]; then
    break
  fi
done
if [[ "$jackal_start_key" != ' ' ]]; then
  printf '\nNo Space received; recording was not started.\n' >&2
  exit 1
fi
trap - INT
mkdir -p -- "$(dirname -- "$jackal_bag_dir")"
printf '\nStarting recording. Space pauses/resumes; Ctrl+C stops and saves.\n'
jackal_record_cmd=(ros2 bag record \
  --output "$jackal_bag_dir" \
  --storage mcap \
  --storage-preset-profile zstd_fast \
  --max-bag-size 2147483648 \
  --qos-profile-overrides-path "$jackal_qos_file" \
  --topics "${jackal_record_topics[@]}")
if [[ "$jackal_onboard" == true ]]; then
  exec "${jackal_guard_cmd[@]}" -- "${jackal_record_cmd[@]}"
fi
exec "${jackal_record_cmd[@]}"
