# Jackal map and trajectory by goal request

The trajectory is divided into **11 goal-request intervals (G1-G11)** using the RViz log from the same session. All figures and PDF labels are in English.

## Figures and data

| File | Contents |
| --- | --- |
| [map_and_path.png](map_and_path.png) / [PDF](map_and_path.pdf) | Full map, colored by goal-request interval |
| [path_detail.png](path_detail.png) / [PDF](path_detail.pdf) | Enlarged path with G1-G11 request markers |
| [map_only.png](map_only.png) / [PDF](map_only.pdf) | Last recorded SLAM occupancy grid |
| [map_and_path_time.png](map_and_path_time.png) / [PDF](map_and_path_time.pdf) | Alternative view colored by elapsed Host receipt time |
| [path_detail_time.png](path_detail_time.png) / [PDF](path_detail_time.pdf) | Enlarged time-colored view |
| [goal_requests.csv](goal_requests.csv) | Request timestamps, estimated robot positions, matching errors, and source log lines |
| [trajectory_by_goal.csv](trajectory_by_goal.csv) | All trajectory samples with interval labels |
| [trajectory_map.csv](trajectory_map.csv) | Map-frame positions and heading without interval labels |

## Reading the annotations

- **G1** starts at the first recorded request; each later marker identifies a new RViz request. Each color continues until the next request, or the end of the recording for G11. Waiting after goal completion may therefore remain in the same color.
- Markers are the **estimated robot locations when requests were issued**, not the clicked destination coordinates. The green dot and red star mark the trajectory start and end.
- G10 and G11 use dashed/dotted lines to expose overlapping outbound and return paths.
- Gray map cells are unknown, white cells are free, and dark cells are occupied.

The current bag contains no navigation action or goal topics. The RViz message is emitted immediately before the asynchronous goal-send call; it does not establish the server acceptance time, result, or destination coordinates. G1-G11 are sequential labels assigned for this figure, not action UUIDs. [Nav2 RViz implementation](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_rviz_plugins/src/nav2_panel.cpp#L1186-L1225)

## Goal-request positions

All positions below are in the `map` frame. Local times use America/New_York (EDT). Elapsed time starts at the first valid trajectory TF receipt on the Host.

| Request | Local time (EDT) | Elapsed (s) | Robot X (m) | Robot Y (m) |
| --- | --- | ---: | ---: | ---: |
| G1 | 14:37:00.551 | 11.51 | 1.000 | -0.000 |
| G2 | 14:38:19.666 | 90.62 | 3.418 | 0.149 |
| G3 | 14:39:42.986 | 173.94 | 5.837 | 0.160 |
| G4 | 14:41:13.556 | 264.51 | 10.099 | 0.171 |
| G5 | 14:43:18.256 | 389.21 | 16.544 | -1.426 |
| G6 | 14:44:20.923 | 451.88 | 19.413 | -1.798 |
| G7 | 14:45:27.380 | 518.34 | 23.164 | -1.864 |
| G8 | 14:47:24.318 | 635.27 | 27.124 | -1.460 |
| G9 | 14:48:22.150 | 693.10 | 27.341 | 1.561 |
| G10 | 14:49:15.050 | 746.00 | 27.176 | 3.184 |
| G11 | 14:50:54.718 | 845.67 | 27.292 | -0.500 |

## Sources and time alignment

- Bag: `/home/nyu/Desktop/jackal_rosbag_repo/recording/jackal_20261014_143642`.
- RViz source: `/home/nyu/.ros/log/rviz2_2785433_1792002934425.log`. A session copy is retained as [rviz_session.log](rviz_session.log); [goal_request_source.json](goal_request_source.json) records its checksum.
- Map: last `/j100_0949/map` message, 1318 x 447 cells at 0.05 m/cell.
- Trajectory: 52,268 samples composed through `map -> odom_start -> odom -> base_link`, interpolating TF by header time and retaining the last received update for duplicate timestamps.

For request positions, each RViz Host log timestamp is matched to the nearest odometry TF Host receipt. The largest matching residual is 9.78 ms. This is a timestamp-matching residual, not a bound on physical position accuracy.

TF receipt times exceed their header timestamps by a median of 1.386 s; clock offset and delivery delay are not separately measured. Request markers are consequently approximate. The trajectory is online localization history over the final recorded grid, including SLAM corrections, rather than a final reoptimized pose graph or external ground truth.

## Regenerate the figures

From this folder, using the cached map/TF arrays and copied RViz log:

```bash
python3 plot_map_path.py
```
