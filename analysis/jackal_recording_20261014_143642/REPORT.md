# Production Recording Continuity: jackal_20261014_143642

Finding: **point clouds are sparse throughout, with multiple continuous gaps lasting tens of seconds to over two minutes. Cameras mainly show scattered missing frames and shorter gaps. There is no evidence that either MCAP split paused all topics together.**

Source: `/home/nyu/Desktop/jackal_rosbag_repo/recording/jackal_20261014_143642`. Time: 2026-10-14, America/New_York (EDT), 14:36:48.797–14:54:14.388, spanning 1045.591 s. Timestamps were read offline from all three MCAP files; extracted counts for all selected topics match metadata.yaml.

## 1. Message intervals

"Bag interval" is the recorded-time difference between adjacent recorded messages; "Header interval" is the difference between those messages' own stamps. These describe gaps in the bag but cannot alone locate loss at the publisher, network, middleware, or recorder. Cross-machine clock offsets are uncalibrated, so bag time minus Header time is not direct network latency.

| Data | Messages | Whole-run mean Hz | Median bag interval s | Maximum bag interval s | Maximum Header interval s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 3D point cloud | 79 | 0.076 | 7.297 | 139.700 | 139.696 |
| RGB image | 6786 | 6.490 | 0.104 | 2.394 | 2.402 |
| Depth image | 7861 | 7.518 | 0.104 | 1.238 | 1.133 |
| 2D scan | 6028 | 5.765 | 0.113 | 1.211 | 1.100 |
| Chassis odometry | 52263 | 49.984 | 0.020 | 0.951 | 0.140 |
| Raw body IMU | 52194 | 49.918 | 0.015 | 0.951 | 0.135 |

LiDAR IMU `/j100_0949/sensors/lidar3d_0/imu/data_raw` has 0 messages throughout. This bag alone cannot distinguish absent publication, incorrect topic configuration, QoS/transport problems, or other causes.

## 2. Point clouds: sparse frames and long gaps

The first of 79 frames arrives at 14:36:53.859; the last at 14:54:08.777. The three files contain 31, 32, and 16 frames, so clouds were not confined to the beginning or one split.

Of 78 adjacent-frame intervals, 34 exceed 10 s and 5 exceed 30 s. The five longest gaps below are bounded by consecutive frame receipt times:

| Previous frame time EDT | Next frame time EDT | Gap |
| --- | --- | ---: |
| 14:51:43.858 | 14:54:03.558 | 139.700 s |
| 14:39:09.732 | 14:40:28.757 | 79.024 s |
| 14:44:37.876 | 14:45:33.273 | 55.397 s |
| 14:50:06.363 | 14:50:49.164 | 42.801 s |
| 14:41:42.269 | 14:42:16.758 | 34.489 s |

During the longest 139.700 s cloud gap, the bag still records 787 scans, 883 RGB images, 913 depth images, 6985 odometry messages, and 6976 body IMU messages. The entire recorder was not inactive for those roughly 140 seconds.

LiDAR metadata specifies mode `1024x10`. This describes the configured scan mode; the difference between theoretical and recorded counts cannot be attributed entirely to network loss from that setting alone.

## 3. Cameras: mainly scattered missing frames throughout

Maximum bag intervals are 2.394 s for RGB and 1.238 s for depth; neither has an adjacent-message gap over 5 s.

Matching images to camera_info by identical Header stamps:

- RGB: 6,786 of 29,454 camera_info stamps have matching images; 22,668 have none in the bag. The longest consecutive run without a matching image is 63 camera_info messages.
- Depth: 7,861 of 15,383 camera_info stamps have matching images; 7,522 have none. The longest consecutive unmatched run is 15 camera_info messages.
- Every recorded image has camera_info with the same stamp. These are missing pairs within the bag, not exact end-to-end loss rates.

## 4. Relationship to split/cache writes

The two splits occur 353.726 s and 727.793 s after the start, matching the user's cache-write logs.

- Odometry intervals across the split boundaries are 0.020061 s and 0.020071 s, consistent with normal 50 Hz operation.
- Each 5-second window immediately before and after either split contains 250 odometry messages.
- The three longest cloud gaps lie within separate files rather than crossing split boundaries.

These splits therefore do not explain sparse clouds and missing camera frames throughout. This does not claim that splitting can never cause any brief blocking.

## 5. Evidence limits and next diagnostics

The bag lacks per-topic, time-resolved loss events, so the terminal's aggregate 45,101 losses cannot be reconstructed individually on a timeline. Recorded gap distribution can be established. Large clouds/images are heavily missing while smaller messages continue, prioritizing large-message publication, transport, DDS reception, and recording load. The bag alone cannot isolate Wi-Fi, drivers, or disk as the cause.

Next, retain sensor settings while comparing cloud reception on Jackal versus Host, and short "cloud + IMU only" versus camera-inclusive bags. Independently check actual LiDAR IMU publication/topic/QoS for its zero count. This analysis changed neither recorder nor robot configuration.

## Files

- [Timing plot](continuity.png): red ticks are cloud frames; black dashed lines are split boundaries; other curves show 10-second counts or adjacent-message intervals.
- [Per-frame cloud timestamps](point_timestamps.csv)
- [Bag intervals over 1 second](gaps_over_1s.csv)
- [Messages per 10 seconds](counts_10s.csv)
- [Full statistics](continuity_summary.json)
- [Reproducible analysis script](analyze_continuity.py): run after loading ROS Jazzy; reads the bag without ROS replay.


## 6. Network throughput and GLIM suitability

Estimate normal payload using each stream's first serialized message size, LiDAR metadata mode `1024x10`, and camera_info Header steps of approximately 33.36 / 66.67 ms, assuming RGB 30 Hz and depth 15 Hz. This is not a measurement of live publisher throughput or router capacity.

| Data | Organized dimensions | Serialized size per frame | Estimated rate | Raw payload |
| --- | --- | ---: | ---: | ---: |
| 3D point cloud | 1024 × 32 | 1.573 MB | 10 Hz | 125.9 Mbps |
| RGB image | 640 × 480 | 0.922 MB | 30 Hz | 221.2 Mbps |
| Depth image | 640 × 480 | 0.614 MB | 15 Hz | 73.7 Mbps |

These streams total approximately **420.8 Mbps (52.6 MB/s, decimal units)** before other topics, protocols, or retries. Recorded counts imply only approximately 85.8 Mbps of serialized payload for these three streams. That is not total link throughput: incomplete receptions, other subscriptions, and protocol overhead are excluded. MCAP zstd compression happens during Host disk writes and does not reduce raw PointCloud2/Image network traffic in this configuration.

Severe large-message loss with continuing small messages such as odometry is consistent with insufficient capacity or large-message transport/buffer problems. First inspect effective wireless throughput, whether the robot-to-client-router Ethernet port negotiated 100 Mbps, and DDS/system fragmentation reassembly and buffers. The bag **cannot alone prove router Wi-Fi throughput is the root cause**; source load and other factors remain possible. Fast DDS documents socket-buffer saturation with high-rate large data, no retransmission of missing BEST_EFFORT samples, and wireless large-message limitations. [Fast DDS large data](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/large_data/large_data.html)

Cloud gaps are not merely stationary waiting. Odometry approximately paired to consecutive cloud receipt times indicates **5.55 m** accumulated travel, **4.72 m** net displacement, and approximately **88°** endpoint heading difference during the longest 139.7 s gap. Another 55.4 s gap spans approximately **3.20 m** displacement. Odometry describes motion scale with drift and approximate receipt-time matching, not external ground truth.

GLIM assessment:

- Do not use this directly as a reliable production input for complete-route reconstruction. Long cloud gaps with substantial motion increase inertial drift and inter-frame registration difficulty; producing some output does not prove accuracy.
- LiDAR IMU count is 0, but raw body IMU is approximately 49.9 Hz. Body IMU use can be investigated after confirming LiDAR–IMU extrinsics, axes, and timing. These are not yet verified; changing only the topic name does not establish suitability. [GLIM sensor parameters](https://github.com/koide3/glim/blob/master/docs/parameters.md)
- GLIM also supports LiDAR-only operation, so missing LiDAR IMU does not make every configuration impossible. That mode cannot recover missing clouds; inspect overlap and motion in sparse segments before trying. [GLIM quick start](https://github.com/koide3/glim/blob/master/docs/quickstart.md)
- Default LiDAR–IMU mapping does not automatically fill missing clouds using recorded RGB/depth images; GLIM camera configuration serves relevant extensions. [GLIM sensor configuration](https://github.com/koide3/glim/blob/master/config/config_sensors.json)
- Retain this bag for single-cloud inspection, local-segment experiments, and odometry/2D-map analysis. GLIM has not been run, so failure of every local segment cannot be assumed. Recollect data for a formal complete-route result.

If continuing Host recording next, prioritize a short wired-link comparison or compare cloud + IMU reception with and without cameras. Accept collection only when reception stays near source rate without long gaps, then record the full route.

Additional files: [payload sizes and motion statistics](payload_and_motion.json), [extraction script](analyze_payload_motion.py).
