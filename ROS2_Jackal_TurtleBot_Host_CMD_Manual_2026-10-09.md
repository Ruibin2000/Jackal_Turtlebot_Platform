# Jackal J100 + TurtleBot 4 + Host：ROS 2 多机器人 CMD 指令手册

> **版本**：2026-10-09（基于当日实际终端输出与网络拓扑记录）  
> **ROS**：Ubuntu 24.04 / ROS 2 Jazzy / Fast DDS (`rmw_fastrtps_cpp`)  
> **目标**：分清 **Jackal 端 / TurtleBot 端 / Host 端** 的操作位置、当前结构、启动和检查命令，避免两个机器人发生 Topic 或运动控制混淆。
>
> **标记**：✅ 已验证配置或当时实测；🔍 只读诊断；⚠️ 待验证、条件性命令或规划项。**所有命令均应在对应设备的终端执行。**

## 目录

1. [当前网络和系统结构](#1-当前网络和系统结构)
2. [ROS-2-命名空间话题与运动控制](#2-ros-2-命名空间话题与运动控制)
3. [Jackal 端：`robot@cpr-j100-0949`](#3-jackal-端robotcpr-j100-0949)
4. [TurtleBot 4 端：`ubuntu@192.168.185.101`](#4-turtlebot-4-端ubuntu192168185101)
5. [Host 端：Alienware](#5-host-端alienware)
6. [每日启动、验证和停止流程](#6-每日启动验证和停止流程)
7. [常见故障排查](#7-常见故障排查)
8. [配置文件与变更原则](#8-配置文件与变更原则)

---

## 1. 当前网络和系统结构

### 1.1 三端分工

| 端 | 当前角色 | ROS 2 程序/设备 | 关键地址 |
|---|---|---|---|
| **Host（Alienware）** | SSH、网络测试、订阅两台机器人数据、RViz、数据录制 | ROS 2 Jazzy、Fast DDS；**不承担目前的 Jackal SLAM/Nav2 计算** | `enp132s0 = 192.168.185.2/24` ✅ |
| **TurtleBot 4（Raspberry Pi）** | Create 3 底盘与雷达/摄像头数据发布，独立底盘控制 | Create 3、RPLIDAR、OAK-D；话题目前主要位于根命名空间 `/` | `eth0 = 192.168.185.101/24` ✅ |
| **Jackal J100（车载计算机）** | 底盘/传感器、**SLAM Toolbox + Nav2/MPPI 本机运行** | Clearpath Platform、Ouster OS1、RealSense D435、SLAM、Nav2 | `br0 = 192.168.185.201/24` ✅ |

### 1.2 逻辑拓扑（以 IP / ROS 事实为准）

```text
                         ROS 2 Domain ID = 0
                   Fast DDS / 192.168.185.0/24

          ┌────────────────── LAN ──────────────────┐
          │                  │                      │
          ▼                  ▼                      ▼
  HOST / Alienware       TurtleBot 4              Jackal J100
  enp132s0: .185.2      Pi eth0: .185.101        PC br0: .185.201
  ├── SSH               ├── ROS /odom, /scan      ├── /j100_0949/*
  ├── RViz              ├── RPLIDAR + OAK-D       ├── Ouster + D435
  └── ros2 bag          ├── usb0: .186.3         ├── SLAM Toolbox
                       │     │                   ├── Nav2 + MPPI
                       │     └── Create 3         └── br0: .131.1
                       │         内部 USB 网络          │
                       └── wlan0: 校园 Wi-Fi          └── 传感器子网

  共享 ROS 网络：192.168.185.0/24
  TurtleBot 内部：192.168.186.0/24（Create 3；不改成 .185）
  Jackal 内部：192.168.131.0/24（传感器；不改成 .185）
```

**物理布线提示**：历史规划中出现过 TP-Link 子路由器以及 NETGEAR GS108Tv3 交换机；**当前交换机端口、管理 IP、VLAN 和网关实体未完全实测，不把规划图当成实际布线**。TurtleBot Netplan 引用了网关 `192.168.185.1`，不等于已验证某一台特定设备正在该地址运行。此前规划的 `.100/.200`（路由器）及 `.102/.103/.202/.203`（RFSoC、Pi-Radio）**不是当前已验证的在线设备**。

### 1.3 ROS 2 共享通信配置

| 设置 | Host | TurtleBot 4 | Jackal J100 |
|---|---|---|---|
| ROS 2 | Jazzy ✅ | Jazzy ✅ | Jazzy ✅ |
| `ROS_DOMAIN_ID` | `0` ✅ | `0` ✅ | `0` ✅ |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` ✅ | 同左 ✅ | 同左 ✅ |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET`（目标配置） | `SUBNET` ✅ | `SUBNET` ✅ |
| XML | 无已确认自定义 Host XML | `/etc/turtlebot4/fastdds_rpi.xml` ✅ | `/home/robot/fastdds_185.xml` ✅ |
| ROS Topic 主要命名空间 | 按具体程序 | `/` | `/j100_0949` |

Jackal 的 Fast DDS XML 用来选择 `192.168.185.201` 通信接口，避免只用 `192.168.131.1` 做 Discovery 导致 Host 漏掉 Topic；对应 systemd 服务已有环境变量覆盖。不要在不了解后果时清空其 Fast DDS 配置。

---

## 2. ROS 2 命名空间、话题与运动控制

### 2.1 不要混用 TurtleBot 与 Jackal 的话题

| 数据 | TurtleBot 4 | Jackal J100 |
|---|---|---|
| 里程计 | `/odom` | `/j100_0949/platform/odom` |
| 2D 激光 | `/scan` | `/j100_0949/sensors/lidar3d_0/scan` |
| 3D 激光 | 目前未确认对应 Topic | `/j100_0949/sensors/lidar3d_0/points` |
| 运动速度命令 | `/cmd_vel`（TurtleBot 链路） | `/j100_0949/cmd_vel`（Jackal 外部指令入口） |
| TF | `/tf`、`/tf_static` | `/j100_0949/tf`、`/j100_0949/tf_static` |
| 地图 | 视 TurtleBot SLAM 是否启动 | `/j100_0949/map`（SLAM 运行时） |
| RGB 摄像头 | `/oakd/rgb/preview/image_raw`（先确认在线） | `/j100_0949/sensors/camera_0/color/image` |
| 深度摄像头 | 通过 `ros2 topic list` 查询实际 OAK-D 名称 | `/j100_0949/sensors/camera_0/depth/image` |

> **重要**：两个机器人的 TF 消息里可能都有 `odom` / `base_link` 这种 frame ID；**Topic 命名空间不能自动让 frame ID 唯一**。Host 做 RViz、TF 分析时，应分别订阅正确的 `/tf` / `/tf_static`，不要把两套 TF 直接合并。

### 2.2 Jackal 已验证的运动控制链

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
                           Twist Mux / 指令优先级
                                      │
                         /j100_0949/platform/cmd_vel
                                      │
                         platform_velocity_controller
                             最终正常软件限幅
                                      │
                               Jackal 电机
```

- **正常运动指令**最终经过 `platform_velocity_controller` 限速。
- Twist Mux 当前优先级：`rc=12`、`joy=10`、`interactive_marker=8`、`external(cmd_vel)=1`；`safety_stop=254`、`emergency_stop=255` 为锁定输入。
- Docking Server 可直接向 `/j100_0949/cmd_vel` 发布，**绕过 Nav2 Velocity Smoother / Collision Monitor，但仍经过 Twist Mux 与底盘控制器**；因此 Collision Monitor 不是所有运动来源的统一安全屏障。
- **实体 Motion Stop / 紧急停止**属于独立的安全手段；软件速度限幅不是物理速度保证。测试前确保车辆静止、场地空旷并能立即停止。
- `/cmd_vel`（根命名空间）已有 TurtleBot `teleop_twist_joy_node` 与 `create3_repub`；**不要以为它在控制 Jackal**。

---

## 3. Jackal 端：`robot@cpr-j100-0949`

**命令执行位置：Jackal 车载计算机**。可在 Host 上先：

```bash
ssh robot@192.168.185.201
```

### J-1. 初始化 ROS 环境与只读基本检查

```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash

hostname
ip -br -4 addr                  # br0 应同时含 .185.201 和 .131.1
ip -4 route
printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

ros2 node list | sort
ros2 topic list | grep '^/j100_0949/' | head -60
```

> 当日实测环境：`ROS_DOMAIN_ID=0`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。如果环境输出不一致，先排查 Shell 启动脚本，不要直接覆盖既有 XML 配置。

### J-2. Clearpath 服务和硬件状态（只读）

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

# 看速率，使用 Ctrl+C 结束
ros2 topic hz /j100_0949/platform/odom
ros2 topic hz /j100_0949/sensors/lidar3d_0/scan
```

### J-3. 启动 SLAM（终端 J-A；需保持运行）

> **仅在未运行同一套 SLAM 的时候执行。** 正在运行时不要再开第二个 `slam.launch.py`。

```bash
source /etc/clearpath/setup.bash

ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

另开 Jackal 终端验证（只读）：

```bash
source /etc/clearpath/setup.bash
ros2 node list | grep slam_toolbox
ros2 topic info /j100_0949/map
ros2 topic hz /j100_0949/map
```

### J-4. 启动自定义低速 Nav2（终端 J-B；需保持运行）

> **务必使用自定义 Launch，不要再用默认 `clearpath_nav2_demos nav2.launch.py`。** 启动前确认旧 Nav2 已停止，不要同时运行两套控制器。

```bash
source /etc/clearpath/setup.bash

ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" \
  setup_path:=/etc/clearpath \
  scan_topic:=/j100_0949/sensors/lidar3d_0/scan \
  use_sim_time:=false
```

只读验证：

```bash
for node in controller_server velocity_smoother collision_monitor bt_navigator; do
  echo "===== $node ====="
  ros2 lifecycle get "/j100_0949/$node"
done

# 正常应为 active [3]
ps -eo pid,args | grep '[r]os2 launch'
```

### J-5. 已生效的低速参数与配置文件

已确认**底盘层**（持久化）：

| 控制项 | 当前参数 |
|---|---|
| 前进、后退 | `linear.x.max_velocity=0.3`、`min_velocity=-0.2` m/s |
| 前进/后退加速 | `linear.x.max_acceleration=0.3`、`max_acceleration_reverse=-0.3` m/s² |
| 转弯 | `angular.z.max_velocity=0.6`、`min_velocity=-0.6` rad/s |
| 角加速 | `angular.z.max_acceleration=0.6`、`max_acceleration_reverse=-0.6` rad/s² |
| 原减速上限保留 | 线/角方向的 `max_deceleration=-4.0` 等配置 |

已确认**Nav2 层**（自定义 Launch 读取的持久文件）：

| 组件 | 配置 |
|---|---|
| MPPI | `vx_max=0.3`、`vx_min=-0.2`、`wz_max=0.6` |
| MPPI 加速/负向加速度 | `ax_max=0.3`、`ax_min=-3.0`、`az_max=0.6` |
| Velocity Smoother | `max_velocity=[0.3,0,0.6]`、`min_velocity=[-0.2,0,-0.6]` |
| Velocity Smoother | `max_accel=[0.3,0,0.6]`、`max_decel=[-4,0,-4]` |

> `FollowPath.ax_min=-3.0` 是 MPPI 的负向加速度边界，**不要误读为“最大后退速度”**。这些参数是当前室内低速测试值，不是厂商认证的安全阈值。

验证运行值：

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

配置文件位置（**不要盲目覆盖**）：

```text
/etc/clearpath/robot.yaml                          # Jackal 底盘统一限速，重启保留
/home/robot/jackal_nav2/nav2_low_speed.yaml        # 自定义 Nav2 参数
/home/robot/jackal_nav2/nav2_low_speed.launch.py   # 自定义 Nav2 启动文件
/home/robot/fastdds_185.xml                        # Jackal DDS 接口绑定
/etc/netplan/50-cloud-init.yaml                    # Jackal 网络，br0 双 IP
/etc/clearpath/platform/config/twist_mux.yaml     # 指令仲裁优先级
```

验证 Launch 真正引用自定义 YAML：

```bash
grep -n -A 2 'file_parameters =' \
  "$HOME/jackal_nav2/nav2_low_speed.launch.py"
python3 -m py_compile "$HOME/jackal_nav2/nav2_low_speed.launch.py"
ros2 launch "$HOME/jackal_nav2/nav2_low_speed.launch.py" --show-args
```

### J-6. 指令来源、限幅输出与车体里程计（只读）

```bash
ros2 topic info -v /j100_0949/cmd_vel_nav
ros2 topic info -v /j100_0949/cmd_vel_smoothed
ros2 topic info -v /j100_0949/cmd_vel
ros2 topic info -v /j100_0949/platform/cmd_vel
ros2 topic info -v /j100_0949/platform/cmd_vel_out

ros2 node info /j100_0949/platform_velocity_controller

# 观察实际里程计；如果车辆不动，速度为零属正常
ros2 topic echo /j100_0949/platform/odom --field twist.twist

# 观察限幅后的命令；没有新命令时不一定持续输出
ros2 topic echo /j100_0949/platform/cmd_vel_out --field twist
```

### J-7. 日志与配置确认（只读）

```bash
sudo journalctl -u clearpath-robot.service \
  -u clearpath-platform.service \
  -u clearpath-sensors.service -n 100 --no-pager

systemctl show clearpath-platform.service -p Environment --no-pager
systemctl show clearpath-sensors.service -p Environment --no-pager

sed -n '1,160p' /etc/netplan/50-cloud-init.yaml
sed -n '1,160p' /etc/clearpath/platform/config/twist_mux.yaml
```

### J-8. 正常停止测试

1. 先取消 RViz/Nav2 的导航目标、确认机器人已静止；Motion Stop 保持可用。
2. 在 **J-B（Nav2 Launch）终端**按 `Ctrl+C`；等待节点退出。
3. 在 **J-A（SLAM Launch）终端**按 `Ctrl+C`；若要继续建图则可保留 SLAM。
4. **不必停止**系统的 `clearpath-platform` / `clearpath-sensors` 服务。
5. 避免 `kill -9`、向运动 Topic 随意发送非零速度、或在 SSH 会话中直接改 Netplan 地址。

---

## 4. TurtleBot 4 端：`ubuntu@192.168.185.101`

**命令执行位置：TurtleBot 4 Raspberry Pi**。从 Host 登录：

```bash
ssh ubuntu@192.168.185.101
```

### T-1. 初始化与检查网络

```bash
source /opt/ros/jazzy/setup.bash

hostname
ip -br -4 addr       # eth0=.185.101; usb0=.186.3
ip -4 route
printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

# 只读检查持久化网络配置
sudo sed -n '1,200p' /etc/netplan/40-ethernets.yaml
```

已验证的网络分工：

| 接口 | 当前地址 | 功能 |
|---|---|---|
| `eth0` | `192.168.185.101/24` ✅ | Host/Jackal 共享 LAN |
| `usb0` | `192.168.186.3/24` ✅ | Raspberry Pi ↔ Create 3 内部通信；**不要改成 185 地址** |
| `wlan0` | `10.33.16.163/23`（当时） | 校园 Wi-Fi，DHCP 可能变化 |

`/etc/netplan/40-ethernets.yaml` 已保存 `eth0` 静态 IP、默认路由（指向 `.185.1`）和 `usb0` 的内部地址，重启后曾验证保留；通常只需**检查，不需要重新配置**。

### T-2. 检查 Create 3、雷达和相机话题

```bash
source /opt/ros/jazzy/setup.bash

ros2 node list | sort
ros2 topic list -t | sort

# TurtleBot 根命名空间，区别于 Jackal /j100_0949/*
ros2 topic info -v /odom
ros2 topic info -v /scan
ros2 topic info -v /cmd_vel
ros2 topic info -v /tf

# 统计发布频率，Ctrl+C 退出
ros2 topic hz /odom
ros2 topic hz /scan

# 只打印一条里程计
ros2 topic echo /odom --once

# 依实际启动的话题确认 OAK-D，相机未在线时可能找不到
ros2 topic list | grep -Ei 'oakd|camera|image|depth'
```

曾从 Host 实测：TurtleBot `/odom` 约 **20 Hz**，`/scan` 约 **8.1 Hz**。`/odom` 的 Publisher 曾确认为 `create3_repub`。摄像头当前是否发布需要以当次 `ros2 topic list` 和 `topic hz` 为准。

**注意**：当前没有实测证据表明 TurtleBot 本机运行独立的 SLAM 或 Nav2，不在此编造 TurtleBot SLAM/Nav2 启动命令。TurtleBot 的全局 `/cmd_vel` 与 Jackal `/j100_0949/cmd_vel` 是不同目标，**在共享 Domain 0 上不要直接 `ros2 topic pub` 非零速度测试**。

### T-3. TurtleBot 的 Fast DDS 与网络问题排查

```bash
printenv | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_AUTOMATIC_DISCOVERY_RANGE|FASTDDS|FASTRTPS'

# 已部署的 DDS XML；只读
sed -n '1,160p' /etc/turtlebot4/fastdds_rpi.xml

# 测试到 Host 与 Jackal 的单播连通性
ping -c 3 -W 1 192.168.185.2
ping -c 3 -W 1 192.168.185.201

ip -4 route get 192.168.185.2
ip -4 route get 192.168.185.201
```

> Fast DDS 自定义 XML 和 Create 3 的 `usb0` 原本工作正常。除非明确定位到相关故障，否则不要删除 XML、改 USB 内网或做 `netplan apply`（可能中断 SSH）。

---

## 5. Host 端：Alienware

**命令执行位置：Host 的本地终端**（例如 `nyu@nyu-Alienware-18-Area-51-AA18250`）。

### H-1. 快速网络验证（只读）

```bash
ip -br -4 addr
ip -4 route
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status

# 正常 Host 有线接口：enp132s0 = 192.168.185.2/24
ip -4 route get 192.168.185.101
ip -4 route get 192.168.185.201

ping -c 3 -W 1 192.168.185.101     # TurtleBot
ping -c 3 -W 1 192.168.185.201     # Jackal
```

如两个 `route get` 没有走 `enp132s0`，先检查本机 Wi-Fi/有线网络优先级。Host 原有校园 Wi-Fi 属另一个子网；**不要把 Host 旧规划地址 `192.168.185.10` 当作现用地址**，当前实测是 `.2`。

### H-2. SSH 到两个机器人

```bash
ssh ubuntu@192.168.185.101  # TurtleBot 4
ssh robot@192.168.185.201   # Jackal J100
```

### H-3. ROS 2 环境与两台机器人 Topic 检查

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0

printenv | grep -E '^(ROS_|RMW_|FASTRTPS_|FASTDDS_)'

# 共用 Domain 0，但不同 Topic 命名空间
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

### H-4. 跨设备数据质量：验证实际在传，不只看到话题名

```bash
# 每条命令持续统计，Ctrl+C 结束，再测下一条
ros2 topic hz /odom                                      # TurtleBot：历史约 20 Hz
ros2 topic hz /scan                                      # TurtleBot：历史约 8.1 Hz
ros2 topic hz /j100_0949/platform/odom                    # Jackal：历史约 50 Hz
ros2 topic hz /j100_0949/sensors/lidar3d_0/scan
ros2 topic hz /j100_0949/sensors/camera_0/color/image    # 历史约 29.6 FPS
ros2 topic hz /j100_0949/sensors/camera_0/depth/image    # 历史约 14~15 FPS
```

历史速率供对照，不保证每次启动都相同。**Topic 能被发现不代表正在持续发布数据**；实际以 `topic hz`、`topic info -v`、硬件状态为准。

### H-5. RViz 可视化（只在 Host 运行，不启动第二套 Nav2）

若已安装 Clearpath 可视化包，可尝试：

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix clearpath_viz

ros2 launch clearpath_viz view_navigation.launch.py \
  namespace:=/j100_0949 \
  use_sim_time:=false
```

只想单独启动 RViz、检查 Jackal 的 namespaced TF 时：

```bash
source /opt/ros/jazzy/setup.bash
rviz2 --ros-args \
  -r /tf:=/j100_0949/tf \
  -r /tf_static:=/j100_0949/tf_static
```

- 查看 **Jackal** 时，Map Topic 指向 `/j100_0949/map`，LiDAR 指向 `/j100_0949/sensors/lidar3d_0/scan`；Fixed Frame 通常取 `map`（前提是对应 TF 链存在）。
- 查看 **TurtleBot** 时，使用全局 `/scan`、`/odom`、`/tf`、`/tf_static`；不要直接把 TurtleBot 与 Jackal 的 TF 订阅混起来。
- Host 曾出现 NVIDIA `Driver/library version mismatch` / RViz `GLX` 初始化问题；如果再次打不开，先运行 `nvidia-smi`、`glxinfo -B`。这与 ROS 通信是否通畅是两个独立问题。
- **Host 只做可视化，SLAM 和 Nav2 已在 Jackal 上运行；不要在 Host 再启动同命名空间的导航栈。**

### H-6. 可选：Host 录制两台机器的核心 ROS Bag（只读订阅）

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

`Ctrl+C` 正常结束并写入 metadata；完成后用 `ros2 bag info <bag_directory>` 检查。默认不录全速点云/图像，以减少网络带宽和磁盘压力；如需回放 `/tf_static`，注意静态 TF 的 QoS 设置。

---

## 6. 每日启动、验证和停止流程

### 6.1 开机后的推荐顺序

1. **Host**：确认有线 `enp132s0=192.168.185.2`，可以 ping `192.168.185.101` 和 `192.168.185.201`。
2. **TurtleBot**：开机后确认 `eth0=.185.101`、`usb0=.186.3`，`/odom`、`/scan` 有数据；无需为此修改内部 Create 3 网络。
3. **Jackal**：开机后确认 Clearpath 三个服务 active、雷达和里程计发布正常。
4. **Jackal J-A 终端**：运行 `slam.launch.py`（若已运行，则不重复启动）。
5. **Jackal J-B 终端**：运行 **`~/jackal_nav2/nav2_low_speed.launch.py`**（若已运行，则不重复启动）。
6. **Jackal/Host**：检查 Nav2 Lifecycle、速度参数与 topic 连接；Host 只运行 RViz/监测。
7. **场地安全检查后**才给出小幅导航目标，首次试跑保持 Motion Stop 可随时触发。

### 6.2 三端快速核对卡（复制到对应终端）

**Host：**

```bash
ip -br -4 addr
ping -c 2 -W 1 192.168.185.101
ping -c 2 -W 1 192.168.185.201
ros2 topic info /odom
ros2 topic info /j100_0949/platform/odom
```

**TurtleBot：**

```bash
source /opt/ros/jazzy/setup.bash
ip -br -4 addr
ros2 topic info -v /odom
ros2 topic info -v /scan
```

**Jackal：**

```bash
source /etc/clearpath/setup.bash
systemctl is-active clearpath-robot clearpath-platform clearpath-sensors
ros2 node list | grep -E 'slam_toolbox|controller_server|velocity_smoother|collision_monitor'
ros2 lifecycle get /j100_0949/controller_server
ros2 param get /j100_0949/platform_velocity_controller linear.x.max_velocity
ros2 param get /j100_0949/controller_server FollowPath.vx_max
```

### 6.3 结束实验

- 先取消导航目标、确认机器人完全停止；保持 Motion Stop 可随时操作。
- 依次在 Jackal 的 **Nav2**、**SLAM** Launch 终端按 `Ctrl+C`；不需要关掉 Clearpath 平台服务。
- Host 的 RViz、bag 录制用 `Ctrl+C` 正常退出。
- 若只需下次继续实验，已保存的 `robot.yaml`、`nav2_low_speed.yaml`、`nav2_low_speed.launch.py` 不需重新编辑。
- **重启并不会自动启动自定义 SLAM/Nav2**：目前它们通过手动 Launch 运行；如果未来要开机自启，需另建并验证 service / Launch 管理配置。

---

## 7. 常见故障排查

| 症状 | 优先检查 | 原则 |
|---|---|---|
| SSH / Ping 不通 | `ip -br -4 addr`、`ip route get <IP>`、`ping -c 3 <IP>` | 先确认接口/IP/链路，不要立即改 DDS |
| Host 能 ping，但看不见 Jackal Topic | 两端 Domain/RMW、Jackal Fast DDS XML、`/j100_0949/` Namespace | Jackal `br0` 有 `.131.1` + `.185.201` 双地址，不要错误使用 `.131.1` 发现路径 |
| 能看到 Topic，但数据频率为 0 | `ros2 topic info -v`、`ros2 topic hz`、硬件驱动/节点日志 | ROS graph 中出现话题不代表持续有数据 |
| TurtleBot 里程计/Scan 异常 | `ros2 topic info -v /odom`、`/scan`、检查 `usb0=.186.3` | TurtleBot 内部 Create 3 网络独立，不要改其地址段 |
| Jackal Nav2 不激活 | `ros2 lifecycle get ...`、Launch 终端日志 | 生命周期 `active [3]` 才表示已激活 |
| Nav2 速度参数重启后变回默认 | `ps -eo pid,args | grep '[r]os2 launch'`、`grep file_parameters ...` | 必须使用 `nav2_low_speed.launch.py`，不要使用官方默认 Nav2 启动命令 |
| Jackal 运动速度限制不一致 | `ros2 param get ... platform_velocity_controller`、`controller_server` 和 `velocity_smoother` | 以底盘正常指令最终限幅为底线，Nav2 同步规划约束 |
| `/cmd_vel` 看似存在但 Jackal 不运动 | 区分 `/cmd_vel` 与 `/j100_0949/cmd_vel`，检查 Twist Mux | 全局 `/cmd_vel` 当前属于 TurtleBot 链路，Jackal 使用 namespaced Topic |
| RViz 打不开 OpenGL | `nvidia-smi`、`glxinfo -B` | 图形驱动问题不代表 ROS 网络坏了 |

**DDS 诊断（Jackal；仅需要时）**：

```bash
# 确认服务配置了自定义 XML
systemctl show clearpath-platform.service -p Environment --no-pager
systemctl show clearpath-sensors.service -p Environment --no-pager

# 如果 tcpdump 已安装，可观察发现包；Ctrl+C/timeout 后退出
sudo timeout 10s tcpdump -ni br0 \
  'udp and dst host 239.255.0.1 and dst port 7400' -c 15
```

---

## 8. 配置文件与变更原则

| 所在端 | 文件 | 作用 | 当前状态 |
|---|---|---|---|
| Jackal | `/etc/clearpath/robot.yaml` | 底盘最终软件限速与机器人配置 | ✅ 已保存、服务已加载 |
| Jackal | `/home/robot/jackal_nav2/nav2_low_speed.yaml` | MPPI + Velocity Smoother 限速 | ✅ 已保存并验证运行值 |
| Jackal | `/home/robot/jackal_nav2/nav2_low_speed.launch.py` | 启动自定义低速 Nav2 | ✅ 进程已显示使用该文件 |
| Jackal | `/etc/clearpath/platform/config/twist_mux.yaml` | 控制源仲裁优先级 | ✅ 已读取 |
| Jackal | `/home/robot/fastdds_185.xml` | 指定有效 Fast DDS 网卡/IP | ✅ 已部署至平台/传感器服务 |
| Jackal | `/etc/netplan/50-cloud-init.yaml` | `br0` 外部/内部双地址 | ✅ 已持久化验证 |
| TurtleBot | `/etc/netplan/40-ethernets.yaml` | `eth0` 外部 / `usb0` Create 3 内部 | ✅ 已持久化验证 |
| TurtleBot | `/etc/turtlebot4/fastdds_rpi.xml` | TurtleBot Fast DDS Profile | ✅ 已存在并读取 |
| Host | Ethernet NetworkManager/Netplan | 当前 `enp132s0=.185.2` | ⚠️ 永久配置存储位置未核实 |

**操作约束：**

- 不要将 TurtleBot 的 `/odom`、`/scan`、`/cmd_vel` 直接替换为 Jackal 的同名想象路径；Jackal 使用 `/j100_0949/...`。
- 不要直接编辑 ROS 安装包 `/opt/ros/jazzy/share/clearpath_nav2_demos/...` 的官方文件；优先修改 `/home/robot/jackal_nav2/` 中的副本。
- 不要在有导航目标执行期间修改控制器参数、重启服务或变更网络路由。
- 修改网卡配置前做备份，避免远程 SSH 断连；正在正常工作的配置尽量保持不动。
- 确认的是**正常 ROS 软件控制链**，不是所有工况下车辆物理速度的硬件安全保证。

### 参考资料

- [Clearpath ROS 配置：Platform extras](https://docs.clearpathrobotics.com/docs/ros/config/yaml/platform/extras/)
- [Clearpath Platform API](https://docs.clearpathrobotics.com/docs/ros/api/platform_api/)
- [ROS 2 Controllers / diff_drive_controller（Jazzy）](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Nav2 MPPI Controller（Jazzy）](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)

---

**最后更新：2026-10-09。** 本手册以最新实测的 `Host .185.2 / TurtleBot .185.101 / Jackal .185.201` 为准；实验后若 IP、ROS Domain、Launch 文件或 Topic 有变更，应更新此文档。 
