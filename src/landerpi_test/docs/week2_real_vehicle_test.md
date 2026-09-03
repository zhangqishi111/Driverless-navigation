# Week 2 实车数据录制与回灌测试

## 1. 测试目标

验证 LanderPi 实车核心传感器接口 `/scan` 与 `/odom` 是否能够稳定发布，
并完成 rosbag 数据录制、完整性检查及离线回放验证，为后续 SLAM、定位和导航调试提供可复现测试数据。

## 2. 测试环境

- Platform: LanderPi Mecanum
- ROS: ROS 2 Humble
- ROS_DOMAIN_ID: 3
- LiDAR: MS200
- Vehicle host: Raspberry Pi
- ROS2 runtime: MentorPi Docker
- Test role: E - 测试与验收

## 3. 实车接口验证

### 3.1 `/scan`

- Message type: `sensor_msgs/msg/LaserScan`
- Publisher: `/scan_to_scan_filter_chain`
- Frame: `lidar_frame`
- Average frequency: approximately 10 Hz
- LaserScan range data: valid
- Result: **PASS**

### 3.2 `/odom`

- Message type: `nav_msgs/msg/Odometry`
- Publisher: `/ekf_filter_node`
- Frame: `odom`
- Child frame: `base_footprint`
- Average frequency: approximately 30 Hz
- Pose data changes with vehicle movement
- Result: **PASS**

## 4. Rosbag 录制结果

Recorded topics:

- `/scan`
- `/odom`

Bag: `rosbag2_2026_09_02-02_17_49`

- Duration: 45.140 s
- Bag size: 3.1 MiB
- Total messages: 1794
- `/scan`: 449 messages
- `/odom`: 1345 messages

Result: **PASS**

> rosbag 二进制数据仅作为本地测试数据保存，不提交至 Git 仓库。

## 5. Rosbag 回放验证

为避免与实车实时 Topic 冲突，回放时进行了 Topic 重映射：

- `/scan` → `/scan_replay`
- `/odom` → `/odom_replay`

Replay result:

- `/scan_replay` Publisher: detected
- `/odom_replay` Publisher: detected
- `/scan_replay`: approximately 10 Hz
- `/odom_replay`: approximately 30 Hz
- LaserScan message replay: PASS
- Odometry message replay: PASS

Result: **PASS**

## 6. 第二周验收结论

实车 `/scan` 与 `/odom` 均能够稳定输出有效数据。

已完成核心传感器数据 rosbag 录制、数据完整性检查和离线回放验证，
回放后的 LaserScan 与 Odometry 数据类型、内容和发布频率均正常。

**Week 2 real-vehicle data recording and replay test: PASS.**
