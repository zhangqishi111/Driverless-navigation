# Driverless Navigation

基于 Raspberry Pi 5 LanderPi 的 ROS 2 SLAM 导航系统。

当前开发基线：

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- SLAM Toolbox
- Nav2 / AMCL
- robot_localization
- RViz2

## 工作空间结构

```text
Driverless-navigation/
├── src/
│   ├── landerpi_base_driver/       # 底盘、里程计及安全接口
│   ├── landerpi_bringup/           # 系统启动、参数、地图和RViz配置
│   ├── landerpi_description/       # URDF/Xacro及仿真模型
│   ├── landerpi_error_evaluator/   # 到达判断和误差计算
│   ├── landerpi_msgs/              # 项目自定义消息（后续启用）
│   ├── landerpi_navigation/        # Nav2与AMCL配置
│   ├── landerpi_sandbox_display/   # 沙盘显示界面
│   ├── landerpi_slam/              # SLAM Toolbox配置
│   ├── landerpi_task_manager/      # 多目标任务（提高项）
│   └── landerpi_test/              # 集成测试和验收工具
├── build/                           # colcon生成，不提交
├── install/                         # colcon生成，不提交
└── log/                             # colcon生成，不提交
```

## 编译

```bash
cd ~/Driverless-navigation
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## A组员：底盘仿真接口验证

完成编译并加载工作空间后，一条命令启动测试场景、小车、激光雷达、
ROS-Gazebo桥接和里程计TF：

```bash
ros2 launch landerpi_base_driver base_sim.launch.py
```

另开一个终端，加载环境并发送低速前进命令：

```bash
source /opt/ros/humble/setup.bash
source ~/Driverless-navigation/install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

确认标准接口和TF存在：

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

预期结果：Gazebo中的小车向前运动，`/scan`约为10 Hz，`/odom`持续更新，
并且两段TF都能查询到。当前模型是无实物阶段使用的替代模型；拿到LanderPi后，
需按实测尺寸、轮距、轮径和厂商驱动接口更新，标准Topic与TF名称保持不变。

## 统一接口

核心Topic：`/scan`、`/odom`、`/cmd_vel`、`/map`、`/robot_pose`、
`/goal_pose`、`/plan`、`/local_plan`、`/actual_path`。

核心TF：`map -> odom -> base_link -> laser_link`。

建图模式由 SLAM Toolbox 发布 `map -> odom`；导航模式由 AMCL 发布，二者不能同时发布。
