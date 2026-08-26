# Driverless Navigation

基于 Raspberry Pi 5 LanderPi 的 ROS 2 SLAM 导航系统。

当前开发基线：

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
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
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 统一接口

核心Topic：`/scan`、`/odom`、`/cmd_vel`、`/map`、`/robot_pose`、
`/goal_pose`、`/plan`、`/local_plan`、`/actual_path`。

核心TF：`map -> odom -> base_link -> laser_link`。

建图模式由 SLAM Toolbox 发布 `map -> odom`；导航模式由 AMCL 发布，二者不能同时发布。
