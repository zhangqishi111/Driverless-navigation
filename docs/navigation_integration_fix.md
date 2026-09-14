# 多目标与动态障碍修复

基线：feature/navigation-multi-goal-test，1b0592786132b9bb5539e8077ba273189011871b。
不需要先合入 dev。必须一起使用本目录中的消息、界面、导航和任务管理包。

## 改动

- 沿用基线的完整多目标链路：界面 PoseArray → task_queue → navigation_command_arbiter → Nav2；任务状态通过 NavigationTaskState 返回界面。
- 全局代价地图增加 /scan 障碍层，开启标记与清除，使重规划考虑可见的新障碍。局部障碍层继续执行碰撞检查。
- 没有任务订阅者时界面拒绝发送、明确显示原因并保留草稿；移除写死的 Connected 提示。

## Ubuntu / ROS 2 更新

将完整修复目录传到 Ubuntu 的独立目录，不要覆盖旧工作空间的 install。
关闭旧导航和界面进程。在新终端执行（将路径及 ROS 发行版替换为实际值）：

```bash
source /opt/ros/humble/setup.bash
cd /path/to/Driverless-navigation-fixed
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix landerpi_navigation
ros2 pkg executables landerpi_task_manager
ros2 launch landerpi_bringup real_system.launch.py mode:=navigation
```

上面的 real_system 会启动底盘、定位安全节点和完整导航，不要同时重复启动旧底盘或旧导航。
如果已有外部底盘启动流程，应在原流程中替换导航启动项为本工作空间的 navigation_real.launch.py，保持已有安全速度链路。
界面也必须在 source 此工作空间 install/setup.bash 后启动。

## 验证

在已 source 新工作空间的另一终端执行：

```bash
ros2 node list
ros2 topic info /navigation_task/goals -v
ros2 topic info /navigation_task/state -v
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /local_costmap/local_costmap plugins
```

必须出现 navigation_task_queue 和 navigation_command_arbiter；goals 至少一个订阅者；state 至少一个发布者；两个 costmap 都有 obstacle_layer。
先确认定位有效，再提交三个目标，检查状态从 Pending/Planning 进入 Navigating，并逐点完成。
在 RViz 同时显示 /scan、全局和局部 costmap、/plan；放入可见障碍后，两张代价地图都应标记障碍，下一次全局重规划应绕行或报告无路可走；移走障碍后在有效雷达清除射线覆盖下应恢复通行。
默认障碍标记距离为 2.5 m，不能据此推断不可见区域或距离外的障碍。

## 测试限制

Windows 可运行离线配置与接口回归检查，但没有 ROS 2、Qt 和实车环境。
在 Ubuntu 编译完成后运行 colcon test 和 colcon test-result --verbose；动态避障与停车距离仍需实车低速验证。
