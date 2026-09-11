# 实车地图定位与导航验收清单

这份清单使用 B 提供的最终地图：

- `src/landerpi_bringup/maps/real_sandbox_clean.pgm`
- `src/landerpi_bringup/maps/real_sandbox_clean.yaml`

不要在本流程中启动 SLAM Toolbox。导航模式下只允许 AMCL 发布
`map -> odom`。

## 一、无实物时可完成的检查

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to landerpi_navigation landerpi_task_manager landerpi_base_driver
source install/setup.bash

# 确认已安装的地图文件和 YAML 指向的图片一致。
MAP_DIR="$(ros2 pkg prefix landerpi_bringup)/share/landerpi_bringup/maps"
cat "$MAP_DIR/real_sandbox_clean.yaml"
test -f "$MAP_DIR/real_sandbox_clean.pgm"

# 只验证实车启动入口的参数展开；无需实物。
ros2 launch landerpi_navigation navigation_real.launch.py --show-args
```

预期 YAML 包含：`image: real_sandbox_clean.pgm`、`resolution: 0.05`、
`origin: [-6.7, -8.12, 0]`。实车启动入口必须使用
`use_sim_time:=false`。

## 二、实物到场后的启动顺序

### 1. 底层和传感器先就绪

先按厂家方式启动底盘、LD19 雷达、里程计和静态 TF；随后启动项目的
安全转发节点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch landerpi_base_driver real_base.launch.py
```

在另一个终端确认 A 提供的接口，而不是仅凭节点“已启动”判断：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

`/scan` 的 `frame_id` 必须与激光 TF 一致。若上述任何一项缺失或持续报
TF 错误，停止在此处并先由 A 修复底层链路。

### 2. 进入定位模式

确认 SLAM Toolbox 未运行后，启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch landerpi_navigation navigation_real.launch.py
```

另开终端检查 Nav2 生命周期和地图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once
```

`/map` 能收到消息且 `map_server` 为 `active`，才说明地图加载成功。

### 3. AMCL 定位验收（尚不下发导航目标）

1. 打开 RViz，选择 **2D Pose Estimate**。
2. 在地图中点击小车实际位置，并拖出小车真实朝向。
3. 等待粒子云收敛，观察激光点云是否与地图墙体重合。
4. 小车静止时，检查 `/amcl_pose` 和 RViz 图标是否基本不跳动。
5. 确认 `map -> odom` 连续稳定，且 SLAM 没有同时发布该 TF。

只有这些条件成立，才可以给一个近距离、空旷目标做首次导航测试。若
定位未收敛、雷达墙体不重合或 TF 不连续，先停止导航；不要通过调大速度、
增大代价地图或修改避障参数来掩盖定位问题。

## 三、首次短距离导航后的记录

记录初始位姿、目标位姿、是否到达、`/amcl_pose` 是否跳变，以及任何
Nav2 报错。麦克纳姆轮的 AMCL 运动模型、粒子参数、激光模型和速度限制
只根据这些真实记录进行后续调参。
