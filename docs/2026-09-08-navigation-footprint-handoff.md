# 矩形碰撞检查改动与实车验证交接

日期：2026-09-08；负责：成员 C；分支：feature/navigation。

## 今天交付什么

本说明随待审核改动提供。目前尚未提交和推送；C 确认并推送后，测试同学才能从远程拉取。不要把旧提交 ed82763 当成已包含本次修改。

修改文件：src/landerpi_navigation/config/nav2_params.yaml。

| 项目 | 原配置 | 本次配置 |
|---|---|---|
| DWB 障碍评分器 | BaseObstacle | ObstacleFootprint |
| 对应权重 | BaseObstacle.scale: 0.02 | ObstacleFootprint.scale: 0.02 |
| 局部/全局脚印 | ±0.135 m × ±0.105 m | 保持一致、不变 |
| 局部/全局膨胀 | 0.10 m，scaling 3.0 | 不变 |
| 全局规划器 | NavFn | 不变 |
| 横移、速度限制、定位参数 | 原配置 | 不变 |

原因：原 BaseObstacle 检查轨迹参考点的栅格代价，并不按矩形四条边检查碰撞。ObstacleFootprint 在各预测姿态下检查矩形轮廓，轮廓碰到致命障碍/未知区会拒绝轨迹，普通膨胀代价用于评分。这是离散地图、离散轨迹的轮廓检查，不是实车连续碰撞的绝对保证，也不是已经解决全部停车问题。

注意：不能把所有膨胀颜色都当成禁止通行；也不能要求整个脚印必须离开全部膨胀颜色才能导航。0.10 m 从障碍栅格计算，并不是车壳外再留 10 cm。矩形自身已含每侧约 2 cm 余量，另有 footprint_padding 需现场读取。共用参数文件也会影响使用该文件的仿真，但此脚印来自实车尺寸，不能据此认定仿真模型匹配。

## 开始前：进入正确环境、更新并核验版本

由现场一人统一控制车辆，其他人暂停手柄和自动跟随等控制任务。上一轮曾出现重复节点及命令结束后车辆继续运动；未确认停止能力前不要发送导航目标。不要直接用批量 pkill 清理他人的进程。

Windows PowerShell（IP 以当天树莓派实际地址为准）：

```powershell
ssh pi@192.168.1.140
```

树莓派宿主机：

```bash
sudo docker exec -it -u ubuntu -w /home/ubuntu MentorPi /bin/bash
```

以下命令均在容器 Bash 的 ubuntu 用户下执行。每个新终端都加载环境；Domain 3 需与现场底盘一致，不要临时各改各的：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=3
cd ~/Driverless-navigation
```

仅在 C 已确认推送后更新，先检查工作区：

```bash
git status --short
git branch --show-current
```

应处于 feature/navigation 且无待保护改动；否则请仓库负责人处理，不强行覆盖。满足条件后：

```bash
git pull --ff-only origin feature/navigation
git log -1 --oneline
colcon build --base-paths src --packages-select landerpi_navigation
source install/setup.bash
grep -nE 'critics:|ObstacleFootprint.scale|inflation_radius:|footprint:' \
  "$(ros2 pkg prefix landerpi_navigation)/share/landerpi_navigation/config/nav2_params.yaml"
```

安装后的参数必须含 ObstacleFootprint 与权重 0.02，两处 inflation_radius 为 0.10。构建失败不要继续。

## 启动与记录

厂家底盘、雷达、EKF、TF 按已有流程只启动一套；不启动 SLAM。本次无需重改厂家 LD19/xacro 或导入修复。查询：

```bash
ros2 topic info /scan -v
ros2 topic hz /scan
# 观察约 10 秒后 Ctrl+C，应该持续有数据；记录掉帧/长间隔。
ros2 run tf2_ros tf2_echo odom base_link
# 持续输出变换后 Ctrl+C；短暂发现等待不等于故障。
ros2 node list | grep cmd_vel_adapter
ros2 topic info /cmd_vel -v
ros2 topic info /controller/cmd_vel -v
```

转发器应只有一个，订阅 /cmd_vel、发布 /controller/cmd_vel。发布端数量不等于实际正在发消息；必须查节点名和实际输出。确实没有转发器时，在已加载项目环境的新终端运行并保持：

```bash
ros2 launch landerpi_base_driver real_base.launch.py
```

让旧导航启动终端 Ctrl+C 正常退出，再在导航终端启动本次版本并保存日志。不要同时启动两份导航：

```bash
source ~/Driverless-navigation/install/setup.bash
mkdir -p ~/nav_validation
RUN_DIR="$HOME/nav_validation/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
echo "$RUN_DIR"
set -o pipefail
ros2 launch landerpi_navigation navigation_real.launch.py 2>&1 | tee "$RUN_DIR/navigation.log"
```

另一个已加载环境的终端核验（不要仅凭配置文本判断运行已生效）：

```bash
ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.critics
ros2 param get /controller_server FollowPath.ObstacleFootprint.scale
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap obstacle_layer.enabled
ros2 param get /amcl use_sim_time
```

应为 active、列表含 ObstacleFootprint 不含 BaseObstacle、权重 0.02、两个半径 0.10、激光障碍层 true、实车时钟 false。若插件加载失败或参数不存在，停止验收，保留启动日志。

RViz 在已能打开图形界面的容器桌面终端启动（普通 SSH 无图形环境时不能保证弹窗）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/Driverless-navigation/install/setup.bash
export ROS_DOMAIN_ID=3
rviz2 -d "$(ros2 pkg prefix landerpi_bringup)/share/landerpi_bringup/rviz/slam_navigation.rviz"
```

Fixed Frame 用 map；显示 /map、/scan、/local_costmap/costmap（Color Scheme=costmap）、/local_costmap/published_footprint（Polygon）。必要时暂时关闭 RobotModel/TF 显示以看清矩形，勿把关闭显示误当成停止节点。初始定位用 2D Pose Estimate，先确认激光与现场墙角匹配，再点 2D Goal Pose。

## 项目一：脚印尺寸、中心和定位测量

1. 车辆停稳，用尺测包括轮胎、固定外伸件和当前机械臂姿态在内的水平投影长宽，记录照片及厘米数，不只测底盘板。若投影超过 23×17 cm，当前矩形可能偏小，应先交给 C 调整。
2. 让 A 根据实际加载的厂家 URDF 确认 base_link 原点在车体哪里；不要从 RViz 模型外观猜。相对该点量前、后、左、右边界距离，分别记为 F、B、L、R。已有激光偏移 7.3 cm 只能辅助定位，不能证明车体中心。
3. 当前矩形假定 F=B=11.5 cm、L=R=8.5 cm，再各加 2 cm。若原点偏心，应按实测四向距离生成非对称脚印，而非继续统一缩小。
4. 执行并保存：

```bash
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint
ros2 param get /local_costmap/local_costmap footprint_padding
ros2 param get /global_costmap/global_costmap footprint_padding
ros2 topic echo /local_costmap/published_footprint --once
ros2 run tf2_ros tf2_echo base_link lidar_frame
```

published_footprint 坐标在其 header 指明的坐标系中，不是以零为中心的车体尺寸；转动后的 x/y 最大值差也不能直接当车长宽。若 padding=0.01，配置矩形之外还会有额外约 1 cm 填充，先记录，明天不要自行清零。

验证看什么：实际外伸部分是否被脚印覆盖，原点是否正确；静止及转动后激光是否仍贴合地图。若墙线出现明显错位或跳变，先排定位，不能用缩脚印掩盖。

## 项目二：膨胀参数对照

先完成项目一、四的基础检查，并确认车辆可停止。只在没有进行中的目标时切参数。没有取消目标的界面时，在导航启动终端 Ctrl+C 结束导航并确认停车，重启后再设置参数；不要拿清地图代替取消目标。

方案 A 是今天保留的基线：矩形评分器 + 半径 0.10 + scaling 3.0。选空旷近目标、平行靠墙直行、路口左转、路口右转各一条路线，用胶带标记起点位置/朝向、记录目标坐标。每条重复 3 次，现场留足停止空间。

方案 B 只调整两处膨胀（试验候选，未认证最优）：

```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.25
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 8.0
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 8.0
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
```

等地图和激光更新后再发送目标。重复同样路线和起点，不同时改速度、脚印或定位参数。scaling 越大，软代价随距离衰减越快；半径越大不等于整圈都禁止通过。NavFn 仍可能生成对矩形转弯不够宽裕的路线，本次没有更换全局规划器。

验证看什么：成功次数、完成时间、车壳到障碍最小实际间隙、是否频繁停顿/左右摇摆、是否出现碰撞拒绝日志。车壳不得擦碰；现场测得越过预留安全余量即停止该路线并记录，不靠继续缩参数强行通过。

测试完成恢复方案 A（四项都恢复）：

```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.10
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 3.0
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.10
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 3.0
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
```

这些是运行时参数，重启恢复仓库基线；把选出的结果发给 C 后再决定永久修改。

## 项目三：证据采集与问题判断

每轮在独立终端先录包，再点目标，车辆停止后 Ctrl+C 正常结束录制：

```bash
mkdir -p ~/nav_validation
ros2 bag record -o "$HOME/nav_validation/bag_$(date +%Y%m%d_%H%M%S)" \
  /tf /tf_static /scan /odom /amcl_pose /map \
  /cmd_vel /controller/cmd_vel /plan \
  /local_costmap/costmap /local_costmap/published_footprint \
  /global_costmap/costmap /navigation_status /rosout
```

额外终端看状态与速度；每个 echo 单独运行，Ctrl+C 只停止观察：

```bash
ros2 topic echo /navigation_status
ros2 topic echo /cmd_vel
ros2 topic echo /controller/cmd_vel
```

保存现场视频和 RViz 截图，记录哪个袋子属于 A/B 哪次路线。rosbag info 后跟实际 bag 目录可检查所需话题是否真的录到；消息数为 0 的话题不能充当证据。/plan 或状态话题未出现时先用 ros2 topic list 核实名称。

| 观察 | 下一步判断 |
|---|---|
| ObstacleFootprint / Trajectory Hits Obstacle | 预测轮廓与障碍栅格相交；比对当时定位、脚印和地图 |
| Trajectory Hits Unknown Region / Goes Off Grid | 未知区或局部地图边界问题 |
| 有速度输出，转发端没有 | 转发器、话题映射或 Domain 问题 |
| 两端均有转向命令，实车不转 | 底盘控制/最小有效速度问题 |
| Failed to make progress | 检查实际移动、里程计、低速死区；不只看膨胀区 |
| AMCL 丢扫描、TF 过期或 LD19 timeout | 先排传感器/时钟/负载，再评价规划参数 |

记录模板：版本 SHA；A/B；路线；起点/目标；3 次成功数；耗时；最小车壳间隙；是否碰撞；是否停顿；原始错误；bag 路径；视频名称。不能只反馈“走了/没走”。

## 项目四：转向、速度链路与脱困测量

先读取限制，不边测边提高：

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /controller_server FollowPath.min_vel_x
ros2 param get /controller_server FollowPath.max_vel_y
ros2 param get /velocity_smoother max_velocity
ros2 param get /velocity_smoother min_velocity
ros2 param get /cmd_vel_adapter max_linear_speed
ros2 param get /cmd_vel_adapter max_angular_speed
ros2 param get /cmd_vel_adapter command_timeout
ros2 param get /behavior_server max_rotational_vel
```

项目配置的 DWB/平滑器/转发器上限是 0.20 m/s、0.50 rad/s；DWB 不主动采样倒退/横移（min_vel_x=0、max_vel_y=0）。恢复行为另有转动上限 1.0 rad/s，可能被转发器截到 0.50，本次未改，应记录恢复是否受影响。不能因厂家底盘能横移就直接开放 Nav2 横移；AMCL 模型、平滑器与底盘映射需配套。

1. 在空旷处，用已验证可随时停止的厂家手柄由 A 分别操作小幅左转、右转。导航先正常关闭，但保留底盘和观察终端。用地面胶带标出起始方向，避免靠目测 RViz 当作真实角度。
2. 逐档小幅增加手柄转向输入，记录 /controller/cmd_vel 的 angular.z、实车是否开始转动、是否抖动；正 z 应左转，负 z 应右转（按车体朝前看）。各方向重复 3 次。此处不提供无人监护定时旋转命令；车曾在发布结束后继续运动，必须由现场确认停止控制。
3. 用下列命令读转动前后 yaw，结合地面实测角度判断，跨 ±180° 时取归一化角差，不能直接相减：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

两个命令分别在独立终端运行。转动约 90° 并停稳，记录真实角度、odom 变化、map 变化；重复左右两侧。两边输入相近但只有一边动，交 A 查底盘；odom 准而 map 跳，交 C 查定位。

4. 恢复导航，在空旷近目标及路口分别测试左右转，观察两端速度。若靠墙原地转受阻，记录是否存在向前挪动再转的空间。需要倒退/横移脱困则列为后续改动，不在此次测试直接开放。
5. 证明停车能力：停止手柄操作后观察实车停止、控制话题归零；导航目标取消后也观察停车。转发器超时仅针对它收到的 /cmd_vel，其他厂家控制源仍在发命令时不能依靠它保证停车。

紧急情况下优先现场硬件停止；只有在控制源已停止时，零速度才不会被后续非零消息覆盖。补发零速度命令如下，不替代取消导航或硬件停止：

```bash
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 回退方法

若新插件无法启动或明显退化，关闭本次导航并确认车辆停止。在项目目录导出旧参数到一个新文件，不重置分支：

```bash
cd ~/Driverless-navigation
ROLLBACK_PARAMS="$HOME/nav_validation/nav2_ed82763_$(date +%Y%m%d_%H%M%S).yaml"
git show ed82763:src/landerpi_navigation/config/nav2_params.yaml > "$ROLLBACK_PARAMS"
ros2 launch landerpi_navigation navigation.launch.py \
  use_sim_time:=false \
  map:="$(ros2 pkg prefix landerpi_bringup)/share/landerpi_bringup/maps/real_sandbox_clean.yaml" \
  params_file:="$ROLLBACK_PARAMS"
```

旧版仅作回退和比较，不意味着它已完整实现矩形轮廓碰撞检查。回退后重新初始定位并核验 FollowPath.critics；不要同时运行新旧导航。

## 源码依据与验收边界

- Humble BaseObstacle：https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp
- Humble ObstacleFootprint：https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp

今天的配置/插件检查不能代替实车验收。通过条件包括：运行插件正确、尺寸与定位可信、直行/左右转可重复完成、障碍前仍能停止/绕行、有足够实测间隙；“只要能动”不算完成。

本机已完成：Ubuntu-22.04 WSL 中 YAML 解析；与 HEAD 的参数语义对比（只有评分器名称及对应权重键变化）；Humble 插件 XML 声明核对和 DWB 动态库加载；Git 差异格式检查。未在树莓派启动新版控制器、未执行实车导航，插件存在于本机也不替代现场 active 状态核验。
