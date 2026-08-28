# landerpi_test

`landerpi_test` 用于 LanderPi SLAM 导航项目的集成测试与阶段验收。

本包主要由 E（测试与任务管理）维护，不重复实现 SLAM、定位、导航或显示功能，而是检查团队各模块是否按照统一接口正确协同工作。

## 1. 当前内容

```text
landerpi_test/
├── docs/
│   └── acceptance_checklist.md
├── scripts/
│   └── launch_sim_all.sh
├── CMakeLists.txt
└── package.xml
```

- `scripts/launch_sim_all.sh`：仿真启动与接口验收脚本
- `docs/acceptance_checklist.md`：阶段验收清单与问题记录

## 2. 正式环境基线

当前团队正式环境统一为：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Fortress
```

E 的验收脚本以 ROS 2 Humble 作为正式基线。若检测到其他 ROS 2 发行版，脚本会给出警告；非 Humble 环境仅可用于临时预检查，不作为正式验收依据。

## 3. 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to landerpi_test
source install/setup.bash
```

## 4. 只执行接口检查

```bash
ros2 run landerpi_test launch_sim_all.sh --check-only
```

脚本会检查核心 Topic：

- `/scan`
- `/odom`
- `/cmd_vel`

以及核心 TF：

- `odom -> base_link`
- `base_link -> laser_link`

同时观察以下运行期接口：

- `/map`
- `/robot_pose`
- `/goal_pose`
- `/plan`
- `/local_plan`
- `/actual_path`

## 5. 检查结果说明

脚本统一使用三种状态：

- `PASS`：接口存在并通过当前检查
- `FAIL`：核心接口缺失或检查失败
- `WAIT`：可选/运行期接口当前尚未出现

结束时会输出汇总，例如：

```text
Summary:
  PASS: 5
  FAIL: 0
  WAIT: 3
[RESULT] PASS
```

如果任一核心检查失败，脚本返回非 0 退出码：

```bash
echo $?
```

- `0`：核心检查通过
- `1`：至少一个核心检查失败
- `2`：完整验收被前置依赖阻塞，例如 `simulation.launch.py` 尚未提供

## 6. 启动完整仿真

当团队统一仿真入口 `landerpi_bringup/simulation.launch.py` 可用后，可执行：

```bash
ros2 run landerpi_test launch_sim_all.sh
```

脚本会调用团队统一仿真入口，等待初始化后再执行接口检查。

当前如果 `simulation.launch.py` 尚未合入，则完整启动模式暂时无法完成最终验收；此时可继续使用 `--check-only` 进行接口预检查。

## 7. 验收文档

阶段验收清单位于：

```text
src/landerpi_test/docs/acceptance_checklist.md
```

测试结果建议统一记录为：

- `PASS`
- `PARTIAL PASS`
- `FAIL`

发现问题时，应记录：

- 问题模块
- 问题现象
- 复现步骤
- 当前状态
- 负责人
- 修复后的复测结果

## 8. 测试原则

1. 正式验收环境以 Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress 为准。
2. 使用项目统一 Topic、Frame、Node 和 Launch 名称。
3. 不为测试另建与正式接口重复的 Topic 或 Frame。
4. 核心 TF 以 `map -> odom -> base_link -> laser_link` 为准。
5. 每个失败项都应可复现。
6. 修复后必须重新测试。
7. 正式验收以真实 Topic、TF、日志和实验数据为依据。
