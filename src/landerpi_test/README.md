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

## 2. 编译

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to landerpi_test
source install/setup.bash
```

如果当前仅使用 ROS 2 Humble 进行本地预检查，可将第一行替换为：

```bash
source /opt/ros/humble/setup.bash
```

正式项目环境以团队最终统一基线为准。

## 3. 只执行接口检查

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

## 4. 检查结果说明

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

## 5. 启动完整仿真

当团队统一仿真入口 `landerpi_bringup/simulation.launch.py` 可用后，可执行：

```bash
ros2 run landerpi_test launch_sim_all.sh
```

脚本会调用团队统一仿真入口，等待初始化后再执行接口检查。

当前如果 `simulation.launch.py` 尚未合入，则完整启动模式暂时无法完成最终验收；此时可继续使用 `--check-only` 进行接口预检查。

## 6. 验收文档

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

## 7. 测试原则

1. 使用项目统一 Topic、Frame、Node 和 Launch 名称。
2. 不为测试另建与正式接口重复的 Topic 或 Frame。
3. 核心 TF 以 `map -> odom -> base_link -> laser_link` 为准。
4. 每个失败项都应可复现。
5. 修复后必须重新测试。
6. 正式验收以真实 Topic、TF、日志和实验数据为依据。
