# LanderPi SLAM 导航系统验收清单

## 1. 文档目的

本文件由 E（测试与任务管理）维护，用于记录 LanderPi SLAM 导航项目的软件集成测试、阶段验收、问题复现与复测结果。

本周目标：在纯软件仿真环境下，对团队“建图 → 定位 → 导航”核心链路进行验收，并形成可持续扩展的测试文档框架。

测试人员不重复实现 A/B/C/D 已负责的功能模块，而是检查各模块是否按照统一接口正确协同工作。

---

## 2. 统一接口

### 2.1 核心 Topic

| 功能 | Topic | 预期消息类型 | 主要负责人 |
|---|---|---|---|
| 激光雷达 | `/scan` | `sensor_msgs/msg/LaserScan` | A |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` | A |
| 底盘速度 | `/cmd_vel` | `geometry_msgs/msg/Twist` | A/C |
| 地图 | `/map` | `nav_msgs/msg/OccupancyGrid` | B |
| 统一实时位姿 | `/robot_pose` | `geometry_msgs/msg/PoseStamped` | A/B |
| 导航目标 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | D/C |
| 全局路径 | `/plan` | `nav_msgs/msg/Path` | C |
| 局部路径 | `/local_plan` | `nav_msgs/msg/Path` | C |
| 实际轨迹 | `/actual_path` | `nav_msgs/msg/Path` | D/C |

### 2.2 核心 TF

正式项目统一 TF：

```text
map
  -> odom
    -> base_link
      -> laser_link
```

检查原则：

- 建图模式：SLAM Toolbox 发布 `map -> odom`
- 导航模式：AMCL 发布 `map -> odom`
- 两者不得同时发布 `map -> odom`
- `odom -> base_link` 必须连续存在
- `base_link -> laser_link` 必须存在
- 测试中不得使用其他临时 Frame 名替代正式接口

---

# 3. 第 1 周：纯软件仿真验收

## 3.1 环境检查

| 编号 | 检查项目 | 验收条件 | 状态 | 备注 |
|---|---|---|---|---|
| W1-ENV-01 | 操作系统 | Ubuntu 22.04 | TODO | |
| W1-ENV-02 | ROS 2 | Humble | TODO | |
| W1-ENV-03 | 仿真平台 | Gazebo Fortress 可启动 | TODO | |
| W1-ENV-04 | 工作空间 | `colcon build` 成功 | TODO | |

## 3.2 基础接口检查

| 编号 | 检查项目 | 验收条件 | 状态 | 备注 |
|---|---|---|---|---|
| W1-INT-01 | `/scan` | Topic 存在且持续有数据 | TODO | A |
| W1-INT-02 | `/odom` | Topic 存在且持续有数据 | TODO | A |
| W1-INT-03 | `/cmd_vel` | Topic 存在 | TODO | A/C |
| W1-TF-01 | `odom -> base_link` | TF 连续存在 | TODO | A |
| W1-TF-02 | `base_link -> laser_link` | TF 存在 | TODO | A |

## 3.3 SLAM 建图检查

| 编号 | 检查项目 | 验收条件 | 状态 | 备注 |
|---|---|---|---|---|
| W1-SLAM-01 | SLAM Toolbox | 节点正常运行 | TODO | B |
| W1-SLAM-02 | `/map` | 地图持续更新 | TODO | B |
| W1-SLAM-03 | 建图运动 | 仿真车运动时地图正常扩展 | TODO | B |
| W1-SLAM-04 | 地图保存 | 成功生成仿真地图文件 | TODO | B |

## 3.4 定位与导航检查

| 编号 | 检查项目 | 验收条件 | 状态 | 备注 |
|---|---|---|---|---|
| W1-NAV-01 | 地图加载 | Map Server 正确加载地图 | TODO | B/C |
| W1-NAV-02 | AMCL | 能建立 `map -> odom` | TODO | B/C |
| W1-NAV-03 | 目标下发 | RViz 可发送导航目标 | TODO | C |
| W1-NAV-04 | 全局路径 | `/plan` 可获得路径 | TODO | C |
| W1-NAV-05 | 小车运动 | Nav2 能驱动车辆运动 | TODO | C |
| W1-NAV-06 | `/robot_pose` | 实时位姿持续更新 | TODO | A/B |
| W1-NAV-07 | `/actual_path` | 实际轨迹持续更新 | TODO | D/C |

## 3.5 E 本周交付检查

| 编号 | 检查项目 | 验收条件 | 状态 |
|---|---|---|---|
| W1-E-01 | 一键启动脚本 | `launch_sim_all.sh` 存在且语法检查通过 | TODO |
| W1-E-02 | 接口检查逻辑 | 能检查核心 Topic 与 TF | TODO |
| W1-E-03 | 验收文档 | 本文件已建立并可持续补充 | TODO |
| W1-E-04 | Git 分支 | 所有 E 内容提交至 `feature/test` | TODO |

---

# 4. 第 1 周验收结论

目标闭环：

```text
启动仿真
  ↓
/scan、/odom、/cmd_vel 正常
  ↓
SLAM 建图
  ↓
保存地图
  ↓
加载地图
  ↓
AMCL 定位
  ↓
发送目标
  ↓
Nav2 规划
  ↓
仿真车运动
```

验收结果：

- [ ] PASS
- [ ] PARTIAL PASS
- [ ] FAIL
- [ ] BLOCKED

主要问题：

- 待填写

---

# 5. 第 2 周：实车接口与数据回灌

E 主要任务：

- 使用 `ros2 bag record` 记录实车雷达和里程计数据
- 保存测试数据
- 使用 `ros2 bag play` 进行数据回灌验证
- 检查统一 Topic 和 TF 是否满足项目规范
- 记录异常、复现步骤和修复后的复测结果

详细测试项将在第 2 周补充。

---

# 6. 第 3 周：单目标导航重复实验

计划选择 5 个不同目标点，每个目标点运行 3 次，共 15 次实验。

记录字段：

| 目标编号 | 次数 | 目标X | 目标Y | 最终X | 最终Y | 导航时间 | 是否到达 | 备注 |
|---|---:|---:|---:|---:|---:|---:|---|---|
| P1 | 1 | | | | | | | |
| P1 | 2 | | | | | | | |
| P1 | 3 | | | | | | | |
| P2 | 1 | | | | | | | |
| P2 | 2 | | | | | | | |
| P2 | 3 | | | | | | | |
| P3 | 1 | | | | | | | |
| P3 | 2 | | | | | | | |
| P3 | 3 | | | | | | | |
| P4 | 1 | | | | | | | |
| P4 | 2 | | | | | | | |
| P4 | 3 | | | | | | | |
| P5 | 1 | | | | | | | |
| P5 | 2 | | | | | | | |
| P5 | 3 | | | | | | | |

后续统计：

- 导航成功率
- 平均导航时间
- 平均终点位置误差
- 最大终点位置误差
- 恢复次数
- 异常次数

---

# 7. 第 4 周：最终验收

计划内容：

- 按标准演示流程完整彩排 3 次
- 动态障碍测试
- 多目标导航测试
- `task_list.yaml` 三目标连续执行
- 系统异常与恢复记录
- 最终验收结果汇总

---

# 8. 问题记录

| ID | 日期 | 模块 | 问题描述 | 复现步骤 | 状态 | 负责人 |
|---|---|---|---|---|---|---|
| ISSUE-001 | | | | | Open | |

状态统一使用：

- `Open`
- `Fixing`
- `Retest`
- `Closed`

---

## BLOCKED 状态说明

`BLOCKED` 表示当前测试无法执行，原因是前置模块、统一 Launch 或相关接口尚未交付，并不等同于被测功能已经失败。

例如：`base_sim.launch.py` 尚未合入时，完整一键仿真验收应记录为 `BLOCKED`。

# 9. 测试原则

1. 所有测试优先使用项目统一 Topic、Frame、Node 和 Launch 名称。
2. 不为测试单独创建已有功能的另一套接口。
3. 每个失败项必须记录复现步骤。
4. 修复后必须重新测试。
5. 测试结果以真实 Topic、TF、日志和实验数据为依据。
6. 正式验收结果不得仅以 RViz 视觉现象作为唯一依据。
7. 测试代码与文档统一提交至 `feature/test` 分支。
