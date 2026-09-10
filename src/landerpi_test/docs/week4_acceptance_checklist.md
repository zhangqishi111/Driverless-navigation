# Week 4 Navigation Acceptance Checklist

## 1. 离线检查

- [ ] 已同步最新 `origin/dev`
- [ ] `feature/slam` 合并 `origin/dev` 无冲突
- [ ] `landerpi_base_driver` 构建成功
- [ ] `landerpi_navigation` 构建成功
- [ ] `landerpi_bringup` 构建成功
- [ ] Navigation 自动测试全部通过
- [ ] `git diff --check` 无异常
- [ ] `real_system.launch.py` 中 mapping / navigation 模式互斥
- [ ] Navigation 模式启动 Localization Safety Guard
- [ ] Navigation 速度链路经过 `/cmd_vel_safe`
- [ ] 正式地图及 YAML 配置正确

---

## 2. TF 与节点检查（实车）

启动正式导航后检查：

```bash
ros2 node list
