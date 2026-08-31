# LanderPi navigation

This package owns the Week 1 map-based navigation mode.  It loads B's
`sim_map.yaml`, starts AMCL and Nav2, and provides project-level integration
topics for the sandbox display.

## Prerequisites

Start the simulation/base layer first.  It must continuously provide:

- `/clock` when `use_sim_time:=true`
- `/scan` (`sensor_msgs/LaserScan`, frame `laser_link`)
- `/odom` plus `odom -> base_link`
- static `base_link -> laser_link`

Do not run `mapping.launch.py` together with this launch file: SLAM Toolbox
and AMCL must never publish `map -> odom` simultaneously.

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch landerpi_navigation navigation.launch.py
```

In RViz, set an initial pose before navigating.  A UI can publish a
`geometry_msgs/PoseStamped` on `/goal_pose` with `header.frame_id: map`.

## Integration topics

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| input | `/goal_pose` | `geometry_msgs/PoseStamped` | Sandbox target in `map` coordinates |
| output | `/robot_pose` | `geometry_msgs/PoseStamped` | AMCL pose for the display, never dead-reckoned |
| output | `/plan` | `nav_msgs/Path` | Global path |
| output | `/local_plan` | `nav_msgs/Path` | Local path |
| output | `/navigation_status` | `std_msgs/String` | `planning`, `navigating`, `arrived`, or a rejection/failure reason |
| output | `/cmd_vel` | `geometry_msgs/Twist` | Command consumed by A's fake driver |

## Acceptance checks

1. Both Nav2 lifecycle managers report `active`.
2. `/amcl_pose` and `/robot_pose` update after an initial pose is set.
3. A free-space `/goal_pose` creates `/plan`, then `/local_plan`, then `/cmd_vel`.
4. Only AMCL publishes `map -> odom` in this mode.
