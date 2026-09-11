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

For a sequential task, publish a `geometry_msgs/PoseArray` containing one to
three poses on `/navigation_task/goals`; its header must also use `map`. The
queue sends only one `NavigateToPose` action at a time. It advances only after
Nav2 reports arrival and the controller output has remained at zero; failure,
cancellation, stale AMCL data, or excessive AMCL covariance clears the queue.

## Integration topics

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| input | `/goal_pose` | `geometry_msgs/PoseStamped` | Sandbox target in `map` coordinates |
| input | `/navigation_task/goals` | `geometry_msgs/PoseArray` | One to three sequential map-frame goals |
| input | `/navigation_task/cancel` | `std_msgs/Empty` | Cancel the active task and clear queued goals |
| output | `/robot_pose` | `geometry_msgs/PoseStamped` | AMCL pose for the display, never dead-reckoned |
| output | `/plan` | `nav_msgs/Path` | Global path |
| output | `/local_plan` | `nav_msgs/Path` | Local path |
| output | `/navigation_status` | `std_msgs/String` | `planning`, `navigating`, `arrived`, or a rejection/failure reason |
| output | `/arrival_status` | `std_msgs/Bool` | Latched, `true` only after Nav2 reports successful arrival; remains independent of diagnostic text |
| output | `/position_error` | `std_msgs/Float32` | Latched map-frame XY distance (metres) from `/robot_pose` to the latest valid target |
| output | `/cmd_vel` | `geometry_msgs/Twist` | Command consumed by A's fake driver |
| output | `/navigation_task/status` | `std_msgs/String` | Latched task id, state, current point, and reason |
| output | `/navigation_task/active` | `std_msgs/Bool` | Latched navigation ownership claim |
| output | `/navigation_task/current_index` | `std_msgs/UInt8` | Latched one-based point index; `0` when idle |

## Acceptance checks

1. Both Nav2 lifecycle managers report `active`.
2. `/amcl_pose` and `/robot_pose` update after an initial pose is set.
3. A free-space `/goal_pose` creates `/plan`, then `/local_plan`, then `/cmd_vel`.
4. Only AMCL publishes `map -> odom` in this mode.

`/local_plan` is intentionally in the `odom` frame because the local
costmap/controller operate in that frame. Consumers must transform it to
`map` for overlay instead of treating this normal frame choice as a warning.
