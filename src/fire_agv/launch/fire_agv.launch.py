import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    start = LaunchConfiguration("start", default="false")
    display = LaunchConfiguration("display", default="false")
    color = LaunchConfiguration("color", default="lid_red")
    target_distance = LaunchConfiguration("target_distance", default="0.2")
    place_color = LaunchConfiguration("place_color", default="pot_blue")
    place_z_offset = LaunchConfiguration("place_z_offset", default="0.06")
    place_down_offset = LaunchConfiguration("place_down_offset", default="0.1")
    place_target_distance = LaunchConfiguration("place_target_distance", default="0.30")
    trajectory_input_dir = LaunchConfiguration("trajectory_input_dir", default="/home/ubuntu/snowman/fire_and_cube/trajectory_replay/trajectory_records")

    trajectory_replay_node = Node(
        package="trajectory_replay",
        executable="replay_trajectory",
        name="trajectory_replay",
        output="screen",
        parameters=[{
            "auto_start": False,
            "input_dir": trajectory_input_dir,
        }],
    )

    fire_agv_node = Node(
        package="fire_agv",
        executable="fire_agv",
        output="screen",
        parameters=[{
            "color": color,
            "start": start,
            "display": display,
            "target_distance": target_distance,
            "place_color": place_color,
            "place_z_offset": place_z_offset,
            "place_down_offset": place_down_offset,
            "place_target_distance": place_target_distance,
        }],
    )

    # 底层节点（相机/底盘/里程计/舵机等）由自启动服务 start_app_node.service
    # 统一拉起 bringup，这里只启动本应用自己的节点，避免重复启动双套节点冲突。
    actions = [
        DeclareLaunchArgument("start", default_value=start),
        DeclareLaunchArgument("display", default_value=display),
        DeclareLaunchArgument("color", default_value=color),
        DeclareLaunchArgument("target_distance", default_value=target_distance),
        DeclareLaunchArgument("place_color", default_value=place_color),
        DeclareLaunchArgument("place_z_offset", default_value=place_z_offset),
        DeclareLaunchArgument("place_down_offset", default_value=place_down_offset),
        DeclareLaunchArgument("place_target_distance", default_value=place_target_distance),
        DeclareLaunchArgument("trajectory_input_dir", default_value=trajectory_input_dir),
        DeclareLaunchArgument(
            "start_kinematics",
            default_value="true",
            description="是否同时启动 kinematics 节点（机械臂 IK/FK 服务）；已在别处启动时设为 false"),
    ]

    # kinematics 节点：提供 /kinematics/set_pose_target 与 /kinematics/get_current_pose
    # （IK/FK）。不在开机自启动链路内（bringup 不含它），由本 launch 一并启动，
    # 写法参考 pick_place.launch.py。用 IfCondition 延迟判断 start_kinematics
    # （声明在本函数返回的 actions 列表中，不能在函数内立即 perform）。
    try:
        kinematics_pkg_path = get_package_share_directory("kinematics")
    except Exception:
        # 源码模式（未 colcon install 时）
        kinematics_pkg_path = "/home/ubuntu/ros2_ws/src/driver/kinematics"
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinematics_pkg_path, "launch/kinematics_node.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_kinematics")),
    ))

    actions += [trajectory_replay_node, fire_agv_node]
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == "__main__":
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
