from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]

REAL_BASE = (
    REPO_ROOT
    / "src"
    / "landerpi_base_driver"
    / "launch"
    / "real_base.launch.py"
)

REAL_SYSTEM = (
    REPO_ROOT
    / "src"
    / "landerpi_bringup"
    / "launch"
    / "real_system.launch.py"
)

NAVIGATION_REAL = (
    REPO_ROOT
    / "src"
    / "landerpi_navigation"
    / "launch"
    / "navigation_real.launch.py"
)

NAVIGATION_LAUNCH = (
    REPO_ROOT
    / "src"
    / "landerpi_navigation"
    / "launch"
    / "navigation.launch.py"
)

GZ_BRIDGE = (
    REPO_ROOT
    / "src"
    / "landerpi_base_driver"
    / "config"
    / "gz_bridge.yaml"
)


def test_real_base_input_topic_is_configurable():
    text = REAL_BASE.read_text()

    assert "LaunchConfiguration('input_topic')" in text
    assert "DeclareLaunchArgument(" in text
    assert "'input_topic'" in text


def test_real_navigation_routes_commands_through_safety_guard():
    text = REAL_SYSTEM.read_text()

    assert "localization_safety_guard.py" in text
    assert "('/cmd_vel_guard_in', '/cmd_vel')" in text
    assert "('/cmd_vel', '/cmd_vel_safe')" in text
    assert "base_input_topic = '/cmd_vel_safe'" in text
    assert "'input_topic': base_input_topic" in text
    assert "'position_variance_threshold': 0.30" in text


def test_sim_navigation_output_matches_gazebo_velocity_input():
    navigation_text = NAVIGATION_LAUNCH.read_text()
    bridge_text = GZ_BRIDGE.read_text()

    assert "dst='cmd_vel_nav_output'" not in navigation_text
    assert "default_value='/cmd_vel'" in navigation_text
    assert "ros_topic_name: /cmd_vel" in bridge_text


def test_sim_and_real_navigation_use_separate_localization_variance_limits():
    navigation_text = NAVIGATION_LAUNCH.read_text()
    real_text = NAVIGATION_REAL.read_text()

    assert "'task_max_position_variance', default_value='0.50'" in navigation_text
    assert "'max_position_variance': task_max_position_variance" in navigation_text
    assert "'task_max_position_variance': '0.25'" in real_text
