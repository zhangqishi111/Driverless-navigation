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
