from pathlib import Path


ROOT = Path(__file__).parents[2]


def test_both_clients_use_the_internal_action_endpoint():
    goal_bridge = (
        ROOT / 'landerpi_navigation' / 'scripts' /
        'goal_bridge.py').read_text()
    task_queue = (
        ROOT / 'landerpi_task_manager' / 'landerpi_task_manager' /
        'task_queue.py').read_text()

    assert "NavigateToPose, '/navigation_command'" in goal_bridge
    assert "NavigateToPose, '/navigation_command'" in task_queue
    assert "NavigateToPose, '/navigate_to_pose'" not in goal_bridge
    assert "NavigateToPose, '/navigate_to_pose'" not in task_queue


def test_only_command_arbiter_owns_the_nav2_action_endpoint():
    arbiter = (
        ROOT / 'landerpi_task_manager' / 'landerpi_task_manager' /
        'command_arbiter.py').read_text()

    assert "NavigateToPose, '/navigate_to_pose'" in arbiter
    assert "NavigateToPose, '/navigation_command'" in arbiter
    assert 'CancelGoal' in arbiter
    assert '/navigate_to_pose/_action/cancel_goal' in arbiter
