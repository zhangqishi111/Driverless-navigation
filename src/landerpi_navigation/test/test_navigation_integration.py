"""Offline regression checks for the observed missing receiver/live obstacle failures."""

import ast
from pathlib import Path
from types import SimpleNamespace

import yaml


SRC = Path(__file__).resolve().parents[2]


def test_both_costmaps_mark_and_clear_live_scan_obstacles():
    config = yaml.safe_load(
        (SRC / 'landerpi_navigation/config/nav2_params.yaml').read_text(encoding='utf-8'))
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        assert params['plugins'].index('obstacle_layer') < params['plugins'].index('inflation_layer')
        layer = params['obstacle_layer']
        assert layer['enabled']
        scan = layer['scan']
        assert scan['topic'] == '/scan'
        assert scan['marking'] and scan['clearing']
        assert scan['data_type'] == 'LaserScan'
        assert scan['raytrace_max_range'] > scan['obstacle_max_range'] > 0


def test_missing_receiver_does_not_publish_or_consume_draft():
    # Execute the actual method without importing ROS/Qt on the review host.
    source = (SRC / 'landerpi_sandbox_display/landerpi_sandbox_display/sandbox_display_node.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SandboxDisplayNode')
    method = next(n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_navigation_task')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<actual publish method>', 'exec'), namespace)
    errors = []
    node = SimpleNamespace(
        navigation_task_publisher=SimpleNamespace(get_subscription_count=lambda: 0),
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    assert namespace['publish_navigation_task'](node, None) is False
    assert '/navigation_task/goals' in errors[0]


def test_launch_contains_complete_multi_goal_backend():
    launch = (SRC / 'landerpi_navigation/launch/navigation.launch.py').read_text(encoding='utf-8')
    setup = (SRC / 'landerpi_task_manager/setup.py').read_text(encoding='utf-8')
    queue = (SRC / 'landerpi_task_manager/landerpi_task_manager/task_queue.py').read_text(encoding='utf-8')
    for executable in ('task_queue', 'navigation_command_arbiter'):
        assert f"executable='{executable}'" in launch
        assert f'{executable} = ' in setup
    assert "NavigationTaskState, '/navigation_task/state'" in queue
    assert "PoseArray, '/navigation_task/goals'" in queue


def test_ui_reports_failed_submission_and_keeps_draft():
    source = (SRC / 'landerpi_sandbox_display/landerpi_sandbox_display/main_window.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    window_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')
    method = next(n for n in window_class.body if isinstance(n, ast.FunctionDef) and n.name == 'submit_navigation_task')
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<actual submit method>', 'exec'), namespace)
    labels, tips = [], []
    goals = [object(), object(), object()]
    window = SimpleNamespace(
        _task_active=False,
        task_draft=SimpleNamespace(goals=lambda: goals),
        task_publish_callback=lambda submitted: False,
        task_summary_value=SimpleNamespace(setText=labels.append, setToolTip=tips.append),
    )
    namespace['submit_navigation_task'](window)
    assert 'Not submitted' in labels[-1]
    assert 'navigation_task_queue' in tips[-1]
    assert len(window.task_draft.goals()) == 3
    window.task_publish_callback = lambda submitted: True
    namespace['submit_navigation_task'](window)
    assert 'waiting for task state' in labels[-1]
