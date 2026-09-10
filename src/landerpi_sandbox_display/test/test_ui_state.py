from landerpi_sandbox_display.ui_state import DisplayState


def test_latest_robot_pose_wins():
    state = DisplayState()

    state.update_robot_pose('pose-1')
    state.update_robot_pose('pose-2')

    snapshot = state.snapshot()

    assert snapshot.values['robot_pose'] == 'pose-2'
    assert snapshot.revisions['robot_pose'] == 2


def test_latest_local_plan_wins():
    state = DisplayState()

    state.update_local_plan('plan-1')
    state.update_local_plan('plan-2')

    snapshot = state.snapshot()

    assert snapshot.values['local_plan'] == 'plan-2'
    assert snapshot.revisions['local_plan'] == 2


def test_snapshot_does_not_expose_internal_dicts():
    state = DisplayState()

    state.update_robot_pose('original')

    snapshot = state.snapshot()

    snapshot.values['robot_pose'] = 'modified'
    snapshot.revisions['robot_pose'] = 999

    new_snapshot = state.snapshot()

    assert new_snapshot.values['robot_pose'] == 'original'
    assert new_snapshot.revisions['robot_pose'] == 1

def test_display_state_tracks_all_display_data():
    state = DisplayState()

    state.update_map('map')
    state.update_amcl_pose('amcl')
    state.update_robot_pose('robot')
    state.update_global_plan('global')
    state.update_local_plan('local')
    state.update_actual_path('actual')

    snapshot = state.snapshot()

    assert snapshot.values['map'] == 'map'
    assert snapshot.values['amcl_pose'] == 'amcl'
    assert snapshot.values['robot_pose'] == 'robot'
    assert snapshot.values['global_plan'] == 'global'
    assert snapshot.values['local_plan'] == 'local'
    assert snapshot.values['actual_path'] == 'actual'

def test_display_state_tracks_position_error():
    state = DisplayState()

    state.update_position_error(0.123)

    snapshot = state.snapshot()

    assert snapshot.values['position_error'] == 0.123
    assert snapshot.revisions['position_error'] == 1