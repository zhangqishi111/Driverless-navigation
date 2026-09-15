from types import SimpleNamespace

from fire_agv.fire_agv_node import FireAgvNode


def make_response():
    return SimpleNamespace(
        success=False,
        message='',
    )


def test_place_success_reports_running_while_place_cube_is_active():
    fake_node = SimpleNamespace(
        last_place_success=False,
        command_mode='place_cube',
        moving=True,
    )

    response = make_response()

    result = FireAgvNode.place_success_srv_callback(
        fake_node,
        None,
        response,
    )

    assert result.success is False
    assert result.message == 'place_running'


def test_place_success_reports_true_after_place_cube_finishes():
    fake_node = SimpleNamespace(
        last_place_success=True,
        command_mode=None,
        moving=False,
    )

    response = make_response()

    result = FireAgvNode.place_success_srv_callback(
        fake_node,
        None,
        response,
    )

    assert result.success is True
    assert result.message == 'place_succeeded'


def test_place_success_reports_failure_when_not_running_or_finished():
    fake_node = SimpleNamespace(
        last_place_success=False,
        command_mode=None,
        moving=False,
    )

    response = make_response()

    result = FireAgvNode.place_success_srv_callback(
        fake_node,
        None,
        response,
    )

    assert result.success is False
    assert result.message == 'place_not_succeeded'
