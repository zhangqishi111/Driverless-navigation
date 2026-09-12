from pathlib import Path

from rosidl_adapter.parser import parse_message_file


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def _parse(message_name):
    return parse_message_file(
        'landerpi_msgs',
        str(PACKAGE_DIR / 'msg' / f'{message_name}.msg'),
    )


def _fields(specification):
    fields = []
    for field in specification.fields:
        field_type = field.type.type
        if field.type.pkg_name:
            field_type = f'{field.type.pkg_name}/{field_type}'
        if field.type.is_array:
            field_type += '[]'
        fields.append((field_type, field.name))
    return fields


def _constants(specification):
    return {
        constant.name: constant.value
        for constant in specification.constants
    }


def test_navigation_point_state_exposes_the_complete_point_result_contract():
    specification = _parse('NavigationPointState')

    assert _constants(specification) == {
        'PENDING': 0,
        'PLANNING': 1,
        'NAVIGATING': 2,
        'WAITING_FOR_STOP': 3,
        'SUCCEEDED': 4,
        'FAILED': 5,
        'CANCELLED': 6,
        'NOT_EXECUTED': 7,
    }
    assert _fields(specification) == [
        ('uint8', 'index'),
        ('geometry_msgs/Pose', 'target'),
        ('uint8', 'state'),
        ('bool', 'has_arrival_error'),
        ('float32', 'arrival_error_m'),
        ('float32', 'elapsed_time_s'),
        ('uint8', 'retry_count'),
        ('string', 'detail'),
    ]


def test_navigation_task_state_exposes_one_atomic_snapshot_contract():
    specification = _parse('NavigationTaskState')

    assert _constants(specification) == {
        'IDLE': 0,
        'ACTIVE': 1,
        'SUCCEEDED': 2,
        'FAILED': 3,
        'CANCELLED': 4,
        'REJECTED': 5,
    }
    assert _fields(specification) == [
        ('std_msgs/Header', 'header'),
        ('uint32', 'task_id'),
        ('bool', 'active'),
        ('uint8', 'current_index'),
        ('uint8', 'total_points'),
        ('uint8', 'state'),
        ('float32', 'elapsed_time_s'),
        ('string', 'detail'),
        ('landerpi_msgs/NavigationPointState[]', 'points'),
    ]
