from landerpi_sandbox_display.coordinate_transform import (
    CoordinateTransform,
)


def create_transform():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=100,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-2.5,
    )

    return transform


def test_world_origin_to_pixel():
    transform = create_transform()

    pixel = transform.world_to_pixel(
        -2.5,
        -2.5,
    )

    assert pixel == (0, 99)


def test_world_zero_to_pixel():
    transform = create_transform()

    pixel = transform.world_to_pixel(
        0.0,
        0.0,
    )

    assert pixel == (50, 49)


def test_world_pixel_round_trip():
    transform = create_transform()

    original_x = 1.0
    original_y = -1.0

    px, py = transform.world_to_pixel(
        original_x,
        original_y,
    )

    x, y = transform.pixel_to_world(
        px,
        py,
    )

    assert abs(x - original_x) <= 0.05
    assert abs(y - original_y) <= 0.05


def test_click_outside_map_returns_none():
    transform = create_transform()

    result = transform.label_to_map_pixel(
        label_x=10,
        label_y=300,
        label_width=900,
        label_height=600,
    )

    assert result is None