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

def test_world_to_widget_uses_centered_map_viewport():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=50,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-1.25,
    )

    result = transform.world_to_widget(
        x=0.0,
        y=0.0,
        widget_width=800,
        widget_height=600,
    )

    assert result is not None

    widget_x, widget_y = result

    assert abs(widget_x - 400.0) < 1e-6
    assert abs(widget_y - 292.0) < 1e-6


def test_widget_to_world_round_trip():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=50,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-1.25,
    )

    original_x = 0.30
    original_y = -0.40

    widget_position = transform.world_to_widget(
        original_x,
        original_y,
        800,
        600,
    )

    assert widget_position is not None

    widget_x, widget_y = widget_position

    world_position = transform.widget_to_world(
        widget_x,
        widget_y,
        800,
        600,
    )

    assert world_position is not None

    world_x, world_y = world_position

    assert abs(world_x - original_x) < 1e-6
    assert abs(world_y - original_y) < 1e-6


def test_widget_to_world_rejects_click_outside_map():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=50,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-1.25,
    )

    # 800x600 中，100x50 地图等比例放大后是 800x400，
    # 上下各留 100 像素空白，因此 y=50 位于地图外。
    result = transform.widget_to_world(
        widget_x=400,
        widget_y=50,
        widget_width=800,
        widget_height=600,
    )

    assert result is None

def test_widget_viewport_wide_widget():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=50,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-1.25,
    )

    viewport = transform.widget_viewport(
        widget_width=800,
        widget_height=600,
    )

    assert viewport == (0.0, 100.0, 800.0, 400.0)


def test_widget_viewport_tall_map():
    transform = CoordinateTransform()

    transform.update_map_info(
        width=100,
        height=100,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-2.5,
    )

    viewport = transform.widget_viewport(
        widget_width=800,
        widget_height=600,
    )

    assert viewport == (100.0, 0.0, 600.0, 600.0)


def test_widget_viewport_invalid_map_returns_none():
    transform = CoordinateTransform()

    viewport = transform.widget_viewport(
        widget_width=800,
        widget_height=600,
    )

    assert viewport is None