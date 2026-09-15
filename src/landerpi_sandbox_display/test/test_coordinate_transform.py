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


def test_zoom_keeps_the_cursor_world_coordinate_stable():
    transform = create_transform()

    anchor = transform.world_to_widget(
        x=0.5,
        y=-0.5,
        widget_width=800,
        widget_height=600,
    )

    assert anchor is not None
    assert transform.zoom_at(
        anchor[0],
        anchor[1],
        800,
        600,
        2.0,
    )

    world = transform.widget_to_world(
        anchor[0],
        anchor[1],
        800,
        600,
    )

    assert world is not None
    assert abs(world[0] - 0.5) < 1e-6
    assert abs(world[1] + 0.5) < 1e-6


def test_pan_preserves_world_to_widget_round_trip():
    transform = create_transform()

    assert transform.zoom_at(400, 300, 800, 600, 2.0)
    assert transform.pan_by(-120, -90, 800, 600)

    widget_position = transform.world_to_widget(
        1.0,
        -1.0,
        800,
        600,
    )
    assert widget_position is not None

    world = transform.widget_to_world(
        widget_position[0],
        widget_position[1],
        800,
        600,
    )
    assert world is not None
    assert abs(world[0] - 1.0) < 1e-6
    assert abs(world[1] + 1.0) < 1e-6


def test_reset_view_restores_automatic_fit():
    transform = create_transform()
    automatic_viewport = transform.widget_viewport(800, 600)

    assert transform.zoom_at(400, 300, 800, 600, 2.0)
    assert transform.pan_by(-100, -50, 800, 600)
    assert transform.reset_view()

    assert transform.zoom_multiplier == 1.0
    assert transform.pan_x == 0.0
    assert transform.pan_y == 0.0
    assert transform.widget_viewport(800, 600) == automatic_viewport
