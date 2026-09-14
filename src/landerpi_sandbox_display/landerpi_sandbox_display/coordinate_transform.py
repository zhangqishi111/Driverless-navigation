class CoordinateTransform:
    def __init__(self):
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.zoom = 1.0
        self._view_center_x = None
        self._view_center_y = None

    def update_map_info(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    ):
        geometry_changed = (
            self.map_width != width
            or self.map_height != height
            or self.resolution != resolution
            or self.origin_x != origin_x
            or self.origin_y != origin_y
        )
        self.map_width = width
        self.map_height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        if geometry_changed:
            self.reset_view()

    def reset_view(self):
        self.zoom = 1.0
        self._view_center_x = self.map_width / 2.0
        self._view_center_y = self.map_height / 2.0

    def zoom_at(
            self,
            factor,
            widget_x,
            widget_y,
            widget_width,
            widget_height,
    ):
        viewport = self.widget_viewport(widget_width, widget_height)
        if viewport is None or factor <= 0:
            return self.zoom

        offset_x, offset_y, displayed_width, _displayed_height = viewport
        old_scale = displayed_width / self.map_width
        map_x = (widget_x - offset_x) / old_scale
        map_y = (widget_y - offset_y) / old_scale

        new_zoom = min(5.0, max(1.0, self.zoom * float(factor)))
        if new_zoom == self.zoom:
            return self.zoom

        self.zoom = new_zoom
        scale = self._base_scale(widget_width, widget_height) * self.zoom
        self._view_center_x = (
            widget_width / 2.0 - widget_x + map_x * scale
        ) / scale
        self._view_center_y = (
            widget_height / 2.0 - widget_y + map_y * scale
        ) / scale
        self._clamp_view_center(widget_width, widget_height)
        return self.zoom

    def pan_view(
            self,
            delta_x,
            delta_y,
            widget_width,
            widget_height,
    ):
        if self.zoom <= 1.0:
            return
        scale = self._base_scale(widget_width, widget_height) * self.zoom
        if scale <= 0:
            return
        self._ensure_view_center()
        self._view_center_x -= float(delta_x) / scale
        self._view_center_y -= float(delta_y) / scale
        self._clamp_view_center(widget_width, widget_height)

    def _base_scale(self, widget_width, widget_height):
        if self.map_width <= 0 or self.map_height <= 0:
            return 0.0
        return min(
            widget_width / self.map_width,
            widget_height / self.map_height,
        )

    def _ensure_view_center(self):
        if self._view_center_x is None:
            self._view_center_x = self.map_width / 2.0
        if self._view_center_y is None:
            self._view_center_y = self.map_height / 2.0

    @staticmethod
    def _clamp_axis(center, map_size, widget_size, scale):
        half_visible = widget_size / (2.0 * scale)
        first_bound = half_visible
        second_bound = map_size - half_visible
        lower = min(first_bound, second_bound)
        upper = max(first_bound, second_bound)
        return min(upper, max(lower, center))

    def _clamp_view_center(self, widget_width, widget_height):
        scale = self._base_scale(widget_width, widget_height) * self.zoom
        if scale <= 0:
            return
        self._ensure_view_center()
        self._view_center_x = self._clamp_axis(
            self._view_center_x, self.map_width, widget_width, scale)
        self._view_center_y = self._clamp_axis(
            self._view_center_y, self.map_height, widget_height, scale)

    def world_to_pixel(self, x, y):
        if self.resolution <= 0:
            return None

        px = (x - self.origin_x) / self.resolution
        py = (y - self.origin_y) / self.resolution

        # ROS地图+y向上，Qt图像+y向下
        py = self.map_height - 1 - py

        return int(round(px)), int(round(py))

    def pixel_to_world(self, px, py):
        if self.resolution <= 0:
            return None

        map_y = self.map_height - 1 - py

        x = px * self.resolution + self.origin_x
        y = map_y * self.resolution + self.origin_y

        return x, y

    def world_to_widget(
            self,
            x,
            y,
            widget_width,
            widget_height,
    ):
        if (
                self.map_width <= 0
                or self.map_height <= 0
                or self.resolution <= 0
                or widget_width <= 0
                or widget_height <= 0
        ):
            return None

        viewport = self.widget_viewport(
            widget_width,
            widget_height,
        )

        if viewport is None:
            return None

        offset_x, offset_y, displayed_width, displayed_height = viewport

        scale = displayed_width / self.map_width

        map_x = (x - self.origin_x) / self.resolution

        map_y = (y - self.origin_y) / self.resolution
        map_y = self.map_height - 1 - map_y

        widget_x = offset_x + map_x * scale
        widget_y = offset_y + map_y * scale

        return widget_x, widget_y

    def widget_to_world(
            self,
            widget_x,
            widget_y,
            widget_width,
            widget_height,
    ):
        if (
                self.map_width <= 0
                or self.map_height <= 0
                or self.resolution <= 0
                or widget_width <= 0
                or widget_height <= 0
        ):
            return None

        viewport = self.widget_viewport(
            widget_width,
            widget_height,
        )

        if viewport is None:
            return None

        offset_x, offset_y, displayed_width, displayed_height = viewport

        scale = displayed_width / self.map_width
        if (
                widget_x < offset_x
                or widget_x >= offset_x + displayed_width
                or widget_y < offset_y
                or widget_y >= offset_y + displayed_height
        ):
            return None

        map_x = (widget_x - offset_x) / scale
        map_y = (widget_y - offset_y) / scale

        map_y = self.map_height - 1 - map_y

        world_x = map_x * self.resolution + self.origin_x
        world_y = map_y * self.resolution + self.origin_y

        return world_x, world_y

    def label_to_map_pixel(
        self,
        label_x,
        label_y,
        label_width,
        label_height,
    ):
        if (
            self.map_width <= 0
            or self.map_height <= 0
            or label_width <= 0
            or label_height <= 0
        ):
            return None

        scale = min(
            label_width / self.map_width,
            label_height / self.map_height,
        )

        displayed_width = self.map_width * scale
        displayed_height = self.map_height * scale

        offset_x = (
            label_width - displayed_width
        ) / 2.0

        offset_y = (
            label_height - displayed_height
        ) / 2.0

        if not (
            offset_x <= label_x < offset_x + displayed_width
            and offset_y <= label_y < offset_y + displayed_height
        ):
            return None

        map_px = (label_x - offset_x) / scale
        map_py = (label_y - offset_y) / scale

        return map_px, map_py

    def widget_viewport(
            self,
            widget_width,
            widget_height,
    ):
        if (
                self.map_width <= 0
                or self.map_height <= 0
                or widget_width <= 0
                or widget_height <= 0
        ):
            return None

        scale = self._base_scale(widget_width, widget_height) * self.zoom

        displayed_width = self.map_width * scale
        displayed_height = self.map_height * scale

        self._ensure_view_center()

        offset_x = (
            widget_width / 2.0 - self._view_center_x * scale
        )

        offset_y = (
            widget_height / 2.0 - self._view_center_y * scale
        )

        return (
            offset_x,
            offset_y,
            displayed_width,
            displayed_height,
        )
