class CoordinateTransform:
    def __init__(self):
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.zoom_multiplier = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

        self.min_zoom_multiplier = 0.5
        self.max_zoom_multiplier = 6.0

    def update_map_info(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    ):
        map_changed = (
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

        if map_changed:
            self.reset_view()

    def reset_view(self):
        changed = (
            self.zoom_multiplier != 1.0
            or self.pan_x != 0.0
            or self.pan_y != 0.0
        )
        self.zoom_multiplier = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        return changed

    def pan_by(self, delta_x, delta_y, widget_width, widget_height):
        components = self._viewport_components(
            widget_width,
            widget_height,
        )
        if components is None:
            return False

        base_x, base_y, displayed_width, displayed_height = components
        old_x = self.pan_x
        old_y = self.pan_y
        offset_x = self._clamp_offset(
            base_x + self.pan_x + delta_x,
            displayed_width,
            widget_width,
        )
        offset_y = self._clamp_offset(
            base_y + self.pan_y + delta_y,
            displayed_height,
            widget_height,
        )
        self.pan_x = offset_x - base_x
        self.pan_y = offset_y - base_y
        return self.pan_x != old_x or self.pan_y != old_y

    def zoom_at(
            self,
            widget_x,
            widget_y,
            widget_width,
            widget_height,
            factor,
    ):
        if factor <= 0:
            return False

        old_zoom = self.zoom_multiplier
        target_zoom = max(
            self.min_zoom_multiplier,
            min(
                self.max_zoom_multiplier,
                old_zoom * factor,
            ),
        )
        if target_zoom == old_zoom:
            return False

        anchor = self.widget_to_world(
            widget_x,
            widget_y,
            widget_width,
            widget_height,
        )
        self.zoom_multiplier = target_zoom
        components = self._viewport_components(
            widget_width,
            widget_height,
        )
        if components is None:
            self.zoom_multiplier = old_zoom
            return False

        base_x, base_y, displayed_width, displayed_height = components
        if anchor is None:
            self.pan_x = 0.0
            self.pan_y = 0.0
            return True

        anchor_x, anchor_y = anchor
        scale = displayed_width / self.map_width
        map_x = (anchor_x - self.origin_x) / self.resolution
        map_y = self.map_height - 1 - (
            (anchor_y - self.origin_y) / self.resolution
        )
        offset_x = self._clamp_offset(
            widget_x - map_x * scale,
            displayed_width,
            widget_width,
        )
        offset_y = self._clamp_offset(
            widget_y - map_y * scale,
            displayed_height,
            widget_height,
        )
        self.pan_x = offset_x - base_x
        self.pan_y = offset_y - base_y
        return True

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

        components = self._viewport_components(
            widget_width,
            widget_height,
        )
        if components is None:
            return None

        base_x, base_y, displayed_width, displayed_height = components
        offset_x = self._clamp_offset(
            base_x + self.pan_x,
            displayed_width,
            widget_width,
        )
        offset_y = self._clamp_offset(
            base_y + self.pan_y,
            displayed_height,
            widget_height,
        )

        return (
            offset_x,
            offset_y,
            displayed_width,
            displayed_height,
        )

    def _viewport_components(self, widget_width, widget_height):
        if (
                self.map_width <= 0
                or self.map_height <= 0
                or widget_width <= 0
                or widget_height <= 0
        ):
            return None

        fit_scale = min(
            widget_width / self.map_width,
            widget_height / self.map_height,
        )
        scale = fit_scale * self.zoom_multiplier
        displayed_width = self.map_width * scale
        displayed_height = self.map_height * scale
        return (
            (widget_width - displayed_width) / 2.0,
            (widget_height - displayed_height) / 2.0,
            displayed_width,
            displayed_height,
        )

    @staticmethod
    def _clamp_offset(offset, displayed_length, widget_length):
        if displayed_length <= widget_length:
            return (widget_length - displayed_length) / 2.0
        return min(
            0.0,
            max(widget_length - displayed_length, offset),
        )
