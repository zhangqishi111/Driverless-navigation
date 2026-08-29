class CoordinateTransform:
    def __init__(self):
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

    def update_map_info(
        self,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    ):
        self.map_width = width
        self.map_height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

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