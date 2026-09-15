#!/usr/bin/env python3
# encoding: utf-8

import cv2
import time
import queue
import signal
import threading

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from interfaces.srv import SetString
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sdk import common, pid
from sensor_msgs.msg import CameraInfo, Image
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition
from std_srvs.srv import Trigger


DISTANCE_TOLERANCE = 0.03
CENTER_TOLERANCE_PX = 30
MAX_LINEAR_SPEED = 0.12
MAX_ANGULAR_SPEED = 0.6
KP_LINEAR = 0.6
KP_ANGULAR = 0.0035
KP_CAMERA_YAW = 0.003
CAMERA_YAW_CENTER = 500
TARGET_LOST_TIME = 1.0
SEARCH_STEP_INTERVAL = 1.5
ACTIVE_PROCESS_INTERVAL = 0.1
PICK_PITCH_NEAR = 75
PICK_PITCH_FAR = 75
PICK_PITCH_RANGE = [-180.0, 180.0]
PICK_IK_MAX_RETRIES = 3
PICK_IK_BACKOFF_SPEED = 0.03
PICK_IK_BACKOFF_TIME = 0.35
PLACE_STOP_DIST = 0.22
PLACE_CACHE_TTL = 5.0
PLACE_SETTLE_TIME = 0.5
PLACE_CREEP_SPEED = 0.10
PLACE_IK_RETRY_MAX = 1
PLACE_IK_RETRY_STEP = 0.03

WAITING_POSE = ((1, 500), (2, 680), (3, 160), (4, 150), (5, 500), (10, 150))
HOLDING_POSE = ((1, 500), (2, 680), (3, 160), (4, 150), (5, 500), (10, 800))
CUBE_PLACE_READY_POSE = ((1, 875), (2, 635), (3, 120), (4, 200), (5, 500))
CUBE_PLACE_DOWN_POSE = ((1, 875), (2, 325), (3, 267), (4, 290), (5, 500))
SEARCH_POSES = (
    (500, 150),
    (700, 150),
    (500, 100),
    (250, 150),
    (700, 100),
    (250, 100),
)
# lid_red 反光兜底：灯光过亮导致盖子反光、A 通道跌破 lab_config 下限时，按阶梯逐级降低 A 阈值重检。
# 实测背景混入分界在 A≈130，故降到 140 即停；B 限 170、L 限 250 排除黄橙与刺眼白斑，避免覆盖全图导致定位错误。
LID_RED_FALLBACK_A_MIN = (170, 160, 150, 140)


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def find_color_blob(source_image, color_ranges, target_color, min_area, max_area):
    h, w = source_image.shape[:2]
    color = color_ranges["lab"]["Stereo"][target_color]
    image = cv2.resize(source_image, (int(w / 2), int(h / 2)))
    image = cv2.GaussianBlur(image, (3, 3), 3)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    base_ranges = list(color.get("ranges") or [color])
    range_sets = [base_ranges]
    if target_color == "lid_red":
        # 反光兜底：阶梯式降 A 阈值逐级检测，命中即返回，中心仍指向盖子主体
        for a_min in LID_RED_FALLBACK_A_MIN:
            range_sets.append([{"min": [0, a_min, 0], "max": [250, 255, 170]}])

    for ranges in range_sets:
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        for item in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(image, tuple(item["min"]), tuple(item["max"])))
        # Blue cube is relatively small in the Aurora image.
        # Do not erode it away before contour extraction.
        if target_color != "blue":
            mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        best = None
        best_area = 0
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if min_area < area < max_area and area > best_area:
                best = contour
                best_area = area
        if best is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(best)
            return (center_x * 2, center_y * 2), radius * 2

    return None, 0


class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 150

    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        center, radius = find_color_blob(source_image, color_ranges, self.target_color, 5, 60000)
        print("[BLUE_DEBUG] color=", self.target_color, "center=", center, "radius=", radius, flush=True)
        if center is None:
            return result_image, None, None, 0

        center_x, center_y = center
        if result_image is not None:
            circle_color = common.range_rgb.get(self.target_color, (0x55, 0x55, 0x55))
            cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)
        self.update_camera_pose(center_x, center_y, w, h)
        return result_image, (self.pitch, self.yaw), (center_x, center_y), radius

    def update_camera_pose(self, center_x, center_y, image_width, image_height):
        center_x_ratio = center_x / image_width
        if abs(center_x_ratio - 0.5) > 0.02:
            self.pid_yaw.SetPoint = 0.5
            self.pid_yaw.update(center_x_ratio)
            self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
        else:
            self.pid_yaw.clear()

        center_y_ratio = center_y / image_height
        if abs(center_y_ratio - 0.5) > 0.02:
            self.pid_pitch.SetPoint = 0.5
            self.pid_pitch.update(center_y_ratio)
            self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
        else:
            self.pid_pitch.clear()


class ColorLocator:
    def __init__(self, target_color):
        self.target_color = target_color

    def proc(self, source_image, result_image, color_ranges):
        center, radius = find_color_blob(source_image, color_ranges, self.target_color, 30, 60000)
        if center is None:
            return result_image, None, 0

        center_x, center_y = center
        if result_image is not None:
            circle_color = common.range_rgb.get(self.target_color, (0x55, 0x55, 0x55))
            cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)
            cv2.circle(result_image, (int(center_x), int(center_y)), 5, circle_color, -1)
        return result_image, (center_x, center_y), radius


class FireAgvNode(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.037],
        [0.0, 0.0, 0.0, 1.0],
    ]

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.running = True
        self.moving = False
        self.start = False
        self.waiting_place = False
        self.command_mode = None
        self.cube_running = False
        self.fire_running = False
        self.pick_ik_retry_count = 0
        self.last_pick_success = False
        self.last_place_success = False
        self.last_process_time = 0.0

        self.last_pitch_yaw = (0, 0)
        self.stamp = time.time()
        self.start_stamp = time.time() + 3
        self.endpoint = None

        self.place_yaw = 30
        self.place_pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.place_servo_yaw = 500

        self.last_target_seen_time = time.time()
        self.searching = False
        self.search_index = 0
        self.last_search_move_time = 0.0

        signal.signal(signal.SIGINT, self.shutdown)
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

        self.joints_pub = self.create_publisher(ServosPosition, "/servo_controller", 1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_odom_pub = self.create_publisher(Pose2D, "/set_odom", 1)

        self.target_color = self.get_param_value("color", "lid_red")
        self.auto_start = parse_bool(self.get_param_value("start", False))
        self.enable_disp = parse_bool(self.get_param_value("display", False))
        self.target_distance = float(self.get_param_value("target_distance", 0.35))
        self.place_color = self.get_param_value("place_color", "green")
        self.place_z_offset = float(self.get_param_value("place_z_offset", 0.06))
        self.place_down_offset = float(self.get_param_value("place_down_offset", 0.03))
        self.last_valid_place_position = None
        self.last_valid_place_dist = None
        self.last_valid_place_time = 0.0
        self.place_commit_stamp = None
        self.last_place_detect_log = 0.0
        self.tracker = None
        self.place_locator = ColorLocator(self.place_color)

        self.get_current_pose_client = self.create_client(GetRobotPose, "/kinematics/get_current_pose")
        self.get_current_pose_client.wait_for_service()
        self.set_pose_target_client = self.create_client(SetRobotPose, "/kinematics/set_pose_target")
        self.set_pose_target_client.wait_for_service()
        self.replay_client = self.create_client(SetString, "/trajectory_replay/replay")
        self.client = self.create_client(Trigger, "/controller_manager/init_finish")
        self.client.wait_for_service()

        self.create_service(Trigger, "~/start", self.start_srv_callback)
        self.create_service(Trigger, "~/stop", self.stop_srv_callback)
        self.create_service(SetString, "~/set_color", self.set_color_srv_callback)
        self.create_service(Trigger, "~/grasp", self.grasp_srv_callback)
        self.create_service(SetString, "~/place", self.place_srv_callback)
        self.create_service(Trigger, "~/grasp_cube", self.grasp_cube_srv_callback)
        self.create_service(Trigger, "~/place_cube", self.place_cube_srv_callback)
        self.create_service(Trigger, "~/pick_success", self.pick_success_srv_callback)
        self.create_service(Trigger, "~/place_success", self.place_success_srv_callback)
        self.create_service(Trigger, "~/cube", self.cube_srv_callback)
        self.create_service(Trigger, "~/fire", self.fire_srv_callback)

        self.image_queue = queue.Queue(maxsize=2)
        rgb_sub = message_filters.Subscriber(self, Image, "/ascamera/camera_publisher/rgb0/image")
        depth_sub = message_filters.Subscriber(self, Image, "/ascamera/camera_publisher/depth0/image_raw")
        info_sub = message_filters.Subscriber(self, CameraInfo, "/ascamera/camera_publisher/depth0/camera_info")
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        self.sync.registerCallback(self.multi_callback)

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_param_value(self, name, default_value):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default_value).value

    def init_process(self):
        self.timer.cancel()
        set_servo_position(self.joints_pub, 1, WAITING_POSE)
        time.sleep(1)

        if self.auto_start:
            self.set_target_color(self.target_color)
            self.start = True

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, "~/init_finish", self.get_node_state)
        self.get_logger().info("\033[1;32m%s\033[0m" % "start")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.stop_base()
        self.get_logger().info("\033[1;32m%s\033[0m" % "shutdown")
        cv2.destroyAllWindows()
        rclpy.shutdown()
        import sys
        sys.exit(0)

    def set_color_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "set_color")
        self.set_target_color(request.data)
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "start")
        if self.tracker is None:
            self.set_target_color(self.target_color)
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info("\033[1;32m%s\033[0m" % "stop")
        self.start = False
        self.moving = False
        self.waiting_place = False
        self.command_mode = None
        self.cube_running = False
        self.fire_running = False
        self.last_pick_success = False
        self.pick_ik_retry_count = 0
        self.stop_base()
        self.reset_search()
        self.reset_place_tracking()
        self.last_pitch_yaw = (0, 0)
        set_servo_position(self.joints_pub, 1, WAITING_POSE)
        response.success = True
        response.message = "stop"
        return response

    def grasp_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        self.start_grasp_action()
        response.success = True
        response.message = "grasp started"
        self.get_logger().info("\033[1;32m%s\033[0m" % "grasp started")
        return response

    def grasp_cube_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        self.start_grasp_cube_action()
        response.success = True
        response.message = "grasp cube started"
        self.get_logger().info("\033[1;32m%s\033[0m" % "grasp cube started")
        return response

    def place_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        if request.data == "":
            response.success = False
            response.message = "place color is empty"
            return response

        self.place_color = request.data
        self.place_locator = ColorLocator(self.place_color)
        self.start = False
        self.command_mode = "place"
        self.reset_search()
        self.reset_place_tracking()
        self.waiting_place = True
        response.success = True
        response.message = "place started"
        self.get_logger().info("\033[1;32mplace color: %s\033[0m" % self.place_color)
        return response

    def pick_success_srv_callback(self, request, response):
        response.success = bool(self.last_pick_success)
        if self.last_pick_success:
            response.message = "pick_succeeded"
        elif self.command_mode == "grasp_cube" or self.moving or self.start:
            response.message = "pick_running"
        else:
            response.message = "pick_not_succeeded"
        return response

    def place_success_srv_callback(self, request, response):
        success = bool(getattr(self, "last_place_success", False))

        response.success = success

        if success:
            response.message = "place_succeeded"
        elif self.command_mode == "place_cube" or self.moving:
            response.message = "place_running"
        else:
            response.message = "place_not_succeeded"

        return response

    def place_cube_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        self.last_place_success = False
        self.command_mode = "place_cube"
        self.start = False
        self.waiting_place = False
        self.moving = True
        threading.Thread(target=self.place_cube, daemon=True).start()
        response.success = True
        response.message = "place cube started"
        self.get_logger().info("\033[1;32m%s\033[0m" % "place cube started")
        return response

    def cube_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        self.cube_running = True
        self.get_logger().info("cube sequence accepted")
        threading.Thread(target=self.run_cube_sequence, daemon=True).start()
        response.success = True
        response.message = "cube started"
        return response

    def fire_srv_callback(self, request, response):
        if self.is_busy():
            response.success = False
            response.message = "busy"
            return response

        self.fire_running = True
        self.get_logger().info("fire sequence accepted")
        threading.Thread(target=self.run_fire_sequence, daemon=True).start()
        response.success = True
        response.message = "fire started"
        return response

    def start_grasp_action(self):
        self.set_target_color(self.target_color)
        self.waiting_place = False
        self.command_mode = "grasp"
        self.last_pick_success = False
        self.pick_ik_retry_count = 0
        self.reset_search()
        self.last_pitch_yaw = (0, 0)
        self.stamp = time.time()
        self.start_stamp = time.time()
        self.start = True

    def start_grasp_cube_action(self):
        self.tracker = ColorTracker("blue")
        self.waiting_place = False
        self.command_mode = "grasp_cube"
        self.last_pick_success = False
        self.pick_ik_retry_count = 0
        self.reset_search()
        self.last_pitch_yaw = (0, 0)
        self.stamp = time.time()
        self.start_stamp = time.time()
        self.start = True

    def set_target_color(self, color):
        self.target_color = color
        self.tracker = ColorTracker(self.target_color)
        self.get_logger().info("\033[1;32mset color: %s\033[0m" % self.target_color)

    def is_busy(self):
        return self.fire_running or self.cube_running or self.moving or self.command_mode is not None or self.waiting_place or self.start

    def is_active(self):
        return self.start or self.waiting_place or self.moving or self.command_mode is not None

    def stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def reset_search(self):
        self.last_target_seen_time = time.time()
        self.searching = False
        self.search_index = 0
        self.last_search_move_time = 0.0

    def reset_place_tracking(self):
        self.place_servo_yaw = 500
        self.place_pid_yaw.clear()
        self.last_valid_place_position = None
        self.last_valid_place_dist = None
        self.last_valid_place_time = 0.0
        self.place_commit_stamp = None

    def update_search(self, label):
        self.stop_base()
        now = time.time()
        if now - self.last_target_seen_time < TARGET_LOST_TIME:
            return
        if not self.searching:
            self.searching = True
            self.search_index = 0
            self.last_search_move_time = 0.0
            self.get_logger().info("%s target lost, start searching" % label)
        if now - self.last_search_move_time < SEARCH_STEP_INTERVAL:
            return

        yaw, pitch = SEARCH_POSES[self.search_index]
        set_servo_position(self.joints_pub, 0.5, ((1, int(yaw)), (4, int(pitch))))
        self.place_servo_yaw = yaw
        self.place_pid_yaw.clear()
        if self.tracker is not None:
            self.tracker.yaw = yaw
            self.tracker.pitch = pitch
            self.tracker.pid_yaw.clear()
            self.tracker.pid_pitch.clear()
        self.last_search_move_time = now
        self.search_index = (self.search_index + 1) % len(SEARCH_POSES)

    def approach_by_vision(self, center_x, image_width, dist, target_distance=None, camera_yaw=None):
        if target_distance is None:
            target_distance = self.target_distance

        distance_error = dist - target_distance
        x_error = center_x - image_width / 2.0
        camera_yaw_error = 0
        if camera_yaw is not None:
            camera_yaw_error = camera_yaw - CAMERA_YAW_CENTER
        elif self.tracker is not None:
            camera_yaw_error = self.tracker.yaw - CAMERA_YAW_CENTER

        if distance_error <= DISTANCE_TOLERANCE and abs(x_error) <= CENTER_TOLERANCE_PX and abs(camera_yaw_error) < 20:
            self.stop_base()
            return False

        twist = Twist()
        if distance_error > DISTANCE_TOLERANCE:
            twist.linear.x = common.set_range(KP_LINEAR * distance_error, 0.0, MAX_LINEAR_SPEED)

        angular = 0.0
        if abs(x_error) > CENTER_TOLERANCE_PX:
            angular += -KP_ANGULAR * x_error
        if abs(camera_yaw_error) >= 20:
            angular += KP_CAMERA_YAW * camera_yaw_error
        twist.angular.z = common.set_range(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        self.cmd_vel_pub.publish(twist)
        return True

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                return future.result()
            time.sleep(0.01)
        return None

    def replay_trajectory(self, route_name):
        if not self.replay_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/trajectory_replay/replay unavailable")
            return False

        self.reset_odom()
        req = SetString.Request()
        req.data = route_name
        self.get_logger().info("start replay trajectory: %s" % route_name)
        res = self.send_request(self.replay_client, req)
        if res is None or not res.success:
            message = "" if res is None else res.message
            self.get_logger().error("replay %s failed: %s" % (route_name, message))
            return False
        self.get_logger().info("replay %s finished" % route_name)
        return True

    def reset_odom(self):
        msg = Pose2D()
        msg.x = 0.0
        msg.y = 0.0
        msg.theta = 0.0
        self.set_odom_pub.publish(msg)
        time.sleep(1)
        self.set_odom_pub.publish(msg)
        time.sleep(1)

    def wait_for_grasp_cube_done(self):
        while rclpy.ok() and (self.command_mode == "grasp_cube" or self.start or self.moving):
            time.sleep(0.1)
        return self.last_pick_success

    def wait_for_grasp_done(self):
        while rclpy.ok() and (self.command_mode == "grasp" or self.start or self.waiting_place or self.moving):
            time.sleep(0.1)
        return self.last_pick_success

    def run_cube_sequence(self):
        try:
            if not self.replay_trajectory("cube1"):
                return

            self.start_grasp_cube_action()
            if not self.wait_for_grasp_cube_done():
                self.get_logger().error("cube sequence stopped: grasp_cube failed")
                return

            if not self.replay_trajectory("cube2"):
                return

            self.command_mode = "place_cube"
            self.moving = True
            self.place_cube()
            self.get_logger().info("cube sequence finished")
        finally:
            self.cube_running = False

    def run_fire_sequence(self):
        try:
            if not self.replay_trajectory("fire1"):
                return

            self.start_grasp_action()
            if not self.wait_for_grasp_done():
                self.get_logger().error("fire sequence stopped: grasp failed")
                return

            if not self.replay_trajectory("fire2"):
                return

            self.get_logger().info("fire sequence finished")
        finally:
            self.fire_running = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if not self.is_active():
            return
        now = time.time()
        if now - self.last_process_time < ACTIVE_PROCESS_INTERVAL:
            return
        self.last_process_time = now

        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request()).pose
        self.endpoint = common.xyz_quat_to_mat(
            [endpoint.position.x, endpoint.position.y, endpoint.position.z],
            [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z],
        )
        return self.endpoint

    def get_roi_distance(self, depth_image, center):
        h, w = depth_image.shape[:2]
        center_x, center_y = center
        roi = [
            int(center_y) - 5,
            int(center_y) + 5,
            int(center_x) - 5,
            int(center_x) + 5,
        ]
        roi[0] = max(roi[0], 0)
        roi[1] = min(roi[1], h)
        roi[2] = max(roi[2], 0)
        roi[3] = min(roi[3], w)

        roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]]
        valid = roi_distance[np.logical_and(roi_distance > 0, roi_distance < 10000)]
        if valid.size == 0:
            return None

        dist = round(float(np.mean(valid) / 1000.0), 3)
        if np.isnan(dist):
            return None
        return dist

    def camera_to_world(self, center, dist, camera_info, camera_offset, world_offset):
        K = camera_info.k
        self.get_endpoint()
        position = depth_pixel_to_camera(center, dist, (K[0], K[4], K[2], K[5]))
        position += np.array(camera_offset)

        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, _pose_R = common.mat_to_xyz_euler(world_pose)
        return np.array(pose_t) + np.array(world_offset)

    def clamp_center(self, center, image_shape):
        h, w = image_shape[:2]
        center_x, center_y = center
        center_x = int(min(max(center_x, 0), w - 1))
        center_y = int(min(max(center_y, 0), h - 1))
        return center_x, center_y

    def put_depth_text(self, depth_color_map, text):
        cv2.putText(depth_color_map, text, (10, 380), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
        cv2.putText(depth_color_map, text, (10, 380), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)

    def handle_pick_frame(self, rgb_image, result_image, depth_image, depth_camera_info, depth_color_map=None):
        result_image, pitch_yaw, center, _radius = self.tracker.proc(rgb_image, result_image, self.lab_data)
        if pitch_yaw is None:
            self.stop_base()
            self.update_search("pick")
            self.stamp = time.time()
            return result_image

        self.reset_search()
        set_servo_position(self.joints_pub, 0.02, ((1, int(pitch_yaw[1])), (4, int(pitch_yaw[0]))))
        center_x, center_y = self.clamp_center(center, depth_image.shape)
        txt = self.update_pick_target(center_x, center_y, pitch_yaw, depth_image, depth_camera_info)

        if result_image is not None:
            cv2.circle(result_image, (center_x, center_y), 5, (255, 255, 255), -1)
        if depth_color_map is not None:
            cv2.circle(depth_color_map, (center_x, center_y), 5, (255, 255, 255), -1)
            self.put_depth_text(depth_color_map, txt)
        self.last_pitch_yaw = pitch_yaw
        return result_image

    def update_pick_target(self, center_x, center_y, pitch_yaw, depth_image, depth_camera_info):
        raw_dist = int(depth_image[center_y, center_x])
        if raw_dist <= 0:
            self.stop_base()
            self.stamp = time.time()
            return "DISTANCE ERROR !!!"
        if raw_dist < 100:
            self.stop_base()
            self.stamp = time.time()
            return "TOO CLOSE !!!"

        current_dist = self.get_roi_distance(depth_image, (center_x, center_y))
        if current_dist is None:
            self.stop_base()
            self.stamp = time.time()
            return "DISTANCE ERROR !!!"

        if self.command_mode != "grasp_cube":
            dis_offset = 0
        else:
            dis_offset = 0.01

        target_dist = self.target_distance - dis_offset

        print(
            "[PICK_DEBUG] current=%.3f target=%.3f tol=%.3f pitch_yaw=%s stable=%s elapsed=%.2f"
            % (
                current_dist,
                target_dist,
                DISTANCE_TOLERANCE,
                str(pitch_yaw),
                str(self.is_pick_target_stable(pitch_yaw)),
                time.time() - self.stamp
            ),
            flush=True
        )

        if current_dist < target_dist - DISTANCE_TOLERANCE:
            twist = Twist()
            twist.linear.x = -0.04

            end_time = time.time() + 0.4
            while time.time() < end_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)

            self.stop_base()
            self.stamp = time.time()
            return "TOO CLOSE, BACKING"

        if current_dist > target_dist + DISTANCE_TOLERANCE:
            self.approach_by_vision(
                center_x,
                depth_image.shape[1],
                current_dist,
                target_dist
            )
            self.stamp = time.time()
            return "Dist: {}mm".format(raw_dist)

        self.stop_base()
        if not self.is_pick_target_stable(pitch_yaw):
            self.stamp = time.time()
            return "Dist: {}mm".format(raw_dist)
        if time.time() - self.stamp <= 2:
            return "Dist: {}mm".format(raw_dist)

        dist = self.get_roi_distance(depth_image, (center_x, center_y))
        if dist is None:
            self.stop_base()
            self.stamp = time.time()
            return "DISTANCE ERROR !!!"
        if dist < 0.10:
            self.get_logger().info("TOO CLOSE !!!")
            self.stop_base()
            self.stamp = time.time()
            return "TOO CLOSE !!!"
        if dist > self.target_distance - dis_offset + DISTANCE_TOLERANCE:
            if self.approach_by_vision(center_x, depth_image.shape[1], dist, self.target_distance-dis_offset):
                self.stamp = time.time()
                return "Dist: {}mm".format(raw_dist)

        self.stop_base()
        compensated_dist = dist + 0.015 + 0.015
        # These offsets are calibrated for the current camera/gripper geometry.
        if self.command_mode != "grasp_cube":
            pos_offset = (0.02, 0.0, 0.024)
        else:
            pos_offset = (0,0,-0.02)
        pose_t = self.camera_to_world(
            (center_x, center_y),
            compensated_dist,
            depth_camera_info,
            (-0.01, -0.03, 0.005),
            pos_offset, # plus[0]=front  plus[1]=left
        )
        self.get_logger().info(
            "pick target center=(%d, %d), depth=%.3fm, position=(%.3f, %.3f, %.3f)"
            % (center_x, center_y, compensated_dist, pose_t[0], pose_t[1], pose_t[2])
        )
        self.stamp = time.time()
        self.start_pick_thread(pose_t)
        return "Dist: {}mm".format(raw_dist)

    def is_pick_target_stable(self, pitch_yaw):
        return abs(self.last_pitch_yaw[0] - pitch_yaw[0]) < 5 and abs(self.last_pitch_yaw[1] - pitch_yaw[1]) < 5

    def find_place_position(self, rgb_image, result_image, depth_image, depth_camera_info):
        result_image, place_center, _place_radius = self.place_locator.proc(rgb_image, result_image, self.lab_data)

        if place_center is not None:
            self.reset_search()
            center_x, center_y = self.clamp_center(place_center, depth_image.shape)
            center = (center_x, center_y)
            # 仅让相机朝向跟随桶（不参与放置判定，避免 servo 微调导致放置前置条件无法满足）
            self.center_place_target(center_x, depth_image.shape[1])

            dist = self.get_roi_distance(depth_image, center)
            if dist is None:
                return result_image, None

            # 持续记录最后有效位置：停车到位或遮挡时直接用此位置放置，
            # 不依赖"中心稳定 2s"（盖子靠近桶时中心必然抖动，稳定窗口永远凑不齐）
            recorded_dist = dist + 0.015 + 0.015
            self.last_valid_place_position = self.camera_to_world(
                center,
                recorded_dist,
                depth_camera_info,
                (-0.01, -0.008, 0.01),
                (-0.01, 0.012, 0.08),
            )
            self.last_valid_place_dist = dist
            self.last_valid_place_time = time.time()
            if time.time() - self.last_place_detect_log >= 0.5:
                self.last_place_detect_log = time.time()
                self.get_logger().info(
                    "%s place target center=(%d, %d), depth=%.3fm, position=(%.3f, %.3f, %.3f)"
                    % (self.place_color, center_x, center_y, dist,
                       self.last_valid_place_position[0],
                       self.last_valid_place_position[1],
                       self.last_valid_place_position[2])
                )

            # 未到停车距离：继续视觉靠近
            if dist > PLACE_STOP_DIST - 0.01 + DISTANCE_TOLERANCE:
                self.place_commit_stamp = None
                self.approach_by_vision(
                    center_x, depth_image.shape[1], dist, PLACE_STOP_DIST - 0.01, self.place_servo_yaw
                )
                return result_image, None

        # —— 已到停车距离，或完全看不到桶：用最后有效位置直接放置 ——
        if self.last_valid_place_position is None:
            return result_image, None
        if time.time() - self.last_valid_place_time > PLACE_CACHE_TTL:
            return result_image, None
        if self.last_valid_place_dist < 0.10:
            return result_image, None

        # 停车后短暂稳定，避免底盘惯性导致缓存位置失真
        if self.place_commit_stamp is None:
            self.place_commit_stamp = time.time()
            self.stop_base()
            return result_image, None
        if time.time() - self.place_commit_stamp < PLACE_SETTLE_TIME:
            return result_image, None
        self.place_commit_stamp = None

        self.get_logger().info(
            "\033[1;33m%s target reached/occluded, place with last valid position\033[0m" % self.place_color
        )
        return result_image, np.array(self.last_valid_place_position, dtype=float)

    def creep_forward(self, dist):
        self.stop_base()
        time.sleep(0.1)
        twist = Twist()
        twist.linear.x = PLACE_CREEP_SPEED
        # 持续发布 cmd_vel 直到移动结束，确保底盘控制器持续收到速度指令
        end = time.time() + dist / PLACE_CREEP_SPEED
        self.get_logger().info("creep forward %.3fm @%.2fm/s" % (dist, PLACE_CREEP_SPEED))
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_base()
        time.sleep(0.2)

    def center_place_target(self, center_x, image_width):
        # 仅让相机朝向跟随桶中心，不参与放置触发判定
        center_x_ratio = center_x / image_width
        if abs(center_x_ratio - 0.5) > 0.02:
            self.place_pid_yaw.SetPoint = 0.5
            self.place_pid_yaw.update(center_x_ratio)
            self.place_servo_yaw = min(max(self.place_servo_yaw + self.place_pid_yaw.output, 0), 1000)
            set_servo_position(self.joints_pub, 0.02, ((1, int(self.place_servo_yaw)),))
        else:
            self.place_pid_yaw.clear()

    def place_on_target(self, place_position, yaw):
        above_position = np.array(place_position, dtype=float)
        above_position[2] += self.place_z_offset
        msg = set_pose_target(above_position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if not res.pulse:
            self.get_logger().info("place target no ik solution")
            return False

        servo_data = res.pulse
        set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
        time.sleep(1)

        release_position = np.array(above_position, dtype=float)
        release_position[2] -= self.place_down_offset
        msg = set_pose_target(release_position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if not res.pulse:
            self.get_logger().info("release target no ik solution")
            return False

        servo_data = res.pulse
        set_servo_position(self.joints_pub, 1, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1, ((10, 150),))
        time.sleep(1)

        msg = set_pose_target(above_position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(1)
        return True

    def place_and_finish(self, place_position, yaw):
        self.stop_base()
        position = np.array(place_position, dtype=float)

        # IK 预检：提前停车后目标可能超出机械臂工作空间（太远），
        # 无解时让底盘前移把目标拉近再重试，最多 PLACE_IK_RETRY_MAX 次。
        for attempt in range(PLACE_IK_RETRY_MAX + 1):
            above_position = np.array(position)
            above_position[2] += self.place_z_offset
            msg = set_pose_target(above_position, yaw, [-180.0, 180.0], 1.0)
            res = self.send_request(self.set_pose_target_client, msg)
            if res.pulse:
                break
            if attempt >= PLACE_IK_RETRY_MAX:
                self.get_logger().info("place target no ik solution, retry searching place target")
                set_servo_position(self.joints_pub, 1, HOLDING_POSE)
                time.sleep(1)
                self.reset_place_tracking()
                self.reset_search()
                self.waiting_place = True
                self.moving = False
                return
            self.get_logger().info(
                "place target no ik solution, creep forward %.3fm" % PLACE_IK_RETRY_STEP
            )
            self.creep_forward(PLACE_IK_RETRY_STEP)
            position[0] -= PLACE_IK_RETRY_STEP

        if not self.place_on_target(position, yaw):
            self.get_logger().info("place failed, retry searching place target")
            set_servo_position(self.joints_pub, 1, HOLDING_POSE)
            time.sleep(1)
            self.reset_place_tracking()
            self.reset_search()
            self.waiting_place = True
            self.moving = False
            return

        set_servo_position(self.joints_pub, 1, WAITING_POSE)
        time.sleep(2)
        self.reset_place_tracking()
        if self.tracker is not None:
            self.tracker.yaw = 500
            self.tracker.pitch = 150
            self.tracker.pid_yaw.clear()
            self.tracker.pid_pitch.clear()
        self.stamp = time.time()
        self.waiting_place = False
        self.command_mode = None
        self.moving = False

    def place_cube(self):
        self.stop_base()
        self.last_place_success = False

        try:
            # Fixed chassis-left placement, mirrored from track_and_grab's right-side sequence.
            set_servo_position(self.joints_pub, 1, CUBE_PLACE_READY_POSE)
            time.sleep(1)

            set_servo_position(self.joints_pub, 1.5, CUBE_PLACE_DOWN_POSE)
            time.sleep(1.5)

            set_servo_position(
                self.joints_pub,
                1,
                CUBE_PLACE_DOWN_POSE + ((10, 150),),
            )
            time.sleep(1.5)

            set_servo_position(self.joints_pub, 1, WAITING_POSE)
            time.sleep(1)

            # 只有完整执行完放置动作后才认为放置成功。
            self.last_place_success = True

        finally:
            self.command_mode = None
            self.moving = False
            self.waiting_place = False
            self.stamp = time.time()

    def pick(self, position, auto_place=True, clear_command=True):
        self.stop_base()
        pitch = PICK_PITCH_NEAR if position[2] < 0.2 else PICK_PITCH_FAR
        msg = set_pose_target(position, pitch, PICK_PITCH_RANGE, 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if not res.pulse:
            if self.command_mode in ("grasp", "grasp_cube") and self.pick_ik_retry_count < PICK_IK_MAX_RETRIES:
                self.pick_ik_retry_count += 1
                self.get_logger().info(
                    "pick target no ik solution, backoff retry %d/%d"
                    % (self.pick_ik_retry_count, PICK_IK_MAX_RETRIES)
                )
                twist = Twist()
                twist.linear.x = -PICK_IK_BACKOFF_SPEED
                self.cmd_vel_pub.publish(twist)
                time.sleep(PICK_IK_BACKOFF_TIME)
                self.stop_base()

                # Re-enter visual search after backing up a little.
                self.reset_search()
                self.last_pitch_yaw = (0, 0)
                self.stamp = time.time()
                self.start_stamp = time.time()
                self.start = True
                self.moving = False
                return False
            if self.command_mode in ("grasp", "grasp_cube"):
                self.get_logger().info("pick ik retry exhausted")
            self.handle_pick_failure("pick target no ik solution")
            return False

        servo_data = res.pulse
        set_servo_position(self.joints_pub, 1, ((1, servo_data[0]),))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
        time.sleep(1.5)

        set_servo_position(self.joints_pub, 0.5, ((10, 800),))
        time.sleep(1)
        # 抬升量：3cm 时抬臂插值弧线可能先下沉导致盖子碰地，加大到 6cm 抬高最小高度
        position[2] += 0.06

        msg = set_pose_target(position, pitch, PICK_PITCH_RANGE, 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if not res.pulse:
            # The gripper is already closed here, so backing the chassis up is unsafe.
            self.handle_pick_failure("lift target no ik solution")
            return False

        servo_data = res.pulse
        set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1, HOLDING_POSE)
        time.sleep(1)

        self.place_yaw = pitch
        self.last_pick_success = True
        self.pick_ik_retry_count = 0
        self.reset_place_tracking()
        self.reset_search()
        self.waiting_place = bool(auto_place)
        if not auto_place and clear_command:
            self.command_mode = None
            self.start = False
        self.stamp = time.time()
        self.moving = False
        return True

    def handle_pick_failure(self, message):
        self.get_logger().info(message)
        # 每次新抓取开始时已经将 last_pick_success 重置为 False。
        # 如果本次抓取已经成功，后续迟到的失败分支不能覆盖成功状态。
        self.stamp = time.time()
        if self.command_mode in ("grasp", "grasp_cube"):
            self.command_mode = None
            self.start = False
        self.moving = False

    def start_pick_thread(self, pose_t):
        if self.moving:
            return
        self.moving = True
        auto_place = self.command_mode != "grasp_cube"
        if self.command_mode in ("grasp", "grasp_cube"):
            self.start = False
        threading.Thread(target=self.pick, args=(pose_t,), kwargs={"auto_place": auto_place}).start()

    def start_place_thread(self, place_position):
        if self.moving:
            return
        self.moving = True
        self.waiting_place = False
        threading.Thread(target=self.place_and_finish, args=(place_position, self.place_yaw)).start()

    def process_frame(self, rgb_image, depth_image, depth_camera_info):
        result_image = np.copy(rgb_image) if self.enable_disp else None
        depth_color_map = None
        if self.enable_disp:
            sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)
            depth_color_map = cv2.applyColorMap((sim_depth_image / 2000.0 * 255.0).astype(np.uint8), cv2.COLORMAP_JET)

        if self.waiting_place and not self.moving:
            result_image, place_position = self.find_place_position(rgb_image, result_image, depth_image, depth_camera_info)
            if place_position is not None:
                self.get_logger().info("\033[1;32m%s target found, start placing\033[0m" % self.place_color)
                self.start_place_thread(place_position)
        elif self.tracker is not None and not self.moving and time.time() > self.start_stamp and self.start:
            result_image = self.handle_pick_frame(rgb_image, result_image, depth_image, depth_camera_info, depth_color_map)

        return result_image, depth_color_map

    def show_frame(self, result_image, depth_color_map):
        if not self.enable_disp:
            return
        if depth_color_map is None:
            return
        display_image = np.concatenate([result_image, depth_color_map], axis=1)
        cv2.imshow("depth", display_image)
        key = cv2.waitKey(1)
        if key == ord("q") or key == 27:
            self.running = False

    def main(self):
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            try:
                if not self.is_active():
                    continue
                rgb_image = np.ndarray(
                    shape=(ros_rgb_image.height, ros_rgb_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_rgb_image.data,
                )
                depth_image = np.ndarray(
                    shape=(ros_depth_image.height, ros_depth_image.width),
                    dtype=np.uint16,
                    buffer=ros_depth_image.data,
                )
                result_image, depth_color_map = self.process_frame(rgb_image, depth_image, depth_camera_info)
                self.show_frame(result_image, depth_color_map)
            except Exception as e:
                self.stop_base()
                self.get_logger().info("error1: " + str(e))

        self.stop_base()
        rclpy.shutdown()


def main():
    node = FireAgvNode("fire_agv")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()


if __name__ == "__main__":
    main()
