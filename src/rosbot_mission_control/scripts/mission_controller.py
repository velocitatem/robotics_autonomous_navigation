#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from rosbot_competition_msgs.msg import SpatialDetection, MissionEvent
from rosbot_competition_msgs.srv import GraspPuck
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


# Shared memory for the state machine
class MissionData:
    def __init__(self):
        self.puck_memory = {}
        self.drop_zone_memory = {}
        self.completed_colors = set()
        self.target_order = ["red", "green", "blue"]
        self.current_target_color = None
        self.target_index = 0
        self.latest_map = None
        self.latest_scan = None
        self.arena = None
        self.initial_robot_pose = None
        self.inspected_tag_corners = set()
        self.scan_points = []
        # Yaw of the laser frame expressed in base_link (radians).
        # On the rosbot the lidar is mounted with rpy=(0,0,3.14), i.e. the
        # laser's +X axis points toward the robot's REAR. We must know this
        # offset to correctly interpret scan angles as "robot-forward",
        # otherwise the safety monitor flips front/rear and the robot drives
        # straight into walls.
        self.laser_yaw_offset = 0.0


mission_data = MissionData()


def _memory_point(entry):
    if isinstance(entry, dict):
        return entry.get("point")
    return entry


def _memory_stamp(entry):
    if isinstance(entry, dict):
        return entry.get("stamp")
    return None


def _is_fresh(entry, max_age_sec):
    if max_age_sec <= 0.0:
        return True
    stamp = _memory_stamp(entry)
    if stamp is None:
        return True
    return (rospy.Time.now() - stamp).to_sec() <= max_age_sec


def _goal_key(x, y, resolution=0.10):
    return (round(float(x) / resolution), round(float(y) / resolution))


# Helper function to get robot pose
def get_robot_pose(tf_buffer):
    try:
        transform = tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(0.5)
        )
        return Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z,
        )
    except Exception as e:
        rospy.logwarn(f"TF lookup failed: {e}")
        return None


def get_robot_pose_yaw(tf_buffer):
    try:
        transform = tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(0.5)
        )
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z,
            ),
            yaw,
        )
    except Exception as e:
        rospy.logwarn(f"TF lookup failed: {e}")
        return None, None


def _angle_wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def publish_event(pub, text):
    msg = MissionEvent(level="INFO", message=text)
    pub.publish(msg)
    rospy.loginfo(text)


def _scan_sector_min(scan, center_angle_base, half_width):
    """Minimum range within an angular sector.

    `center_angle_base` is interpreted in the robot/base frame (0 = forward,
    pi = rear). Internally each ray's laser-frame angle is rotated by
    `mission_data.laser_yaw_offset` so callers can reason in base coordinates
    even when the lidar is mounted at a non-zero yaw.
    """
    if scan is None or not scan.ranges or scan.angle_increment == 0.0:
        return None
    offset = mission_data.laser_yaw_offset
    values = []
    for idx, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance <= scan.range_min:
            continue
        angle_base = scan.angle_min + idx * scan.angle_increment + offset
        if abs(_angle_wrap(angle_base - center_angle_base)) <= half_width:
            values.append(distance)
    if not values:
        return None
    return min(values)


def _scan_best_open_direction(scan, half_width=0.35, prefer_angle=None):
    """Sample the lidar in narrow sectors and pick the one with the largest
    minimum range. Returns the chosen sector center in BASE frame (0 = robot
    forward) so callers can use it directly as a heading. `prefer_angle` is
    also expected in base frame.
    """
    if scan is None or not scan.ranges or scan.angle_increment == 0.0:
        return None
    samples = 24
    best = None
    for k in range(samples):
        # Sector centers are sampled in the BASE frame.
        center = -math.pi + (2.0 * math.pi) * (k + 0.5) / samples
        clearance = _scan_sector_min(scan, center, half_width)
        if clearance is None:
            continue
        score = clearance
        if prefer_angle is not None:
            misalign = abs(_angle_wrap(center - prefer_angle))
            score -= 0.6 * misalign
        if best is None or score > best[0]:
            best = (score, center, clearance)
    return None if best is None else best[1]


class SafetyMonitor:
    def __init__(self):
        self.front_stop_m = float(rospy.get_param("~front_stop_m", 0.32))
        self.rear_stop_m = float(rospy.get_param("~rear_stop_m", 0.24))
        self.side_stop_m = float(rospy.get_param("~side_stop_m", 0.18))
        self.sector_width_rad = float(rospy.get_param("~safety_sector_width_rad", 0.45))
        self.max_safe_linear = float(rospy.get_param("~max_safe_linear_speed", 0.06))
        self.max_safe_angular = float(rospy.get_param("~max_safe_angular_speed", 0.45))

    def filter_twist(self, cmd):
        filtered = Twist()
        filtered.linear.x = _clamp(cmd.linear.x, -self.max_safe_linear, self.max_safe_linear)
        filtered.angular.z = _clamp(cmd.angular.z, -self.max_safe_angular, self.max_safe_angular)

        if filtered.linear.x > 0.0 and not self.can_drive_forward():
            filtered.linear.x = 0.0
        if filtered.linear.x < 0.0 and not self.can_drive_backward():
            filtered.linear.x = 0.0
        if abs(filtered.angular.z) > 0.0 and not self.can_rotate():
            filtered.angular.z = 0.0
        return filtered

    def can_drive_forward(self):
        front = _scan_sector_min(mission_data.latest_scan, 0.0, self.sector_width_rad)
        return front is None or front > self.front_stop_m

    def can_drive_backward(self):
        rear = _scan_sector_min(mission_data.latest_scan, math.pi, self.sector_width_rad)
        return rear is None or rear > self.rear_stop_m

    def can_rotate(self):
        # In-place rotation is the safest recovery motion for this circular base.
        # Blocking it in corners can deadlock the mission before the robot can face open space.
        return True


def publish_safe_twist(cmd_pub, cmd, safety, event_pub=None, label="motion"):
    filtered = safety.filter_twist(cmd)
    if (
        abs(cmd.linear.x) > 1e-4
        and abs(filtered.linear.x) < 1e-4
        or abs(cmd.angular.z) > 1e-4
        and abs(filtered.angular.z) < 1e-4
    ):
        if event_pub is not None:
            rospy.logwarn_throttle(2.0, "[MISSION] Safety gate blocked %s.", label)
    cmd_pub.publish(filtered)
    return abs(filtered.linear.x) > 1e-4 or abs(filtered.angular.z) > 1e-4


def is_goal_safe(goal, safety_margin_m=0.20):
    arena = mission_data.arena
    if arena is not None:
        if not arena.contains(goal["x"], goal["y"], safety_margin_m):
            return False
    grid = mission_data.latest_map
    if grid is None or not grid.data:
        return True
    gx = int((goal["x"] - grid.info.origin.position.x) / grid.info.resolution)
    gy = int((goal["y"] - grid.info.origin.position.y) / grid.info.resolution)
    if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
        return False
    data = np.array(grid.data, dtype=np.int16).reshape((grid.info.height, grid.info.width))
    radius = max(1, int(math.ceil(safety_margin_m / max(grid.info.resolution, 1e-3))))
    x0, x1 = max(0, gx - radius), min(grid.info.width, gx + radius + 1)
    y0, y1 = max(0, gy - radius), min(grid.info.height, gy + radius + 1)
    patch = data[y0:y1, x0:x1]
    return not np.any(patch > 50)


def servo_to_map_point(
    tf_buffer,
    cmd_pub,
    event_pub,
    target_x,
    target_y,
    tolerance_m,
    timeout_sec,
    linear_speed,
    angular_gain,
    max_angular,
    label,
    safety=None,
):
    """Reactive go-to-point that uses lidar to route around obstacles.

    When the desired heading toward the target is blocked, instead of
    stopping, the controller picks the most open lidar sector (biased
    toward the target bearing) and rotates onto it before driving.
    """

    publish_event(event_pub, f"[MISSION] Servo navigation to {label}.")
    safety = safety or SafetyMonitor()
    start = rospy.Time.now()
    rate = rospy.Rate(10)
    last_distance = None
    stagnant_cycles = 0
    while not rospy.is_shutdown():
        robot, yaw = get_robot_pose_yaw(tf_buffer)
        if robot is None or yaw is None:
            return False
        dx = target_x - robot.x
        dy = target_y - robot.y
        distance = math.hypot(dx, dy)
        if distance <= tolerance_m:
            cmd_pub.publish(Twist())
            return True
        bearing = math.atan2(dy, dx)
        target_heading_error = _angle_wrap(bearing - yaw)

        scan = mission_data.latest_scan
        # heading_error is the angle (in the robot frame) we want to rotate
        # toward. Default to the target bearing; if that direction is too
        # close to an obstacle, fall back to the most open lidar sector.
        heading_error = target_heading_error
        target_clearance = _scan_sector_min(scan, target_heading_error, 0.35)
        if target_clearance is not None and target_clearance < safety.front_stop_m + 0.05:
            best_angle = _scan_best_open_direction(
                scan, half_width=0.30, prefer_angle=target_heading_error
            )
            if best_angle is not None:
                heading_error = _angle_wrap(best_angle)

        cmd = Twist()
        cmd.angular.z = _clamp(angular_gain * heading_error, -max_angular, max_angular)
        if abs(heading_error) < 0.5:
            forward_clear = _scan_sector_min(scan, 0.0, 0.30)
            if forward_clear is None or forward_clear > safety.front_stop_m + 0.04:
                cmd.linear.x = linear_speed * max(0.25, 1.0 - abs(heading_error))

        if cmd.linear.x > 0.0 and last_distance is not None and distance >= last_distance - 0.01:
            stagnant_cycles += 1
        else:
            stagnant_cycles = 0
        last_distance = distance
        if stagnant_cycles >= 120:
            cmd_pub.publish(Twist())
            publish_event(event_pub, f"[MISSION] Servo navigation to {label} made no progress.")
            return False

        publish_safe_twist(cmd_pub, cmd, safety, event_pub, label)
        if (rospy.Time.now() - start).to_sec() >= timeout_sec:
            cmd_pub.publish(Twist())
            publish_event(event_pub, f"[MISSION] Servo navigation to {label} timed out.")
            return False
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    cmd_pub.publish(Twist())
    return False


class OccupancyAnalyzer:
    def __init__(self):
        self.occupied_threshold = int(rospy.get_param("~bbox_occupied_threshold", 50))
        self.bbox_eps_m = float(rospy.get_param("~bbox_eps_m", 0.05))
        self.bbox_min_perimeter_cells = int(rospy.get_param("~bbox_min_perimeter_cells", 60))
        self.bbox_min_free_cells = int(rospy.get_param("~bbox_min_free_cells", 40))
        self.frontier_min_unknown_neighbors = int(
            rospy.get_param("~frontier_min_unknown_neighbors", 1)
        )
        self.frontier_cluster_min_cells = int(
            rospy.get_param("~frontier_cluster_min_cells", 4)
        )
        self.goal_clearance_cells = int(rospy.get_param("~goal_clearance_cells", 2))

    def compute_bbox(self, grid):
        if grid is None or not grid.data:
            return None
        data = np.array(grid.data, dtype=np.int16).reshape((grid.info.height, grid.info.width))
        ys, xs = np.where(data > self.occupied_threshold)
        if xs.size < self.bbox_min_perimeter_cells:
            ys, xs = np.where(data == 0)
            if xs.size < self.bbox_min_free_cells:
                return None
        min_x, min_y = self._grid_to_world(grid, xs.min(), ys.min())
        max_x, max_y = self._grid_to_world(grid, xs.max(), ys.max())
        return {
            "min_x": min(min_x, max_x),
            "min_y": min(min_y, max_y),
            "max_x": max(min_x, max_x),
            "max_y": max(min_y, max_y),
        }

    def compute_points_bbox(self, points):
        if len(points) < self.bbox_min_free_cells:
            return None
        xs = [point.x for point in points]
        ys = [point.y for point in points]
        if max(xs) - min(xs) < 0.5 or max(ys) - min(ys) < 0.5:
            return None
        return {
            "min_x": min(xs),
            "min_y": min(ys),
            "max_x": max(xs),
            "max_y": max(ys),
        }

    def is_stable(self, prev_bbox, new_bbox):
        if prev_bbox is None or new_bbox is None:
            return False
        keys = ("min_x", "min_y", "max_x", "max_y")
        return all(abs(prev_bbox[key] - new_bbox[key]) <= self.bbox_eps_m for key in keys)

    def frontier_goal(self, grid, robot):
        if grid is None or robot is None:
            return None
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data, dtype=np.int16).reshape((height, width))
        frontier_mask = np.zeros((height, width), dtype=bool)

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] != 0:
                    continue
                unknown_neighbors = np.count_nonzero(data[y - 1 : y + 2, x - 1 : x + 2] < 0)
                if unknown_neighbors >= self.frontier_min_unknown_neighbors:
                    frontier_mask[y, x] = True

        clusters = self._cluster_frontiers(frontier_mask)
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.frontier_cluster_min_cells:
                continue
            cx = sum(cell[0] for cell in cluster) / float(len(cluster))
            cy = sum(cell[1] for cell in cluster) / float(len(cluster))
            if not self._is_clear(data, int(round(cx)), int(round(cy))):
                continue
            wx, wy = self._grid_to_world(grid, cx, cy)
            dist = math.hypot(wx - robot.x, wy - robot.y)
            score = dist - 0.03 * len(cluster)
            candidates.append((score, wx, wy))

        if not candidates:
            return None
        _score, x, y = min(candidates, key=lambda item: item[0])
        return {"x": x, "y": y, "yaw": math.atan2(y - robot.y, x - robot.x)}

    def _cluster_frontiers(self, mask):
        height, width = mask.shape
        seen = np.zeros_like(mask, dtype=bool)
        clusters = []
        for y in range(height):
            for x in range(width):
                if seen[y, x] or not mask[y, x]:
                    continue
                stack = [(x, y)]
                seen[y, x] = True
                cluster = []
                while stack:
                    cx, cy = stack.pop()
                    cluster.append((cx, cy))
                    for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if seen[ny, nx] or not mask[ny, nx]:
                            continue
                        seen[ny, nx] = True
                        stack.append((nx, ny))
                clusters.append(cluster)
        return clusters

    def _is_clear(self, data, x, y):
        height, width = data.shape
        radius = self.goal_clearance_cells
        x0, x1 = max(0, x - radius), min(width, x + radius + 1)
        y0, y1 = max(0, y - radius), min(height, y + radius + 1)
        patch = data[y0:y1, x0:x1]
        return not np.any(patch > self.occupied_threshold)

    def _grid_to_world(self, grid, gx, gy):
        return (
            grid.info.origin.position.x + (float(gx) + 0.5) * grid.info.resolution,
            grid.info.origin.position.y + (float(gy) + 0.5) * grid.info.resolution,
        )


class ArenaModel:
    def __init__(self, bbox):
        self.bbox = dict(bbox)
        self.color_to_corner = {}
        self.corner_to_color = {}
        self.start_corner_idx = None

    def corners(self):
        return [
            Point(x=self.bbox["min_x"], y=self.bbox["min_y"], z=0.0),
            Point(x=self.bbox["min_x"], y=self.bbox["max_y"], z=0.0),
            Point(x=self.bbox["max_x"], y=self.bbox["max_y"], z=0.0),
            Point(x=self.bbox["max_x"], y=self.bbox["min_y"], z=0.0),
        ]

    def center(self):
        return Point(
            x=0.5 * (self.bbox["min_x"] + self.bbox["max_x"]),
            y=0.5 * (self.bbox["min_y"] + self.bbox["max_y"]),
            z=0.0,
        )

    def contains(self, x, y, margin_m=0.0):
        return (
            self.bbox["min_x"] + margin_m <= x <= self.bbox["max_x"] - margin_m
            and self.bbox["min_y"] + margin_m <= y <= self.bbox["max_y"] - margin_m
        )

    def nearest_corner(self, point):
        corners = self.corners()
        distances = [math.hypot(point.x - corner.x, point.y - corner.y) for corner in corners]
        return int(np.argmin(distances)), float(min(distances))

    def assign_color(self, color, point, max_distance_m):
        corner_idx, distance = self.nearest_corner(point)
        if distance > max_distance_m:
            return False
        previous_color = self.corner_to_color.get(corner_idx)
        if previous_color and previous_color != color:
            self.color_to_corner.pop(previous_color, None)
        previous_corner = self.color_to_corner.get(color)
        if previous_corner is not None:
            self.corner_to_color.pop(previous_corner, None)
        self.color_to_corner[color] = corner_idx
        self.corner_to_color[corner_idx] = color
        return True

    def unmapped_corners(self):
        return [idx for idx in range(4) if idx not in self.corner_to_color]

    def set_start_corner(self, robot_pose):
        if robot_pose is None:
            return
        self.start_corner_idx, _distance = self.nearest_corner(robot_pose)

    def vantage(self, corner_idx, inset_m):
        corner = self.corners()[corner_idx]
        center = self.center()
        dx = center.x - corner.x
        dy = center.y - corner.y
        norm = max(math.hypot(dx, dy), 1e-3)
        x = corner.x + inset_m * dx / norm
        y = corner.y + inset_m * dy / norm
        return {"x": x, "y": y, "yaw": math.atan2(corner.y - y, corner.x - x)}

    def quadrant_goal(self, corner_idx, inset_m):
        corner = self.corners()[corner_idx]
        center = self.center()
        x = 0.5 * (corner.x + center.x)
        y = 0.5 * (corner.y + center.y)
        dx = center.x - x
        dy = center.y - y
        norm = max(math.hypot(dx, dy), 1e-3)
        x += inset_m * dx / norm
        y += inset_m * dy / norm
        return {"x": x, "y": y, "yaw": math.atan2(corner.y - y, corner.x - x)}


def _goal_pose(goal):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(goal["x"])
    pose.pose.position.y = float(goal["y"])
    yaw = float(goal.get("yaw", 0.0))
    half = 0.5 * yaw
    pose.pose.orientation.z = math.sin(half)
    pose.pose.orientation.w = math.cos(half)
    return pose


def navigate_to_goal(
    move_base,
    move_base_ready,
    tf_buffer,
    cmd_pub,
    event_pub,
    goal,
    timeout_sec,
    tolerance_m,
    linear_speed,
    angular_gain,
    max_angular,
    label,
    safety=None,
):
    safety = safety or SafetyMonitor()
    safe_margin = float(rospy.get_param("~safe_goal_margin_m", 0.20))
    if not is_goal_safe(goal, safe_margin):
        publish_event(
            event_pub,
            f"[MISSION] Refusing unsafe navigation goal for {label} at ({goal['x']:.2f}, {goal['y']:.2f}).",
        )
        return False, move_base_ready

    move_base_ready = move_base_ready or move_base.wait_for_server(rospy.Duration(1.0))
    if not move_base_ready:
        arrived = servo_to_map_point(
            tf_buffer,
            cmd_pub,
            event_pub,
            goal["x"],
            goal["y"],
            tolerance_m,
            timeout_sec,
            linear_speed,
            angular_gain,
            max_angular,
            label,
            safety,
        )
        return arrived, move_base_ready

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose = _goal_pose(goal)
    move_base.send_goal(mb_goal)
    start = rospy.Time.now()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        robot = get_robot_pose(tf_buffer)
        if robot is not None:
            if math.hypot(robot.x - goal["x"], robot.y - goal["y"]) <= tolerance_m:
                move_base.cancel_goal()
                return True, move_base_ready
        state = move_base.get_state()
        if state == 3:
            return True, move_base_ready
        if state in [4, 5, 9]:
            return False, move_base_ready
        if (rospy.Time.now() - start).to_sec() >= timeout_sec:
            move_base.cancel_goal()
            publish_event(event_pub, f"[MISSION] Navigation to {label} timed out.")
            return False, move_base_ready
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    move_base.cancel_goal()
    return False, move_base_ready


class InitScan(smach.State):
    def __init__(self, tf_buffer, event_pub, analyzer):
        smach.State.__init__(self, outcomes=["scan_done"])
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.analyzer = analyzer
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.init_spin_yaw_speed = float(rospy.get_param("~init_spin_yaw_speed", 0.5))
        self.init_spin_duration_sec = float(rospy.get_param("~init_spin_duration_sec", 13.0))
        self.init_max_attempts = int(rospy.get_param("~init_max_attempts", 3))
        self.init_nudge_distance_m = float(rospy.get_param("~init_nudge_distance_m", 0.4))
        self.goal_tolerance_m = float(rospy.get_param("~explore_goal_tolerance_m", 0.18))
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: INIT_SCAN")
        previous_bbox = None
        last_bbox = None
        for attempt in range(1, self.init_max_attempts + 1):
            self._spin_once()
            self._accumulate_scan_points()
            bbox = self.analyzer.compute_bbox(mission_data.latest_map)
            if bbox is None:
                bbox = self.analyzer.compute_points_bbox(mission_data.scan_points)
            if bbox is not None:
                last_bbox = bbox
                if self.analyzer.is_stable(previous_bbox, bbox) or attempt == self.init_max_attempts:
                    mission_data.arena = ArenaModel(bbox)
                    robot = get_robot_pose(self.tf_buffer)
                    mission_data.initial_robot_pose = robot
                    mission_data.arena.set_start_corner(robot)
                    publish_event(self.event_pub, "[MISSION] SLAM arena bounds initialized.")
                    return "scan_done"
                previous_bbox = bbox
            publish_event(self.event_pub, "[MISSION] Arena bounds not stable yet; continuing rotation-only scan.")

        if last_bbox is not None:
            mission_data.arena = ArenaModel(last_bbox)
            robot = get_robot_pose(self.tf_buffer)
            mission_data.initial_robot_pose = robot
            mission_data.arena.set_start_corner(robot)
        return "scan_done"

    def _spin_once(self):
        cmd = Twist()
        cmd.angular.z = self.init_spin_yaw_speed
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() >= self.init_spin_duration_sec:
                break
            publish_safe_twist(self.cmd_pub, cmd, self.safety, self.event_pub, "init scan")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        self.cmd_pub.publish(Twist())
        try:
            rospy.sleep(0.3)
        except rospy.ROSInterruptException:
            pass

    def _accumulate_scan_points(self):
        scan = mission_data.latest_scan
        robot, yaw = get_robot_pose_yaw(self.tf_buffer)
        if scan is None or robot is None or yaw is None:
            return
        step = max(1, len(scan.ranges) // 180)
        offset = mission_data.laser_yaw_offset
        new_points = []
        for idx in range(0, len(scan.ranges), step):
            distance = scan.ranges[idx]
            if not math.isfinite(distance):
                continue
            if distance <= scan.range_min or distance >= scan.range_max:
                continue
            # angle in world = robot yaw + (ray angle in base frame)
            #                = robot yaw + (ray angle in laser frame) + laser->base yaw offset
            angle = yaw + scan.angle_min + idx * scan.angle_increment + offset
            new_points.append(
                Point(
                    x=robot.x + distance * math.cos(angle),
                    y=robot.y + distance * math.sin(angle),
                    z=0.0,
                )
            )
        if new_points:
            mission_data.scan_points.extend(new_points)
            mission_data.scan_points = mission_data.scan_points[-1200:]
            publish_event(
                self.event_pub,
                f"[MISSION] Accumulated {len(mission_data.scan_points)} lidar wall points for arena fit.",
            )

    def _nudge_to_frontier(self):
        robot = get_robot_pose(self.tf_buffer)
        goal = self.analyzer.frontier_goal(mission_data.latest_map, robot)
        if goal is None and robot is not None:
            _point, yaw = get_robot_pose_yaw(self.tf_buffer)
            yaw = yaw if yaw is not None else 0.0
            goal = {
                "x": robot.x + self.init_nudge_distance_m * math.cos(yaw),
                "y": robot.y + self.init_nudge_distance_m * math.sin(yaw),
                "yaw": yaw,
            }
        if goal is None:
            return
        arrived, self.move_base_ready = navigate_to_goal(
            self.move_base,
            self.move_base_ready,
            self.tf_buffer,
            self.cmd_pub,
            self.event_pub,
            goal,
            12.0,
            self.goal_tolerance_m,
            self.servo_goal_linear_speed,
            self.servo_goal_angular_gain,
            self.servo_goal_max_angular,
            "frontier nudge",
        )
        if not arrived:
            try:
                rospy.sleep(0.3)
            except rospy.ROSInterruptException:
                pass


class DiscoverCorners(smach.State):
    def __init__(self, tf_buffer, event_pub, analyzer):
        smach.State.__init__(self, outcomes=["tags_complete", "tags_missing", "need_scan"])
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.analyzer = analyzer
        self.required_drop_zone_count = int(
            rospy.get_param("~required_drop_zone_count", len(mission_data.target_order))
        )
        self.tag_corner_snap_distance_m = float(
            rospy.get_param("~tag_corner_snap_distance_m", 0.50)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: DISCOVER_CORNERS")
        if mission_data.arena is None:
            bbox = self.analyzer.compute_bbox(mission_data.latest_map)
            if bbox is None:
                bbox = self.analyzer.compute_points_bbox(mission_data.scan_points)
            if bbox is None:
                publish_event(self.event_pub, "[MISSION] Arena bounds unavailable; rescanning.")
                return "need_scan"
            mission_data.arena = ArenaModel(bbox)
            mission_data.arena.set_start_corner(get_robot_pose(self.tf_buffer))

        for color, entry in list(mission_data.drop_zone_memory.items()):
            point = _memory_point(entry)
            if point is None:
                continue
            if mission_data.arena.assign_color(color, point, self.tag_corner_snap_distance_m):
                corner_idx = mission_data.arena.color_to_corner[color]
                mission_data.inspected_tag_corners.add(corner_idx)
                publish_event(
                    self.event_pub,
                    f"[MISSION] Mapped {color} tag to arena corner {corner_idx}.",
                )

        known = len(
            [color for color in mission_data.target_order if color in mission_data.arena.color_to_corner]
        )
        publish_event(
            self.event_pub,
            f"[MISSION] Corner tags known: {known}/{self.required_drop_zone_count}.",
        )
        if known >= self.required_drop_zone_count:
            return "tags_complete"
        return "tags_missing"


class TagTour(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["tour_progress"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor()
        self.corner_vantage_inset_m = float(rospy.get_param("~corner_vantage_inset_m", 0.55))
        self.tag_tour_goal_timeout_sec = float(
            rospy.get_param("~tag_tour_goal_timeout_sec", 25.0)
        )
        self.tag_dwell_sec = float(rospy.get_param("~tag_dwell_sec", 1.5))
        self.goal_tolerance_m = float(rospy.get_param("~explore_goal_tolerance_m", 0.18))
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: TAG_TOUR")
        arena = mission_data.arena
        if arena is None:
            return "tour_progress"
        target_corners = [
            idx for idx in arena.unmapped_corners() if idx not in mission_data.inspected_tag_corners
        ]
        if not target_corners:
            mission_data.inspected_tag_corners.clear()
            target_corners = arena.unmapped_corners()
        goals = [(idx, arena.vantage(idx, self.corner_vantage_inset_m)) for idx in target_corners]
        robot = get_robot_pose(self.tf_buffer)
        if robot is not None:
            goals.sort(key=lambda item: math.hypot(item[1]["x"] - robot.x, item[1]["y"] - robot.y))

        for corner_idx, goal in goals:
            if corner_idx not in arena.unmapped_corners():
                continue
            publish_event(self.event_pub, f"[MISSION] Visiting corner {corner_idx} to read tag.")
            _arrived, self.move_base_ready = navigate_to_goal(
                self.move_base,
                self.move_base_ready,
                self.tf_buffer,
                self.cmd_pub,
                self.event_pub,
                goal,
                self.tag_tour_goal_timeout_sec,
                self.goal_tolerance_m,
                self.servo_goal_linear_speed,
                self.servo_goal_angular_gain,
                self.servo_goal_max_angular,
                f"corner {corner_idx} tag vantage",
                self.safety,
            )
            try:
                rospy.sleep(self.tag_dwell_sec)
            except rospy.ROSInterruptException:
                pass
            mission_data.inspected_tag_corners.add(corner_idx)
            break
        return "tour_progress"


class LocatePuck(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["found_puck", "continue_search", "all_done"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor()
        self.known_object_stale_sec = float(rospy.get_param("~known_object_stale_sec", 0.0))
        self.puck_search_quadrant_inset_m = float(
            rospy.get_param("~puck_search_quadrant_inset_m", 0.45)
        )
        self.locate_goal_timeout_sec = float(rospy.get_param("~locate_goal_timeout_sec", 20.0))
        self.goal_tolerance_m = float(rospy.get_param("~explore_goal_tolerance_m", 0.18))
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: LOCATE_PUCK")
        next_color = self._next_target_color()
        if next_color is None:
            return "all_done"
        mission_data.current_target_color = next_color
        if self._target_is_known(next_color):
            publish_event(self.event_pub, f"[MISSION] Located known {next_color} puck.")
            return "found_puck"

        arena = mission_data.arena
        if arena is None:
            self._active_scan()
            return "found_puck" if self._target_is_known(next_color) else "continue_search"

        goals = [
            (idx, arena.quadrant_goal(idx, self.puck_search_quadrant_inset_m))
            for idx in range(4)
        ]
        robot = get_robot_pose(self.tf_buffer)
        if robot is not None:
            goals.sort(key=lambda item: math.hypot(item[1]["x"] - robot.x, item[1]["y"] - robot.y))
        if arena.start_corner_idx is not None:
            goals.sort(key=lambda item: item[0] == arena.start_corner_idx)

        for corner_idx, goal in goals:
            publish_event(
                self.event_pub,
                f"[MISSION] Searching quadrant near corner {corner_idx} for {next_color} puck.",
            )
            _arrived, self.move_base_ready = navigate_to_goal(
                self.move_base,
                self.move_base_ready,
                self.tf_buffer,
                self.cmd_pub,
                self.event_pub,
                goal,
                self.locate_goal_timeout_sec,
                self.goal_tolerance_m,
                self.servo_goal_linear_speed,
                self.servo_goal_angular_gain,
                self.servo_goal_max_angular,
                f"{next_color} puck search",
                self.safety,
            )
            self._active_scan()
            if self._target_is_known(next_color):
                publish_event(self.event_pub, f"[MISSION] Found {next_color} puck.")
                return "found_puck"
        return "found_puck" if self._target_is_known(next_color) else "continue_search"

    def _next_target_color(self):
        while mission_data.target_index < len(mission_data.target_order):
            color = mission_data.target_order[mission_data.target_index]
            if color not in mission_data.completed_colors:
                return color
            mission_data.target_index += 1
        return None

    def _target_is_known(self, color):
        entry = mission_data.puck_memory.get(color)
        return entry is not None and _is_fresh(entry, self.known_object_stale_sec)

    def _active_scan(self):
        cmd = Twist()
        cmd.angular.z = float(rospy.get_param("~active_scan_yaw_speed", 0.45))
        duration = float(rospy.get_param("~active_scan_duration_sec", 5.0))
        iterations = max(1, int(duration * 10.0))
        for _ in range(iterations):
            if self._target_is_known(mission_data.current_target_color):
                break
            publish_safe_twist(self.cmd_pub, cmd, self.safety, self.event_pub, "puck search scan")
            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break
        self.cmd_pub.publish(Twist())


class Approach(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["arrived", "failed"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor()
        self.approach_distance_m = float(rospy.get_param("~approach_distance_m", 0.30))
        self.goal_tolerance_m = float(rospy.get_param("~approach_goal_tolerance_m", 0.18))
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: APPROACH")
        color = mission_data.current_target_color
        if not color or color not in mission_data.puck_memory:
            return "failed"

        puck = _memory_point(mission_data.puck_memory[color])
        if puck is None:
            return "failed"
        robot = get_robot_pose(self.tf_buffer)
        if not robot:
            return "failed"

        dx = puck.x - robot.x
        dy = puck.y - robot.y
        norm = math.hypot(dx, dy)

        # Stand off from the puck so visual servo can make the final alignment.
        approach_dist = self.approach_distance_m
        goal_x = puck.x - approach_dist * (dx / max(norm, 1e-3))
        goal_y = puck.y - approach_dist * (dy / max(norm, 1e-3))
        yaw = math.atan2(dy, dx)

        goal_dict = {"x": goal_x, "y": goal_y, "yaw": yaw}

        publish_event(
            self.event_pub,
            f"[MISSION] Approaching {color} puck at ({goal_x:.2f}, {goal_y:.2f})",
        )
        arrived, self.move_base_ready = navigate_to_goal(
            self.move_base,
            self.move_base_ready,
            self.tf_buffer,
            self.cmd_pub,
            self.event_pub,
            goal_dict,
            30.0,
            self.goal_tolerance_m,
            self.servo_goal_linear_speed,
            self.servo_goal_angular_gain,
            self.servo_goal_max_angular,
            f"{color} puck",
            self.safety,
        )
        if arrived:
            return "arrived"
        return "failed"


class VisualServo(smach.State):
    def __init__(self, event_pub):
        smach.State.__init__(
            self, outcomes=["ready_to_grab", "wall_detected", "timeout"]
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.event_pub = event_pub
        self.safety = SafetyMonitor()
        self.bridge = CvBridge()
        self.depth_sub = None
        self.latest_depth = None
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/depth/image_raw"
        )

    def _depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception:
            pass

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: VISUAL_SERVO")
        self.latest_depth = None
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self._depth_cb
        )

        rospy.sleep(0.5)  # Wait for depth

        # Check if wall
        if self.latest_depth is not None:
            h, w = self.latest_depth.shape[:2]
            patch = self.latest_depth[
                h // 2 - 20 : h // 2 + 20, w // 2 - 20 : w // 2 + 20
            ]
            # If the variance in depth is extremely low, it might be a flat wall
            if np.var(patch) < 50:  # Very flat
                publish_event(
                    self.event_pub,
                    "[MISSION] Warning: Wall detected! Aborting visual servo.",
                )
                self.depth_sub.unregister()
                return "wall_detected"

        publish_event(self.event_pub, "[MISSION] Servoing towards puck...")
        cmd = Twist()
        cmd.linear.x = float(rospy.get_param("~visual_servo_speed", 0.04))

        # Open loop servoing for now (ideally PID based on camera)
        # We stop when depth says center pixel is ~0.06m away
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > rospy.Duration(5.0):
                self.cmd_pub.publish(Twist())
                self.depth_sub.unregister()
                return "timeout"

            if self.latest_depth is not None:
                h, w = self.latest_depth.shape[:2]
                center_depth = self.latest_depth[h // 2, w // 2]
                if 0 < center_depth < 60:  # 60mm
                    publish_event(
                        self.event_pub, "[MISSION] Perfect grasp distance reached."
                    )
                    break

            if not publish_safe_twist(
                self.cmd_pub, cmd, self.safety, self.event_pub, "visual servo"
            ):
                self.cmd_pub.publish(Twist())
                publish_event(
                    self.event_pub,
                    "[MISSION] Visual servo blocked by safety gate.",
                )
                self.depth_sub.unregister()
                return "wall_detected"
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

        self.cmd_pub.publish(Twist())
        self.depth_sub.unregister()
        return "ready_to_grab"


class Grab(smach.State):
    def __init__(self, event_pub):
        smach.State.__init__(self, outcomes=["grabbed", "missed"])
        self.grasp_srv = rospy.ServiceProxy("/grasp_puck", GraspPuck)
        self.event_pub = event_pub
        self.safety = SafetyMonitor()

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: GRAB")
        try:
            resp = self.grasp_srv(mission_data.current_target_color)
            if resp.success:
                publish_event(
                    self.event_pub,
                    f"[MISSION] Grabbed {mission_data.current_target_color} puck successfully!",
                )
                return "grabbed"
            else:
                publish_event(
                    self.event_pub,
                    f"[MISSION] Missed puck! Load threshold not reached.",
                )
        except Exception as e:
            publish_event(self.event_pub, f"[MISSION] Grasp service failed: {e}")

        # Back up a bit if we missed
        cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        cmd = Twist()
        cmd.linear.x = -float(rospy.get_param("~backup_speed", 0.08))
        for _ in range(15):
            if not publish_safe_twist(cmd_pub, cmd, self.safety, self.event_pub, "miss backup"):
                break
            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break
        cmd_pub.publish(Twist())

        return "missed"


class Deliver(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["delivered", "failed"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.release_srv = rospy.ServiceProxy("/release_puck", Trigger)
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor()
        self.dropoff_distance_m = float(rospy.get_param("~dropoff_distance_m", 0.35))
        self.goal_tolerance_m = float(rospy.get_param("~deliver_goal_tolerance_m", 0.20))
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: DELIVER")
        color = mission_data.current_target_color
        if not color:
            return "failed"

        arena = mission_data.arena
        if arena is None or color not in arena.color_to_corner:
            publish_event(
                self.event_pub,
                f"[MISSION] Unknown mapped corner for {color}. Re-exploring...",
            )
            return "failed"

        corner_idx = arena.color_to_corner[color]
        goal_dict = arena.vantage(corner_idx, self.dropoff_distance_m)

        publish_event(
            self.event_pub, f"[MISSION] Delivering {color} puck to ArUco zone..."
        )
        arrived, self.move_base_ready = navigate_to_goal(
            self.move_base,
            self.move_base_ready,
            self.tf_buffer,
            self.cmd_pub,
            self.event_pub,
            goal_dict,
            45.0,
            self.goal_tolerance_m,
            self.servo_goal_linear_speed,
            self.servo_goal_angular_gain,
            self.servo_goal_max_angular,
            f"{color} drop zone",
            self.safety,
        )
        if arrived:
            publish_event(self.event_pub, "[MISSION] Arrived at drop zone. Releasing.")
            self.release_srv()
            mission_data.completed_colors.add(color)
            mission_data.target_index += 1
            if color in mission_data.puck_memory:
                del mission_data.puck_memory[color]
            mission_data.current_target_color = None

            # Back up after releasing
            self._backup_after_release()

            return "delivered"
        return "failed"

    def _backup_after_release(self):
        cmd = Twist()
        cmd.linear.x = -float(rospy.get_param("~backup_speed", 0.08))
        for _ in range(20):
            if not publish_safe_twist(self.cmd_pub, cmd, self.safety, self.event_pub, "release backup"):
                break
            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break
        self.cmd_pub.publish(Twist())


def _on_detection(msg):
    point = Point(x=msg.map_point.x, y=msg.map_point.y, z=msg.map_point.z)
    stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
    entry = {"point": point, "stamp": stamp}
    if msg.object_class == "puck" and msg.color:
        mission_data.puck_memory[msg.color] = entry
    elif msg.object_class == "drop_zone" and msg.color:
        mission_data.drop_zone_memory[msg.color] = entry


def _on_map(msg):
    mission_data.latest_map = msg


def _on_scan(msg):
    mission_data.latest_scan = msg


def _wait_for_move_base(timeout_sec):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("[MISSION] Waiting up to %.1fs for move_base action server...", timeout_sec)
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while not rospy.is_shutdown():
        if client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("[MISSION] move_base action server is up.")
            return True
        if rospy.Time.now() >= deadline:
            rospy.logwarn("[MISSION] move_base did not come up in time; falling back to reactive servo.")
            return False
    return False


def _wait_for_first_scan(timeout_sec):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if mission_data.latest_scan is not None:
            rospy.loginfo("[MISSION] Lidar scan stream is live.")
            return True
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            return False
    rospy.logwarn("[MISSION] No lidar scans received before timeout.")
    return False


def _calibrate_laser_offset(tf_buffer, timeout_sec=10.0):
    """Look up the static base_link -> laser transform and stash its yaw.

    The rosbot mounts the rplidar with rpy=(0,0,3.14), i.e. the laser frame's
    +X is the robot's REAR. We need the offset to interpret scan angles as
    robot-forward in safety/servo logic. If the lookup fails we fall back to
    the scan's own frame_id and assume zero offset (warns loudly).
    """
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    laser_frame = "laser"
    if mission_data.latest_scan is not None and mission_data.latest_scan.header.frame_id:
        laser_frame = mission_data.latest_scan.header.frame_id
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            tf = tf_buffer.lookup_transform(
                "base_link", laser_frame, rospy.Time(0), rospy.Duration(0.5)
            )
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            mission_data.laser_yaw_offset = yaw
            rospy.loginfo(
                "[MISSION] base_link -> %s yaw offset = %.3f rad (%.1f deg).",
                laser_frame, yaw, math.degrees(yaw),
            )
            return True
        except Exception:
            try:
                rospy.sleep(0.25)
            except rospy.ROSInterruptException:
                return False
    rospy.logwarn(
        "[MISSION] Could not look up base_link -> %s; assuming 0 yaw offset. "
        "If the lidar is mounted rotated, safety checks will be wrong.",
        laser_frame,
    )
    mission_data.laser_yaw_offset = 0.0
    return False


def main():
    rospy.init_node("mission_controller")

    mission_data.target_order = rospy.get_param("~target_order", mission_data.target_order)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    analyzer = OccupancyAnalyzer()

    event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=10)
    rospy.Subscriber(
        "/perception/spatial_detections", SpatialDetection, _on_detection, queue_size=30
    )
    rospy.Subscriber("/map", OccupancyGrid, _on_map, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, _on_scan, queue_size=1)

    rospy.wait_for_service("/grasp_puck")
    rospy.wait_for_service("/release_puck")

    _wait_for_first_scan(float(rospy.get_param("~map_wait_sec", 30.0)))
    _calibrate_laser_offset(tf_buffer)
    _wait_for_move_base(float(rospy.get_param("~move_base_wait_sec", 60.0)))

    # Build SMACH
    sm = smach.StateMachine(outcomes=["MISSION_COMPLETE", "ABORTED"])
    with sm:
        smach.StateMachine.add(
            "INIT_SCAN",
            InitScan(tf_buffer, event_pub, analyzer),
            transitions={"scan_done": "DISCOVER_CORNERS"},
        )
        smach.StateMachine.add(
            "DISCOVER_CORNERS",
            DiscoverCorners(tf_buffer, event_pub, analyzer),
            transitions={
                "tags_complete": "LOCATE_PUCK",
                "tags_missing": "TAG_TOUR",
                "need_scan": "INIT_SCAN",
            },
        )
        smach.StateMachine.add(
            "TAG_TOUR",
            TagTour(tf_buffer, event_pub),
            transitions={"tour_progress": "DISCOVER_CORNERS"},
        )
        smach.StateMachine.add(
            "LOCATE_PUCK",
            LocatePuck(tf_buffer, event_pub),
            transitions={
                "found_puck": "APPROACH",
                "continue_search": "LOCATE_PUCK",
                "all_done": "MISSION_COMPLETE",
            },
        )
        smach.StateMachine.add(
            "APPROACH",
            Approach(tf_buffer, event_pub),
            transitions={"arrived": "VISUAL_SERVO", "failed": "LOCATE_PUCK"},
        )
        smach.StateMachine.add(
            "VISUAL_SERVO",
            VisualServo(event_pub),
            transitions={
                "ready_to_grab": "GRAB",
                "wall_detected": "LOCATE_PUCK",
                "timeout": "LOCATE_PUCK",
            },
        )
        smach.StateMachine.add(
            "GRAB",
            Grab(event_pub),
            transitions={"grabbed": "DELIVER", "missed": "VISUAL_SERVO"},
        )
        smach.StateMachine.add(
            "DELIVER",
            Deliver(tf_buffer, event_pub),
            transitions={"delivered": "LOCATE_PUCK", "failed": "LOCATE_PUCK"},
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("mission_smach", sm, "/SM_ROOT")
    sis.start()

    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
