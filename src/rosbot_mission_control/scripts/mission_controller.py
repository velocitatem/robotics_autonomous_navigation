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
from std_srvs.srv import Trigger, Empty
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


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
        # Color of the puck currently in the gripper, or None when empty.
        # Used so LOCATE_PUCK / APPROACH cannot re-target the puck the robot
        # is carrying (whose perception fix appears at the robot's own pose).
        self.carrying_color = None
        # Yaw of the laser frame expressed in base_link (radians).
        # On the rosbot the lidar is mounted with rpy=(0,0,3.14), i.e. the
        # laser's +X axis points toward the robot's REAR. We must know this
        # offset to correctly interpret scan angles as "robot-forward",
        # otherwise the safety monitor flips front/rear and the robot drives
        # straight into walls.
        self.laser_yaw_offset = 0.0
        # Map-frame (x, y) of every non-target, non-carried puck. Refreshed by
        # PuckObstacleBroadcaster and consumed by SafetyMonitor + servo_to_map_point
        # so the robot does not drive over floor-level pucks (the lidar at
        # ~0.12 m never sees them).
        self.latest_puck_obstacles = []


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


def clear_move_base_costmaps():
    """Wipe both costmaps so previous puck markings are forgotten when the
    set of obstacle pucks changes (after a grasp or a release). Pucks are
    invisible to the lidar so they cannot be raytrace-cleared in the normal
    way; this short call keeps the obstacle layer honest.
    """
    try:
        proxy = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        proxy.wait_for_service(timeout=1.0)
        proxy()
    except Exception as exc:
        rospy.logwarn_throttle(5.0, "[MISSION] clear_costmaps failed: %s", str(exc))


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


def _nearest_puck_distance(tf_buffer):
    """Closest horizontal distance to any cached non-target / non-carried puck.

    Used to limit in-place rotation when a floor-level puck is within the
    robot's sweep radius (lidar still does not see them).
    """
    obstacles = mission_data.latest_puck_obstacles
    if not obstacles:
        return None
    robot = get_robot_pose(tf_buffer)
    if robot is None:
        return None
    nearest = None
    for px, py in obstacles:
        d = math.hypot(px - robot.x, py - robot.y)
        if nearest is None or d < nearest:
            nearest = d
    return nearest


def _puck_clearance_in_path(tf_buffer, half_width_rad, max_range_m, base_angle=0.0):
    """Distance to the closest non-target/non-carried puck inside the given
    sector (centred on `base_angle` in the robot frame).

    Returns None if no puck is in the cone or the robot pose is unavailable.
    The pucks are stored in map frame, so we transform via the current
    robot pose+yaw lookup.
    """
    obstacles = mission_data.latest_puck_obstacles
    if not obstacles:
        return None
    robot, yaw = get_robot_pose_yaw(tf_buffer)
    if robot is None or yaw is None:
        return None
    nearest = None
    for px, py in obstacles:
        dx = px - robot.x
        dy = py - robot.y
        distance = math.hypot(dx, dy)
        if distance > max_range_m:
            continue
        bearing = math.atan2(dy, dx)
        if abs(_angle_wrap(bearing - yaw - base_angle)) > half_width_rad:
            continue
        if nearest is None or distance < nearest:
            nearest = distance
    return nearest


class SafetyMonitor:
    def __init__(self, tf_buffer=None):
        self.front_stop_m = float(rospy.get_param("~front_stop_m", 0.32))
        self.rear_stop_m = float(rospy.get_param("~rear_stop_m", 0.24))
        self.side_stop_m = float(rospy.get_param("~side_stop_m", 0.18))
        self.sector_width_rad = float(rospy.get_param("~safety_sector_width_rad", 0.45))
        self.max_safe_linear = float(rospy.get_param("~max_safe_linear_speed", 0.06))
        self.max_safe_angular = float(rospy.get_param("~max_safe_angular_speed", 0.45))
        # Distance at which a non-target puck blocks forward motion. Rounded
        # up from robot_radius (0.11 m) + a short reaction margin, so the
        # gripper does not bowl through the puck when only the camera/memory
        # knows it is there.
        self.puck_stop_m = float(rospy.get_param("~puck_stop_m", 0.32))
        # When a non-carried puck is this close, stop spinning in place so the
        # footprint does not sweep the puck. Outside `puck_rotation_outer_m`,
        # angular velocity is unconstrained for these obstacles.
        self.puck_rotation_inner_m = float(
            rospy.get_param("~puck_rotation_inner_m", 0.18)
        )
        self.puck_rotation_outer_m = float(
            rospy.get_param("~puck_rotation_outer_m", 0.40)
        )
        self.tf_buffer = tf_buffer

    def filter_twist(self, cmd):
        filtered = Twist()
        filtered.linear.x = _clamp(
            cmd.linear.x, -self.max_safe_linear, self.max_safe_linear
        )
        filtered.angular.z = _clamp(
            cmd.angular.z, -self.max_safe_angular, self.max_safe_angular
        )

        if filtered.linear.x > 0.0 and not self.can_drive_forward():
            filtered.linear.x = 0.0
        if filtered.linear.x < 0.0 and not self.can_drive_backward():
            filtered.linear.x = 0.0
        if abs(filtered.angular.z) > 0.0 and not self.can_rotate():
            filtered.angular.z = 0.0
        # Diff-drive in-place rotation can still clip a floor-level puck the
        # lidar never sees. Scale (or cut) angular motion near cached obstacles.
        if self.tf_buffer is not None and abs(filtered.angular.z) > 1e-6:
            nearest = _nearest_puck_distance(self.tf_buffer)
            if nearest is not None and nearest < self.puck_rotation_outer_m:
                if nearest <= self.puck_rotation_inner_m:
                    filtered.angular.z = 0.0
                else:
                    span = self.puck_rotation_outer_m - self.puck_rotation_inner_m
                    if span > 1e-6:
                        scale = (nearest - self.puck_rotation_inner_m) / span
                        filtered.angular.z *= _clamp(scale, 0.0, 1.0)
        return filtered

    def can_drive_forward(self):
        front = _scan_sector_min(mission_data.latest_scan, 0.0, self.sector_width_rad)
        if front is not None and front <= self.front_stop_m:
            return False
        # Floor-level pucks are invisible to the lidar; consult the cached
        # puck_memory (refreshed by PuckObstacleBroadcaster) so we don't
        # bump them off course while approaching another puck or zone.
        if self.tf_buffer is not None:
            puck = _puck_clearance_in_path(
                self.tf_buffer, self.sector_width_rad, self.puck_stop_m + 0.10
            )
            if puck is not None and puck <= self.puck_stop_m:
                return False
        return True

    def can_drive_backward(self):
        rear = _scan_sector_min(
            mission_data.latest_scan, math.pi, self.sector_width_rad
        )
        if rear is not None and rear <= self.rear_stop_m:
            return False
        if self.tf_buffer is not None:
            puck = _puck_clearance_in_path(
                self.tf_buffer, self.sector_width_rad, self.puck_stop_m + 0.10, math.pi
            )
            if puck is not None and puck <= self.puck_stop_m:
                return False
        return True

    def can_rotate(self):
        # In-place rotation is the safest recovery motion for this circular base.
        # Blocking it in corners can deadlock the mission before the robot can face open space.
        return True


class PuckObstacleBroadcaster:
    """Publishes /puck_obstacles as a PointCloud2 of every puck the perception
    layer knows about that the robot is *not* currently approaching or
    carrying. move_base picks this up as an extra observation source so it
    plans around floor-level pucks; the SafetyMonitor and the reactive servo
    consume the cached list directly via mission_data.latest_puck_obstacles.
    """

    def __init__(self, frame_id="map", rate_hz=5.0, height_m=0.10):
        self.frame_id = frame_id
        self.height_m = float(height_m)
        self.pub = rospy.Publisher("/puck_obstacles", PointCloud2, queue_size=1)
        period = max(1.0 / float(rate_hz), 0.05)
        self.timer = rospy.Timer(rospy.Duration(period), self._tick)

    def _selected_pucks(self):
        carrying = mission_data.carrying_color
        # While we're heading to grab a puck (carrying_color is None and
        # current_target_color is set), exclude that target colour so the
        # safety gate doesn't refuse the very approach we asked for.
        target = mission_data.current_target_color if not carrying else None
        out = []
        for color, entry in mission_data.puck_memory.items():
            if color == carrying:
                continue
            if color == target:
                continue
            point = _memory_point(entry)
            if point is None:
                continue
            out.append((float(point.x), float(point.y), self.height_m))
        return out

    def _tick(self, _event):
        pts = self._selected_pucks()
        # Cache flat (x, y) for the in-process consumers.
        mission_data.latest_puck_obstacles = [(x, y) for x, y, _ in pts]
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        cloud = pc2.create_cloud_xyz32(header, pts)
        try:
            self.pub.publish(cloud)
        except rospy.ROSException:
            pass


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
    # Reject map goals that would sit on top of a known *other* puck (already
    # excluded from puck_memory for the current approach target by the
    # broadcaster). Stops move_base and reactive servo from cutting corners
    # through a stationary puck the lidar cannot mark.
    puck_clear = float(rospy.get_param("~goal_puck_clearance_m", 0.28))
    for px, py in mission_data.latest_puck_obstacles:
        if math.hypot(goal["x"] - px, goal["y"] - py) < puck_clear:
            return False
    grid = mission_data.latest_map
    if grid is None or not grid.data:
        return True
    gx = int((goal["x"] - grid.info.origin.position.x) / grid.info.resolution)
    gy = int((goal["y"] - grid.info.origin.position.y) / grid.info.resolution)
    if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
        return False
    data = np.array(grid.data, dtype=np.int16).reshape(
        (grid.info.height, grid.info.width)
    )
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
    safety = safety or SafetyMonitor(tf_buffer)
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
        if (
            target_clearance is not None
            and target_clearance < safety.front_stop_m + 0.05
        ):
            best_angle = _scan_best_open_direction(
                scan, half_width=0.30, prefer_angle=target_heading_error
            )
            if best_angle is not None:
                heading_error = _angle_wrap(best_angle)

        cmd = Twist()
        cmd.angular.z = _clamp(angular_gain * heading_error, -max_angular, max_angular)
        if abs(heading_error) < 0.5:
            forward_clear = _scan_sector_min(scan, 0.0, 0.30)
            # Floor-level pucks are invisible to the lidar; fold the cached
            # puck obstacles into the same "is forward open" decision so the
            # reactive servo doesn't drive over them.
            puck_clear = _puck_clearance_in_path(
                tf_buffer, half_width_rad=0.30, max_range_m=1.0
            )
            blocked = (
                forward_clear is not None
                and forward_clear <= safety.front_stop_m + 0.04
            ) or (puck_clear is not None and puck_clear <= safety.puck_stop_m + 0.04)
            if not blocked:
                cmd.linear.x = linear_speed * max(0.25, 1.0 - abs(heading_error))

        if (
            cmd.linear.x > 0.0
            and last_distance is not None
            and distance >= last_distance - 0.01
        ):
            stagnant_cycles += 1
        else:
            stagnant_cycles = 0
        last_distance = distance
        if stagnant_cycles >= 120:
            cmd_pub.publish(Twist())
            publish_event(
                event_pub, f"[MISSION] Servo navigation to {label} made no progress."
            )
            return False

        publish_safe_twist(cmd_pub, cmd, safety, event_pub, label)
        if (rospy.Time.now() - start).to_sec() >= timeout_sec:
            cmd_pub.publish(Twist())
            publish_event(
                event_pub, f"[MISSION] Servo navigation to {label} timed out."
            )
            return False
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    cmd_pub.publish(Twist())
    return False


def servo_align_to_yaw(
    tf_buffer,
    cmd_pub,
    event_pub,
    target_yaw,
    timeout_sec,
    angular_gain,
    max_angular,
    label,
    safety,
    tolerance_rad,
):
    """Rotate in place to face an ArUco / map heading after a position-only servo."""
    publish_event(
        event_pub, f"[MISSION] Aligning camera heading for {label} (yaw toward tag)."
    )
    safety = safety or SafetyMonitor(tf_buffer)
    start = rospy.Time.now()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        _robot, yaw = get_robot_pose_yaw(tf_buffer)
        if _robot is None or yaw is None:
            return False
        err = _angle_wrap(float(target_yaw) - yaw)
        if abs(err) < tolerance_rad:
            cmd_pub.publish(Twist())
            return True
        if (rospy.Time.now() - start).to_sec() >= timeout_sec:
            cmd_pub.publish(Twist())
            publish_event(
                event_pub,
                f"[MISSION] Heading align for {label} timed out (yaw err ~{err:.2f} rad).",
            )
            return False
        cmd = Twist()
        cmd.angular.z = _clamp(angular_gain * err, -max_angular, max_angular)
        publish_safe_twist(cmd_pub, cmd, safety, event_pub, label)
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
        self.bbox_min_perimeter_cells = int(
            rospy.get_param("~bbox_min_perimeter_cells", 60)
        )
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
        data = np.array(grid.data, dtype=np.int16).reshape(
            (grid.info.height, grid.info.width)
        )
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
        return all(
            abs(prev_bbox[key] - new_bbox[key]) <= self.bbox_eps_m for key in keys
        )

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
                unknown_neighbors = np.count_nonzero(
                    data[y - 1 : y + 2, x - 1 : x + 2] < 0
                )
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
                    for nx, ny in (
                        (cx + 1, cy),
                        (cx - 1, cy),
                        (cx, cy + 1),
                        (cx, cy - 1),
                    ):
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
        distances = [
            math.hypot(point.x - corner.x, point.y - corner.y) for corner in corners
        ]
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
    safety = safety or SafetyMonitor(tf_buffer)
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
                if "yaw" in goal:
                    ytol = float(rospy.get_param("~tag_vantage_yaw_tolerance_rad", 0.2))
                    ytimeout = float(
                        rospy.get_param("~tag_vantage_yaw_align_timeout_sec", 10.0)
                    )
                    servo_align_to_yaw(
                        tf_buffer,
                        cmd_pub,
                        event_pub,
                        float(goal["yaw"]),
                        ytimeout,
                        angular_gain,
                        max_angular,
                        label,
                        safety,
                        ytol,
                    )
                return True, move_base_ready
        state = move_base.get_state()
        if state == 3:
            if "yaw" in goal:
                _, yaw_now = get_robot_pose_yaw(tf_buffer)
                if yaw_now is not None:
                    ytol = float(rospy.get_param("~tag_vantage_yaw_tolerance_rad", 0.2))
                    if abs(_angle_wrap(float(goal["yaw"]) - yaw_now)) > ytol:
                        ytimeout = float(
                            rospy.get_param("~tag_vantage_yaw_align_timeout_sec", 10.0)
                        )
                        servo_align_to_yaw(
                            tf_buffer,
                            cmd_pub,
                            event_pub,
                            float(goal["yaw"]),
                            ytimeout,
                            angular_gain,
                            max_angular,
                            label,
                            safety,
                            ytol,
                        )
            return True, move_base_ready
        if state in [4, 5, 9]:
            # 4=ABORTED, 5=REJECTED, 9=LOST. Surface this so the operator
            # doesn't have to guess why DELIVER kept silently bouncing back
            # to LOCATE_PUCK.
            state_names = {4: "ABORTED", 5: "REJECTED", 9: "LOST"}
            publish_event(
                event_pub,
                f"[MISSION] move_base {state_names.get(state, state)} navigation to "
                f"{label} at ({goal['x']:.2f}, {goal['y']:.2f}); falling back to servo.",
            )
            arrived = servo_to_map_point(
                tf_buffer,
                cmd_pub,
                event_pub,
                goal["x"],
                goal["y"],
                tolerance_m,
                max(timeout_sec * 0.5, 8.0),
                linear_speed,
                angular_gain,
                max_angular,
                label,
                safety,
            )
            if arrived and "yaw" in goal:
                ytol = float(rospy.get_param("~tag_vantage_yaw_tolerance_rad", 0.2))
                ytimeout = float(
                    rospy.get_param("~tag_vantage_yaw_align_timeout_sec", 10.0)
                )
                servo_align_to_yaw(
                    tf_buffer,
                    cmd_pub,
                    event_pub,
                    float(goal["yaw"]),
                    ytimeout,
                    angular_gain,
                    max_angular,
                    label,
                    safety,
                    ytol,
                )
            return arrived, move_base_ready
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
        self.safety = SafetyMonitor(self.tf_buffer)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.init_spin_yaw_speed = float(rospy.get_param("~init_spin_yaw_speed", 0.5))
        self.init_spin_duration_sec = float(
            rospy.get_param("~init_spin_duration_sec", 13.0)
        )
        self.init_max_attempts = int(rospy.get_param("~init_max_attempts", 3))
        self.init_nudge_distance_m = float(
            rospy.get_param("~init_nudge_distance_m", 0.4)
        )
        self.goal_tolerance_m = float(
            rospy.get_param("~explore_goal_tolerance_m", 0.18)
        )
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
                if (
                    self.analyzer.is_stable(previous_bbox, bbox)
                    or attempt == self.init_max_attempts
                ):
                    mission_data.arena = ArenaModel(bbox)
                    robot = get_robot_pose(self.tf_buffer)
                    mission_data.initial_robot_pose = robot
                    mission_data.arena.set_start_corner(robot)
                    publish_event(
                        self.event_pub, "[MISSION] SLAM arena bounds initialized."
                    )
                    return "scan_done"
                previous_bbox = bbox
            publish_event(
                self.event_pub,
                "[MISSION] Arena bounds not stable yet; continuing rotation-only scan.",
            )

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
            publish_safe_twist(
                self.cmd_pub, cmd, self.safety, self.event_pub, "init scan"
            )
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
        smach.State.__init__(
            self, outcomes=["tags_complete", "tags_missing", "need_scan"]
        )
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
                publish_event(
                    self.event_pub, "[MISSION] Arena bounds unavailable; rescanning."
                )
                return "need_scan"
            mission_data.arena = ArenaModel(bbox)
            mission_data.arena.set_start_corner(get_robot_pose(self.tf_buffer))

        for color, entry in list(mission_data.drop_zone_memory.items()):
            point = _memory_point(entry)
            if point is None:
                continue
            if mission_data.arena.assign_color(
                color, point, self.tag_corner_snap_distance_m
            ):
                corner_idx = mission_data.arena.color_to_corner[color]
                mission_data.inspected_tag_corners.add(corner_idx)
                publish_event(
                    self.event_pub,
                    f"[MISSION] Mapped {color} tag to arena corner {corner_idx}.",
                )

        known = len(
            [
                color
                for color in mission_data.target_order
                if color in mission_data.arena.color_to_corner
            ]
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
        self.safety = SafetyMonitor(self.tf_buffer)
        self.corner_vantage_inset_m = float(
            rospy.get_param("~corner_vantage_inset_m", 0.55)
        )
        # Make corner vantages scale with the discovered arena size. A fixed
        # metric inset can still hug walls when SLAM's early bbox is oversized.
        self.corner_vantage_center_fraction = float(
            rospy.get_param("~corner_vantage_center_fraction", 0.65)
        )
        self.tag_vantage_wall_margin_m = float(
            rospy.get_param("~tag_vantage_wall_margin_m", 0.45)
        )
        self.tag_tour_use_move_base = bool(
            rospy.get_param("~tag_tour_use_move_base", False)
        )
        self.tag_tour_goal_timeout_sec = float(
            rospy.get_param("~tag_tour_goal_timeout_sec", 25.0)
        )
        self.tag_dwell_sec = float(rospy.get_param("~tag_dwell_sec", 1.5))
        self.goal_tolerance_m = float(
            rospy.get_param("~explore_goal_tolerance_m", 0.18)
        )
        self.servo_goal_linear_speed = float(
            rospy.get_param("~servo_goal_linear_speed", 0.08)
        )
        self.servo_goal_angular_gain = float(
            rospy.get_param("~servo_goal_angular_gain", 1.2)
        )
        self.servo_goal_max_angular = float(
            rospy.get_param("~servo_goal_max_angular", 0.65)
        )

    def _corner_goal(self, arena, corner_idx):
        corner = arena.corners()[corner_idx]
        center = arena.center()
        corner_to_center = max(
            math.hypot(center.x - corner.x, center.y - corner.y), 1e-3
        )
        adaptive_inset = max(
            self.corner_vantage_inset_m,
            self.corner_vantage_center_fraction * corner_to_center,
        )
        adaptive_inset = min(adaptive_inset, max(0.10, corner_to_center - 0.05))
        goal = arena.vantage(corner_idx, adaptive_inset)

        margin = max(0.0, self.tag_vantage_wall_margin_m)
        x_lo = arena.bbox["min_x"] + margin
        x_hi = arena.bbox["max_x"] - margin
        y_lo = arena.bbox["min_y"] + margin
        y_hi = arena.bbox["max_y"] - margin
        if x_lo <= x_hi:
            goal["x"] = _clamp(goal["x"], x_lo, x_hi)
        if y_lo <= y_hi:
            goal["y"] = _clamp(goal["y"], y_lo, y_hi)
        goal["yaw"] = math.atan2(corner.y - goal["y"], corner.x - goal["x"])
        return goal, adaptive_inset

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: TAG_TOUR")
        arena = mission_data.arena
        if arena is None:
            return "tour_progress"
        target_corners = [
            idx
            for idx in arena.unmapped_corners()
            if idx not in mission_data.inspected_tag_corners
        ]
        if not target_corners:
            mission_data.inspected_tag_corners.clear()
            target_corners = arena.unmapped_corners()
        goals = []
        for idx in target_corners:
            goal, inset = self._corner_goal(arena, idx)
            goals.append((idx, goal, inset))
        robot = get_robot_pose(self.tf_buffer)
        if robot is not None:
            goals.sort(
                key=lambda item: math.hypot(
                    item[1]["x"] - robot.x, item[1]["y"] - robot.y
                )
            )

        for corner_idx, goal, inset in goals:
            if corner_idx not in arena.unmapped_corners():
                continue
            nav_mode = "move_base" if self.tag_tour_use_move_base else "servo"
            publish_event(
                self.event_pub,
                f"[MISSION] Visiting corner {corner_idx} to read tag "
                f"(mode={nav_mode}, inset={inset:.2f} m, "
                f"goal=({goal['x']:.2f}, {goal['y']:.2f})).",
            )
            if self.tag_tour_use_move_base:
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
            else:
                _arrived = servo_to_map_point(
                    self.tf_buffer,
                    self.cmd_pub,
                    self.event_pub,
                    goal["x"],
                    goal["y"],
                    self.goal_tolerance_m,
                    self.tag_tour_goal_timeout_sec,
                    self.servo_goal_linear_speed,
                    self.servo_goal_angular_gain,
                    self.servo_goal_max_angular,
                    f"corner {corner_idx} tag vantage",
                    self.safety,
                )
                if _arrived:
                    ytol = float(rospy.get_param("~tag_vantage_yaw_tolerance_rad", 0.2))
                    ytimeout = float(
                        rospy.get_param("~tag_vantage_yaw_align_timeout_sec", 10.0)
                    )
                    servo_align_to_yaw(
                        self.tf_buffer,
                        self.cmd_pub,
                        self.event_pub,
                        float(goal["yaw"]),
                        ytimeout,
                        self.servo_goal_angular_gain,
                        self.servo_goal_max_angular,
                        f"corner {corner_idx} tag vantage",
                        self.safety,
                        ytol,
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
        smach.State.__init__(
            self, outcomes=["found_puck", "continue_search", "all_done"]
        )
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_ready = self.move_base.wait_for_server(rospy.Duration(1.0))
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.safety = SafetyMonitor(self.tf_buffer)
        self.known_object_stale_sec = float(
            rospy.get_param("~known_object_stale_sec", 0.0)
        )
        self.puck_search_quadrant_inset_m = float(
            rospy.get_param("~puck_search_quadrant_inset_m", 0.45)
        )
        self.locate_goal_timeout_sec = float(
            rospy.get_param("~locate_goal_timeout_sec", 20.0)
        )
        self.goal_tolerance_m = float(
            rospy.get_param("~explore_goal_tolerance_m", 0.18)
        )
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
            return (
                "found_puck" if self._target_is_known(next_color) else "continue_search"
            )

        goals = [
            (idx, arena.quadrant_goal(idx, self.puck_search_quadrant_inset_m))
            for idx in range(4)
        ]
        robot = get_robot_pose(self.tf_buffer)
        if robot is not None:
            goals.sort(
                key=lambda item: math.hypot(
                    item[1]["x"] - robot.x, item[1]["y"] - robot.y
                )
            )
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
            publish_safe_twist(
                self.cmd_pub, cmd, self.safety, self.event_pub, "puck search scan"
            )
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
        self.safety = SafetyMonitor(self.tf_buffer)
        self.approach_distance_m = float(rospy.get_param("~approach_distance_m", 0.30))
        self.goal_tolerance_m = float(
            rospy.get_param("~approach_goal_tolerance_m", 0.18)
        )
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
    """Final close-range alignment + creep-up before calling the gripper.

    The previous implementation aborted whenever a 40x40 depth patch had
    variance below 50 (in metres squared!) which is practically always
    true for a small cylindrical puck against a flat floor / wall, so the
    state could never make progress. This version is a closed-loop
    controller driven by the perception-published map-frame puck pose:
        1. Look up the latest puck position from puck_memory and the
           robot pose from TF.
        2. Rotate to face the puck if the heading error is large.
        3. Otherwise creep forward through the SafetyMonitor until the
           planar distance to the puck drops below the grasp threshold.
        4. Bail out cleanly on timeout or when the puck disappears.
    The depth image is no longer used as a wall detector; the safety
    monitor's lidar-based front clearance is the authoritative obstacle
    check.
    """

    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(
            self, outcomes=["ready_to_grab", "wall_detected", "timeout"]
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.event_pub = event_pub
        self.tf_buffer = tf_buffer
        self.safety = SafetyMonitor(self.tf_buffer)

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: VISUAL_SERVO")

        color = mission_data.current_target_color
        if not color:
            return "wall_detected"

        max_duration = rospy.Duration(
            float(rospy.get_param("~visual_servo_timeout_sec", 12.0))
        )
        grasp_distance = float(rospy.get_param("~visual_servo_grasp_distance_m", 0.18))
        align_tolerance = float(
            rospy.get_param("~visual_servo_align_tolerance_rad", 0.12)
        )
        align_gain = float(rospy.get_param("~visual_servo_align_gain", 1.4))
        max_angular = float(rospy.get_param("~visual_servo_max_angular", 0.7))
        forward_speed = float(rospy.get_param("~visual_servo_speed", 0.05))
        blocked_grace_sec = float(
            rospy.get_param("~visual_servo_blocked_grace_sec", 1.5)
        )

        # Snapshot the puck pose once at entry. The depth camera is mounted
        # high on the rosbot and tilts forward; a small ground puck typically
        # falls below the vertical FOV before grasp distance, so perception
        # can't refresh the memory in this final approach. We trust the pose
        # that Approach already navigated to and keep refreshing only when a
        # newer detection comes in.
        entry = mission_data.puck_memory.get(color)
        if entry is None:
            publish_event(
                self.event_pub,
                f"[MISSION] Visual servo has no {color} puck pose; aborting.",
            )
            return "timeout"
        target_point = _memory_point(entry)
        if target_point is None:
            return "timeout"
        last_used_stamp = _memory_stamp(entry)

        rate = rospy.Rate(10)
        start = rospy.Time.now()
        last_progress_time = start
        last_distance = None
        blocked_since = None

        while not rospy.is_shutdown():
            if rospy.Time.now() - start > max_duration:
                self.cmd_pub.publish(Twist())
                publish_event(
                    self.event_pub,
                    f"[MISSION] Visual servo timed out before grasp ({color}).",
                )
                return "timeout"

            # Opportunistically refresh the target if perception republished a
            # newer pose (e.g. while still aligning at standoff range).
            entry = mission_data.puck_memory.get(color)
            if entry is not None:
                stamp = _memory_stamp(entry)
                if (
                    stamp is not None
                    and last_used_stamp is not None
                    and stamp > last_used_stamp
                ):
                    new_point = _memory_point(entry)
                    if new_point is not None:
                        target_point = new_point
                        last_used_stamp = stamp

            puck_point = target_point

            robot, yaw = get_robot_pose_yaw(self.tf_buffer)
            if robot is None or yaw is None:
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue

            dx = puck_point.x - robot.x
            dy = puck_point.y - robot.y
            distance = math.hypot(dx, dy)

            if distance <= grasp_distance:
                self.cmd_pub.publish(Twist())
                publish_event(
                    self.event_pub,
                    f"[MISSION] Visual servo at grasp distance ({distance:.2f} m).",
                )
                return "ready_to_grab"

            if last_distance is None or distance < last_distance - 0.01:
                last_distance = distance
                last_progress_time = rospy.Time.now()
            elif (rospy.Time.now() - last_progress_time).to_sec() > 5.0:
                self.cmd_pub.publish(Twist())
                publish_event(
                    self.event_pub,
                    f"[MISSION] Visual servo not converging on {color} puck.",
                )
                return "timeout"

            heading_error = _angle_wrap(math.atan2(dy, dx) - yaw)

            cmd = Twist()
            if abs(heading_error) > align_tolerance:
                cmd.angular.z = _clamp(
                    align_gain * heading_error, -max_angular, max_angular
                )
            else:
                cmd.linear.x = forward_speed
                cmd.angular.z = _clamp(0.6 * heading_error, -max_angular, max_angular)

            moved = publish_safe_twist(
                self.cmd_pub, cmd, self.safety, self.event_pub, "visual servo"
            )
            if not moved and cmd.linear.x > 0.0 and abs(cmd.angular.z) < 1e-3:
                # Forward motion gated by lidar (likely a real obstacle in
                # front). Allow a short grace period so a transient laser
                # spike doesn't kill the approach.
                if blocked_since is None:
                    blocked_since = rospy.Time.now()
                elif (rospy.Time.now() - blocked_since).to_sec() > blocked_grace_sec:
                    self.cmd_pub.publish(Twist())
                    publish_event(
                        self.event_pub,
                        "[MISSION] Visual servo blocked by lidar safety gate.",
                    )
                    return "wall_detected"
            else:
                blocked_since = None

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

        self.cmd_pub.publish(Twist())
        return "timeout"


class Grab(smach.State):
    def __init__(self, event_pub, tf_buffer=None):
        smach.State.__init__(self, outcomes=["grabbed", "missed"])
        self.grasp_srv = rospy.ServiceProxy("/grasp_puck", GraspPuck)
        self.event_pub = event_pub
        self.tf_buffer = tf_buffer
        self.safety = SafetyMonitor(self.tf_buffer)

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: GRAB")
        color = mission_data.current_target_color
        try:
            resp = self.grasp_srv(color)
            if resp.success:
                publish_event(
                    self.event_pub,
                    f"[MISSION] Grabbed {color} puck successfully!",
                )
                # Mark this colour as carried so perception won't overwrite
                # the original puck pose with the (now meaningless) detection
                # of the held puck floating above the robot. Drop the stale
                # ground pose too so any fallback path doesn't re-use it.
                mission_data.carrying_color = color
                mission_data.puck_memory.pop(color, None)
                # Drop any cached marks of the carried puck so the costmap
                # only reflects the remaining obstacle pucks.
                clear_move_base_costmaps()
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
            if not publish_safe_twist(
                cmd_pub, cmd, self.safety, self.event_pub, "miss backup"
            ):
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
        self.safety = SafetyMonitor(self.tf_buffer)
        # The 0.25 m drop-zone marker sits at lidar height, so the standoff has
        # to clear robot_radius (~0.11) + inflation (~0.18) + the marker half
        # width (~0.13) = ~0.42 m before move_base will plan a path. Default a
        # bit further out and let retries push us closer if needed.
        self.dropoff_distance_m = float(rospy.get_param("~dropoff_distance_m", 0.55))
        self.goal_tolerance_m = float(
            rospy.get_param("~deliver_goal_tolerance_m", 0.25)
        )
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
        max_retries = int(rospy.get_param("~deliver_retry_limit", 3))
        retry_insets = [
            self.dropoff_distance_m,
            self.dropoff_distance_m + 0.10,
            self.dropoff_distance_m + 0.20,
        ]
        per_attempt_timeout = float(
            rospy.get_param("~deliver_attempt_timeout_sec", 35.0)
        )

        for attempt in range(max_retries):
            inset = retry_insets[min(attempt, len(retry_insets) - 1)]
            goal_dict = arena.vantage(corner_idx, inset)
            # First attempt asks move_base for a planned path. Subsequent
            # attempts skip straight to the reactive servo because experience
            # shows move_base burns ~90 s in rotate-recovery near the
            # drop-zone markers before aborting, which makes the robot look
            # frozen. Servo navigation has lidar-based obstacle avoidance and
            # is what reliably gets us in for the release.
            use_move_base = attempt == 0
            publish_event(
                self.event_pub,
                f"[MISSION] Delivering {color} puck to ArUco zone "
                f"(attempt {attempt + 1}/{max_retries}, inset {inset:.2f} m, "
                f"goal=({goal_dict['x']:.2f}, {goal_dict['y']:.2f}), "
                f"mode={'move_base' if use_move_base else 'servo'})...",
            )
            if use_move_base:
                arrived, self.move_base_ready = navigate_to_goal(
                    self.move_base,
                    self.move_base_ready,
                    self.tf_buffer,
                    self.cmd_pub,
                    self.event_pub,
                    goal_dict,
                    per_attempt_timeout,
                    self.goal_tolerance_m,
                    self.servo_goal_linear_speed,
                    self.servo_goal_angular_gain,
                    self.servo_goal_max_angular,
                    f"{color} drop zone",
                    self.safety,
                )
            else:
                arrived = servo_to_map_point(
                    self.tf_buffer,
                    self.cmd_pub,
                    self.event_pub,
                    goal_dict["x"],
                    goal_dict["y"],
                    self.goal_tolerance_m,
                    per_attempt_timeout,
                    self.servo_goal_linear_speed,
                    self.servo_goal_angular_gain,
                    self.servo_goal_max_angular,
                    f"{color} drop zone",
                    self.safety,
                )
            if arrived:
                publish_event(
                    self.event_pub,
                    f"[MISSION] Arrived at {color} drop zone. Releasing.",
                )
                self.release_srv()
                mission_data.completed_colors.add(color)
                mission_data.target_index += 1
                mission_data.puck_memory.pop(color, None)
                mission_data.carrying_color = None
                mission_data.current_target_color = None
                self._backup_after_release()
                # The released puck (now stationary on the floor) becomes
                # an obstacle for the next leg's planning. Reset costmaps
                # so the obstacle layer immediately reflects only the still-
                # remaining pucks via the next /puck_obstacles cloud.
                clear_move_base_costmaps()
                return "delivered"
            publish_event(
                self.event_pub,
                f"[MISSION] Delivery attempt {attempt + 1} for {color} failed; "
                "retrying with relaxed standoff.",
            )

        # Exhausted all retries. Release the puck in place and ADVANCE to the
        # next colour so we don't keep re-targeting this one forever.
        publish_event(
            self.event_pub,
            f"[MISSION] All delivery attempts to {color} drop zone failed; "
            "releasing puck where the robot stands and skipping this colour.",
        )
        try:
            self.release_srv()
        except Exception as exc:
            rospy.logwarn("[MISSION] Release service failed: %s", str(exc))
        mission_data.carrying_color = None
        mission_data.puck_memory.pop(color, None)
        # Skip this colour permanently: mark completed AND bump the index so
        # _next_target_color moves on.
        mission_data.completed_colors.add(color)
        mission_data.target_index += 1
        mission_data.current_target_color = None
        self._backup_after_release()
        clear_move_base_costmaps()
        return "failed"

    def _backup_after_release(self):
        cmd = Twist()
        cmd.linear.x = -float(rospy.get_param("~backup_speed", 0.08))
        for _ in range(20):
            if not publish_safe_twist(
                self.cmd_pub, cmd, self.safety, self.event_pub, "release backup"
            ):
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
        # While the robot is carrying a puck of this colour, the camera sees
        # the held puck right above the base_link and reports its map-frame
        # position ~= robot pose. Ignoring those updates keeps the original
        # ground-truth pickup pose from being clobbered with garbage.
        if mission_data.carrying_color == msg.color:
            return
        mission_data.puck_memory[msg.color] = entry
    elif msg.object_class == "drop_zone" and msg.color:
        mission_data.drop_zone_memory[msg.color] = entry


def _on_map(msg):
    mission_data.latest_map = msg


def _on_scan(msg):
    mission_data.latest_scan = msg


def _wait_for_move_base(timeout_sec):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo(
        "[MISSION] Waiting up to %.1fs for move_base action server...", timeout_sec
    )
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    while not rospy.is_shutdown():
        if client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("[MISSION] move_base action server is up.")
            return True
        if rospy.Time.now() >= deadline:
            rospy.logwarn(
                "[MISSION] move_base did not come up in time; falling back to reactive servo."
            )
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
    if (
        mission_data.latest_scan is not None
        and mission_data.latest_scan.header.frame_id
    ):
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
                laser_frame,
                yaw,
                math.degrees(yaw),
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

    mission_data.target_order = rospy.get_param(
        "~target_order", mission_data.target_order
    )

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

    # Stream the floor-level pucks into move_base's costmap and the in-process
    # safety gate. Lidar at z~0.12 m never sees the 3 cm tall pucks, so without
    # this the robot would happily plan straight through them.
    PuckObstacleBroadcaster(rate_hz=5.0)

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
            VisualServo(tf_buffer, event_pub),
            transitions={
                "ready_to_grab": "GRAB",
                "wall_detected": "LOCATE_PUCK",
                "timeout": "LOCATE_PUCK",
            },
        )
        smach.StateMachine.add(
            "GRAB",
            Grab(event_pub, tf_buffer),
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
