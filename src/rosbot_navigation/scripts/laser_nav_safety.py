#!/usr/bin/env python3
"""Filter move_base /cmd_vel before twist_mux: laser-based stop zones + speed caps.

move_base + costmaps plan around walls, but the planner can still command motion
while the local map lags or in tight clearances. This node mirrors the
mission SafetyMonitor gating so *navigation* always respects forward/rear
lidar clearance, with base_link-consistent sector math via TF (laser yaw).
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _angle_wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _scan_sector_min(
    scan, center_angle_base, half_width, laser_yaw_offset
):
    if scan is None or not scan.ranges or scan.angle_increment == 0.0:
        return None
    values = []
    for idx, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance <= scan.range_min:
            continue
        angle_base = (
            scan.angle_min + idx * scan.angle_increment + laser_yaw_offset
        )
        if abs(_angle_wrap(angle_base - center_angle_base)) <= half_width:
            values.append(distance)
    if not values:
        return None
    return min(values)


class LaserNavSafety:
    def __init__(self):
        self.enabled = bool(rospy.get_param("~enabled", True))
        in_topic = rospy.get_param("~input_topic", "/cmd_vel_nav_unsafe")
        out_topic = rospy.get_param("~output_topic", "/cmd_vel_nav")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        # Wall-clock budget for "is the scan fresh enough?". This is measured
        # against the receive time, not the message header, so cross-host
        # clock skew (e.g. robot publishing /scan with stamps that lag the
        # workstation clock by several seconds because NTP isn't running)
        # does not cause every cmd_vel to be zeroed and the local planner
        # to oscillate-abort. The header stamp is still used as a sanity
        # fallback when its absolute drift is small.
        self.scan_max_age = float(rospy.get_param("~scan_max_age_sec", 1.5))
        # If the gap between header.stamp and rospy.Time.now() exceeds this,
        # we assume cross-host clock skew and trust receive time only.
        self.scan_stamp_skew_tolerance = float(
            rospy.get_param("~scan_stamp_skew_tolerance_sec", 0.5)
        )

        self.front_stop_m = float(rospy.get_param("~front_stop_m", 0.32))
        self.rear_stop_m = float(rospy.get_param("~rear_stop_m", 0.24))
        self.sector_width_rad = float(
            rospy.get_param("~safety_sector_width_rad", 0.45)
        )
        self.max_safe_linear = float(
            rospy.get_param("~max_safe_linear_speed", 0.12)
        )
        self.max_safe_angular = float(
            rospy.get_param("~max_safe_angular_speed", 0.60)
        )

        self._lock = threading.Lock()
        self._scan = None
        self._scan_stamp = None
        self._scan_received_at = None
        self.laser_yaw_offset = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._laser_frame = rospy.get_param("~laser_frame", "laser")

        self._pub = rospy.Publisher(out_topic, Twist, queue_size=1)
        rospy.Subscriber(in_topic, Twist, self._on_cmd, queue_size=1)
        rospy.Subscriber(
            self.scan_topic, LaserScan, self._on_scan, queue_size=1
        )
        rospy.loginfo(
            "laser_nav_safety: enabled=%s, %s -> %s",
            self.enabled,
            in_topic,
            out_topic,
        )

    def _on_scan(self, msg):
        with self._lock:
            if msg.header and msg.header.frame_id:
                self._laser_frame = msg.header.frame_id
            self._scan = msg
            self._scan_stamp = msg.header.stamp
            self._scan_received_at = rospy.Time.now()

    def _calibrate_laser(self):
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    self._laser_frame,
                    rospy.Time(0),
                    rospy.Duration(0.5),
                )
                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                self.laser_yaw_offset = yaw
                rospy.loginfo(
                    "laser_nav_safety: base_link -> %s yaw = %.3f rad (%.1f deg).",
                    self._laser_frame,
                    yaw,
                    math.degrees(yaw),
                )
                return
            except Exception:
                try:
                    rospy.sleep(0.2)
                except rospy.ROSInterruptException:
                    return
        rospy.logwarn(
            "laser_nav_safety: could not get base_link -> %s; using 0 yaw offset.",
            self._laser_frame,
        )
        self.laser_yaw_offset = 0.0

    def _scan_fresh(self):
        with self._lock:
            if self._scan is None or self._scan_received_at is None:
                return False, None
            now = rospy.Time.now()
            # Primary freshness check: how long since we *received* the scan.
            # This is robust to cross-host clock skew because the receive
            # time is always wall-clock-local to this node.
            recv_age = (now - self._scan_received_at).to_sec()
            if recv_age > self.scan_max_age:
                return False, None
            # Secondary sanity check on the stamp, but only when the stamp
            # is plausibly synced to local time. If the stamp appears wildly
            # ahead/behind (which means the publishing host's clock is off,
            # not that the scan is stale), ignore it instead of declaring
            # the scan stale; otherwise every cmd_vel gets gagged and the
            # local planner oscillates and aborts.
            if self._scan_stamp is not None:
                stamp_drift = abs((now - self._scan_stamp).to_sec())
                if stamp_drift <= self.scan_stamp_skew_tolerance:
                    if stamp_drift > self.scan_max_age:
                        return False, None
            return True, self._scan

    def _on_cmd(self, msg):
        if not self.enabled:
            self._pub.publish(msg)
            return

        ok, scan = self._scan_fresh()
        if not ok:
            rospy.logwarn_throttle(
                2.0,
                "laser_nav_safety: stale or missing /scan; holding base.",
            )
            self._pub.publish(Twist())
            return

        out = Twist()
        out.linear.x = _clamp(
            msg.linear.x, -self.max_safe_linear, self.max_safe_linear
        )
        out.angular.z = _clamp(
            msg.angular.z,
            -self.max_safe_angular,
            self.max_safe_angular,
        )

        if out.linear.x > 0.0:
            front = _scan_sector_min(
                scan, 0.0, self.sector_width_rad, self.laser_yaw_offset
            )
            if front is not None and front <= self.front_stop_m:
                out.linear.x = 0.0
        if out.linear.x < 0.0:
            rear = _scan_sector_min(
                scan,
                math.pi,
                self.sector_width_rad,
                self.laser_yaw_offset,
            )
            if rear is not None and rear <= self.rear_stop_m:
                out.linear.x = 0.0

        self._pub.publish(out)


def main():
    rospy.init_node("laser_nav_safety")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    node = LaserNavSafety()
    try:
        rospy.wait_for_message(scan_topic, LaserScan, timeout=30.0)
    except rospy.ROSException:
        rospy.logwarn("laser_nav_safety: no scan yet; continuing with 0 offset.")
    node._calibrate_laser()
    rospy.spin()


if __name__ == "__main__":
    main()
