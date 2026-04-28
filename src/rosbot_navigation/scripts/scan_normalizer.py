#!/usr/bin/env python3
"""Normalize ``sensor_msgs/LaserScan`` so consumers see a stable ray count.

slam_toolbox / Karto lock onto the number of range readings from the very
first ``LaserScan`` they see and emit
``LaserRangeScan contains <N> range readings, expected <M>`` whenever a later
message disagrees. On real lidars (and some Gazebo plugins) ``angle_min``,
``angle_max`` and ``angle_increment`` occasionally drift by floating-point
rounding so ``len(ranges)`` flips between, e.g., 1946 and 1947 beams.

This relay subscribes to the lidar's ``/scan``, locks in the expected beam
count from the first message, then forces every subsequent message to that
exact length by truncating or padding (with ``range_min``) ``ranges`` and
``intensities``. ``angle_max`` is recomputed from
``angle_min + (n - 1) * angle_increment`` to stay self-consistent.

The default topology is ``/scan`` -> ``/scan_normalized`` so SLAM can be
pointed at the normalized topic without touching the lidar driver. Other
consumers (move_base costmap obstacle layer, ``laser_nav_safety``,
``mission_controller``) iterate scan ranges by index and tolerate variable
beam counts, so they keep reading ``/scan`` directly.
"""

import math

import rospy
from sensor_msgs.msg import LaserScan


class ScanNormalizer:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_normalized")
        self.expected_count = int(rospy.get_param("~expected_count", 0))
        self.fix_count = bool(rospy.get_param("~fix_count", True))
        self.warn_throttle_sec = float(rospy.get_param("~warn_throttle_sec", 5.0))

        self._locked_count = self.expected_count if self.expected_count > 0 else None
        self._locked_increment = None
        self._published_count_lock = False

        self._pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=5)
        rospy.Subscriber(self.input_topic, LaserScan, self._on_scan, queue_size=5)
        rospy.loginfo(
            "[scan_normalizer] %s -> %s (fix_count=%s, expected_count=%s)",
            self.input_topic,
            self.output_topic,
            self.fix_count,
            self._locked_count if self._locked_count is not None else "auto",
        )

    def _on_scan(self, msg):
        if not self.fix_count:
            self._pub.publish(msg)
            return

        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            self._pub.publish(msg)
            return

        if self._locked_count is None:
            self._locked_count = n
            self._locked_increment = msg.angle_increment
            rospy.loginfo(
                "[scan_normalizer] locked beam count = %d (angle_increment=%.6f)",
                n,
                msg.angle_increment,
            )

        if n == self._locked_count:
            self._pub.publish(msg)
            return

        rospy.logwarn_throttle(
            self.warn_throttle_sec,
            "[scan_normalizer] incoming scan has %d rays, expected %d; normalizing.",
            n,
            self._locked_count,
        )

        target = self._locked_count
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.angle_max = msg.angle_min + (target - 1) * msg.angle_increment

        if n > target:
            out.ranges = list(msg.ranges[:target])
            if msg.intensities:
                out.intensities = list(msg.intensities[:target])
        else:
            pad = target - n
            invalid = msg.range_min if msg.range_min > 0.0 else 0.0
            out.ranges = list(msg.ranges) + [invalid] * pad
            if msg.intensities:
                out.intensities = list(msg.intensities) + [0.0] * pad

        # Defensive: replace NaN/inf with range_min so Karto doesn't choke either.
        cleaned = []
        for r in out.ranges:
            if not math.isfinite(r):
                cleaned.append(0.0)
            else:
                cleaned.append(r)
        out.ranges = cleaned

        self._pub.publish(out)


def main():
    rospy.init_node("scan_normalizer")
    ScanNormalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
