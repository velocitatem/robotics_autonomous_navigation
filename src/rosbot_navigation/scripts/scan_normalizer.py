#!/usr/bin/env python3
"""Normalize ``sensor_msgs/LaserScan`` so slam_toolbox / Karto stop rejecting it.

slam_toolbox embeds karto_sdk, which on every scan validates that
``len(ranges)`` equals the value Karto stored at sensor registration:

.. code-block:: cpp

    m_NumberOfRangeReadings =
        static_cast<kt_int32u>(
            math::Round((angle_max - angle_min) / angle_increment) + residual);

where ``residual = 0`` when the scan is detected as a 360 lidar (the angular
span is roughly :math:`2\\pi`) and ``residual = 1`` otherwise. Many drivers
(RPLIDAR, certain Slamtec firmwares, custom lidars) publish
``angle_increment = (angle_max - angle_min) / (N - 1)`` while the message
contains ``N`` rays, so Karto keeps complaining
``LaserRangeScan contains N range readings, expected N - 1`` for every scan
and refuses to build a map.

This node replicates Karto's formula on the first scan, decides the count
slam_toolbox is going to demand, and then trims (or pads) every subsequent
scan to that count. The metadata fields (``angle_min``, ``angle_increment``,
``angle_max``) are left untouched, so Karto computes the same expected count
on every message and our trimmed payload matches it exactly.

Default topology: ``/scan`` -> ``/scan_normalized``. ``slam.launch`` is
pointed at ``/scan_normalized`` automatically when ``enable_scan_normalizer``
is true. Other consumers (move_base obstacle layer, safety nodes, mission
controller) keep reading ``/scan`` because they iterate ranges by index and
tolerate variable beam counts.
"""

import math

import rospy
from sensor_msgs.msg import LaserScan


_TWO_PI = 2.0 * math.pi


def karto_expected_count(msg):
    """Replicate slam_toolbox karto_sdk's ``LaserRangeFinder`` ray count.

    Returns the integer number of range readings ``slam_toolbox`` will
    expect for ``msg`` once it registers it as a sensor. Returns ``None``
    if the message metadata is unusable.
    """
    inc = msg.angle_increment
    if inc == 0.0:
        return None
    delta = msg.angle_max - msg.angle_min
    # Karto detects a 360-degree lidar when delta is close to 2*pi (older
    # versions) or when delta + inc is close to 2*pi (post slam_toolbox#288).
    # In either case residual = 0; otherwise residual = 1. Use a tolerance
    # generous enough to cover both definitions and FP slack.
    tol = 1.5 * abs(inc)
    if abs(delta - _TWO_PI) <= tol or abs(delta + inc - _TWO_PI) <= tol:
        residual = 0
    else:
        residual = 1
    return int(round(delta / inc)) + residual


class ScanNormalizer:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_normalized")
        self.expected_count = int(rospy.get_param("~expected_count", 0))
        self.fix_count = bool(rospy.get_param("~fix_count", True))
        self.warn_throttle_sec = float(rospy.get_param("~warn_throttle_sec", 5.0))

        self._locked_count = (
            self.expected_count if self.expected_count > 0 else None
        )

        self._pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=5)
        rospy.Subscriber(
            self.input_topic, LaserScan, self._on_scan, queue_size=5
        )
        rospy.loginfo(
            "[scan_normalizer] %s -> %s (fix_count=%s, expected_count=%s)",
            self.input_topic,
            self.output_topic,
            self.fix_count,
            self._locked_count if self._locked_count is not None else "auto",
        )

    def _lock_count(self, msg):
        n = len(msg.ranges)
        karto_n = karto_expected_count(msg)
        if karto_n is None or karto_n <= 0:
            target = n
            note = "no usable angle metadata; locking to ranges length"
        elif karto_n == n:
            target = n
            note = "lidar count matches karto formula"
        elif abs(karto_n - n) <= max(4, n // 100):
            # Off-by-one (or small drift) between driver count and karto's
            # formula. Lock to karto's count so the in-process consistency
            # check passes. Other consumers iterate by index and tolerate
            # losing one ray at the angle_max edge.
            target = karto_n
            note = (
                "lidar publishes %d but slam_toolbox will expect %d; "
                "trimming to %d to keep karto consistency check happy"
            ) % (n, karto_n, karto_n)
        else:
            target = n
            note = (
                "karto formula gives %d but lidar publishes %d (gap too "
                "large); locking to ranges length and hoping karto agrees"
            ) % (karto_n, n)
        self._locked_count = target
        rospy.loginfo(
            "[scan_normalizer] locked beam count = %d (angle_increment=%.6f, %s).",
            target,
            msg.angle_increment,
            note,
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
            self._lock_count(msg)

        target = self._locked_count

        if n != target:
            rospy.logwarn_throttle(
                self.warn_throttle_sec,
                "[scan_normalizer] incoming scan has %d rays, normalizing to %d.",
                n,
                target,
            )

        # Always rebuild output. We must replace ranges/intensities in-place
        # rather than passing msg through, because we may need to trim or
        # pad to match karto's expected count. Metadata (angle_min,
        # angle_max, angle_increment) is preserved unchanged so karto still
        # computes the same expected count on every message.
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        if n == target:
            ranges_out = list(msg.ranges)
            intens_out = list(msg.intensities) if msg.intensities else []
        elif n > target:
            ranges_out = list(msg.ranges[:target])
            intens_out = (
                list(msg.intensities[:target]) if msg.intensities else []
            )
        else:
            pad = target - n
            invalid = msg.range_min if msg.range_min > 0.0 else 0.0
            ranges_out = list(msg.ranges) + [invalid] * pad
            intens_out = (
                list(msg.intensities) + [0.0] * pad
                if msg.intensities
                else []
            )

        # Replace NaN/inf with 0.0; karto rejects non-finite range values.
        out.ranges = [r if math.isfinite(r) else 0.0 for r in ranges_out]
        out.intensities = intens_out
        self._pub.publish(out)


def main():
    rospy.init_node("scan_normalizer")
    ScanNormalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
