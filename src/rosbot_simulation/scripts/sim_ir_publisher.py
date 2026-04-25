#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import LaserScan, Range


class SimIRPublisher:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.range_topic = rospy.get_param("~range_topic", "/range/fr")
        self.frame_id = rospy.get_param("~frame_id", "laser")
        self.max_range = float(rospy.get_param("~max_range", 0.3))
        self.min_range = float(rospy.get_param("~min_range", 0.01))

        self.pub = rospy.Publisher(self.range_topic, Range, queue_size=1)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)

    def _on_scan(self, msg):
        if not msg.ranges:
            return
        center_idx = len(msg.ranges) // 2
        reading = float(msg.ranges[center_idx])
        if not math.isfinite(reading):
            return
        out = Range()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.radiation_type = Range.INFRARED
        out.field_of_view = 0.05
        out.min_range = self.min_range
        out.max_range = self.max_range
        out.range = max(self.min_range, min(reading, self.max_range))
        self.pub.publish(out)


def main():
    rospy.init_node("sim_ir_publisher")
    SimIRPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
