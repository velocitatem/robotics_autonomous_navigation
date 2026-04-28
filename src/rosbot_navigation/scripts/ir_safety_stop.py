#!/usr/bin/env python3

import math
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class IRSafetyStop:
    def __init__(self):
        self.stop_distance = rospy.get_param("~stop_distance", 0.14)
        self.clear_distance = rospy.get_param("~clear_distance", 0.18)
        self.input_topic = rospy.get_param("~range_topic", "/range/fr")
        self.filter_window = max(1, int(rospy.get_param("~filter_window", 5)))
        self.activate_samples = max(1, int(rospy.get_param("~activate_samples", 2)))
        self.clear_samples = max(1, int(rospy.get_param("~clear_samples", 2)))
        self.active = False
        self.readings = deque(maxlen=self.filter_window)
        self.stop_hits = 0
        self.clear_hits = 0

        self.cmd_pub = rospy.Publisher("/cmd_vel_safety", Twist, queue_size=1)
        self.range_sub = rospy.Subscriber(
            self.input_topic, Range, self._on_range, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_stop)
        rospy.loginfo(
            "[SAFETY] IR stop watching %s (stop=%.3f m, clear=%.3f m, "
            "filter_window=%d, activate_samples=%d, clear_samples=%d)",
            self.input_topic,
            self.stop_distance,
            self.clear_distance,
            self.filter_window,
            self.activate_samples,
            self.clear_samples,
        )

    def _filtered_reading(self):
        ordered = sorted(self.readings)
        return ordered[len(ordered) // 2]

    def _on_range(self, msg):
        reading = msg.range
        if reading <= 0.0 or not math.isfinite(reading):
            return

        self.readings.append(reading)
        filtered = self._filtered_reading()

        if filtered <= self.stop_distance:
            self.stop_hits += 1
            self.clear_hits = 0
            if self.stop_hits >= self.activate_samples:
                if not self.active:
                    rospy.logwarn("[SAFETY] IR stop active, range=%.3f m", filtered)
                self.active = True
        elif filtered >= self.clear_distance:
            self.clear_hits += 1
            self.stop_hits = 0
            if self.clear_hits >= self.clear_samples:
                if self.active:
                    rospy.loginfo("[SAFETY] IR stop cleared, range=%.3f m", filtered)
                self.active = False
        else:
            self.stop_hits = 0
            self.clear_hits = 0

    def _publish_stop(self, _event):
        if not self.active:
            return
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rospy.init_node("ir_safety_stop")
    IRSafetyStop()
    rospy.spin()


if __name__ == "__main__":
    main()
