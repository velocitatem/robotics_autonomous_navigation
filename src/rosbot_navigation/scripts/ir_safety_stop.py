#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class IRSafetyStop:
    def __init__(self):
        self.stop_distance = rospy.get_param("~stop_distance", 0.14)
        self.clear_distance = rospy.get_param("~clear_distance", 0.18)
        self.input_topic = rospy.get_param("~range_topic", "/range/fr")
        self.active = False

        self.cmd_pub = rospy.Publisher("/cmd_vel_safety", Twist, queue_size=1)
        self.range_sub = rospy.Subscriber(
            self.input_topic, Range, self._on_range, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_stop)

    def _on_range(self, msg):
        reading = msg.range
        if reading <= 0.0:
            return
        if reading <= self.stop_distance:
            if not self.active:
                rospy.logwarn("[SAFETY] IR stop active, range=%.3f m", reading)
            self.active = True
        elif reading >= self.clear_distance:
            if self.active:
                rospy.loginfo("[SAFETY] IR stop cleared, range=%.3f m", reading)
            self.active = False

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
