#!/usr/bin/env python3

import collections
import rospy
from rosbot_competition_msgs.srv import GraspPuck, GraspPuckResponse
from rosbot_competition_msgs.msg import MissionEvent
from std_msgs.msg import Float32, Float64, String
from std_srvs.srv import Trigger, TriggerResponse


class ManipulationNode:
    def __init__(self):
        self.servo_load_topic = rospy.get_param("~servo_load_topic", "/servoLoad")
        self.command_topic = rospy.get_param(
            "~gripper_command_topic", "/gripper/command"
        )

        self.open_angle = float(rospy.get_param("~open_angle", 1.15))
        self.close_angle = float(rospy.get_param("~close_angle", 0.22))
        self.angle_step = abs(float(rospy.get_param("~angle_step", 0.03)))
        self.step_sleep_sec = float(rospy.get_param("~step_sleep_sec", 0.06))

        self.default_load_baseline = float(
            rospy.get_param("~default_load_baseline", 0.18)
        )
        self.load_spike_threshold = float(rospy.get_param("~load_spike_threshold", 1.5))
        self.release_pause_sec = float(rospy.get_param("~release_pause_sec", 0.2))

        self.load_history = collections.deque(maxlen=10)
        self.latest_load = self.default_load_baseline
        self.have_load = False

        self.command_pub = rospy.Publisher(self.command_topic, Float64, queue_size=5)
        self.event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=30)
        self.load_sub = rospy.Subscriber(
            self.servo_load_topic, Float32, self._on_servo_load, queue_size=5
        )

        self.grasp_srv = rospy.Service("/grasp_puck", GraspPuck, self._on_grasp_request)
        self.release_srv = rospy.Service(
            "/release_puck", Trigger, self._on_release_request
        )

        self._publish_angle(self.open_angle)
        rospy.loginfo("[GRIPPER] Manipulation node online")

    def _on_servo_load(self, msg):
        val = float(msg.data)
        self.latest_load = val
        self.load_history.append(val)
        self.have_load = True

    def _publish_angle(self, angle):
        self.command_pub.publish(Float64(data=float(angle)))

    def _on_grasp_request(self, request):
        if self.have_load and len(self.load_history) > 0:
            baseline = sum(self.load_history) / len(self.load_history)
        else:
            baseline = self.default_load_baseline

        direction = -1.0 if self.open_angle > self.close_angle else 1.0
        angle = self.open_angle
        peak_load = baseline
        success = False

        self._publish_angle(self.open_angle)
        rospy.sleep(self.step_sleep_sec)

        while direction * (self.close_angle - angle) > 0.0 and not rospy.is_shutdown():
            angle += direction * self.angle_step
            if direction < 0.0:
                angle = max(angle, self.close_angle)
            else:
                angle = min(angle, self.close_angle)

            self._publish_angle(angle)
            rospy.sleep(self.step_sleep_sec)

            current_load = self.latest_load if self.have_load else baseline

            # Update baseline dynamically as rolling average if not spiking
            if len(self.load_history) > 0:
                baseline = sum(self.load_history) / len(self.load_history)

            peak_load = max(peak_load, current_load)
            if (current_load - baseline) >= self.load_spike_threshold:
                success = True
                break

        if not success:
            self._publish_angle(self.close_angle)

        status = "successful" if success else "failed"
        msg_text = f"[GRIPPER] Grasp {status}. PeakLoad={peak_load:.2f} baseline={baseline:.2f} target={request.color}"
        event_msg = MissionEvent(level="INFO", message=msg_text)
        self.event_pub.publish(event_msg)
        rospy.loginfo(msg_text)

        response = GraspPuckResponse()
        response.success = success
        response.peak_load = peak_load
        response.final_angle = angle if success else self.close_angle
        response.message = status
        return response

    def _on_release_request(self, _request):
        self._publish_angle(self.open_angle)
        rospy.sleep(max(0.0, self.release_pause_sec))

        msg_text = f"[GRIPPER] Released puck at angle {self.open_angle:.2f}"
        event_msg = MissionEvent(level="INFO", message=msg_text)
        self.event_pub.publish(event_msg)
        rospy.loginfo(msg_text)
        return TriggerResponse(success=True, message="released")


def main():
    rospy.init_node("manipulation_node")
    ManipulationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
