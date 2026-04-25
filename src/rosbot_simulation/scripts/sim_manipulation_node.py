#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from rosbot_competition_msgs.msg import MissionEvent
from rosbot_competition_msgs.srv import GraspPuck, GraspPuckResponse
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse


class SimManipulationNode:
    def __init__(self):
        self.grasp_radius_m = float(rospy.get_param("~grasp_radius_m", 0.25))
        self.robot_model_name = rospy.get_param("~robot_model_name", "rosbot")
        self.robot_link_name = rospy.get_param("~robot_link_name", "base_link")
        self.puck_prefix = rospy.get_param("~puck_prefix", "puck_")
        self.servo_load_topic = rospy.get_param("~servo_load_topic", "/servoLoad")
        self.load_spike_value = float(rospy.get_param("~load_spike_value", 1.2))
        self.attach_offset_z = float(rospy.get_param("~attach_offset_z", 0.10))

        self.model_states = None
        self.attached_model = None
        self.model_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1
        )
        self.load_pub = rospy.Publisher(self.servo_load_topic, Float32, queue_size=2)
        self.event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=20)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.follow_timer = rospy.Timer(rospy.Duration(0.05), self._follow_attached)

        self.grasp_srv = rospy.Service("/grasp_puck", GraspPuck, self._on_grasp)
        self.release_srv = rospy.Service("/release_puck", Trigger, self._on_release)

        rospy.loginfo("[SIM] Sim manipulation online")

    def _on_model_states(self, msg):
        self.model_states = msg

    def _on_grasp(self, req):
        if self.model_states is None:
            return GraspPuckResponse(False, 0.0, 0.0, "no model_states yet")

        nearest = self._find_nearest_puck(req.color)
        if nearest is None:
            return GraspPuckResponse(False, 0.0, 0.0, "no nearby puck")

        puck_name, distance = nearest
        if distance > self.grasp_radius_m:
            return GraspPuckResponse(False, 0.0, 0.0, "puck too far")

        ok = self._attach_model(puck_name)
        if not ok:
            return GraspPuckResponse(False, 0.0, 0.0, "attach failed")

        self.attached_model = puck_name
        self.load_pub.publish(Float32(data=self.load_spike_value))
        self._event(f"[SIM] Attached {puck_name} for requested color '{req.color}'")
        return GraspPuckResponse(True, self.load_spike_value, 0.0, "attached")

    def _on_release(self, _req):
        if not self.attached_model:
            return TriggerResponse(success=True, message="nothing attached")
        ok = self._detach_model(self.attached_model)
        if ok:
            self._event(f"[SIM] Detached {self.attached_model}")
            self.attached_model = None
            return TriggerResponse(success=True, message="detached")
        return TriggerResponse(success=False, message="detach failed")

    def _find_nearest_puck(self, color):
        names = self.model_states.name
        poses = self.model_states.pose
        if self.robot_model_name not in names:
            return None
        robot_idx = names.index(self.robot_model_name)
        robot_pose = poses[robot_idx]

        best = None
        for idx, name in enumerate(names):
            if not name.startswith(self.puck_prefix):
                continue
            if color and color not in name:
                continue
            p = poses[idx].position
            d = math.hypot(p.x - robot_pose.position.x, p.y - robot_pose.position.y)
            if best is None or d < best[1]:
                best = (name, d)

        if best is None and color:
            for idx, name in enumerate(names):
                if not name.startswith(self.puck_prefix):
                    continue
                p = poses[idx].position
                d = math.hypot(p.x - robot_pose.position.x, p.y - robot_pose.position.y)
                if best is None or d < best[1]:
                    best = (name, d)
        return best

    def _attach_model(self, model_name):
        try:
            self._teleport_on_robot(model_name)
            return True
        except rospy.ServiceException as err:
            rospy.logwarn("[SIM] attach failed: %s", str(err))
            return False

    def _detach_model(self, model_name):
        try:
            # Keep current detached pose; gravity will handle motion afterwards.
            state = ModelState()
            state.model_name = model_name
            state.reference_frame = "world"
            self.set_state_srv(state)
            return True
        except rospy.ServiceException as err:
            rospy.logwarn("[SIM] detach failed: %s", str(err))
            return False

    def _follow_attached(self, _event):
        if not self.attached_model:
            return
        if self.model_states is None:
            return
        try:
            self._teleport_on_robot(self.attached_model)
        except rospy.ServiceException:
            pass

    def _teleport_on_robot(self, model_name):
        names = self.model_states.name
        poses = self.model_states.pose
        if self.robot_model_name not in names:
            return
        robot_idx = names.index(self.robot_model_name)
        robot_pose = poses[robot_idx]

        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = robot_pose.position.x
        state.pose.position.y = robot_pose.position.y
        state.pose.position.z = robot_pose.position.z + self.attach_offset_z
        state.pose.orientation = robot_pose.orientation
        state.reference_frame = "world"
        self.set_state_srv(state)

    def _event(self, text):
        self.event_pub.publish(MissionEvent(level="INFO", message=text))
        rospy.loginfo(text)


def main():
    rospy.init_node("sim_manipulation_node")
    SimManipulationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
