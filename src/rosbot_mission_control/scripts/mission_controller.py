#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rosbot_competition_msgs.msg import SpatialDetection, MissionEvent
from rosbot_competition_msgs.srv import GraspPuck
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Shared memory for the state machine
class MissionData:
    def __init__(self):
        self.puck_memory = {}
        self.drop_zone_memory = {}
        self.completed_colors = set()
        self.target_order = ["red", "green", "blue"]
        self.current_target_color = None
        self.explore_waypoints = []
        self.explore_idx = 0


mission_data = MissionData()


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


def publish_event(pub, text):
    msg = MissionEvent(level="INFO", message=text)
    pub.publish(msg)
    rospy.loginfo(text)


class Explore(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["found_puck", "continue_explore"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: EXPLORE")

        # Check if we already found a puck we haven't delivered
        for color in mission_data.target_order:
            if color not in mission_data.completed_colors:
                if color in mission_data.puck_memory:
                    mission_data.current_target_color = color
                    publish_event(
                        self.event_pub,
                        f"[MISSION] Aborting explore, approaching known puck: {color}",
                    )
                    return "found_puck"

        if not mission_data.explore_waypoints:
            rospy.sleep(1.0)
            return "continue_explore"

        waypoint = mission_data.explore_waypoints[
            mission_data.explore_idx % len(mission_data.explore_waypoints)
        ]
        mission_data.explore_idx += 1

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = float(waypoint.get("x", 0.0))
        goal.target_pose.pose.position.y = float(waypoint.get("y", 0.0))
        yaw = float(waypoint.get("yaw", 0.0))
        half = 0.5 * yaw
        goal.target_pose.pose.orientation.z = math.sin(half)
        goal.target_pose.pose.orientation.w = math.cos(half)

        publish_event(
            self.event_pub, f"[MISSION] Exploring waypoint {mission_data.explore_idx}"
        )
        self.move_base.send_goal(goal)

        # Monitor while navigating
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            state = self.move_base.get_state()
            if state in [3, 4, 5]:  # SUCCEEDED, ABORTED, REJECTED
                break

            # Check if perception found something new
            for color in mission_data.target_order:
                if (
                    color not in mission_data.completed_colors
                    and color in mission_data.puck_memory
                ):
                    self.move_base.cancel_goal()
                    mission_data.current_target_color = color
                    publish_event(
                        self.event_pub,
                        f"[MISSION] Puck found! Switching to approach: {color}",
                    )
                    return "found_puck"
            rate.sleep()

        return "continue_explore"


class Approach(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["arrived", "failed"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: APPROACH")
        color = mission_data.current_target_color
        if not color or color not in mission_data.puck_memory:
            return "failed"

        puck = mission_data.puck_memory[color]
        robot = get_robot_pose(self.tf_buffer)
        if not robot:
            return "failed"

        dx = puck.x - robot.x
        dy = puck.y - robot.y
        norm = math.hypot(dx, dy)

        # Goal is 0.4m in front of the puck
        approach_dist = 0.4
        goal_x = puck.x - approach_dist * (dx / max(norm, 1e-3))
        goal_y = puck.y - approach_dist * (dy / max(norm, 1e-3))
        yaw = math.atan2(dy, dx)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y

        half = 0.5 * yaw
        goal.target_pose.pose.orientation.z = math.sin(half)
        goal.target_pose.pose.orientation.w = math.cos(half)

        publish_event(
            self.event_pub,
            f"[MISSION] Approaching {color} puck at ({goal_x:.2f}, {goal_y:.2f})",
        )
        self.move_base.send_goal(goal)

        self.move_base.wait_for_result(rospy.Duration(30.0))
        if self.move_base.get_state() == 3:
            return "arrived"
        return "failed"


class VisualServo(smach.State):
    def __init__(self, event_pub):
        smach.State.__init__(
            self, outcomes=["ready_to_grab", "wall_detected", "timeout"]
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
        self.event_pub = event_pub
        self.bridge = CvBridge()
        self.depth_sub = None
        self.latest_depth = None

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
            "/camera/aligned_depth_to_color/image_raw", Image, self._depth_cb
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
        cmd.linear.x = 0.1

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

            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        self.depth_sub.unregister()
        return "ready_to_grab"


class Grab(smach.State):
    def __init__(self, event_pub):
        smach.State.__init__(self, outcomes=["grabbed", "missed"])
        self.grasp_srv = rospy.ServiceProxy("/grasp_puck", GraspPuck)
        self.event_pub = event_pub

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
        cmd.linear.x = -0.1
        for _ in range(15):
            cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        cmd_pub.publish(Twist())

        return "missed"


class Deliver(smach.State):
    def __init__(self, tf_buffer, event_pub):
        smach.State.__init__(self, outcomes=["delivered", "failed"])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.release_srv = rospy.ServiceProxy("/release_puck", Trigger)
        self.tf_buffer = tf_buffer
        self.event_pub = event_pub

    def execute(self, userdata):
        publish_event(self.event_pub, "[MISSION] State: DELIVER")
        color = mission_data.current_target_color
        if not color:
            return "failed"

        zone = mission_data.drop_zone_memory.get(color)
        if not zone:
            publish_event(
                self.event_pub,
                f"[MISSION] Unknown drop zone for {color}. Re-exploring...",
            )
            # In a real scenario, we might hold the puck and look for the tag.
            # For now, drop it and fail
            self.release_srv()
            return "failed"

        robot = get_robot_pose(self.tf_buffer)
        if not robot:
            return "failed"

        dx = zone.x - robot.x
        dy = zone.y - robot.y
        norm = math.hypot(dx, dy)

        drop_dist = 0.35
        goal_x = zone.x - drop_dist * (dx / max(norm, 1e-3))
        goal_y = zone.y - drop_dist * (dy / max(norm, 1e-3))
        yaw = math.atan2(dy, dx)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        half = 0.5 * yaw
        goal.target_pose.pose.orientation.z = math.sin(half)
        goal.target_pose.pose.orientation.w = math.cos(half)

        publish_event(
            self.event_pub, f"[MISSION] Delivering {color} puck to ArUco zone..."
        )
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result(rospy.Duration(45.0))

        if self.move_base.get_state() == 3:
            publish_event(self.event_pub, "[MISSION] Arrived at drop zone. Releasing.")
            self.release_srv()
            mission_data.completed_colors.add(color)
            if color in mission_data.puck_memory:
                del mission_data.puck_memory[color]
            mission_data.current_target_color = None

            # Back up after releasing
            cmd_pub = rospy.Publisher("/cmd_vel_servo", Twist, queue_size=1)
            cmd = Twist()
            cmd.linear.x = -0.15
            for _ in range(20):
                cmd_pub.publish(cmd)
                rospy.sleep(0.1)
            cmd_pub.publish(Twist())

            return "delivered"
        return "failed"


def _on_detection(msg):
    point = Point(x=msg.map_point.x, y=msg.map_point.y, z=msg.map_point.z)
    if msg.object_class == "puck" and msg.color:
        mission_data.puck_memory[msg.color] = point
    elif msg.object_class == "drop_zone" and msg.color:
        mission_data.drop_zone_memory[msg.color] = point


def main():
    rospy.init_node("mission_controller")

    # Load config
    mission_data.explore_waypoints = rospy.get_param("~explore_waypoints", [])

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=10)
    rospy.Subscriber(
        "/perception/spatial_detections", SpatialDetection, _on_detection, queue_size=30
    )

    rospy.wait_for_service("/grasp_puck")
    rospy.wait_for_service("/release_puck")

    # Build SMACH
    sm = smach.StateMachine(outcomes=["MISSION_COMPLETE", "ABORTED"])
    with sm:
        smach.StateMachine.add(
            "EXPLORE",
            Explore(tf_buffer, event_pub),
            transitions={"found_puck": "APPROACH", "continue_explore": "EXPLORE"},
        )
        smach.StateMachine.add(
            "APPROACH",
            Approach(tf_buffer, event_pub),
            transitions={"arrived": "VISUAL_SERVO", "failed": "EXPLORE"},
        )
        smach.StateMachine.add(
            "VISUAL_SERVO",
            VisualServo(event_pub),
            transitions={
                "ready_to_grab": "GRAB",
                "wall_detected": "EXPLORE",
                "timeout": "EXPLORE",
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
            transitions={"delivered": "EXPLORE", "failed": "EXPLORE"},
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("mission_smach", sm, "/SM_ROOT")
    sis.start()

    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
