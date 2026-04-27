#!/usr/bin/env python3

import math

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros

# Importing tf2_geometry_msgs registers PointStamped/PoseStamped converters
# with tf2_ros.Buffer. Without it, tf_buffer.transform(PointStamped) raises
# TypeException and every detection callback crashes silently.
import tf2_geometry_msgs  # noqa: F401  (registration side-effect)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from rosbot_competition_msgs.msg import SpatialDetection, MissionEvent
from sensor_msgs.msg import CameraInfo, Image
from image_geometry import PinholeCameraModel


class PerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.1))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 2.5))
        self.min_contour_area = float(rospy.get_param("~min_contour_area", 450.0))
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius_px", 2))
        self.publish_cooldown_sec = float(rospy.get_param("~publish_cooldown_sec", 0.4))
        self.min_publish_distance_m = float(
            rospy.get_param("~min_publish_distance_m", 0.05)
        )
        # Remove noisy sky/ceiling band: CV + ArUco run on the image below this strip.
        self.crop_top_fraction = float(rospy.get_param("~crop_top_fraction", 0.20))

        self.hsv_ranges = rospy.get_param("~hsv_ranges", {})
        aruco_cfg = rospy.get_param("~aruco", {})
        self.marker_color_map = {
            int(marker_id): color
            for marker_id, color in aruco_cfg.get(
                "marker_color_map", {1: "red", 2: "green", 3: "blue"}
            ).items()
        }

        dictionary_name = aruco_cfg.get("dictionary", "DICT_4X4_50")
        self.aruco, self.aruco_dict, self.aruco_detector = self._build_aruco_detector(
            dictionary_name
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.last_detection = {}

        self.det_pub = rospy.Publisher(
            "/perception/spatial_detections", SpatialDetection, queue_size=30
        )
        self.annotated_pub = rospy.Publisher(
            "/perception/annotated_image", Image, queue_size=1
        )
        self.event_pub = rospy.Publisher("/mission_events", MissionEvent, queue_size=30)

        self.camera_model = PinholeCameraModel()
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1
        )

        self.color_sub = message_filters.Subscriber(self.color_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.08
        )
        self.sync.registerCallback(self._on_image_pair)

        rospy.loginfo("[VISION] Perception node online")

    def _build_aruco_detector(self, dictionary_name):
        if not hasattr(cv2, "aruco"):
            rospy.logwarn("[VISION] OpenCV ArUco module not available")
            return None, None, None

        aruco = cv2.aruco
        dict_id = getattr(aruco, dictionary_name, aruco.DICT_4X4_50)
        dictionary = aruco.getPredefinedDictionary(dict_id)

        if hasattr(aruco, "ArucoDetector") and hasattr(aruco, "DetectorParameters"):
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        else:
            detector = aruco.DetectorParameters_create()

        return aruco, dictionary, detector

    def _on_camera_info(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        if self.camera_info_sub is not None:
            self.camera_info_sub.unregister()
            self.camera_info_sub = None
            rospy.loginfo("[VISION] Camera intrinsics initialized")

    def _on_image_pair(self, color_msg, depth_msg):
        if self.fx is None:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as err:
            rospy.logwarn_throttle(
                2.0, "[VISION] CvBridge conversion failed: %s", str(err)
            )
            return

        depth_m = self._to_meters(depth)
        if depth_m is None:
            return

        h = bgr.shape[0]
        crop_top = min(h - 1, int(round(h * self.crop_top_fraction)))
        if crop_top > 0:
            bgr = bgr[crop_top:, :]
            depth_m = depth_m[crop_top:, :]

        overlay = bgr.copy()
        timestamp = color_msg.header.stamp
        camera_frame = (
            self.camera_frame_override
            if self.camera_frame_override
            else color_msg.header.frame_id
        )

        puck_candidates = self._detect_pucks(overlay)
        for candidate in puck_candidates:
            self._publish_spatial_detection(
                overlay=overlay,
                stamp=timestamp,
                camera_frame=camera_frame,
                depth_m=depth_m,
                object_class="puck",
                color=candidate["color"],
                marker_id=-1,
                u=candidate["u"],
                v=candidate["v"],
                crop_top_px=crop_top,
                confidence=candidate["confidence"],
            )

        drop_zone_candidates = self._detect_markers(overlay)
        for candidate in drop_zone_candidates:
            self._publish_spatial_detection(
                overlay=overlay,
                stamp=timestamp,
                camera_frame=camera_frame,
                depth_m=depth_m,
                object_class="drop_zone",
                color=candidate["color"],
                marker_id=candidate["marker_id"],
                u=candidate["u"],
                v=candidate["v"],
                crop_top_px=crop_top,
                confidence=1.0,
            )

        try:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            )
        except CvBridgeError:
            pass

    def _to_meters(self, depth_image):
        if depth_image is None:
            return None

        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * self.depth_scale

        if depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
            return depth_image.astype(np.float32)

        rospy.logwarn_throttle(
            5.0, "[VISION] Unsupported depth dtype: %s", str(depth_image.dtype)
        )
        return None

    def _detect_pucks(self, overlay):
        hsv = cv2.cvtColor(overlay, cv2.COLOR_BGR2HSV)
        detections = []
        image_area = float(overlay.shape[0] * overlay.shape[1])

        # Always re-run HSV detection on every frame. The previous version
        # cached colours that had ever been published and skipped them, which
        # froze the puck position estimate at the very first (far, noisy)
        # detection and left the mission state machine driving toward stale
        # coordinates that no longer matched reality.
        for color_name, ranges in self.hsv_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for hsv_range in ranges:
                lower = np.array(hsv_range["lower"], dtype=np.uint8)
                upper = np.array(hsv_range["upper"], dtype=np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self.min_contour_area:
                continue

            moments = cv2.moments(largest)
            if moments["m00"] <= 0.0:
                continue

            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            confidence = min(1.0, area / max(image_area * 0.08, 1.0))

            cv2.drawContours(overlay, [largest], -1, (255, 255, 255), 2)
            cv2.circle(overlay, (u, v), 4, (0, 0, 0), -1)
            cv2.putText(
                overlay,
                f"{color_name} puck",
                (u + 6, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            detections.append(
                {"color": color_name, "u": u, "v": v, "confidence": confidence}
            )

        return detections

    def _detect_markers(self, overlay):
        detections = []
        if self.aruco is None or self.aruco_dict is None:
            return detections

        gray = cv2.cvtColor(overlay, cv2.COLOR_BGR2GRAY)
        if hasattr(self.aruco, "ArucoDetector") and hasattr(
            self.aruco_detector, "detectMarkers"
        ):
            corners, ids, _rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = self.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_detector
            )

        if ids is None or len(ids) == 0:
            return detections

        self.aruco.drawDetectedMarkers(overlay, corners, ids)
        for idx, marker_id in enumerate(ids.flatten()):
            pts = corners[idx][0]
            center = np.mean(pts, axis=0)
            u = int(center[0])
            v = int(center[1])
            color = self.marker_color_map.get(int(marker_id), "unknown")
            cv2.putText(
                overlay,
                f"ID {int(marker_id)} ({color})",
                (u + 8, v + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
            detections.append(
                {"marker_id": int(marker_id), "color": color, "u": u, "v": v}
            )

        return detections

    def _publish_spatial_detection(
        self,
        overlay,
        stamp,
        camera_frame,
        depth_m,
        object_class,
        color,
        marker_id,
        u,
        v,
        crop_top_px,
        confidence,
    ):
        z = self._sample_depth(depth_m, u, v)
        if z is None:
            return

        u_full = u
        v_full = v + int(crop_top_px)
        ray = self.camera_model.projectPixelTo3dRay((u_full, v_full))
        ray_z = ray[2]
        x = ray[0] * (z / ray_z)
        y = ray[1] * (z / ray_z)

        source = PointStamped()
        source.header.stamp = stamp
        source.header.frame_id = camera_frame
        source.point = Point(x=x, y=y, z=z)

        try:
            map_point = self.tf_buffer.transform(
                source, self.map_frame, rospy.Duration(0.07)
            )
        except tf2_ros.ExtrapolationException:
            source.header.stamp = rospy.Time(0)
            try:
                map_point = self.tf_buffer.transform(
                    source, self.map_frame, rospy.Duration(0.07)
                )
                rospy.logwarn_throttle(
                    2.0,
                    "[VISION] Camera/TF timestamps are out of sync; using latest TF for projection.",
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException,
            ):
                return
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
        ):
            return

        key = f"{object_class}:{color}:{marker_id}"
        if not self._should_publish(key, map_point.point, stamp):
            return

        msg = SpatialDetection()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.object_class = object_class
        msg.color = color
        msg.marker_id = marker_id
        msg.map_point = map_point.point
        msg.depth_m = float(z)
        msg.confidence = float(confidence)
        self.det_pub.publish(msg)

        cv2.circle(overlay, (u, v), 7, (0, 255, 255), 2)

        event_msg = MissionEvent()
        event_msg.level = "INFO"
        event_msg.message = f"[VISION] Mapped {color} {object_class} at ({map_point.point.x:.2f}, {map_point.point.y:.2f})"
        self.event_pub.publish(event_msg)

    def _should_publish(self, key, current_point, stamp):
        previous = self.last_detection.get(key)
        if previous is None:
            self.last_detection[key] = (stamp, current_point)
            return True

        previous_stamp, previous_point = previous
        dt = (stamp - previous_stamp).to_sec()
        distance = math.hypot(
            current_point.x - previous_point.x, current_point.y - previous_point.y
        )
        if dt >= self.publish_cooldown_sec or distance >= self.min_publish_distance_m:
            self.last_detection[key] = (stamp, current_point)
            return True
        return False

    def _sample_depth(self, depth_m, u, v):
        height, width = depth_m.shape[:2]
        x0 = max(0, int(u) - self.depth_patch_radius)
        y0 = max(0, int(v) - self.depth_patch_radius)
        x1 = min(width - 1, int(u) + self.depth_patch_radius)
        y1 = min(height - 1, int(v) + self.depth_patch_radius)

        patch = depth_m[y0 : y1 + 1, x0 : x1 + 1].reshape(-1)
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch > self.min_depth_m) & (patch < self.max_depth_m)]
        if patch.size == 0:
            return None
        return float(np.median(patch))


def main():
    rospy.init_node("perception_node")
    PerceptionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
